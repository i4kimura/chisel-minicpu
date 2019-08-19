package cache_subsystem

import chisel3._
import chisel3.util._
import chisel3.Bool

import wt_cache_pkg._
import ariane_pkg._

class wt_icache (
  // ID to be used for read transactions
  RdTxId:Int   = 1,
  // contains cacheable regions
  ArianeCfg:ariane_cfg_t)
    extends Module {
  val io = IO(new Bundle {
    val flush_i = Input (Bool())   // flush the icache, flush and kill have to be asserted together
    val en_i    = Input (Bool())   // enable icache
    val miss_o  = Output(Bool())   // to performance counter

    // address translation requests
    val areq_i = Input (new icache_areq_i_t())
    val areq_o = Output(new icache_areq_o_t())
    // data requests
    val dreq_i = Input (new icache_dreq_i_t())
    val dreq_o = Output(new icache_dreq_o_t())

    // refill port
    val mem_rtrn_if = Flipped(ValidIO(new icache_rtrn_t()))
    val mem_data_if = DecoupledIO(new icache_req_t())
  })

  // signals
  val cache_en_d  = Wire(Bool())
  val cache_en_q  = RegInit(false.B)           // cache is enabled
  val vaddr_d     = Wire(UInt(64.W))
  val vaddr_q     = RegInit(0.U(64.W))
  val paddr_is_nc = Wire(Bool())              // asserted if physical address is non-cacheable
  val cl_hit      = Wire(Vec(ICACHE_SET_ASSOC, Bool())) // hit from tag compare
  val cache_rden  = Wire(Bool())               // triggers cache lookup
  val cache_wren  = Wire(Bool())               // triggers write to cacheline
  val cmp_en_d    = Wire(Bool())
  val cmp_en_q    = RegInit(false.B)             // enable tag comparison in next cycle. used to cut long path due to NC signal.
  val flush_d     = Wire(Bool())
  val flush_q     = RegInit(false.B)             // used to register and signal pending flushes

  // replacement strategy
  val update_lfsr    = Wire(Bool())                               // shift the LFSR
  val inv_way        = Wire(UInt(log2Ceil(ICACHE_SET_ASSOC).W))   // first non-valid encountered
  val rnd_way        = Wire(UInt(log2Ceil(ICACHE_SET_ASSOC).W))   // random index for replacement
  val repl_way       = Wire(UInt(log2Ceil(ICACHE_SET_ASSOC).W))   // way to replace
  val repl_way_oh_d  = Wire(UInt(ICACHE_SET_ASSOC.W))
  val repl_way_oh_q  = RegInit(0.U(ICACHE_SET_ASSOC.W))           // way to replace (onehot)
  val all_ways_valid = Wire(Bool())                               // we need to switch repl strategy since all are valid

  // invalidations / flushing
  val inv_en      = Wire(Bool())                        // incoming invalidations
  val flush_en    = Wire(Bool())                        // used to flush cache entries
  val flush_done  = Wire(Bool())
  val flush_cnt_d = Wire(UInt(ICACHE_CL_IDX_WIDTH.W))
  val flush_cnt_q = RegInit(0.U(ICACHE_CL_IDX_WIDTH.W)) // used to flush cache entries

  // mem arrays
  val cl_we        = Wire(Bool())                                           // write enable to memory array
  val cl_req       = Wire(UInt(ICACHE_SET_ASSOC.W))                         // request to memory array
  val cl_index     = Wire(UInt(ICACHE_CL_IDX_WIDTH.W))                      // this is a cache-line index, to memory array
  val cl_offset_d  = Wire(UInt(ICACHE_OFFSET_WIDTH.W))                      // offset in cache line
  val cl_offset_q  = RegInit(0.U(ICACHE_OFFSET_WIDTH.W))                    // offset in cache line
  val cl_tag_d     = Wire(UInt(ICACHE_TAG_WIDTH.W))                         // this is the cache tag
  val cl_tag_q     = RegInit(0.U(ICACHE_TAG_WIDTH.W))                       // this is the cache tag
  val cl_tag_rdata = Wire(Vec(ICACHE_SET_ASSOC, UInt(ICACHE_TAG_WIDTH .W))) // these are the tags coming from the tagmem
  val cl_rdata     = Wire(Vec(ICACHE_SET_ASSOC, UInt(ICACHE_LINE_WIDTH.W))) // these are the cachelines coming from the cache
  val cl_sel       = Wire(Vec(ICACHE_SET_ASSOC, UInt(FETCH_WIDTH.W)))      // selected word from each cacheline

  val vld_req   = Wire(UInt(ICACHE_SET_ASSOC.W))    // bit enable for valid regs
  val vld_we    = Wire(Bool())                      // valid bits write enable
  val vld_wdata = Wire(UInt(ICACHE_SET_ASSOC.W))    // valid bits to write
  val vld_rdata = Wire(Vec(ICACHE_SET_ASSOC, Bool())) // valid bits coming from valid regs
  val vld_addr  = Wire(UInt(ICACHE_CL_IDX_WIDTH.W)) // valid bit

  // cpmtroller FSM
  val flush_s :: idle_s :: read_s :: miss_s :: tlb_miss_s :: kill_atrans_s :: kill_miss_s :: Nil = Enum(7)
  val state_d = Wire(UInt(3.W))
  val state_q = RegInit(idle_s)

  ///////////////////////////////////////////////////////
  // address -> cl_index mapping, interface plumbing
  ///////////////////////////////////////////////////////

  // extract tag from physical address, check if NC
  cl_tag_d  := Mux(io.areq_i.fetch_valid,
                   io.areq_i.fetch_paddr(ICACHE_TAG_WIDTH+ICACHE_INDEX_WIDTH-1, ICACHE_INDEX_WIDTH),
                   cl_tag_q)

  // noncacheable if request goes to I/O space, or if cache is disabled
  paddr_is_nc := (~cache_en_q) | (~is_inside_cacheable_regions(ArianeCfg, Cat(cl_tag_d, VecInit(Seq.fill(ICACHE_INDEX_WIDTH)(false.B)).asUInt)))

  // pass exception through
  io.dreq_o.ex := io.areq_i.fetch_exception

  // latch this in case we have to stall later on
  // make sure this is 32bit aligned

  vaddr_d := Mux(io.dreq_o.ready & io.dreq_i.req, io.dreq_i.vaddr, vaddr_q)
  io.areq_o.fetch_vaddr := Cat(vaddr_q >> 2, 0.U(2.W))

  // split virtual address into index and offset to address cache arrays
  cl_index  := vaddr_d(ICACHE_INDEX_WIDTH-1, ICACHE_OFFSET_WIDTH)

  if (ArianeCfg.Axi64BitCompliant) { // gen_axi_offset

    // if we generate a noncacheable access, the word will be at offset 0 or 4 in the cl coming from memory
    cl_offset_d := Mux(io.dreq_o.ready & io.dreq_i.req, Cat(io.dreq_i.vaddr >> 2, 0.U(2.W)),
                   Mux(paddr_is_nc  & io.mem_data_if.valid, cl_offset_q(2) << 2 , // needed since we transfer 32bit over a 64bit AXI bus in this case
                       cl_offset_q))
    // request word address instead of cl address in case of NC access
    io.mem_data_if.bits.paddr := Mux(paddr_is_nc, Cat(cl_tag_d, vaddr_q(ICACHE_INDEX_WIDTH-1, 3), 0.U(3.W)),                                          // align to 32bit
                                                  Cat(cl_tag_d, vaddr_q(ICACHE_INDEX_WIDTH-1, ICACHE_OFFSET_WIDTH), VecInit(Seq.fill(ICACHE_OFFSET_WIDTH)(false.B)).asUInt)) // align to cl
  } else { // : gen_piton_offset
    // icache fills are either cachelines or 4byte fills, depending on whether they go to the Piton I/O space or not.
    // since the piton cache system replicates the data, we can always index the full CL
    cl_offset_d := Mux(io.dreq_o.ready & io.dreq_i.req, Cat(io.dreq_i.vaddr >> 2, 0.U(2.W)), cl_offset_q)

    // request word address instead of cl address in case of NC access
    io.mem_data_if.bits.paddr := Mux(paddr_is_nc, Cat(cl_tag_d, vaddr_q(ICACHE_INDEX_WIDTH-1, 2), 0.U(2.W)),                                         // align to 32bit
                                                  Cat(cl_tag_d, vaddr_q(ICACHE_INDEX_WIDTH-1, ICACHE_OFFSET_WIDTH), VecInit(Seq.fill(ICACHE_OFFSET_WIDTH)(false.B)).asUInt)) // align to cl
  }

  io.mem_data_if.bits.tid   := RdTxId.asUInt

  io.mem_data_if.bits.nc    := paddr_is_nc
  // way that is being replaced
  io.mem_data_if.bits.way   := repl_way
  io.dreq_o.vaddr           := vaddr_q

  ///////////////////////////////////////////////////////
  // main control logic
  ///////////////////////////////////////////////////////

  // default assignment
  state_d      := state_q
  cache_en_d   := cache_en_q & io.en_i// disabling the cache is always possible, enable needs to go via flush
  flush_en     := false.B
  cmp_en_d     := false.B
  cache_rden   := false.B
  cache_wren   := false.B
  inv_en       := false.B
  flush_d      := flush_q | io.flush_i // register incoming flush

  // interfaces
  io.dreq_o.ready      := false.B
  io.areq_o.fetch_req  := false.B
  io.dreq_o.valid      := false.B
  io.mem_data_if.valid := false.B
  // performance counter
  io.miss_o            := false.B

  // handle invalidations unconditionally
  // note: invald are mutually exclusive with
  // ifills, since both arrive over the same IF
  // however, we need to make sure below that we
  // do not trigger a cache readout at the same time...
  when (io.mem_rtrn_if.valid && (io.mem_rtrn_if.bits.rtype === icache_inv_req_s)) {
    inv_en := true.B
  }

  state_d := flush_s
  switch (state_q) {
    //////////////////////////////////
    // this clears all valid bits
    is(flush_s) {
      flush_en := true.B
      when (flush_done) {
        state_d := idle_s
        flush_d := false.B
        // if the cache was not enabled set this
        cache_en_d := io.en_i
      }
    }
    //////////////////////////////////
    // wait for an incoming request
    is(idle_s) {
      // only enable tag comparison if cache is enabled
      cmp_en_d := cache_en_q

      // handle pending flushes, or perform cache clear upon enable
      when (flush_d || (io.en_i && !cache_en_q)) {
        state_d    := flush_s
        // wait for incoming requests
      } .otherwise {
        // mem requests are for sure invals here
        when (!io.mem_rtrn_if.valid) {
          io.dreq_o.ready := true.B
          // we have a new request
          when (io.dreq_i.req) {
            cache_rden       := true.B
            state_d          := read_s
          }
        }
        when (io.dreq_i.kill_s1) {
          state_d := idle_s
        }
      }
    }
    //////////////////////////////////
    // check whether we have a hit
    // in case the cache is disabled,
    // or in case the address is NC, we
    // reuse the miss mechanism to handle
    // the request
    is(read_s) {
      state_d          := tlb_miss_s
      io.areq_o.fetch_req := true.B
      // only enable tag comparison if cache is enabled
      cmp_en_d    := cache_en_q
      // readout speculatively
      cache_rden  := cache_en_q

      when (io.areq_i.fetch_valid) {
        // check if we have to flush
        when (flush_d) {
          state_d  := idle_s
          // we have a hit or an exception output valid result
        } .elsewhen ((cl_hit.asUInt.orR && cache_en_q) || io.areq_i.fetch_exception.valid) {
          io.dreq_o.valid  := ~io.dreq_i.kill_s2  // just don't output in this case
          state_d          := idle_s

          // we can accept another request
          // and stay here, but only if no inval is coming in
          // note: we are not expecting ifill return packets here...
          when (!io.mem_rtrn_if.valid) {
            io.dreq_o.ready     := true.B
            when (io.dreq_i.req) {
              state_d          := read_s
            }
          }
          // if a request is being killed at this stage,
          // we have to bail out and wait for the address translation to complete
          when (io.dreq_i.kill_s1) {
            state_d := idle_s
          }
          // we have a miss / NC transaction
        } .elsewhen (io.dreq_i.kill_s2) {
          state_d := idle_s
        } .otherwise {
          cmp_en_d := false.B
          // only count this as a miss if the cache is enabled, and
          // the address is cacheable
          // send out ifill request
          io.mem_data_if.valid := true.B
          when (io.mem_data_if.ready) {
            io.miss_o := ~paddr_is_nc
            state_d   := miss_s
          }
        }
        // bail out if this request is being killed (and we missed on the TLB)
      } .elsewhen  (io.dreq_i.kill_s2 || flush_d) {
        state_d  := kill_atrans_s
      }
    }
    //////////////////////////////////
    // wait until the memory transaction
    // returns. do not write to memory
    // if the nc bit is set.
    is(tlb_miss_s) {
      io.areq_o.fetch_req := true.B
      // only enable tag comparison if cache is enabled
      cmp_en_d := cache_en_q
      // readout speculatively
      cache_rden := cache_en_q

      when (io.areq_i.fetch_valid) {
        // check if we have to kill this request
        when (io.dreq_i.kill_s2 | flush_d) {
          state_d  := idle_s
          // check whether we got an exception
        } .elsewhen  (io.areq_i.fetch_exception.valid) {
          io.dreq_o.valid := true.B
          state_d      := idle_s
          // re-trigger cache readout for tag comparison and cache line selection
          // but if we got an invalidation, we have to wait another cycle
        } .elsewhen  (!io.mem_rtrn_if.valid) {
          state_d          := read_s
        }
        // bail out if this request is being killed
      } .elsewhen (io.dreq_i.kill_s2 || flush_d) {
        state_d  := kill_atrans_s
      }
    }
    //////////////////////////////////
    // wait until the memory transaction
    // returns. do not write to memory
    // if the nc bit is set.
    is(miss_s) {
      // note: this is mutually exclusive with ICACHE_INV_REQ,
      // so we do not have to check for invals here
      when (io.mem_rtrn_if.valid && io.mem_rtrn_if.bits.rtype === icache_ifill_ack_s) {
        state_d      := idle_s
        // only return data if request is not being killed
        when (!(io.dreq_i.kill_s2 || flush_d)) {
          io.dreq_o.valid := true.B
          // only write to cache if this address is cacheable
          cache_wren   := ~paddr_is_nc
        }
        // bail out if this request is being killed
      } .elsewhen (io.dreq_i.kill_s2 || flush_d) {
        state_d  := kill_miss_s
      }
    }
    //////////////////////////////////
    // killed address translation,
    // wait until paddr is valid, and go
    // back to idle
    is(kill_atrans_s) {
      io.areq_o.fetch_req := true.B
      when (io.areq_i.fetch_valid) {
        state_d      := idle_s
      }
    }
    //////////////////////////////////
    // killed miss,
    // wait until memory responds and
    // go back to idle
    is(kill_miss_s) {
      when (io.mem_rtrn_if.valid && io.mem_rtrn_if.bits.rtype === icache_ifill_ack_s) {
        state_d := idle_s
      }
    }
  }

  ///////////////////////////////////////////////////////
  // valid bit invalidation and replacement strategy
  ///////////////////////////////////////////////////////

  // note: it cannot happen that we get an invalidation + a cl replacement
  // in the same cycle as these requests arrive via the same interface
  // flushes take precedence over invalidations (it is ok if we ignore
  // the inval since the cache is cleared anyway)

  flush_cnt_d := Mux(flush_done, 0.U,
                 Mux(flush_en,   flush_cnt_q + 1.U,
                     flush_cnt_q))

  flush_done  := (flush_cnt_q === ((ICACHE_NUM_WORDS-1).U))

  // invalidation/clearing address
  // flushing takes precedence over invals
  vld_addr := Mux(flush_en, flush_cnt_q,
              Mux(inv_en, io.mem_rtrn_if.bits.inv.idx(ICACHE_INDEX_WIDTH-1, ICACHE_OFFSET_WIDTH),
                  cl_index))

  vld_req  := Mux(flush_en || cache_rden, VecInit(Seq.fill(ICACHE_SET_ASSOC)(true.B)).asUInt,
              Mux(io.mem_rtrn_if.bits.inv.all && inv_en, VecInit(Seq.fill(ICACHE_SET_ASSOC)(true.B)).asUInt,
              Mux(io.mem_rtrn_if.bits.inv.vld && inv_en, icache_way_bin2oh(io.mem_rtrn_if.bits.inv.way),
                  repl_way_oh_q)))

  vld_wdata := Mux(cache_wren, VecInit(Seq.fill(ICACHE_SET_ASSOC)(true.B )).asUInt,
                               VecInit(Seq.fill(ICACHE_SET_ASSOC)(false.B)).asUInt)

  vld_we    := (cache_wren | inv_en | flush_en)
  // vld_req   := (vld_we | cache_rden)


  // chose random replacement if all are valid
  update_lfsr   := cache_wren & all_ways_valid
  repl_way      := Mux(all_ways_valid, rnd_way, inv_way)
  repl_way_oh_d := Mux(cmp_en_q, icache_way_bin2oh(repl_way), repl_way_oh_q)

  // enable signals for memory arrays
  cl_req   := Mux(cache_rden, VecInit(Seq.fill(ICACHE_SET_ASSOC)(true.B)).asUInt,
              Mux(cache_wren, repl_way_oh_q,
                  VecInit(Seq.fill(ICACHE_SET_ASSOC)(false.B)).asUInt))
  cl_we    := cache_wren


  // find invalid cache line
  val i_lzc = Module(new lzc(ICACHE_SET_ASSOC))
  i_lzc.io.in_i  := vld_rdata.map(x => ~x)
  inv_way        := i_lzc.io.cnt_o
  all_ways_valid := i_lzc.io.empty_o

  // generate random cacheline index
  val i_lfsr = Module(new lfsr_8bit(ICACHE_SET_ASSOC))
  i_lfsr.io.en_i := update_lfsr
  // i_lfsr.refill_way_oh  :=
  rnd_way := i_lfsr.io.refill_way_bin


  ///////////////////////////////////////////////////////
  // tag comparison, hit generation
  ///////////////////////////////////////////////////////

  val hit_idx = Wire(UInt(log2Ceil(ICACHE_SET_ASSOC).W))

  for (i <- 0 until ICACHE_SET_ASSOC) { // : gen_tag_cmpsel
    cl_hit(i) := (cl_tag_rdata(i) === cl_tag_d) & vld_rdata(i)
    val cl_sel_data = Wire(Vec(ICACHE_LINE_WIDTH/FETCH_WIDTH, UInt(FETCH_WIDTH.W)))
    for(j <- 0 until ICACHE_LINE_WIDTH/FETCH_WIDTH) { cl_sel_data(j) := cl_rdata(i)(FETCH_WIDTH*j + FETCH_WIDTH-1, FETCH_WIDTH*j) }
    cl_sel(i) := cl_sel_data(cl_offset_q)
  }


  val i_lzc_hit = Module(new lzc(ICACHE_SET_ASSOC))
  i_lzc_hit.io.in_i := cl_hit
  hit_idx := i_lzc_hit.io.cnt_o
  // i_lzc_hit.empty_o :=

  val w_mem_rtrn_data = Wire(Vec(ICACHE_LINE_WIDTH/FETCH_WIDTH, UInt(FETCH_WIDTH.W)))
  for(j <- 0 until ICACHE_LINE_WIDTH/FETCH_WIDTH) { w_mem_rtrn_data(j) := io.mem_rtrn_if.bits.data(FETCH_WIDTH*j + FETCH_WIDTH-1, FETCH_WIDTH*j) }
  io.dreq_o.data := Mux(cmp_en_q, cl_sel(hit_idx), w_mem_rtrn_data(cl_offset_q))


  ///////////////////////////////////////////////////////
  // memory arrays and regs
  ///////////////////////////////////////////////////////

  val cl_tag_valid_rdata = Wire(Vec(ICACHE_SET_ASSOC, UInt((ICACHE_TAG_WIDTH+1).W)))

  for (i <- 0 until ICACHE_SET_ASSOC) { // gen_sram

    // Tag RAM
    val tag_sram = Module(new sram(ICACHE_TAG_WIDTH+1, ICACHE_NUM_WORDS))
    tag_sram.io.req_i   := vld_req(i)
    tag_sram.io.we_i    := vld_we
    tag_sram.io.addr_i  := vld_addr
    // we can always use the saved tag here since it takes a
    // couple of cycle until we write to the cache upon a miss
    tag_sram.io.wdata_i := Cat(vld_wdata(i), cl_tag_q)
    tag_sram.io.be_i    := VecInit(Seq.fill(ICACHE_TAG_WIDTH)(true.B)).asUInt
    cl_tag_valid_rdata(i) := tag_sram.io.rdata_o

    cl_tag_rdata(i) := cl_tag_valid_rdata(i)(ICACHE_TAG_WIDTH-1, 0)
    vld_rdata(i)    := cl_tag_valid_rdata(i)(ICACHE_TAG_WIDTH)

    // Data RAM
    val data_sram = Module(new sram(ICACHE_LINE_WIDTH, ICACHE_NUM_WORDS))
    data_sram.io.req_i   := cl_req(i)
    data_sram.io.we_i    := cl_we
    data_sram.io.addr_i  := cl_index
    data_sram.io.wdata_i := io.mem_rtrn_if.bits.data
    data_sram.io.be_i    := VecInit(Seq.fill(ICACHE_LINE_WIDTH)(true.B)).asUInt
    cl_rdata(i) := data_sram.io.rdata_o
  }


  cl_tag_q      := cl_tag_d
  flush_cnt_q   := flush_cnt_d
  vaddr_q       := vaddr_d
  cmp_en_q      := cmp_en_d
  cache_en_q    := cache_en_d
  flush_q       := flush_d
  state_q       := state_d
  cl_offset_q   := cl_offset_d
  repl_way_oh_q := repl_way_oh_d

}


object wt_icache extends App {
  class ariane_default_cfg extends ariane_cfg_t {
    val RASDepth   = 2
    val BTBEntries = 32
    val BHTEntries = 128
    // PMAs
    val NrNonIdempotentRules  = 2                        // Number of non idempotent rules
    val NonIdempotentAddrBase = Array(0, 0) // base which needs to match
    val NonIdempotentLength   = Array(0, 0) // bit mask which bits to consider when matching the rule
    val NrExecuteRegionRules  = 2                        // Number of regions which have execute property
    val ExecuteRegionAddrBase = Array(0, 0) // base which needs to match
    val ExecuteRegionLength   = Array(0, 0) // bit mask which bits to consider when matching the rule
    val NrCachedRegionRules   = 2                        // Number of regions which have cached property
    val CachedRegionAddrBase  = Array(0, 0) // base which needs to match
    val CachedRegionLength    = Array(0, 0) // bit mask which bits to consider when matching the rule
                                            // cache config
    val Axi64BitCompliant = false // set to 1 when using in conjunction with 64bit AXI bus adapter
    val SwapEndianess     = 0     // set to 1 to swap endianess inside L1.5 openpiton adapter
                                  //
    val DmBaseAddress  = 0         // offset of the debug module
  }

  val cfg = new ariane_default_cfg

  chisel3.Driver.execute(args, () => new wt_icache(1, cfg))
}
