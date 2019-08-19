// Author: Michael Schaffner <schaffner@iis.ee.ethz.ch>, ETH Zurich
// Date: 13.09.2018
// Description: miss controller for wb dcache. Note that the current assumption
// is that the port with the highest index issues writes instead of reads.

package cache_subsystem

import chisel3._
import chisel3.util._
import chisel3.Bool

import wt_cache_pkg._
import ariane_pkg._


class wt_dcache_missunit (
  Axi64BitCompliant:Boolean = false, // set this to 1 when using in conjunction with 64bit AXI bus adapter
  AmoTxId          :Int     = 1,    // TX id to be used for AMOs
  NumPorts         :Int     = 3     // number of miss ports
) extends Module {
  val io = IO(new Bundle {
    // cache management, signals from/to core
    val enable_i    = Input (Bool()) // from CSR
    val flush_i     = Input (Bool()) // flush request, this waits for pending tx (write, read) to finish and will clear the cache
    val flush_ack_o = Output(Bool()) // send a single cycle acknowledge signal when the cache is flushed
    val miss_o      = Output(Bool()) // we missed on a ld/st
                                     // local cache management signals
    val wbuffer_empty_i = Input (Bool())
    val cache_en_o      = Output(Bool()) // local cache enable signal
                                         // AMO interface
    val amo_req_i  = Input (new amo_req_t())
    val amo_resp_o = Output(new amo_resp_t())

    val miss_if        = Flipped(Vec(NumPorts, new dcache_miss_if())) // miss handling interface (ld, ptw, wbuffer)
    val miss_rtrn_id_o = Output(UInt(CACHE_ID_WIDTH.W))

    // from writebuffer
    // used to check for address collisions with read operations
    val tx_paddr_if = Flipped(Vec(DCACHE_MAX_TX, ValidIO(UInt(64.W))))

    // write interface to cache memory
    val wr_cl_if = new dcache_write_if()
    val wr_vld_bits_o   = Output(UInt(DCACHE_SET_ASSOC.W   ))

    // memory interface
    val mem_rtrn_if = Flipped(ValidIO(new dcache_rtrn_t()))
    val mem_data_if = DecoupledIO(new dcache_req_t())
  })

  // controller FSM
  val idle_s :: drain_s :: amo_s :: flush_s :: store_wait_s :: load_wait_s :: amo_wait_s :: Nil = Enum(7)
  val state_q = RegInit (idle_s)
  val state_d = WireInit (idle_s)

  // MSHR for reads
  class mshr_t extends Bundle {
    val paddr          = UInt(64.W)
    val size           = UInt(3.W)
    val vld_bits       = Vec(DCACHE_SET_ASSOC, Bool())
    val id             = UInt(CACHE_ID_WIDTH.W)
    val nc             = Bool()
    val repl_way       = UInt(log2Ceil(DCACHE_SET_ASSOC).W)
    val miss_port_idx  = UInt(log2Ceil(NumPorts).W)
  }

  val mshr_d = Wire(new mshr_t())
  val mshr_q = Reg(new mshr_t())
  val repl_way = Wire(UInt(log2Ceil(DCACHE_SET_ASSOC).W))
  val inv_way  = Wire(UInt(log2Ceil(DCACHE_SET_ASSOC).W))
  val rnd_way  = Wire(UInt(log2Ceil(DCACHE_SET_ASSOC).W))
  val mshr_vld_d    = Wire(Bool())
  val mshr_vld_q    = RegInit(false.B)
  val mshr_vld_q1   = RegInit(false.B)
  val mshr_allocate = RegInit(false.B)
  val update_lfsr    = Wire(Bool())
  val all_ways_valid = Wire(Bool())

  val enable_d      = Wire(Bool())
  val enable_q      = RegInit(false.B)
  val flush_ack_d   = Wire(Bool())
  val flush_ack_q   = RegInit(false.B)
  val flush_en      = Wire(Bool())
  val flush_done    = Wire(Bool())
  val mask_reads    = Wire(Bool())
  val lock_reqs     = Wire(Bool())
  val amo_sel       = Wire(Bool())
  val miss_is_write = Wire(Bool())
  val amo_req_d     = Wire(Bool())
  val amo_req_q     = RegInit(false.B)
  val amo_data      = Wire(UInt(64.W))
  val tmp_paddr     = Wire(UInt(64.W))
  val amo_rtrn_mux  = Wire(UInt(64.W))

  val miss_port_idx = Wire(UInt(log2Ceil(NumPorts).W))
  val cnt_d = Wire(UInt(DCACHE_CL_IDX_WIDTH.W))
  val cnt_q = RegInit (0.U(DCACHE_CL_IDX_WIDTH.W))
  val miss_req_masked_d = Wire(Vec(NumPorts, Bool()))
  val miss_req_masked_q = RegInit(VecInit(Seq.fill(NumPorts)(false.B)))

  val inv_vld     = Wire(Bool())
  val inv_vld_all = Wire(Bool())
  val cl_write_en = Wire(Bool())
  val load_ack    = Wire(Bool())
  val store_ack   = Wire(Bool())
  val amo_ack     = Wire(Bool())

  val mshr_rdrd_collision_d = Wire(Vec(NumPorts, Bool()))
  val mshr_rdrd_collision_q = RegInit(VecInit(Seq.fill(NumPorts)(false.B)))
  val mshr_rdrd_collision   = Wire(Vec(NumPorts, Bool()))
  val mshr_rdwr_collision   = Wire(Bool())

  ///////////////////////////////////////////////////////
  // input arbitration and general control sigs
  ///////////////////////////////////////////////////////

  io.cache_en_o   := enable_q
  cnt_d           := Mux(flush_en, cnt_q + 1.U, 0.U)
  flush_done      := cnt_q === (DCACHE_NUM_WORDS-1).U

  for(i <- 0 until NumPorts) {
    miss_req_masked_d(i) := Mux(lock_reqs,  miss_req_masked_q(i),
                            Mux(mask_reads, io.miss_if(i).we & io.miss_if(i).req,
                               io.miss_if(i).req))
  }
  miss_is_write     := io.miss_if(miss_port_idx).we

  // read port arbiter
  val i_lzc_reqs = Module (new lzc(NumPorts))
  i_lzc_reqs.io.in_i    := miss_req_masked_d
  miss_port_idx := i_lzc_reqs.io.cnt_o
  // i_lzc_reqs.io.empty_o

  io.miss_if.map(x => x.ack := false.B)
  when (!amo_sel) {
    io.miss_if(miss_port_idx).ack := io.mem_data_if.ready & io.mem_data_if.valid
  }

  ///////////////////////////////////////////////////////
  // MSHR and way replacement logic (only for read ops)
  ///////////////////////////////////////////////////////

  // find invalid cache line
  val i_lzc_inv = Module(new lzc(DCACHE_SET_ASSOC))
  i_lzc_inv.io.in_i := io.miss_if(miss_port_idx).vld_bits.map(x => ~x)
  inv_way           := i_lzc_inv.io.cnt_o
  all_ways_valid    := i_lzc_inv.io.empty_o


  // generate random cacheline index
  val i_lfsr_inv = Module(new lfsr_8bit(0, DCACHE_SET_ASSOC))
  i_lfsr_inv.io.en_i := update_lfsr
  // i_lfsr_inv.refill_way_oh
  rnd_way := i_lfsr_inv.io.refill_way_bin

  repl_way               := Mux(all_ways_valid, rnd_way, inv_way)

  mshr_d.size            := Mux(mshr_allocate, io.miss_if(miss_port_idx).size    , mshr_q.size)
  mshr_d.paddr           := Mux(mshr_allocate, io.miss_if(miss_port_idx).paddr   , mshr_q.paddr)
  mshr_d.vld_bits        := Mux(mshr_allocate, io.miss_if(miss_port_idx).vld_bits, mshr_q.vld_bits)
  mshr_d.id              := Mux(mshr_allocate, io.miss_if(miss_port_idx).id      , mshr_q.id)
  mshr_d.nc              := Mux(mshr_allocate, io.miss_if(miss_port_idx).nc      , mshr_q.nc)
  mshr_d.repl_way        := Mux(mshr_allocate, repl_way                          , mshr_q.repl_way)
  mshr_d.miss_port_idx   := Mux(mshr_allocate, miss_port_idx                     , mshr_q.miss_port_idx)

  // currently we only have one outstanding read TX, hence an incoming load clears the MSHR
  mshr_vld_d := Mux(mshr_allocate, true.B,
                   Mux(load_ack, false.B, mshr_vld_q))

  io.miss_o := Mux(mshr_allocate, ~io.miss_if(miss_port_idx).nc, false.B)


  for(k<-0 until NumPorts) { // : gen_rdrd_collision
    mshr_rdrd_collision(k)   := (mshr_q.paddr(63, DCACHE_OFFSET_WIDTH) === io.miss_if(k).paddr(63, DCACHE_OFFSET_WIDTH)) && (mshr_vld_q | mshr_vld_q1)
    mshr_rdrd_collision_d(k) := Mux(!io.miss_if(k).req, false.B, mshr_rdrd_collision_q(k)) | mshr_rdrd_collision(k)
  }

  // read/write collision, stalls the corresponding request
  // write collides with MSHR
  mshr_rdwr_collision := (mshr_q.paddr(63, DCACHE_OFFSET_WIDTH) === io.miss_if(NumPorts-1).paddr(63, DCACHE_OFFSET_WIDTH)) && mshr_vld_q

  // read collides with inflight TX
  var tx_rdwr_collision = WireInit(false.B)
  for(k <- 0 until DCACHE_MAX_TX) {
    tx_rdwr_collision = tx_rdwr_collision | ((io.miss_if(miss_port_idx).paddr(63, DCACHE_OFFSET_WIDTH) === io.tx_paddr_if(k).bits(63, DCACHE_OFFSET_WIDTH)) && io.tx_paddr_if(k).valid)
  }

  ///////////////////////////////////////////////////////
  // to memory
  ///////////////////////////////////////////////////////

  // if size = 32bit word, select appropriate offset, replicate for openpiton...
  amo_data := Mux(io.amo_req_i.size === 2.U, Cat(io.amo_req_i.operand_b(31, 0),
                                                 io.amo_req_i.operand_b(31, 0)),
                                                 io.amo_req_i.operand_b)

  val amo_rtrn_mux_word_w = Wire(UInt(32.W))
  // note: openpiton returns a full cacheline!
  if (Axi64BitCompliant) { // begin : gen_axi_rtrn_mux
    amo_rtrn_mux := io.mem_rtrn_if.bits.data(63, 0)
  } else {
    val mem_rtrn_vec_w = Wire(Vec(io.mem_rtrn_if.bits.data.getWidth / 64, UInt(64.W)))
    for(i <- 0 until io.mem_rtrn_if.bits.data.getWidth / 64) {
      mem_rtrn_vec_w(i) := io.mem_rtrn_if.bits.data(64*i + 63, 64*i)
    }
    amo_rtrn_mux := mem_rtrn_vec_w(io.amo_req_i.operand_a(DCACHE_OFFSET_WIDTH-1, 3))
  }
  when (io.amo_req_i.operand_a(2)) {
    amo_rtrn_mux_word_w := amo_rtrn_mux(63, 32)
  } .otherwise {
    amo_rtrn_mux_word_w := amo_rtrn_mux(31,  0)
  }

  // always sign extend 32bit values
  io.amo_resp_o.result := Mux(io.amo_req_i.size === 2.U,
                              Cat(VecInit(Seq.fill(32)(amo_rtrn_mux_word_w(31))).asUInt, amo_rtrn_mux_word_w),
                              amo_rtrn_mux)

  amo_req_d := io.amo_req_i.req

  // outgoing memory requests (AMOs are always uncached)
  io.mem_data_if.bits.tid    := Mux(amo_sel, AmoTxId.U          , io.miss_if(miss_port_idx).id)
  io.mem_data_if.bits.nc     := Mux(amo_sel, true.B             , io.miss_if(miss_port_idx).nc)
  io.mem_data_if.bits.way    := Mux(amo_sel, 0.U                , repl_way)
  io.mem_data_if.bits.data   := Mux(amo_sel, amo_data           , io.miss_if(miss_port_idx).wdata)
  io.mem_data_if.bits.size   := Mux(amo_sel, io.amo_req_i.size  , io.miss_if(miss_port_idx).size)
  io.mem_data_if.bits.amo_op := Mux(amo_sel, io.amo_req_i.amo_op, AMO_NONE)

  tmp_paddr            := Mux(amo_sel, io.amo_req_i.operand_a, io.miss_if(miss_port_idx).paddr)
  io.mem_data_if.bits.paddr  := paddrSizeAlign(tmp_paddr, io.mem_data_if.bits.size)

  ///////////////////////////////////////////////////////
  // back-off mechanism for LR/SC completion guarantee
  ///////////////////////////////////////////////////////

  val sc_fail = Wire(Bool())
  val sc_pass = Wire(Bool())
  val sc_backoff_over = Wire(Bool())
  val i_exp_backoff = Module(new exp_backoff(3, 16))
  i_exp_backoff.io.set_i     := sc_fail
  i_exp_backoff.io.clr_i     := sc_pass
  sc_backoff_over := i_exp_backoff.io.is_zero_o

  ///////////////////////////////////////////////////////
  // responses from memory
  ///////////////////////////////////////////////////////

  // keep track of pending stores
  val store_sent = Wire(Bool())
  val stores_inflight_d = Wire(UInt(log2Ceil(DCACHE_MAX_TX + 1).W))
  val stores_inflight_q = RegInit(0.U(log2Ceil(DCACHE_MAX_TX + 1).W))
  store_sent := io.mem_data_if.valid & io.mem_data_if.ready & (io.mem_data_if.bits.rtype === DCACHE_STORE_REQ)

  stores_inflight_d := Mux(store_ack && store_sent, stores_inflight_q,
                       Mux(store_ack              , stores_inflight_q - 1.U,
                       Mux(store_sent             , stores_inflight_q + 1.U,
                           stores_inflight_q)))

  // incoming responses
  load_ack           := false.B
  store_ack          := false.B
  amo_ack            := false.B
  inv_vld            := false.B
  inv_vld_all        := false.B
  sc_fail            := false.B
  sc_pass            := false.B
  io.miss_if.map(x => x.rtrn_vld := false.B)
  when (io.mem_rtrn_if.valid) {
    switch (io.mem_rtrn_if.bits.rtype) {
      is (dcache_load_ack_s) {
        when (mshr_vld_q) {
          load_ack := true.B
          io.miss_if(mshr_q.miss_port_idx).rtrn_vld := true.B
        }
      }
      is (dcache_store_ack_s) {
        when (stores_inflight_q =/= 0.U) {
          store_ack := true.B
          io.miss_if((NumPorts-1).U).rtrn_vld := true.B
        }
      }
      is (dcache_atomic_ack_s) {
        when (amo_req_q) {
          amo_ack := true.B
          // need to set SC backoff counter if
          // this op failed
          when (io.amo_req_i.amo_op === AMO_SC) {
            when (io.amo_resp_o.result =/= 0.U) {
              sc_fail := true.B
            } .otherwise {
              sc_pass := true.B
            }
          }
        }
      }
      is (dcache_inv_req_s) {
        inv_vld     := io.mem_rtrn_if.bits.inv.vld | io.mem_rtrn_if.bits.inv.all
        inv_vld_all := io.mem_rtrn_if.bits.inv.all
      }
      // TODO:
        // DCACHE_INT_REQ: {
        // }
    }
  }

  // to write buffer
  io.miss_rtrn_id_o := io.mem_rtrn_if.bits.tid

///////////////////////////////////////////////////////
// writes to cache memory
///////////////////////////////////////////////////////

  // cacheline write port
  io.wr_cl_if.nc      := mshr_q.nc
  io.wr_cl_if.vld     := load_ack | (io.wr_cl_if.we.orR)

  io.wr_cl_if.we     := Mux(flush_en      , true.B,
                        Mux(inv_vld_all   , true.B,
                        Mux(inv_vld       , dcache_way_bin2oh(io.mem_rtrn_if.bits.inv.way),
                        Mux(cl_write_en   , dcache_way_bin2oh(mshr_q.repl_way) , false.B))))

  io.wr_cl_if.idx     := Mux(flush_en, cnt_q,
                         Mux(inv_vld,  io.mem_rtrn_if.bits.inv.idx(DCACHE_INDEX_WIDTH-1, DCACHE_OFFSET_WIDTH),
                                      mshr_q.paddr(DCACHE_INDEX_WIDTH-1, DCACHE_OFFSET_WIDTH)))

  io.wr_cl_if.tag     := mshr_q.paddr(DCACHE_TAG_WIDTH+DCACHE_INDEX_WIDTH-1, DCACHE_INDEX_WIDTH)
  io.wr_cl_if.off     := mshr_q.paddr(DCACHE_OFFSET_WIDTH-1, 0)
  io.wr_cl_if.data    := io.mem_rtrn_if.bits.data
  io.wr_cl_if.data_be := Mux(cl_write_en, true.B, false.B)   // we only write complete cachelines into the memory

  io.wr_vld_bits_o   := Mux(flush_en   , 0.U,
                        Mux(inv_vld    , 0.U,
                        Mux(cl_write_en, dcache_way_bin2oh(mshr_q.repl_way), 0.U)))

  // only NC responses write to the cache
  cl_write_en     := load_ack & ~mshr_q.nc

  ///////////////////////////////////////////////////////
  // main control logic for generating tx
  ///////////////////////////////////////////////////////

  // default assignment
  state_d          := state_q

  io.flush_ack_o      := false.B
  io.mem_data_if.bits.rtype := DCACHE_LOAD_REQ
  io.mem_data_if.valid   := false.B
  io.amo_resp_o.ack   := false.B
  io.miss_if.map(x => x.replay := false.B)

  // disabling cache is possible anytime, enabling goes via flush
  enable_d         := enable_q & io.enable_i
  flush_ack_d      := flush_ack_q
  flush_en         := false.B
  amo_sel          := false.B
  update_lfsr      := false.B
  mshr_allocate    := false.B
  lock_reqs        := false.B
  mask_reads       := mshr_vld_q

  // interfaces
  switch (state_q) {
    //////////////////////////////////
    // wait for misses / amo ops
    is(idle_s) {
      when (io.flush_i || (io.enable_i && !enable_q)) {
        when (io.wbuffer_empty_i && !mshr_vld_q) {
          flush_ack_d := io.flush_i
          state_d     := flush_s
        } .otherwise {
          state_d     := drain_s
        }
      } .elsewhen (io.amo_req_i.req) {
        when (io.wbuffer_empty_i && !mshr_vld_q) {
          state_d     := amo_s
        } .otherwise {
          state_d     := drain_s
        }
        // we've got a miss to handle
      } .elsewhen (miss_req_masked_d.asUInt.orR) {
        // this is a write miss, just pass through (but check whether write collides with MSHR)
        when (miss_is_write) {
          // stall in case this write collides with the MSHR address
          when (!mshr_rdwr_collision) {
            io.mem_data_if.valid            := true.B
            io.mem_data_if.bits.rtype          := DCACHE_STORE_REQ
            when (!io.mem_data_if.ready) {
              state_d := store_wait_s
            }
          }
          // this is a read miss, can only allocate 1 MSHR
          // in case of a load_ack we can accept a new miss, since the MSHR is being cleared
        } .elsewhen (!mshr_vld_q || load_ack) {
          // replay the read request in case the address has collided with MSHR during the time the request was pending
          // i.e., the cache state may have been updated in the mean time due to a refill at the same CL address
          when (mshr_rdrd_collision_d(miss_port_idx)) {
            io.miss_if(miss_port_idx).replay := true.B
            // stall in case this CL address overlaps with a write TX that is in flight
          } .elsewhen (!tx_rdwr_collision) {
            io.mem_data_if.valid         := true.B
            io.mem_data_if.bits.rtype       := DCACHE_LOAD_REQ
            update_lfsr               := all_ways_valid & io.mem_data_if.ready;// need to evict a random way
            mshr_allocate             := io.mem_data_if.ready
            when (!io.mem_data_if.ready) {
              state_d := load_wait_s
            }
          }
        }
      }
    }
    //////////////////////////////////
    // wait until this request is acked
    is (store_wait_s) {
      lock_reqs                 := true.B
      io.mem_data_if.valid            := true.B
      io.mem_data_if.bits.rtype          := DCACHE_STORE_REQ
      when (io.mem_data_if.ready) {
        state_d := idle_s
      }
    }
    //////////////////////////////////
    // wait until this request is acked
    is(load_wait_s) {
      lock_reqs                 := true.B
      io.mem_data_if.valid            := true.B
      io.mem_data_if.bits.rtype          := DCACHE_LOAD_REQ
      when (io.mem_data_if.ready) {
        update_lfsr   := all_ways_valid;// need to evict a random way
        mshr_allocate := true.B;
        state_d       := idle_s
      }
    }
    //////////////////////////////////
    // only handle stores, do not accept new read requests
    // wait until MSHR is cleared and wbuffer is empty
    is (drain_s) {
      mask_reads := true.B
      // these are writes, check whether they collide with MSHR
      when (miss_req_masked_d.asUInt.orR && !mshr_rdwr_collision) {
        io.mem_data_if.valid            := true.B
        io.mem_data_if.bits.rtype          := DCACHE_STORE_REQ
      }

      when (io.wbuffer_empty_i && !mshr_vld_q) {
        state_d := idle_s
      }
    }
    //////////////////////////////////
    // flush the cache
    is (flush_s) {
      // internal flush signal
      flush_en   := true.B
      when (flush_done) {
        state_d     := idle_s
        io.flush_ack_o := flush_ack_q
        flush_ack_d := false.B
        enable_d    := io.enable_i
      }
    }
    //////////////////////////////////
    // send out amo op request
    is (amo_s) {
      io.mem_data_if.bits.rtype := DCACHE_ATOMIC_REQ
      amo_sel          := true.B
      // if this is an LR, we need to consult the backoff counter
      when ((io.amo_req_i.amo_op =/= AMO_LR) || sc_backoff_over) {
        io.mem_data_if.valid   := true.B
        when (io.mem_data_if.ready) {
          state_d := amo_wait_s
        }
      }
    }
    //////////////////////////////////
    // block and wait until AMO OP returns
    is (amo_wait_s) {
      amo_sel := true.B
      when (amo_ack) {
        io.amo_resp_o.ack := true.B
        state_d        := idle_s
      }
    }
  }

  ///////////////////////////////////////////////////////
  // ff's
  ///////////////////////////////////////////////////////

  state_q               := state_d
  cnt_q                 := cnt_d
  enable_q              := enable_d
  flush_ack_q           := flush_ack_d
  mshr_vld_q            := mshr_vld_d
  mshr_vld_q1           := mshr_vld_q
  mshr_q                := mshr_d
  mshr_rdrd_collision_q := mshr_rdrd_collision_d
  miss_req_masked_q     := miss_req_masked_d
  amo_req_q             := amo_req_d
  stores_inflight_q     := stores_inflight_d

}


object wt_dcache_missunit extends App {
  chisel3.Driver.execute(args, () => new wt_dcache_missunit(false, 1, 3))
}
