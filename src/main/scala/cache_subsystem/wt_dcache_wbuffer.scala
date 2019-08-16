package cache_subsystem

import chisel3._
import chisel3.util._
import chisel3.Bool

import wt_cache_pkg._
import ariane_pkg._

// Description: coalescing write buffer for wb dcache
//
// A couple of notes:
//
// 1) the write buffer behaves as a fully-associative cache, and is therefore coalescing.
//    this cache is used by the cache readout logic to forward data to the load unit.
//
//    each byte can be in the following states (valid/dirty/txblock):
//
//    0/0/0:    invalid -> free entry in the buffer
//    1/1/0:    valid and dirty, Byte is hence not part of TX in-flight
//    1/0/1:    valid and not dirty, Byte is part of a TX in-flight
//    1/1/1:    valid and, part of tx and dirty. this means that the byte has been
//              overwritten while in TX and needs to be retransmitted once the write of that byte returns.
//    1/0/0:    this would represent a clean state, but is never reached in the wbuffer in the current implementation.
//              this is because when a TX returns, and the byte is in state [1/0/1], it is written to cache if needed and
//              its state is immediately cleared to 0/x/x.
//
//    this state is used to distinguish between bytes that have been written and not
//    yet sent to the memory subsystem, and bytes that are part of a transaction.
//
// 2) further, each word in the write buffer has a cache states (checked, hit_oh)
//
//    checked == 0: unknown cache state
//    checked == 1: cache state has been looked up, valid way is stored in "hit_oh"
//
//    cache invalidations/refills affecting a particular word will clear its word state to 0,
//    so another lookup has to be done. note that these lookups are triggered as soon as there is
//    a valid word with checked == 0 in the write buffer.
//
// 3) returning write ACKs trigger a cache update if the word is present in the cache, and evict that
//    word from the write buffer. if the word is not allocated to the cache, it is just evicted from the write buffer.
//    if the word cache state is VOID, the pipeline is stalled until it is clear whether that word is in the cache or not.
//
// 4) we handle NC writes using the writebuffer circuitry. upon an NC request, the writebuffer will first be drained.
//    then, only the NC word is written into the write buffer and no further write requests are acknowledged until that
//    word has been evicted from the write buffer.

class wt_dcache_wbuffer (
  ArianeCfg:ariane_cfg_t     // contains cacheable regions
) extends Module {
  val io = IO(new Bundle {
    val cache_en_i = Input(Bool())  // writes are treated as NC if disabled
    val empty_o    = Output(Bool()) // asserted if no data is present in write buffer

    // core request ports
    val req_port_i = Input (new dcache_req_i_t())
    val req_port_o = Output(new dcache_req_o_t())

    // interface to miss handler
    val miss_ack_i      = Input(Bool())
    val miss_paddr_o    = Output(UInt(64.W))
    val miss_req_o      = Output(Bool())
    val miss_we_o       = Output(Bool())                    // always 1 here
    val miss_wdata_o    = Output(UInt(64.W))
    val miss_vld_bits_o = Output(UInt(DCACHE_SET_ASSOC.W))  // unused here (set to 0)
    val miss_nc_o       = Output(Bool())                    // request to I/O space
    val miss_size_o     = Output(UInt(3.W))                 //
    val miss_id_o       = Output(UInt(CACHE_ID_WIDTH.W))    // ID of this transaction (wbuffer uses all IDs from 0 to DCACHE_MAX_TX-1)
                                                            // write responses from memory
    val miss_rtrn_vld_i = Input(Bool())
    val miss_rtrn_id_i  = Input(UInt(CACHE_ID_WIDTH.W))     // transaction ID to clear
                                                            // cache read interface
    val rd_tag_o      = Output(UInt(DCACHE_TAG_WIDTH.W   )) // tag in - comes one cycle later
    val rd_idx_o      = Output(UInt(DCACHE_CL_IDX_WIDTH.W))
    val rd_off_o      = Output(UInt(DCACHE_OFFSET_WIDTH.W))
    val rd_req_o      = Output(Bool())                      // read the word at offset off_i[:3] in all ways
    val rd_tag_only_o = Output(Bool())                      // set to 1 here as we do not have to read the data arrays
    val rd_ack_i      = Input(Bool())
    val rd_data_i     = Input(UInt(64.W))                   // unused
    val rd_vld_bits_i = Input(UInt(DCACHE_SET_ASSOC.W))     // unused
    val rd_hit_oh_i   = Input(UInt(DCACHE_SET_ASSOC.W))
    // cacheline writes
    val wr_cl_vld_i = Input(Bool())
    val wr_cl_idx_i = Input(UInt(DCACHE_CL_IDX_WIDTH.W))
    // cache word write interface
    val wr_req_o     = Output(UInt(DCACHE_SET_ASSOC.W))
    val wr_ack_i     = Input(Bool())
    val wr_idx_o     = Output(UInt(DCACHE_CL_IDX_WIDTH.W))
    val wr_off_o     = Output(UInt(DCACHE_OFFSET_WIDTH.W))
    val wr_data_o    = Output(UInt(64.W))
    val wr_data_be_o = Output(UInt(8.W))
    // to forwarding logic and miss unit
    val wbuffer_data_o = Output(Vec(DCACHE_WBUF_DEPTH, new wbuffer_t()))
    val tx_paddr_o = Output(Vec(DCACHE_MAX_TX, UInt(64.W)))           // used to check for address collisions with read operations
    val tx_vld_o   = Output(Vec(DCACHE_MAX_TX, Bool()))
  })

  val tx_stat_d      = Wire(Vec(DCACHE_MAX_TX, new tx_stat_t()))
  val tx_stat_q      = Reg(Vec(DCACHE_MAX_TX, new tx_stat_t()))
  val wbuffer_d      = Wire(Vec(DCACHE_WBUF_DEPTH, new wbuffer_t()))
  val wbuffer_q      = Reg(Vec(DCACHE_WBUF_DEPTH, new wbuffer_t()))
  val valid          = Wire(Vec(DCACHE_WBUF_DEPTH, Bool()))
  val dirty          = Wire(Vec(DCACHE_WBUF_DEPTH, Bool()))
  val tocheck        = Wire(Vec(DCACHE_WBUF_DEPTH, Bool()))
  val wbuffer_hit_oh = Wire(Vec(DCACHE_WBUF_DEPTH, Bool()))
  val inval_hit      = Wire(Vec(DCACHE_WBUF_DEPTH, Bool()))
  val bdirty         = Wire(Vec(DCACHE_WBUF_DEPTH, UInt(8.W)))

  val next_ptr = Wire(UInt(log2Ceil(DCACHE_WBUF_DEPTH).W))
  val dirty_ptr    = Wire(UInt(log2Ceil(DCACHE_WBUF_DEPTH).W))
  val hit_ptr      = Wire(UInt(log2Ceil(DCACHE_WBUF_DEPTH).W))
  val wr_ptr       = Wire(UInt(log2Ceil(DCACHE_WBUF_DEPTH).W))
  val check_ptr_d  = Wire(UInt(log2Ceil(DCACHE_WBUF_DEPTH).W))
  val check_ptr_q  = RegInit(0.U(log2Ceil(DCACHE_WBUF_DEPTH).W))
  val check_ptr_q1 = RegInit(0.U(log2Ceil(DCACHE_WBUF_DEPTH).W))
  val rtrn_ptr     = Wire(UInt(log2Ceil(DCACHE_WBUF_DEPTH).W))
  val tx_id        = Wire(UInt(CACHE_ID_WIDTH.W))
  val rtrn_id      = Wire(UInt(CACHE_ID_WIDTH.W))

  val bdirty_off = Wire(UInt(3.W))
  val tx_be      = Wire(UInt(8.W))
  val wr_paddr   = Wire(UInt(64.W))
  val rd_paddr   = Wire(UInt(64.W))
  val rd_tag_d = Wire(UInt(DCACHE_TAG_WIDTH.W))
  val rd_tag_q = RegInit(0.U(DCACHE_TAG_WIDTH.W))
  val rd_hit_oh_d = Wire(UInt(DCACHE_SET_ASSOC.W))
  val rd_hit_oh_q = RegInit(0.U(DCACHE_SET_ASSOC.W))
  val check_en_d    = Wire(Bool())
  val check_en_q    = RegInit(false.B)
  val check_en_q1   = RegInit(false.B)
  val full          = Reg(Bool())
  val dirty_rd_en   = Reg(Bool())
  val rdy           = Reg(Bool())
  val rtrn_empty    = Reg(Bool())
  val evict         = Reg(Bool())
  val nc_pending_d  = Wire(Bool())
  val nc_pending_q  = RegInit(false.B)
  val addr_is_nc    = Reg(Bool())
  val wbuffer_wren  = Reg(Bool())
  val free_tx_slots = Reg(Bool())

  val wr_cl_vld_q = RegInit(false.B)
  val wr_cl_vld_d = Wire(Bool())
  val wr_cl_idx_q = RegInit(0.U(DCACHE_CL_IDX_WIDTH.W))
  val wr_cl_idx_d = Wire(UInt(DCACHE_CL_IDX_WIDTH.W))

  val debug_paddr = Wire(Vec(DCACHE_WBUF_DEPTH, UInt(64.W)))

  val wbuffer_check_mux = Wire(new wbuffer_t())
  val wbuffer_dirty_mux = Wire(new wbuffer_t())

  ///////////////////////////////////////////////////////
  // misc
  ///////////////////////////////////////////////////////

  io.miss_nc_o := nc_pending_q

  // noncacheable if request goes to I/O space, or if cache is disabled
  addr_is_nc := (~io.cache_en_i) |
                (~is_inside_cacheable_regions(ArianeCfg, Cat(io.req_port_i.address_tag, VecInit(Seq.fill(DCACHE_INDEX_WIDTH)(false.B)).asUInt)))

  io.miss_we_o       := true.B
  io.miss_vld_bits_o := 0.U
  io.wbuffer_data_o  := wbuffer_q

  for (k <- 0 until DCACHE_MAX_TX) { // begin : gen_tx_vld
    io.tx_vld_o(k)   := tx_stat_q(k).vld
    io.tx_paddr_o(k) := wbuffer_q(tx_stat_q(k).ptr).wtag<<3
  }

  ///////////////////////////////////////////////////////
  // openpiton does not understand byte enable sigs
  // need to convert to the four cases:
  // 00: byte
  // 01: halfword
  // 10: word
  // 11: dword
  // non-contiguous writes need to be serialized!
  // e.g. merged dwords with BE like this: 8'b01001100
  ///////////////////////////////////////////////////////

  // get byte offset
  val i_vld_bdirty = Module(new lzc(8))
  i_vld_bdirty.io.in_i := bdirty(dirty_ptr).toBools
  bdirty_off := i_vld_bdirty.io.cnt_o
  // i_vld_bdirty.io.empty_o :=

  // add the offset to the physical base address of this buffer entry
  io.miss_paddr_o := Cat(wbuffer_dirty_mux.wtag, bdirty_off)
  io.miss_id_o    := tx_id

  // is there any dirty word to be transmitted, and is there a free TX slot?
  io.miss_req_o := (dirty.foldLeft(false.B)(_|_)) && free_tx_slots

  // get size of aligned words, and the corresponding byte enables
  // note: openpiton can only handle aligned offsets + size, and hence
  // we have to split unaligned data into multiple transfers (see toSize64)
  // e.g. if we have the following valid bytes: 0011_1001 -> TX0: 0000_0001, TX1: 0000_1000, TX2: 0011_0000
  io.miss_size_o  := toSize64(bdirty(dirty_ptr))

  // replicate transfers shorter than a dword
  io.miss_wdata_o := repData64(wbuffer_dirty_mux.data,
                               bdirty_off,
                               io.miss_size_o(1, 0))
  tx_be := toByteEnable8(bdirty_off, io.miss_size_o(1, 0))

  ///////////////////////////////////////////////////////
  // TX status registers and ID counters
  ///////////////////////////////////////////////////////

  // TODO: todo: make this fall through if timing permits it
  val i_rtrn_id_fifo = Module(new fifo_v3(UInt(CACHE_ID_WIDTH.W), false, DCACHE_MAX_TX))
  i_rtrn_id_fifo.io.flush_i    := false.B
  i_rtrn_id_fifo.io.testmode_i := false.B
  // i_rtrn_id_fifo.io.full_o     :=
  rtrn_empty := i_rtrn_id_fifo.io.empty_o
  // i_rtrn_id_fifo.io.usage_o    :=
  i_rtrn_id_fifo.io.data_i     := io.miss_rtrn_id_i
  i_rtrn_id_fifo.io.push_i     := io.miss_rtrn_vld_i
  rtrn_id := i_rtrn_id_fifo.io.data_o
  i_rtrn_id_fifo.io.pop_i      := evict

  for (i <- 0 until DCACHE_MAX_TX) { tx_stat_d(i) := tx_stat_q(i) }
  evict       := false.B
  io.wr_req_o := 0.U

  // clear entry if it is clear whether it can be pushed to the cache or not
  when ((!rtrn_empty) && wbuffer_q(rtrn_ptr).checked) {
    // check if data is clean and can be written, otherwise skip
    // check if CL is present, otherwise skip
    when ((io.wr_data_be_o.orR) && (wbuffer_q(rtrn_ptr).hit_oh.orR)) {
      io.wr_req_o := wbuffer_q(rtrn_ptr).hit_oh
      when (io.wr_ack_i) {
        evict    := true.B
        tx_stat_d(rtrn_id).vld := false.B
      }
    } .otherwise {
      evict := true.B
      tx_stat_d(rtrn_id).vld := false.B
    }
  }

  // allocate a new entry
  when (dirty_rd_en) {
    tx_stat_d(tx_id).vld := true.B
    tx_stat_d(tx_id).ptr := dirty_ptr
    tx_stat_d(tx_id).be  := tx_be
  }

  free_tx_slots := io.tx_vld_o.map(x => ~x).foldLeft(false.B)(_|_)

  // next word to lookup in the cache
  val i_tx_id_rr = Module(new rr_arb_tree(UInt(1.W), DCACHE_MAX_TX, false, false, true))
  i_tx_id_rr.io.flush_i := false.B
  i_tx_id_rr.io.rr_i    := 0.U
  i_tx_id_rr.io.req_i   := ~(io.tx_vld_o.asUInt)
  // i_tx_id_rr.io.gnt_o   :=
  // i_tx_id_rr.io.data_i  := 0.U
  i_tx_id_rr.io.gnt_i   := dirty_rd_en
  // i_tx_id_rr.io.req_o   :=
  // i_tx_id_rr.io.data_o  :=
  tx_id := i_tx_id_rr.io.idx_o


///////////////////////////////////////////////////////
// cache readout & update
///////////////////////////////////////////////////////

  rd_tag_d   := rd_paddr >> DCACHE_INDEX_WIDTH

  // trigger TAG readout in cache
  io.rd_tag_only_o := true.B
  rd_paddr         := wbuffer_check_mux.wtag << 3
  io.rd_req_o      := tocheck.foldLeft(false.B)(_|_)
  io.rd_tag_o      := rd_tag_q;//delay by one cycle
  io.rd_idx_o      := rd_paddr(DCACHE_INDEX_WIDTH-1, DCACHE_OFFSET_WIDTH)
  io.rd_off_o      := rd_paddr(DCACHE_OFFSET_WIDTH-1, 0)
  check_en_d       := io.rd_req_o & io.rd_ack_i

  // cache update port
  rtrn_ptr       := tx_stat_q(rtrn_id).ptr
  // if we wrote into a word while it was in-flight, we cannot write the dirty bytes to the cache
  // when the TX returns
  io.wr_data_be_o := tx_stat_q(rtrn_id).be & (~wbuffer_q(rtrn_ptr).dirty)
  wr_paddr        := wbuffer_q(rtrn_ptr).wtag << 3
  io.wr_idx_o     := wr_paddr(DCACHE_INDEX_WIDTH-1, DCACHE_OFFSET_WIDTH)
  io.wr_off_o     := wr_paddr(DCACHE_OFFSET_WIDTH-1, 0)
  io.wr_data_o    := wbuffer_q(rtrn_ptr).data


  ///////////////////////////////////////////////////////
  // readout of status bits, index calculation
  ///////////////////////////////////////////////////////

  val wtag_comp = Wire(Vec(DCACHE_WBUF_DEPTH, UInt(DCACHE_CL_IDX_WIDTH.W)))

  wr_cl_vld_d := io.wr_cl_vld_i
  wr_cl_idx_d := io.wr_cl_idx_i

  for (k <- 0 until DCACHE_WBUF_DEPTH) { // : gen_flags
    // only for debug, will be pruned
    debug_paddr(k) := wbuffer_q(k).wtag << 3

    // dirty bytes that are ready for transmission.
    // note that we cannot retransmit a word that is already in-flight
    // since the multiple transactions might overtake each other in the memory system!
    bdirty(k) := Mux(wbuffer_q(k).txblock.orR, 0.U, wbuffer_q(k).dirty & wbuffer_q(k).valid)

    dirty(k)          := bdirty(k).orR
    valid(k)          := wbuffer_q(k).valid.orR
    wbuffer_hit_oh(k) := valid(k) & (wbuffer_q(k).wtag === Cat(io.req_port_i.address_tag, io.req_port_i.address_index(DCACHE_INDEX_WIDTH-1, 3)))

    // checks if an invalidation/cache refill hits a particular word
    // note: an invalidation can hit multiple words!
    // need to respect previous cycle, too, since we add a cycle of latency to the rd_hit_oh_i signal...
    wtag_comp(k)  := wbuffer_q(k).wtag(DCACHE_INDEX_WIDTH-4, DCACHE_OFFSET_WIDTH-3)
    inval_hit(k)  := (wr_cl_vld_d & valid(k) & (wtag_comp(k) === wr_cl_idx_d)) |
                     (wr_cl_vld_q & valid(k) & (wtag_comp(k) === wr_cl_idx_q))

    // these word have to be looked up in the cache
    tocheck(k)       := (~wbuffer_q(k).checked) & valid(k)
  }

  wr_ptr     := Mux(wbuffer_hit_oh.foldLeft(false.B)(_|_), hit_ptr, next_ptr)
  io.empty_o := ~(valid.foldLeft(false.B)(_|_))
  rdy        := (wbuffer_hit_oh.foldLeft(false.B)(_|_)) | (~full)

  // next free entry in the buffer
  val i_vld_lzc = Module(new lzc(DCACHE_WBUF_DEPTH ))
  i_vld_lzc.io.in_i    := valid.map(x => ~x)
  next_ptr := i_vld_lzc.io.cnt_o
  full     := i_vld_lzc.io.empty_o

  // get index of hit
  val i_hit_lzc = Module(new lzc(DCACHE_WBUF_DEPTH))
  i_hit_lzc.io.in_i    := wbuffer_hit_oh
  hit_ptr := i_hit_lzc.io.cnt_o
  // i_hit_lzc.io.empty_o :=

  // next dirty word to serve
  val i_dirty_rr = Module(new rr_arb_tree(new wbuffer_t, DCACHE_WBUF_DEPTH, false, false, true))
  i_dirty_rr.io.flush_i := 0.U
  i_dirty_rr.io.rr_i    := 0.U
  i_dirty_rr.io.req_i   := dirty
  // i_dirty_rr.io.gnt_o   :=
  i_dirty_rr.io.data_i  := wbuffer_q
  i_dirty_rr.io.gnt_i   := dirty_rd_en
  // i_dirty_rr.io.req_o   :=
  wbuffer_dirty_mux := i_dirty_rr.io.data_o
  dirty_ptr         := i_dirty_rr.io.idx_o

  // next word to lookup in the cache
  val i_clean_rr = Module(new rr_arb_tree(new wbuffer_t, DCACHE_WBUF_DEPTH, false, false, false))
  i_clean_rr.io.flush_i:= 0.U
  i_clean_rr.io.rr_i   := 0.U
  i_clean_rr.io.req_i  := tocheck
  // i_clean_rr.io.gnt_o  :=
  i_clean_rr.io.data_i := wbuffer_q
  i_clean_rr.io.gnt_i  := check_en_d
  // i_clean_rr.io.req_o  :=
  wbuffer_check_mux := i_clean_rr.io.data_o
  check_ptr_d       := i_clean_rr.io.idx_o

  ///////////////////////////////////////////////////////
  // update logic
  ///////////////////////////////////////////////////////

  io.req_port_o.data_rvalid := 0.U
  io.req_port_o.data_rdata  := 0.U

  rd_hit_oh_d := io.rd_hit_oh_i

  // TODO: rewrite and separate into MUXES and write strobe logic
  wbuffer_d              := wbuffer_q
  nc_pending_d           := nc_pending_q
  dirty_rd_en            := false.B
  io.req_port_o.data_gnt := false.B
  wbuffer_wren           := false.B

  // TAG lookup returns, mark corresponding word
  when (check_en_q1) {
    when (wbuffer_q(check_ptr_q1).valid.orR) {
      wbuffer_d(check_ptr_q1).checked := true.B
      wbuffer_d(check_ptr_q1).hit_oh := rd_hit_oh_q
    }
  }

  // if an invalidation or cache line refill comes in and hits on the write buffer,
  // we have to discard our knowledge of the corresponding cacheline state
  for (k <- 0 until DCACHE_WBUF_DEPTH) {
    when (inval_hit(k)) {
      wbuffer_d(k).checked := false.B
    }
  }

  // once TX write response came back, we can clear the TX block. if it was not dirty, we
  // can completely evict it - otherwise we have to leave it there for retransmission
  when (evict) {
    for (k <- 0 until 8) {
      when (tx_stat_q(rtrn_id).be(k)) {
        wbuffer_d(rtrn_ptr).txblock(k) := false.B
        when (!wbuffer_q(rtrn_ptr).dirty(k)) {
          wbuffer_d(rtrn_ptr).valid(k) := false.B

          // NOTE: this is not strictly needed, but makes it much
          // easier to debug, since no invalid data remains in the buffer
          // wbuffer_d(rtrn_ptr).data[k*8 +:8] := 0.U
        }
      }
    }
    // if all bytes are evicted, clear the cache status flag
    when (wbuffer_d(rtrn_ptr).valid === 0.U) {
      wbuffer_d(rtrn_ptr).checked := false.B
    }
  }

  // mark bytes sent out to the memory system
  when (io.miss_req_o && io.miss_ack_i) {
    dirty_rd_en := true.B
      for (k <- 0 until 8) {
        when (tx_be(k)) {
          wbuffer_d(dirty_ptr).dirty(k)   := false.B
          wbuffer_d(dirty_ptr).txblock(k) := true.B
        }
      }
  }

  // write new word into the buffer
  when (io.req_port_i.data_req && rdy) {
    // in case we have an NC address, need to drain the buffer first
    // in case we are serving an NC address,  we block until it is written to memory
    when (io.empty_o || !(addr_is_nc || nc_pending_q)) {
      wbuffer_wren              := true.B

      io.req_port_o.data_gnt    := true.B
      nc_pending_d              := addr_is_nc

      wbuffer_d(wr_ptr).checked := false.B
      wbuffer_d(wr_ptr).wtag    := Cat(io.req_port_i.address_tag, io.req_port_i.address_index(DCACHE_INDEX_WIDTH-1, 3))

      // mark bytes as dirty
      for (k <- 0 until 8) {
        when (io.req_port_i.data_be(k)) {
          wbuffer_d(wr_ptr).valid(k)       := true.B
          wbuffer_d(wr_ptr).dirty(k)       := true.B
          wbuffer_d(wr_ptr).data(k*8+7,k*8) := io.req_port_i.data_wdata(k*8+7,k*8)
        }
      }
    }
  }


  ///////////////////////////////////////////////////////
  // ff's
  ///////////////////////////////////////////////////////

  wbuffer_q     := wbuffer_d
  for (i <- 0 until DCACHE_MAX_TX) { tx_stat_q(i) := tx_stat_d(i) }
  nc_pending_q  := nc_pending_d
  check_ptr_q   := check_ptr_d
  check_ptr_q1  := check_ptr_q
  check_en_q    := check_en_d
  check_en_q1   := check_en_q
  rd_tag_q      := rd_tag_d
  rd_hit_oh_q   := rd_hit_oh_d
  wr_cl_vld_q   := wr_cl_vld_d
  wr_cl_idx_q   := wr_cl_idx_d

  ///////////////////////////////////////////////////////
  // assertions
  ///////////////////////////////////////////////////////

// //pragma translate_off
// `ifndef VERILATOR
//
//   hot1: assert property (
//     @(posedge clk_i) disable iff (!rst_ni) req_port_i.data_req |-> $onehot0(wbuffer_hit_oh))
//       else $fatal(1,"[l1 dcache wbuffer] wbuffer_hit_oh signal must be hot1")
//
//   tx_status: assert property (
//     @(posedge clk_i) disable iff (!rst_ni) evict && miss_ack_i && miss_req_o |-> (tx_id != rtrn_id))
//       else $fatal(1,"[l1 dcache wbuffer] cannot allocate and clear same tx slot id in the same cycle")
//
//   tx_valid0: assert property (
//     @(posedge clk_i) disable iff (!rst_ni) evict |-> tx_stat_q(rtrn_id).vld)
//       else $fatal(1,"[l1 dcache wbuffer] evicting invalid transaction slot")
//
//   tx_valid1: assert property (
//     @(posedge clk_i) disable iff (!rst_ni) evict |-> |wbuffer_q(rtrn_ptr).valid)
//       else $fatal(1,"[l1 dcache wbuffer] wbuffer entry corresponding to this transaction is invalid")
//
//   write_full: assert property (
//     @(posedge clk_i) disable iff (!rst_ni) req_port_i.data_req |-> req_port_o.data_gnt |-> ((!full) || (|wbuffer_hit_oh)))
//       else $fatal(1,"[l1 dcache wbuffer] cannot write if full or no hit")
//
//   unused0: assert property (
//     @(posedge clk_i) disable iff (!rst_ni) !req_port_i.tag_valid)
//       else $fatal(1,"[l1 dcache wbuffer] req_port_i.tag_valid should not be asserted")
//
//   unused1: assert property (
//     @(posedge clk_i) disable iff (!rst_ni) !req_port_i.kill_req)
//       else $fatal(1,"[l1 dcache wbuffer] req_port_i.kill_req should not be asserted")
//
//   for (genvar k=0; k<DCACHE_WBUF_DEPTH; k++) { : gen_assert1
//     for (genvar j=0; j<8; j++) { : gen_assert2
//       byteStates: assert property (
//         @(posedge clk_i) disable iff (!rst_ni) {wbuffer_q(k).valid[j], wbuffer_q(k).dirty[j], wbuffer_q(k).txblock[j]} inside {3'b000, 3'b110, 3'b101, 3'b111} )
//           else $fatal(1,"[l1 dcache wbuffer] byte %02d of wbuffer entry %02d has invalid state: valid=%01b, dirty=%01b, txblock=%01b",
//             j,k,
//             wbuffer_q(k).valid[j],
//             wbuffer_q(k).dirty[j],
//             wbuffer_q(k).txblock[j])
//     }
//   }
// `endif
// //pragma translate_on

}


object wt_dcache_wbuffer extends App {
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

  chisel3.Driver.execute(args, () => new wt_dcache_wbuffer(cfg))
}
