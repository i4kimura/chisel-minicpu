package cache_subsystem

import chisel3._
import chisel3.util._
import chisel3.Bool

import wt_cache_pkg._
import ariane_pkg._

class wt_dcache (
  // ID to be used for read and AMO transactions.
  // note that the write buffer uses all IDs up to DCACHE_MAX_TX-1 for write transactions
  RdAmoTxId:Int    = 1,
  // contains cacheable regions
  ArianeCfg:ariane_cfg_t)
    extends Module {
  val io = IO(new Bundle {
    // Cache management
    val enable_i        = Input(Bool())  // from CSR
    val flush_i         = Input(Bool())  // high until acknowledged
    val flush_ack_o     = Output(Bool()) // send a single cycle acknowledge signal when the cache is flushed
    val miss_o          = Output(Bool()) // we missed on a ld/st
    val wbuffer_empty_o = Output(Bool())

    // AMO interface
    val amo_req_i = Input(new amo_req_t())
    val amo_resp_o = Output(new amo_resp_t())

    // Request ports
    val req_ports_i = Input(Vec(3, new dcache_req_i_t()))
    val req_ports_o = Output(Vec(3, new dcache_req_o_t()))

    val mem_rtrn_if = Flipped(ValidIO(new dcache_rtrn_t()))
    val mem_data_if = DecoupledIO(new dcache_req_t())
  })

  // LD unit and PTW
  val NumPorts:Int = 3

  // miss unit <-> read controllers
  val cache_en = Wire(Bool())

  // miss unit <-> memory
  val w_wr_cl_if = Wire(new dcache_write_if())
  val wr_vld_bits   = Wire(UInt(DCACHE_SET_ASSOC.W))

  val w_wr_if    = Wire(new dcache_word_wr_if())

  // miss unit <-> controllers/wbuffer
  val w_miss_if = Wire(Vec(NumPorts, new dcache_miss_if()))
  val miss_rtrn_id  = Wire(UInt(CACHE_ID_WIDTH.W))

  // memory <-> read controllers/miss unit
  val w_rd_if = Wire(Vec(NumPorts, new dcache_rd_if()))

  val rd_data     = Wire(UInt(64.W))
  val rd_vld_bits = Wire(Vec(DCACHE_SET_ASSOC, Bool()))
  val rd_hit_oh   = Wire(Vec(DCACHE_SET_ASSOC, Bool()))

  // miss unit <-> wbuffer
  val w_tx_paddr_if = Wire(Vec(DCACHE_MAX_TX, ValidIO(UInt(64.W))))
  // val tx_paddr = Wire(Vec(DCACHE_MAX_TX, UInt(64.W)))
  // val tx_vld   = Wire(Vec(DCACHE_MAX_TX, Bool()))

  // wbuffer <-> memory
  val wbuffer_data = Wire(Vec(DCACHE_WBUF_DEPTH, new wbuffer_t()))


  ///////////////////////////////////////////////////////
  // miss handling unit
  ///////////////////////////////////////////////////////

  val i_wt_dcache_missunit = Module(new wt_dcache_missunit(ArianeCfg.Axi64BitCompliant, RdAmoTxId, NumPorts))
  i_wt_dcache_missunit.io.enable_i           := io.enable_i
  i_wt_dcache_missunit.io.flush_i            := io.flush_i
  io.flush_ack_o := i_wt_dcache_missunit.io.flush_ack_o
  io.miss_o      := i_wt_dcache_missunit.io.miss_o
  i_wt_dcache_missunit.io.wbuffer_empty_i    := io.wbuffer_empty_o
  cache_en       := i_wt_dcache_missunit.io.cache_en_o
  // amo interface
  i_wt_dcache_missunit.io.amo_req_i          := io.amo_req_i
  io.amo_resp_o := i_wt_dcache_missunit.io.amo_resp_o

  i_wt_dcache_missunit.io.miss_if <> w_miss_if    // miss handling interface
  miss_rtrn_id := i_wt_dcache_missunit.io.miss_rtrn_id_o

  // from writebuffer
  i_wt_dcache_missunit.io.tx_paddr_if <> w_tx_paddr_if

  // cache memory interface
  i_wt_dcache_missunit.io.wr_cl_if <> w_wr_cl_if

  wr_vld_bits   := i_wt_dcache_missunit.io.wr_vld_bits_o

  // memory interface
  i_wt_dcache_missunit.io.mem_rtrn_if <> io.mem_rtrn_if
  i_wt_dcache_missunit.io.mem_data_if <> io.mem_data_if


  ///////////////////////////////////////////////////////
  // read controllers (LD unit and PTW/MMU)
  ///////////////////////////////////////////////////////

  // note: last read port is used by the write buffer
  for (k <- 0 until NumPorts-1) { // : gen_rd_ports
    val i_wt_dcache_ctrl = Module(new wt_dcache_ctrl (RdAmoTxId, ArianeCfg))
    i_wt_dcache_ctrl.io.cache_en_i      := cache_en
    // reqs from core
    i_wt_dcache_ctrl.io.req_port_i      := io.req_ports_i(k)
    io.req_ports_o(k) := i_wt_dcache_ctrl.io.req_port_o

    // miss interface
    i_wt_dcache_ctrl.io.miss_if <> w_miss_if(k)

    // used to detect readout mux collisions
    i_wt_dcache_ctrl.io.wr_cl_vld_i     := w_wr_cl_if.vld

    // cache mem interface
    i_wt_dcache_ctrl.io.rd_if <> w_rd_if(k)

    i_wt_dcache_ctrl.io.rd_data_i       := rd_data
    i_wt_dcache_ctrl.io.rd_vld_bits_i   := rd_vld_bits
    i_wt_dcache_ctrl.io.rd_hit_oh_i     := rd_hit_oh
  }
  ///////////////////////////////////////////////////////
  // store unit controller
  ///////////////////////////////////////////////////////

  val i_wt_dcache_wbuffer = Module(new wt_dcache_wbuffer (ArianeCfg))
  io.wbuffer_empty_o := i_wt_dcache_wbuffer.io.empty_o
  // TODO: fix this
  i_wt_dcache_wbuffer.io.cache_en_i      := cache_en
  // .cache_en_i      := '0
  // request ports from core (store unit)
  i_wt_dcache_wbuffer.io.req_port_i      := io.req_ports_i(2)
  io.req_ports_o(2) := i_wt_dcache_wbuffer.io.req_port_o

  // miss unit interface
  i_wt_dcache_wbuffer.io.miss_if <> w_miss_if(2)
  i_wt_dcache_wbuffer.io.miss_rtrn_id_i := miss_rtrn_id

  // cache read interface
  i_wt_dcache_wbuffer.io.rd_if <> w_rd_if(2)

  i_wt_dcache_wbuffer.io.rd_data_i       := rd_data
  i_wt_dcache_wbuffer.io.rd_vld_bits_i   := rd_vld_bits
  i_wt_dcache_wbuffer.io.rd_hit_oh_i     := rd_hit_oh
  // incoming invalidations/cache refills
  i_wt_dcache_wbuffer.io.wr_cl_vld_i     := w_wr_cl_if.vld
  i_wt_dcache_wbuffer.io.wr_cl_idx_i     := w_wr_cl_if.idx

  // single word write interface
  i_wt_dcache_wbuffer.io.wr_if <> w_wr_if

  // write buffer forwarding
  wbuffer_data := i_wt_dcache_wbuffer.io.wbuffer_data_o
  i_wt_dcache_wbuffer.io.tx_paddr_if <> w_tx_paddr_if

  ///////////////////////////////////////////////////////
  // memory arrays, arbitration and tag comparison
  ///////////////////////////////////////////////////////

  val i_wt_dcache_mem = Module(new wt_dcache_mem (ArianeCfg.Axi64BitCompliant, NumPorts))
  // read ports
  i_wt_dcache_mem.io.rd_if <> w_rd_if

  rd_vld_bits := i_wt_dcache_mem.io.rd_vld_bits_o
  rd_hit_oh   := i_wt_dcache_mem.io.rd_hit_oh_o
  rd_data     := i_wt_dcache_mem.io.rd_data_o

  // cacheline write port
  i_wt_dcache_mem.io.wr_cl_if <> w_wr_cl_if
  i_wt_dcache_mem.io.wr_vld_bits_i     := wr_vld_bits

  i_wt_dcache_mem.io.wr_if <> w_wr_if // single word write port
  i_wt_dcache_mem.io.wbuffer_data_i := wbuffer_data  // write buffer forwarding
}


object wt_dcache extends App {
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

  chisel3.Driver.execute(args, () => new wt_dcache(1, cfg))
}
