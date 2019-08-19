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

    val mem_rtrn_vld_i = Input(Bool())
    val mem_rtrn_i     = Input(new dcache_rtrn_t())
    val mem_data_req_o = Output(Bool())
    val mem_data_ack_i = Input(Bool())
    val mem_data_o     = Output(new dcache_req_t())
  })

  // LD unit and PTW
  val NumPorts:Int = 3

  // miss unit <-> read controllers
  val cache_en = Wire(Bool())

  // miss unit <-> memory
  val wr_cl_vld     = Wire(Bool())
  val wr_cl_nc      = Wire(Bool())
  val wr_cl_we      = Wire(UInt(DCACHE_SET_ASSOC.W))
  val wr_cl_tag     = Wire(UInt(DCACHE_TAG_WIDTH.W))
  val wr_cl_idx     = Wire(UInt(DCACHE_CL_IDX_WIDTH.W))
  val wr_cl_off     = Wire(UInt(DCACHE_OFFSET_WIDTH.W))
  val wr_cl_data    = Wire(UInt(DCACHE_LINE_WIDTH.W))
  val wr_cl_data_be = Wire(UInt((DCACHE_LINE_WIDTH/8).W))
  val wr_vld_bits   = Wire(UInt(DCACHE_SET_ASSOC.W))
  val wr_req        = Wire(Vec(DCACHE_SET_ASSOC, Bool()))
  val wr_ack        = Wire(Bool())
  val wr_idx        = Wire(UInt(DCACHE_CL_IDX_WIDTH.W))
  val wr_off        = Wire(UInt(DCACHE_OFFSET_WIDTH.W))
  val wr_data       = Wire(UInt(64.W))
  val wr_data_be    = Wire(UInt(8.W))

  // miss unit <-> controllers/wbuffer
  val miss_req      = Wire(Vec(NumPorts, Bool()))
  val miss_ack      = Wire(Vec(NumPorts, Bool()))
  val miss_nc       = Wire(Vec(NumPorts, Bool()))
  val miss_we       = Wire(Vec(NumPorts, Bool()))
  val miss_wdata    = Wire(Vec(NumPorts, UInt(64.W)))
  val miss_paddr    = Wire(Vec(NumPorts, UInt(64.W)))
  val miss_vld_bits = Wire(Vec(NumPorts, Vec(DCACHE_SET_ASSOC, Bool())))
  val miss_size     = Wire(Vec(NumPorts, UInt(3.W)))
  val miss_id       = Wire(Vec(NumPorts, UInt(CACHE_ID_WIDTH.W)))
  val miss_replay   = Wire(Vec(NumPorts, Bool()))
  val miss_rtrn_vld = Wire(Vec(NumPorts, Bool()))
  val miss_rtrn_id  = Wire(UInt(CACHE_ID_WIDTH.W))

  // memory <-> read controllers/miss unit
  val w_rd_if = Wire(Vec(NumPorts, new dcache_rd_if()))

  val rd_data     = Wire(UInt(64.W))
  val rd_vld_bits = Wire(Vec(DCACHE_SET_ASSOC, Bool()))
  val rd_hit_oh   = Wire(Vec(DCACHE_SET_ASSOC, Bool()))

  // miss unit <-> wbuffer
  val tx_paddr = Wire(Vec(DCACHE_MAX_TX, UInt(64.W)))
  val tx_vld   = Wire(Vec(DCACHE_MAX_TX, Bool()))

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
  // miss handling interface
  i_wt_dcache_missunit.io.miss_req_i         := miss_req
  miss_ack := i_wt_dcache_missunit.io.miss_ack_o
  i_wt_dcache_missunit.io.miss_nc_i          := miss_nc
  i_wt_dcache_missunit.io.miss_we_i          := miss_we
  i_wt_dcache_missunit.io.miss_wdata_i       := miss_wdata
  i_wt_dcache_missunit.io.miss_paddr_i       := miss_paddr
  i_wt_dcache_missunit.io.miss_vld_bits_i    := miss_vld_bits
  i_wt_dcache_missunit.io.miss_size_i        := miss_size
  i_wt_dcache_missunit.io.miss_id_i          := miss_id
  miss_replay   := i_wt_dcache_missunit.io.miss_replay_o
  miss_rtrn_vld := i_wt_dcache_missunit.io.miss_rtrn_vld_o
  miss_rtrn_id  := i_wt_dcache_missunit.io.miss_rtrn_id_o
    // from writebuffer
  i_wt_dcache_missunit.io.tx_paddr_i         := tx_paddr
  i_wt_dcache_missunit.io.tx_vld_i           := tx_vld
    // cache memory interface
  wr_cl_vld     := i_wt_dcache_missunit.io.wr_cl_vld_o
  wr_cl_nc      := i_wt_dcache_missunit.io.wr_cl_nc_o
  wr_cl_we      := i_wt_dcache_missunit.io.wr_cl_we_o
  wr_cl_tag     := i_wt_dcache_missunit.io.wr_cl_tag_o
  wr_cl_idx     := i_wt_dcache_missunit.io.wr_cl_idx_o
  wr_cl_off     := i_wt_dcache_missunit.io.wr_cl_off_o
  wr_cl_data    := i_wt_dcache_missunit.io.wr_cl_data_o
  wr_cl_data_be := i_wt_dcache_missunit.io.wr_cl_data_be_o
  wr_vld_bits   := i_wt_dcache_missunit.io.wr_vld_bits_o
    // memory interface
  i_wt_dcache_missunit.io.mem_rtrn_vld_i     := io.mem_rtrn_vld_i
  i_wt_dcache_missunit.io.mem_rtrn_i         := io.mem_rtrn_i
  io.mem_data_req_o := i_wt_dcache_missunit.io.mem_data_req_o
  i_wt_dcache_missunit.io.mem_data_ack_i     := io.mem_data_ack_i
  io.mem_data_o := i_wt_dcache_missunit.io.mem_data_o


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
    miss_req (k) := i_wt_dcache_ctrl.io.miss_req_o
    i_wt_dcache_ctrl.io.miss_ack_i      := miss_ack(k)
    miss_we(k)       := i_wt_dcache_ctrl.io.miss_we_o
    miss_wdata(k)    := i_wt_dcache_ctrl.io.miss_wdata_o
    miss_vld_bits(k) := i_wt_dcache_ctrl.io.miss_vld_bits_o
    miss_paddr (k)   := i_wt_dcache_ctrl.io.miss_paddr_o
    miss_nc    (k)   := i_wt_dcache_ctrl.io.miss_nc_o
    miss_size  (k)   := i_wt_dcache_ctrl.io.miss_size_o
    miss_id    (k)   := i_wt_dcache_ctrl.io.miss_id_o
    i_wt_dcache_ctrl.io.miss_replay_i   := miss_replay(k)
    i_wt_dcache_ctrl.io.miss_rtrn_vld_i := miss_rtrn_vld (k)
    // used to detect readout mux collisions
    i_wt_dcache_ctrl.io.wr_cl_vld_i     := wr_cl_vld

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
  miss_req(2) := i_wt_dcache_wbuffer.io.miss_req_o
  i_wt_dcache_wbuffer.io.miss_ack_i      := miss_ack(2)
  miss_we(2)       := i_wt_dcache_wbuffer.io.miss_we_o
  miss_wdata(2)    := i_wt_dcache_wbuffer.io.miss_wdata_o
  miss_vld_bits(2) := i_wt_dcache_wbuffer.io.miss_vld_bits_o
  miss_paddr(2)    := i_wt_dcache_wbuffer.io.miss_paddr_o
  miss_nc(2)       := i_wt_dcache_wbuffer.io.miss_nc_o
  miss_size(2)     := i_wt_dcache_wbuffer.io.miss_size_o
  miss_id(2)       := i_wt_dcache_wbuffer.io.miss_id_o
  i_wt_dcache_wbuffer.io.miss_rtrn_vld_i := miss_rtrn_vld(2)
  i_wt_dcache_wbuffer.io.miss_rtrn_id_i  := miss_rtrn_id

  // cache read interface
  i_wt_dcache_wbuffer.io.rd_if <> w_rd_if(2)

  i_wt_dcache_wbuffer.io.rd_data_i       := rd_data
  i_wt_dcache_wbuffer.io.rd_vld_bits_i   := rd_vld_bits
  i_wt_dcache_wbuffer.io.rd_hit_oh_i     := rd_hit_oh
     // incoming invalidations/cache refills
  i_wt_dcache_wbuffer.io.wr_cl_vld_i     := wr_cl_vld
  i_wt_dcache_wbuffer.io.wr_cl_idx_i     := wr_cl_idx
    // single word write interface
  wr_req := i_wt_dcache_wbuffer.io.wr_req_o
  i_wt_dcache_wbuffer.io.wr_ack_i := wr_ack
  wr_idx     := i_wt_dcache_wbuffer.io.wr_idx_o
  wr_off     := i_wt_dcache_wbuffer.io.wr_off_o
  wr_data    := i_wt_dcache_wbuffer.io.wr_data_o
  wr_data_be := i_wt_dcache_wbuffer.io.wr_data_be_o
  // write buffer forwarding
  wbuffer_data := i_wt_dcache_wbuffer.io.wbuffer_data_o
  tx_paddr     := i_wt_dcache_wbuffer.io.tx_paddr_o
  tx_vld       := i_wt_dcache_wbuffer.io.tx_vld_o

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
  i_wt_dcache_mem.io.wr_cl_vld_i       := wr_cl_vld
  i_wt_dcache_mem.io.wr_cl_nc_i        := wr_cl_nc
  i_wt_dcache_mem.io.wr_cl_we_i        := wr_cl_we
  i_wt_dcache_mem.io.wr_cl_tag_i       := wr_cl_tag
  i_wt_dcache_mem.io.wr_cl_idx_i       := wr_cl_idx
  i_wt_dcache_mem.io.wr_cl_off_i       := wr_cl_off
  i_wt_dcache_mem.io.wr_cl_data_i      := wr_cl_data
  i_wt_dcache_mem.io.wr_cl_data_be_i   := wr_cl_data_be
  i_wt_dcache_mem.io.wr_vld_bits_i     := wr_vld_bits
  // single word write port
  i_wt_dcache_mem.io.wr_req_i          := wr_req
  wr_ack := i_wt_dcache_mem.io.wr_ack_o
  i_wt_dcache_mem.io.wr_idx_i          := wr_idx
  i_wt_dcache_mem.io.wr_off_i          := wr_off
  i_wt_dcache_mem.io.wr_data_i         := wr_data
  i_wt_dcache_mem.io.wr_data_be_i      := wr_data_be
  // write buffer forwarding
  i_wt_dcache_mem.io.wbuffer_data_i    := wbuffer_data
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
