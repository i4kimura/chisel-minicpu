package cache_subsystem

import chisel3._
import chisel3.util._
import chisel3.Bool

import wt_cache_pkg._
import ariane_pkg._

class wt_dcache_mem
  ( Axi64BitCompliant: Boolean = false, // set this to 1 when using in conjunction with 64bit AXI bus adapter
    NumPorts: Int = 3)
    extends Module {
  val io = IO(new Bundle {
    // ports
    val rd_tag_i      = Input (Vec(NumPorts, UInt(DCACHE_TAG_WIDTH.W   )))              // tag in - comes one cycle later
    val rd_idx_i      = Input (Vec(NumPorts, UInt(DCACHE_CL_IDX_WIDTH.W)))
    val rd_off_i      = Input (Vec(NumPorts, UInt(DCACHE_OFFSET_WIDTH.W)))
    val rd_req_i      = Input (Vec(NumPorts, Bool()))                                    // read the word at offset off_i[:3] in all ways
    val rd_tag_only_i = Input (Vec(NumPorts, Bool()))                                    // only do a tag/valid lookup, no access to data arrays
    val rd_prio_i     = Input (Vec(NumPorts, Bool()))                                    // 0: low prio, 1: high prio
    val rd_ack_o      = Output(Vec(NumPorts, Bool()))
    val rd_vld_bits_o = Output(Vec(DCACHE_SET_ASSOC, Bool()))
    val rd_hit_oh_o   = Output(Vec(DCACHE_SET_ASSOC, Bool()))
    val rd_data_o     = Output(UInt(64.W))

    // only available on port 0, uses address signals of port 0
    val wr_cl_vld_i     = Input(Bool())
    val wr_cl_nc_i      = Input(Bool())                                   // noncacheable access
    val wr_cl_we_i      = Input(UInt(DCACHE_SET_ASSOC.W     ))            // writes a full cacheline
    val wr_cl_tag_i     = Input(UInt(DCACHE_TAG_WIDTH.W     ))
    val wr_cl_idx_i     = Input(UInt(DCACHE_CL_IDX_WIDTH.W  ))
    val wr_cl_off_i     = Input(UInt(DCACHE_OFFSET_WIDTH.W  ))
    val wr_cl_data_i    = Input(UInt(DCACHE_LINE_WIDTH.W    ))
    val wr_cl_data_be_i = Input(UInt((DCACHE_LINE_WIDTH/8).W))
    val wr_vld_bits_i   = Input(UInt(DCACHE_SET_ASSOC.W     ))

    // separate port for single word write, no tag access
    val wr_req_i     = Input (Vec(DCACHE_SET_ASSOC, Bool()))            // write a single word to offset off_i[:3]
    val wr_ack_o     = Output(Bool())
    val wr_idx_i     = Input (UInt(DCACHE_CL_IDX_WIDTH.W))
    val wr_off_i     = Input (UInt(DCACHE_OFFSET_WIDTH.W))
    val wr_data_i    = Input (UInt(64.W))
    val wr_data_be_i = Input (UInt(8.W))

    // forwarded wbuffer
    val wbuffer_data_i = Input(Vec(DCACHE_WBUF_DEPTH, new wbuffer_t()))
  })

  val bank_req   = Wire(UInt(DCACHE_NUM_BANKS.W))
  val bank_we    = Wire(UInt(DCACHE_NUM_BANKS.W))
  val bank_be    = Wire(Vec(DCACHE_NUM_BANKS, Vec(DCACHE_SET_ASSOC, UInt(8.W))))
  val bank_idx   = Wire(Vec(DCACHE_NUM_BANKS, UInt(DCACHE_CL_IDX_WIDTH.W)))
  val bank_idx_d = Wire(UInt(DCACHE_CL_IDX_WIDTH.W))
  val bank_idx_q = RegInit(0.U(DCACHE_CL_IDX_WIDTH.W))
  val bank_off_d = Wire(UInt(DCACHE_OFFSET_WIDTH.W))
  val bank_off_q = RegInit(0.U(DCACHE_OFFSET_WIDTH.W))

  val bank_wdata = Reg(Vec(DCACHE_NUM_BANKS, Vec(DCACHE_SET_ASSOC, UInt(64.W)))) //
  val bank_rdata = Reg(Vec(DCACHE_NUM_BANKS, Vec(DCACHE_SET_ASSOC, UInt(64.W)))) //
  val rdata_cl   = Reg(Vec(DCACHE_SET_ASSOC, UInt(64.W)))                        // selected word from each cacheline

  val rd_tag    = Wire(UInt(DCACHE_TAG_WIDTH.W))
  val vld_req   = Wire(UInt(DCACHE_SET_ASSOC.W))                                   // bit enable for valid regs
  val vld_we    = Wire(Bool())                                                   // valid bits write enable
  val vld_wdata = Wire(UInt(DCACHE_SET_ASSOC.W))                                   // valid bits to write
  val tag_rdata = Wire(Vec(DCACHE_SET_ASSOC, UInt(DCACHE_TAG_WIDTH.W)))          // these are the tags coming from the tagmem
  val vld_addr  = Wire(UInt(DCACHE_CL_IDX_WIDTH.W))                              // valid bit

  val vld_sel_d = Wire(UInt(log2Ceil(NumPorts).W))
  val vld_sel_q = RegInit(0.U(log2Ceil(NumPorts).W))

  val wbuffer_hit_oh   = Wire(Vec(DCACHE_WBUF_DEPTH, Bool()))
  val wbuffer_be       = Wire(Vec(8, Bool()))
  val wbuffer_rdata    = Wire(Vec(8, UInt(8.W)))
  val rdata            = Wire(UInt(64.W))
  val wbuffer_cmp_addr = Wire(UInt(64.W))

  val cmp_en_d = Wire(Bool())
  val cmp_en_q = RegInit(false.B)
  val rd_acked = Wire(Bool())
  val bank_collision = Wire(Vec(NumPorts, Bool()))
  val rd_req_masked  = Wire(Vec(NumPorts, Bool()))
  val rd_req_prio    = Wire(Vec(NumPorts, Bool()))

  ///////////////////////////////////////////////////////
  // arbiter
  ///////////////////////////////////////////////////////

  // Priority is highest for lowest read port index
  //
  // SRAM bank mapping:
  //
  // Bank 0                   Bank 2
  // [way0, w0] [way1, w0] .. [way0, w1] [way1, w1] ..

  // byte enable mapping
  for (k <- 0 until DCACHE_NUM_BANKS) {
    for (j <- 0 until DCACHE_SET_ASSOC) {
      bank_be(k)(j) := Mux(io.wr_cl_we_i(j) & io.wr_cl_vld_i, io.wr_cl_data_be_i(k*8+7, k*8),
                       Mux(io.wr_req_i  (j) & io.wr_ack_o,    io.wr_data_be_i, 0.U))

      bank_wdata(k)(j) := Mux(io.wr_cl_we_i(j) & io.wr_cl_vld_i, io.wr_cl_data_i(k*64+63, k*64),
                              io.wr_data_i)
    }
  }

  vld_wdata  := io.wr_vld_bits_i;
  vld_addr   := Mux(io.wr_cl_vld_i, io.wr_cl_idx_i, io.rd_idx_i(vld_sel_d))
  rd_tag     := io.rd_tag_i(vld_sel_q)                               // delayed by one cycle
  bank_off_d := Mux(io.wr_cl_vld_i, io.wr_cl_off_i, io.rd_off_i(vld_sel_d))
  bank_idx_d := Mux(io.wr_cl_vld_i, io.wr_cl_idx_i, io.rd_idx_i(vld_sel_d))
  vld_req    := Mux(io.wr_cl_vld_i, io.wr_cl_we_i , Mux(rd_acked, true.B, false.B))

  // priority masking
  // disable low prio requests when any of the high prio reqs is present
  for(i <- 0 until NumPorts) {
    rd_req_prio(i)   := io.rd_req_i(i) & io.rd_prio_i(i)
    rd_req_masked(i) := Mux(rd_req_prio.asUInt.orR, rd_req_prio(i), io.rd_req_i(i))
  }
  val rd_req = Wire(Bool())
  val wire_zero = Wire(Vec(NumPorts, UInt(1.W)))
  for (i <- 0 until NumPorts) { wire_zero(i) := 0.U }
  val i_rr_arb_tree = Module(new rr_arb_tree(UInt(1.W), NumPorts, false, false, false))
  i_rr_arb_tree.io.flush_i := false.B
  i_rr_arb_tree.io.rr_i    := VecInit(Seq.fill(log2Ceil(NumPorts))(false.B))
  i_rr_arb_tree.io.req_i   := rd_req_masked
  io.rd_ack_o := i_rr_arb_tree.io.gnt_o
  i_rr_arb_tree.io.data_i  := wire_zero
  i_rr_arb_tree.io.gnt_i   := ~io.wr_cl_vld_i
  rd_req := i_rr_arb_tree.io.req_o
  // i_rr_arb_tree.io.data_o  :=
  vld_sel_d := i_rr_arb_tree.io.idx_o

  rd_acked := rd_req & ~io.wr_cl_vld_i;

  vld_we      := io.wr_cl_vld_i
  bank_req    := 0.U
  io.wr_ack_o := 0.U
  bank_we     := 0.U
  for(i <- 0 until DCACHE_NUM_BANKS) {
    bank_idx(i) := io.wr_idx_i
  }

  for(k <- 0 until NumPorts) {
    bank_collision(k) := (io.rd_off_i(k)(DCACHE_OFFSET_WIDTH-1,3) === io.wr_off_i(DCACHE_OFFSET_WIDTH-1,3))
  }

  when (io.wr_cl_vld_i && io.wr_cl_we_i.orR) {
    bank_req := 1.U
    bank_we  := 1.U
    for(i <- 0 until DCACHE_NUM_BANKS) { bank_idx(i) := io.wr_cl_idx_i }
  } .otherwise {
    when (rd_acked) {
      when(!io.rd_tag_only_i(vld_sel_d)) {
        bank_req                                                   := dcache_cl_bin2oh(io.rd_off_i(vld_sel_d)(DCACHE_OFFSET_WIDTH-1,3));
        bank_idx(io.rd_off_i(vld_sel_d)(DCACHE_OFFSET_WIDTH-1, 3)) := io.rd_idx_i(vld_sel_d)
      }
    }
    when (io.wr_req_i.asUInt.orR) {
      when (io.rd_tag_only_i(vld_sel_d) || !(io.rd_ack_o(vld_sel_d) && bank_collision(vld_sel_d))) {
        io.wr_ack_o := true.B
        // bank_req |= dcache_cl_bin2oh(io.wr_off_i(DCACHE_OFFSET_WIDTH-1,3));
        bank_req := dcache_cl_bin2oh(io.wr_off_i(DCACHE_OFFSET_WIDTH-1,3));
        bank_we  := dcache_cl_bin2oh(io.wr_off_i(DCACHE_OFFSET_WIDTH-1,3));
      }
    }
  }

  ///////////////////////////////////////////////////////
  // tag comparison, hit generatio, readoud muxes
  ///////////////////////////////////////////////////////

  val wr_cl_off       = Wire(UInt(DCACHE_OFFSET_WIDTH.W))
  val wbuffer_hit_idx = Wire(UInt(log2Ceil(DCACHE_WBUF_DEPTH).W))
  val rd_hit_idx      = Wire(UInt(log2Ceil(DCACHE_SET_ASSOC).W))

  cmp_en_d := vld_req.orR & ~vld_we

  // word tag comparison in write buffer
  wbuffer_cmp_addr := Mux(io.wr_cl_vld_i, Cat(io.wr_cl_tag_i, io.wr_cl_idx_i, io.wr_cl_off_i),
                                          Cat(rd_tag, bank_idx_q, bank_off_q))
  // hit generation
  for (i <- 0 until DCACHE_SET_ASSOC) { // gen_tag_cmpsel
                                        // tag comparison of ways >0
    io.rd_hit_oh_o(i) := (rd_tag === tag_rdata(i)) & io.rd_vld_bits_o(i)  & cmp_en_q
    // byte offset mux of ways >0
    rdata_cl(i) := bank_rdata(bank_off_q(DCACHE_OFFSET_WIDTH-1,3))(i)
  }

  for(k <- 0 until DCACHE_WBUF_DEPTH) { // gen_wbuffer_hit
    wbuffer_hit_oh(k) := (io.wbuffer_data_i(k).valid.asUInt.orR) & (io.wbuffer_data_i(k).wtag === (wbuffer_cmp_addr >> 3))
  }

  val i_lzc_wbuffer_hit = Module(new lzc(DCACHE_WBUF_DEPTH))
  i_lzc_wbuffer_hit.io.in_i    := wbuffer_hit_oh
  wbuffer_hit_idx := i_lzc_wbuffer_hit.io.cnt_o
  // i_lzc_wbuffer_hit.io.empty_o :=

  val i_lzc_rd_hit = Module(new lzc(DCACHE_SET_ASSOC))
  i_lzc_rd_hit.io.in_i := io.rd_hit_oh_o
  rd_hit_idx := i_lzc_rd_hit.io.cnt_o
  // i_lzc_rd_hit.io.empty_o :=

  wbuffer_rdata := io.wbuffer_data_i(wbuffer_hit_idx).data
  wbuffer_be    := Mux(wbuffer_hit_oh.asUInt.orR, io.wbuffer_data_i(wbuffer_hit_idx).valid, VecInit(Seq.fill(8)(false.B)))

  if (Axi64BitCompliant) { // gen_axi_off
    wr_cl_off := Mux(io.wr_cl_nc_i, 0.U, io.wr_cl_off_i(DCACHE_OFFSET_WIDTH-1,3))
  } else { // : gen_piton_off
    wr_cl_off := io.wr_cl_off_i(DCACHE_OFFSET_WIDTH-1,3)
  }

  val wr_cl_data_w = Wire(Vec(DCACHE_LINE_WIDTH / 64, UInt(64.W)))
  for (i <- 0 until DCACHE_LINE_WIDTH / 64) {
    wr_cl_data_w(i) := io.wr_cl_data_i(i * 64 + 63, i * 64)
  }
  rdata := Mux(io.wr_cl_vld_i, wr_cl_data_w(wr_cl_off), rdata_cl(rd_hit_idx))

  // overlay bytes that hit in the write buffer
  val rd_data_w = Wire(Vec(8, UInt(8.W)))
  for(k <- 0 until 8) { // gen_rd_data
    rd_data_w(k) := Mux(wbuffer_be(k), wbuffer_rdata(k), rdata(8*k+7, 8*k))
  }
  io.rd_data_o := rd_data_w.asUInt

  ///////////////////////////////////////////////////////
  // memory arrays and regs
  ///////////////////////////////////////////////////////

  val vld_tag_rdata = Wire(Vec(DCACHE_SET_ASSOC, UInt((DCACHE_TAG_WIDTH+1).W)))

  for (k <- 0 until DCACHE_NUM_BANKS) { // : gen_data_banks
    // Data RAM
    val i_data_sram = Module(new sram(DCACHE_SET_ASSOC * 64, DCACHE_NUM_WORDS))
    i_data_sram.io.req_i   := bank_req   (k)
    i_data_sram.io.we_i    := bank_we    (k)
    i_data_sram.io.addr_i  := bank_idx   (k)
    i_data_sram.io.wdata_i := bank_wdata (k).asUInt
    i_data_sram.io.be_i    := bank_be    (k).asUInt
    val bank_rdata_w = Wire(UInt((DCACHE_SET_ASSOC * 64).W))
    bank_rdata_w := i_data_sram.io.rdata_o
    for(i <- 0 until DCACHE_SET_ASSOC) { bank_rdata(k)(i) := bank_rdata_w(64*i+63, 64*i) }
  }

  for (i <- 0 until DCACHE_SET_ASSOC) { // : gen_tag_srams

    tag_rdata(i)        := vld_tag_rdata(i)(DCACHE_TAG_WIDTH-1, 0)
    io.rd_vld_bits_o(i) := vld_tag_rdata(i)(DCACHE_TAG_WIDTH)

    // Tag RAM
    val i_tag_sram = Module(new sram (DCACHE_TAG_WIDTH + 1, DCACHE_NUM_WORDS ))
    i_tag_sram.io.req_i   :=  vld_req(i)
    i_tag_sram.io.we_i    :=  vld_we
    i_tag_sram.io.addr_i  :=  vld_addr
    i_tag_sram.io.wdata_i :=  Cat(vld_wdata(i), io.wr_cl_tag_i)
    i_tag_sram.io.be_i    :=  true.B
    vld_tag_rdata(i) := i_tag_sram.io.rdata_o
  }

  bank_idx_q := bank_idx_d
  bank_off_q := bank_off_d
  vld_sel_q  := vld_sel_d
  cmp_en_q   := cmp_en_d

  ///////////////////////////////////////////////////////
  // assertions
  ///////////////////////////////////////////////////////
}


object wt_dcache_mem extends App {
  chisel3.Driver.execute(args, () => new wt_dcache_mem(false, 3))
}
