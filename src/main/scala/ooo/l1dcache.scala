package ooo

import chisel3._
import chisel3.util._
import chisel3.Bool

trait L1DCacheParams {
  def nSets:         Int
  def nWays:         Int
  def rowBits:       Int
  def nTLBEntries:   Int
  def blockBytes:    Int // TODO this is ignored in favor of p(CacheBlockBytes) in BaseTile
}


class L1DCacheBundle [Conf <: RVConfig](conf: Conf) extends Bundle {
  /* CPU Request */
  val cpu_rd_req  = Flipped(Decoupled(new LsuRdReqIo(conf)))
  val cpu_rd_resp = Decoupled(new LsuRdRespIo (conf))
  val cpu_wr_req  = Flipped(Decoupled(new LsuWrReqIo (conf)))

  /* L1DCache External Request */
  val ext_rd_req  = Decoupled(new LsuRdReqIo(conf))
  val ext_rd_resp = Flipped(Decoupled(new LsuRdRespIo (conf)))
  val ext_wr_req  = Decoupled(new LsuWrReqIo (conf))
}


class L1DCache [Conf <: RVConfig](conf: Conf) extends Module
{
  val io = IO(new L1DCacheBundle(conf))

  val TagLsb = 10
  val TagBit = conf.bus_width - TagLsb

  val u_l1dc_fsm  = Module(new l1dc_fsm(conf, TagLsb, TagBit))
  val u_l1dc_tag  = Module(new l1dc_tag(conf, TagLsb, TagBit))
  val u_l1dc_data = Module(new l1dc_data(conf, TagLsb))

  //** L1-State Machine **
  u_l1dc_fsm.io.cpu_rd_req  <> io.cpu_rd_req
  u_l1dc_fsm.io.cpu_rd_resp <> io.cpu_rd_resp
  u_l1dc_fsm.io.cpu_wr_req  <> io.cpu_wr_req

  u_l1dc_fsm.io.ext_rd_req  <> io.ext_rd_req
  u_l1dc_fsm.io.ext_rd_resp <> io.ext_rd_resp
  u_l1dc_fsm.io.ext_wr_req  <> io.ext_wr_req

  u_l1dc_fsm.io.f1_tag_tag        := u_l1dc_tag.io.read_tag    // read port
  u_l1dc_fsm.io.f1_tag_read_valid := u_l1dc_tag.io.read_valid  // valid bit
  u_l1dc_fsm.io.f1_tag_read_dirty := u_l1dc_tag.io.read_dirty  // dirty bit

  //** L1-Data **
  u_l1dc_data.io.rd_idx  := 0.U
  // u_l1dc_data.io.rd_data

  u_l1dc_data.io.wr_we   := false.B
  u_l1dc_data.io.wr_idx  := 0.U  // write index
  u_l1dc_data.io.wr_data := 0.U  //write port (128-bit line)

  //** L1-Tag **
  u_l1dc_tag.io.req_index := 0.U    // 10-bit index

  u_l1dc_tag.io.update_index := 0.U // 10-bit index
  u_l1dc_tag.io.update_we    := u_l1dc_fsm.io.data_update_we  // write enable

  u_l1dc_tag.io.write_tag   := 0.U       // write port
  u_l1dc_tag.io.write_valid := true.B    // write enable
  u_l1dc_tag.io.write_dirty := false.B   // dirty bit

}

//
// L1DCache State Machine
//
class l1dc_fsm [Conf <: RVConfig](conf: Conf, TagLsb: Int, TagBit: Int) extends Module
{
  val io = IO(new Bundle {
    /* L1ICache Request */
    val cpu_rd_req  = Flipped(Decoupled(new LsuRdReqIo(conf)))
    val cpu_rd_resp = Decoupled(new LsuRdRespIo (conf))
    val cpu_wr_req  = Flipped(Decoupled(new LsuWrReqIo (conf)))

    /* L1ICache Request */
    val ext_rd_req  = Decoupled(new LsuRdReqIo(conf))
    val ext_rd_resp = Flipped(Decoupled(new LsuRdRespIo (conf)))
    val ext_wr_req  = Decoupled(new LsuWrReqIo (conf))

    val f1_tag_tag        = Input(UInt(TagBit.W))
    val f1_tag_read_valid = Input(Bool())
    val f1_tag_read_dirty = Input(Bool())

    val data_update_we    = Output(Bool())          // write enable
    val data_update_index = Output(UInt(TagLsb.W))  // write tag index
    val data_update_data  = Output(UInt(128.W))     // write data
  })

  val (compareTag ::
       allocate   ::
       waitExtReq ::
       writeBack  :: Nil) = Enum(4)


  val state = RegInit(compareTag)

  val f1_cpu_rd_req_valid = RegNext (io.cpu_rd_req.fire())
  val f1_cpu_wr_req_valid = RegNext (io.cpu_wr_req.fire())

  val data_req_count = Reg(UInt(2.W))

  io.cpu_rd_req.ready := true.B

  io.cpu_rd_resp.valid := false.B
  io.cpu_rd_resp.bits.rddata := 0.S

  io.cpu_wr_req.ready := true.B

  val ext_rd_resp_ready = Wire(Bool())
  io.ext_rd_resp.ready := RegNext(ext_rd_resp_ready)

  io.ext_wr_req.valid := false.B
  io.ext_wr_req.bits.addr   := 0.U
  io.ext_wr_req.bits.wrdata := 0.S
  io.ext_wr_req.bits.size   := 0.U

  val f1_cpu_rd_req_addr = RegNext (io.cpu_rd_req.bits.addr)

  io.data_update_we    := false.B
  io.data_update_index := f1_cpu_rd_req_addr(TagLsb-1, 0)
  io.data_update_data  := io.ext_rd_resp.bits.rddata.asUInt

  io.ext_rd_req.valid     := false.B
  io.ext_rd_req.bits.addr := 0.U
  io.ext_rd_req.bits.size := 0.U

  ext_rd_resp_ready := false.B
  switch (state) {
    is(compareTag) {
      when (f1_cpu_rd_req_valid || f1_cpu_wr_req_valid) {
        when (io.f1_tag_tag =/= f1_cpu_rd_req_addr(TagBit+TagLsb-1, TagLsb)) {
          /*compulsory miss or miss with clean block*/
          when (io.f1_tag_read_valid || !io.f1_tag_read_dirty) {
            /*wait till a new block is allocated*/
            state := waitExtReq
          } .otherwise {
            /*wait till write is completed*/
            state := writeBack
          }
        }
      }
    }
    is(waitExtReq) {
      when (io.ext_rd_req.fire()) {
        state := allocate
      }
    }
    is(writeBack) {
      /*write back is completed*/
      when (io.ext_wr_req.fire()) {
        /*issue new memory request (allocating a new line)*/
        io.ext_rd_req.valid := true.B
        state   := allocate
      }
    }
    is(allocate) {
      /*memory controller has responded*/
      when (io.ext_rd_resp.fire()) {
        when (data_req_count === 4.U) {
          /*re-compare tag for write miss (need modify correct word)*/
          state  := compareTag
          data_req_count := 0.U
          ext_rd_resp_ready := true.B
        } .otherwise {
          data_req_count := data_req_count + 1.U
        }
        /*update cache line data*/
        io.data_update_we := true.B
      }
    }
  }
}


class l1dc_data [Conf <: RVConfig](conf: Conf, TagLsb: Int) extends Module
{
  val io = IO(new Bundle {
    val rd_idx  = Input(UInt(TagLsb.W))
    val rd_data = Output(UInt(128.W))

    val wr_we   = Input(Bool())
    val wr_idx  = Input(UInt(TagLsb.W)) // write index
    val wr_data = Input (UInt(128.W))  //write port (128-bit line)
  })

  val mem = SyncReadMem(Math.pow(2, TagLsb).toInt, UInt(128.W))
  when (io.wr_we) {
    mem.write(io.wr_idx, io.wr_data)
  }

  io.rd_data := mem.read(io.rd_idx)

}

class l1dc_tag [Conf <: RVConfig](conf: Conf, TagLsb: Int, TagBit: Int) extends Module
{
  val io = IO(new Bundle {
    val req_index = Input(UInt(TagLsb.W))    // 10-bit index

    val update_index = Input(UInt(TagLsb.W)) // 10-bit index
    val update_we    = Input(Bool())         // write enable

    val write_tag   = Input(UInt(TagBit.W))  // write port
    val write_valid = Input(Bool())          // valid bit
    val write_dirty = Input(Bool())          // dirty bit

    val read_tag   = Output(UInt(TagBit.W))  // read port
    val read_valid = Output(Bool())          // valid bit
    val read_dirty = Output(Bool())          // dirty bit
  })

  val TableSize = Math.pow(2,TagLsb).toInt

  val mem_tag   = SyncReadMem(TableSize, UInt(TagBit.W))
  val mem_valid = RegInit(VecInit(Seq.fill(TableSize)(false.B)))
  val mem_dirty = Reg(Vec(TableSize, Bool()))

  when (io.update_we) {
    mem_tag.write(io.update_index, io.write_tag)
    mem_valid(io.update_index) := io.write_valid
    mem_dirty(io.update_index) := io.write_dirty
  }

  io.read_tag   := mem_tag.read(io.req_index)
  // io.read_valid := RegNext(mem_valid(io.req_index))
  io.read_valid := RegNext(mem_valid.apply(io.req_index))
  io.read_dirty := RegNext(mem_dirty.apply(io.req_index))

}


object L1DCache extends App {
  chisel3.Driver.execute(args, () => new L1DCache(new RV64IConfig))
}
