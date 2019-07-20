package ooo

import chisel3._
import chisel3.util._
import chisel3.Bool

class LsuReqIo [Conf <: RVConfig](conf: Conf) extends Bundle {
  override def cloneType: this.type =
    new LsuReqIo(conf).asInstanceOf[this.type]

  val cmd    = Output(UInt(2.W))
  val addr   = Output(UInt(conf.bus_width.W))
  val size   = Output(UInt(3.W))
  val wrdata = Output(SInt(conf.xlen.W))
}

class LsuRespIo [Conf <: RVConfig](conf: Conf) extends Bundle {
  override def cloneType: this.type =
    new LsuRespIo(conf).asInstanceOf[this.type]

  val rddata = Output(SInt(conf.xlen.W))
}


class LsuReadIo [Conf <: RVConfig](conf: Conf) extends Bundle {
  override def cloneType: this.type =
    new LsuReadIo(conf).asInstanceOf[this.type]

  val rddata = Output(SInt(conf.xlen.W))
}


class Lsu [Conf <: RVConfig](conf: Conf) extends Module {
  val io = IO (new Bundle {
    val ctrl = Input (new CtrlSignals())

    val op0 = Input(SInt(conf.xlen.W))
    val op1 = Input(SInt(conf.xlen.W))

    val lsu_req = Decoupled(new LsuReqIo(conf))
    val lsu_resp = Flipped(Decoupled(new LsuRespIo(conf)))

    val lsu_read = Valid(new LsuReadIo(conf))
  })

  val mem_ctrl_mem_v    = RegNext (io.ctrl.mem_v)
  val mem_ctrl_mem_cmd  = RegNext (io.ctrl.mem_cmd)
  val mem_ctrl_mem_type = RegNext (io.ctrl.mem_type)

  val mem_rdata_op0     = RegNext (io.op0)
  val mem_rdata_op1     = RegNext (io.op1)

  io.lsu_req.valid       := mem_ctrl_mem_v
  io.lsu_req.bits.cmd    := mem_ctrl_mem_cmd
  io.lsu_req.bits.size   := mem_ctrl_mem_type
  io.lsu_req.bits.addr   := mem_rdata_op0.asUInt
  io.lsu_req.bits.wrdata := mem_rdata_op1

  io.lsu_resp.ready := true.B

  io.lsu_read.valid       := io.lsu_resp.fire()
  io.lsu_read.bits.rddata := io.lsu_resp.bits.rddata
}
