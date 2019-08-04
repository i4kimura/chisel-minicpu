package ooo

import chisel3._
import chisel3.util._
import chisel3.Bool

import DecodeConsts._

class LsuRdReqIo [Conf <: RVConfig](conf: Conf) extends Bundle {
  override def cloneType: this.type =
    new LsuRdReqIo(conf).asInstanceOf[this.type]

  val addr   = Output(UInt(conf.bus_width.W))
  val size   = Output(UInt(3.W))
}

class LsuWrReqIo [Conf <: RVConfig](conf: Conf) extends Bundle {
  override def cloneType: this.type =
    new LsuWrReqIo(conf).asInstanceOf[this.type]

  val addr   = Output(UInt(conf.bus_width.W))
  val size   = Output(UInt(3.W))
  val wrdata = Output(SInt(conf.xlen.W))
}


class LsuRdRespIo [Conf <: RVConfig](conf: Conf) extends Bundle {
  override def cloneType: this.type =
    new LsuRdRespIo(conf).asInstanceOf[this.type]

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

    val lsu_rd_req  = Decoupled(new LsuRdReqIo(conf))
    val lsu_rd_resp = Flipped(Decoupled(new LsuRdRespIo(conf)))
    val lsu_wr_req  = Decoupled(new LsuWrReqIo(conf))

    val lsu_read = Valid(new LsuReadIo(conf))
  })

  val mem_ctrl_mem_v    = RegNext (io.ctrl.mem_v)
  val mem_ctrl_mem_cmd  = RegNext (io.ctrl.mem_cmd)
  val mem_ctrl_mem_type = RegNext (io.ctrl.mem_type)

  val mem_rdata_op0     = RegNext (io.op0)
  val mem_rdata_op1     = RegNext (io.op1)

  io.lsu_rd_req.valid       := mem_ctrl_mem_v && (mem_ctrl_mem_cmd === MCMD_RD)
  io.lsu_rd_req.bits.size   := mem_ctrl_mem_type
  io.lsu_rd_req.bits.addr   := mem_rdata_op0.asUInt

  io.lsu_rd_resp.ready := true.B

  io.lsu_wr_req.valid       := mem_ctrl_mem_v && (mem_ctrl_mem_cmd === MCMD_WR)
  io.lsu_wr_req.bits.size   := mem_ctrl_mem_type
  io.lsu_wr_req.bits.addr   := mem_rdata_op0.asUInt
  io.lsu_wr_req.bits.wrdata := mem_rdata_op1

  io.lsu_read.valid       := io.lsu_rd_resp.fire()
  io.lsu_read.bits.rddata := io.lsu_rd_resp.bits.rddata
}
