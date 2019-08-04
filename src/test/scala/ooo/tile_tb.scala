package ooo

import chisel3._
import chisel3.util._
import chisel3.Bool


class ExtBus [Conf <: RVConfig](conf: Conf) extends Bundle {
  override def cloneType: this.type =
    new ExtBus(conf).asInstanceOf[this.type]

  val req    = Output(Bool())
  val addr   = Output(UInt(conf.bus_width.W))
  val wrdata = Output(SInt(32.W))
  val rddata = Input(SInt(32.W))
}


class CpuTopIo [Conf <: RVConfig](conf: Conf) extends Bundle {
  override def cloneType: this.type =
    new CpuTopIo(conf).asInstanceOf[this.type]

  val ext_bus = Flipped(new ExtBus(conf))
}


class OooTileTb [Conf <: RVConfig](conf: Conf) extends Module {
  val io = IO (new CpuTopIo(conf))

  val memory = Module(new CpuMemory(conf))
  val tile   = Module(new OooTile(conf))

  // Connect TILE and Memory
  memory.io.mem <> tile.io
  // Memory Load for External Debug
  memory.io.ext <> io.ext_bus

  // io.dbg_monitor <> tile.io.dbg_monitor
}
