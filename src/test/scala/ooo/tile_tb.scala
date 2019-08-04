package ooo

import chisel3._
import chisel3.util._
import chisel3.Bool
import chisel3.experimental.{withClock, withReset, withClockAndReset}


class CpuTopIo [Conf <: RVConfig](conf: Conf) extends Bundle {
  override def cloneType: this.type =
    new CpuTopIo(conf).asInstanceOf[this.type]

  val ext_bus = Flipped(new ExtBus(conf))
  val cpu_reset = Input(Bool())
}


class OooTileTb [Conf <: RVConfig](conf: Conf) extends Module {
  val io = IO (new CpuTopIo(conf))

  val memory = Module(new CpuMemory(conf))
  val tile   = withReset(io.cpu_reset) {
    Module(new OooTile(conf))
  }

  // Connect TILE and Memory
  memory.io.mem <> tile.io
  // Memory Load for External Debug
  memory.io.ext <> io.ext_bus

  // io.dbg_monitor <> tile.io.dbg_monitor
}
