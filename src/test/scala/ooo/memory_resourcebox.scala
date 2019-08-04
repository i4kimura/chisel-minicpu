package ooo

import chisel3._
import chisel3.util._
import chisel3.Bool


class MemoryResourceBox [Conf <: RVConfig](conf: Conf) extends Module {
  val io = IO(new Bundle {
    val mem = new MemoryIo(conf)
    val ext = Flipped(new ExtBus(conf))
  })

  val mem_resource_box_core = Module(new MemoryResourceBoxCore(conf))

  mem_resource_box_core.io.clock <> clock
  mem_resource_box_core.io.mem <> io.mem
  mem_resource_box_core.io.ext <> io.ext
}


class MemoryResourceBoxCore [Conf <: RVConfig](conf: Conf) extends BlackBox with HasBlackBoxResource {
  val io = IO(new Bundle {
    val clock = Input(Clock())
    val mem = new MemoryIo(conf)
    val ext = Flipped(new ExtBus(conf))
  })

  setResource ("/memory_real.v")
}
