package cpu

import chisel3._
import chisel3.util._

class MemoryResourceBox [Conf <: RVConfig](conf: Conf) extends BlackBox with HasBlackBoxResource {
  val io = IO(new Bundle {
    val clock = Input(Clock())
    val mem = new MemoryIo(conf)
  })

  setResource ("/memory_real.v")
}
