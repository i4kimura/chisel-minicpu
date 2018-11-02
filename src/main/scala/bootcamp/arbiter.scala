package arbiter2

import chisel3._
import chisel3.util._

class Arbiter2 extends Module {
  // Example circuit using a priority arbiter
  val io = IO(new Bundle {
    val in = Flipped(Vec(2, Decoupled(UInt(8.W))))
    val out = Decoupled(UInt(8.W))
  })
  // Arbiter doesn't have a convenience constructor, so it's built like any Module
  val arbiter = Module(new Arbiter(UInt(8.W), 2))  // 2 to 1 Priority Arbiter
  arbiter.io.in <> io.in
  io.out <> arbiter.io.out
}
