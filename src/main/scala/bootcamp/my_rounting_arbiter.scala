package my_routing_arbiter

import chisel3._
import chisel3.util._

class MyRoutingArbiter(numChannels: Int) extends Module {
  val io = IO(new Bundle {
    val in = Vec(numChannels, Flipped(Decoupled(UInt(8.W))))
    val out = Decoupled(UInt(8.W))
  } )

  // YOUR CODE BELOW
  io.out.valid := io.in.map(_.valid).reduce(_ || _)
  val channel = PriorityMux( io.in.map(_.valid).zipWithIndex.map { case (valid, index) => (valid, index.U) } )

  io.out.bits := io.in(channel).bits
  for ((ready, index) <- io.in.map(_.ready).zipWithIndex) { ready := io.out.ready && channel === index.U }
}
