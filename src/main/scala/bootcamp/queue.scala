package queue

import chisel3._
import chisel3.util._

class Queue extends Module {
  // Example circuit using a Queue
  val io = IO(new Bundle {
    val in = Flipped(Decoupled(UInt(8.W)))
    val out = Decoupled(UInt(8.W))
  })
  val queue = Queue(io.in, 2)  // 2-element queue
  io.out <> queue
}
