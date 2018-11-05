package test_firrtl

import chisel3._
import chisel3.util._
import firrtl._

class DelayBy2(width: Int) extends Module {
  val io = IO(new Bundle {
    val in  = Input(UInt(width.W))
    val out = Output(UInt(width.W))
  })
  val r0 = RegNext(io.in)
  val r1 = RegNext(r0)
  io.out := r1
}
