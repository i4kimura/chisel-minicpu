package fir

import chisel3._
import chisel3.util._

class My4ElementFir(b0: Int, b1: Int, b2: Int, b3: Int) extends Module {
  val io = IO(new Bundle {
    val in = Input(UInt(8.W))
    val out = Output(UInt(8.W))
  })

  val r_x1 = RegNext(io.in, 0.U)
  val r_x2 = RegNext(r_x1, 0.U)
  val r_x3 = RegNext(r_x2, 0.U)

  io.out := io.in * b0.U(8.W) + r_x1 * b1.U(8.W) + r_x2 * b2.U(8.W) + r_x3 * b3.U(8.W);
}
