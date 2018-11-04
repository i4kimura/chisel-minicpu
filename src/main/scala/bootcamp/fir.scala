package fir

import chisel3._
import chisel3.util._
// get some math functions
import scala.math.{abs, round, cos, Pi, pow}

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


class GenericFIR (length: Int) extends Module {
  require(length >= 1)
  val io = IO (new Bundle {
    val consts = Input(Vec(length, UInt(8.W)))
    val in     = Input(UInt(8.W))
    val out    = Output(UInt(8.W))
  })

  val taps := RegInit(Vec(Seq.fill(length)(0.U(8.W))))
  taps.
  for (i <- 0 until length) {
    if (i == 0) taps(0) := io.in
    else        taps(i) := taps(i-1)
  }

  println("tap(0) = %d", taps(0))

  io.out := (taps zip io.consts).map { case (a, b) => a * b }.reduce(_ + _)

}


// our FIR has parameterized window length, IO bitwidth, and windowing function
class MyFir(length: Int, bitwidth: Int, window: (Int, Int) => Seq[Int]) extends Module {
  val io = IO(new Bundle {
    val in = Input(UInt(bitwidth.W))
    val out = Output(UInt((bitwidth*2+length-1).W)) // expect bit growth, conservative but lazy
  })

  // calculate the coefficients using the provided window function, mapping to UInts
  val coeffs = window(length, bitwidth).map(_.U)

  // create an array holding the output of the delays
  // note: we avoid using a Vec here since we don't need dynamic indexing
  val delays = Seq.fill(length)(Wire(UInt(bitwidth.W))).scan(io.in)( (prev: UInt, next: UInt) => {
    next := RegNext(prev)
    next
  })

  // multiply, putting result in "mults"
  val mults = delays.zip(coeffs).map{ case(delay: UInt, coeff: UInt) => delay * coeff }

  // add up multiplier outputs with bit growth
  val result = mults.reduce(_+&_)

  // connect output
  io.out := result
}
