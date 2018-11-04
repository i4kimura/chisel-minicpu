package chisel_type

import chisel3._
import chisel3.util._
import chisel3.experimental._
import dsptools.numbers._


class Bundle1 extends Bundle {
  val a = UInt(8.W)
  override def cloneType = (new Bundle1).asInstanceOf[this.type]
}

class Bundle2 extends Bundle1 {
  val b = UInt(16.W)
  override def cloneType = (new Bundle2).asInstanceOf[this.type]
}

class TypeConvertDemo extends Module {
  val io = IO(new Bundle {
    val in  = Input(UInt(4.W))
    val out = Output(SInt(4.W))
  })
  io.out := io.in.asTypeOf(io.out)
}


class ShiftRegisterIO[T <: Data](gen: T, n: Int) extends Bundle {
    require (n >= 0, "Shift register must have non-negative shift")

    val in = Input(gen.cloneType)
    val out = Output(Vec(n + 1, gen.cloneType)) // + 1 because in is included in out
}

class ShiftRegister[T <: Data](gen: T, n: Int) extends Module {
    val io = IO(new ShiftRegisterIO(gen, n))

    io.out.foldLeft(io.in) { case (in, out) =>
        out := in
        RegNext(in)
    }
}


object Mac {
  def apply[T <: Data : Ring](a: T, b: T, c: T): T = {
    a * b + c
  }
}


class Mac[T <: Data : Ring](genIn : T, genOut: T) extends Module {
  val io = IO(new Bundle {
    val a = Input(genIn.cloneType)
    val b = Input(genIn.cloneType)
    val c = Input(genIn.cloneType)
    val out = Output(genOut.cloneType)
  })
  io.out := io.a * io.b + io.c
}
