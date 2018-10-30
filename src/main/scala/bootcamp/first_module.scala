package first_module

import chisel3._
import chisel3.util._

// Chisel Code: Declare a new module definition
class Passthrough extends Module {
  val io = IO(new Bundle {
    val in = Input(UInt(4.W))
    val out = Output(UInt(4.W))
  })
  io.out := io.in
}

// Chisel Code, but pass in a parameter to set widths of ports
class PassthroughGenerator(width: Int) extends Module {
  val io = IO(new Bundle {
    val in = Input(UInt(width.W))
    val out = Output(UInt(width.W))
  })
  io.out := io.in
}

// Let's now generate modules with different widths
// println(getVerilog(new PassthroughGenerator(10)))
// println(getVerilog(new PassthroughGenerator(20)))
