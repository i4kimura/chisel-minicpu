package cpu

import chisel3._
import chisel3.util._
import chisel3.Bool

import DecodeConsts._


class RegIo extends Bundle {
  val rden0   = Input  (Bool())
  val rdaddr0 = Input  (UInt(5.W))
  val rddata0 = Output (SInt(64.W))

  val rden1   = Input  (Bool())
  val rdaddr1 = Input  (UInt(5.W))
  val rddata1 = Output (SInt(64.W))

  val wren   = Input  (Bool())
  val wraddr = Input  (UInt(5.W))
  val wrdata = Input  (SInt(64.W))
}


class Regs [Conf <: RVConfig](conf: Conf) extends Module {
  val io = IO (new RegIo)

  // val r_regs = RegInit( Vec(32, SInt(conf.xlen.W)).asTypeOf(0.U) )
  val r_regs = Mem(32, SInt(conf.xlen.W))

  when (io.rden0 && (io.rdaddr0 =/= 0.U(64.W))) {
    io.rddata0 := r_regs(io.rdaddr0)
  } .otherwise {
    io.rddata0 := 0.S(64.W)
  }

  when (io.rden1 && (io.rdaddr1 =/= 0.U(64.W))) {
    io.rddata1 := r_regs(io.rdaddr1)
  } .otherwise {
    io.rddata1 := 0.S(64.W)
  }

  when (io.wren && (io.wraddr =/= 0.U(64.W))) {
    r_regs(io.wraddr) := io.wrdata
  }
}
