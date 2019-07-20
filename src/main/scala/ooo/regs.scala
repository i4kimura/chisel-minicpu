package ooo

import chisel3._
import chisel3.util._
import chisel3.Bool

import DecodeConsts._

class RegReadIo extends Bundle {
  val en   = Input  (Vec(2, Bool()))
  val addr = Input  (Vec(2, UInt(5.W)))
  val data = Output (Vec(2, SInt(64.W)))
}

class RegWriteIo extends Bundle {
  val en   = Input  (Bool())
  val addr = Input  (UInt(5.W))
  val data = Input  (SInt(64.W))
}


class RegIo [Conf <: RVConfig](conf: Conf) extends Bundle {
  val read  = Vec(conf.fetch_width, new RegReadIo)
  val write = new RegWriteIo
}


class Regs [Conf <: RVConfig](conf: Conf) extends Module {
  val io = IO (new RegIo(conf))

  val r_regs = SyncReadMem(32, SInt(conf.xlen.W))

  for (w <- 0 until conf.fetch_width) {
    for (port <- 0 until 2) {
      io.read(w).data(port) := 0.S(conf.xlen.W)
      when (io.read(w).en(port) && (io.read(w).addr(port) =/= 0.U)) {
        io.read(w).data(port) := Mux (io.write.en & (io.write.addr === io.read(w).addr(port)),
                                      io.write.data,
                                      r_regs(io.read(w).addr(port)))
      }
    }
  }

  when (io.write.en && (io.write.addr =/= 0.U)) {
    r_regs(io.write.addr) := io.write.data
  }
}
