package registerfile

import chisel3._
import chisel3.util._

class RegisterFile(readPorts: Int) extends Module {
  require(readPorts >= 0)
  val io = IO(new Bundle {
    val wen   = Input(Bool())
    val waddr = Input(UInt(5.W))
    val wdata = Input(UInt(32.W))
    val raddr = Input(Vec(readPorts, UInt(5.W)))
    val rdata = Output(Vec(readPorts, UInt(32.W)))
  })

  // A Register of a vector of UInts
  // fromBits(0.U) is a bit of a hack to have reg reset to zero
  val reg = RegInit( Vec(32, UInt(32.W)).fromBits(0.U) )

  when (io.wen && io.waddr != 0.U) {
    reg(io.waddr) := io.wdata
  }

  for(idx <- 0 until readPorts) {
    io.rdata(idx) := reg(io.raddr(idx))
  }
}
