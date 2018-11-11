package cpu

import chisel3._

class MemoryIo extends Bundle {
  val wen    = Input(Bool())
  val wraddr = Input(UInt(8.W))
  val wrdata = Input(UInt(32.W))

  val ren    = Input(Bool())
  val rdaddr = Input(UInt(8.W))
  val rddata = Output(UInt(32.W))
  val rden   = Output(Bool())

  // External Memory Input
  val extwen  = Input(Bool())
  val extaddr = Input(UInt(8.W))
  val extdata = Input(UInt(32.W))
}

class Memory extends Module {
  val io = IO(new MemoryIo)

  val memory = Mem(256, UInt(32.W))
  val r_en   = Reg(Bool())

  when (io.extwen) {
    memory(io.extaddr) := io.extdata
    printf(s"<Info : Address %x : Write %x>\n", io.extaddr, io.extdata)
  } .otherwise {
    when (io.wen) {
      memory(io.wraddr) := io.wrdata
    }
  }

  r_en := io.ren

  io.rddata := memory(io.rdaddr)
  io.rden  := r_en

  // External Data Input
}
