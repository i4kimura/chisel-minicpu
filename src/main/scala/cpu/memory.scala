package cpu

import chisel3._


class MemoryIo (bus_width: Int) extends Bundle {
  val wen    = Input(Bool())
  val wraddr = Input(UInt(bus_width.W))
  val wrdata = Input(UInt(32.W))

  val ren    = Input(Bool())
  val rdaddr = Input(UInt(bus_width.W))
  val rddata = Output(UInt(32.W))
  val rden   = Output(Bool())

  val ext_bus = new Bus(bus_width)
}

class Memory (bus_width: Int) extends Module {
  val io = IO(new MemoryIo(bus_width))

  val memory = Mem(math.pow(2, bus_width).toInt , UInt(32.W))
  val r_en   = Reg(Bool())

  when (io.ext_bus.req) {
    memory(io.ext_bus.addr) := io.ext_bus.data
    printf(p"<Info : Address 0x${Hexadecimal(io.ext_bus.addr)} : Write 0x${Hexadecimal(io.ext_bus.data)}>\n")
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
