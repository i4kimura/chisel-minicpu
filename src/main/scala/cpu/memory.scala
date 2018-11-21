package cpu

import chisel3._

import BusConsts._

class MemoryIo (bus_width: Int) extends Bundle {
  val inst_bus = Flipped(new InstBus(bus_width))
  val data_bus = Flipped(new DataBus(bus_width))

  val ext_bus = new Bus(bus_width)
}

class Memory (bus_width: Int) extends Module {
  val io = IO(new MemoryIo(bus_width))

  val memory = Mem(math.pow(2, bus_width).toInt , UInt(32.W))

  when (io.ext_bus.req) {
    memory(io.ext_bus.addr) := io.ext_bus.data.asUInt
    printf(p"<Info : Address 0x${Hexadecimal(io.ext_bus.addr)} : Write 0x${Hexadecimal(io.ext_bus.data)}>\n")
  }

  /* Inst Bus */
  io.inst_bus.rddata := memory(io.inst_bus.addr(bus_width-1, 2)).asSInt
  io.inst_bus.ack    := io.inst_bus.req

  /* Data Bus */
  io.data_bus.rddata := memory(io.data_bus.addr(bus_width-1, 2)).asSInt
  io.data_bus.ack    := io.data_bus.req & (io.data_bus.cmd === CMD_RD)

  when(io.data_bus.req & (io.data_bus.cmd === CMD_WR)) {
    memory(io.data_bus.addr(bus_width-1, 2)) := io.data_bus.wrdata.asUInt
  }

}
