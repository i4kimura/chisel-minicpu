package cpu

import chisel3._
import chisel3.util._
import chisel3.Bool

import BusConsts._

class MemoryIo (bus_width: Int) extends Bundle {
  val inst_bus = Flipped(new InstBus(bus_width))
  val data_bus = Flipped(new DataBus(bus_width))

  val ext_bus = new Bus(bus_width)
}

class Memory (bus_width: Int) extends Module {
  val io = IO(new MemoryIo(bus_width))

  val memory = Mem(math.pow(2, bus_width).toInt , UInt(64.W))

  when (io.ext_bus.req) {
    val tmp_val = memory(io.ext_bus.addr(bus_width-1, 1))
    when(io.ext_bus.addr(0) === 0.U) {
      memory(io.ext_bus.addr(bus_width-1, 1)) := Cat(tmp_val(63,32), io.ext_bus.data.asUInt)
    } .otherwise {
      memory(io.ext_bus.addr(bus_width-1, 1)) := Cat(io.ext_bus.data.asUInt, tmp_val(31, 0))
    }
    printf(p"<Info : Address 0x${Hexadecimal(io.ext_bus.addr)} : Write 0x${Hexadecimal(io.ext_bus.data)}>\n")
  }

  /* Inst Bus */
  val inst_rd_tmp_val = memory(io.inst_bus.addr(bus_width-1, 3))
  when(io.inst_bus.addr(2)) {
    io.inst_bus.rddata := inst_rd_tmp_val(63,32).asSInt
  } .otherwise {
    io.inst_bus.rddata := inst_rd_tmp_val(31, 0).asSInt
  }
  io.inst_bus.ack    := io.inst_bus.req

  /* Data Bus */
  val data_rd_tmp_val = memory(io.data_bus.addr(bus_width-1, 3))
  when(io.inst_bus.addr(2)) {
    io.data_bus.rddata := data_rd_tmp_val(63,32).asSInt
  } .otherwise {
    io.data_bus.rddata := data_rd_tmp_val(31, 0).asSInt
  }
  io.data_bus.ack    := io.data_bus.req & (io.data_bus.cmd === CMD_RD)

  when(io.data_bus.req & (io.data_bus.cmd === CMD_WR)) {
    memory(io.data_bus.addr(bus_width-1, 2)) := io.data_bus.wrdata.asUInt
  }

}
