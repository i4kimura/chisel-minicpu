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

  val inst_rd_data = Wire(Vec(8, UInt(8.W)))
  val data_rd_data = Wire(Vec(8, UInt(8.W)))

  for (bank <- 0 until 8) {

    val memory = Mem(math.pow(2, bus_width).toInt , UInt(8.W))

    val bank_idx = bank.U(3.W)
    val data_msb = (bank & 0x3)*8+7
    val data_lsb = (bank & 0x3)*8+0

    when (io.ext_bus.req) {
      when(io.ext_bus.addr(0) === bank_idx(2)) {
        memory(io.ext_bus.addr(bus_width-1, 1)) := io.ext_bus.data(data_msb, data_lsb)
        printf(p"<Info : Bank ${Decimal(bank_idx)} : Address 0x${Hexadecimal(io.ext_bus.addr)} : Write 0x${Hexadecimal(io.ext_bus.data(data_msb, data_lsb))}>\n")
      }
    }

    /* Inst Bus */
    inst_rd_data(bank) := memory(io.inst_bus.addr(bus_width-1, 3))

    /* Data Bus */
    data_rd_data(bank) := memory(io.data_bus.addr(bus_width-1, 3))

    when(io.data_bus.req & (io.data_bus.cmd === CMD_WR) & io.data_bus.addr(2) === bank_idx(2)) {
      memory(io.data_bus.addr(bus_width-1, 3)) := io.data_bus.wrdata(data_msb, data_lsb)
    }
  }

  io.inst_bus.ack := io.inst_bus.req
  when(io.inst_bus.req & io.inst_bus.addr(2) === 0.U(1.W)) {
    io.inst_bus.rddata := Cat(inst_rd_data(3), inst_rd_data(2), inst_rd_data(1), inst_rd_data(0)).asSInt
  } .otherwise {
    io.inst_bus.rddata := Cat(inst_rd_data(7), inst_rd_data(6), inst_rd_data(5), inst_rd_data(4)).asSInt
  }

  io.data_bus.ack := io.data_bus.req & (io.data_bus.cmd === CMD_RD)
  when(io.data_bus.req & io.data_bus.addr(2) === 0.U(1.W)) {
    io.data_bus.rddata := Cat(data_rd_data(3), data_rd_data(2), data_rd_data(1), data_rd_data(0)).asSInt
  } .otherwise {
    io.data_bus.rddata := Cat(data_rd_data(7), data_rd_data(6), data_rd_data(5), data_rd_data(4)).asSInt
  }

}
