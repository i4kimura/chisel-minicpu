package cpu

import chisel3._
import chisel3.util._
import chisel3.Bool

import BusConsts._
import DecodeConsts._

class MemoryIo [Conf <: RVConfig](conf: Conf) extends Bundle {
  val inst_bus = Flipped(new InstBus(conf))
  val data_bus = Flipped(new DataBus(conf))

  val ext_bus = new Bus(conf)
}

class Memory [Conf <: RVConfig](conf: Conf) extends Module {
  val io = IO(new MemoryIo(conf))

  val inst_rd_data = Wire(Vec(8, UInt(8.W)))
  val data_rd_data = Wire(Vec(8, UInt(8.W)))

  for (bank <- 0 until 8) {

    val memory = Mem(math.pow(2, conf.bus_width).toInt , UInt(8.W))

    val bank_idx = bank.U(3.W)

    when (io.ext_bus.req) {
      val data_msb = (bank & 0x3)*8+7
      val data_lsb = (bank & 0x3)*8+0
      when(io.ext_bus.addr(0) === bank_idx(2)) {
        memory(io.ext_bus.addr(conf.bus_width-1, 1)) := io.ext_bus.data(data_msb, data_lsb)
      }
    }

    /* Inst Bus */
    inst_rd_data(bank) := memory(io.inst_bus.addr(conf.bus_width-1, 3))

    /* Data Bus */
    data_rd_data(bank) := memory(io.data_bus.addr(conf.bus_width-1, 3))

    val data_msb = bank * 8 + 7
    val data_lsb = bank * 8 + 0
    when(io.data_bus.req & (io.data_bus.cmd === CMD_WR)) {
      switch (io.data_bus.size) {
        is (MT_D) {
          memory(io.data_bus.addr(conf.bus_width-1, 3)) := io.data_bus.wrdata(data_msb, data_lsb)
        }
        is (MT_W) {
          when (io.data_bus.addr(2) === bank_idx(2)) {
            // printf("Write MT_W : %x <= %x\n", io.data_bus.addr(conf.bus_width-1, 3), io.data_bus.wrdata(data_msb, data_lsb))
            memory(io.data_bus.addr(conf.bus_width-1, 3)) := io.data_bus.wrdata((bank & 0x3)*8+7, (bank & 0x3)*8+0)
          }
        }
        is (MT_H) {
          when (io.data_bus.addr(2,1) === bank_idx(2,1)) {
            memory(io.data_bus.addr(conf.bus_width-1, 3)) := io.data_bus.wrdata((bank & 0x1)*8+7, (bank & 0x1)*8+0)
          }
        }
        is (MT_B) {
          when (io.data_bus.addr(2,0) === bank_idx(2,0)) {
            memory(io.data_bus.addr(conf.bus_width-1, 3)) := io.data_bus.wrdata(7, 0)
          }
        }
      }
    }
  }

  val inst_bus_ack    = RegNext(io.inst_bus.req)
  val inst_bus_rddata = Reg(SInt(64.W))

  io.inst_bus.ack    := inst_bus_ack
  io.inst_bus.rddata := inst_bus_rddata
  when(io.inst_bus.req & io.inst_bus.addr(2) === 0.U(1.W)) {
    inst_bus_rddata := Cat(inst_rd_data(3), inst_rd_data(2), inst_rd_data(1), inst_rd_data(0)).asSInt
  } .otherwise {
    // printf("PC = %x, data = %x\n", io.inst_bus.addr, Cat(inst_rd_data(7), inst_rd_data(6), inst_rd_data(5), inst_rd_data(4)))
    inst_bus_rddata := Cat(inst_rd_data(7), inst_rd_data(6), inst_rd_data(5), inst_rd_data(4)).asSInt
  }

  val data_bus_ack    = RegNext(io.data_bus.req & (io.data_bus.cmd === CMD_RD))
  val data_bus_rddata = Reg(SInt(64.W))

  io.data_bus.ack    := data_bus_ack
  io.data_bus.rddata := data_bus_rddata
  switch (io.data_bus.size) {
    is(MT_D) {
      data_bus_rddata := Cat(data_rd_data(7), data_rd_data(6), data_rd_data(5), data_rd_data(4),
                             data_rd_data(3), data_rd_data(2), data_rd_data(1), data_rd_data(0)).asSInt
    }
    is(MT_W) {
      data_bus_rddata := Cat(Fill(32, 0.U(1.W)), data_rd_data(io.data_bus.addr(2) * 4.U + 3.U),
                                                 data_rd_data(io.data_bus.addr(2) * 4.U + 2.U),
                                                 data_rd_data(io.data_bus.addr(2) * 4.U + 1.U),
                                                 data_rd_data(io.data_bus.addr(2) * 4.U + 0.U)).asSInt
    }
    is(MT_WU) {
      data_bus_rddata := Cat(Fill(32, 0.U(1.W)), data_rd_data(io.data_bus.addr(2) * 4.U + 3.U),
                                                 data_rd_data(io.data_bus.addr(2) * 4.U + 2.U),
                                                 data_rd_data(io.data_bus.addr(2) * 4.U + 1.U),
                                                 data_rd_data(io.data_bus.addr(2) * 4.U + 0.U)).asSInt
    }
    is(MT_HU) {
      val target_data = Cat(data_rd_data(io.data_bus.addr(2,1) * 2.U + 1.U), data_rd_data(io.data_bus.addr(2,1) * 2.U + 0.U))
      data_bus_rddata := Cat(0.U(48.W), target_data).asSInt
    }
    is(MT_H ) {
      val target_data = Cat(data_rd_data(io.data_bus.addr(2,1) * 2.U + 1.U), data_rd_data(io.data_bus.addr(2,1) * 2.U + 0.U))
      data_bus_rddata := Cat(Fill(48, target_data(15)), target_data).asSInt
    }
    is(MT_BU) { data_bus_rddata := Cat(Fill(56, 0.U), data_rd_data(io.data_bus.addr(2, 0))).asSInt }
    is(MT_B ) {
      val target_data = data_rd_data(io.data_bus.addr(2,0))
      data_bus_rddata := Cat(Fill(56, target_data(7)), target_data).asSInt
    }
  }

}
