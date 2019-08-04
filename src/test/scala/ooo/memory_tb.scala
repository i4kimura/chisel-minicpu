package ooo

import chisel3._
import chisel3.util._
import chisel3.Bool

import BusConsts._
import DecodeConsts._

class MemoryIo [Conf <: RVConfig](conf: Conf) extends Bundle {
  override def cloneType: this.type =
    new MemoryIo(conf).asInstanceOf[this.type]

  val l1i_rd_req  = Flipped(Decoupled(new FrontEndReqIo(conf)))
  val l1i_rd_resp = Decoupled(new FrontEndRespIo(conf))

  val l1d_rd_req  = Flipped(Decoupled(new LsuRdReqIo(conf)))
  val l1d_rd_resp = Decoupled(new LsuRdRespIo(conf))
  val l1d_wr_req  = Flipped(Decoupled(new LsuWrReqIo (conf)))
}

class CpuMemory [Conf <: RVConfig](conf: Conf) extends Module {
  val io = IO (new Bundle {
    val mem = new MemoryIo(conf)
    val ext = Flipped(new ExtBus(conf))
  })

  val memory = conf.debug match {
    case true => {
      val mem_model = Module(new MemoryModel(conf))
      mem_model
    }
    // case _ => {
    //   val mem_model = Module(new MemoryResourceBox(conf))
    //   mem_model
    // }
  }

  memory.io.mem <> io.mem
  memory.io.ext <> io.ext
}

class MemoryModel [Conf <: RVConfig](conf: Conf) extends Module {
  val io = IO (new Bundle {
    val mem = new MemoryIo(conf)
    val ext = Flipped(new ExtBus(conf))
  })

  val memory_byte_w = conf.memory_byte_w
  val bank_w:Int    = (scala.math.log10(conf.memory_byte_w) / scala.math.log10(2)).asInstanceOf[Int]

  val inst_rd_data = Wire(Vec(memory_byte_w, UInt(8.W)))
  val data_rd_data = Wire(Vec(memory_byte_w, UInt(8.W)))
  val ext_rd_data  = Wire(Vec(memory_byte_w, UInt(8.W)))

  when (io.ext.req) {
    printf("Load Memory : %x <= %x\n", Cat(io.ext.addr(conf.bus_width-1, 0), 0.U(2.W)), io.ext.wrdata.asUInt)
  }

  for (bank <- 0 until memory_byte_w) {
    val data_msb = (bank & 0x3)*8+7
    val data_lsb = (bank & 0x3)*8+0

    val memory = SyncReadMem(math.pow(2, conf.bus_width + 2).toInt , UInt(8.W))

    val bank_idx = bank.U(bank_w-1, 0)

    val ext_bank_addr = io.ext.addr(conf.bus_width-1, 3)
    val ext_bank_data = io.ext.wrdata(data_msb, data_lsb)

    val mem_inst_rd_addr = io.mem.l1i_rd_req.bits.addr(conf.bus_width-1, 3)
    val mem_data_rd_addr = io.mem.l1d_rd_req.bits.addr(conf.bus_width-1, 3)

    when (io.ext.req) {
      when(io.ext.addr(bank_w-1, 2) === bank_idx(bank_w-1, 2)) {
        memory.write(ext_bank_addr, ext_bank_data)
      }
      /* Ext Bus */
    }
    ext_rd_data(bank) := memory.read(ext_bank_addr)

    /* Inst Bus */
    inst_rd_data(bank) := memory.read(mem_inst_rd_addr)

    /* Data Bus */
    data_rd_data(bank) := memory(mem_data_rd_addr)
    // when (io.mem.l1d_rd_req.req === true.B && io.mem.l1d_rd_req.cmd === MCMD_RD) {
    //   printf("Load Memory : %x <= %x\n", io.mem.l1d_rd_req.bits.addr(conf.bus_width-1, 0), data_rd_data(bank))
    // }

    val mem_data_wr_addr = io.mem.l1d_wr_req.bits.addr(conf.bus_width-1, 3)
    val mem_data_wr_data = io.mem.l1d_wr_req.bits.wrdata(data_msb, data_lsb)

    when(io.mem.l1d_wr_req.fire()) {
      switch (io.mem.l1d_wr_req.bits.size) {
        is (MT_D) {
          memory.write(mem_data_wr_addr, mem_data_wr_data)
        }
        is (MT_W) {
          when (io.mem.l1d_wr_req.bits.addr(2) === bank_idx(2)) {
            memory.write(mem_data_wr_addr, io.mem.l1d_wr_req.bits.wrdata((bank & 0x3)*8+7, (bank & 0x3)*8+0))
          }
        }
        is (MT_H) {
          when (io.mem.l1d_wr_req.bits.addr(2,1) === bank_idx(2,1)) {
            memory.write(mem_data_wr_addr, io.mem.l1d_wr_req.bits.wrdata((bank & 0x1)*8+7, (bank & 0x1)*8+0))
          }
        }
        is (MT_B) {
          when (io.mem.l1d_wr_req.bits.addr(2,0) === bank_idx(2,0)) {
            memory.write(mem_data_wr_addr, io.mem.l1d_wr_req.bits.wrdata(7, 0))
          }
        }
      }
    }
  }

  val inst_bus_ack    = RegNext(io.mem.l1i_rd_req.valid)
  val inst_bus_rddata = Wire(SInt(64.W))

  io.mem.l1i_rd_req.ready := true.B

  io.mem.l1i_rd_resp.valid     := inst_bus_ack
  io.mem.l1i_rd_resp.bits.data := inst_bus_rddata.asUInt
  when(io.mem.l1i_rd_req.fire() & io.mem.l1i_rd_req.bits.addr(2) === 0.U(1.W)) {
    inst_bus_rddata := Cat(inst_rd_data(3), inst_rd_data(2), inst_rd_data(1), inst_rd_data(0)).asSInt
  } .otherwise {
    inst_bus_rddata := Cat(inst_rd_data(7), inst_rd_data(6), inst_rd_data(5), inst_rd_data(4)).asSInt
  }

  val data_bus_ack    = RegNext(io.mem.l1d_rd_req.fire())
  val data_bus_rddata = Reg(SInt(64.W))

  io.mem.l1d_rd_req.ready := true.B
  io.mem.l1d_wr_req.ready := true.B

  io.mem.l1d_rd_resp.valid       := data_bus_ack
  io.mem.l1d_rd_resp.bits.rddata := data_bus_rddata
  switch (io.mem.l1d_rd_req.bits.size) {
    is(MT_D) {
      data_bus_rddata := Cat(data_rd_data(7), data_rd_data(6), data_rd_data(5), data_rd_data(4),
                             data_rd_data(3), data_rd_data(2), data_rd_data(1), data_rd_data(0)).asSInt
    }
    is(MT_W) {
      data_bus_rddata := Cat(Fill(32, 0.U(1.W)), data_rd_data(io.mem.l1d_rd_req.bits.addr(2) * 4.U + 3.U),
                                                 data_rd_data(io.mem.l1d_rd_req.bits.addr(2) * 4.U + 2.U),
                                                 data_rd_data(io.mem.l1d_rd_req.bits.addr(2) * 4.U + 1.U),
                                                 data_rd_data(io.mem.l1d_rd_req.bits.addr(2) * 4.U + 0.U)).asSInt
    }
    is(MT_WU) {
      data_bus_rddata := Cat(Fill(32, 0.U(1.W)), data_rd_data(io.mem.l1d_rd_req.bits.addr(2) * 4.U + 3.U),
                                                 data_rd_data(io.mem.l1d_rd_req.bits.addr(2) * 4.U + 2.U),
                                                 data_rd_data(io.mem.l1d_rd_req.bits.addr(2) * 4.U + 1.U),
                                                 data_rd_data(io.mem.l1d_rd_req.bits.addr(2) * 4.U + 0.U)).asSInt
    }
    is(MT_HU) {
      val target_data = Cat(data_rd_data(io.mem.l1d_rd_req.bits.addr(2,1) * 2.U + 1.U), data_rd_data(io.mem.l1d_rd_req.bits.addr(2,1) * 2.U + 0.U))
      data_bus_rddata := Cat(0.U(48.W), target_data).asSInt
    }
    is(MT_H ) {
      val target_data = Cat(data_rd_data(io.mem.l1d_rd_req.bits.addr(2,1) * 2.U + 1.U), data_rd_data(io.mem.l1d_rd_req.bits.addr(2,1) * 2.U + 0.U))
      data_bus_rddata := Cat(Fill(48, target_data(15)), target_data).asSInt
    }
    is(MT_BU) { data_bus_rddata := Cat(Fill(56, 0.U), data_rd_data(io.mem.l1d_rd_req.bits.addr(2, 0))).asSInt }
    is(MT_B ) {
      val target_data = data_rd_data(io.mem.l1d_rd_req.bits.addr(2,0))
      data_bus_rddata := Cat(Fill(56, target_data(7)), target_data).asSInt
    }
  }

  val ext_bus_rddata = Wire(SInt(32.W))
  ext_bus_rddata := Cat (ext_rd_data(io.mem.l1d_rd_req.bits.addr(2) * 4.U + 3.U),
                         ext_rd_data(io.mem.l1d_rd_req.bits.addr(2) * 4.U + 2.U),
                         ext_rd_data(io.mem.l1d_rd_req.bits.addr(2) * 4.U + 1.U),
                         ext_rd_data(io.mem.l1d_rd_req.bits.addr(2) * 4.U + 0.U)).asSInt
  io.ext.rddata := ext_bus_rddata
}
