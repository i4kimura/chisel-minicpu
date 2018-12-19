package cpu

import chisel3._
import chisel3.util._
import chisel3.Bool

import DecodeConsts._


class CpuDebugMonitor [Conf <: RVConfig](conf: Conf) extends Bundle {
  val inst_fetch_req    = if (conf.debug == true) Output(Bool())                 else Output(UInt(0.W))
  val inst_fetch_addr   = if (conf.debug == true) Output(UInt(conf.bus_width.W)) else Output(UInt(0.W))
  val inst_fetch_ack    = if (conf.debug == true) Output(Bool())                 else Output(UInt(0.W))
  val inst_fetch_rddata = if (conf.debug == true) Output(SInt(32.W))             else Output(SInt(0.W))

  val pc_update_cause = if(conf.debug == true) Output(UInt(3.W)) else Output(UInt(0.W))

  val inst_valid = if (conf.debug == true) Output(Bool())                  else Output(UInt(0.W))
  val inst_addr  = if (conf.debug == true) Output(UInt(conf.bus_width.W))  else Output(UInt(0.W))
  val inst_hex   = if (conf.debug == true) Output(UInt(32.W))              else Output(UInt(0.W))

  val reg_wren   = if (conf.debug == true) Output(Bool())            else Output(UInt(0.W))
  val reg_wraddr = if (conf.debug == true) Output(UInt(5.W))         else Output(UInt(0.W))
  val reg_wrdata = if (conf.debug == true) Output(SInt(conf.xlen.W)) else Output(SInt(0.W))

  // ALU
  val alu_rdata0  = if (conf.debug == true) Output(SInt(conf.xlen.W))  else Output(SInt(0.W))
  val alu_reg_rs0 = if (conf.debug == true) Output(UInt(5.W))          else Output(SInt(0.W))
  val alu_rdata1  = if (conf.debug == true) Output(SInt(conf.xlen.W))  else Output(SInt(0.W))
  val alu_reg_rs1 = if (conf.debug == true) Output(UInt(5.W))          else Output(SInt(0.W))
  val alu_func    = if (conf.debug == true) Output(UInt(ALU_OP_SIZE))  else Output(UInt(0.W))

  val mem_inst_valid = if (conf.debug == true) Output(Bool())            else Output(UInt(0.W))
  val mem_inst_rd    = if (conf.debug == true) Output(UInt(5.W))         else Output(UInt(0.W))
  val mem_alu_res    = if (conf.debug == true) Output(SInt(conf.xlen.W)) else Output(SInt(0.W))

  // CSRFile
  val csr_cmd   = if (conf.debug == true) Output(UInt(CSR.CSRCMD_SZ)) else Output(UInt(0.W))
  val csr_addr  = if (conf.debug == true) Output(UInt(12.W))          else Output(UInt(0.W))
  val csr_wdata = if (conf.debug == true) Output(UInt(conf.xlen.W))   else Output(UInt(0.W))

  // DataBus
  val data_bus_req    = if (conf.debug == true) Output(Bool())                 else Output(UInt(0.W))
  val data_bus_cmd    = if (conf.debug == true) Output(UInt(2.W))              else Output(UInt(0.W))
  val data_bus_addr   = if (conf.debug == true) Output(UInt(conf.bus_width.W)) else Output(UInt(0.W))
  val data_bus_wrdata = if (conf.debug == true) Output(SInt(conf.xlen.W))      else Output(SInt(0.W))
  val data_bus_ack    = if (conf.debug == true) Output(Bool())                 else Output(UInt(0.W))
  val data_bus_rddata = if (conf.debug == true) Output(SInt(conf.xlen.W))      else Output(SInt(0.W))
}

class CpuTopIo [Conf <: RVConfig](conf: Conf) extends Bundle {
  val run         = Input(Bool())
  val ext_bus     = new Bus(conf)
  val dbg_monitor = new CpuDebugMonitor(conf)
}


class CpuIo [Conf <: RVConfig](conf: Conf) extends Bundle {
  val run      = Input(Bool())

  val inst_bus = new InstBus(conf)
  val data_bus = new DataBus(conf)

  val dbg_monitor = new CpuDebugMonitor(conf)
}


class CpuTop [Conf <: RVConfig](conf: Conf) extends Module {
  val io = IO (new CpuTopIo(conf))

  val memory = Module(new Memory(conf))
  val cpu    = Module(new Cpu(conf))

  cpu.io.run       := io.run

  // Connect CPU and Memory
  memory.io.inst_bus <> cpu.io.inst_bus
  memory.io.data_bus <> cpu.io.data_bus
  // Memory Load for External Debug
  memory.io.ext_bus  <> io.ext_bus

  io.dbg_monitor <> cpu.io.dbg_monitor
}


class Cpu [Conf <: RVConfig](conf: Conf) extends Module {
  val io = IO (new CpuIo(conf))

  val cycle = RegInit(0.U(32.W))
  cycle := cycle + 1.U

  val u_cpath    = Module (new CtlPath)
  val u_regs     = Module (new Regs(conf))
  val u_alu      = Module (new Alu(conf))
  val u_mem_sext = Module (new SExt(conf))
  val u_csrfile  = Module (new CsrFile(conf))

  val if_inst_addr = RegInit(0.U(conf.bus_width.W))
  val if_inst_en   = RegInit(false.B)

  // Get Instruction
  val dec_inst_data  = Wire(UInt(32.W))
  val dec_inst_addr  = Reg(UInt(conf.bus_width.W))
  val dec_inst_valid = Wire(Bool())

  val dec_inst_rs2 = dec_inst_data(24,20)
  val dec_inst_rs1 = dec_inst_data(19,15)
  val dec_inst_rd  = dec_inst_data(11, 7)

  val dec_imm_i      = dec_inst_data(31, 20).asSInt
  val dec_imm_b      = Cat(dec_inst_data(31), dec_inst_data(7), dec_inst_data(30,25), dec_inst_data(11,8))
  val dec_imm_b_sext = Cat(Fill(19,dec_imm_b(11)), dec_imm_b, 0.U)
  val dec_imm_s      = Cat(dec_inst_data(31, 25), dec_inst_data(11,7)).asSInt
  val dec_imm_j      = Cat(dec_inst_data(31), dec_inst_data(19,12), dec_inst_data(20), dec_inst_data(30,21), 0.U(1.W))
  val dec_imm_j_sext = Cat(Fill(64-21, dec_imm_j(20)), dec_imm_j, 0.U(1.W))

  val dec_rdata_op0 = Wire(SInt(conf.xlen.W))
  val dec_rdata_op1 = Wire(SInt(conf.xlen.W))

  val dec_inst_wb_en = u_cpath.io.ctl.wb_en & (dec_inst_rd =/= 0.U)

  val dec_jalr_en   = u_cpath.io.ctl.jalr
  val dec_jal_en    = u_cpath.io.ctl.jal
  val dec_br_en     = u_cpath.io.ctl.br
  val dec_csr_wbcsr = u_cpath.io.ctl.wbcsr
  val dec_mret_en   = (dec_csr_wbcsr === CSR.Mret)
  val dec_ecall_en  = (dec_csr_wbcsr === CSR.Inst)
  val dec_jump_en   = dec_jalr_en | dec_jal_en | dec_br_en | dec_mret_en

  val dec_stall_en  = dec_inst_valid & (u_cpath.io.ctl.mem_cmd =/= MCMD_X)

  val wb_mem_rdval = Wire(SInt(conf.xlen.W))

  val ex_inst_valid = RegNext(dec_inst_valid)
  val ex_inst_data  = RegNext(dec_inst_data)
  val ex_inst_addr  = RegNext(dec_inst_addr)

  val ex_inst_rs0 = RegNext (dec_inst_rs1)
  val ex_inst_rs1 = RegNext (dec_inst_rs2)
  val ex_alu_func = RegNext (u_cpath.io.ctl.alu_fun)
  val ex_alu_op0  = Wire (SInt(conf.xlen.W))
  val ex_alu_op1  = Wire (SInt(conf.xlen.W))
  val ex_inst_rd = RegNext (dec_inst_rd)

  val ex_jalr_en   = RegNext (dec_jalr_en)
  val ex_jal_en    = RegNext (dec_jal_en)
  val ex_br_en     = RegNext (dec_br_en)
  val ex_br_jump   = ex_br_en & (u_alu.io.res === 1.S)
  val ex_mret_en   = RegNext (dec_mret_en)
  val ex_ecall_en  = RegNext (dec_ecall_en)
  val ex_jump_en   = ex_inst_valid & (ex_jalr_en | ex_jal_en | ex_br_jump | ex_mret_en | ex_ecall_en)

  val ex_op0_sel = RegNext (u_cpath.io.ctl.op0_sel)
  val ex_op1_sel = RegNext (u_cpath.io.ctl.op1_sel)

  val ex_imm_i      = RegNext (dec_imm_i)
  val ex_imm_b_sext = RegNext (dec_imm_b_sext)
  val ex_imm_s      = RegNext (dec_imm_s)
  val ex_imm_j      = RegNext (dec_imm_j)

  val ex_inst_wb_en    = RegNext (dec_inst_wb_en)
  val ex_ctrl_mem_v    = RegNext (u_cpath.io.ctl.mem_v)
  val ex_ctrl_mem_cmd  = RegNext (u_cpath.io.ctl.mem_cmd)
  val ex_ctrl_mem_type = RegNext (u_cpath.io.ctl.mem_type)
  val ex_rdata_op0     = RegNext (dec_rdata_op0)
  val ex_rdata_op1     = RegNext (dec_rdata_op1)
  val ex_csr_wbcsr     = RegNext (dec_csr_wbcsr)

  val ex_mem_val = Wire(SInt(conf.xlen.W))

  //
  // Memory Access Stage Signals
  //
  val mem_inst_valid = RegNext(ex_inst_valid)
  val mem_inst_data  = RegNext(ex_inst_data)
  val mem_inst_addr  = RegNext(ex_inst_addr)

  val mem_inst_rd    = RegNext (ex_inst_rd)
  val mem_inst_wb_en = RegNext (ex_inst_wb_en)
  val mem_alu_res    = RegNext (u_alu.io.res)

  val mem_imm_i      = RegNext (ex_imm_i)

  val mem_ctrl_mem_v    = RegNext (ex_ctrl_mem_v)
  val mem_ctrl_mem_cmd  = RegNext (ex_ctrl_mem_cmd)
  val mem_ctrl_mem_type = RegNext (ex_ctrl_mem_type)
  val mem_rdata_op0     = RegNext (ex_rdata_op0)
  val mem_rdata_op1     = RegNext (ex_rdata_op1)

  val mem_mem_val = RegNext (ex_mem_val)

  val mem_jalr_en   = RegNext (ex_jalr_en)
  val mem_jal_en    = RegNext (ex_jal_en)
  val mem_br_en     = RegNext (ex_br_en)
  val mem_mret_en   = RegNext (ex_mret_en)
  val mem_ecall_en  = RegNext (ex_ecall_en)
  val mem_csr_wbcsr = RegNext (ex_csr_wbcsr)

  //
  // WB-Stage Signals
  //

  val wb_inst_valid = RegNext (mem_inst_valid)
  val wb_inst_data  = RegNext (mem_inst_data)
  val wb_inst_addr  = RegNext (mem_inst_addr)

  val wb_rdata_op0  = RegNext (mem_rdata_op0)
  val wb_rdata_op1  = RegNext (mem_rdata_op1)

  val wb_inst_wb_en = RegNext (mem_inst_wb_en)
  val wb_inst_rd    = RegNext (mem_inst_rd)

  val wb_alu_res    = RegNext (mem_alu_res)

  val wb_imm_i      = RegNext (mem_imm_i)

  val wb_jalr_en   = RegNext (mem_jalr_en)
  val wb_jal_en    = RegNext (mem_jal_en)
  val wb_br_en     = RegNext (mem_br_en)
  val wb_mret_en   = RegNext (mem_mret_en)
  val wb_ecall_en  = RegNext (mem_ecall_en)
  val wb_csr_wbcsr = RegNext (mem_csr_wbcsr)

  val wb_ctrl_mem_cmd  = RegNext (mem_ctrl_mem_cmd)
  val wb_ctrl_mem_type = RegNext (mem_ctrl_mem_type)

  if_inst_en := io.run

  if_inst_addr := MuxCase (if_inst_addr, Array (
    (ex_inst_valid & ex_jalr_en)  -> u_alu.io.res.asUInt,
    (ex_inst_valid & ex_jal_en)   -> (ex_inst_addr + ex_imm_j),
    (ex_inst_valid & ex_br_jump)  -> (ex_inst_addr + ex_imm_b_sext),
    (ex_inst_valid & ex_mret_en)  -> u_csrfile.io.mepc,
    (ex_inst_valid & ex_ecall_en) -> u_csrfile.io.mtvec,
    (if_inst_en & !dec_stall_en)  -> (if_inst_addr + 4.U)
  ))

  // if (conf.debug == true) {
  //   when (if_inst_en & dec_jalr_en) {
  //     printf("%d : JALR is enable %x, %x\n", cycle, dec_rdata_op0.asUInt, if_inst_addr)
  //   }
  //   when (if_inst_en & dec_jal_en) {
  //     printf("%d : JAL  is enable %x, %x\n", cycle, dec_rdata_op0.asUInt, if_inst_addr)
  //   }
  //   when (if_inst_en & dec_br_en) {
  //     printf("%d : BR   is enable %x, %x\n", cycle, dec_rdata_op0.asUInt, if_inst_addr)
  //   }
  //   when (if_inst_en & dec_mret_en) {
  //     printf("%d : MRET is enable %x, %x\n", cycle, dec_rdata_op0.asUInt, if_inst_addr)
  //   }
  //   when (if_inst_en & dec_ecall_en) {
  //     printf("%d : ECAL is enable %x, %x\n", cycle, dec_rdata_op0.asUInt, if_inst_addr)
  //   }
  // }

  io.inst_bus.req  := if_inst_en & !(ex_jump_en | dec_stall_en)
  io.inst_bus.addr := if_inst_addr

  dec_inst_data := io.inst_bus.rddata.asUInt
  dec_inst_addr := if_inst_addr
  when  (if_inst_en & io.inst_bus.ack) {
    dec_inst_valid := Mux(ex_jump_en, false.B, io.inst_bus.ack)
  } .otherwise {
    dec_inst_valid := false.B
  }

  // Opcode extraction and Register Read

  u_cpath.io.inst := Mux(io.inst_bus.ack, dec_inst_data, 0.U)

  u_regs.io.rden0   := true.B
  u_regs.io.rdaddr0 := dec_inst_rs1
  dec_rdata_op0     := u_regs.io.rddata0

  u_regs.io.rden1   := true.B
  u_regs.io.rdaddr1 := dec_inst_rs2
  dec_rdata_op1     := u_regs.io.rddata1

  ex_alu_op0 := 0.S
  switch(ex_op0_sel) {
    is (OP0_RS1) {
      // Register Forwarding
      when (mem_inst_valid & mem_inst_wb_en & (mem_inst_rd === ex_inst_rs0)) {
        ex_alu_op0 := mem_alu_res
      } .elsewhen (wb_inst_valid & wb_inst_wb_en & (wb_inst_rd === ex_inst_rs0)) {
        ex_alu_op0 := Mux (wb_ctrl_mem_cmd === MCMD_RD, wb_mem_rdval, wb_alu_res)
      } .otherwise {
        ex_alu_op0 := ex_rdata_op0
      }
    }
    is (OP0_IMU) { ex_alu_op0 := Cat(ex_inst_data(31, 12), Fill(12,0.U)).asSInt }
    is (OP0_IMZ) { ex_alu_op0 := Cat(Fill(27,0.U), ex_inst_data(19,15)).asSInt }
  }

  ex_alu_op1 := 0.S
  switch(ex_op1_sel) {
    // Register Forwarding
    is (OP1_PC ) { ex_alu_op1 := ex_inst_addr.asSInt }
    is (OP1_RS2) {
      when (mem_inst_valid & mem_inst_wb_en & (mem_inst_rd === ex_inst_rs1)) {
        ex_alu_op1 := mem_alu_res
      } .elsewhen (wb_inst_valid & wb_inst_wb_en & (wb_inst_rd === ex_inst_rs1)) {
        ex_alu_op1 := Mux (wb_ctrl_mem_cmd === MCMD_RD, wb_mem_rdval, wb_alu_res)
      } .elsewhen (ex_csr_wbcsr =/= CSR.X) {
        ex_alu_op1 := u_csrfile.io.rw.rdata.asSInt
      } .otherwise {
        ex_alu_op1 := ex_rdata_op1
      }
    }
    is (OP1_IMI) { ex_alu_op1 := Cat(Fill(20,ex_imm_i(11)), ex_imm_i).asSInt }
    is (OP1_IMS) { ex_alu_op1 := Cat(Fill(20,ex_imm_s(11)), ex_imm_s).asSInt }
  }

  when (mem_inst_valid & mem_inst_wb_en & (mem_inst_rd === ex_inst_rs1)) {
    ex_mem_val := mem_alu_res
  } .elsewhen (wb_inst_valid & wb_inst_wb_en & (wb_inst_rd === ex_inst_rs1)) {
    ex_mem_val := wb_alu_res
  } .otherwise {
    ex_mem_val := ex_rdata_op1
  }

  //
  // ALU Input
  //
  u_alu.io.func := ex_alu_func
  u_alu.io.op0  := ex_alu_op0
  u_alu.io.op1  := ex_alu_op1

  io.data_bus.req    := mem_ctrl_mem_v
  io.data_bus.cmd    := mem_ctrl_mem_cmd
  io.data_bus.size   := mem_ctrl_mem_type
  io.data_bus.addr   := mem_alu_res.asUInt
  io.data_bus.wrdata := mem_mem_val

  //
  // WB-Stage Signals
  //
  u_regs.io.wren   := wb_inst_valid & wb_inst_wb_en
  u_regs.io.wraddr := wb_inst_rd
  when ((wb_jal_en === Y) | (wb_jalr_en === Y)) {
    u_regs.io.wrdata := wb_inst_addr.asSInt + 4.S
  } .elsewhen (wb_ctrl_mem_cmd === MCMD_RD) {
    u_regs.io.wrdata := wb_mem_rdval
  } .otherwise {
    u_regs.io.wrdata := wb_alu_res
  }

  u_mem_sext.io.in_val   := io.data_bus.rddata
  u_mem_sext.io.ext_type := wb_ctrl_mem_type
  wb_mem_rdval           := u_mem_sext.io.out_val

  //
  // CSR Port
  //
  u_csrfile.io.rw.cmd     := ex_csr_wbcsr
  u_csrfile.io.rw.addr    := ex_imm_i.asUInt
  u_csrfile.io.rw.wdata   := ex_alu_op0.asUInt
  u_csrfile.io.ecall_inst := ex_ecall_en

  if (conf.debug == true) {

    val debug_pc_update_cause = Wire(UInt(3.W))
    debug_pc_update_cause := MuxCase (0.U, Array (
      (ex_inst_valid & ex_jalr_en)  -> 1.U,
      (ex_inst_valid & ex_jal_en)   -> 2.U,
      (ex_inst_valid & ex_br_jump)  -> 3.U,
      (ex_inst_valid & ex_mret_en)  -> 4.U,
      (ex_inst_valid & ex_ecall_en) -> 5.U,
      (if_inst_en)                  -> 6.U
    ))

    /* Debug-Port */
    io.dbg_monitor.inst_fetch_req    := io.inst_bus.req
    io.dbg_monitor.inst_fetch_addr   := io.inst_bus.addr
    io.dbg_monitor.inst_fetch_ack    := io.inst_bus.ack
    io.dbg_monitor.inst_fetch_rddata := io.inst_bus.rddata

    io.dbg_monitor.pc_update_cause := debug_pc_update_cause

    io.dbg_monitor.inst_valid := wb_inst_valid
    io.dbg_monitor.inst_addr  := wb_inst_addr
    io.dbg_monitor.inst_hex   := wb_inst_data

    io.dbg_monitor.reg_wren   := u_regs.io.wren
    io.dbg_monitor.reg_wraddr := u_regs.io.wraddr
    io.dbg_monitor.reg_wrdata := u_regs.io.wrdata

    io.dbg_monitor.csr_cmd   := ex_csr_wbcsr
    io.dbg_monitor.csr_addr  := ex_imm_i.asUInt
    io.dbg_monitor.csr_wdata := ex_alu_op0.asUInt

    io.dbg_monitor.alu_rdata0  := u_alu.io.op0
    io.dbg_monitor.alu_reg_rs0 := Mux(ex_op0_sel === OP0_RS1, ex_inst_rs0, 0.U)
    io.dbg_monitor.alu_rdata1  := u_alu.io.op1
    io.dbg_monitor.alu_reg_rs1 := Mux(ex_op1_sel === OP1_RS2, ex_inst_rs1, 0.U)
    io.dbg_monitor.alu_func    := Mux(ex_inst_valid, u_alu.io.func, 0.U)

    io.dbg_monitor.mem_inst_valid := mem_inst_valid & mem_inst_wb_en
    io.dbg_monitor.mem_inst_rd    := mem_inst_rd
    io.dbg_monitor.mem_alu_res    := mem_alu_res

    io.dbg_monitor.data_bus_req    := io.data_bus.req
    io.dbg_monitor.data_bus_cmd    := io.data_bus.cmd
    io.dbg_monitor.data_bus_addr   := io.data_bus.addr
    io.dbg_monitor.data_bus_wrdata := io.data_bus.wrdata
    io.dbg_monitor.data_bus_ack    := io.data_bus.ack
    io.dbg_monitor.data_bus_rddata := io.data_bus.rddata
  }
}


class SExt [Conf <: RVConfig](conf: Conf) extends Module {
  val io = IO (new Bundle {
    val in_val   = Input(SInt(conf.xlen.W))
    val ext_type = Input(UInt(MT_SIZE))

    val out_val = Output(SInt(conf.xlen.W))
  })

  io.out_val := io.in_val
  switch (io.ext_type) {
    is (MT_B ) { io.out_val := Cat(Fill(conf.xlen-8 , io.in_val( 7)), io.in_val( 7, 0)).asSInt }
    is (MT_BU) { io.out_val := Cat(Fill(conf.xlen-8 , 0.U          ), io.in_val( 7, 0)).asSInt }
    is (MT_H ) { io.out_val := Cat(Fill(conf.xlen-16, io.in_val(15)), io.in_val(15, 0)).asSInt }
    is (MT_HU) { io.out_val := Cat(Fill(conf.xlen-16, 0.U          ), io.in_val(15, 0)).asSInt }
    is (MT_W ) { io.out_val := Cat(Fill(conf.xlen-32, io.in_val(31)), io.in_val(31, 0)).asSInt }
    is (MT_WU) { io.out_val := Cat(Fill(conf.xlen-32, 0.U          ), io.in_val(31, 0)).asSInt }
    is (MT_D ) { io.out_val := io.in_val                                                       }
  }
}


object CpuTop extends App {
  chisel3.Driver.execute(args, () => new CpuTop(new RV64ISynth))
}
