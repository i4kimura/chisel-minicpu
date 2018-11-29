package cpu

import chisel3._
import chisel3.util._
import chisel3.Bool

import DecodeConsts._

class Bus (implicit val conf: RV64IConf) extends Bundle {
  val req  = Input(Bool())
  val addr = Input(UInt(conf.bus_width.W))
  val data = Input(SInt(32.W))
}


class InstBus (implicit val conf: RV64IConf) extends Bundle {
  val req    = Output(Bool())
  val addr   = Output(UInt(conf.bus_width.W))

  val ack    = Input(Bool())
  val rddata = Input(SInt(32.W))
}


class DataBus (implicit val conf: RV64IConf) extends Bundle {
  val req    = Output(Bool())
  val cmd    = Output(UInt(2.W))
  val addr   = Output(UInt(conf.bus_width.W))
  val size   = Output(UInt(3.W))
  val wrdata = Output(SInt(64.W))

  val ack    = Input(Bool())
  val rddata = Input(SInt(64.W))
}


class RegIo extends Bundle {
  val rden0   = Input  (Bool())
  val rdaddr0 = Input  (UInt(5.W))
  val rddata0 = Output (SInt(64.W))

  val rden1   = Input  (Bool())
  val rdaddr1 = Input  (UInt(5.W))
  val rddata1 = Output (SInt(64.W))

  val wren   = Input  (Bool())
  val wraddr = Input  (UInt(5.W))
  val wrdata = Input  (SInt(64.W))
}


class CpuDebugMonitor extends Bundle {
  val inst_valid = Output(Bool())
  val inst_addr  = Output(UInt(32.W))
  val inst_hex   = Output(UInt(32.W))

  val reg_wren   = Output(Bool())
  val reg_wraddr = Output(UInt(5.W))
  val reg_wrdata = Output(SInt(64.W))

  // ALU
  val alu_rdata0 = Output(SInt(64.W))
  val alu_rdata1 = Output(SInt(64.W))
  val alu_func   = Output(UInt(ALU_OP_SIZE))

  // DataBus
  val data_bus_req    = Output(Bool())
  val data_bus_cmd    = Output(UInt(2.W))
  val data_bus_addr   = Output(UInt(32.W))
  val data_bus_wrdata = Output(SInt(64.W))
  val data_bus_ack    = Output(Bool())
  val data_bus_rddata = Output(SInt(64.W))
}

class CpuTopIo (implicit val conf: RV64IConf) extends Bundle {
  val run         = Input(Bool())
  val ext_bus     = new Bus()
  val dbg_monitor = new CpuDebugMonitor()
}


class CpuIo (implicit val conf: RV64IConf) extends Bundle {
  val run      = Input(Bool())

  val inst_bus = new InstBus()
  val data_bus = new DataBus()

  val dbg_monitor = new CpuDebugMonitor()
}


class CpuTop (implicit val conf: RV64IConf) extends Module {
  val io = IO (new CpuTopIo())

  val memory = Module(new Memory())
  val cpu    = Module(new Cpu())

  cpu.io.run       := io.run

  // Connect CPU and Memory
  memory.io.inst_bus <> cpu.io.inst_bus
  memory.io.data_bus <> cpu.io.data_bus
  // Memory Load for External Debug
  memory.io.ext_bus  <> io.ext_bus

  io.dbg_monitor <> cpu.io.dbg_monitor
}


class Cpu (implicit val conf: RV64IConf) extends Module {
  val io = IO (new CpuIo())

  val u_cpath   = Module (new CtlPath)
  val u_regs    = Module (new Regs)
  val u_alu     = Module (new Alu)
  val u_csrfile = Module (new CsrFile)

  val if_inst_addr = RegInit(0.U(conf.bus_width.W))
  val if_inst_en   = RegInit(false.B)

  // Get Instruction
  val dec_inst_data  = Reg(UInt(32.W))
  val dec_inst_addr  = Reg(UInt(conf.bus_width.W))
  val dec_inst_valid = Reg(Bool())

  val dec_imm_i      = dec_inst_data(31, 20).asSInt
  val dec_imm_b      = Cat(dec_inst_data(31), dec_inst_data(7), dec_inst_data(30,25), dec_inst_data(11,8))
  val dec_imm_b_sext = Cat(Fill(19,dec_imm_b(11)), dec_imm_b, 0.U)
  val dec_imm_s      = Cat(dec_inst_data(31, 25), dec_inst_data(11,7)).asSInt
  val dec_imm_j      = Cat(dec_inst_data(31), dec_inst_data(19,12), dec_inst_data(20), dec_inst_data(30,21), 0.U(1.W))
  val dec_imm_j_sext = Cat(Fill(64-21, dec_imm_j(20)), dec_imm_j, 0.U(1.W))

  val dec_reg_op0 = Wire(SInt(conf.xlen.W))
  val dec_reg_op1 = Wire(SInt(conf.xlen.W))

  val dec_jalr_en = u_cpath.io.ctl.jalr
  val dec_jal_en  = u_cpath.io.ctl.jal
  val dec_br_en   = u_cpath.io.ctl.br & (u_alu.io.res === 1.S)
  val dec_mret_en = (u_cpath.io.ctl.wbcsr === CSR.Mret)
  val dec_ecall_en = (u_cpath.io.ctl.wbcsr === CSR.Inst)
  val dec_jump_en = if_inst_en & (dec_jalr_en | dec_jal_en | dec_br_en | dec_mret_en)

  if_inst_en := io.run

  if_inst_addr := MuxCase (0.U, Array (
    (if_inst_en & dec_jalr_en) -> dec_reg_op0.asUInt,
    (if_inst_en & dec_jal_en)  -> (dec_inst_addr + dec_imm_j),
    (if_inst_en & dec_br_en)   -> (dec_inst_addr + dec_imm_b_sext),
    (if_inst_en & dec_mret_en) -> u_csrfile.io.mepc,
    (if_inst_en & dec_ecall_en)-> u_csrfile.io.mtvec,
    (if_inst_en & io.inst_bus.ack) -> (if_inst_addr + 4.U)
  ))

  io.inst_bus.req  := if_inst_en
  io.inst_bus.addr := if_inst_addr

  io.data_bus.req    := u_cpath.io.ctl.mem_v
  io.data_bus.cmd    := u_cpath.io.ctl.mem_cmd
  io.data_bus.size   := u_cpath.io.ctl.mem_type
  io.data_bus.addr   := u_alu.io.res.asUInt
  io.data_bus.wrdata := dec_reg_op1

  when  (if_inst_en & io.inst_bus.ack) {
    dec_inst_data := io.inst_bus.rddata.asUInt
    dec_inst_addr := if_inst_addr
    dec_inst_valid := Mux(dec_jump_en, false.B, true.B)
  } .otherwise {
    dec_inst_valid := false.B
  }

  // Opcode extraction and Register Read
  val dec_inst_rs2 = dec_inst_data(24,20)
  val dec_inst_rs1 = dec_inst_data(19,15)
  val dec_inst_rd  = dec_inst_data(11, 7)

  u_cpath.io.inst := dec_inst_data

  u_regs.io.rden0   := true.B
  u_regs.io.rdaddr0 := dec_inst_rs1
  dec_reg_op0       := u_regs.io.rddata0

  u_regs.io.rden1   := true.B
  u_regs.io.rdaddr1 := dec_inst_rs2
  when(u_cpath.io.ctl.wbcsr =/= CSR.X) {
    dec_reg_op1       := u_csrfile.io.rw.rdata.asSInt
  } .otherwise {
    dec_reg_op1       := u_regs.io.rddata1
  }

  u_alu.io.func := u_cpath.io.ctl.alu_fun
  u_alu.io.op0 := 0.S
  switch(u_cpath.io.ctl.op1_sel) {
    is (OP1_RS1) { u_alu.io.op0 := dec_reg_op0 }
    is (OP1_IMU) { u_alu.io.op0 := Cat(dec_inst_data(31, 12), Fill(12,0.U)).asSInt }
    is (OP1_IMZ) { u_alu.io.op0 := Cat(Fill(27,0.U), dec_inst_data(19,15)).asSInt }
  }

  u_alu.io.op1 := 0.S
  switch(u_cpath.io.ctl.op2_sel) {
    is (OP2_PC ) { u_alu.io.op1 := dec_inst_addr.asSInt }
    is (OP2_RS2) { u_alu.io.op1 := dec_reg_op1 }
    is (OP2_IMI) { u_alu.io.op1 := Cat(Fill(20,dec_imm_i(11)), dec_imm_i).asSInt }
    is (OP2_IMS) { u_alu.io.op1 := Cat(Fill(20,dec_imm_s(11)), dec_imm_s).asSInt }
  }

  u_regs.io.wren   := u_cpath.io.ctl.wb_en
  u_regs.io.wraddr := dec_inst_rd
  u_regs.io.wrdata := Mux ((u_cpath.io.ctl.jal === Y) | (u_cpath.io.ctl.jalr === Y), dec_inst_addr.asSInt + 4.S,
                      Mux (u_cpath.io.ctl.mem_cmd =/= MCMD_X, io.data_bus.rddata,
                      u_alu.io.res))

  /* CSR Port */
  u_csrfile.io.rw.cmd   := u_cpath.io.ctl.wbcsr
  u_csrfile.io.rw.addr  := dec_imm_i.asUInt
  u_csrfile.io.rw.wdata := u_alu.io.res.asUInt
  u_csrfile.io.ecall_inst := dec_ecall_en

  /* Debug-Port */
  io.dbg_monitor.inst_valid := dec_inst_valid
  io.dbg_monitor.inst_addr  := dec_inst_addr
  io.dbg_monitor.inst_hex   := dec_inst_data

  io.dbg_monitor.reg_wren   := u_regs.io.wren
  io.dbg_monitor.reg_wraddr := u_regs.io.wraddr
  io.dbg_monitor.reg_wrdata := u_regs.io.wrdata

  io.dbg_monitor.alu_rdata0 := u_alu.io.op0
  io.dbg_monitor.alu_rdata1 := u_alu.io.op1
  io.dbg_monitor.alu_func   := u_alu.io.func

  io.dbg_monitor.data_bus_req    := io.data_bus.req
  io.dbg_monitor.data_bus_cmd    := io.data_bus.cmd
  io.dbg_monitor.data_bus_addr   := io.data_bus.addr
  io.dbg_monitor.data_bus_wrdata := io.data_bus.wrdata
  io.dbg_monitor.data_bus_ack    := io.data_bus.ack
  io.dbg_monitor.data_bus_rddata := io.data_bus.rddata

}


class Regs (implicit val conf: RV64IConf)  extends Module {
  val io = IO (new RegIo)

  // val r_regs = RegInit( Vec(32, SInt(conf.xlen.W)).asTypeOf(0.U) )
  val r_regs = Mem(32, SInt(conf.xlen.W))

  when (io.rden0 && (io.rdaddr0 =/= 0.U(64.W))) {
    io.rddata0 := r_regs(io.rdaddr0)
  } .otherwise {
    io.rddata0 := 0.S(64.W)
  }

  when (io.rden1 && (io.rdaddr1 =/= 0.U(64.W))) {
    io.rddata1 := r_regs(io.rdaddr1)
  } .otherwise {
    io.rddata1 := 0.S(64.W)
  }

  when (io.wren && (io.wraddr =/= 0.U(64.W))) {
    r_regs(io.wraddr) := io.wrdata
  }
}


class Alu (implicit val conf: RV64IConf) extends Module {
  val io = IO (new Bundle {
    val func = Input (UInt(ALU_OP_SIZE))
    val op0  = Input (SInt(conf.xlen.W))
    val op1  = Input (SInt(conf.xlen.W))

    val res  = Output (SInt(conf.xlen.W))
  })

  // when (io.func =/= ALU_X) {
  //   printf("ALU : OP1[0x")
  //   PrintHex(io.i_op0, 16, writer)
  //   printf("] OP2[0x")
  //   PrintHex(io.i_op1, 16, writer)
  //   printf("] %d\n", io.i_func)
  // }

  val w_res = Wire(SInt(conf.xlen.W))
  w_res := 0.S
  switch (io.func) {
    is (ALU_ADD  ) { w_res := io.op0 + io.op1                               }
    is (ALU_SUB  ) { w_res := io.op0 - io.op1                               }
    is (ALU_SLL  ) { w_res := (io.op0.asUInt << io.op1(5,0).asUInt).asSInt  }
    is (ALU_SRL  ) { w_res := (io.op0.asUInt >> io.op1(5,0).asUInt).asSInt  }
    is (ALU_SRA  ) { w_res := (io.op0 >> io.op1(5,0).asUInt).asSInt         }
    is (ALU_AND  ) { w_res := io.op0 & io.op1                               }
    is (ALU_OR   ) { w_res := io.op0 | io.op1                               }
    is (ALU_XOR  ) { w_res := io.op0 ^ io.op1                               }
    is (ALU_SLT  ) { w_res := Mux(io.op0        <  io.op1,        1.S, 0.S) }
    is (ALU_SLTU ) { w_res := Mux(io.op0.asUInt <  io.op1.asUInt, 1.S, 0.S) }
    is (ALU_SNE  ) { w_res := Mux(io.op0 =/= io.op1, 1.S, 0.S)              }
    is (ALU_SEQ  ) { w_res := Mux(io.op0 === io.op1, 1.S, 0.S)              }
    is (ALU_SGE  ) { w_res := Mux(io.op0        >= io.op1,        1.S, 0.S) }
    is (ALU_SGEU ) { w_res := Mux(io.op0.asUInt >= io.op1.asUInt, 1.S, 0.S) }
    is (ALU_COPY1) { w_res := io.op0                                        }
    is (ALU_COPY2) { w_res := io.op1                                        }
    is (ALU_ADDW ) { w_res := (io.op0(31, 0) + io.op1(31, 0)).asSInt        }
    is (ALU_SUBW ) { w_res := (io.op0(31, 0) - io.op1(31, 0)).asSInt               }
    is (ALU_SLLW ) { w_res := (io.op0(31, 0).asUInt << io.op1(5,0).asUInt).asSInt  }
    is (ALU_SRLW ) { w_res := (io.op0(31, 0).asUInt >> io.op1(5,0).asUInt).asSInt  }
    is (ALU_SRAW ) { w_res := (io.op0(31, 0) >> io.op1(5,0).asUInt).asSInt         }
  }

  val r_res = Reg(SInt(conf.xlen.W))
  r_res := w_res
  io.res := w_res
}


object CpuTop extends App {
  implicit val conf = RV64IConf()
  chisel3.Driver.execute(args, () => new CpuTop())
}
