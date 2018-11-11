package cpu

import chisel3._
import chisel3.Bool

import DecodeConsts._

class InstBus extends Bundle {
  val req  = Input(Bool())
  val addr = Input(UInt(8.W))
  val data = Input(UInt(32.W))
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


class CpuTopIo extends Bundle {
  val run       = Input(Bool())
  val instbus   = new InstBus()
  val debugpath = Flipped(new InstBus())
}


class CpuTop extends Module {
  val io = IO (new CpuTopIo())

  val memory = Module(new Memory)
  val cpu    = Module(new Cpu)

  val w_instReq  = cpu.io.o_instReq
  val w_instAddr = cpu.io.o_instAddr

  // Connect CPU and Memory
  memory.io.ren    := w_instReq
  memory.io.rdaddr := w_instAddr(7, 2)

  memory.io.wen    := false.B
  memory.io.wraddr := 0.U
  memory.io.wrdata := 0.U

  cpu.io.i_instAck   := memory.io.rden
  cpu.io.i_instData  := memory.io.rddata

  cpu.io.i_run       := io.run

  // Memory Load for External Debug
  memory.io.extwen  := io.instbus.req
  memory.io.extaddr := io.instbus.addr
  memory.io.extdata := io.instbus.data

  io.debugpath.req  := w_instReq
  io.debugpath.addr := w_instAddr
  io.debugpath.data := memory.io.rddata

}


class Cpu extends Module {
  val io = IO (new Bundle {
    val i_run      = Input(Bool())

    val o_instAddr = Output(UInt(8.W))
    val o_instReq  = Output(Bool())

    val i_instAck  = Input(Bool())
    val i_instData = Input(UInt(32.W))
  })

  val r_inst_addr = RegInit(0.U(8.W))
  val r_inst_en   = RegInit(false.B)

  r_inst_en := io.i_run

  when(r_inst_en & io.i_instAck) {
    r_inst_addr := r_inst_addr + 4.U
  }

  io.o_instAddr := r_inst_addr
  io.o_instReq  := r_inst_en

  // Get Instruction
  val r_inst_r1 = Reg(UInt(32.W))
  when  (r_inst_en & io.i_instAck) {
    r_inst_r1 := io.i_instData;
  }

  // Opcode extraction and Register Read
  val w_ra_func7  = Wire(UInt(7.W))
  val w_ra_rs2    = Wire(UInt(5.W))
  val w_ra_rs1    = Wire(UInt(5.W))
  val w_ra_func3  = Wire(UInt(3.W))
  val w_ra_rd     = Wire(UInt(5.W))
  val w_ra_opcode = Wire(UInt(5.W))

  w_ra_func7  := r_inst_r1(31,25)
  w_ra_rs2    := r_inst_r1(24,20)
  w_ra_rs1    := r_inst_r1(19,15)
  w_ra_func3  := r_inst_r1(14,12)
  w_ra_rd     := r_inst_r1(11, 7)
  w_ra_opcode := r_inst_r1( 6, 2)

  val cpath = Module(new CtlPath())

  cpath.io.inst := io.i_instData

  val u_regs = Module(new Regs)
  val w_ex_op1 = Wire(SInt(64.W))
  val w_ex_op2 = Wire(SInt(64.W))

  u_regs.io.rden0   := (w_ra_opcode === "hc".asUInt(5.W)) // OP
  u_regs.io.rdaddr0 := w_ra_rs1
  w_ex_op1          := u_regs.io.rddata0

  u_regs.io.rden1   := (w_ra_opcode === "hc".asUInt(5.W)) // OP
  u_regs.io.rdaddr1 := w_ra_rs2
  w_ex_op2          := u_regs.io.rddata1

  u_regs.io.wren   := false.B
  u_regs.io.wraddr := 0.U
  u_regs.io.wrdata := 0.S

  val r_ex_func3 = Reg(UInt(3.W))
  r_ex_func3 := w_ra_func3

  val u_alu = Module (new Alu)
  u_alu.io.i_func := cpath.io.ctl.alu_fun
  u_alu.io.i_op0  := Mux(cpath.io.ctl.op1_sel === OP1_RS1,  w_ex_op1,
                     Mux(cpath.io.ctl.op1_sel === OP1_IMU, 0.S,
                     Mux(cpath.io.ctl.op1_sel === OP1_IMZ, 0.S, 0.S)))
  u_alu.io.i_op1  := Mux(cpath.io.ctl.op2_sel === OP2_RS2,  w_ex_op2,
                     Mux(cpath.io.ctl.op2_sel === OP2_IMI, 0.S,
                     Mux(cpath.io.ctl.op2_sel === OP2_IMS, 0.S, 0.S)))
}


class Regs extends Module {
  val io = IO (new RegIo)

  // val r_regs = RegInit( Vec(32, SInt(64.W)).asTypeOf(0.U) )
  val r_regs = Mem(32, SInt(64.W))

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


class Alu extends Module {
  val io = IO (new Bundle {
    val i_func = Input (UInt(4.W))
    val i_op0  = Input (SInt(64.W))
    val i_op1  = Input (SInt(64.W))

    val o_res  = Output (SInt(64.W))
  })

  val w_res = Wire(SInt(64.W))
  when (io.i_func === ALU_ADD) {
    w_res := io.i_op0 + io.i_op1
  } .elsewhen (io.i_func === ALU_SUB) {
    w_res := io.i_op0 - io.i_op1
  } .elsewhen (io.i_func === ALU_SLL) {
    w_res := (io.i_op0.asUInt << io.i_op1(5,0).asUInt).asSInt
  } .elsewhen (io.i_func === ALU_SRL) {
    w_res := (io.i_op0.asUInt >> io.i_op1(5,0).asUInt).asSInt
  } .elsewhen (io.i_func === ALU_SRA) {
    w_res := (io.i_op0 >> io.i_op1(5,0).asUInt).asSInt
  } .elsewhen (io.i_func === ALU_AND) {
    w_res := io.i_op0 & io.i_op1
  } .elsewhen (io.i_func === ALU_OR) {
    w_res := io.i_op0 | io.i_op1
  } .elsewhen (io.i_func === ALU_XOR) {
    w_res := io.i_op0 ^ io.i_op1
  } .elsewhen (io.i_func === ALU_SLT) {
    w_res := (io.i_op0 > io.i_op1).asSInt
  } .elsewhen (io.i_func === ALU_SLTU) {
    w_res := (io.i_op0.asUInt > io.i_op1.asUInt).asSInt
  } .elsewhen (io.i_func === ALU_COPY1) {
    w_res := io.i_op0
  } .otherwise {
    w_res := io.i_op0
  }

  val r_res = Reg(SInt(64.W))
  r_res := w_res
  io.o_res := w_res
}
