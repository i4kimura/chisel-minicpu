package cpu

import chisel3._
import chisel3.util._
import chisel3.Bool

import DecodeConsts._

class Bus (val bus_width: Int) extends Bundle {
  val req  = Input(Bool())
  val addr = Input(UInt(bus_width.W))
  val data = Input(UInt(32.W))
}


object PrintHex
{
  def apply(x: UInt, length: Int) =
  {
    require(length > 0)
    for (i <- length-1 to 0 by -1) {
      printf("%x", ((x >> (i * 4)) & 0x0f.U))
    }
    0
  }

  def apply(x: SInt, length: Int) =
  {
    require(length > 0)
    for (i <- length-1 to 0 by -1) {
      printf("%x", ((x >> (i * 4)) & 0x0f.S))
    }
    0
  }
}


// object PrintDec
// {
//   def apply(x: UInt, length: Int) =
//   {
//     require(length > 0)
//     for (i <- length-1 to 0 by -1) {
//       printf("%d", ((x / BigDecimal(math.pow(10, i)).toBigInt)) & 0xf.U)
//     }
//   }
//
//   // def apply(x: SInt, length: Int) =
//   // {
//   //   require(length > 0)
//   //   for (i <- length-1 to 0 by -1) {
//   //     printf("%x", (x.asUInt / (math.pow(10, i).toBigInt)) & 0xf.U)
//   //   }
//   // }
// }


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


class CpuTopIo (bus_width: Int) extends Bundle {
  val run       = Input(Bool())
  val ext_bus   = new Bus(bus_width)
}


class CpuDebugMonitor extends Bundle {
  val inst_valid = Output(Bool())
  val inst_addr  = Output(UInt(32.W))
  val inst_hex   = Output(UInt(32.W))
  val reg_wren   = Output(Bool())
  val reg_wraddr = Output(UInt(5.W))
  val reg_wrdata = Output(SInt(64.W))
}

class CpuIo (bus_width: Int) extends Bundle {
  val run      = Input(Bool())

  val inst_addr = Output(UInt(bus_width.W))
  val inst_req  = Output(Bool())

  val inst_ack  = Input(Bool())
  val inst_data = Input(UInt(32.W))

  val dbg_monitor = new CpuDebugMonitor()
}


class CpuTop (bus_width: Int) extends Module {
  val io = IO (new CpuTopIo(bus_width))

  val memory = Module(new Memory(bus_width))
  val cpu    = Module(new Cpu(bus_width))

  val w_instReq  = cpu.io.inst_req
  val w_instAddr = cpu.io.inst_addr

  // Connect CPU and Memory
  memory.io.ren    := w_instReq
  memory.io.rdaddr := w_instAddr(bus_width-1, 2)

  memory.io.wen    := false.B
  memory.io.wraddr := 0.U
  memory.io.wrdata := 0.U

  cpu.io.inst_ack   := memory.io.rden
  cpu.io.inst_data  := memory.io.rddata

  cpu.io.run       := io.run

  // Memory Load for External Debug
  memory.io.ext_bus  <> io.ext_bus

  //
  // Monitor for Debug
  //
  val cycle = RegInit(0.U(32.W))
  cycle := cycle + 1.U

  when (cpu.io.dbg_monitor.inst_valid) {
    val hexbus_width = 8
    printf("%d : ", cycle)
    when (cpu.io.dbg_monitor.reg_wren) {
      printf("x%d<=0x", cpu.io.dbg_monitor.reg_wraddr)
      PrintHex(cpu.io.dbg_monitor.reg_wrdata, 16)
    } .otherwise {
      printf("                     ")
    }
    printf(" : 0x")
    PrintHex(cpu.io.dbg_monitor.inst_addr, 8)
    printf(" : INST(0x")
    PrintHex(cpu.io.dbg_monitor.inst_hex, 8)
    printf(") : DASM(")
    PrintHex(cpu.io.dbg_monitor.inst_hex, 8)
    printf(")\n")
  }
}


class Cpu (bus_width: Int) extends Module {
  val io = IO (new CpuIo(bus_width))

  val r_inst_addr = RegInit(0.U(bus_width.W))
  val r_inst_en   = RegInit(false.B)

  r_inst_en := io.run

  when(r_inst_en & io.inst_ack) {
    r_inst_addr := r_inst_addr + 4.U
  }

  io.inst_addr := r_inst_addr
  io.inst_req  := r_inst_en

  // Get Instruction
  val r_inst_r1       = Reg(UInt(32.W))
  val r_inst_addr_r1  = Reg(UInt(bus_width.W))
  val r_inst_valid_r1 = Reg(Bool())
  when  (r_inst_en & io.inst_ack) {
    r_inst_r1       := io.inst_data
    r_inst_addr_r1  := r_inst_addr
    r_inst_valid_r1 := true.B
  } .otherwise {
    r_inst_valid_r1 := false.B
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

  cpath.io.inst := r_inst_r1

  val u_regs = Module(new Regs)
  val w_ex_op1 = Wire(SInt(64.W))
  val w_ex_op2 = Wire(SInt(64.W))

  u_regs.io.rden0   := (cpath.io.ctl.op1_sel === OP1_RS1)
  u_regs.io.rdaddr0 := w_ra_rs1
  w_ex_op1          := u_regs.io.rddata0

  u_regs.io.rden1   := (cpath.io.ctl.op2_sel === OP2_RS2)
  u_regs.io.rdaddr1 := w_ra_rs2
  w_ex_op2          := u_regs.io.rddata1

  val r_ex_func3 = Reg(UInt(3.W))
  r_ex_func3 := w_ra_func3

  val u_alu = Module (new Alu)
  u_alu.io.func := cpath.io.ctl.alu_fun
  u_alu.io.op0  := Mux(cpath.io.ctl.op1_sel === OP1_RS1, w_ex_op1,
                     Mux(cpath.io.ctl.op1_sel === OP1_IMU, Cat(r_inst_r1(31, 12), Fill(12,0.U)).asSInt,
                     Mux(cpath.io.ctl.op1_sel === OP1_IMZ, Cat(Fill(27,0.U), r_inst_r1(19,15)).asSInt,
                     0.S)))
  val imm_i = r_inst_r1(31, 20).asSInt
  val imm_s = Cat(r_inst_r1(31, 25), r_inst_r1(11,7)).asSInt
  u_alu.io.op1  := Mux(cpath.io.ctl.op2_sel === OP2_RS2, w_ex_op2,
                     Mux(cpath.io.ctl.op2_sel === OP2_IMI, Cat(Fill(20,imm_i(11)), imm_i).asSInt,
                     Mux(cpath.io.ctl.op2_sel === OP2_IMS, Cat(Fill(20,imm_s(11)), imm_s).asSInt,
                     0.S)))

  u_regs.io.wren   := (cpath.io.ctl.alu_fun =/= ALU_X)
  u_regs.io.wraddr := w_ra_rd
  u_regs.io.wrdata := u_alu.io.res

  io.dbg_monitor.inst_valid := r_inst_valid_r1
  io.dbg_monitor.inst_addr  := r_inst_addr_r1
  io.dbg_monitor.inst_hex   := r_inst_r1
  io.dbg_monitor.reg_wren   := u_regs.io.wren
  io.dbg_monitor.reg_wraddr := u_regs.io.wraddr
  io.dbg_monitor.reg_wrdata := u_regs.io.wrdata

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
    val func = Input (UInt(4.W))
    val op0  = Input (SInt(64.W))
    val op1  = Input (SInt(64.W))

    val res  = Output (SInt(64.W))
  })

  // when (io.func =/= ALU_X) {
  //   printf("ALU : OP1[0x")
  //   PrintHex(io.op0, 16)
  //   printf("] OP2[0x")
  //   PrintHex(io.op1, 16)
  //   printf("] %d => ANS[0x", io.func)
  //   PrintHex(io.res, 16)
  //   printf("]\n")
  // }

  val w_res = Wire(SInt(64.W))
  when (io.func === ALU_ADD) {
    w_res := io.op0 + io.op1
  } .elsewhen (io.func === ALU_SUB) {
    w_res := io.op0 - io.op1
  } .elsewhen (io.func === ALU_SLL) {
    w_res := (io.op0.asUInt << io.op1(5,0).asUInt).asSInt
  } .elsewhen (io.func === ALU_SRL) {
    w_res := (io.op0.asUInt >> io.op1(5,0).asUInt).asSInt
  } .elsewhen (io.func === ALU_SRA) {
    w_res := (io.op0 >> io.op1(5,0).asUInt).asSInt
  } .elsewhen (io.func === ALU_AND) {
    w_res := io.op0 & io.op1
  } .elsewhen (io.func === ALU_OR) {
    w_res := io.op0 | io.op1
  } .elsewhen (io.func === ALU_XOR) {
    w_res := io.op0 ^ io.op1
  } .elsewhen (io.func === ALU_SLT) {
    w_res := Mux(io.op0 < io.op1, 1.S, 0.S)
  } .elsewhen (io.func === ALU_SLTU) {
    w_res := Mux(io.op0.asUInt < io.op1.asUInt, 1.S, 0.S)
  } .elsewhen (io.func === ALU_COPY1) {
    w_res := io.op0
  } .otherwise {
    w_res := io.op0
  }

  val r_res = Reg(SInt(64.W))
  r_res := w_res
  io.res := w_res
}


object CpuTop extends App {
  chisel3.Driver.execute(args, () => new CpuTop(bus_width = 16))
}
