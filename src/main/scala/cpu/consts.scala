package cpu

import chisel3._
import chisel3.util._

object BusConsts
{
  val CMD_WR   = 1.U(2.W)
  val CMD_RD   = 2.U(2.W)
  val CMD_IDLE = 0.U(2.W)
}


object DecodeConsts
{
  //************************************
  // Control Signals

  val Y      = true.B
  val N      = false.B

  // RS1 Operand Select Signal
  val OP0_SZ = 2.W
  def OP0_RS1 = 0.U(OP0_SZ) // Register Source #1
  def OP0_IMU = 1.U(OP0_SZ) // immediate, U-type
  def OP0_IMZ = 2.U(OP0_SZ) // Zero-extended rs1 field of inst, for CSRI instructions
  def OP0_X   = 0.U(OP0_SZ)

  // RS2 Operand Select Signal
  val OP1_SZ = 2.W
  def OP1_RS2 = 0.U(OP1_SZ) // Register Source #2
  def OP1_IMI = 1.U(OP1_SZ) // immediate, I-type
  def OP1_IMS = 2.U(OP1_SZ) // immediate, S-type
  def OP1_PC  = 3.U(OP1_SZ) // PC
  def OP1_X   = 0.U(OP1_SZ)

  // ALU Operation Signal
  val ALU_OP_SIZE = 5.W
  def ALU_ADD    =  1.U(ALU_OP_SIZE)
  def ALU_SUB    =  2.U(ALU_OP_SIZE)
  def ALU_SLL    =  3.U(ALU_OP_SIZE)
  def ALU_SRL    =  4.U(ALU_OP_SIZE)
  def ALU_SRA    =  5.U(ALU_OP_SIZE)
  def ALU_AND    =  6.U(ALU_OP_SIZE)
  def ALU_OR     =  7.U(ALU_OP_SIZE)
  def ALU_XOR    =  8.U(ALU_OP_SIZE)
  def ALU_SLT    =  9.U(ALU_OP_SIZE)
  def ALU_SLTU   = 10.U(ALU_OP_SIZE)
  def ALU_SNE    = 11.U(ALU_OP_SIZE)
  def ALU_SEQ    = 12.U(ALU_OP_SIZE)
  def ALU_SGE    = 13.U(ALU_OP_SIZE)
  def ALU_SGEU   = 14.U(ALU_OP_SIZE)
  def ALU_COPY1  = 15.U(ALU_OP_SIZE)
  def ALU_ADDW   = 16.U(ALU_OP_SIZE)
  def ALU_SUBW   = 17.U(ALU_OP_SIZE)
  def ALU_SLLW   = 18.U(ALU_OP_SIZE)
  def ALU_SRLW   = 19.U(ALU_OP_SIZE)
  def ALU_SRAW   = 20.U(ALU_OP_SIZE)
  def ALU_COPY2  = 21.U(ALU_OP_SIZE)
  def ALU_MUL    = 22.U(ALU_OP_SIZE)
  def ALU_MULH   = 23.U(ALU_OP_SIZE)
  def ALU_MULHSU = 24.U(ALU_OP_SIZE)
  def ALU_MULHU  = 25.U(ALU_OP_SIZE)
  def ALU_MULW   = 26.U(ALU_OP_SIZE)
  def ALU_X      = 0.U(ALU_OP_SIZE)

  // Memory Mask Type Signal
  val MT_SIZE = 3.W
  def MT_B   = 1.U(MT_SIZE)
  def MT_BU  = 2.U(MT_SIZE)
  def MT_H   = 3.U(MT_SIZE)
  def MT_HU  = 4.U(MT_SIZE)
  def MT_W   = 5.U(MT_SIZE)
  def MT_WU  = 6.U(MT_SIZE)
  def MT_D   = 7.U(MT_SIZE)
  def MT_X   = 0.U(MT_SIZE)

  // Memory Functions (read, write, fence)
  val MCMD_SIZE = 2.W
  def MCMD_WR = 1.U(MCMD_SIZE)
  def MCMD_RD = 2.U(MCMD_SIZE)
  def MCMD_FE = 3.U(MCMD_SIZE)
  def MCMD_X  = 0.U(MCMD_SIZE)
}


object Causes {
  val MisFetch		= 0x0
  val FetchAccess	= 0x1
  val IllegalInst	= 0x2
  val BreakPoint	= 0x3
  val MisalignLoad	= 0x4
  val LoadAccess	= 0x5
  val MisalignStore = 0x6
  val StoreAccess	= 0x7
  val UserEcall		= 0x8
  val MachineEcall	= 0xb
}
