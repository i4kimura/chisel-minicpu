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

  // PC Select Signal
  val PC_4   = 0.U(3.W)  // PC + 4
  val PC_BR  = 1.U(3.W)  // branch_target
  val PC_J   = 2.U(3.W)  // jump_target
  val PC_JR  = 3.U(3.W)  // jump_reg_target
  val PC_EXC = 4.U(3.W)  // exception

  // Branch Type
  val BR_N   = 0.U(4.W)  // Next
  val BR_NE  = 1.U(4.W)  // Branch on NotEqual
  val BR_EQ  = 2.U(4.W)  // Branch on Equal
  val BR_GE  = 3.U(4.W)  // Branch on Greater/Equal
  val BR_GEU = 4.U(4.W)  // Branch on Greater/Equal Unsigned
  val BR_LT  = 5.U(4.W)  // Branch on Less Than
  val BR_LTU = 6.U(4.W)  // Branch on Less Than Unsigned
  val BR_J   = 7.U(4.W)  // Jump
  val BR_JR  = 8.U(4.W)  // Jump Register

  // RS1 Operand Select Signal
  val OP1_RS1 = 0.U(2.W) // Register Source #1
  val OP1_IMU = 1.U(2.W) // immediate, U-type
  val OP1_IMZ = 2.U(2.W) // Zero-extended rs1 field of inst, for CSRI instructions
  val OP1_X   = 0.U(2.W)

  // RS2 Operand Select Signal
  val OP2_RS2 = 0.U(2.W) // Register Source #2
  val OP2_IMI = 1.U(2.W) // immediate, I-type
  val OP2_IMS = 2.U(2.W) // immediate, S-type
  val OP2_PC  = 3.U(2.W) // PC
  val OP2_X   = 0.U(2.W)

  // Register File Write Enable Signal
  val REN_0   = false.B
  val REN_1   = true.B
  val REN_X   = false.B

  // ALU Operation Signal
  val ALU_OP_SIZE = 5.W
  val ALU_ADD    =  1.U(ALU_OP_SIZE)
  val ALU_SUB    =  2.U(ALU_OP_SIZE)
  val ALU_SLL    =  3.U(ALU_OP_SIZE)
  val ALU_SRL    =  4.U(ALU_OP_SIZE)
  val ALU_SRA    =  5.U(ALU_OP_SIZE)
  val ALU_AND    =  6.U(ALU_OP_SIZE)
  val ALU_OR     =  7.U(ALU_OP_SIZE)
  val ALU_XOR    =  8.U(ALU_OP_SIZE)
  val ALU_SLT    =  9.U(ALU_OP_SIZE)
  val ALU_SLTU   = 10.U(ALU_OP_SIZE)
  val ALU_SNE    = 11.U(ALU_OP_SIZE)
  val ALU_SEQ    = 12.U(ALU_OP_SIZE)
  val ALU_SGE    = 13.U(ALU_OP_SIZE)
  val ALU_SGEU   = 14.U(ALU_OP_SIZE)
  val ALU_COPY1  = 15.U(ALU_OP_SIZE)
  val ALU_ADDW   = 16.U(ALU_OP_SIZE)
  val ALU_SUBW   = 17.U(ALU_OP_SIZE)
  val ALU_SLLW   = 18.U(ALU_OP_SIZE)
  val ALU_SRLW   = 19.U(ALU_OP_SIZE)
  val ALU_SRAW   = 20.U(ALU_OP_SIZE)
  val ALU_COPY2  = 21.U(ALU_OP_SIZE)
  val ALU_MUL    = 22.U(ALU_OP_SIZE)
  val ALU_MULH   = 23.U(ALU_OP_SIZE)
  val ALU_MULHSU = 24.U(ALU_OP_SIZE)
  val ALU_MULHU  = 25.U(ALU_OP_SIZE)
  val ALU_MULW   = 26.U(ALU_OP_SIZE)
  val ALU_X      = 0.U(ALU_OP_SIZE)

  // Writeback Select Signal
  val WB_ALU  = 0.U(2.W)
  val WB_MEM  = 1.U(2.W)
  val WB_PC4  = 2.U(2.W)
  val WB_CSR  = 3.U(2.W)
  val WB_X    = 0.U(2.W)

  // Memory Mask Type Signal
  val MT_SIZE = 3.W
  val MT_B   = 1.U(MT_SIZE)
  val MT_BU  = 2.U(MT_SIZE)
  val MT_H   = 3.U(MT_SIZE)
  val MT_HU  = 4.U(MT_SIZE)
  val MT_W   = 5.U(MT_SIZE)
  val MT_WU  = 6.U(MT_SIZE)
  val MT_D   = 7.U(MT_SIZE)
  val MT_X   = 0.U(MT_SIZE)

  // Memory Functions (read, write, fence)
  val MCMD_WR = 1.U(2.W)
  val MCMD_RD = 2.U(2.W)
  val MCMD_FE = 3.U(2.W)
  val MCMD_X  = 0.U(2.W)

  // Cache Flushes & Sync Primitives
  val M_N      = 0.U(3.W)
  val M_SI     = 1.U(3.W)   // synch instruction stream
  val M_SD     = 2.U(3.W)   // synch data stream
  val M_FA     = 3.U(3.W)   // flush all caches
  val M_FD     = 4.U(3.W)   // flush data cache

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
