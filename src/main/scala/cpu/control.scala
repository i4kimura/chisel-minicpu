package cpu

import chisel3._
import chisel3.util._

import Instructions._
import DecodeConsts._

object CtlIdx
{
  val InstVld = 0
  val Op1     = 1
  val Op2     = 2
  val Alu     = 3
  val Wb      = 4
  val WbCsr   = 5
  val Jal     = 6
  val Jalr    = 7
  val Br      = 8
  val MemV    = 9
  val MemCmd  = 10
  val MemType = 11

}


class CtrlSignals extends Bundle()
{
  val exe_kill   = Output(Bool())    // squash EX stage (exception/mret occurred)
  val pc_sel     = Output(UInt(3.W))
  val brjmp_sel  = Output(Bool())
  val op1_sel    = Output(UInt(2.W))
  val op2_sel    = Output(UInt(2.W))
  val alu_fun    = Output(UInt(ALU_OP_SIZE))
  val wb_en      = Output(UInt(2.W))
  val rf_wen     = Output(Bool())
  val bypassable = Output(Bool())     // instruction's result can be bypassed
  val wbcsr      = Output(UInt(3.W))
  val jal        = Output(Bool())
  val jalr       = Output(Bool())
  val br         = Output(Bool())

  val mem_v    = Output(Bool())
  val mem_cmd  = Output(UInt(2.W))
  val mem_type = Output(UInt(3.W))

  val exception = Output(Bool())
}


class CpathIo extends Bundle()
{
  val inst = Input(UInt(32.W))
  val ctl  = new CtrlSignals()
  override def clone = { new CpathIo().asInstanceOf[this.type] }
}


class CtlPath extends Module
{
  val io = IO(new CpathIo())
  io := DontCare

  val csignals =
    ListLookup(io.inst,
                   List(N, OP1_X  ,  OP2_X  , ALU_X     , N  ,CSR.X,     N,   N,     N,    N,      MCMD_X ,  MT_X  ),
      Array(      /* val  | op1   |   op2     |  ALU    | WB |WB_CSR    | JAL | JALR | BR | MEM_V | MEM_CMD | MEM_T */
                  /* inst |  sel  |    sel    |   fcn   |    |          |     |      |    |       |         |       */
        LD      -> List(Y, OP1_RS1, OP2_IMI , ALU_ADD   , Y,  CSR.X,     N,   N,     N,    Y,      MCMD_RD,  MT_D  ),
        LW      -> List(Y, OP1_RS1, OP2_IMI , ALU_ADD   , Y,  CSR.X,     N,   N,     N,    Y,      MCMD_RD,  MT_W  ),
        LB      -> List(Y, OP1_RS1, OP2_IMI , ALU_ADD   , Y,  CSR.X,     N,   N,     N,    Y,      MCMD_RD,  MT_B  ),
        LBU     -> List(Y, OP1_RS1, OP2_IMI , ALU_ADD   , Y,  CSR.X,     N,   N,     N,    Y,      MCMD_RD,  MT_BU ),
        LH      -> List(Y, OP1_RS1, OP2_IMI , ALU_ADD   , Y,  CSR.X,     N,   N,     N,    Y,      MCMD_RD,  MT_H  ),
        LHU     -> List(Y, OP1_RS1, OP2_IMI , ALU_ADD   , Y,  CSR.X,     N,   N,     N,    Y,      MCMD_RD,  MT_HU ),
        SD      -> List(Y, OP1_RS1, OP2_IMS , ALU_ADD   , N,  CSR.X,     N,   N,     N,    Y,      MCMD_WR,  MT_D  ),
        SW      -> List(Y, OP1_RS1, OP2_IMS , ALU_ADD   , N,  CSR.X,     N,   N,     N,    Y,      MCMD_WR,  MT_W  ),
        SB      -> List(Y, OP1_RS1, OP2_IMS , ALU_ADD   , N,  CSR.X,     N,   N,     N,    Y,      MCMD_WR,  MT_B  ),
        SH      -> List(Y, OP1_RS1, OP2_IMS , ALU_ADD   , N,  CSR.X,     N,   N,     N,    Y,      MCMD_WR,  MT_H  ),

        AUIPC   -> List(Y, OP1_IMU, OP2_PC  , ALU_ADD   , Y,  CSR.X,     N,   N,     N,    N,      MCMD_X,   MT_X  ),
        LUI     -> List(Y, OP1_IMU, OP2_X   , ALU_COPY1 , Y,  CSR.X,     N,   N,     N,    N,      MCMD_X,   MT_X  ),

        MUL     -> List(Y, OP1_RS1, OP2_RS2 , ALU_MUL   , Y,  CSR.X,     N,   N,     N,    N,      MCMD_X,   MT_X  ),
        MULH    -> List(Y, OP1_RS1, OP2_RS2 , ALU_MULH  , Y,  CSR.X,     N,   N,     N,    N,      MCMD_X,   MT_X  ),
        MULHSU  -> List(Y, OP1_RS1, OP2_RS2 , ALU_MULHSU, Y,  CSR.X,     N,   N,     N,    N,      MCMD_X,   MT_X  ),
        MULHU   -> List(Y, OP1_RS1, OP2_RS2 , ALU_MULHU , Y,  CSR.X,     N,   N,     N,    N,      MCMD_X,   MT_X  ),
        MULW    -> List(Y, OP1_RS1, OP2_RS2 , ALU_MULW  , Y,  CSR.X,     N,   N,     N,    N,      MCMD_X,   MT_X  ),

        ADDI    -> List(Y, OP1_RS1, OP2_IMI , ALU_ADD   , Y,  CSR.X,     N,   N,     N,    N,      MCMD_X,   MT_X  ),
        ANDI    -> List(Y, OP1_RS1, OP2_IMI , ALU_AND   , Y,  CSR.X,     N,   N,     N,    N,      MCMD_X,   MT_X  ),
        ORI     -> List(Y, OP1_RS1, OP2_IMI , ALU_OR    , Y,  CSR.X,     N,   N,     N,    N,      MCMD_X,   MT_X  ),
        XORI    -> List(Y, OP1_RS1, OP2_IMI , ALU_XOR   , Y,  CSR.X,     N,   N,     N,    N,      MCMD_X,   MT_X  ),
        SLTI    -> List(Y, OP1_RS1, OP2_IMI , ALU_SLT   , Y,  CSR.X,     N,   N,     N,    N,      MCMD_X,   MT_X  ),
        SLTIU   -> List(Y, OP1_RS1, OP2_IMI , ALU_SLTU  , Y,  CSR.X,     N,   N,     N,    N,      MCMD_X,   MT_X  ),
        SLLI    -> List(Y, OP1_RS1, OP2_IMI , ALU_SLL   , Y,  CSR.X,     N,   N,     N,    N,      MCMD_X,   MT_X  ),
        SRAI    -> List(Y, OP1_RS1, OP2_IMI , ALU_SRA   , Y,  CSR.X,     N,   N,     N,    N,      MCMD_X,   MT_X  ),
        SRLI    -> List(Y, OP1_RS1, OP2_IMI , ALU_SRL   , Y,  CSR.X,     N,   N,     N,    N,      MCMD_X,   MT_X  ),

        SLL     -> List(Y, OP1_RS1, OP2_RS2 , ALU_SLL   , Y,  CSR.X,     N,   N,     N,    N,      MCMD_X,   MT_X  ),
        ADD     -> List(Y, OP1_RS1, OP2_RS2 , ALU_ADD   , Y,  CSR.X,     N,   N,     N,    N,      MCMD_X,   MT_X  ),
        SUB     -> List(Y, OP1_RS1, OP2_RS2 , ALU_SUB   , Y,  CSR.X,     N,   N,     N,    N,      MCMD_X,   MT_X  ),
        SLT     -> List(Y, OP1_RS1, OP2_RS2 , ALU_SLT   , Y,  CSR.X,     N,   N,     N,    N,      MCMD_X,   MT_X  ),
        SLTU    -> List(Y, OP1_RS1, OP2_RS2 , ALU_SLTU  , Y,  CSR.X,     N,   N,     N,    N,      MCMD_X,   MT_X  ),
        AND     -> List(Y, OP1_RS1, OP2_RS2 , ALU_AND   , Y,  CSR.X,     N,   N,     N,    N,      MCMD_X,   MT_X  ),
        OR      -> List(Y, OP1_RS1, OP2_RS2 , ALU_OR    , Y,  CSR.X,     N,   N,     N,    N,      MCMD_X,   MT_X  ),
        XOR     -> List(Y, OP1_RS1, OP2_RS2 , ALU_XOR   , Y,  CSR.X,     N,   N,     N,    N,      MCMD_X,   MT_X  ),
        SRA     -> List(Y, OP1_RS1, OP2_RS2 , ALU_SRA   , Y,  CSR.X,     N,   N,     N,    N,      MCMD_X,   MT_X  ),
        SRL     -> List(Y, OP1_RS1, OP2_RS2 , ALU_SRL   , Y,  CSR.X,     N,   N,     N,    N,      MCMD_X,   MT_X  ),

        ADDIW   -> List(Y, OP1_RS1, OP2_IMI , ALU_ADDW  , Y,  CSR.X,     N,   N,     N,    N,      MCMD_X,   MT_X  ),
        SLLIW   -> List(Y, OP1_RS1, OP2_IMI , ALU_SLLW  , Y,  CSR.X,     N,   N,     N,    N,      MCMD_X,   MT_X  ),
        SRLIW   -> List(Y, OP1_RS1, OP2_IMI , ALU_SRLW  , Y,  CSR.X,     N,   N,     N,    N,      MCMD_X,   MT_X  ),
        SRAIW   -> List(Y, OP1_RS1, OP2_IMI , ALU_SRAW  , Y,  CSR.X,     N,   N,     N,    N,      MCMD_X,   MT_X  ),
        ADDW    -> List(Y, OP1_RS1, OP2_RS2 , ALU_ADDW  , Y,  CSR.X,     N,   N,     N,    N,      MCMD_X,   MT_X  ),
        SUBW    -> List(Y, OP1_RS1, OP2_RS2 , ALU_SUBW  , Y,  CSR.X,     N,   N,     N,    N,      MCMD_X,   MT_X  ),
        SLLW    -> List(Y, OP1_RS1, OP2_RS2 , ALU_SLLW  , Y,  CSR.X,     N,   N,     N,    N,      MCMD_X,   MT_X  ),
        SRLW    -> List(Y, OP1_RS1, OP2_RS2 , ALU_SRLW  , Y,  CSR.X,     N,   N,     N,    N,      MCMD_X,   MT_X  ),
        SRAW    -> List(Y, OP1_RS1, OP2_RS2 , ALU_SRAW  , Y,  CSR.X,     N,   N,     N,    N,      MCMD_X,   MT_X  ),

        JAL     -> List(Y, OP1_X  , OP2_X   , ALU_X     , Y,  CSR.X,     Y,   N,     N,    N,      MCMD_X,   MT_X  ),
        JALR    -> List(Y, OP1_RS1, OP2_IMI , ALU_X     , Y,  CSR.X,     N,   Y,     N,    N,      MCMD_X,   MT_X  ),
        BEQ     -> List(Y, OP1_RS1, OP2_RS2 , ALU_SEQ   , N,  CSR.X,     N,   N,     Y,    N,      MCMD_X,   MT_X  ),
        BNE     -> List(Y, OP1_RS1, OP2_RS2 , ALU_SNE   , N,  CSR.X,     N,   N,     Y,    N,      MCMD_X,   MT_X  ),
        BGE     -> List(Y, OP1_RS1, OP2_RS2 , ALU_SGE   , N,  CSR.X,     N,   N,     Y,    N,      MCMD_X,   MT_X  ),
        BGEU    -> List(Y, OP1_RS1, OP2_RS2 , ALU_SGEU  , N,  CSR.X,     N,   N,     Y,    N,      MCMD_X,   MT_X  ),
        BLT     -> List(Y, OP1_RS1, OP2_RS2 , ALU_SLT   , N,  CSR.X,     N,   N,     Y,    N,      MCMD_X,   MT_X  ),
        BLTU    -> List(Y, OP1_RS1, OP2_RS2 , ALU_SLTU  , N,  CSR.X,     N,   N,     Y,    N,      MCMD_X,   MT_X  ),

        CSRRWI  -> List(Y, OP1_IMZ, OP2_RS2 , ALU_COPY2 , Y,  CSR.Exch , N,   N,     N,    N,      MCMD_X,   MT_X  ),
        CSRRSI  -> List(Y, OP1_IMZ, OP2_RS2 , ALU_COPY2 , Y,  CSR.Set  , N,   N,     N,    N,      MCMD_X,   MT_X  ),
        CSRRCI  -> List(Y, OP1_IMZ, OP2_RS2 , ALU_COPY2 , Y,  CSR.Clear, N,   N,     N,    N,      MCMD_X,   MT_X  ),
        CSRRW   -> List(Y, OP1_RS1, OP2_RS2 , ALU_COPY2 , Y,  CSR.Exch , N,   N,     N,    N,      MCMD_X,   MT_X  ),
        CSRRS   -> List(Y, OP1_RS1, OP2_RS2 , ALU_COPY2 , Y,  CSR.Set  , N,   N,     N,    N,      MCMD_X,   MT_X  ),
        CSRRC   -> List(Y, OP1_RS1, OP2_RS2 , ALU_COPY2 , Y,  CSR.Clear, N,   N,     N,    N,      MCMD_X,   MT_X  ),

        ECALL   -> List(Y, OP1_X  , OP2_X  ,  ALU_COPY1 , Y,  CSR.Inst,  N,   N,     N,    N,      MCMD_X,   MT_X  ),
        MRET    -> List(Y, OP1_X  , OP2_X  ,  ALU_COPY1 , Y,  CSR.Mret,  N,   N,     N,    N,      MCMD_X,   MT_X  ),
        DRET    -> List(Y, OP1_X  , OP2_X  ,  ALU_COPY1 , Y,  CSR.Inst,  N,   N,     N,    N,      MCMD_X,   MT_X  ),
        EBREAK  -> List(Y, OP1_X  , OP2_X  ,  ALU_COPY1 , Y,  CSR.Inst,  N,   N,     N,    N,      MCMD_X,   MT_X  ),
        WFI     -> List(Y, OP1_X  , OP2_X  ,  ALU_COPY1 , Y,  CSR.Inst,  N,   N,     N,    N,      MCMD_X,   MT_X  ), // implemented as a NOP

        FENCE_I -> List(Y, OP1_X  , OP2_X  ,  ALU_X     , N,  CSR.X,     N,   N,     N,    N,      MCMD_X,   MT_X  ),
        FENCE   -> List(Y, OP1_X  , OP2_X  ,  ALU_X     , N,  CSR.X,     N,   N,     N,    N,      MCMD_X,   MT_X  )
          // we are already sequentially consistent, so no need to honor the fence instruction
      ))

  io.ctl.exe_kill   := false.B    // squash EX stage (exception/mret occurred)
  io.ctl.pc_sel     := 0.U
  io.ctl.brjmp_sel  := false.B
  io.ctl.op1_sel    := csignals(CtlIdx.Op1)
  io.ctl.op2_sel    := csignals(CtlIdx.Op2)
  io.ctl.alu_fun    := csignals(CtlIdx.Alu)
  io.ctl.wb_en      := csignals(CtlIdx.Wb)
  io.ctl.rf_wen     := false.B
  io.ctl.bypassable := false.B     // instruction's result can be bypassed
  io.ctl.wbcsr      := csignals(CtlIdx.WbCsr)

  io.ctl.jal        := csignals(CtlIdx.Jal)
  io.ctl.jalr       := csignals(CtlIdx.Jalr)
  io.ctl.br         := csignals(CtlIdx.Br)

  io.ctl.mem_v      := csignals(CtlIdx.MemV)
  io.ctl.mem_cmd    := csignals(CtlIdx.MemCmd)
  io.ctl.mem_type   := csignals(CtlIdx.MemType)

  io.ctl.exception  := false.B

}
