package cpu

import chisel3._
import chisel3.util._

import Instructions._
import DecodeConsts._

class CtrlSignals extends Bundle()
{
  val exe_kill  = Output(Bool())    // squash EX stage (exception/mret occurred)
  val pc_sel    = Output(UInt(3.W))
  val brjmp_sel = Output(Bool())
  val op1_sel   = Output(UInt(2.W))
  val op2_sel   = Output(UInt(2.W))
  val alu_fun   = Output(UInt(4.W))
  val wb_sel    = Output(UInt(2.W))
  val rf_wen    = Output(Bool())
  val bypassable = Output(Bool())     // instruction's result can be bypassed
  val csr_cmd   = Output(UInt(2.W))
  val jal       = Output(Bool())
  val jalr      = Output(Bool())
  val br        = Output(Bool())

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
                   List(N, OP1_X  ,  OP2_X  , ALU_X    , N,   N,     N,    N,      MCMD_X ,  MT_X  ),
      Array(      /* val  | op1   |   op2     |  ALU   | JAL | JALR | BR | MEM_V | MEM_CMD | MEM_T */
                  /* inst |  sel  |    sel    |   fcn  |     |      |    |       |         |       */
        LW      -> List(Y, OP1_RS1, OP2_IMI , ALU_ADD  , N,   N,     N,    Y,      MCMD_RD,  MT_W  ),
        LB      -> List(Y, OP1_RS1, OP2_IMI , ALU_ADD  , N,   N,     N,    Y,      MCMD_RD,  MT_B  ),
        LBU     -> List(Y, OP1_RS1, OP2_IMI , ALU_ADD  , N,   N,     N,    Y,      MCMD_RD,  MT_BU ),
        LH      -> List(Y, OP1_RS1, OP2_IMI , ALU_ADD  , N,   N,     N,    Y,      MCMD_RD,  MT_H  ),
        LHU     -> List(Y, OP1_RS1, OP2_IMI , ALU_ADD  , N,   N,     N,    Y,      MCMD_RD,  MT_HU ),
        SW      -> List(Y, OP1_RS1, OP2_IMS , ALU_ADD  , N,   N,     N,    Y,      MCMD_WR,  MT_W  ),
        SB      -> List(Y, OP1_RS1, OP2_IMS , ALU_ADD  , N,   N,     N,    Y,      MCMD_WR,  MT_B  ),
        SH      -> List(Y, OP1_RS1, OP2_IMS , ALU_ADD  , N,   N,     N,    Y,      MCMD_WR,  MT_H  ),

        AUIPC   -> List(Y, OP1_IMU, OP2_PC  , ALU_ADD  , N,   N,     N,    N,      MCMD_X,   MT_X  ),
        LUI     -> List(Y, OP1_IMU, OP2_X   , ALU_COPY1, N,   N,     N,    N,      MCMD_X,   MT_X  ),

        ADDI    -> List(Y, OP1_RS1, OP2_IMI , ALU_ADD  , N,   N,     N,    N,      MCMD_X,   MT_X  ),
        ANDI    -> List(Y, OP1_RS1, OP2_IMI , ALU_AND  , N,   N,     N,    N,      MCMD_X,   MT_X  ),
        ORI     -> List(Y, OP1_RS1, OP2_IMI , ALU_OR   , N,   N,     N,    N,      MCMD_X,   MT_X  ),
        XORI    -> List(Y, OP1_RS1, OP2_IMI , ALU_XOR  , N,   N,     N,    N,      MCMD_X,   MT_X  ),
        SLTI    -> List(Y, OP1_RS1, OP2_IMI , ALU_SLT  , N,   N,     N,    N,      MCMD_X,   MT_X  ),
        SLTIU   -> List(Y, OP1_RS1, OP2_IMI , ALU_SLTU , N,   N,     N,    N,      MCMD_X,   MT_X  ),
        SLLI    -> List(Y, OP1_RS1, OP2_IMI , ALU_SLL  , N,   N,     N,    N,      MCMD_X,   MT_X  ),
        SRAI    -> List(Y, OP1_RS1, OP2_IMI , ALU_SRA  , N,   N,     N,    N,      MCMD_X,   MT_X  ),
        SRLI    -> List(Y, OP1_RS1, OP2_IMI , ALU_SRL  , N,   N,     N,    N,      MCMD_X,   MT_X  ),

        SLL     -> List(Y, OP1_RS1, OP2_RS2 , ALU_SLL  , N,   N,     N,    N,      MCMD_X,   MT_X  ),
        ADD     -> List(Y, OP1_RS1, OP2_RS2 , ALU_ADD  , N,   N,     N,    N,      MCMD_X,   MT_X  ),
        SUB     -> List(Y, OP1_RS1, OP2_RS2 , ALU_SUB  , N,   N,     N,    N,      MCMD_X,   MT_X  ),
        SLT     -> List(Y, OP1_RS1, OP2_RS2 , ALU_SLT  , N,   N,     N,    N,      MCMD_X,   MT_X  ),
        SLTU    -> List(Y, OP1_RS1, OP2_RS2 , ALU_SLTU , N,   N,     N,    N,      MCMD_X,   MT_X  ),
        AND     -> List(Y, OP1_RS1, OP2_RS2 , ALU_AND  , N,   N,     N,    N,      MCMD_X,   MT_X  ),
        OR      -> List(Y, OP1_RS1, OP2_RS2 , ALU_OR   , N,   N,     N,    N,      MCMD_X,   MT_X  ),
        XOR     -> List(Y, OP1_RS1, OP2_RS2 , ALU_XOR  , N,   N,     N,    N,      MCMD_X,   MT_X  ),
        SRA     -> List(Y, OP1_RS1, OP2_RS2 , ALU_SRA  , N,   N,     N,    N,      MCMD_X,   MT_X  ),
        SRL     -> List(Y, OP1_RS1, OP2_RS2 , ALU_SRL  , N,   N,     N,    N,      MCMD_X,   MT_X  ),

        ADDIW   -> List(Y, OP1_RS1, OP2_IMI , ALU_ADD  , N,   N,     N,    N,      MCMD_X,   MT_X  ),
        SLLIW   -> List(Y, OP1_RS1, OP2_IMI , ALU_SLL  , N,   N,     N,    N,      MCMD_X,   MT_X  ),
        SRLIW   -> List(Y, OP1_RS1, OP2_IMI , ALU_SRL  , N,   N,     N,    N,      MCMD_X,   MT_X  ),
        SRAIW   -> List(Y, OP1_RS1, OP2_IMI , ALU_SRA  , N,   N,     N,    N,      MCMD_X,   MT_X  ),
        ADDW    -> List(Y, OP1_RS1, OP2_RS2 , ALU_ADD  , N,   N,     N,    N,      MCMD_X,   MT_X  ),
        SUBW    -> List(Y, OP1_RS1, OP2_RS2 , ALU_SUB  , N,   N,     N,    N,      MCMD_X,   MT_X  ),
        SLLW    -> List(Y, OP1_RS1, OP2_RS2 , ALU_SLL  , N,   N,     N,    N,      MCMD_X,   MT_X  ),
        SRLW    -> List(Y, OP1_RS1, OP2_RS2 , ALU_SRL  , N,   N,     N,    N,      MCMD_X,   MT_X  ),
        SRAW    -> List(Y, OP1_RS1, OP2_RS2 , ALU_SRA  , N,   N,     N,    N,      MCMD_X,   MT_X  ),

        JAL     -> List(Y, OP1_X  , OP2_X   , ALU_X    , Y,   N,     N,    N,      MCMD_X,   MT_X  ),
        JALR    -> List(Y, OP1_RS1, OP2_IMI , ALU_X    , N,   Y,     N,    N,      MCMD_X,   MT_X  ),
        BEQ     -> List(Y, OP1_RS1, OP2_RS2 , ALU_SEQ  , N,   N,     Y,    N,      MCMD_X,   MT_X  ),
        BNE     -> List(Y, OP1_RS1, OP2_RS2 , ALU_SNE  , N,   N,     Y,    N,      MCMD_X,   MT_X  ),
        BGE     -> List(Y, OP1_RS1, OP2_RS2 , ALU_SGE  , N,   N,     Y,    N,      MCMD_X,   MT_X  ),
        BGEU    -> List(Y, OP1_RS1, OP2_RS2 , ALU_SGEU , N,   N,     Y,    N,      MCMD_X,   MT_X  ),
        BLT     -> List(Y, OP1_RS1, OP2_RS2 , ALU_SLT  , N,   N,     Y,    N,      MCMD_X,   MT_X  ),
        BLTU    -> List(Y, OP1_RS1, OP2_RS2 , ALU_SLTU , N,   N,     Y,    N,      MCMD_X,   MT_X  ),

        CSRRWI  -> List(Y, OP1_IMZ, OP2_X   , ALU_COPY1, N,   N,     N,    N,      MCMD_X,   MT_X  ),
        CSRRSI  -> List(Y, OP1_IMZ, OP2_X   , ALU_COPY1, N,   N,     N,    N,      MCMD_X,   MT_X  ),
        CSRRCI  -> List(Y, OP1_IMZ, OP2_X   , ALU_COPY1, N,   N,     N,    N,      MCMD_X,   MT_X  ),
        CSRRW   -> List(Y, OP1_RS1, OP2_X   , ALU_COPY1, N,   N,     N,    N,      MCMD_X,   MT_X  ),
        CSRRS   -> List(Y, OP1_RS1, OP2_X   , ALU_COPY1, N,   N,     N,    N,      MCMD_X,   MT_X  ),
        CSRRC   -> List(Y, OP1_RS1, OP2_X   , ALU_COPY1, N,   N,     N,    N,      MCMD_X,   MT_X  ),

        ECALL   -> List(Y, OP1_X  , OP2_X  ,  ALU_X    , N,   N,     N,    N,      MCMD_X,   MT_X  ),
        MRET    -> List(Y, OP1_X  , OP2_X  ,  ALU_X    , N,   N,     N,    N,      MCMD_X,   MT_X  ),
        DRET    -> List(Y, OP1_X  , OP2_X  ,  ALU_X    , N,   N,     N,    N,      MCMD_X,   MT_X  ),
        EBREAK  -> List(Y, OP1_X  , OP2_X  ,  ALU_X    , N,   N,     N,    N,      MCMD_X,   MT_X  ),
        WFI     -> List(Y, OP1_X  , OP2_X  ,  ALU_X    , N,   N,     N,    N,      MCMD_X,   MT_X  ), // implemented as a NOP

        FENCE_I -> List(Y, OP1_X  , OP2_X  ,  ALU_X    , N,   N,     N,    N,      MCMD_X,   MT_X  ),
        FENCE   -> List(Y, OP1_X  , OP2_X  ,  ALU_X    , N,   N,     N,    N,      MCMD_X,   MT_X  )
          // we are already sequentially consistent, so no need to honor the fence instruction
      ))

  io.ctl.exe_kill   := false.B    // squash EX stage (exception/mret occurred)
  io.ctl.pc_sel     := 0.U
  io.ctl.brjmp_sel  := false.B
  io.ctl.op1_sel    := csignals(1)
  io.ctl.op2_sel    := csignals(2)
  io.ctl.alu_fun    := csignals(3)
  io.ctl.wb_sel     := 0.U
  io.ctl.rf_wen     := false.B
  io.ctl.bypassable := false.B     // instruction's result can be bypassed
  io.ctl.csr_cmd    := 0.U

  io.ctl.jal        := csignals(4)
  io.ctl.jalr       := csignals(5)
  io.ctl.br         := csignals(6)

  io.ctl.mem_v      := csignals(7)
  io.ctl.mem_cmd    := csignals(8)
  io.ctl.mem_type   := csignals(9)

  io.ctl.exception  := false.B

}
