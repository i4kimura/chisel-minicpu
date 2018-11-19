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
                   List(N, OP1_X  ,  OP2_X  , ALU_X    ),
      Array(      /* val  | op1   |   op2     |  ALU   | JAL | JALR | BR |*/
                  /* inst |  sel  |    sel    |   fcn  |                  */
        LW      -> List(Y, OP1_RS1, OP2_IMI , ALU_ADD  , N,   N,     N),
        LB      -> List(Y, OP1_RS1, OP2_IMI , ALU_ADD  , N,   N,     N),
        LBU     -> List(Y, OP1_RS1, OP2_IMI , ALU_ADD  , N,   N,     N),
        LH      -> List(Y, OP1_RS1, OP2_IMI , ALU_ADD  , N,   N,     N),
        LHU     -> List(Y, OP1_RS1, OP2_IMI , ALU_ADD  , N,   N,     N),
        SW      -> List(Y, OP1_RS1, OP2_IMS , ALU_ADD  , N,   N,     N),
        SB      -> List(Y, OP1_RS1, OP2_IMS , ALU_ADD  , N,   N,     N),
        SH      -> List(Y, OP1_RS1, OP2_IMS , ALU_ADD  , N,   N,     N),

        AUIPC   -> List(Y, OP1_IMU, OP2_PC  , ALU_ADD  , N,   N,     N),
        LUI     -> List(Y, OP1_IMU, OP2_X   , ALU_COPY1, N,   N,     N),

        ADDI    -> List(Y, OP1_RS1, OP2_IMI , ALU_ADD  , N,   N,     N),
        ANDI    -> List(Y, OP1_RS1, OP2_IMI , ALU_AND  , N,   N,     N),
        ORI     -> List(Y, OP1_RS1, OP2_IMI , ALU_OR   , N,   N,     N),
        XORI    -> List(Y, OP1_RS1, OP2_IMI , ALU_XOR  , N,   N,     N),
        SLTI    -> List(Y, OP1_RS1, OP2_IMI , ALU_SLT  , N,   N,     N),
        SLTIU   -> List(Y, OP1_RS1, OP2_IMI , ALU_SLTU , N,   N,     N),
        SLLI    -> List(Y, OP1_RS1, OP2_IMI , ALU_SLL  , N,   N,     N),
        SRAI    -> List(Y, OP1_RS1, OP2_IMI , ALU_SRA  , N,   N,     N),
        SRLI    -> List(Y, OP1_RS1, OP2_IMI , ALU_SRL  , N,   N,     N),

        SLL     -> List(Y, OP1_RS1, OP2_RS2 , ALU_SLL  , N,   N,     N),
        ADD     -> List(Y, OP1_RS1, OP2_RS2 , ALU_ADD  , N,   N,     N),
        SUB     -> List(Y, OP1_RS1, OP2_RS2 , ALU_SUB  , N,   N,     N),
        SLT     -> List(Y, OP1_RS1, OP2_RS2 , ALU_SLT  , N,   N,     N),
        SLTU    -> List(Y, OP1_RS1, OP2_RS2 , ALU_SLTU , N,   N,     N),
        AND     -> List(Y, OP1_RS1, OP2_RS2 , ALU_AND  , N,   N,     N),
        OR      -> List(Y, OP1_RS1, OP2_RS2 , ALU_OR   , N,   N,     N),
        XOR     -> List(Y, OP1_RS1, OP2_RS2 , ALU_XOR  , N,   N,     N),
        SRA     -> List(Y, OP1_RS1, OP2_RS2 , ALU_SRA  , N,   N,     N),
        SRL     -> List(Y, OP1_RS1, OP2_RS2 , ALU_SRL  , N,   N,     N),

        ADDIW   -> List(Y, OP1_RS1, OP2_IMI , ALU_ADD  , N,   N,     N),
        SLLIW   -> List(Y, OP1_RS1, OP2_IMI , ALU_SLL  , N,   N,     N),
        SRLIW   -> List(Y, OP1_RS1, OP2_IMI , ALU_SRL  , N,   N,     N),
        SRAIW   -> List(Y, OP1_RS1, OP2_IMI , ALU_SRA  , N,   N,     N),
        ADDW    -> List(Y, OP1_RS1, OP2_RS2 , ALU_ADD  , N,   N,     N),
        SUBW    -> List(Y, OP1_RS1, OP2_RS2 , ALU_SUB  , N,   N,     N),
        SLLW    -> List(Y, OP1_RS1, OP2_RS2 , ALU_SLL  , N,   N,     N),
        SRLW    -> List(Y, OP1_RS1, OP2_RS2 , ALU_SRL  , N,   N,     N),
        SRAW    -> List(Y, OP1_RS1, OP2_RS2 , ALU_SRA  , N,   N,     N),

        JAL     -> List(Y, OP1_X  , OP2_X   , ALU_X    , Y,   N,     N),
        JALR    -> List(Y, OP1_RS1, OP2_IMI , ALU_X    , N,   Y,     N),
        BEQ     -> List(Y, OP1_RS1, OP2_RS2 , ALU_SEQ  , N,   N,     Y),
        BNE     -> List(Y, OP1_RS1, OP2_RS2 , ALU_SNE  , N,   N,     Y),
        BGE     -> List(Y, OP1_RS1, OP2_RS2 , ALU_SGE  , N,   N,     Y),
        BGEU    -> List(Y, OP1_RS1, OP2_RS2 , ALU_SGEU , N,   N,     Y),
        BLT     -> List(Y, OP1_RS1, OP2_RS2 , ALU_SLT  , N,   N,     Y),
        BLTU    -> List(Y, OP1_RS1, OP2_RS2 , ALU_SLTU , N,   N,     Y),

        CSRRWI  -> List(Y, OP1_IMZ, OP2_X   , ALU_COPY1, N,   N,     N),
        CSRRSI  -> List(Y, OP1_IMZ, OP2_X   , ALU_COPY1, N,   N,     N),
        CSRRCI  -> List(Y, OP1_IMZ, OP2_X   , ALU_COPY1, N,   N,     N),
        CSRRW   -> List(Y, OP1_RS1, OP2_X   , ALU_COPY1, N,   N,     N),
        CSRRS   -> List(Y, OP1_RS1, OP2_X   , ALU_COPY1, N,   N,     N),
        CSRRC   -> List(Y, OP1_RS1, OP2_X   , ALU_COPY1, N,   N,     N),

        ECALL   -> List(Y, OP1_X  , OP2_X  ,  ALU_X    , N,   N,     N),
        MRET    -> List(Y, OP1_X  , OP2_X  ,  ALU_X    , N,   N,     N),
        DRET    -> List(Y, OP1_X  , OP2_X  ,  ALU_X    , N,   N,     N),
        EBREAK  -> List(Y, OP1_X  , OP2_X  ,  ALU_X    , N,   N,     N),
        WFI     -> List(Y, OP1_X  , OP2_X  ,  ALU_X    , N,   N,     N), // implemented as a NOP

        FENCE_I -> List(Y, OP1_X  , OP2_X  ,  ALU_X    , N,   N,     N),
        FENCE   -> List(Y, OP1_X  , OP2_X  ,  ALU_X    , N,   N,     N)
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

  io.ctl.exception  := false.B

}
