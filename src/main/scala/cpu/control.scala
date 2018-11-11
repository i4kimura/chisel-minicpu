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
      Array(      /* val  | op1   |   op2     |  ALU   */
                  /* inst |  sel  |    sel    |   fcn  */
        LW      -> List(Y, OP1_RS1, OP2_IMI , ALU_ADD  ),
        LB      -> List(Y, OP1_RS1, OP2_IMI , ALU_ADD  ),
        LBU     -> List(Y, OP1_RS1, OP2_IMI , ALU_ADD  ),
        LH      -> List(Y, OP1_RS1, OP2_IMI , ALU_ADD  ),
        LHU     -> List(Y, OP1_RS1, OP2_IMI , ALU_ADD  ),
        SW      -> List(Y, OP1_RS1, OP2_IMS , ALU_ADD  ),
        SB      -> List(Y, OP1_RS1, OP2_IMS , ALU_ADD  ),
        SH      -> List(Y, OP1_RS1, OP2_IMS , ALU_ADD  ),

        AUIPC   -> List(Y, OP1_IMU, OP2_PC  , ALU_ADD  ),
        LUI     -> List(Y, OP1_IMU, OP2_X   , ALU_COPY1),

        ADDI    -> List(Y, OP1_RS1, OP2_IMI , ALU_ADD  ),
        ANDI    -> List(Y, OP1_RS1, OP2_IMI , ALU_AND  ),
        ORI     -> List(Y, OP1_RS1, OP2_IMI , ALU_OR   ),
        XORI    -> List(Y, OP1_RS1, OP2_IMI , ALU_XOR  ),
        SLTI    -> List(Y, OP1_RS1, OP2_IMI , ALU_SLT  ),
        SLTIU   -> List(Y, OP1_RS1, OP2_IMI , ALU_SLTU ),
        SLLI    -> List(Y, OP1_RS1, OP2_IMI , ALU_SLL  ),
        SRAI    -> List(Y, OP1_RS1, OP2_IMI , ALU_SRA  ),
        SRLI    -> List(Y, OP1_RS1, OP2_IMI , ALU_SRL  ),

        SLL     -> List(Y, OP1_RS1, OP2_RS2 , ALU_SLL  ),
        ADD     -> List(Y, OP1_RS1, OP2_RS2 , ALU_ADD  ),
        SUB     -> List(Y, OP1_RS1, OP2_RS2 , ALU_SUB  ),
        SLT     -> List(Y, OP1_RS1, OP2_RS2 , ALU_SLT  ),
        SLTU    -> List(Y, OP1_RS1, OP2_RS2 , ALU_SLTU ),
        AND     -> List(Y, OP1_RS1, OP2_RS2 , ALU_AND  ),
        OR      -> List(Y, OP1_RS1, OP2_RS2 , ALU_OR   ),
        XOR     -> List(Y, OP1_RS1, OP2_RS2 , ALU_XOR  ),
        SRA     -> List(Y, OP1_RS1, OP2_RS2 , ALU_SRA  ),
        SRL     -> List(Y, OP1_RS1, OP2_RS2 , ALU_SRL  ),

        JAL     -> List(Y, OP1_X  , OP2_X   , ALU_X    ),
        JALR    -> List(Y, OP1_RS1, OP2_IMI , ALU_X    ),
        BEQ     -> List(Y, OP1_X  , OP2_X   , ALU_X    ),
        BNE     -> List(Y, OP1_X  , OP2_X   , ALU_X    ),
        BGE     -> List(Y, OP1_X  , OP2_X   , ALU_X    ),
        BGEU    -> List(Y, OP1_X  , OP2_X   , ALU_X    ),
        BLT     -> List(Y, OP1_X  , OP2_X   , ALU_X    ),
        BLTU    -> List(Y, OP1_X  , OP2_X   , ALU_X    ),

        CSRRWI  -> List(Y, OP1_IMZ, OP2_X   , ALU_COPY1),
        CSRRSI  -> List(Y, OP1_IMZ, OP2_X   , ALU_COPY1),
        CSRRCI  -> List(Y, OP1_IMZ, OP2_X   , ALU_COPY1),
        CSRRW   -> List(Y, OP1_RS1, OP2_X   , ALU_COPY1),
        CSRRS   -> List(Y, OP1_RS1, OP2_X   , ALU_COPY1),
        CSRRC   -> List(Y, OP1_RS1, OP2_X   , ALU_COPY1),

        ECALL   -> List(Y, OP1_X  , OP2_X  ,  ALU_X    ),
        MRET    -> List(Y, OP1_X  , OP2_X  ,  ALU_X    ),
        DRET    -> List(Y, OP1_X  , OP2_X  ,  ALU_X    ),
        EBREAK  -> List(Y, OP1_X  , OP2_X  ,  ALU_X    ),
        WFI     -> List(Y, OP1_X  , OP2_X  ,  ALU_X    ), // implemented as a NOP

        FENCE_I -> List(Y, OP1_X  , OP2_X  ,  ALU_X    ),
        FENCE   -> List(Y, OP1_X  , OP2_X  ,  ALU_X    )
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

  io.ctl.exception  := false.B

}
