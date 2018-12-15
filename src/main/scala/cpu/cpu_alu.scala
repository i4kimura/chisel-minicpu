package cpu

import chisel3._
import chisel3.util._
import chisel3.Bool

import DecodeConsts._

class Alu [Conf <: RVConfig](conf: Conf) extends Module {
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

  val w_mul_xlen2 = Wire(SInt((conf.xlen*2).W))
  w_mul_xlen2 := 0.S
  switch (io.func) {
    is (ALU_MUL   ) { w_mul_xlen2 := (io.op0.asSInt * io.op1.asSInt).asSInt }
    is (ALU_MULH  ) { w_mul_xlen2 := (io.op0.asSInt * io.op1.asSInt).asSInt }
    is (ALU_MULHSU) { w_mul_xlen2 := (io.op0.asSInt * io.op1.asUInt).asSInt }
    is (ALU_MULHU ) { w_mul_xlen2 := (io.op0.asUInt * io.op1.asUInt).asSInt }
  }

  val w_mul_xlen = Wire(SInt((conf.xlen).W))
  w_mul_xlen := io.op0(31, 0).asSInt * io.op1(31, 0).asSInt

  val w_res = Wire(SInt(conf.xlen.W))
  w_res := 0.S
  switch (io.func) {
    is (ALU_ADD   ) { w_res := io.op0 + io.op1                               }
    is (ALU_SUB   ) { w_res := io.op0 - io.op1                               }
    is (ALU_SLL   ) { w_res := (io.op0.asUInt << io.op1(5,0).asUInt).asSInt  }
    is (ALU_SRL   ) { w_res := (io.op0.asUInt >> io.op1(5,0).asUInt).asSInt  }
    is (ALU_SRA   ) { w_res := (io.op0 >> io.op1(5,0).asUInt).asSInt         }
    is (ALU_AND   ) { w_res := io.op0 & io.op1                               }
    is (ALU_OR    ) { w_res := io.op0 | io.op1                               }
    is (ALU_XOR   ) { w_res := io.op0 ^ io.op1                               }
    is (ALU_SLT   ) { w_res := Mux(io.op0        <  io.op1,        1.S, 0.S) }
    is (ALU_SLTU  ) { w_res := Mux(io.op0.asUInt <  io.op1.asUInt, 1.S, 0.S) }
    is (ALU_SNE   ) { w_res := Mux(io.op0 =/= io.op1, 1.S, 0.S)              }
    is (ALU_SEQ   ) { w_res := Mux(io.op0 === io.op1, 1.S, 0.S)              }
    is (ALU_SGE   ) { w_res := Mux(io.op0        >= io.op1,        1.S, 0.S) }
    is (ALU_SGEU  ) { w_res := Mux(io.op0.asUInt >= io.op1.asUInt, 1.S, 0.S) }
    is (ALU_COPY1 ) { w_res := io.op0                                        }
    is (ALU_COPY2 ) { w_res := io.op1                                        }
    is (ALU_ADDW  ) { w_res := (io.op0(31, 0) + io.op1(31, 0)).asSInt        }
    is (ALU_SUBW  ) { w_res := (io.op0(31, 0) - io.op1(31, 0)).asSInt               }
    is (ALU_SLLW  ) { w_res := ((io.op0(31, 0).asSInt << io.op1(4,0).asUInt)(31, 0)).asSInt  }
    is (ALU_SRLW  ) { w_res := ((io.op0(31, 0).asUInt >> io.op1(4,0).asUInt)(31, 0)).asSInt  }
    is (ALU_SRAW  ) { w_res := ((io.op0(31, 0).asSInt >> io.op1(4,0).asUInt)(31, 0)).asSInt  }
    is (ALU_MUL   ) { w_res := w_mul_xlen2((conf.xlen-1),   0).asSInt         }
    is (ALU_MULH  ) { w_res := w_mul_xlen2((conf.xlen*2-1), conf.xlen).asSInt }
    is (ALU_MULHSU) { w_res := w_mul_xlen2((conf.xlen*2-1), conf.xlen).asSInt }
    is (ALU_MULHU ) { w_res := w_mul_xlen2((conf.xlen*2-1), conf.xlen).asSInt }
    is (ALU_MULW  ) { w_res := w_mul_xlen(31, 0).asSInt                       }
  }

  val r_res = Reg(SInt(conf.xlen.W))
  r_res := w_res
  io.res := w_res
}
