// Copyright 2018 ETH Zurich and University of Bologna.
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the "License"); you may not use this file except in
// compliance with the License.  You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.
//
//
// Author:         Florian Zaruba - zarubaf@iis.ee.ethz.ch
// Engineer:       Sven Stucki - svstucki@student.ethz.ch
//
// Design Name:    Compressed instruction decoder
// Project Name:   zero-riscy
// Language:       SystemVerilog
//
// Description:    Decodes RISC-V compressed instructions into their RV32
//                 equivalent. This module is fully combinatorial.

package ariane

import chisel3._
import chisel3.util._
import chisel3.Bool

import riscv_pkg._

class compressed_decoder extends Module
{
  val io = IO(new Bundle {
    val instr_i = Input(UInt(32.W))
    val instr_o = Output(UInt(32.W))
    val illegal_instr_o = Output(Bool())
    val is_compressed_o = Output(Bool())
  })

  // -------------------
  // Compressed Decoder
  // -------------------
  io.illegal_instr_o := false.B
  io.instr_o         := 0.U
  io.is_compressed_o := true.B
  io.instr_o         := io.instr_i

  // Default
  io.is_compressed_o := false.B

  // I: |    imm[11:0]    | rs1 | funct3 |    rd    | opcode |
  // S: | imm[11:5] | rs2 | rs1 | funct3 | imm[4:0] | opcode |
  switch (io.instr_i(1,0)) {
    // C0
    is(OpcodeC0) {
      io.illegal_instr_o := true.B // Default
      switch (io.instr_i(15,13)) {
        is(OpcodeC0Addi4spn) {
          // c.addi4spn -> addi rd', x2, imm
          io.instr_o := Cat(0.U(2.W), io.instr_i(10,7), io.instr_i(12,11), io.instr_i(5), io.instr_i(6), 0.U(2.W), 2.U(5.W), 0.U(3.W), 1.U(2.W), io.instr_i(4,2), OpcodeOpImm)
          when (io.instr_i(12,5) === 0.U(8.W)) { io.illegal_instr_o := true.B }
        }

        is(OpcodeC0Fld) {
          // c.fld -> fld rd', imm(rs1')
          // CLD: | funct3 | imm[5:3] | rs1' | imm[7:6] | rd' | C0 |
          io.instr_o := Cat(0.U(4.W), io.instr_i(6,5), io.instr_i(12,10), 0.U(3.W), 1.U(2.W), io.instr_i(9,7), 3.U(3.W), 1.U(2.W), io.instr_i(4,2), OpcodeLoadFp)
        }

        is(OpcodeC0Lw) {
          // c.lw -> lw rd', imm(rs1')
          io.instr_o := Cat(0.U(5.W), io.instr_i(5), io.instr_i(12,10), io.instr_i(6), 0.U(2.W), 1.U(2.W), io.instr_i(9,7), 2.U(3.W), 1.U(2.W), io.instr_i(4,2), OpcodeLoad)
        }

        is(OpcodeC0Ld) {
          // c.ld -> ld rd', imm(rs1')
          // CLD: | funct3 | imm[5:3] | rs1' | imm[7:6] | rd' | C0 |
          io.instr_o := Cat(0.U(4.W), io.instr_i(6,5), io.instr_i(12,10), 0.U(3.W), 1.U(2.W), io.instr_i(9,7), 3.U(3.W), 1.U(2.W), io.instr_i(4,2), OpcodeLoad)
        }

        is(OpcodeC0Fsd) {
          // c.fsd -> fsd rs2', imm(rs1')
          io.instr_o := Cat(0.U(4.W), io.instr_i(6,5), io.instr_i(12), 1.U(2.W), io.instr_i(4,2), 1.U(2.W), io.instr_i(9,7), 3.U(3.W), io.instr_i(11,10), 0.U(3.W), OpcodeStoreFp)
        }

        is(OpcodeC0Sw) {
          // c.sw -> sw rs2', imm(rs1')
          io.instr_o := Cat(0.U(5.W), io.instr_i(5), io.instr_i(12), 1.U(2.W), io.instr_i(4,2), 1.U(2.W), io.instr_i(9,7), 2.U(3.W), io.instr_i(11,10), io.instr_i(6), 0.U(2.W), OpcodeStore)
        }

        is(OpcodeC0Sd) {
          // c.sd -> sd rs2', imm(rs1')
          io.instr_o := Cat(0.U(4.W), io.instr_i(6,5), io.instr_i(12), 1.U(2.W), io.instr_i(4,2), 1.U(2.W), io.instr_i(9,7), 3.U(3.W), io.instr_i(11,10), 0.U(3.W), OpcodeStore)
        }
      }
    }

    // C1
    is(OpcodeC1) {
      switch (io.instr_i(15,13)) {
        is(OpcodeC1Addi) {
          // c.addi -> addi rd, rd, nzimm
          // c.nop -> addi 0, 0, 0
          io.instr_o := Cat(VecInit(Seq.fill(6)(io.instr_i(12))).asUInt, io.instr_i(12), io.instr_i(6,2), io.instr_i(11,7), 0.U(3.W), io.instr_i(11,7), OpcodeOpImm)
        }

        // c.addiw -> addiw rd, rd, nzimm for RV64
        is(OpcodeC1Addiw) {
          when (io.instr_i(11,7) =/= 0.U(5.W)) { // only valid if the destination is not r0
            io.instr_o := Cat(VecInit(Seq.fill(6)(io.instr_i(12))).asUInt, io.instr_i(12), io.instr_i(6,2), io.instr_i(11,7), 0.U(3.W), io.instr_i(11,7), OpcodeOpImm32)
          } .otherwise {
            io.illegal_instr_o := true.B
          }
        }

        is(OpcodeC1Li) {
          // c.li -> addi rd, x0, nzimm
          io.instr_o := Cat(VecInit(Seq.fill(6)(io.instr_i(12))).asUInt, io.instr_i(12), io.instr_i(6,2), 0.U(5.W), 0.U(3.W), io.instr_i(11,7), OpcodeOpImm)
        }

        is(OpcodeC1LuiAddi16sp) {
          // c.lui -> lui rd, imm
          io.instr_o := Cat(VecInit(Seq.fill(15)(io.instr_i(12))).asUInt, io.instr_i(6,2), io.instr_i(11,7), OpcodeLui)

          when (io.instr_i(11,7) === 2.U(5.W)) {
            // c.addi16sp -> addi x2, x2, nzimm
            io.instr_o := Cat(VecInit(Seq.fill(3)(io.instr_i(12))).asUInt, io.instr_i(4,3), io.instr_i(5), io.instr_i(2), io.instr_i(6), 0.U(4.W), 2.U(5.W), 0.U(3.W), 2.U(5.W), OpcodeOpImm)
          }

          when (Cat(io.instr_i(12), io.instr_i(6,2)) === 0.U(6.W)) {
            io.illegal_instr_o := true.B
          }
        }

        is(OpcodeC1MiscAlu) {
          switch (io.instr_i(11,10)) {
            is(0.U(2.W), 1.U(2.W)) {
              // 00: c.srli -> srli rd, rd, shamt
              // 01: c.srai -> srai rd, rd, shamt
              io.instr_o := Cat(false.B, io.instr_i(10), 0.U(4.W), io.instr_i(12), io.instr_i(6,2), 1.U(2.W), io.instr_i(9,7), 5.U(3.W), 1.U(2.W), io.instr_i(9,7), OpcodeOpImm)
            }
            is(2.U(2.W)) {
              // c.andi -> andi rd, rd, imm
              io.instr_o := Cat(VecInit(Seq.fill(6)(io.instr_i(12))).asUInt, io.instr_i(12), io.instr_i(6,2), 1.U(2.W), io.instr_i(9,7), 7.U(3.W), 1.U(2.W), io.instr_i(9,7), OpcodeOpImm)
            }
            is(3.U(2.W)) {
              switch (Cat(io.instr_i(12), io.instr_i(6,5))) {
                is(0.U(3.W)) {
                  // c.sub -> sub rd', rd', rs2'
                  io.instr_o := Cat(1.U(2.W), 0.U(5.W), 1.U(2.W), io.instr_i(4,2), 1.U(2.W), io.instr_i(9,7), 0.U(3.W), 1.U(2.W), io.instr_i(9,7), OpcodeOp)
                }
                is(1.U(3.W)) {
                  // c.xor -> xor rd', rd', rs2'
                  io.instr_o := Cat(0.U(7.W), 1.U(2.W), io.instr_i(4,2), 1.U(2.W), io.instr_i(9,7), 4.U(3.W), 1.U(2.W), io.instr_i(9,7), OpcodeOp)
                }

                is(2.U(3.W)) {
                  // c.or  -> or  rd', rd', rs2'
                  io.instr_o := Cat(0.U(7.W), 1.U(2.W), io.instr_i(4,2), 1.U(2.W), io.instr_i(9,7), 6.U(3.W), 1.U(2.W), io.instr_i(9,7), OpcodeOp)
                }

                is(3.U(3.W)) {
                  // c.and -> and rd', rd', rs2'
                  io.instr_o := Cat(0.U(7.W), 1.U(2.W), io.instr_i(4,2), 1.U(2.W), io.instr_i(9,7), 7.U(3.W), 1.U(2.W), io.instr_i(9,7), OpcodeOp)
                }

                is(4.U(3.W)) {
                  // c.subw -> subw rd', rd', rs2'
                  io.instr_o := Cat(1.U(2.W), 0.U(5.W), 1.U(2.W), io.instr_i(4,2), 1.U(2.W), io.instr_i(9,7), 0.U(3.W), 1.U(2.W), io.instr_i(9,7), OpcodeOp32)
                }
                is(5.U(3.W)) {
                  // c.addw -> addw rd', rd', rs2'
                  io.instr_o := Cat(0.U(2.W), 0.U(5.W), 1.U(2.W), io.instr_i(4,2), 1.U(2.W), io.instr_i(9,7), 0.U(3.W), 1.U(2.W), io.instr_i(9,7), OpcodeOp32)
                }

                is(6.U(3.W), 7.U(3.W)) {
                  // 100: c.subw
                  // 101: c.addw
                  io.illegal_instr_o := true.B
                  io.instr_o := Cat(0.U(16.W), io.instr_i)
                }
              }
            }
          }
        }

        is(OpcodeC1J) {
          // 101: c.j   -> jal x0, imm
          io.instr_o := Cat(io.instr_i(12), io.instr_i(8), io.instr_i(10,9), io.instr_i(6), io.instr_i(7), io.instr_i(2), io.instr_i(11), io.instr_i(5,3), VecInit(Seq.fill(9)(io.instr_i(12))).asUInt, 0.U(4.W), ~io.instr_i(15), OpcodeJal)
        }

        is(OpcodeC1Beqz, OpcodeC1Bnez) {
          // 0: c.beqz -> beq rs1', x0, imm
          // 1: c.bnez -> bne rs1', x0, imm
          io.instr_o := Cat(VecInit(Seq.fill(4)(io.instr_i(12))).asUInt, io.instr_i(6,5), io.instr_i(2), 0.U(5.W), 1.U(2.W), io.instr_i(9,7), 0.U(2.W), io.instr_i(13), io.instr_i(11,10), io.instr_i(4,3), io.instr_i(12), OpcodeBranch)
        }
      }
    }

    // C2
    is(OpcodeC2) {
      io.illegal_instr_o := true.B  // Default

      switch (io.instr_i(15,13)) {
        is(OpcodeC2Slli) {
          // c.slli -> slli rd, rd, shamt
          io.instr_o := Cat(0.U(6.W), io.instr_i(12), io.instr_i(6,2), io.instr_i(11,7), 1.U(3.W), io.instr_i(11,7), OpcodeOpImm)
        }

        is(OpcodeC2Fldsp) {
          // c.fldsp -> fld rd, imm(x2)
          io.instr_o := Cat(0.U(3.W), io.instr_i(4,2), io.instr_i(12), io.instr_i(6,5), 0.U(3.W), 2.U(5.W), 3.U(3.W), io.instr_i(11,7), OpcodeLoadFp)
          when (io.instr_i(11,7) === 0.U(5.W)) { io.illegal_instr_o := true.B }
        }

        is(OpcodeC2Lwsp) {
          // c.lwsp -> lw rd, imm(x2)
          io.instr_o := Cat(0.U(4.W), io.instr_i(3,2), io.instr_i(12), io.instr_i(6,4), 0.U(2.W), 2.U(5.W), 2.U(3.W), io.instr_i(11,7), OpcodeLoad)
          when (io.instr_i(11,7) === 0.U(5.W)) { io.illegal_instr_o := true.B }
        }

        is(OpcodeC2Ldsp) {
          // c.ldsp -> ld rd, imm(x2)
          io.instr_o := Cat(0.U(3.W), io.instr_i(4,2), io.instr_i(12), io.instr_i(6,5), 0.U(3.W), 2.U(5.W), 3.U(3.W), io.instr_i(11,7), OpcodeLoad)
          when (io.instr_i(11,7) === 0.U(5.W)) { io.illegal_instr_o := true.B }
        }

        is(OpcodeC2JalrMvAdd) {
          when (io.instr_i(12) === false.B) {
            // c.mv -> add rd/rs1, x0, rs2
            io.instr_o := Cat(0.U(7.W), io.instr_i(6,2), 0.U(5.W), 0.U(3.W), io.instr_i(11,7), OpcodeOp)

            when (io.instr_i(6,2) === 0.U(5.W)) {
              // c.jr -> jalr x0, rd/rs1, 0
              io.instr_o := Cat(0.U(12.W), io.instr_i(11,7), 0.U(3.W), 0.U(5.W), OpcodeJalr)
              // rs1 =/= 0
              io.illegal_instr_o := Mux(io.instr_i(11,7) =/= 0.U, false.B, true.B)
            }
          } .otherwise {
            // c.add -> add rd, rd, rs2
            io.instr_o := Cat(0.U(7.W), io.instr_i(6,2), io.instr_i(11,7), 0.U(3.W), io.instr_i(11,7), OpcodeOp)

            when (io.instr_i(11,7) === 0.U(5.W) && io.instr_i(6,2) === 0.U(5.W)) {
              // c.ebreak -> ebreak
              io.instr_o := 0x00100073.U(32.W)
            } .elsewhen (io.instr_i(11,7) =/= 0.U(5.W) && io.instr_i(6,2) === 0.U(5.W)) {
              // c.jalr -> jalr x1, rs1, 0
              io.instr_o := Cat(0.U(12.W), io.instr_i(11,7), 0.U(3.W), 1.U(5.W), OpcodeJalr)
            }
          }
        }

        is(OpcodeC2Fsdsp) {
          // c.fsdsp -> fsd rs2, imm(x2)
          io.instr_o := Cat(0.U(3.W), io.instr_i(9,7), io.instr_i(12), io.instr_i(6,2), 2.U(5.W), 3.U(3.W), io.instr_i(11,10), 0.U(3.W), OpcodeStoreFp)
        }

        is(OpcodeC2Swsp) {
          // c.swsp -> sw rs2, imm(x2)
          io.instr_o := Cat(0.U(4.W), io.instr_i(8,7), io.instr_i(12), io.instr_i(6,2), 2.U(5.W), 2.U(3.W), io.instr_i(11,9), 0.U(2.W), OpcodeStore)
        }

        is(OpcodeC2Sdsp) {
          // c.sdsp -> sd rs2, imm(x2)
          io.instr_o := Cat(0.U(3.W), io.instr_i(9,7), io.instr_i(12), io.instr_i(6,2), 2.U(5.W), 3.U(3.W), io.instr_i(11,10), 0.U(3.W), OpcodeStore)
        }
      }
    }
    // normal instruction

  }

  // Check if the instruction was illegal, if it was then output the offending instruction (zero-extended)
  when (io.illegal_instr_o && io.is_compressed_o) {
    io.instr_o := io.instr_i
  }
}


object compressed_decoder extends App {
  chisel3.Driver.emitVerilog(new compressed_decoder())
}
