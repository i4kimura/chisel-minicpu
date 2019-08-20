// Copyright 2018 - 2019 ETH Zurich and University of Bologna.
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 2.0 (the "License"); you may not use this file except in
// compliance with the License.  You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-2.0. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.
// Author: Florian Zaruba, ETH Zurich
// Date: 08.02.2018
// Migrated: Luis Vitorio Cargnini, IEEE
// Date: 09.06.2018

package cache_subsystem

import chisel3._
import chisel3.util._
import chisel3.Bool

import ariane_pkg._
import riscv_pkg._

// ------------------------------
// Instruction Scanner
// ------------------------------
class instr_scan extends Module {
  val io = IO(new Bundle {
    val instr_i      = Input(UInt(32.W))        // expect aligned instruction, compressed or not
    val rvi_return_o = Output(Bool())
    val rvi_call_o   = Output(Bool())
    val rvi_branch_o = Output(Bool())
    val rvi_jalr_o   = Output(Bool())
    val rvi_jump_o   = Output(Bool())
    val rvi_imm_o    = Output(UInt(64.W))
    val rvc_branch_o = Output(Bool())
    val rvc_jump_o   = Output(Bool())
    val rvc_jr_o     = Output(Bool())
    val rvc_return_o = Output(Bool())
    val rvc_jalr_o   = Output(Bool())
    val rvc_call_o   = Output(Bool())
    val rvc_imm_o    = Output(UInt(64.W))
  })

  val is_rvc = Wire(Bool())
  is_rvc := (io.instr_i(1,0) =/= 3.U(2.W))
  // check that rs1 is either x1 or x5 and that rs1 is not x1 or x5
  io.rvi_return_o := io.rvi_jalr_o & ((io.instr_i(11,7) === 1.U(5.W)) | io.instr_i(11,7) === 5.U(5.W)) &
                                     (io.instr_i(19,15) =/= io.instr_i(11,7))
  // Opocde is JAL(R) and destination register is either x1 or x5
  io.rvi_call_o   := (io.rvi_jalr_o | io.rvi_jump_o) & ((io.instr_i(11,7) === 1.U(5.W)) | io.instr_i(11,7) === 5.U(5.W))
    // differentiates between JAL and BRANCH opcode, JALR comes from BHT
  io.rvi_imm_o    := Mux(io.instr_i(3), uj_imm(io.instr_i), sb_imm(io.instr_i))
  io.rvi_branch_o := (io.instr_i(6,0) === OpcodeBranch)
  io.rvi_jalr_o   := (io.instr_i(6,0) === OpcodeJalr)
  io.rvi_jump_o   := (io.instr_i(6,0) === OpcodeJal)

  // opcode JAL
  io.rvc_jump_o   := (io.instr_i(15,13) === OpcodeC1J) & is_rvc & (io.instr_i(1,0) === OpcodeC1)
  // always links to register 0
  val is_jal_r = Wire(Bool())
  is_jal_r := (io.instr_i(15,13) === OpcodeC2JalrMvAdd) &
              (io.instr_i( 6, 2) === 0.U(5.W)) &
              (io.instr_i( 1, 0) === OpcodeC2) &
              is_rvc
  io.rvc_jr_o := is_jal_r & ~io.instr_i(12)
  // always links to register 1 e.g., it is a jump
  io.rvc_jalr_o  := is_jal_r & io.instr_i(12)
  io.rvc_call_o  := io.rvc_jalr_o

  io.rvc_branch_o := ((io.instr_i(15,13) === OpcodeC1Beqz) | (io.instr_i(15,13) === OpcodeC1Bnez)) &
                      (io.instr_i( 1, 0) === OpcodeC1) &
                       is_rvc
  // check that rs1 is x1 or x5
  io.rvc_return_o := ((io.instr_i(11,7) === 1.U(5.W)) | (io.instr_i(11,7) === 5.U(5.W))) & io.rvc_jr_o

  // differentiates between JAL and BRANCH opcode, JALR comes from BHT
  io.rvc_imm_o := Mux(io.instr_i(14), Cat(VecInit(Seq.fill(56)(io.instr_i(12))).asUInt, io.instr_i(6,5), io.instr_i(2),    io.instr_i(11,10), io.instr_i(4,3), 0.U(1.W)),
                                      Cat(VecInit(Seq.fill(53)(io.instr_i(12))).asUInt, io.instr_i(  8), io.instr_i(10,9), io.instr_i(    6), io.instr_i(7), io.instr_i(2), io.instr_i(11), io.instr_i(5,3), 0.U(1.W)))
}


object instr_scan extends App {
  chisel3.Driver.execute(args, () => new instr_scan())
}
