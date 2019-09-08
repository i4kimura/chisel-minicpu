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
// File:   issue_read_operands.sv
// Author: Florian Zaruba <zarubaf@ethz.ch>
// Date:   8.4.2017
//
// Copyright (C) 2017 ETH Zurich, University of Bologna
// All rights reserved.
//
// Description: Issues instruction from the scoreboard and fetches the operands
//              This also includes all the forwarding logic
//

package ariane

import chisel3._
import chisel3.util._
import chisel3.Bool

import riscv_pkg._
import ariane_pkg._
import ariane_fu_op._
import ariane_fu_t._
import ariane_if_id._


class decoder extends Module {

  //
  // xxx : Temporary Loading
  //
  // --------------------
  // Privilege Spec
  // --------------------
  type priv_lvl_t = UInt
  val PRIV_LVL_M:priv_lvl_t = 3.U(2.W)
  val PRIV_LVL_S:priv_lvl_t = 1.U(2.W)
  val PRIV_LVL_U:priv_lvl_t = 0.U(2.W)
  object priv_lvl_t extends UIntFactory {
    override def apply(): UInt = apply(2.W)
  }
  val ENABLE_CYCLE_COUNT = true
  val ENABLE_WFI         = true
  val ZERO_TVAL          = false

  // --------------------
  // Opcodes
  // --------------------
  // RV32/64G listings:
  // Quadrant 0
  val OpcodeLoad      = Integer.parseInt("0000011", 2).U
  val OpcodeLoadFp    = Integer.parseInt("0000111", 2).U
  val OpcodeCustom0   = Integer.parseInt("0001011", 2).U
  val OpcodeMiscMem   = Integer.parseInt("0001111", 2).U
  val OpcodeOpImm     = Integer.parseInt("0010011", 2).U
  val OpcodeAuipc     = Integer.parseInt("0010111", 2).U
  val OpcodeOpImm32   = Integer.parseInt("0011011", 2).U
  // Quadrant 1
  val OpcodeStore     = Integer.parseInt("0100011", 2).U
  val OpcodeStoreFp   = Integer.parseInt("0100111", 2).U
  val OpcodeCustom1   = Integer.parseInt("0101011", 2).U
  val OpcodeAmo       = Integer.parseInt("0101111", 2).U
  val OpcodeOp        = Integer.parseInt("0110011", 2).U
  val OpcodeLui       = Integer.parseInt("0110111", 2).U
  val OpcodeOp32      = Integer.parseInt("0111011", 2).U
  // Quadrant 2
  val OpcodeMadd      = Integer.parseInt("1000011", 2).U
  val OpcodeMsub      = Integer.parseInt("1000111", 2).U
  val OpcodeNmsub     = Integer.parseInt("1001011", 2).U
  val OpcodeNmadd     = Integer.parseInt("1001111", 2).U
  val OpcodeOpFp      = Integer.parseInt("1010011", 2).U
  val OpcodeRsrvd1    = Integer.parseInt("1010111", 2).U
  val OpcodeCustom2   = Integer.parseInt("1011011", 2).U
  // Quadrant 3
  val OpcodeBranch    = Integer.parseInt("1100011", 2).U
  val OpcodeJalr      = Integer.parseInt("1100111", 2).U
  val OpcodeRsrvd2    = Integer.parseInt("1101011", 2).U
  val OpcodeJal       = Integer.parseInt("1101111", 2).U
  val OpcodeSystem    = Integer.parseInt("1110011", 2).U
  val OpcodeRsrvd3    = Integer.parseInt("1110111", 2).U
  val OpcodeCustom3   = Integer.parseInt("1111011", 2).U

  // RV64C listings:
  // Quadrant 0
  val OpcodeC0             = Integer.parseInt("00", 2).U
  val OpcodeC0Addi4spn     = Integer.parseInt("000", 3).U
  val OpcodeC0Fld          = Integer.parseInt("001", 3).U
  val OpcodeC0Lw           = Integer.parseInt("010", 3).U
  val OpcodeC0Ld           = Integer.parseInt("011", 3).U
  val OpcodeC0Rsrvd        = Integer.parseInt("100", 3).U
  val OpcodeC0Fsd          = Integer.parseInt("101", 3).U
  val OpcodeC0Sw           = Integer.parseInt("110", 3).U
  val OpcodeC0Sd           = Integer.parseInt("111", 3).U
  // Quadrant 1
  val OpcodeC1             = Integer.parseInt("01", 2).U
  val OpcodeC1Addi         = Integer.parseInt("000", 2).U
  val OpcodeC1Addiw        = Integer.parseInt("001", 2).U
  val OpcodeC1Li           = Integer.parseInt("010", 2).U
  val OpcodeC1LuiAddi16sp  = Integer.parseInt("011", 2).U
  val OpcodeC1MiscAlu      = Integer.parseInt("100", 2).U
  val OpcodeC1J            = Integer.parseInt("101", 2).U
  val OpcodeC1Beqz         = Integer.parseInt("110", 2).U
  val OpcodeC1Bnez         = Integer.parseInt("111", 2).U
  // Quadrant 2
  val OpcodeC2             = Integer.parseInt("10", 2).U
  val OpcodeC2Slli         = Integer.parseInt("000", 2).U
  val OpcodeC2Fldsp        = Integer.parseInt("001", 2).U
  val OpcodeC2Lwsp         = Integer.parseInt("010", 2).U
  val OpcodeC2Ldsp         = Integer.parseInt("011", 2).U
  val OpcodeC2JalrMvAdd    = Integer.parseInt("100", 2).U
  val OpcodeC2Fsdsp        = Integer.parseInt("101", 2).U
  val OpcodeC2Swsp         = Integer.parseInt("110", 2).U
  val OpcodeC2Sdsp         = Integer.parseInt("111", 2).U

  //
  // xxx: Temporary Loading End
  //

  val io = IO(new Bundle {
    val debug_req_i            = Input(Bool())                     // external debug request
    val pc_i                   = Input(UInt(64.W))                 // PC from IF
    val is_compressed_i        = Input(Bool())                     // is a compressed instruction
    val compressed_instr_i     = Input(UInt(16.W))                 // compressed form of instruction
    val is_illegal_i           = Input(Bool())                     // illegal compressed instruction
    val instruction_i          = Input(UInt(32.W))                 // instruction from IF
    val branch_predict_i       = Input(new branchpredict_sbe_t())
    val ex_i                   = Input(new exception_t())          // if an exception occured in if
    val irq_i                  = Input(UInt(2.W))                  // external interrupt
    val irq_ctrl_i             = Input(new irq_ctrl_t())           // interrupt control and status information from CSRs

    // From CSR
    val priv_lvl_i             = Input(priv_lvl_t())               // current privilege level
    val debug_mode_i           = Input(Bool())                     // we are in debug mode
    val fs_i                   = Input(xs_t())                     // floating point extension status
    val frm_i                  = Input(UInt(3.W))                  // floating-point dynamic rounding mode
    val tvm_i                  = Input(Bool())                     // trap virtual memory
    val tw_i                   = Input(Bool())                     // timeout wait
    val tsr_i                  = Input(Bool())                     // trap sret
    val instruction_o          = Output(new scoreboard_entry_t())  // scoreboard entry to scoreboard
    val is_control_flow_instr_o= Output(Bool())                     // this instruction will change the control flow
  })

  val illegal_instr = Wire(Bool())
  // this instruction is an environment call (ecall), it is handled like an exception
  val ecall = Wire(Bool())
  // this instruction is a software break-point
  val ebreak = Wire(Bool())
  // this instruction needs floating-point rounding-mode verification
  val check_fprm = Wire(Bool())
  val instr = Wire(new instruction_t())
  instr := io.instruction_i

  // --------------------
  // Immediate select
  // --------------------
  val NOIMM = 0.U(3.W)
  val IIMM  = 1.U(3.W)
  val SIMM  = 2.U(3.W)
  val SBIMM = 3.U(3.W)
  val UIMM  = 4.U(3.W)
  val JIMM  = 5.U(3.W)
  val RS3   = 6.U(3.W)

  val imm_select = Wire(UInt(3.W))

  val imm_i_type  = Wire(UInt(64.W))
  val imm_s_type  = Wire(UInt(64.W))
  val imm_sb_type = Wire(UInt(64.W))
  val imm_u_type  = Wire(UInt(64.W))
  val imm_uj_type = Wire(UInt(64.W))
  val imm_bi_type = Wire(UInt(64.W))

  imm_select                     := NOIMM
  io.is_control_flow_instr_o     := false.B
  illegal_instr                  := false.B
  io.instruction_o.pc            := io.pc_i
  io.instruction_o.trans_id      := 0.U
  io.instruction_o.fu            := 0.U(4.W)
  // io.instruction_o.op            := ADD
  io.instruction_o.op            := 0.U
  io.instruction_o.rs1           := 0.U(6.W)
  io.instruction_o.rs2           := 0.U(6.W)
  io.instruction_o.rd            := 0.U(6.W)
  io.instruction_o.use_pc        := false.B
  io.instruction_o.is_compressed := io.is_compressed_i
  io.instruction_o.use_zimm      := false.B
  io.instruction_o.bp            := io.branch_predict_i
  ecall                          := false.B
  ebreak                         := false.B
  check_fprm                     := false.B

  when (~io.ex_i.valid) {
    illegal_instr := true.B
    switch (instr.asRType().opcode) {
      is(0x73.U) { // OpcodeSystem) {
        io.instruction_o.fu  := 6.U(4.W) // CSR
        io.instruction_o.rs1 := instr.asIType().rs1
        io.instruction_o.rd  := instr.asIType().rd

        switch (instr.asIType().funct3) {
          is(0.U(3.W)) {
            // check if the RD and and RS1 fields are zero, this may be reset for the SENCE.VMA instruction
            when (instr.asIType().rs1 =/= 0.U || instr.asIType().rd =/= 0.U) {
              // decode the immiediate field

              illegal_instr := true.B // Default
              switch (instr.asIType().imm) {
                // ECALL -> inject exception
                is(0.U(12.W)) { ecall := true.B }
                // EBREAK -> inject exception
                is(1.U(12.W)) { ebreak := true.B }
                // SRET
                is(Integer.parseInt("100000010", 2).U) {
                  io.instruction_o.op := SRET
                  // check privilege level, SRET can only be executed in S and M mode
                  // we'll just decode an illegal instruction if we are in the wrong privilege level
                  when (io.priv_lvl_i === PRIV_LVL_U) {
                    illegal_instr := true.B
                    //  do not change privilege level if this is an illegal instruction
                    io.instruction_o.op := ADD
                  }
                  // if we are in S-Mode and Trap SRET (tsr) is set -> trap on illegal instruction
                  when (io.priv_lvl_i === PRIV_LVL_S && io.tsr_i) {
                    illegal_instr := true.B
                    //  do not change privilege level if this is an illegal instruction
                    io.instruction_o.op := ADD
                  }
                }
                // MRET
                is(Integer.parseInt("1100000010", 2).U) {
                  io.instruction_o.op := MRET
                  // check privilege level, MRET can only be executed in M mode
                  // otherwise we decode an illegal instruction
                  when (io.priv_lvl_i === PRIV_LVL_U || io.priv_lvl_i === PRIV_LVL_S) {
                    illegal_instr := true.B
                  }
                }
                // DRET
                is(Integer.parseInt("11110110010", 2).U) {
                  io.instruction_o.op := DRET
                  // check that we are in debug mode when executing this instruction
                  illegal_instr := Mux(!io.debug_mode_i, true.B, false.B)
                }
                // WFI
                is(Integer.parseInt("100000101", 2).U) {
                  if (ENABLE_WFI) {
                    io.instruction_o.op := WFI
                  }
                  // if timeout wait is set, trap on an illegal instruction in S Mode
                  // (after 0 cycles timeout)
                  when (io.priv_lvl_i === PRIV_LVL_S && io.tw_i) {
                    illegal_instr := true.B
                    io.instruction_o.op := ADD
                  }
                  // we don't support U mode interrupts so WFI is illegal in this context
                  when (io.priv_lvl_i === PRIV_LVL_U) {
                    illegal_instr := true.B
                    io.instruction_o.op := ADD
                  }
                }
                // SFENCE.VMA
                is(Integer.parseInt("0001001", 2).U) {
                  when (instr.instr(31,25) === Integer.parseInt("1001", 2).U) {
                    // check privilege level, SFENCE.VMA can only be executed in M/S mode
                    // otherwise decode an illegal instruction
                    illegal_instr   := Mux((io.priv_lvl_i === PRIV_LVL_M || io.priv_lvl_i === PRIV_LVL_S), false.B, true.B)
                    io.instruction_o.op := SFENCE_VMA
                    // check TVM flag and intercept SFENCE.VMA call if necessary
                    when (io.priv_lvl_i === PRIV_LVL_S && io.tvm_i) {
                      illegal_instr := true.B
                    }
                  }
                }
              }
            }
          }
          // atomically swaps values in the CSR and integer register
          is(1.U(3.W)) { // CSRRW
            imm_select := IIMM
            io.instruction_o.op := CSR_WRITE
          }
          // atomically set values in the CSR and write back to rd
          is(2.U(3.W)) {// CSRRS
            imm_select := IIMM
            // this is just a read
            when (instr.asIType().rs1 === 0.U(5.W)) {
              io.instruction_o.op := CSR_READ
            } .otherwise {
              io.instruction_o.op := CSR_SET
            }
          }
          // atomically clear values in the CSR and write back to rd
          is(3.U(3.W)) { // CSRRC
            imm_select := IIMM
            // this is just a read
            when (instr.asIType().rs1 === 0.U(5.W)) {
              io.instruction_o.op := CSR_READ
            } .otherwise {
              io.instruction_o.op := CSR_CLEAR
            }
          }
          // use zimm and iimm
          is(5.U(3.W)) { // CSRRWI
            io.instruction_o.rs1 := instr.asIType().rs1
            imm_select := IIMM
            io.instruction_o.use_zimm := true.B
            io.instruction_o.op := CSR_WRITE
          }
          is(6.U(3.W)) { // CSRRSI
            io.instruction_o.rs1 := instr.asIType().rs1
            imm_select := IIMM
            io.instruction_o.use_zimm := true.B
            // this is just a read
            when (instr.asIType().rs1 === 0.U(5.W)) {
              io.instruction_o.op := CSR_READ
            } .otherwise {
              io.instruction_o.op := CSR_SET
            }
          }
          is(7.U(3.W)) { // CSRRCI
            io.instruction_o.rs1 := instr.asIType().rs1
            imm_select := IIMM
            io.instruction_o.use_zimm := true.B
            // this is just a read
            when (instr.asIType().rs1 === 0.U(5.W)) {
              io.instruction_o.op := CSR_READ
            } .otherwise {
              io.instruction_o.op := CSR_CLEAR
            }
          }
        }
      }
      // Memory ordering instructions
      is(OpcodeMiscMem) {
        io.instruction_o.fu  := CSR
        io.instruction_o.rs1 := 0.U(REG_ADDR_SIZE.W)
        io.instruction_o.rs2 := 0.U(REG_ADDR_SIZE.W)
        io.instruction_o.rd  := 0.U(REG_ADDR_SIZE.W)

        illegal_instr := true.B
        switch (instr.asSType().funct3) {
          // FENCE
          // Currently implemented as a whole DCache flush boldly ignoring other things
          is(0.U(3.W)) {
            io.instruction_o.op := FENCE
          }
          // FENCE.I
          is(1.U(3.W)) {
            when (instr.instr(31,20) =/= 0.U) {
              illegal_instr := true.B
            }
            io.instruction_o.op := FENCE_I
          }
        }

        when (instr.asSType().rs1 =/= 0.U || instr.asSType().imm0 =/= 0.U || instr.instr(31,28) =/= 0.U) {
          illegal_instr := true.B
        }
      }

      // --------------------------
      // Reg-Reg Operations
      // --------------------------
      is(OpcodeOp) {
        // --------------------------------------------
        // Vectorial Floating-Point Reg-Reg Operations
        // --------------------------------------------
        when (instr.asRvfType().funct2 === 2.U) { // Prefix 10 for all Xfvec ops

          // only generate decoder if FP extensions are enabled (static)
          when (FP_PRESENT.B && XFVEC.B && io.fs_i =/= Off) {
            val allow_replication = Wire(Bool()) // control honoring of replication flag

            io.instruction_o.fu  := FPU_VEC // Same unit, but sets 'vectorial' signal
            io.instruction_o.rs1 := instr.asRvfType().rs1
            io.instruction_o.rs2 := instr.asRvfType().rs2
            io.instruction_o.rd  := instr.asRvfType().rd
            check_fprm           := true.B
            allow_replication    := true.B

            // decode vectorial FP instruction
            illegal_instr := true.B // Default
            switch (instr.asRvfType().vecfltop) {
              is(Integer.parseInt("00001", 2).U) {
                io.instruction_o.op  := FADD // vfadd.vfmt - Vectorial FP Addition
                io.instruction_o.rs1 := 0.U(REG_ADDR_SIZE.W)  // Operand A is set to 0
                io.instruction_o.rs2 := instr.asRvfType().rs1 // Operand B is set to rs1
                imm_select           := IIMM              // Operand C is set to rs2
              }
              is(Integer.parseInt("00010", 2).U) {
                io.instruction_o.op  := FSUB // vfsub.vfmt - Vectorial FP Subtraction
                io.instruction_o.rs1 := 0.U(REG_ADDR_SIZE.W)   // Operand A is set to 0
                io.instruction_o.rs2 := instr.asRvfType().rs1 // Operand B is set to rs1
                imm_select           := IIMM              // Operand C is set to rs2
              }
              is(Integer.parseInt("00011", 2).U) {
                io.instruction_o.op := FMUL // vfmul.vfmt - Vectorial FP Multiplication
              }
              is(Integer.parseInt("00100", 2).U) {
                io.instruction_o.op := FDIV // vfdiv.vfmt - Vectorial FP Division
              }
              is(Integer.parseInt("00101", 2).U) {
                io.instruction_o.op := VFMIN // vfmin.vfmt - Vectorial FP Minimum
                  check_fprm       := false.B  // rounding mode irrelevant
              }
              is(Integer.parseInt("00110", 2).U) {
                io.instruction_o.op := VFMAX; // vfmax.vfmt - Vectorial FP Maximum
                check_fprm       := false.B;  // rounding mode irrelevant
              }
              is(Integer.parseInt("00111", 2).U) {
                io.instruction_o.op  := FSQRT // vfsqrt.vfmt - Vectorial FP Square Root
                allow_replication := false.B  // only one operand
                when (instr.asRvfType().rs2 =/= Integer.parseInt("00000", 2).U) {
                  illegal_instr := true.B // rs2 must be 0
                }
              }
              is(Integer.parseInt("01000", 2).U) {
                io.instruction_o.op := FMADD // vfmac.vfmt - Vectorial FP Multiply-Accumulate
                  imm_select          := SIMM  // rd into result field (upper bits don't matter)
              }
              is(Integer.parseInt("01001", 2).U) {
                io.instruction_o.op := FMSUB // vfmre.vfmt - Vectorial FP Multiply-Reduce
                imm_select          := SIMM  // rd into result field (upper bits don't matter)
              }
              is(Integer.parseInt("01100", 2).U) {
                // Default
                illegal_instr := true.B
                switch (instr.asRvfType().rs2) { // operation encoded in rs2, `inside` for matching ?
                  is(Integer.parseInt("00000", 2).U) {
                    io.instruction_o.rs2 := instr.asRvfType().rs1; // set rs2 := rs1 so we can map FMV to SGNJ in the unit
                    when (instr.asRvfType().repl) {
                      io.instruction_o.op := FMV_X2F // vfmv.vfmt.x - GPR to FPR Move
                    } .otherwise {
                      io.instruction_o.op := FMV_F2X // vfmv.x.vfmt - FPR to GPR Move
                    }
                    check_fprm := false.B;              // no rounding for moves
                  }
                  is(Integer.parseInt("00001", 2).U) {
                    io.instruction_o.op  := FCLASS; // vfclass.vfmt - Vectorial FP Classify
                    check_fprm        := false.B;   // no rounding for classification
                    allow_replication := false.B;   // R must not be set
                  }
                  is(Integer.parseInt("00010", 2).U) { io.instruction_o.op := FCVT_F2I } // vfcvt.x.vfmt - Vectorial FP to Int Conversion
                  is(Integer.parseInt("00011", 2).U) { io.instruction_o.op := FCVT_I2F } // vfcvt.vfmt.x - Vectorial Int to FP Conversion
                  is(Integer.parseInt("001??", 2).U) {
                    io.instruction_o.op  := FCVT_F2F; // vfcvt.vfmt.vfmt - Vectorial FP to FP Conversion
                    io.instruction_o.rs2 := instr.asRvfType().rd; // set rs2 := rd as target vector for conversion
                    imm_select        := IIMM;     // rs2 holds part of the intruction

                    // TODO CHECK R bit for valid fmt combinations
                    // determine source format
                    illegal_instr := true.B  // Default
                    switch (instr.asRvfType().rs2(21,20)) {
                      // Only process instruction if corresponding extension is active (static)
                      is(Integer.parseInt("00", 2).U) { if (!RVFVEC)     { illegal_instr := true.B } }
                      is(Integer.parseInt("01", 2).U) { if (!XF16ALTVEC) { illegal_instr := true.B } }
                      is(Integer.parseInt("10", 2).U) { if (!XF16VEC)    { illegal_instr := true.B } }
                      is(Integer.parseInt("11", 2).U) { if (!XF8VEC)     { illegal_instr := true.B } }
                    }
                  }
                }
              }
              is(Integer.parseInt("01101", 2).U) {
                check_fprm := false.B;         // no rounding for sign-injection
                io.instruction_o.op := VFSGNJ; // vfsgnj.vfmt - Vectorial FP Sign Injection
              }
              is(Integer.parseInt("01110", 2).U) {
                check_fprm := false.B;          // no rounding for sign-injection
                io.instruction_o.op := VFSGNJN; // vfsgnjn.vfmt - Vectorial FP Negated Sign Injection
              }
              is(Integer.parseInt("01111", 2).U) {
                check_fprm := false.B;          // no rounding for sign-injection
                io.instruction_o.op := VFSGNJX; // vfsgnjx.vfmt - Vectorial FP XORed Sign Injection
              }
              is(Integer.parseInt("10000", 2).U) {
                check_fprm := false.B;          // no rounding for comparisons
                io.instruction_o.op := VFEQ;    // vfeq.vfmt - Vectorial FP Equality
              }
              is(Integer.parseInt("10001", 2).U) {
                check_fprm := false.B;          // no rounding for comparisons
                io.instruction_o.op := VFNE;    // vfne.vfmt - Vectorial FP Non-Equality
              }
              is(Integer.parseInt("10010", 2).U) {
                check_fprm := false.B;          // no rounding for comparisons
                io.instruction_o.op := VFLT;    // vfle.vfmt - Vectorial FP Less Than
              }
              is(Integer.parseInt("10011", 2).U) {
                check_fprm := false.B;          // no rounding for comparisons
                io.instruction_o.op := VFGE;    // vfge.vfmt - Vectorial FP Greater or Equal
              }
              is(Integer.parseInt("10100", 2).U) {
                check_fprm := false.B;          // no rounding for comparisons
                io.instruction_o.op := VFLE;    // vfle.vfmt - Vectorial FP Less or Equal
              }
              is(Integer.parseInt("10101", 2).U) {
                check_fprm := false.B;          // no rounding for comparisons
                io.instruction_o.op := VFGT;    // vfgt.vfmt - Vectorial FP Greater Than
              }
              is(Integer.parseInt("11000", 2).U) {
                io.instruction_o.op  := VFCPKAB_S; // vfcpka/b.vfmt.s - Vectorial FP Cast-and-Pack from 2x FP32, lowest 4 entries
                imm_select        := SIMM;      // rd into result field (upper bits don't matter)
                if (!RVF) { illegal_instr := true.B } // if we don't support RVF, we can't cast from FP32

                // check destination format
                illegal_instr := true.B // Default
                switch (instr.asRvfType().vfmt) {
                  // Only process instruction if corresponding extension is active and FLEN suffices (static)
                  is(Integer.parseInt("00", 2).U) {
                    if (!RVFVEC)            { illegal_instr := true.B } // destination vector not supported
                    when (instr.asRvfType().repl) { illegal_instr := true.B } // no entries 2/3 in vector of 2 fp32
                  }
                  is(Integer.parseInt("01", 2).U) {
                    if (!XF16ALTVEC) { illegal_instr := true.B } // destination vector not supported
                  }
                  is(Integer.parseInt("10", 2).U) {
                    if (!XF16VEC) { illegal_instr := true.B } // destination vector not supported
                  }
                  is(Integer.parseInt("11", 2).U) {
                    if (!XF8VEC) { illegal_instr := true.B } // destination vector not supported
                  }
                }
              }
              is(Integer.parseInt("11001", 2).U) {
                io.instruction_o.op  := VFCPKCD_S; // vfcpkc/d.vfmt.s - Vectorial FP Cast-and-Pack from 2x FP32, second 4 entries
                imm_select        := SIMM;      // rd into result field (upper bits don't matter)
                if (!RVF) { illegal_instr := true.B } // if we don't support RVF, we can't cast from FP32

                // check destination format
                illegal_instr := true.B // Default
                switch (instr.asRvfType().vfmt) {
                  // Only process instruction if corresponding extension is active and FLEN suffices (static)
                  is(Integer.parseInt("00", 2).U) { illegal_instr := true.B } // no entries 4-7 in vector of 2 FP32
                  is(Integer.parseInt("01", 2).U) { illegal_instr := true.B } // no entries 4-7 in vector of 4 FP16ALT
                  is(Integer.parseInt("10", 2).U) { illegal_instr := true.B } // no entries 4-7 in vector of 4 FP16
                  is(Integer.parseInt("11", 2).U) {
                    if (!XF8VEC) { illegal_instr := true.B } // destination vector not supported
                  }
                }
              }
              is(Integer.parseInt("11010", 2).U) {
                io.instruction_o.op  := VFCPKAB_D; // vfcpka/b.vfmt.d - Vectorial FP Cast-and-Pack from 2x FP64, lowest 4 entries
                imm_select        := SIMM;      // rd into result field (upper bits don't matter)
                if (!RVD) { illegal_instr := true.B } // if we don't support RVD, we can't cast from FP64

                // check destination format
                illegal_instr := true.B // Default
                switch (instr.asRvfType().vfmt) {
                  // Only process instruction if corresponding extension is active and FLEN suffices (static)
                  is(Integer.parseInt("00", 2).U) {
                    if (!RVFVEC)            { illegal_instr := true.B } // destination vector not supported
                    when (instr.asRvfType().repl) { illegal_instr := true.B } // no entries 2/3 in vector of 2 fp32
                  }
                  is(Integer.parseInt("01", 2).U) {
                    if (!XF16ALTVEC) { illegal_instr := true.B } // destination vector not supported
                  }
                  is(Integer.parseInt("10", 2).U) {
                    if (!XF16VEC) { illegal_instr := true.B } // destination vector not supported
                  }
                  is(Integer.parseInt("11", 2).U) {
                    if (!XF8VEC) { illegal_instr := true.B } // destination vector not supported
                  }
                }
              }
              is(Integer.parseInt("11011", 2).U) {
                io.instruction_o.op  := VFCPKCD_D; // vfcpka/b.vfmt.d - Vectorial FP Cast-and-Pack from 2x FP64, second 4 entries
                imm_select        := SIMM;      // rd into result field (upper bits don't matter)
                if (!RVD) { illegal_instr := true.B } // if we don't support RVD, we can't cast from FP64

                // check destination format
                illegal_instr := true.B
                switch (instr.asRvfType().vfmt) {
                  // Only process instruction if corresponding extension is active and FLEN suffices (static)
                  is(Integer.parseInt("00", 2).U) { illegal_instr := true.B } // no entries 4-7 in vector of 2 FP32
                  is(Integer.parseInt("01", 2).U) { illegal_instr := true.B } // no entries 4-7 in vector of 4 FP16ALT
                  is(Integer.parseInt("10", 2).U) { illegal_instr := true.B } // no entries 4-7 in vector of 4 FP16
                  is(Integer.parseInt("11", 2).U) {
                    if (!XF8VEC) { illegal_instr := true.B } // destination vector not supported
                  }
                }
              }
            }

            // check format
            illegal_instr := true.B // Default
            switch (instr.asRvfType().vfmt) {
              // Only process instruction if corresponding extension is active (static)
              is(Integer.parseInt("00", 2).U) { if (!RVFVEC)     { illegal_instr := true.B } }
              is(Integer.parseInt("01", 2).U) { if (!XF16ALTVEC) { illegal_instr := true.B } }
              is(Integer.parseInt("10", 2).U) { if (!XF16VEC)    { illegal_instr := true.B } }
              is(Integer.parseInt("11", 2).U) { if (!XF8VEC)     { illegal_instr := true.B } }
            }

            // check disallowed replication
            when (~allow_replication & instr.asRvfType().repl) {
              illegal_instr := true.B
            }

            // check rounding mode
            when (check_fprm) {
              illegal_instr := true.B  // Default
              switch (io.frm_i) { // actual rounding mode from frm csr
                is(0.U(3.W), 1.U(3.W), 2.U(3.W), 3.U(3.W), 4.U(3.W)) { //legal rounding modes
                  illegal_instr := false.B
                }
              }
            }
          } .otherwise { // No vectorial FP enabled (static)
            illegal_instr := true.B
          }
        } .otherwise {
          // ---------------------------
          // Integer Reg-Reg Operations
          // ---------------------------
          io.instruction_o.fu  := Mux(instr.asRType().funct7 === Integer.parseInt("000_0001", 2).U, MULT, ALU)
          io.instruction_o.rs1 := instr.asRType().rs1
          io.instruction_o.rs2 := instr.asRType().rs2
          io.instruction_o.rd  := instr.asRType().rd

          illegal_instr := true.B
          switch (Cat(instr.asRType().funct7, instr.asRType().funct3)) {
            is(Integer.parseInt("0000000000", 2).U) { io.instruction_o.op := ADD    }  // Add
            is(Integer.parseInt("0100000000", 2).U) { io.instruction_o.op := SUB    }  // Sub
            is(Integer.parseInt("0000000010", 2).U) { io.instruction_o.op := SLTS   }  // Set Lower Than
            is(Integer.parseInt("0000000011", 2).U) { io.instruction_o.op := SLTU   }  // Set Lower Than Unsigned
            is(Integer.parseInt("0000000100", 2).U) { io.instruction_o.op := XORL   }  // Xor
            is(Integer.parseInt("0000000110", 2).U) { io.instruction_o.op := ORL    }  // Or
            is(Integer.parseInt("0000000111", 2).U) { io.instruction_o.op := ANDL   }  // And
            is(Integer.parseInt("0000000001", 2).U) { io.instruction_o.op := SLL    }  // Shift Left Logical
            is(Integer.parseInt("0000000101", 2).U) { io.instruction_o.op := SRL    }  // Shift Right Logical
            is(Integer.parseInt("0100000101", 2).U) { io.instruction_o.op := SRA    }  // Shift Right Arithmetic

            is(Integer.parseInt("0000001000", 2).U) { io.instruction_o.op := MUL    }
            is(Integer.parseInt("0000001001", 2).U) { io.instruction_o.op := MULH   }
            is(Integer.parseInt("0000001010", 2).U) { io.instruction_o.op := MULHSU }
            is(Integer.parseInt("0000001011", 2).U) { io.instruction_o.op := MULHU  }
            is(Integer.parseInt("0000001100", 2).U) { io.instruction_o.op := DIV    }
            is(Integer.parseInt("0000001101", 2).U) { io.instruction_o.op := DIVU   }
            is(Integer.parseInt("0000001110", 2).U) { io.instruction_o.op := REM    }
            is(Integer.parseInt("0000001111", 2).U) { io.instruction_o.op := REMU   }
          }
        }
      }
      // // --------------------------
      // // 32bit Reg-Reg Operations
      // // --------------------------
      // is(OpcodeOp32) {
      //   io.instruction_o.fu  := Mux(instr.asRType().funct7 === Integer.parseInt("000_0001", 2).U, MULT, ALU)
      //   io.instruction_o.rs1 := instr.asRType().rs1
      //   io.instruction_o.rs2 := instr.asRType().rs2
      //   io.instruction_o.rd  := instr.asRType().rd
      //
      //   illegal_instr := true.B // Default
      //   switch (Cat(instr.asRType().funct7, instr.asRType().funct3)) {
      //     is(Integer.parseInt("000_0000_000", 2).U) { io.instruction_o.op := ADDW  } // addw
      //     is(Integer.parseInt("010_0000_000", 2).U) { io.instruction_o.op := SUBW  } // subw
      //     is(Integer.parseInt("000_0000_001", 2).U) { io.instruction_o.op := SLLW  } // sllw
      //     is(Integer.parseInt("000_0000_101", 2).U) { io.instruction_o.op := SRLW  } // srlw
      //     is(Integer.parseInt("010_0000_101", 2).U) { io.instruction_o.op := SRAW  } // sraw
      //
      //     is(Integer.parseInt("000_0001_000", 2).U) { io.instruction_o.op := MULW  }
      //     is(Integer.parseInt("000_0001_100", 2).U) { io.instruction_o.op := DIVW  }
      //     is(Integer.parseInt("000_0001_101", 2).U) { io.instruction_o.op := DIVUW }
      //     is(Integer.parseInt("000_0001_110", 2).U) { io.instruction_o.op := REMW  }
      //     is(Integer.parseInt("000_0001_111", 2).U) { io.instruction_o.op := REMUW }
      //   }
      // }
      // // --------------------------------
      // // Reg-Immediate Operations
      // // --------------------------------
      // is(OpcodeOpImm) {
      //   io.instruction_o.fu  := ALU
      //   imm_select := IIMM
      //   io.instruction_o.rs1 := instr.asIType().rs1
      //   io.instruction_o.rd  := instr.asIType().rd
      //
      //   switch (instr.asIType().funct3) {
      //     is(Integer.parseInt("000", 2).U) { io.instruction_o.op := ADD  }  // Add Immediate
      //     is(Integer.parseInt("010", 2).U) { io.instruction_o.op := SLTS }  // Set to one if Lower Than Immediate
      //     is(Integer.parseInt("011", 2).U) { io.instruction_o.op := SLTU }  // Set to one if Lower Than Immediate Unsigned
      //     is(Integer.parseInt("100", 2).U) { io.instruction_o.op := XORL }  // Exclusive Or with Immediate
      //     is(Integer.parseInt("110", 2).U) { io.instruction_o.op := ORL  }  // Or with Immediate
      //     is(Integer.parseInt("111", 2).U) { io.instruction_o.op := ANDL }  // And with Immediate
      //
      //     is(Integer.parseInt("001", 2).U) {
      //       io.instruction_o.op := SLL;  // Shift Left Logical by Immediate
      //       when (instr.instr(31,26) =/= 0.U(6.W)) {
      //         illegal_instr := true.B
      //       }
      //     }
      //
      //     is(Integer.parseInt("101", 2).U) {
      //       when (instr.instr(31,26) === 0.U(6.W)) {
      //         io.instruction_o.op := SRL;  // Shift Right Logical by Immediate
      //       } .elsewhen (instr.instr(31,26) === Integer.parseInt("010_000", 2).U) {
      //         io.instruction_o.op := SRA;  // Shift Right Arithmetically by Immediate
      //       } .otherwise {
      //         illegal_instr := true.B
      //       }
      //     }
      //   }
      // }
      //
      // // --------------------------------
      // // 32 bit Reg-Immediate Operations
      // // --------------------------------
      // is(OpcodeOpImm32) {
      //   io.instruction_o.fu  := ALU
      //   imm_select := IIMM
      //   io.instruction_o.rs1 := instr.asIType().rs1
      //   io.instruction_o.rd  := instr.asIType().rd
      //
      //   illegal_instr := true.B // Default
      //   switch (instr.asIType().funct3) {
      //     is(Integer.parseInt("000", 2).U) { io.instruction_o.op := ADDW }  // Add Immediate
      //
      //     is(Integer.parseInt("001", 2).U) {
      //       io.instruction_o.op := SLLW;  // Shift Left Logical by Immediate
      //       when (instr.instr(31,25) =/= 0.U(7.W)) {
      //         illegal_instr := true.B
      //       }
      //     }
      //
      //     is(Integer.parseInt("101", 2).U) {
      //       when (instr.instr(31,25) === 0.U(7.W)) {
      //         io.instruction_o.op := SRLW;  // Shift Right Logical by Immediate
      //       } .elsewhen (instr.instr(31,25) === Integer.parseInt("010_0000", 2).U) {
      //         io.instruction_o.op := SRAW;  // Shift Right Arithmetically by Immediate
      //       } .otherwise {
      //         illegal_instr := true.B
      //       }
      //     }
      //   }
      // }
      // // --------------------------------
      // // LSU
      // // --------------------------------
      // is(OpcodeStore) {
      //   io.instruction_o.fu  := STORE
      //   imm_select := SIMM
      //   io.instruction_o.rs1  := instr.asSType().rs1
      //   io.instruction_o.rs2  := instr.asSType().rs2
      //   // determine store size
      //   illegal_instr := true.B  // Default
      //   switch (instr.asSType().funct3) {
      //     is(Integer.parseInt("000", 2).U) { io.instruction_o.op  := SB }
      //     is(Integer.parseInt("001", 2).U) { io.instruction_o.op  := SH }
      //     is(Integer.parseInt("010", 2).U) { io.instruction_o.op  := SW }
      //     is(Integer.parseInt("011", 2).U) { io.instruction_o.op  := SD }
      //   }
      // }
      //
      // is(OpcodeLoad) {
      //   io.instruction_o.fu  := LOAD
      //   imm_select := IIMM
      //   io.instruction_o.rs1 := instr.asIType().rs1
      //   io.instruction_o.rd  := instr.asIType().rd
      //
      //   // determine load size and signed type
      //   illegal_instr := true.B // Default
      //   switch (instr.asIType().funct3) {
      //     is(Integer.parseInt("000", 2).U) { io.instruction_o.op  := LB  }
      //     is(Integer.parseInt("001", 2).U) { io.instruction_o.op  := LH  }
      //     is(Integer.parseInt("010", 2).U) { io.instruction_o.op  := LW  }
      //     is(Integer.parseInt("100", 2).U) { io.instruction_o.op  := LBU }
      //     is(Integer.parseInt("101", 2).U) { io.instruction_o.op  := LHU }
      //     is(Integer.parseInt("110", 2).U) { io.instruction_o.op  := LWU }
      //     is(Integer.parseInt("011", 2).U) { io.instruction_o.op  := LD  }
      //   }
      // }
      //
      // // --------------------------------
      // // Floating-Point Load/store
      // // --------------------------------
      // is(OpcodeStoreFp) {
      //   when (FP_PRESENT.B && io.fs_i =/= Off) { // only generate decoder if FP extensions are enabled (static)
      //     io.instruction_o.fu  := STORE
      //     imm_select := SIMM
      //     io.instruction_o.rs1        := instr.asSType().rs1
      //     io.instruction_o.rs2        := instr.asSType().rs2
      //
      //     // determine store size
      //     illegal_instr := true.B
      //     switch (instr.asSType().funct3) {
      //       // Only process instruction if corresponding extension is active (static)
      //       is(Integer.parseInt("000", 2).U) {
      //         if (XF8) { io.instruction_o.op := FSB }
      //         else     { illegal_instr := true.B }
      //       }
      //       is(Integer.parseInt("001", 2).U) {
      //         if (XF16 | XF16ALT) io.instruction_o.op := FSH
      //         else illegal_instr := true.B
      //       }
      //       is(Integer.parseInt("010", 2).U) {
      //         if(RVF) io.instruction_o.op := FSW
      //         else illegal_instr := true.B
      //       }
      //       is(Integer.parseInt("011", 2).U) {
      //         if (RVD) io.instruction_o.op := FSD
      //         else illegal_instr := true.B
      //       }
      //     }
      //   } .otherwise {
      //     illegal_instr := true.B
      //   }
      // }
      // is(OpcodeLoadFp) {
      //   when (FP_PRESENT.B && io.fs_i =/= Off) { // only generate decoder if FP extensions are enabled (static)
      //     io.instruction_o.fu  := LOAD
      //     imm_select := IIMM
      //     io.instruction_o.rs1       := instr.asIType().rs1
      //     io.instruction_o.rd        := instr.asIType().rd
      //     // determine load size
      //     illegal_instr := true.B // Default
      //     switch (instr.asIType().funct3) {
      //       // Only process instruction if corresponding extension is active (static)
      //       is(0.U(3.W)) {
      //         if (XF8) io.instruction_o.op := FLB
      //         else illegal_instr := true.B
      //       }
      //       is(Integer.parseInt("001", 2).U) {
      //         if (XF16 | XF16ALT) io.instruction_o.op := FLH
      //         else illegal_instr := true.B
      //       }
      //       is(Integer.parseInt("010", 2).U) {
      //         if (RVF) io.instruction_o.op  := FLW
      //         else illegal_instr := true.B
      //       }
      //       is(Integer.parseInt("011", 2).U) {
      //         if (RVD) io.instruction_o.op  := FLD
      //         else illegal_instr := true.B
      //       }
      //     }
      //   } .otherwise {
      //     illegal_instr := true.B
      //   }
      // }
      // // ----------------------------------
      // // Floating-Point Reg-Reg Operations
      // // ----------------------------------
      // is(OpcodeMadd,
      //   OpcodeMsub,
      //   OpcodeNmsub,
      //   OpcodeNmadd) {
      //   when (FP_PRESENT.B && io.fs_i =/= Off) { // only generate decoder if FP extensions are enabled (static)
      //     io.instruction_o.fu  := FPU
      //     io.instruction_o.rs1 := instr.asR4Type().rs1
      //     io.instruction_o.rs2 := instr.asR4Type().rs2
      //     io.instruction_o.rd  := instr.asR4Type().rd
      //     imm_select        := RS3; // rs3 into result field
      //     check_fprm        := true.B
      //     // select the correct fused operation
      //     io.instruction_o.op := FMADD;  // Default fmadd.fmt - FP Fused multiply-add
      //     switch (instr.asR4Type().opcode) {
      //       is(OpcodeMsub)  { io.instruction_o.op := FMSUB  } // fmsub.fmt - FP Fused multiply-subtract
      //         is(OpcodeNmsub) { io.instruction_o.op := FNMSUB } // fnmsub.fmt - FP Negated fused multiply-subtract
      //         is(OpcodeNmadd) { io.instruction_o.op := FNMADD } // fnmadd.fmt - FP Negated fused multiply-add
      //     }
      //
      //     // determine fp format
      //     illegal_instr := true.B // Default
      //     switch (instr.asR4Type().funct2) {
      //       // Only process instruction if corresponding extension is active (static)
      //       is(Integer.parseInt("00", 2).U) { if (!RVF)             { illegal_instr := true.B }}
      //       is(Integer.parseInt("01", 2).U) { if (!RVD)             { illegal_instr := true.B }}
      //       is(Integer.parseInt("10", 2).U) { if (!XF16 & !XF16ALT) { illegal_instr := true.B }}
      //       is(Integer.parseInt("11", 2).U) { if (!XF8)             { illegal_instr := true.B }}
      //     }
      //
      //     // check rounding mode
      //     when (check_fprm) {
      //       illegal_instr := true.B // Default
      //       switch (instr.asRfType().rm) {
      //         is(0.U(3.W), 1.U(3.W), 2.U(3.W), 3.U(3.W), 4.U(3.W)) {
      //           //legal rounding modes
      //         }
      //         is(5.U(3.W)) {      // Alternative Half-Precsision encded as fmt=10 and rm=101
      //           when (!XF16ALT.B || (instr.asRfType().fmt =/= 2.U(2.W))) {
      //             illegal_instr := true.B
      //             switch (io.frm_i) { // actual rounding mode from frm csr
      //               is(0.U(3.W), 1.U(3.W), 2.U(3.W), 3.U(3.W), 4.U(3.W)) {  //legal rounding modes
      //                 illegal_instr := false.B
      //               }
      //             }
      //           }
      //         }
      //         is(7.U(3.W)) {
      //           // rounding mode from frm csr
      //           illegal_instr := true.B
      //           switch (io.frm_i) {
      //             is(0.U(3.W), 1.U(3.W), 2.U(3.W), 3.U(3.W), 4.U(3.W)) { //legal rounding modes
      //               illegal_instr := false.B
      //             }
      //           }
      //         }
      //       }
      //     } .otherwise {
      //       illegal_instr := true.B
      //     }
      //   }
      // }
      // is(OpcodeOpFp) {
      //   when (FP_PRESENT.B && io.fs_i =/= Off) { // only generate decoder if FP extensions are enabled (static)
      //     io.instruction_o.fu  := FPU
      //     io.instruction_o.rs1 := instr.asRfType().rs1
      //     io.instruction_o.rs2 := instr.asRfType().rs2
      //     io.instruction_o.rd  := instr.asRfType().rd
      //     check_fprm        := true.B
      //
      //     // decode FP instruction
      //     illegal_instr := true.B // Default
      //     switch (instr.asRfType().funct5) {
      //       is(Integer.parseInt("00000", 2).U) {
      //         io.instruction_o.op  := FADD             // fadd.fmt - FP Addition
      //           io.instruction_o.rs1 := 0.U(REG_ADDR_SIZE.W) // Operand A is set to 0
      //         io.instruction_o.rs2 := instr.asRfType().rs1; // Operand B is set to rs1
      //         imm_select        := IIMM;             // Operand C is set to rs2
      //       }
      //     }
      //     is(Integer.parseInt("00001", 2).U) {
      //       io.instruction_o.op  := FSUB               // fsub.fmt - FP Subtraction
      //         io.instruction_o.rs1 := 0.U(REG_ADDR_SIZE.W)               // Operand A is set to 0
      //       io.instruction_o.rs2 := instr.asRfType().rs1; // Operand B is set to rs1
      //       imm_select        := IIMM;             // Operand C is set to rs2
      //     }
      //     is(Integer.parseInt("00010", 2).U) { io.instruction_o.op := FMUL }  // fmul.fmt - FP Multiplication
      //       is(Integer.parseInt("00011", 2).U) { io.instruction_o.op := FDIV }  // fdiv.fmt - FP Division
      //       is(Integer.parseInt("01011", 2).U) {
      //         io.instruction_o.op := FSQRT; // fsqrt.fmt - FP Square Root
      //
      //         // rs2 must be zero
      //         when (instr.asRfType().rs2 =/= 0.U) {
      //           illegal_instr := true.B
      //         }
      //       }
      //     is(Integer.parseInt("00100", 2).U) {
      //       io.instruction_o.op := FSGNJ; // fsgn{j[n]/jx}.fmt - FP Sign Injection
      //       check_fprm       := false.B;  // instruction encoded in rm, do the check here
      //       if (XF16ALT) {        // FP16ALT instructions encoded in rm separately (static)
      //         when (instr.asRfType().rm === 3.U(3.W) || instr.asRfType().rm === 7.U(3.W)) {
      //           illegal_instr := true.B
      //         }
      //       } else {
      //         when (!(instr.asRfType().rm === 0.U(3.W) || instr.asRfType().rm === 1.U(3.W) || instr.asRfType().rm === 2.U(3.W))) {
      //           illegal_instr := true.B
      //         }
      //       }
      //     }
      //     is(Integer.parseInt("00101", 2).U) {
      //       io.instruction_o.op := FMIN_MAX; // fmin/fmax.fmt - FP Minimum / Maximum
      //       check_fprm       := false.B;     // instruction encoded in rm, do the check here
      //       if (XF16ALT) {           // FP16ALT instructions encoded in rm separately (static)
      //         when (instr.asRfType().rm === 3.U(3.W) || instr.asRfType().rm === 7.U(3.W)) {
      //           illegal_instr := true.B
      //         }
      //       } else {
      //         when (!(instr.asRfType().rm === 0.U(3.W) || instr.asRfType().rm === 1.U(3.W) || instr.asRfType().rm === 2.U(3.W))) {
      //           illegal_instr := true.B
      //         }
      //       }
      //     }
      //     is(Integer.parseInt("01000", 2).U) {
      //       io.instruction_o.op  := FCVT_F2F; // fcvt.fmt.fmt - FP to FP Conversion
      //       io.instruction_o.rs2 := instr.asRvfType().rs1; // tie rs2 to rs1 to be safe (vectors use rs2)
      //       imm_select        := IIMM;     // rs2 holds part of the intruction
      //       when (instr.asRfType().rs2(24,23) =/= 0.U(2.W)) { illegal_instr := true.B } // bits [22:20] used, other bits must be 0
      //
      //       // check source format
      //       illegal_instr := true.B // Default
      //       switch (instr.asRfType().rs2(22,20)) {
      //         // Only process instruction if corresponding extension is active (static)
      //         is(Integer.parseInt("000", 2).U) { if(!RVF)     { illegal_instr := true.B }}
      //         is(Integer.parseInt("001", 2).U) { if(!RVD)     { illegal_instr := true.B }}
      //         is(Integer.parseInt("010", 2).U) { if(!XF16)    { illegal_instr := true.B }}
      //         is(Integer.parseInt("110", 2).U) { if(!XF16ALT) { illegal_instr := true.B }}
      //         is(Integer.parseInt("011", 2).U) { if(!XF8)     { illegal_instr := true.B }}
      //       }
      //     }
      //     is(Integer.parseInt("10100", 2).U) {
      //       io.instruction_o.op := FCMP; // feq/flt/fle.fmt - FP Comparisons
      //       check_fprm       := false.B; // instruction encoded in rm, do the check here
      //       if (XF16ALT) {       // FP16ALT instructions encoded in rm separately (static)
      //         when (instr.asRfType().rm === 3.U(3.W) || instr.asRfType().rm === 7.U(3.W)) {
      //           illegal_instr := true.B
      //         }
      //       } else {
      //         when (!(instr.asRfType().rm === 0.U(3.W) || instr.asRfType().rm === 1.U(3.W) || instr.asRfType().rm === 2.U(3.W))) {
      //           illegal_instr := true.B
      //         }
      //       }
      //     }
      //     is(Integer.parseInt("11000", 2).U) {
      //       io.instruction_o.op := FCVT_F2I; // fcvt.ifmt.fmt - FP to Int Conversion
      //       imm_select       := IIMM;     // rs2 holds part of the instruction
      //       when (instr.asRfType().rs2(24,22) =/= 0.U(3.W)) { illegal_instr := true.B } // bits [21:20] used, other bits must be 0
      //     }
      //     is(Integer.parseInt("11010", 2).U) {
      //       io.instruction_o.op := FCVT_I2F;  // fcvt.fmt.ifmt - Int to FP Conversion
      //       imm_select       := IIMM;     // rs2 holds part of the instruction
      //       when (instr.asRfType().rs2(24,22) =/= 0.U(3.W)) { illegal_instr := true.B } // bits [21:20] used, other bits must be 0
      //     }
      //     is(Integer.parseInt("11100", 2).U) {
      //       io.instruction_o.rs2 := instr.asRfType().rs1; // set rs2 := rs1 so we can map FMV to SGNJ in the unit
      //       check_fprm        := false.B; // instruction encoded in rm, do the check here
      //       when (instr.asRfType().rm === 0.U(3.W) || (XF16ALT.B && instr.asRfType().rm === Integer.parseInt("100", 2).U)) { // FP16ALT has separate encoding
      //         io.instruction_o.op := FMV_F2X;       // fmv.ifmt.fmt - FPR to GPR Move
      //       } .elsewhen ((instr.asRfType().rm === Integer.parseInt("001", 2).U) || (XF16ALT.B && instr.asRfType().rm === Integer.parseInt("101", 2).U)) { // FP16ALT has separate encoding
      //         io.instruction_o.op := FCLASS; // fclass.fmt - FP Classify
      //       } .otherwise {
      //         illegal_instr := true.B
      //       }
      //       // rs2 must be zero
      //       when (instr.asRfType().rs2 =/= 0.U) {
      //         illegal_instr := true.B
      //       }
      //     }
      //     is(Integer.parseInt("11110", 2).U) {
      //       io.instruction_o.op := FMV_X2F;   // fmv.fmt.ifmt - GPR to FPR Move
      //       io.instruction_o.rs2 := instr.asRfType().rs1; // set rs2 := rs1 so we can map FMV to SGNJ in the unit
      //       check_fprm       := false.B; // instruction encoded in rm, do the check here
      //       when (!(instr.asRfType().rm === 0.U(3.W) || (XF16ALT.B && instr.asRfType().rm === Integer.parseInt("100", 2).U))) {
      //         illegal_instr := true.B
      //       }
      //       // rs2 must be zero
      //       when (instr.asRfType().rs2 =/= 0.U) { illegal_instr := true.B }
      //     }
      //   }
      //
      //   // check format
      //   illegal_instr := true.B
      //   switch (instr.asRfType().fmt) {
      //     // Only process instruction if corresponding extension is active (static)
      //     is(Integer.parseInt("00", 2).U) { if (!RVF)             { illegal_instr := true.B }}
      //     is(Integer.parseInt("01", 2).U) { if (!RVD)             { illegal_instr := true.B }}
      //     is(Integer.parseInt("10", 2).U) { if (!XF16 & !XF16ALT) { illegal_instr := true.B }}
      //     is(Integer.parseInt("11", 2).U) { if (!XF8)             { illegal_instr := true.B }}
      //   }
      //
      //   // check rounding mode
      //   when (check_fprm) {
      //     illegal_instr := true.B  // Default
      //     switch (instr.asRfType().rm) {
      //       is(0.U(3.W), 1.U(3.W), 2.U(3.W), 3.U(3.W), 4.U(3.W)) {  //legal rounding modes
      //       }
      //       is(Integer.parseInt("101", 2).U) {      // Alternative Half-Precsision encded as fmt=10 and rm=101
      //         when (!XF16ALT.B || (instr.asRfType().fmt =/= 2.U(2.W))) {
      //           illegal_instr := true.B
      //         }
      //         illegal_instr := true.B  // Default
      //         switch (io.frm_i) { // actual rounding mode from frm csr
      //           is(0.U(3.W), 1.U(3.W), 2.U(3.W), 3.U(3.W), 4.U(3.W)) { //legal rounding modes
      //             illegal_instr := false.B
      //           }
      //         }
      //       }
      //       is(Integer.parseInt("111", 2).U) {
      //         // rounding mode from frm csr
      //         illegal_instr := true.B  // Default
      //         switch (io.frm_i) {
      //           is(0.U(3.W), 1.U(3.W), 2.U(3.W), 3.U(3.W), 4.U(3.W)) { //legal rounding modes
      //             illegal_instr := false.B
      //           }
      //         }
      //       }
      //     }
      //   } .otherwise {
      //     illegal_instr := true.B
      //   }
      // }
      //
      // // ----------------------------------
      // // Atomic Operations
      // // ----------------------------------
      // is(OpcodeAmo) {
      //   // we are going to use the load unit for AMOs
      //   io.instruction_o.fu  := STORE
      //   io.instruction_o.rs1 := instr.asAType().rs1
      //   io.instruction_o.rs2 := instr.asAType().rs2
      //   io.instruction_o.rd  := instr.asAType().rd
      //   // TODO(zarubaf): Ordering
      //   // words
      //   when (RVA.B && instr.asSType().funct3 === 2.U(3.W)) {
      //     illegal_instr := true.B // Default
      //     switch (instr.instr(31,27)) {
      //       is("h0".U)  { io.instruction_o.op := AMO_ADDW }
      //       is("h1".U)  { io.instruction_o.op := AMO_SWAPW }
      //       is("h2".U) {
      //         io.instruction_o.op := AMO_LRW
      //         when (instr.asAType().rs2 =/= 0.U) { illegal_instr := true.B }
      //       }
      //       is("h3 ".U) { io.instruction_o.op := AMO_SCW   }
      //       is("h4 ".U) { io.instruction_o.op := AMO_XORW  }
      //       is("h8 ".U) { io.instruction_o.op := AMO_ORW   }
      //       is("hC ".U) { io.instruction_o.op := AMO_ANDW  }
      //       is("h10".U) { io.instruction_o.op := AMO_MINW  }
      //       is("h14".U) { io.instruction_o.op := AMO_MAXW  }
      //       is("h18".U) { io.instruction_o.op := AMO_MINWU }
      //       is("h1C".U) { io.instruction_o.op := AMO_MAXWU }
      //     }
      //
      //     // double words
      //   } .elsewhen (RVA.B && instr.asSType().funct3 === 3.U(3.W)) {
      //     illegal_instr := true.B  // Default
      //     switch (instr.instr(31,27)) {
      //       is("h0".U) { io.instruction_o.op := AMO_ADDD  }
      //       is("h1".U) { io.instruction_o.op := AMO_SWAPD }
      //       is("h2".U) {
      //         io.instruction_o.op := AMO_LRD
      //         when (instr.asAType().rs2 =/= 0.U) { illegal_instr := true.B }
      //       }
      //       is("h3 ".U) { io.instruction_o.op := AMO_SCD   }
      //       is("h4 ".U) { io.instruction_o.op := AMO_XORD  }
      //       is("h8 ".U) { io.instruction_o.op := AMO_ORD   }
      //       is("hC ".U) { io.instruction_o.op := AMO_ANDD  }
      //       is("h10".U) { io.instruction_o.op := AMO_MIND  }
      //       is("h14".U) { io.instruction_o.op := AMO_MAXD  }
      //       is("h18".U) { io.instruction_o.op := AMO_MINDU }
      //       is("h1C".U) { io.instruction_o.op := AMO_MAXDU }
      //     }
      //   } .otherwise {
      //     illegal_instr := true.B
      //   }
      // }
      //
      // // --------------------------------
      // // Control Flow Instructions
      // // --------------------------------
      // is(OpcodeBranch) {
      //   imm_select              := SBIMM
      //   io.instruction_o.fu        := CTRL_FLOW
      //   io.instruction_o.rs1  := instr.asSType().rs1
      //   io.instruction_o.rs2  := instr.asSType().rs2
      //
      //   io.is_control_flow_instr_o := true.B
      //
      //   io.is_control_flow_instr_o := false.B  // Default
      //   illegal_instr           := true.B   // Default
      //   switch (instr.asSType().funct3) {
      //     is(Integer.parseInt("000", 2).U) { io.instruction_o.op := EQ  }
      //     is(Integer.parseInt("001", 2).U) { io.instruction_o.op := NE  }
      //     is(Integer.parseInt("100", 2).U) { io.instruction_o.op := LTS }
      //     is(Integer.parseInt("101", 2).U) { io.instruction_o.op := GES }
      //     is(Integer.parseInt("110", 2).U) { io.instruction_o.op := LTU }
      //     is(Integer.parseInt("111", 2).U) { io.instruction_o.op := GEU }
      //   }
      // }
      // // Jump and link register
      // is(OpcodeJalr) {
      //   io.instruction_o.fu        := CTRL_FLOW
      //   io.instruction_o.op        := JALR
      //   io.instruction_o.rs1  := instr.asIType().rs1
      //   imm_select              := IIMM
      //   io.instruction_o.rd   := instr.asIType().rd
      //   io.is_control_flow_instr_o := true.B
      //   // invalid jump and link register -> reserved for vector encoding
      //   when (instr.asIType().funct3 =/= 0.U(3.W)) { illegal_instr := true.B }
      // }
      // // Jump and link
      // is(OpcodeJal) {
      //   io.instruction_o.fu        := CTRL_FLOW
      //   imm_select                 := JIMM
      //   io.instruction_o.rd        := instr.asUType().rd
      //   io.is_control_flow_instr_o := true.B
      // }
      //
      // is(OpcodeAuipc) {
      //   io.instruction_o.fu     := ALU
      //   imm_select              := UIMM
      //   io.instruction_o.use_pc := true.B
      //   io.instruction_o.rd     := instr.asUType().rd
      // }
      //
      // is(OpcodeLui) {
      //   imm_select          := UIMM
      //   io.instruction_o.fu := ALU
      //   io.instruction_o.rd := instr.asUType().rd
      // }
    }
  }

  def uj_imm (instruction_i: UInt): UInt = {
    return Cat(VecInit(Seq.fill(44)(instruction_i(31))).asUInt,
               instruction_i(19,12),
               instruction_i(20),
               instruction_i(30,21),
               0.U(1.W))
  }

  def i_imm (instruction_i: UInt): UInt = {
    return Cat(VecInit(Seq.fill(52)(instruction_i(31))).asUInt,
               instruction_i(31,20))
  }

  def sb_imm (instruction_i: UInt): UInt = {
    return Cat(VecInit(Seq.fill(51)(instruction_i(31))).asUInt,
               instruction_i(31),
               instruction_i(7),
               instruction_i(30,25),
               instruction_i(11, 8),
               0.U(1.W))
  }

  // --------------------------------
  // Sign extend immediate
  // --------------------------------
  imm_i_type  := i_imm(io.instruction_i)
  imm_s_type  := Cat( VecInit(Seq.fill(52)(io.instruction_i(31))).asUInt, io.instruction_i(31,25), io.instruction_i(11,7))
  imm_sb_type := sb_imm(io.instruction_i)
  imm_u_type  := Cat( VecInit(Seq.fill(32)(io.instruction_i(31))).asUInt, io.instruction_i(31,12), 0.U(12.W) ) // JAL, AUIPC, sign extended to 64 bit
  imm_uj_type := uj_imm(io.instruction_i)
  imm_bi_type := Cat( VecInit(Seq.fill(59)(io.instruction_i(24))).asUInt, io.instruction_i(24,20) )

  // NOIMM, IIMM, SIMM, BIMM, UIMM, JIMM, RS3
  // select immediate
  io.instruction_o.result := 0.U(64.W)  // Default
  io.instruction_o.use_imm := false.B   // Default
  switch (imm_select) {
    is(IIMM) {
      io.instruction_o.result := imm_i_type
      io.instruction_o.use_imm := true.B
    }
    is(SIMM) {
      io.instruction_o.result := imm_s_type
      io.instruction_o.use_imm := true.B
    }
    is(SBIMM) {
      io.instruction_o.result := imm_sb_type
      io.instruction_o.use_imm := true.B
    }
    is(UIMM) {
      io.instruction_o.result := imm_u_type
      io.instruction_o.use_imm := true.B
    }
    is(JIMM) {
      io.instruction_o.result := imm_uj_type
      io.instruction_o.use_imm := true.B
    }
    is(RS3) {
      // result holds address of fp operand rs3
      io.instruction_o.result := Cat(0.U(59.W), instr.asR4Type().rs3)
      io.instruction_o.use_imm := false.B
    }
  }


  //
  // xxx : Temporary Loading
  //
  val INSTR_ADDR_MISALIGNED =  0.U(64.W)
  val INSTR_ACCESS_FAULT    =  1.U(64.W)
  val ILLEGAL_INSTR         =  2.U(64.W)
  val BREAKPOINT            =  3.U(64.W)
  val LD_ADDR_MISALIGNED    =  4.U(64.W)
  val LD_ACCESS_FAULT       =  5.U(64.W)
  val ST_ADDR_MISALIGNED    =  6.U(64.W)
  val ST_ACCESS_FAULT       =  7.U(64.W)
  val ENV_CALL_UMODE        =  8.U(64.W) // environment call from user mode
  val ENV_CALL_SMODE        =  9.U(64.W) // environment call from supervisor mode
  val ENV_CALL_MMODE        = 11.U(64.W) // environment call from machine mode
  val INSTR_PAGE_FAULT      = 12.U(64.W) // Instruction page fault
  val LOAD_PAGE_FAULT       = 13.U(64.W) // Load page fault
  val STORE_PAGE_FAULT      = 15.U(64.W) // Store page fault
  val DEBUG_REQUEST         = 24.U(64.W) // Debug request

  //
  // xxx: Temporary Loading End
  //

  // ---------------------
  // Exception handling
  // ---------------------
  val interrupt_cause = Wire(UInt(64.W))

  // this instruction has already executed if the exception is valid
  io.instruction_o.valid   := io.instruction_o.ex.valid

  interrupt_cause       := 0.U
  io.instruction_o.ex      := io.ex_i
  // look if we didn't already get an exception in any previous
  // stage - we should not overwrite it as we retain order regarding the exception
  when (~io.ex_i.valid) {
    // if we didn't already get an exception save the instruction here as we may need it
    // in the commit stage if we got a access exception to one of the CSR registers
    io.instruction_o.ex.tval  := Mux(io.is_compressed_i, Cat(0.U(48.W), io.compressed_instr_i), Cat(0.U(32.W), io.instruction_i))
    // instructions which will throw an exception are marked as valid
    // e.g.: they can be committed anytime and do not need to wait for any functional unit
    // check here if we decoded an invalid instruction or if the compressed decoder already decoded
    // a invalid instruction
    when (illegal_instr || io.is_illegal_i) {
      io.instruction_o.ex.valid := true.B
      // we decoded an illegal exception here
      io.instruction_o.ex.cause := ILLEGAL_INSTR
      // we got an ecall, set the correct cause depending on the current privilege level
    } .elsewhen (ecall) {
      // this exception is valid
      io.instruction_o.ex.valid := true.B
      // depending on the privilege mode, set the appropriate cause
      switch (io.priv_lvl_i) {
        is(PRIV_LVL_M) { io.instruction_o.ex.cause := ENV_CALL_MMODE }
        is(PRIV_LVL_S) { io.instruction_o.ex.cause := ENV_CALL_SMODE }
        is(PRIV_LVL_U) { io.instruction_o.ex.cause := ENV_CALL_UMODE }
      }
    }
  } .elsewhen (ebreak) {
    // this exception is valid
    io.instruction_o.ex.valid := true.B
    // set breakpoint cause
    io.instruction_o.ex.cause := BREAKPOINT
  }

  //
  // xxx : Temporary Loading
  //
  val IRQ_S_SOFT  = 1
  val IRQ_M_SOFT  = 3
  val IRQ_S_TIMER = 5
  val IRQ_M_TIMER = 7
  val IRQ_S_EXT   = 9
  val IRQ_M_EXT   = 11

  val S_SW_INTERRUPT    = ((BigInt(1) << 63) | IRQ_S_SOFT ).U(64.W)
  val M_SW_INTERRUPT    = ((BigInt(1) << 63) | IRQ_M_SOFT ).U(64.W)
  val S_TIMER_INTERRUPT = ((BigInt(1) << 63) | IRQ_S_TIMER).U(64.W)
  val M_TIMER_INTERRUPT = ((BigInt(1) << 63) | IRQ_M_TIMER).U(64.W)
  val S_EXT_INTERRUPT   = ((BigInt(1) << 63) | IRQ_S_EXT  ).U(64.W)
  val M_EXT_INTERRUPT   = ((BigInt(1) << 63) | IRQ_M_EXT  ).U(64.W)

  val SupervisorIrq = 1
  val MachineIrq = 0
  //
  // xxx: Temporary Loading End
  //

  // -----------------
  // Interrupt Control
  // -----------------
  // we decode an interrupt the same as an exception, hence it will be taken if the instruction did not
  // throw any previous exception.
  // we have three interrupt sources: external interrupts, software interrupts, timer interrupts (order of precedence)
  // for two privilege levels: Supervisor and Machine Mode
  // Supervisor Timer Interrupt
  when (io.irq_ctrl_i.mie(S_TIMER_INTERRUPT(5,0)) && io.irq_ctrl_i.mip(S_TIMER_INTERRUPT(5,0))) {
    interrupt_cause := S_TIMER_INTERRUPT
  }
  // Supervisor Software Interrupt
  when (io.irq_ctrl_i.mie(S_SW_INTERRUPT(5,0)) && io.irq_ctrl_i.mip(S_SW_INTERRUPT(5,0))) {
    interrupt_cause := S_SW_INTERRUPT
  }
  // Supervisor External Interrupt
  // The logical-OR of the software-writable bit and the signal from the external interrupt controller is
  // used to generate external interrupts to the supervisor
  when (io.irq_ctrl_i.mie(S_EXT_INTERRUPT(5,0)) && (io.irq_ctrl_i.mip(S_EXT_INTERRUPT(5,0)) | io.irq_i(SupervisorIrq))) {
    interrupt_cause := S_EXT_INTERRUPT
  }
  // Machine Timer Interrupt
  when (io.irq_ctrl_i.mip(M_TIMER_INTERRUPT(5,0)) && io.irq_ctrl_i.mie(M_TIMER_INTERRUPT(5,0))) {
    interrupt_cause := M_TIMER_INTERRUPT
  }
  // Machine Mode Software Interrupt
  when (io.irq_ctrl_i.mip(M_SW_INTERRUPT(5,0)) && io.irq_ctrl_i.mie(M_SW_INTERRUPT(5,0))) {
    interrupt_cause := M_SW_INTERRUPT
  }
  // Machine Mode External Interrupt
  when (io.irq_ctrl_i.mip(M_EXT_INTERRUPT(5,0)) && io.irq_ctrl_i.mie(M_EXT_INTERRUPT(5,0))) {
    interrupt_cause := M_EXT_INTERRUPT
  }

  when (interrupt_cause(63) && io.irq_ctrl_i.global_enable) {
    // However, if bit i in mideleg is set, interrupts are considered to be globally enabled if the hart’s current privilege
    // mode equals the delegated privilege mode (S or U) and that mode’s interrupt enable bit
    // (SIE or UIE in mstatus) is set, or if the current privilege mode is less than the delegated privilege mode.
    when (io.irq_ctrl_i.mideleg(interrupt_cause(5,0))) {
      when ((io.irq_ctrl_i.sie && io.priv_lvl_i === PRIV_LVL_S) || io.priv_lvl_i === PRIV_LVL_U) {
        io.instruction_o.ex.valid := true.B
        io.instruction_o.ex.cause := interrupt_cause
      }
    } .otherwise {
      io.instruction_o.ex.valid := true.B
      io.instruction_o.ex.cause := interrupt_cause
    }
  }

  // a debug request has precendece over everything else
  when (io.debug_req_i && !io.debug_mode_i) {
    io.instruction_o.ex.valid := true.B
    io.instruction_o.ex.cause := DEBUG_REQUEST
  }
}


object decoder extends App {
  chisel3.Driver.emitVerilog(new decoder())
}
