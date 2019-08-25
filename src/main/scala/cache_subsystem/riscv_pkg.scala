/* Copyright 2018 ETH Zurich and University of Bologna.
 * Copyright and related rights are licensed under the Solderpad Hardware
 * License, Version 0.51 (the “License”); you may not use this file except in
 * compliance with the License.  You may obtain a copy of the License at
 * http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
 * or agreed to in writing, software, hardware and materials distributed under
 * this License is distributed on an “AS IS” BASIS, WITHOUT WARRANTIES OR
 * CONDITIONS OF ANY KIND, either express or implied. See the License for the
 * specific language governing permissions and limitations under the License.
 *
 * File:   riscv_pkg.sv
 * Author: Florian Zaruba <zarubaf@iis.ee.ethz.ch>
 * Date:   30.6.2017
 *
 * Description: Common RISC-V definitions.
 */

package cache_subsystem

import chisel3._
import chisel3.util._
import chisel3.Bool

object riscv_pkg
{
  // --------------------
  // Privilege Spec
  // --------------------
  type priv_lvl_t = UInt
  val PRIV_LVL_M:priv_lvl_t = 3.U(2.W)
  val PRIV_LVL_S:priv_lvl_t = 1.U(2.W)
  val PRIV_LVL_U:priv_lvl_t = 0.U(2.W)

  // type which holds xlen
  type xlen_t = UInt
  val XLEN_32 :xlen_t = 1.U(2.W)
  val XLEN_64 :xlen_t = 2.U(2.W)
  val XLEN_128:xlen_t = 3.U(2.W)

  type xs_t = UInt
  val Off    :xs_t = 0.U(2.W)
  val Initial:xs_t = 1.U(2.W)
  val Clean  :xs_t = 2.U(2.W)
  val Dirty  :xs_t = 3.U(2.W)

  // typedef struct packed {
  //     logic         sd;     // signal dirty state - read-only
  //     logic [62:36] wpri4;  // writes preserved reads ignored
  //     xlen_t        sxl;    // variable supervisor mode xlen - hardwired to zero
  //     xlen_t        uxl;    // variable user mode xlen - hardwired to zero
  //     logic [8:0]   wpri3;  // writes preserved reads ignored
  //     logic         tsr;    // trap sret
  //     logic         tw;     // time wait
  //     logic         tvm;    // trap virtual memory
  //     logic         mxr;    // make executable readable
  //     logic         sum;    // permit supervisor user memory access
  //     logic         mprv;   // modify privilege - privilege level for ld/st
  //     xs_t          xs;     // extension register - hardwired to zero
  //     xs_t          fs;     // floating point extension register
  //     priv_lvl_t    mpp;    // holds the previous privilege mode up to machine
  //     logic [1:0]   wpri2;  // writes preserved reads ignored
  //     logic         spp;    // holds the previous privilege mode up to supervisor
  //     logic         mpie;   // machine interrupts enable bit active prior to trap
  //     logic         wpri1;  // writes preserved reads ignored
  //     logic         spie;   // supervisor interrupts enable bit active prior to trap
  //     logic         upie;   // user interrupts enable bit active prior to trap - hardwired to zero
  //     logic         mie;    // machine interrupts enable
  //     logic         wpri0;  // writes preserved reads ignored
  //     logic         sie;    // supervisor interrupts enable
  //     logic         uie;    // user interrupts enable - hardwired to zero
  // } status_rv64_t;
  //
  // typedef struct packed {
  //     logic         sd;     // signal dirty - read-only - hardwired zero
  //     logic [7:0]   wpri3;  // writes preserved reads ignored
  //     logic         tsr;    // trap sret
  //     logic         tw;     // time wait
  //     logic         tvm;    // trap virtual memory
  //     logic         mxr;    // make executable readable
  //     logic         sum;    // permit supervisor user memory access
  //     logic         mprv;   // modify privilege - privilege level for ld/st
  //     logic [1:0]   xs;     // extension register - hardwired to zero
  //     logic [1:0]   fs;     // extension register - hardwired to zero
  //     priv_lvl_t    mpp;    // holds the previous privilege mode up to machine
  //     logic [1:0]   wpri2;  // writes preserved reads ignored
  //     logic         spp;    // holds the previous privilege mode up to supervisor
  //     logic         mpie;   // machine interrupts enable bit active prior to trap
  //     logic         wpri1;  // writes preserved reads ignored
  //     logic         spie;   // supervisor interrupts enable bit active prior to trap
  //     logic         upie;   // user interrupts enable bit active prior to trap - hardwired to zero
  //     logic         mie;    // machine interrupts enable
  //     logic         wpri0;  // writes preserved reads ignored
  //     logic         sie;    // supervisor interrupts enable
  //     logic         uie;    // user interrupts enable - hardwired to zero
  // } status_rv32_t;
  //
  // typedef struct packed {
  //     logic [3:0]  mode;
  //     logic [15:0] asid;
  //     logic [43:0] ppn;
  // } satp_t;
  //
  // --------------------
  // Instruction Types
  // --------------------
  class rtype_t extends Bundle {
    val funct7 = UInt(7.W)
    val rs2    = UInt(5.W)
    val rs1    = UInt(5.W)
    val funct3 = UInt(3.W)
    val rd     = UInt(5.W)
    val opcode = UInt(7.W)
  }

  def rtype_t(in:instruction_t) : rtype_t = {
    val rtype_v = Wire(new rtype_t)
    rtype_v.funct7 := in.instr(31,25)
    rtype_v.rs2    := in.instr(24,20)
    rtype_v.rs1    := in.instr(19,15)
    rtype_v.funct3 := in.instr(14,12)
    rtype_v.rd     := in.instr(11, 7)
    rtype_v.opcode := in.instr( 6, 0)

    return rtype_v
  }

  class r4type_t extends Bundle {
    val rs3    = UInt(3.W)
    val funct2 = UInt(2.W)
    val rs2    = UInt(5.W)
    val rs1    = UInt(5.W)
    val funct3 = UInt(3.W)
    val rd     = UInt(5.W)
    val opcode = UInt(7.W)
  }

  class rftype_t extends Bundle {
    val funct5 = UInt(5.W)
    val fmt    = UInt(2.W)
    val rs2    = UInt(5.W)
    val rs1    = UInt(5.W)
    val rm     = UInt(3.W)
    val rd     = UInt(5.W)
    val opcode = UInt(7.W)
  } // floating-point

  class rvftype_t extends Bundle {
    val funct2   = UInt(2.W)
    val vecfltop = UInt(5.W)
    val rs2      = UInt(5.W)
    val rs1      = UInt(5.W)
    val repl     = Bool()
    val vfmt     = UInt(2.W)
    val rd       = UInt(5.W)
    val opcode   = UInt(7.W)
  }  // vectorial floating-point


  def rvftype_t(in:instruction_t) : rvftype_t = {
    val rvftype_v = Wire(new rvftype_t)
    rvftype_v.funct2   := in.instr(31,30)
    rvftype_v.vecfltop := in.instr(29,25)
    rvftype_v.rs2      := in.instr(24,20)
    rvftype_v.rs1      := in.instr(19,15)
    rvftype_v.repl     := in.instr(14,14)
    rvftype_v.vfmt     := in.instr(13,12)
    rvftype_v.rd       := in.instr(11, 7)
    rvftype_v.opcode   := in.instr( 6, 0)

    return rvftype_v
  }


  class itype_t extends Bundle {
    val imm    = UInt(12.W)
    val rs1    = UInt(5.W)
    val funct3 = UInt(3.W)
    val rd     = UInt(5.W)
    val opcode = UInt(7.W)
  }

  def itype_t(in:instruction_t) : itype_t = {
    val itype_v = Wire(new itype_t)
    itype_v.imm    := in.instr(31,20)
    itype_v.rs1    := in.instr(19,15)
    itype_v.funct3 := in.instr(14,12)
    itype_v.rd     := in.instr(11, 7)
    itype_v.opcode := in.instr( 6, 0)

    return itype_v
  }

  class stype_t extends Bundle {
    val imm    = UInt(7.W)
    val rs2    = UInt(5.W)
    val rs1    = UInt(5.W)
    val funct3 = UInt(3.W)
    val imm0   = UInt(5.W)
    val opcode = UInt(7.W)
  }

  def stype_t(in:instruction_t) : stype_t = {
    val stype_v = Wire(new stype_t)
    stype_v.imm    := in.instr(31,25)
    stype_v.rs2    := in.instr(24,20)
    stype_v.rs1    := in.instr(19,15)
    stype_v.funct3 := in.instr(14,12)
    stype_v.imm0   := in.instr(11, 7)
    stype_v.opcode := in.instr( 6, 0)

    return stype_v
  }

  class utype_t extends Bundle {
    val funct3 = UInt(10.W)
    val rd     = UInt(5.W)
    val opcode = UInt(7.W)
  }

  // atomic instructions
  class atype_t extends Bundle {
    val funct5 = UInt(5.W)
    val aq     = UInt(1.W)
    val rl     = UInt(1.W)
    val rs2    = UInt(5.W)
    val rs1    = UInt(5.W)
    val funct3 = UInt(3.W)
    val rd     = UInt(5.W)
    val opcode = UInt(7.W)
  }

  class instruction_t extends Bundle {
    val instr = UInt(32.W)
    // val  rtype_t        rtype
    // val  r4type_t       r4type
    // val  rftype_t       rftype
    // val  rvftype_t      rvftype
    // val  itype_t        itype
    // val  stype_t        stype
    // val  utype_t        utype
    // val  atype_t        atype
  }

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

  // // ----------------------
  // // Virtual Memory
  // // ----------------------
  // // memory management, pte
  // class extends Bundle {
  //     logic [9:0]  reserved
  //     logic [43:0] ppn
  //     logic [1:0]  rsw
  //     logic d
  //     logic a
  //     logic g
  //     logic u
  //     logic x
  //     logic w
  //     logic r
  //     logic v
  // } pte_t
  //
  // ----------------------
  // Exception Cause Codes
  // ----------------------
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

  // val int unsigned IRQ_S_SOFT  = 1
  // val int unsigned IRQ_M_SOFT  = 3
  // val int unsigned IRQ_S_TIMER = 5
  // val int unsigned IRQ_M_TIMER = 7
  // val int unsigned IRQ_S_EXT   = 9
  // val int unsigned IRQ_M_EXT   = 11
  //
  // val logic [63:0] MIP_SSIP = 1 << IRQ_S_SOFT
  // val logic [63:0] MIP_MSIP = 1 << IRQ_M_SOFT
  // val logic [63:0] MIP_STIP = 1 << IRQ_S_TIMER
  // val logic [63:0] MIP_MTIP = 1 << IRQ_M_TIMER
  // val logic [63:0] MIP_SEIP = 1 << IRQ_S_EXT
  // val logic [63:0] MIP_MEIP = 1 << IRQ_M_EXT
  //
  // val logic [63:0] S_SW_INTERRUPT    = (1 << 63) | IRQ_S_SOFT
  // val logic [63:0] M_SW_INTERRUPT    = (1 << 63) | IRQ_M_SOFT
  // val logic [63:0] S_TIMER_INTERRUPT = (1 << 63) | IRQ_S_TIMER
  // val logic [63:0] M_TIMER_INTERRUPT = (1 << 63) | IRQ_M_TIMER
  // val logic [63:0] S_EXT_INTERRUPT   = (1 << 63) | IRQ_S_EXT
  // val logic [63:0] M_EXT_INTERRUPT   = (1 << 63) | IRQ_M_EXT
  //
  // // -----
  // // CSRs
  // // -----
  // typedef enum logic [11:0] {
  //     // Floating-Point CSRs
  //     CSR_FFLAGS         = 12'h001,
  //     CSR_FRM            = 12'h002,
  //     CSR_FCSR           = 12'h003,
  //     CSR_FTRAN          = 12'h800,
  //     // Supervisor Mode CSRs
  //     CSR_SSTATUS        = 12'h100,
  //     CSR_SIE            = 12'h104,
  //     CSR_STVEC          = 12'h105,
  //     CSR_SCOUNTEREN     = 12'h106,
  //     CSR_SSCRATCH       = 12'h140,
  //     CSR_SEPC           = 12'h141,
  //     CSR_SCAUSE         = 12'h142,
  //     CSR_STVAL          = 12'h143,
  //     CSR_SIP            = 12'h144,
  //     CSR_SATP           = 12'h180,
  //     // Machine Mode CSRs
  //     CSR_MSTATUS        = 12'h300,
  //     CSR_MISA           = 12'h301,
  //     CSR_MEDELEG        = 12'h302,
  //     CSR_MIDELEG        = 12'h303,
  //     CSR_MIE            = 12'h304,
  //     CSR_MTVEC          = 12'h305,
  //     CSR_MCOUNTEREN     = 12'h306,
  //     CSR_MSCRATCH       = 12'h340,
  //     CSR_MEPC           = 12'h341,
  //     CSR_MCAUSE         = 12'h342,
  //     CSR_MTVAL          = 12'h343,
  //     CSR_MIP            = 12'h344,
  //     CSR_PMPCFG0        = 12'h3A0,
  //     CSR_PMPADDR0       = 12'h3B0,
  //     CSR_MVENDORID      = 12'hF11,
  //     CSR_MARCHID        = 12'hF12,
  //     CSR_MIMPID         = 12'hF13,
  //     CSR_MHARTID        = 12'hF14,
  //     CSR_MCYCLE         = 12'hB00,
  //     CSR_MINSTRET       = 12'hB02,
  //     // Performance counters (Machine Mode)
  //     CSR_ML1_ICACHE_MISS = 12'hB03,  // L1 Instr Cache Miss
  //     CSR_ML1_DCACHE_MISS = 12'hB04,  // L1 Data Cache Miss
  //     CSR_MITLB_MISS      = 12'hB05,  // ITLB Miss
  //     CSR_MDTLB_MISS      = 12'hB06,  // DTLB Miss
  //     CSR_MLOAD           = 12'hB07,  // Loads
  //     CSR_MSTORE          = 12'hB08,  // Stores
  //     CSR_MEXCEPTION      = 12'hB09,  // Taken exceptions
  //     CSR_MEXCEPTION_RET  = 12'hB0A,  // Exception return
  //     CSR_MBRANCH_JUMP    = 12'hB0B,  // Software change of PC
  //     CSR_MCALL           = 12'hB0C,  // Procedure call
  //     CSR_MRET            = 12'hB0D,  // Procedure Return
  //     CSR_MMIS_PREDICT    = 12'hB0E,  // Branch mis-predicted
  //     CSR_MSB_FULL        = 12'hB0F,  // Scoreboard full
  //     CSR_MIF_EMPTY       = 12'hB10,  // instruction fetch queue empty
  //     CSR_MHPM_COUNTER_17 = 12'hB11,  // reserved
  //     CSR_MHPM_COUNTER_18 = 12'hB12,  // reserved
  //     CSR_MHPM_COUNTER_19 = 12'hB13,  // reserved
  //     CSR_MHPM_COUNTER_20 = 12'hB14,  // reserved
  //     CSR_MHPM_COUNTER_21 = 12'hB15,  // reserved
  //     CSR_MHPM_COUNTER_22 = 12'hB16,  // reserved
  //     CSR_MHPM_COUNTER_23 = 12'hB17,  // reserved
  //     CSR_MHPM_COUNTER_24 = 12'hB18,  // reserved
  //     CSR_MHPM_COUNTER_25 = 12'hB19,  // reserved
  //     CSR_MHPM_COUNTER_26 = 12'hB1A,  // reserved
  //     CSR_MHPM_COUNTER_27 = 12'hB1B,  // reserved
  //     CSR_MHPM_COUNTER_28 = 12'hB1C,  // reserved
  //     CSR_MHPM_COUNTER_29 = 12'hB1D,  // reserved
  //     CSR_MHPM_COUNTER_30 = 12'hB1E,  // reserved
  //     CSR_MHPM_COUNTER_31 = 12'hB1F,  // reserved
  //     // Cache Control (platform specifc)
  //     CSR_DCACHE         = 12'h701,
  //     CSR_ICACHE         = 12'h700,
  //     // Triggers
  //     CSR_TSELECT        = 12'h7A0,
  //     CSR_TDATA1         = 12'h7A1,
  //     CSR_TDATA2         = 12'h7A2,
  //     CSR_TDATA3         = 12'h7A3,
  //     CSR_TINFO          = 12'h7A4,
  //     // Debug CSR
  //     CSR_DCSR           = 12'h7b0,
  //     CSR_DPC            = 12'h7b1,
  //     CSR_DSCRATCH0      = 12'h7b2, // optional
  //     CSR_DSCRATCH1      = 12'h7b3, // optional
  //     // Counters and Timers (User Mode - R/O Shadows)
  //     CSR_CYCLE          = 12'hC00,
  //     CSR_TIME           = 12'hC01,
  //     CSR_INSTRET        = 12'hC02,
  //     // Performance counters (User Mode - R/O Shadows)
  //     CSR_L1_ICACHE_MISS = 12'hC03,  // L1 Instr Cache Miss
  //     CSR_L1_DCACHE_MISS = 12'hC04,  // L1 Data Cache Miss
  //     CSR_ITLB_MISS      = 12'hC05,  // ITLB Miss
  //     CSR_DTLB_MISS      = 12'hC06,  // DTLB Miss
  //     CSR_LOAD           = 12'hC07,  // Loads
  //     CSR_STORE          = 12'hC08,  // Stores
  //     CSR_EXCEPTION      = 12'hC09,  // Taken exceptions
  //     CSR_EXCEPTION_RET  = 12'hC0A,  // Exception return
  //     CSR_BRANCH_JUMP    = 12'hC0B,  // Software change of PC
  //     CSR_CALL           = 12'hC0C,  // Procedure call
  //     CSR_RET            = 12'hC0D,  // Procedure Return
  //     CSR_MIS_PREDICT    = 12'hC0E,  // Branch mis-predicted
  //     CSR_SB_FULL        = 12'hC0F,  // Scoreboard full
  //     CSR_IF_EMPTY       = 12'hC10,  // instruction fetch queue empty
  //     CSR_HPM_COUNTER_17 = 12'hC11,  // reserved
  //     CSR_HPM_COUNTER_18 = 12'hC12,  // reserved
  //     CSR_HPM_COUNTER_19 = 12'hC13,  // reserved
  //     CSR_HPM_COUNTER_20 = 12'hC14,  // reserved
  //     CSR_HPM_COUNTER_21 = 12'hC15,  // reserved
  //     CSR_HPM_COUNTER_22 = 12'hC16,  // reserved
  //     CSR_HPM_COUNTER_23 = 12'hC17,  // reserved
  //     CSR_HPM_COUNTER_24 = 12'hC18,  // reserved
  //     CSR_HPM_COUNTER_25 = 12'hC19,  // reserved
  //     CSR_HPM_COUNTER_26 = 12'hC1A,  // reserved
  //     CSR_HPM_COUNTER_27 = 12'hC1B,  // reserved
  //     CSR_HPM_COUNTER_28 = 12'hC1C,  // reserved
  //     CSR_HPM_COUNTER_29 = 12'hC1D,  // reserved
  //     CSR_HPM_COUNTER_30 = 12'hC1E,  // reserved
  //     CSR_HPM_COUNTER_31 = 12'hC1F  // reserved
  // } csr_reg_t
  //
  // val logic [63:0] SSTATUS_UIE  = 64'h00000001
  // val logic [63:0] SSTATUS_SIE  = 64'h00000002
  // val logic [63:0] SSTATUS_SPIE = 64'h00000020
  // val logic [63:0] SSTATUS_SPP  = 64'h00000100
  // val logic [63:0] SSTATUS_FS   = 64'h00006000
  // val logic [63:0] SSTATUS_XS   = 64'h00018000
  // val logic [63:0] SSTATUS_SUM  = 64'h00040000
  // val logic [63:0] SSTATUS_MXR  = 64'h00080000
  // val logic [63:0] SSTATUS_UPIE = 64'h00000010
  // val logic [63:0] SSTATUS_UXL  = 64'h0000000300000000
  // val logic [63:0] SSTATUS64_SD = 64'h8000000000000000
  // val logic [63:0] SSTATUS32_SD = 64'h80000000
  //
  // val logic [63:0] MSTATUS_UIE  = 64'h00000001
  // val logic [63:0] MSTATUS_SIE  = 64'h00000002
  // val logic [63:0] MSTATUS_HIE  = 64'h00000004
  // val logic [63:0] MSTATUS_MIE  = 64'h00000008
  // val logic [63:0] MSTATUS_UPIE = 64'h00000010
  // val logic [63:0] MSTATUS_SPIE = 64'h00000020
  // val logic [63:0] MSTATUS_HPIE = 64'h00000040
  // val logic [63:0] MSTATUS_MPIE = 64'h00000080
  // val logic [63:0] MSTATUS_SPP  = 64'h00000100
  // val logic [63:0] MSTATUS_HPP  = 64'h00000600
  // val logic [63:0] MSTATUS_MPP  = 64'h00001800
  // val logic [63:0] MSTATUS_FS   = 64'h00006000
  // val logic [63:0] MSTATUS_XS   = 64'h00018000
  // val logic [63:0] MSTATUS_MPRV = 64'h00020000
  // val logic [63:0] MSTATUS_SUM  = 64'h00040000
  // val logic [63:0] MSTATUS_MXR  = 64'h00080000
  // val logic [63:0] MSTATUS_TVM  = 64'h00100000
  // val logic [63:0] MSTATUS_TW   = 64'h00200000
  // val logic [63:0] MSTATUS_TSR  = 64'h00400000
  // val logic [63:0] MSTATUS32_SD = 64'h80000000
  // val logic [63:0] MSTATUS_UXL  = 64'h0000000300000000
  // val logic [63:0] MSTATUS_SXL  = 64'h0000000C00000000
  // val logic [63:0] MSTATUS64_SD = 64'h8000000000000000

  // typedef enum logic [2:0] {
  //     CSRRW  = 3'h1,
  //     CSRRS  = 3'h2,
  //     CSRRC  = 3'h3,
  //     CSRRWI = 3'h5,
  //     CSRRSI = 3'h6,
  //     CSRRCI = 3'h7
  // } csr_op_t
  //
  // // decoded CSR address
  // class extends Bundle {
  //     logic [1:0]  rw
  //     priv_lvl_t   priv_lvl
  //     logic  [7:0] address
  // } csr_addr_t
  //
  // typedef union packed {
  //     csr_reg_t   address
  //     csr_addr_t  csr_decode
  // } csr_t
  //
  // // Floating-Point control and status register (32-bit!)
  // class extends Bundle {
  //     logic [31:15] reserved;  // reserved for L extension, return 0 otherwise
  //     logic [6:0]   fprec;     // div/sqrt precision control
  //     logic [2:0]   frm;       // float rounding mode
  //     logic [4:0]   fflags;    // float exception flags
  // } fcsr_t
  //
  // // -----
  // // Debug
  // // -----
  // class extends Bundle {
  //     logic [31:28]     xdebugver
  //     logic [27:16]     zero2
  //     logic             ebreakm
  //     logic             zero1
  //     logic             ebreaks
  //     logic             ebreaku
  //     logic             stepie
  //     logic             stopcount
  //     logic             stoptime
  //     logic [8:6]       cause
  //     logic             zero0
  //     logic             mprven
  //     logic             nmip
  //     logic             step
  //     priv_lvl_t        prv
  // } dcsr_t
  //
  // // Instruction Generation *incomplete*
  // function automatic logic [31:0] jal (logic[4:0] rd, logic [20:0] imm)
  //     // OpCode Jal
  //     return {imm[20], imm[10:1], imm[11], imm[19:12], rd, 7'h6f}
  // endfunction
  //
  // function automatic logic [31:0] jalr (logic[4:0] rd, logic[4:0] rs1, logic [11:0] offset)
  //     // OpCode Jal
  //     return {offset[11:0], rs1, 3'b0, rd, 7'h67}
  // endfunction
  //
  // function automatic logic [31:0] andi (logic[4:0] rd, logic[4:0] rs1, logic [11:0] imm)
  //     // OpCode andi
  //     return {imm[11:0], rs1, 3'h7, rd, 7'h13}
  // endfunction
  //
  // function automatic logic [31:0] slli (logic[4:0] rd, logic[4:0] rs1, logic [5:0] shamt)
  //     // OpCode slli
  //     return {6'b0, shamt[5:0], rs1, 3'h1, rd, 7'h13}
  // endfunction
  //
  // function automatic logic [31:0] srli (logic[4:0] rd, logic[4:0] rs1, logic [5:0] shamt)
  //     // OpCode srli
  //     return {6'b0, shamt[5:0], rs1, 3'h5, rd, 7'h13}
  // endfunction
  //
  // function automatic logic [31:0] load (logic [2:0] size, logic[4:0] dest, logic[4:0] base, logic [11:0] offset)
  //     // OpCode Load
  //     return {offset[11:0], base, size, dest, 7'h03}
  // endfunction
  //
  // function automatic logic [31:0] auipc (logic[4:0] rd, logic [20:0] imm)
  //     // OpCode Auipc
  //     return {imm[20], imm[10:1], imm[11], imm[19:12], rd, 7'h17}
  // endfunction
  //
  // function automatic logic [31:0] store (logic [2:0] size, logic[4:0] src, logic[4:0] base, logic [11:0] offset)
  //     // OpCode Store
  //     return {offset[11:5], src, base, size, offset[4:0], 7'h23}
  // endfunction
  //
  // function automatic logic [31:0] float_load (logic [2:0] size, logic[4:0] dest, logic[4:0] base, logic [11:0] offset)
  //     // OpCode Load
  //     return {offset[11:0], base, size, dest, 7'b00_001_11}
  // endfunction
  //
  // function automatic logic [31:0] float_store (logic [2:0] size, logic[4:0] src, logic[4:0] base, logic [11:0] offset)
  //     // OpCode Store
  //     return {offset[11:5], src, base, size, offset[4:0], 7'b01_001_11}
  // endfunction
  //
  // function automatic logic [31:0] csrw (csr_reg_t csr, logic[4:0] rs1)
  //                      // CSRRW, rd, OpCode System
  //     return {csr, rs1, 3'h1, 5'h0, 7'h73}
  // endfunction
  //
  // function automatic logic [31:0] csrr (csr_reg_t csr, logic [4:0] dest)
  //               // rs1, CSRRS, rd, OpCode System
  //     return {csr, 5'h0, 3'h2, dest, 7'h73}
  // endfunction
  //
  // function automatic logic [31:0] ebreak ()
  //     return 32'h00100073
  // endfunction
  //
  // function automatic logic [31:0] nop ()
  //     return 32'h00000013
  // endfunction
  //
  // function automatic logic [31:0] illegal ()
  //     return 32'h00000000
  // endfunction
  //
  //
  // // trace log compatible to spikes commit log feature
  // // pragma translate_off
  // function string spikeCommitLog(logic [63:0] pc, priv_lvl_t priv_lvl, logic [31:0] instr, logic [4:0] rd, logic [63:0] result, logic rd_fpr)
  //     string rd_s
  //     string instr_word
  //
  //     automatic string rf_s = rd_fpr ? "f" : "x"
  //
  //     if (instr[1:0] != 2'b11) begin
  //       instr_word = $sformatf("(0x%h)", instr[15:0])
  //     end else begin
  //       instr_word = $sformatf("(0x%h)", instr)
  //     end
  //
  //     if (rd < 10) rd_s = $sformatf("%s %0d", rf_s, rd)
  //     else rd_s = $sformatf("%s%0d", rf_s, rd)
  //
  //     if (rd_fpr || rd != 0) begin
  //         // 0 0x0000000080000118 (0xeecf8f93) x31 0x0000000080004000
  //         return $sformatf("%d 0x%h %s %s 0x%h\n", priv_lvl, pc, instr_word, rd_s, result)
  //     end else begin
  //         // 0 0x000000008000019c (0x0040006f)
  //         return $sformatf("%d 0x%h %s\n", priv_lvl, pc, instr_word)
  //     end
  // endfunction
  // // pragma translate_on
  //
  // class extends Bundle {
  //     byte priv
  //     longint unsigned pc
  //     byte is_fp
  //     byte rd
  //     longint unsigned data
  //     int unsigned instr
  //     byte was_exception
  // } commit_log_t

}
