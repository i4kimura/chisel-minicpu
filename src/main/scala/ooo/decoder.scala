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
import ariane_fu_op._
import ariane_fu_t._
import ariane_if_id._


object ariane_fu_op {
  // ---------------
  // EX Stage
  // ---------------
  type fu_op_t = UInt
  // basic ALU op
  val ADD  = 0.U(7.W)
  val SUB  = 1.U(7.W)
  val ADDW = 2.U(7.W)
  val SUBW = 3.U(7.W)
  // logic operations
  val XORL = 4.U(7.W)
  val ORL  = 5.U(7.W)
  val ANDL = 6.U(7.W)
  // shifts
  val SRA  =  7.U(7.W)
  val SRL  =  8.U(7.W)
  val SLL  =  9.U(7.W)
  val SRLW = 10.U(7.W)
  val SLLW = 11.U(7.W)
  val SRAW = 12.U(7.W)
  // comparisons
  val LTS = 13.U(7.W)
  val LTU = 14.U(7.W)
  val GES = 15.U(7.W)
  val GEU = 16.U(7.W)
  val EQ  = 17.U(7.W)
  val NE  = 18.U(7.W)
  // jumps
  val JALR   = 19.U(7.W)
  val BRANCH = 20.U(7.W)
  // set lower than operations
  val SLTS = 21.U(7.W)
  val SLTU = 22.U(7.W)
  // CSR functions
  val MRET       = 23.U(7.W)
  val SRET       = 24.U(7.W)
  val DRET       = 25.U(7.W)
  val ECALL      = 26.U(7.W)
  val WFI        = 27.U(7.W)
  val FENCE      = 28.U(7.W)
  val FENCE_I    = 29.U(7.W)
  val SFENCE_VMA = 30.U(7.W)
  val CSR_WRITE  = 31.U(7.W)
  val CSR_READ   = 32.U(7.W)
  val CSR_SET    = 33.U(7.W)
  val CSR_CLEAR  = 34.U(7.W)
  // LSU functions
  val LD  = 35.U(7.W)
  val SD  = 36.U(7.W)
  val LW  = 37.U(7.W)
  val LWU = 38.U(7.W)
  val SW  = 39.U(7.W)
  val LH  = 40.U(7.W)
  val LHU = 41.U(7.W)
  val SH  = 42.U(7.W)
  val LB  = 43.U(7.W)
  val SB  = 44.U(7.W)
  val LBU = 45.U(7.W)
  // Atomic Memory Operations
  val AMO_LRW    = 46.U(7.W)
  val AMO_LRD    = 47.U(7.W)
  val AMO_SCW    = 48.U(7.W)
  val AMO_SCD    = 49.U(7.W)
  val AMO_SWAPW  = 50.U(7.W)
  val AMO_ADDW   = 51.U(7.W)
  val AMO_ANDW   = 52.U(7.W)
  val AMO_ORW    = 53.U(7.W)
  val AMO_XORW   = 54.U(7.W)
  val AMO_MAXW   = 55.U(7.W)
  val AMO_MAXWU  = 56.U(7.W)
  val AMO_MINW   = 57.U(7.W)
  val AMO_MINWU  = 58.U(7.W)
  val AMO_SWAPD  = 59.U(7.W)
  val AMO_ADDD   = 60.U(7.W)
  val AMO_ANDD   = 61.U(7.W)
  val AMO_ORD    = 62.U(7.W)
  val AMO_XORD   = 63.U(7.W)
  val AMO_MAXD   = 64.U(7.W)
  val AMO_MAXDU  = 65.U(7.W)
  val AMO_MIND   = 66.U(7.W)
  val AMO_MINDU  = 67.U(7.W)
  // Multiplications
  val MUL    = 68.U(7.W)
  val MULH   = 69.U(7.W)
  val MULHU  = 70.U(7.W)
  val MULHSU = 71.U(7.W)
  val MULW   = 72.U(7.W)
  // Divisions
  val DIV   = 73.U(7.W)
  val DIVU  = 74.U(7.W)
  val DIVW  = 75.U(7.W)
  val DIVUW = 76.U(7.W)
  val REM   = 77.U(7.W)
  val REMU  = 78.U(7.W)
  val REMW  = 79.U(7.W)
  val REMUW = 80.U(7.W)
  // Floating-Point Load and Store Instructions
  val FLD = 81.U(7.W)
  val FLW = 82.U(7.W)
  val FLH = 83.U(7.W)
  val FLB = 84.U(7.W)
  val FSD = 85.U(7.W)
  val FSW = 86.U(7.W)
  val FSH = 87.U(7.W)
  val FSB = 88.U(7.W)
  // Floating-Point Computational Instructions
  val FADD     = 89.U(7.W)
  val FSUB     = 90.U(7.W)
  val FMUL     = 91.U(7.W)
  val FDIV     = 92.U(7.W)
  val FMIN_MAX = 93.U(7.W)
  val FSQRT    = 94.U(7.W)
  val FMADD    = 95.U(7.W)
  val FMSUB    = 96.U(7.W)
  val FNMSUB   = 97.U(7.W)
  val FNMADD   = 98.U(7.W)
  // Floating-Point Conversion and Move Instructions
  val FCVT_F2I =  99.U(7.W)
  val FCVT_I2F = 100.U(7.W)
  val FCVT_F2F = 101.U(7.W)
  val FSGNJ    = 102.U(7.W)
  val FMV_F2X  = 103.U(7.W)
  val FMV_X2F  = 104.U(7.W)
  // Floating-Point Compare Instructions
  val FCMP = 105.U(7.W)
  // Floating-Point Classify Instruction
  val FCLASS = 106.U(7.W)
  // Vectorial Floating-Point Instructions that don't directly map onto the scalar ones
  val VFMIN     = 107.U(7.W)
  val VFMAX     = 108.U(7.W)
  val VFSGNJ    = 109.U(7.W)
  val VFSGNJN   = 110.U(7.W)
  val VFSGNJX   = 111.U(7.W)
  val VFEQ      = 112.U(7.W)
  val VFNE      = 113.U(7.W)
  val VFLT      = 114.U(7.W)
  val VFGE      = 115.U(7.W)
  val VFLE      = 116.U(7.W)
  val VFGT      = 117.U(7.W)
  val VFCPKAB_S = 118.U(7.W)
  val VFCPKCD_S = 119.U(7.W)
  val VFCPKAB_D = 120.U(7.W)
  val VFCPKCD_D = 121.U(7.W)
  object fu_op_t extends UIntFactory {
    override def apply(): UInt = apply(7.W)
  }
}


object ariane_fu_t {
  type fu_t = UInt
  val NONE      :fu_t = 0.U(4.W)
  val LOAD      :fu_t = 1.U(4.W)
  val STORE     :fu_t = 2.U(4.W)
  val ALU       :fu_t = 3.U(4.W)
  val CTRL_FLOW :fu_t = 4.U(4.W)
  val MULT      :fu_t = 5.U(4.W)
  val CSR       :fu_t = 6.U(4.W)
  val FPU       :fu_t = 7.U(4.W)
  val FPU_VEC   :fu_t = 8.U(4.W)
  object fu_t extends UIntFactory {
    override def apply(): UInt = apply(4.W)
  }
}


import ariane_fu_op._
import ariane_fu_t._

object ariane_pkg
{

  def log2(x: Int): Int = (scala.math.log(x) / scala.math.log(2)).asInstanceOf[Int]

  // ---------------
  // Global Config
  // ---------------
  // This is the new user config interface system. If you need to parameterize something
  // within Ariane add a field here and assign a default value to the config. Please make
  // sure to add a propper parameter check to the `check_cfg` function.
  val NrMaxRules = 16;

  abstract class ariane_cfg_t {
    val RASDepth  : Int
    val BTBEntries: Int
    val BHTEntries: Int
    // PMAs
    val NrNonIdempotentRules  : Int                        // Number of non idempotent rules
    val NonIdempotentAddrBase : Array[Int]                 // base which needs to match
    val NonIdempotentLength   : Array[Int]   // bit mask which bits to consider when matching the rule
    val NrExecuteRegionRules  : Int            // Number of regions which have execute property
    val ExecuteRegionAddrBase : Array[Int]   // base which needs to match
    val ExecuteRegionLength   : Array[Int]   // bit mask which bits to consider when matching the rule
    val NrCachedRegionRules   : Int            // Number of regions which have cached property
    val CachedRegionAddrBase  : Array[Int] // base which needs to match
    val CachedRegionLength    : Array[Int] // bit mask which bits to consider when matching the rule
                                                           // cache config
    val Axi64BitCompliant : Boolean // set to 1 when using in conjunction with 64bit AXI bus adapter
    val SwapEndianess     : Int     // set to 1 to swap endianess inside L1.5 openpiton adapter
                                    //
    val DmBaseAddress : Int         // offset of the debug module
  }

  def range_check(base: Int, len: Int, address: UInt) : Bool = {
    // if len is a power of two, and base is properly aligned, this check could be simplified
    return (address >= base.asUInt) && (address < (base.asUInt+len.asUInt))
  }

  def is_inside_nonidempotent_regions (Cfg: ariane_cfg_t, address: UInt) : Bool = {
    val pass = Wire(UInt(NrMaxRules.W))
    for (k <- 0 until Cfg.NrNonIdempotentRules) {
      pass(k) := range_check(Cfg.NonIdempotentAddrBase(k), Cfg.NonIdempotentLength(k), address)
    }
    return pass.orR
  }

  def is_inside_execute_regions (Cfg: ariane_cfg_t, address: UInt) : Bool = {
    // if we don't specify any region we assume everything is accessible
    val pass = Wire(UInt(NrMaxRules.W))
    for (k <- 0 until Cfg.NrExecuteRegionRules) {
      pass(k) := range_check(Cfg.ExecuteRegionAddrBase(k), Cfg.ExecuteRegionLength(k), address)
    }
    return pass.orR
  }

  def is_inside_cacheable_regions (Cfg: ariane_cfg_t, address:UInt) : Bool = {
    val pass = Wire(Vec(Cfg.NrCachedRegionRules, Bool()))
    for (k <- 0 until Cfg.NrCachedRegionRules) {
      pass(k) := range_check(Cfg.CachedRegionAddrBase(k), Cfg.CachedRegionLength(k), address)
    }
    return pass.foldLeft(false.B)(_|_)
  }

  val CONFIG_L1I_CACHELINE_WIDTH: Int = 128
  val CONFIG_L1I_ASSOCIATIVITY  : Int = 4
  val CONFIG_L1I_SIZE           : Int = 16*1024
  val CONFIG_L1D_CACHELINE_WIDTH: Int = 128
  val CONFIG_L1D_ASSOCIATIVITY  : Int = 8
  val CONFIG_L1D_SIZE           : Int = 32*1024

  // I$
  val ICACHE_LINE_WIDTH : Int = CONFIG_L1I_CACHELINE_WIDTH
  val ICACHE_SET_ASSOC  : Int = CONFIG_L1I_ASSOCIATIVITY
  val ICACHE_INDEX_WIDTH: Int = log2(CONFIG_L1I_SIZE / ICACHE_SET_ASSOC)
  val ICACHE_TAG_WIDTH  : Int = 56 - ICACHE_INDEX_WIDTH
  // D$
  val DCACHE_LINE_WIDTH : Int = CONFIG_L1D_CACHELINE_WIDTH
  val DCACHE_SET_ASSOC  : Int = CONFIG_L1D_ASSOCIATIVITY
  val DCACHE_INDEX_WIDTH: Int = log2(CONFIG_L1D_SIZE / DCACHE_SET_ASSOC)
  val DCACHE_TAG_WIDTH  : Int = 56 - DCACHE_INDEX_WIDTH

  // --------------------
  // Atomics (amot_t)
  // --------------------
  val AMO_NONE = Integer.parseInt("0000", 2).U
  val AMO_LR   = Integer.parseInt("0001", 2).U
  val AMO_SC   = Integer.parseInt("0010", 2).U
  val AMO_SWAP = Integer.parseInt("0011", 2).U
  val AMO_ADD  = Integer.parseInt("0100", 2).U
  val AMO_AND  = Integer.parseInt("0101", 2).U
  val AMO_OR   = Integer.parseInt("0110", 2).U
  val AMO_XOR  = Integer.parseInt("0111", 2).U
  val AMO_MAX  = Integer.parseInt("1000", 2).U
  val AMO_MAXU = Integer.parseInt("1001", 2).U
  val AMO_MIN  = Integer.parseInt("1010", 2).U
  val AMO_MINU = Integer.parseInt("1011", 2).U
  val AMO_CAS1 = Integer.parseInt("1100", 2).U  // unused, not part of riscv spec, but provided in OpenPiton
  val AMO_CAS2 = Integer.parseInt("1101", 2).U  // unused, not part of riscv spec, but provided in OpenPiton

  // D$ data requests
  class dcache_req_i_t extends Bundle {
    val address_index = UInt(DCACHE_INDEX_WIDTH.W)
    val address_tag   = UInt(DCACHE_TAG_WIDTH.W)
    val data_wdata    = UInt(64.W)
    val data_req      = Bool()
    val data_we       = Bool()
    val data_be       = UInt(8.W)
    val data_size     = UInt(2.W)
    val kill_req      = Bool()
    val tag_valid     = Bool()
  }

  class dcache_req_o_t extends Bundle {
    val data_gnt    = Bool()
    val data_rvalid = Bool()
    val data_rdata  = UInt(64.W)
  }


  //     val ariane_cfg_t ArianeDefaultConfig = '{
  //       RASDepth: 2,
  //       BTBEntries: 32,
  //       BHTEntries: 128,
  //       // idempotent region
  //       NrNonIdempotentRules: 2,
  //       NonIdempotentAddrBase: {64'b0, 64'b0},
  //       NonIdempotentLength:   {64'b0, 64'b0},
  //       NrExecuteRegionRules: 3,
  //       //                      DRAM,          Boot ROM,   Debug Module
  //       ExecuteRegionAddrBase: {64'h8000_0000, 64'h1_0000, 64'h0},
  //       ExecuteRegionLength:   {64'h40000000,  64'h10000,  64'h1000},
  //       // cached region
  //       NrCachedRegionRules:    1,
  //       CachedRegionAddrBase:  {64'h8000_0000},
  //       CachedRegionLength:    {64'h40000000},
  //       //  cache config
  //       Axi64BitCompliant:      1'b1,
  //       SwapEndianess:          1'b0,
  //       // debug
  //       DmBaseAddress:          64'h0
  //     }
  //
  //     // Function being called to check parameters
  //     function automatic void check_cfg (ariane_cfg_t Cfg)
  //       // pragma translate_off
  //       `ifndef VERILATOR
  //         assert(Cfg.RASDepth > 0)
  //         assert(2**$clog2(Cfg.BTBEntries)  == Cfg.BTBEntries)
  //         assert(2**$clog2(Cfg.BHTEntries)  == Cfg.BHTEntries)
  //         assert(Cfg.NrNonIdempotentRules <= NrMaxRules)
  //         assert(Cfg.NrExecuteRegionRules <= NrMaxRules)
  //         assert(Cfg.NrCachedRegionRules  <= NrMaxRules)
  //       `endif
  //       // pragma translate_on
  //     endfunction
  //


  // TODO: Slowly move those parameters to the new system.
  val NR_SB_ENTRIES = 8 // number of scoreboard entries
  val TRANS_ID_BITS = log2Up (NR_SB_ENTRIES) // depending on the number of scoreboard entries we need that many bits
                                             // to uniquely identify the entry in the scoreboard
  val ASID_WIDTH    = 1
  val BITS_SATURATION_COUNTER = 2
  val NR_COMMIT_PORTS = 2

  val ENABLE_RENAME = false.B

  val ISSUE_WIDTH = 1
  // amount of pipeline registers inserted for load/store return path
  // this can be tuned to trade-off IPC vs. cycle time
  val NR_LOAD_PIPE_REGS = 1
  val NR_STORE_PIPE_REGS = 0

  // depth of store-buffers, this needs to be a power of two
  val DEPTH_SPEC   = 4

  // `ifdef WT_DCACHE
  // in this case we can use a small commit queue since we have a write buffer in the dcache
  // we could in principle do without the commit queue in this case, but the timing degrades if we do that due
  // to longer paths into the commit stage
  val DEPTH_COMMIT = 4
  // `else
  // // allocate more space for the commit buffer to be on the save side, this needs to be a power of two
  // val DEPTH_COMMIT = 8
  // `endif


  // ifdef PITON_ARIANE
  // Floating-point extensions configuration
  val RVF = true // Is F extension enabled
  val RVD = true // Is D extension enabled
  // else
  // Floating-point extensions configuration
  // val RVF = true // Is F extension enabled
  // val RVD = true // Is D extension enabled
  // endif
  val RVA = true // Is A extension enabled

  // Transprecision floating-point extensions configuration
  val XF16    = false // Is half-precision float extension (Xf16) enabled
  val XF16ALT = false // Is alternative half-precision float extension (Xf16alt) enabled
  val XF8     = false // Is quarter-precision float extension (Xf8) enabled
  val XFVEC   = false // Is vectorial float extension (Xfvec) enabled

  // Transprecision float unit
  val LAT_COMP_FP32    = 2
  val LAT_COMP_FP64    = 3
  val LAT_COMP_FP16    = 1
  val LAT_COMP_FP16ALT = 1
  val LAT_COMP_FP8     = 1
  val LAT_DIVSQRT      = 2
  val LAT_NONCOMP      = 1
  val LAT_CONV         = 2

  // --------------------------------------
  // vvvv Don't change these by hand! vvvv
  val FP_PRESENT = RVF | RVD | XF16 | XF16ALT | XF8

  // Length of widest floating-point format
  val FLEN =      if(RVD    ) 64 // D ext.
             else if(RVF    ) 32 // F ext.
             else if(XF16   ) 16 // Xf16 ext.
             else if(XF16ALT) 16 // Xf16alt ext.
             else if(XF8    ) 8  // Xf8 ext.
             else             0  // Unused in case of no FP

  val NSX = XF16 | XF16ALT | XF8 | XFVEC; // Are non-standard extensions present?

  val RVFVEC     = RVF     & XFVEC & (FLEN>32) // FP32 vectors available if vectors and larger fmt enabled
  val XF16VEC    = XF16    & XFVEC & (FLEN>16) // FP16 vectors available if vectors and larger fmt enabled
  val XF16ALTVEC = XF16ALT & XFVEC & (FLEN>16) // FP16ALT vectors available if vectors and larger fmt enabled
  val XF8VEC     = XF8     & XFVEC & (FLEN>8 ) // FP8 vectors available if vectors and larger fmt enabled

  // ^^^^ until here ^^^^
  // ---------------------

  val ARIANE_MARCHID = 3.U(64.W)

  var ISA_CODE =        (RVA.asInstanceOf[Int] <<  0)  // A - Atomic Instructions extension
  ISA_CODE = ISA_CODE | (1                     <<  2)  // C - Compressed extension
  ISA_CODE = ISA_CODE | (RVD.asInstanceOf[Int] <<  3)  // D - Double precsision floating-point extension
  ISA_CODE = ISA_CODE | (RVF.asInstanceOf[Int] <<  5)  // F - Single precsision floating-point extension
  ISA_CODE = ISA_CODE | (1                     <<  8)  // I - RV32I/64I/128I base ISA
  ISA_CODE = ISA_CODE | (1                     << 12)  // M - Integer Multiply/Divide extension
  ISA_CODE = ISA_CODE | (0                     << 13)  // N - User level interrupts supported
  ISA_CODE = ISA_CODE | (1                     << 18)  // S - Supervisor mode implemented
  ISA_CODE = ISA_CODE | (1                     << 20)  // U - User mode implemented
  ISA_CODE = ISA_CODE | (NSX.asInstanceOf[Int] << 23)  // X - Non-standard extensions present
  ISA_CODE = ISA_CODE | (1                     << 63)  // RV64


  // 32 registers + 1 bit for re-naming = 6
  //
  //     // static debug hartinfo
  //     val dm::hartinfo_t DebugHartInfo = '{
  //                                                 zero1:        '0,
  //                                                 nscratch:      2, // Debug module needs at least two scratch regs
  //                                                 zero0:        '0,
  //                                                 dataaccess: true.B, // data registers are memory mapped in the debugger
  //                                                 datasize: dm::DataCount,
  //                                                 dataaddr: dm::DataAddr
  //                                               }
  //
  //     // enables a commit log which matches spikes commit log format for easier trace comparison
  //     val bit ENABLE_SPIKE_COMMIT_LOG = true.B

  // ------------- Dangerouse -------------
  // if set to zero a flush will not invalidate the cache-lines, in a single core environment
  // where coherence is not necessary this can improve performance. This needs to be switched on
  // when more than one core is in a system
  val INVALIDATE_ON_FLUSH = true
  // `ifdef SPIKE_TANDEM
  //     // enable performance cycle counter, if set to zero mcycle will be incremented
  //     // with instret (non RISC-V conformal)
  //     val bit ENABLE_CYCLE_COUNT = false.B
  //     // mark WIF as nop
  //     val bit ENABLE_WFI = false.B
  //     // Spike zeros tval on all exception except memory faults
  //     val bit ZERO_TVAL = true.B
  // `else
  val ENABLE_CYCLE_COUNT = true
  val ENABLE_WFI         = true
  val ZERO_TVAL          = false
  // `endif

  //     // read mask for SSTATUS over MMSTATUS
  //     val logic [63:0] SMODE_STATUS_READ_MASK = riscv::SSTATUS_UIE
  //                                                    | riscv::SSTATUS_SIE
  //                                                    | riscv::SSTATUS_SPIE
  //                                                    | riscv::SSTATUS_SPP
  //                                                    | riscv::SSTATUS_FS
  //                                                    | riscv::SSTATUS_XS
  //                                                    | riscv::SSTATUS_SUM
  //                                                    | riscv::SSTATUS_MXR
  //                                                    | riscv::SSTATUS_UPIE
  //                                                    | riscv::SSTATUS_SPIE
  //                                                    | riscv::SSTATUS_UXL
  //                                                    | riscv::SSTATUS64_SD
  //
  //     val logic [63:0] SMODE_STATUS_WRITE_MASK = riscv::SSTATUS_SIE
  //                                                     | riscv::SSTATUS_SPIE
  //                                                     | riscv::SSTATUS_SPP
  //                                                     | riscv::SSTATUS_FS
  //                                                     | riscv::SSTATUS_SUM
  //                                                     | riscv::SSTATUS_MXR
  // ---------------
  // Fetch Stage
  // ---------------

  // leave as is (fails with >8 entries and wider fetch width)
  val FETCH_FIFO_DEPTH:Int  = 4
  val FETCH_WIDTH:Int       = 32
  // val FETCH_WIDTH:Int       = 64
  // maximum instructions we can fetch on one request (we support compressed instructions)
  val INSTR_PER_FETCH:Int = FETCH_WIDTH / 16


  // Only use struct when signals have same direction
  // exception
  class exception_t extends Bundle {
    val cause = UInt(64.W) // cause of exception
    val tval  = UInt(64.W) // additional information of causing exception (e.g.: instruction causing it),

    // address of LD/ST fault
    val valid = Bool()
  }


  // typedef enum logic [2:0] {
  //   NoCF,   // No control flow prediction
  //   Branch, // Branch
  //   Jump,   // Jump to address from immediate
  //   JumpR,  // Jump to address from registers
  //   Return  // Return Address Prediction
  // } cf_t

  // cf_t
  val NoCF   = Integer.parseInt("000", 2).U
  val Branch = Integer.parseInt("001", 2).U
  val Jump   = Integer.parseInt("010", 2).U
  val JumpR  = Integer.parseInt("011", 2).U
  val Return = Integer.parseInt("100", 2).U

  // branch-predict
  // this is the struct we get back from ex stage and we will use it to update
  // all the necessary data structures
  // bp_resolve_t
  class bp_resolve_t extends Bundle {
    val valid          = Bool()     // prediction with all its values is valid
    val pc             = UInt(64.W) // PC of predict or mis-predict
    val target_address = UInt(64.W) // target address at which to jump, or not
    val is_mispredict  = Bool()     // set if this was a mis-predict
    val is_taken       = Bool()     // branch is taken
    val cf_type        = UInt(3.W)  // Type of control flow change (cf_t)
  }

  // branchpredict scoreboard entry
  // this is the struct which we will inject into the pipeline to guide the various
  // units towards the correct branch decision and resolve
  class branchpredict_sbe_t extends Bundle {
    val cf              = UInt(3.W)    // type of control flow prediction
    val predict_address = UInt(64.W)   // target address at which to jump, or not
  }

  class btb_update_t extends Bundle {
    val valid = Bool()
    val pc = UInt(64.W)             // update at PC
    val target_address = UInt(64.W)
  }

  class btb_prediction_t extends Bundle {
    val valid = Bool()
    val target_address = UInt(64.W)
  }

  class ras_t extends Bundle {
    val valid = Bool()
    val ra = UInt(64.W)
  }

  class bht_update_t extends Bundle {
    val valid = Bool()
    val pc    = UInt(64.W)       // update at PC
    val taken = Bool()
  }

  class bht_prediction_t extends Bundle {
    val valid = Bool()
    val taken = Bool()
  }

  val EXC_OFF_RST      = "h80".U

  val SupervisorIrq = 1
  val MachineIrq = 0

  // All information needed to determine whether we need to associate an interrupt
  // with the corresponding instruction or not.
  class irq_ctrl_t extends Bundle {
    val mie     = UInt(64.W)
    val mip     = UInt(64.W)
    val mideleg = UInt(64.W)
    val sie     = Bool()
    val global_enable = Bool()
  }

  //     // ---------------
  //     // Cache config
  //     // ---------------
  //
  // // for usage in OpenPiton we have to propagate the openpiton L15 configuration from l15.h
  // `ifdef PITON_ARIANE

  // `else
  // // align to openpiton for the time being (this should be more configurable in the future)
  // // I$
  // val ICACHE_INDEX_WIDTH: Int = 12;  // in bit
  // val ICACHE_TAG_WIDTH  : Int = 44;  // in bit
  // val ICACHE_LINE_WIDTH : Int = 128; // in bit
  // val ICACHE_SET_ASSOC  : Int = 4
  // // D$
  // val DCACHE_INDEX_WIDTH: Int = 12;  // in bit
  // val DCACHE_TAG_WIDTH  : Int = 44;  // in bit
  // val DCACHE_LINE_WIDTH : Int = 128; // in bit
  // val DCACHE_SET_ASSOC  : Int = 8
  // `endif

  //     typedef struct packed {
  //         fu_t                      fu
  //         fu_op_t                     operator
  //         logic [63:0]              operand_a
  //         logic [63:0]              operand_b
  //         logic [63:0]              imm
  //         logic [TRANS_ID_BITS-1:0] trans_id
  //     } fu_data_t
  //
  //     def is_branch (input fu_op_t op)
  //         unique case (op) inside
  //             EQ, NE, LTS, GES, LTU, GEU: return true.B
  //             default                   : return false.B; // all other ops
  //         endcase
  //     endfunction
  //
  //     // -------------------------------
  //     // Extract Src/Dst FP Reg from Op
  //     // -------------------------------
  //     def is_rs1_fpr (input fu_op_t op)
  //         if (FP_PRESENT) begin // makes function static for non-fp case
  //             unique case (op) inside
  //                 [FMUL:FNMADD],                   // Computational Operations (except ADD/SUB)
  //                 FCVT_F2I,                        // Float-Int Casts
  //                 FCVT_F2F,                        // Float-Float Casts
  //                 FSGNJ,                           // Sign Injections
  //                 FMV_F2X,                         // FPR-GPR Moves
  //                 FCMP,                            // Comparisons
  //                 FCLASS,                          // Classifications
  //                 [VFMIN:VFCPKCD_D] : return true.B; // Additional Vectorial FP ops
  //                 default           : return false.B; // all other ops
  //             endcase
  //         end else
  //             return false.B
  //     endfunction
  //
  //     def is_rs2_fpr (input fu_op_t op)
  //         if (FP_PRESENT) begin // makes function static for non-fp case
  //             unique case (op) inside
  //                 [FSD:FSB],                       // FP Stores
  //                 [FADD:FMIN_MAX],                 // Computational Operations (no sqrt)
  //                 [FMADD:FNMADD],                  // Fused Computational Operations
  //                 FCVT_F2F,                        // Vectorial F2F Conversions requrie target
  //                 [FSGNJ:FMV_F2X],                 // Sign Injections and moves mapped to SGNJ
  //                 FCMP,                            // Comparisons
  //                 [VFMIN:VFCPKCD_D] : return true.B; // Additional Vectorial FP ops
  //                 default           : return false.B; // all other ops
  //             endcase
  //         end else
  //             return false.B
  //     endfunction
  //
  //     // ternary operations encode the rs3 address in the imm field, also add/sub
  //     def is_imm_fpr (input fu_op_t op)
  //         if (FP_PRESENT) begin // makes function static for non-fp case
  //             unique case (op) inside
  //                 [FADD:FSUB],                         // ADD/SUB need inputs as Operand B/C
  //                 [FMADD:FNMADD],                      // Fused Computational Operations
  //                 [VFCPKAB_S:VFCPKCD_D] : return true.B; // Vectorial FP cast and pack ops
  //                 default               : return false.B; // all other ops
  //             endcase
  //         end else
  //             return false.B
  //     endfunction
  //
  //     def is_rd_fpr (input fu_op_t op)
  //         if (FP_PRESENT) begin // makes function static for non-fp case
  //             unique case (op) inside
  //                 [FLD:FLB],                           // FP Loads
  //                 [FADD:FNMADD],                       // Computational Operations
  //                 FCVT_I2F,                            // Int-Float Casts
  //                 FCVT_F2F,                            // Float-Float Casts
  //                 FSGNJ,                               // Sign Injections
  //                 FMV_X2F,                             // GPR-FPR Moves
  //                 [VFMIN:VFSGNJX],                     // Vectorial MIN/MAX and SGNJ
  //                 [VFCPKAB_S:VFCPKCD_D] : return true.B; // Vectorial FP cast and pack ops
  //                 default               : return false.B; // all other ops
  //             endcase
  //         end else
  //             return false.B
  //     endfunction
  //
  //     def is_amo (fu_op_t op)
  //         case (op) inside
  //             [AMO_LRW:AMO_MINDU]: begin
  //                 return true.B
  //             end
  //             default: return false.B
  //         endcase
  //     endfunction
  //
  //     typedef struct packed {
  //         logic                     valid
  //         logic [63:0]              vaddr
  //         logic [63:0]              data
  //         logic [7:0]               be
  //         fu_t                      fu
  //         fu_op_t                     operator
  //         logic [TRANS_ID_BITS-1:0] trans_id
  //     } lsu_ctrl_t
  //
  // ---------------
  // ID/EX/WB Stage
  // ---------------
  class scoreboard_entry_t extends Bundle {
    val pc = UInt(64.W)                  // PC of instruction
    // val trans_id = UInt(TRANS_ID_BITS.W) // this can potentially be simplified, we could index the scoreboard entry
    val trans_id = UInt(3.W) // this can potentially be simplified, we could index the scoreboard entry
                                     // with the transaction id in any case make the width more generic
    val fu  = fu_t()                     // functional unit to use
    val op  = fu_op_t()                  // operation to perform in each functional unit
    // val rs1 = UInt(REG_ADDR_SIZE.W)      // register source address 1
    // val rs2 = UInt(REG_ADDR_SIZE.W)      // register source address 2
    // val rd  = UInt(REG_ADDR_SIZE.W)      // register destination address
    val rs1 = UInt(6.W)      // register source address 1
    val rs2 = UInt(6.W)      // register source address 2
    val rd  = UInt(6.W)      // register destination address
    val result = UInt(64.W)              // for unfinished instructions this field also holds the immediate,
                                         // for unfinished floating-point that are partly encoded in rs2, this field also holds rs2
                                         // for unfinished floating-point fused operations (FMADD, FMSUB, FNMADD, FNMSUB)
                                         // this field holds the address of the third operand from the floating-point register file
    val valid    = Bool()                // is the result valid
    val use_imm  = Bool()                // should we use the immediate as operand b?
    val use_zimm = Bool()                // use zimm as operand a
    val use_pc   = Bool()                // set if we need to use the PC as operand a, PC from exception
    val ex = new exception_t()           // exception has occurred
    val bp = new branchpredict_sbe_t()   // branch predict scoreboard data structure
    val is_compressed = Bool()           // signals a compressed instructions, we need this information at the commit stage if
                                         // we want jump accordingly e.g.: +4, +2
  }


  //     typedef struct packed {
  //         logic                  valid;      // valid flag
  //         logic                  is_2M;      //
  //         logic                  is_1G;      //
  //         logic [26:0]           vpn
  //         logic [ASID_WIDTH-1:0] asid
  //         riscv::pte_t           content
  //     } tlb_update_t
  //
  //     val logic [3:0] MODE_SV39 = 4'h8
  //     val logic [3:0] MODE_OFF = 4'h0
  //
  //     // Bits required for representation of physical address space as 4K pages
  //     // (e.g. 27*4K == 39bit address space).
  //     val PPN4K_WIDTH = 38
  //

  // ----------------------
  // cache request ports
  // ----------------------
  // I$ address translation requests
  class icache_areq_i_t extends Bundle {
    val fetch_valid     = Bool()            // address translation valid
    val fetch_paddr     = UInt(64.W)        // physical address in
    val fetch_exception = new exception_t() // exception occurred during fetch
  }

  class icache_areq_o_t extends Bundle {
    val fetch_req   = Bool()       // address translation request
    val fetch_vaddr = UInt(64.W)   // virtual address out
  }

  // I$ data requests
  class icache_dreq_i_t extends Bundle {
    val req     = Bool()     // we request a new word
    val kill_s1 = Bool()     // kill the current request
    val kill_s2 = Bool()     // kill the last request
    val vaddr   = UInt(64.W) // 1st cycle: 12 bit index is taken for lookup
  }

  class icache_dreq_o_t extends Bundle {
    val ready = Bool()              // icache is ready
    val valid = Bool()              // signals a valid read
    val data  = UInt(FETCH_WIDTH.W) // 2+ cycle out: tag
    val vaddr = UInt(64.W)          // virtual address out
    val ex    = new exception_t()   // we've encountered an exception
  }

  // AMO request going to cache. this request is unconditionally valid as soon
  // as request goes high.
  // Furthermore, those signals are kept stable until the response indicates
  // completion by asserting ack.
  class amo_req_t extends Bundle {
    val req       = Bool()       // this request is valid
    val amo_op    = UInt(4.W)    // atomic memory operation to perform
    val size      = UInt(2.W)    // 2'b10 --> word operation, 2'b11 --> double word operation
    val operand_a = UInt(64.W)   // address
    val operand_b = UInt(64.W)   // data as layuoted in the register
  }

  // AMO response coming from cache.
  class amo_resp_t extends Bundle {
    val ack = Bool()         // response is valid
    val result = UInt(64.W)  // sign-extended, result
  }



  // ----------------------
  // Arithmetic Functions
  // ----------------------
  def sext32 (operand: UInt): UInt = {
    return Cat(VecInit(Seq.fill(32)(operand(31))).asUInt, operand(31, 0))
  }

  // ----------------------
  // Immediate functions
  // ----------------------
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

  //     // ----------------------
  //     // LSU Functions
  //     // ----------------------
  //     // align data to address e.g.: shift data to be naturally 64
  //     def [63:0] data_align (logic [2:0] addr, logic [63:0] data)
  //         case (addr)
  //             3'b000: return data
  //             3'b001: return {data[55:0], data[63:56]}
  //             3'b010: return {data[47:0], data[63:48]}
  //             3'b011: return {data[39:0], data[63:40]}
  //             3'b100: return {data[31:0], data[63:32]}
  //             3'b101: return {data[23:0], data[63:24]}
  //             3'b110: return {data[15:0], data[63:16]}
  //             3'b111: return {data[7:0],  data[63:8]}
  //         endcase
  //         return data
  //     endfunction
  //
  //     // generate byte enable mask
  //     def [7:0] be_gen(logic [2:0] addr, logic [1:0] size)
  //         case (size)
  //             2'b11: begin
  //                 return 8'b1111_1111
  //             end
  //             2'b10: begin
  //                 case (addr[2:0])
  //                     3'b000: return 8'b0000_1111
  //                     3'b001: return 8'b0001_1110
  //                     3'b010: return 8'b0011_1100
  //                     3'b011: return 8'b0111_1000
  //                     3'b100: return 8'b1111_0000
  //                 endcase
  //             end
  //             2'b01: begin
  //                 case (addr[2:0])
  //                     3'b000: return 8'b0000_0011
  //                     3'b001: return 8'b0000_0110
  //                     3'b010: return 8'b0000_1100
  //                     3'b011: return 8'b0001_1000
  //                     3'b100: return 8'b0011_0000
  //                     3'b101: return 8'b0110_0000
  //                     3'b110: return 8'b1100_0000
  //                 endcase
  //             end
  //             2'b00: begin
  //                 case (addr[2:0])
  //                     3'b000: return 8'b0000_0001
  //                     3'b001: return 8'b0000_0010
  //                     3'b010: return 8'b0000_0100
  //                     3'b011: return 8'b0000_1000
  //                     3'b100: return 8'b0001_0000
  //                     3'b101: return 8'b0010_0000
  //                     3'b110: return 8'b0100_0000
  //                     3'b111: return 8'b1000_0000
  //                 endcase
  //             end
  //         endcase
  //         return 8'b0
  //     endfunction
  //
  //     // ----------------------
  //     // Extract Bytes from Op
  //     // ----------------------
  //     def [1:0] extract_transfer_size(fu_op_t op)
  //         case (op)
  //             LD, SD, FLD, FSD,
  //             AMO_LRD,   AMO_SCD,
  //             AMO_SWAPD, AMO_ADDD,
  //             AMO_ANDD,  AMO_ORD,
  //             AMO_XORD,  AMO_MAXD,
  //             AMO_MAXDU, AMO_MIND,
  //             AMO_MINDU: begin
  //                 return 2'b11
  //             end
  //             LW, LWU, SW, FLW, FSW,
  //             AMO_LRW,   AMO_SCW,
  //             AMO_SWAPW, AMO_ADDW,
  //             AMO_ANDW,  AMO_ORW,
  //             AMO_XORW,  AMO_MAXW,
  //             AMO_MAXWU, AMO_MINW,
  //             AMO_MINWU: begin
  //                 return 2'b10
  //             end
  //             LH, LHU, SH, FLH, FSH: return 2'b01
  //             LB, LBU, SB, FLB, FSB: return 2'b00
  //             default:     return 2'b11
  //         endcase
  //     endfunction
  // endpackage
}

import ariane_pkg._

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
  object xs_t extends UIntFactory {
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

  // Transprecision floating-point extensions configuration
  val XF16   :Boolean = false // Is half-precision float extension (Xf16) enabled
  val XF16ALT:Boolean = false // Is alternative half-precision float extension (Xf16alt) enabled
  val XF8    :Boolean = false // Is quarter-precision float extension (Xf8) enabled
  val XFVEC  :Boolean = false // Is vectorial float extension (Xfvec) enabled

  val RVF = true // Is F extension enabled
  val RVD = true // Is D extension enabled

  val FP_PRESENT:Boolean = RVF | RVD | XF16 | XF16ALT | XF8

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
                  is(Integer.parseInt("00100", 2).U,
                     Integer.parseInt("00101", 2).U,
                     Integer.parseInt("00110", 2).U,
                     Integer.parseInt("00111", 2).U) {

                    io.instruction_o.op  := FCVT_F2F; // vfcvt.vfmt.vfmt - Vectorial FP to FP Conversion
                    io.instruction_o.rs2 := instr.asRvfType().rd; // set rs2 := rd as target vector for conversion
                    imm_select        := IIMM;     // rs2 holds part of the intruction

                    // TODO CHECK R bit for valid fmt combinations
                    // determine source format
                    illegal_instr := true.B  // Default
                    switch (instr.asRvfType().rs2(21,20)) {
                      // Only process instruction if corresponding extension is active (static)
                      is(Integer.parseInt("00", 2).U) { when (!RVFVEC.B    ) { illegal_instr := true.B } }
                      is(Integer.parseInt("01", 2).U) { when (!XF16ALTVEC.B) { illegal_instr := true.B } }
                      is(Integer.parseInt("10", 2).U) { when (!XF16VEC.B   ) { illegal_instr := true.B } }
                      is(Integer.parseInt("11", 2).U) { when (!XF8VEC.B    ) { illegal_instr := true.B } }
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
