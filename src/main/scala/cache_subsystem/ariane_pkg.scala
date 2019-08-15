package cache_subsystem

import chisel3._
import chisel3.util._
import chisel3.Bool


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
    val Axi64BitCompliant : Int     // set to 1 when using in conjunction with 64bit AXI bus adapter
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

//
//     // TODO: Slowly move those parameters to the new system.
//     val NR_SB_ENTRIES = 8; // number of scoreboard entries
//     val TRANS_ID_BITS = $clog2(NR_SB_ENTRIES); // depending on the number of scoreboard entries we need that many bits
//                                                       // to uniquely identify the entry in the scoreboard
//     val ASID_WIDTH    = 1
//     val BITS_SATURATION_COUNTER = 2
//     val NR_COMMIT_PORTS = 2
//
//     val ENABLE_RENAME = 1'b0
//
//     val ISSUE_WIDTH = 1
//     // amount of pipeline registers inserted for load/store return path
//     // this can be tuned to trade-off IPC vs. cycle time
//     val int unsigned NR_LOAD_PIPE_REGS = 1
//     val int unsigned NR_STORE_PIPE_REGS = 0
//
//     // depth of store-buffers, this needs to be a power of two
//     val int unsigned DEPTH_SPEC   = 4
//
// `ifdef WT_DCACHE
//     // in this case we can use a small commit queue since we have a write buffer in the dcache
//     // we could in principle do without the commit queue in this case, but the timing degrades if we do that due
//     // to longer paths into the commit stage
//     val int unsigned DEPTH_COMMIT = 4
// `else
//     // allocate more space for the commit buffer to be on the save side, this needs to be a power of two
//     val int unsigned DEPTH_COMMIT = 8
// `endif
//
//
// `ifdef PITON_ARIANE
//     // Floating-point extensions configuration
//     val bit RVF = 1'b1; // Is F extension enabled
//     val bit RVD = 1'b1; // Is D extension enabled
// `else
//     // Floating-point extensions configuration
//     val bit RVF = 1'b1; // Is F extension enabled
//     val bit RVD = 1'b1; // Is D extension enabled
// `endif
//     val bit RVA = 1'b1; // Is A extension enabled
//
//     // Transprecision floating-point extensions configuration
//     val bit XF16    = 1'b0; // Is half-precision float extension (Xf16) enabled
//     val bit XF16ALT = 1'b0; // Is alternative half-precision float extension (Xf16alt) enabled
//     val bit XF8     = 1'b0; // Is quarter-precision float extension (Xf8) enabled
//     val bit XFVEC   = 1'b0; // Is vectorial float extension (Xfvec) enabled
//
//     // Transprecision float unit
//     val int unsigned LAT_COMP_FP32    = 'd2
//     val int unsigned LAT_COMP_FP64    = 'd3
//     val int unsigned LAT_COMP_FP16    = 'd1
//     val int unsigned LAT_COMP_FP16ALT = 'd1
//     val int unsigned LAT_COMP_FP8     = 'd1
//     val int unsigned LAT_DIVSQRT      = 'd2
//     val int unsigned LAT_NONCOMP      = 'd1
//     val int unsigned LAT_CONV         = 'd2
//
//     // --------------------------------------
//     // vvvv Don't change these by hand! vvvv
//     val bit FP_PRESENT = RVF | RVD | XF16 | XF16ALT | XF8
//
//     // Length of widest floating-point format
//     val FLEN    = RVD     ? 64 : // D ext.
//                          RVF     ? 32 : // F ext.
//                          XF16    ? 16 : // Xf16 ext.
//                          XF16ALT ? 16 : // Xf16alt ext.
//                          XF8     ? 8 :  // Xf8 ext.
//                          0;             // Unused in case of no FP
//
//     val bit NSX = XF16 | XF16ALT | XF8 | XFVEC; // Are non-standard extensions present?
//
//     val bit RVFVEC     = RVF     & XFVEC & FLEN>32; // FP32 vectors available if vectors and larger fmt enabled
//     val bit XF16VEC    = XF16    & XFVEC & FLEN>16; // FP16 vectors available if vectors and larger fmt enabled
//     val bit XF16ALTVEC = XF16ALT & XFVEC & FLEN>16; // FP16ALT vectors available if vectors and larger fmt enabled
//     val bit XF8VEC     = XF8     & XFVEC & FLEN>8;  // FP8 vectors available if vectors and larger fmt enabled
//     // ^^^^ until here ^^^^
//     // ---------------------
//
//     val logic [63:0] ARIANE_MARCHID = 64'd3
//
//     val logic [63:0] ISA_CODE = (RVA <<  0)  // A - Atomic Instructions extension
//                                      | (1   <<  2)  // C - Compressed extension
//                                      | (RVD <<  3)  // D - Double precsision floating-point extension
//                                      | (RVF <<  5)  // F - Single precsision floating-point extension
//                                      | (1   <<  8)  // I - RV32I/64I/128I base ISA
//                                      | (1   << 12)  // M - Integer Multiply/Divide extension
//                                      | (0   << 13)  // N - User level interrupts supported
//                                      | (1   << 18)  // S - Supervisor mode implemented
//                                      | (1   << 20)  // U - User mode implemented
//                                      | (NSX << 23)  // X - Non-standard extensions present
//                                      | (1   << 63); // RV64
//
//     // 32 registers + 1 bit for re-naming = 6
//     val REG_ADDR_SIZE = 6
//     val NR_WB_PORTS = 4
//
//     // static debug hartinfo
//     val dm::hartinfo_t DebugHartInfo = '{
//                                                 zero1:        '0,
//                                                 nscratch:      2, // Debug module needs at least two scratch regs
//                                                 zero0:        '0,
//                                                 dataaccess: 1'b1, // data registers are memory mapped in the debugger
//                                                 datasize: dm::DataCount,
//                                                 dataaddr: dm::DataAddr
//                                               }
//
//     // enables a commit log which matches spikes commit log format for easier trace comparison
//     val bit ENABLE_SPIKE_COMMIT_LOG = 1'b1
//
//     // ------------- Dangerouse -------------
//     // if set to zero a flush will not invalidate the cache-lines, in a single core environment
//     // where coherence is not necessary this can improve performance. This needs to be switched on
//     // when more than one core is in a system
//     val logic INVALIDATE_ON_FLUSH = 1'b1
// `ifdef SPIKE_TANDEM
//     // enable performance cycle counter, if set to zero mcycle will be incremented
//     // with instret (non RISC-V conformal)
//     val bit ENABLE_CYCLE_COUNT = 1'b0
//     // mark WIF as nop
//     val bit ENABLE_WFI = 1'b0
//     // Spike zeros tval on all exception except memory faults
//     val bit ZERO_TVAL = 1'b1
// `else
//     val bit ENABLE_CYCLE_COUNT = 1'b1
//     val bit ENABLE_WFI = 1'b1
//     val bit ZERO_TVAL = 1'b0
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
//     // ---------------
//     // Fetch Stage
//     // ---------------
//
//     // leave as is (fails with >8 entries and wider fetch width)
//     val int unsigned FETCH_FIFO_DEPTH  = 4
//     val int unsigned FETCH_WIDTH       = 32
//     // maximum instructions we can fetch on one request (we support compressed instructions)
//     val int unsigned INSTR_PER_FETCH = FETCH_WIDTH / 16
//
//     // Only use struct when signals have same direction
//     // exception
//     typedef struct packed {
//          logic [63:0] cause; // cause of exception
//          logic [63:0] tval;  // additional information of causing exception (e.g.: instruction causing it),
//                              // address of LD/ST fault
//          logic        valid
//     } exception_t
//
//     typedef enum logic [2:0] {
//       NoCF,   // No control flow prediction
//       Branch, // Branch
//       Jump,   // Jump to address from immediate
//       JumpR,  // Jump to address from registers
//       Return  // Return Address Prediction
//     } cf_t
//
//     // branch-predict
//     // this is the struct we get back from ex stage and we will use it to update
//     // all the necessary data structures
//     // bp_resolve_t
//     typedef struct packed {
//         logic        valid;           // prediction with all its values is valid
//         logic [63:0] pc;              // PC of predict or mis-predict
//         logic [63:0] target_address;  // target address at which to jump, or not
//         logic        is_mispredict;   // set if this was a mis-predict
//         logic        is_taken;        // branch is taken
//         cf_t         cf_type;         // Type of control flow change
//     } bp_resolve_t
//
//     // branchpredict scoreboard entry
//     // this is the struct which we will inject into the pipeline to guide the various
//     // units towards the correct branch decision and resolve
//     typedef struct packed {
//         cf_t         cf;              // type of control flow prediction
//         logic [63:0] predict_address; // target address at which to jump, or not
//     } branchpredict_sbe_t
//
//     typedef struct packed {
//         logic        valid
//         logic [63:0] pc;             // update at PC
//         logic [63:0] target_address
//     } btb_update_t
//
//     typedef struct packed {
//         logic        valid
//         logic [63:0] target_address
//     } btb_prediction_t
//
//     typedef struct packed {
//         logic        valid
//         logic [63:0] ra
//     } ras_t
//
//     typedef struct packed {
//         logic        valid
//         logic [63:0] pc;          // update at PC
//         logic        taken
//     } bht_update_t
//
//     typedef struct packed {
//         logic       valid
//         logic       taken
//     } bht_prediction_t
//
//     typedef enum logic[3:0] {
//         NONE,      // 0
//         LOAD,      // 1
//         STORE,     // 2
//         ALU,       // 3
//         CTRL_FLOW, // 4
//         MULT,      // 5
//         CSR,       // 6
//         FPU,       // 7
//         FPU_VEC    // 8
//     } fu_t
//
//     val EXC_OFF_RST      = 8'h80
//
//     val SupervisorIrq = 1
//     val MachineIrq = 0
//
//     // All information needed to determine whether we need to associate an interrupt
//     // with the corresponding instruction or not.
//     typedef struct packed {
//       logic [63:0] mie
//       logic [63:0] mip
//       logic [63:0] mideleg
//       logic        sie
//       logic        global_enable
//     } irq_ctrl_t
//
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

//     // ---------------
//     // EX Stage
//     // ---------------
//     typedef enum logic [6:0] { // basic ALU op
//                                ADD, SUB, ADDW, SUBW,
//                                // logic operations
//                                XORL, ORL, ANDL,
//                                // shifts
//                                SRA, SRL, SLL, SRLW, SLLW, SRAW,
//                                // comparisons
//                                LTS, LTU, GES, GEU, EQ, NE,
//                                // jumps
//                                JALR, BRANCH,
//                                // set lower than operations
//                                SLTS, SLTU,
//                                // CSR functions
//                                MRET, SRET, DRET, ECALL, WFI, FENCE, FENCE_I, SFENCE_VMA, CSR_WRITE, CSR_READ, CSR_SET, CSR_CLEAR,
//                                // LSU functions
//                                LD, SD, LW, LWU, SW, LH, LHU, SH, LB, SB, LBU,
//                                // Atomic Memory Operations
//                                AMO_LRW, AMO_LRD, AMO_SCW, AMO_SCD,
//                                AMO_SWAPW, AMO_ADDW, AMO_ANDW, AMO_ORW, AMO_XORW, AMO_MAXW, AMO_MAXWU, AMO_MINW, AMO_MINWU,
//                                AMO_SWAPD, AMO_ADDD, AMO_ANDD, AMO_ORD, AMO_XORD, AMO_MAXD, AMO_MAXDU, AMO_MIND, AMO_MINDU,
//                                // Multiplications
//                                MUL, MULH, MULHU, MULHSU, MULW,
//                                // Divisions
//                                DIV, DIVU, DIVW, DIVUW, REM, REMU, REMW, REMUW,
//                                // Floating-Point Load and Store Instructions
//                                FLD, FLW, FLH, FLB, FSD, FSW, FSH, FSB,
//                                // Floating-Point Computational Instructions
//                                FADD, FSUB, FMUL, FDIV, FMIN_MAX, FSQRT, FMADD, FMSUB, FNMSUB, FNMADD,
//                                // Floating-Point Conversion and Move Instructions
//                                FCVT_F2I, FCVT_I2F, FCVT_F2F, FSGNJ, FMV_F2X, FMV_X2F,
//                                // Floating-Point Compare Instructions
//                                FCMP,
//                                // Floating-Point Classify Instruction
//                                FCLASS,
//                                // Vectorial Floating-Point Instructions that don't directly map onto the scalar ones
//                                VFMIN, VFMAX, VFSGNJ, VFSGNJN, VFSGNJX, VFEQ, VFNE, VFLT, VFGE, VFLE, VFGT, VFCPKAB_S, VFCPKCD_S, VFCPKAB_D, VFCPKCD_D
//                              } fu_op
//
//     typedef struct packed {
//         fu_t                      fu
//         fu_op                     operator
//         logic [63:0]              operand_a
//         logic [63:0]              operand_b
//         logic [63:0]              imm
//         logic [TRANS_ID_BITS-1:0] trans_id
//     } fu_data_t
//
//     def is_branch (input fu_op op)
//         unique case (op) inside
//             EQ, NE, LTS, GES, LTU, GEU: return 1'b1
//             default                   : return 1'b0; // all other ops
//         endcase
//     endfunction
//
//     // -------------------------------
//     // Extract Src/Dst FP Reg from Op
//     // -------------------------------
//     def is_rs1_fpr (input fu_op op)
//         if (FP_PRESENT) begin // makes function static for non-fp case
//             unique case (op) inside
//                 [FMUL:FNMADD],                   // Computational Operations (except ADD/SUB)
//                 FCVT_F2I,                        // Float-Int Casts
//                 FCVT_F2F,                        // Float-Float Casts
//                 FSGNJ,                           // Sign Injections
//                 FMV_F2X,                         // FPR-GPR Moves
//                 FCMP,                            // Comparisons
//                 FCLASS,                          // Classifications
//                 [VFMIN:VFCPKCD_D] : return 1'b1; // Additional Vectorial FP ops
//                 default           : return 1'b0; // all other ops
//             endcase
//         end else
//             return 1'b0
//     endfunction
//
//     def is_rs2_fpr (input fu_op op)
//         if (FP_PRESENT) begin // makes function static for non-fp case
//             unique case (op) inside
//                 [FSD:FSB],                       // FP Stores
//                 [FADD:FMIN_MAX],                 // Computational Operations (no sqrt)
//                 [FMADD:FNMADD],                  // Fused Computational Operations
//                 FCVT_F2F,                        // Vectorial F2F Conversions requrie target
//                 [FSGNJ:FMV_F2X],                 // Sign Injections and moves mapped to SGNJ
//                 FCMP,                            // Comparisons
//                 [VFMIN:VFCPKCD_D] : return 1'b1; // Additional Vectorial FP ops
//                 default           : return 1'b0; // all other ops
//             endcase
//         end else
//             return 1'b0
//     endfunction
//
//     // ternary operations encode the rs3 address in the imm field, also add/sub
//     def is_imm_fpr (input fu_op op)
//         if (FP_PRESENT) begin // makes function static for non-fp case
//             unique case (op) inside
//                 [FADD:FSUB],                         // ADD/SUB need inputs as Operand B/C
//                 [FMADD:FNMADD],                      // Fused Computational Operations
//                 [VFCPKAB_S:VFCPKCD_D] : return 1'b1; // Vectorial FP cast and pack ops
//                 default               : return 1'b0; // all other ops
//             endcase
//         end else
//             return 1'b0
//     endfunction
//
//     def is_rd_fpr (input fu_op op)
//         if (FP_PRESENT) begin // makes function static for non-fp case
//             unique case (op) inside
//                 [FLD:FLB],                           // FP Loads
//                 [FADD:FNMADD],                       // Computational Operations
//                 FCVT_I2F,                            // Int-Float Casts
//                 FCVT_F2F,                            // Float-Float Casts
//                 FSGNJ,                               // Sign Injections
//                 FMV_X2F,                             // GPR-FPR Moves
//                 [VFMIN:VFSGNJX],                     // Vectorial MIN/MAX and SGNJ
//                 [VFCPKAB_S:VFCPKCD_D] : return 1'b1; // Vectorial FP cast and pack ops
//                 default               : return 1'b0; // all other ops
//             endcase
//         end else
//             return 1'b0
//     endfunction
//
//     def is_amo (fu_op op)
//         case (op) inside
//             [AMO_LRW:AMO_MINDU]: begin
//                 return 1'b1
//             end
//             default: return 1'b0
//         endcase
//     endfunction
//
//     typedef struct packed {
//         logic                     valid
//         logic [63:0]              vaddr
//         logic [63:0]              data
//         logic [7:0]               be
//         fu_t                      fu
//         fu_op                     operator
//         logic [TRANS_ID_BITS-1:0] trans_id
//     } lsu_ctrl_t
//
//     // ---------------
//     // IF/ID Stage
//     // ---------------
//     // store the decompressed instruction
//     typedef struct packed {
//         logic [63:0]           address;        // the address of the instructions from below
//         logic [31:0]           instruction;    // instruction word
//         branchpredict_sbe_t    branch_predict; // this field contains branch prediction information regarding the forward branch path
//         exception_t            ex;             // this field contains exceptions which might have happened earlier, e.g.: fetch exceptions
//     } fetch_entry_t
//
//     // ---------------
//     // ID/EX/WB Stage
//     // ---------------
//     typedef struct packed {
//         logic [63:0]              pc;            // PC of instruction
//         logic [TRANS_ID_BITS-1:0] trans_id;      // this can potentially be simplified, we could index the scoreboard entry
//                                                  // with the transaction id in any case make the width more generic
//         fu_t                      fu;            // functional unit to use
//         fu_op                     op;            // operation to perform in each functional unit
//         logic [REG_ADDR_SIZE-1:0] rs1;           // register source address 1
//         logic [REG_ADDR_SIZE-1:0] rs2;           // register source address 2
//         logic [REG_ADDR_SIZE-1:0] rd;            // register destination address
//         logic [63:0]              result;        // for unfinished instructions this field also holds the immediate,
//                                                  // for unfinished floating-point that are partly encoded in rs2, this field also holds rs2
//                                                  // for unfinished floating-point fused operations (FMADD, FMSUB, FNMADD, FNMSUB)
//                                                  // this field holds the address of the third operand from the floating-point register file
//         logic                     valid;         // is the result valid
//         logic                     use_imm;       // should we use the immediate as operand b?
//         logic                     use_zimm;      // use zimm as operand a
//         logic                     use_pc;        // set if we need to use the PC as operand a, PC from exception
//         exception_t               ex;            // exception has occurred
//         branchpredict_sbe_t       bp;            // branch predict scoreboard data structure
//         logic                     is_compressed; // signals a compressed instructions, we need this information at the commit stage if
//                                                  // we want jump accordingly e.g.: +4, +2
//     } scoreboard_entry_t
//
//
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
//     // ----------------------
//     // cache request ports
//     // ----------------------
//     // I$ address translation requests
//     typedef struct packed {
//         logic                     fetch_valid;     // address translation valid
//         logic [63:0]              fetch_paddr;     // physical address in
//         exception_t               fetch_exception; // exception occurred during fetch
//     } icache_areq_i_t
//
//     typedef struct packed {
//         logic                     fetch_req;       // address translation request
//         logic [63:0]              fetch_vaddr;     // virtual address out
//     } icache_areq_o_t
//
//     // I$ data requests
//     typedef struct packed {
//         logic                     req;                    // we request a new word
//         logic                     kill_s1;                // kill the current request
//         logic                     kill_s2;                // kill the last request
//         logic [63:0]              vaddr;                  // 1st cycle: 12 bit index is taken for lookup
//     } icache_dreq_i_t
//
//     typedef struct packed {
//         logic                     ready;                  // icache is ready
//         logic                     valid;                  // signals a valid read
//         logic [FETCH_WIDTH-1:0]   data;                   // 2+ cycle out: tag
//         logic [63:0]              vaddr;                  // virtual address out
//         exception_t               ex;                     // we've encountered an exception
//     } icache_dreq_o_t
//
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
//

//
//     // ----------------------
//     // Arithmetic Functions
//     // ----------------------
//     def [63:0] sext32 (logic [31:0] operand)
//         return {{32{operand[31]}}, operand[31:0]}
//     endfunction
//
//     // ----------------------
//     // Immediate functions
//     // ----------------------
//     def [63:0] uj_imm (logic [31:0] instruction_i)
//         return { {44 {instruction_i[31]}}, instruction_i[19:12], instruction_i[20], instruction_i[30:21], 1'b0 }
//     endfunction
//
//     def [63:0] i_imm (logic [31:0] instruction_i)
//         return { {52 {instruction_i[31]}}, instruction_i[31:20] }
//     endfunction
//
//     def [63:0] sb_imm (logic [31:0] instruction_i)
//         return { {51 {instruction_i[31]}}, instruction_i[31], instruction_i[7], instruction_i[30:25], instruction_i[11:8], 1'b0 }
//     endfunction
//
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
//     def [1:0] extract_transfer_size(fu_op op)
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
