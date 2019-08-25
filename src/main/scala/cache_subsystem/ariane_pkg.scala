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
  val TRANS_ID_BITS = log2Ceil(NR_SB_ENTRIES) // depending on the number of scoreboard entries we need that many bits
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
  val REG_ADDR_SIZE = 6
  val NR_WB_PORTS = 4
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

  type fu_t = UInt
  val NONE     :fu_t = 0.U
  val LOAD     :fu_t = 1.U
  val STORE    :fu_t = 2.U
  val ALU      :fu_t = 3.U
  val CTRL_FLOW:fu_t = 4.U
  val MULT     :fu_t = 5.U
  val CSR      :fu_t = 6.U
  val FPU      :fu_t = 7.U
  val FPU_VEC  :fu_t = 8.U

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

  // ---------------
  // EX Stage
  // ---------------
  type fu_op = UInt
  // basic ALU op
  val ADD  = 0.U
  val SUB  = 1.U
  val ADDW = 2.U
  val SUBW = 3.U
  // logic operations
  val XORL = 4.U
  val ORL  = 5.U
  val ANDL = 6.U
  // shifts
  val SRA  =  7.U
  val SRL  =  8.U
  val SLL  =  9.U
  val SRLW = 10.U
  val SLLW = 11.U
  val SRAW = 12.U
  // comparisons
  val LTS = 13.U
  val LTU = 14.U
  val GES = 15.U
  val GEU = 16.U
  val EQ  = 17.U
  val NE  = 18.U
  // jumps
  val JALR   = 19.U
  val BRANCH = 20.U
  // set lower than operations
  val SLTS = 21.U
  val SLTU = 22.U
  // CSR functions
  val MRET       = 23.U
  val SRET       = 24.U
  val DRET       = 25.U
  val ECALL      = 26.U
  val WFI        = 27.U
  val FENCE      = 28.U
  val FENCE_I    = 29.U
  val SFENCE_VMA = 30.U
  val CSR_WRITE  = 31.U
  val CSR_READ   = 32.U
  val CSR_SET    = 33.U
  val CSR_CLEAR  = 34.U
  // LSU functions
  val LD  = 35.U
  val SD  = 36.U
  val LW  = 37.U
  val LWU = 38.U
  val SW  = 39.U
  val LH  = 40.U
  val LHU = 41.U
  val SH  = 42.U
  val LB  = 43.U
  val SB  = 44.U
  val LBU = 45.U
  // Atomic Memory Operations
  val AMO_LRW    = 46.U
  val AMO_LRD    = 47.U
  val AMO_SCW    = 48.U
  val AMO_SCD    = 49.U
  val AMO_SWAPW  = 50.U
  val AMO_ADDW   = 51.U
  val AMO_ANDW   = 52.U
  val AMO_ORW    = 53.U
  val AMO_XORW   = 54.U
  val AMO_MAXW   = 55.U
  val AMO_MAXWU  = 56.U
  val AMO_MINW   = 57.U
  val AMO_MINWU  = 58.U
  val AMO_SWAPD  = 59.U
  val AMO_ADDD   = 60.U
  val AMO_ANDD   = 61.U
  val AMO_ORD    = 62.U
  val AMO_XORD   = 63.U
  val AMO_MAXD   = 64.U
  val AMO_MAXDU  = 65.U
  val AMO_MIND   = 66.U
  val AMO_MINDU  = 67.U
  // Multiplications
  val MUL    = 68.U
  val MULH   = 69.U
  val MULHU  = 70.U
  val MULHSU = 71.U
  val MULW   = 72.U
  // Divisions
  val DIV   = 73.U
  val DIVU  = 74.U
  val DIVW  = 75.U
  val DIVUW = 76.U
  val REM   = 77.U
  val REMU  = 78.U
  val REMW  = 79.U
  val REMUW = 80.U
  // Floating-Point Load and Store Instructions
  val FLD = 81.U
  val FLW = 82.U
  val FLH = 83.U
  val FLB = 84.U
  val FSD = 85.U
  val FSW = 86.U
  val FSH = 87.U
  val FSB = 88.U
  // Floating-Point Computational Instructions
  val FADD     = 89.U
  val FSUB     = 90.U
  val FMUL     = 91.U
  val FDIV     = 92.U
  val FMIN_MAX = 93.U
  val FSQRT    = 94.U
  val FMADD    = 95.U
  val FMSUB    = 96.U
  val FNMSUB   = 97.U
  val FNMADD   = 98.U
  // Floating-Point Conversion and Move Instructions
  val FCVT_F2I = 99.U
  val FCVT_I2F = 100.U
  val FCVT_F2F = 101.U
  val FSGNJ    = 102.U
  val FMV_F2X  = 103.U
  val FMV_X2F  = 104.U
  // Floating-Point Compare Instructions
  val FCMP = 105.U
  // Floating-Point Classify Instruction
  val FCLASS = 106.U
  // Vectorial Floating-Point Instructions that don't directly map onto the scalar ones
  val VFMIN     = 107.U
  val VFMAX     = 108.U
  val VFSGNJ    = 109.U
  val VFSGNJN   = 110.U
  val VFSGNJX   = 111.U
  val VFEQ      = 112.U
  val VFNE      = 113.U
  val VFLT      = 114.U
  val VFGE      = 115.U
  val VFLE      = 116.U
  val VFGT      = 117.U
  val VFCPKAB_S = 118.U
  val VFCPKCD_S = 119.U
  val VFCPKAB_D = 120.U
  val VFCPKCD_D = 121.U

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
  //             EQ, NE, LTS, GES, LTU, GEU: return true.B
  //             default                   : return false.B; // all other ops
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
  //                 [VFMIN:VFCPKCD_D] : return true.B; // Additional Vectorial FP ops
  //                 default           : return false.B; // all other ops
  //             endcase
  //         end else
  //             return false.B
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
  //                 [VFMIN:VFCPKCD_D] : return true.B; // Additional Vectorial FP ops
  //                 default           : return false.B; // all other ops
  //             endcase
  //         end else
  //             return false.B
  //     endfunction
  //
  //     // ternary operations encode the rs3 address in the imm field, also add/sub
  //     def is_imm_fpr (input fu_op op)
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
  //                 [VFCPKAB_S:VFCPKCD_D] : return true.B; // Vectorial FP cast and pack ops
  //                 default               : return false.B; // all other ops
  //             endcase
  //         end else
  //             return false.B
  //     endfunction
  //
  //     def is_amo (fu_op op)
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
  //         fu_op                     operator
  //         logic [TRANS_ID_BITS-1:0] trans_id
  //     } lsu_ctrl_t
  //
  // ---------------
  // IF/ID Stage
  // ---------------
  // store the decompressed instruction
  class fetch_entry_t extends Bundle {
    val address = UInt(64.W)                       // the address of the instructions from below
    val instruction = UInt(32.W)                   // instruction word
    val branch_predict = new branchpredict_sbe_t() // this field contains branch prediction information regarding the forward branch path
    val ex = new exception_t()                     // this field contains exceptions which might have happened earlier, e.g.: fetch exceptions
  }

  // ---------------
  // ID/EX/WB Stage
  // ---------------
  class scoreboard_entry_t extends Bundle {
    val pc = UInt(64.W)                  // PC of instruction
    val trans_id = UInt(TRANS_ID_BITS.W) // this can potentially be simplified, we could index the scoreboard entry
                                         // with the transaction id in any case make the width more generic
    val fu  = fu_t(4.W)                  // functional unit to use
    val op  = fu_op                      // operation to perform in each functional unit
    val rs1 = UInt(REG_ADDR_SIZE.W)      // register source address 1
    val rs2 = UInt(REG_ADDR_SIZE.W)      // register source address 2
    val rd  = UInt(REG_ADDR_SIZE.W)      // register destination address
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
}
