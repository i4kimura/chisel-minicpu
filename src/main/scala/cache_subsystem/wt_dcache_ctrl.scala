package cache_subsystem

import chisel3._
import chisel3.util._
import chisel3.Bool

object wt_cache_pkg
{

  def log2(x: Int): Int = (scala.math.log(x) / scala.math.log(2)).asInstanceOf[Int]

  // these parames need to coincide with the
  // L1.5 parameterization, do not change
  if (PITON_ARIANE) {

    if (CONFIG_L15_ASSOCIATIVITY) {
      val CONFIG_L15_ASSOCIATIVITY = 4
    }

    if (L15_THREADID_WIDTH) {
      // this results in 8 pending tx slots in the writebuffer
      val L15_THREADID_WIDTH = 3
    }

    if (TLB_CSM_WIDTH) {
      val TLB_CSM_WIDTH = 33
    }

    val L15_SET_ASSOC           = CONFIG_L15_ASSOCIATIVITY
    val L15_TID_WIDTH           = L15_THREADID_WIDTH
    val L15_TLB_CSM_WIDTH       = TLB_CSM_WIDTH
  } else {
    val L15_SET_ASSOC           = ariane_pkg::DCACHE_SET_ASSOC;// align with dcache for compatibility with the standard Ariane setup
    val L15_TID_WIDTH           = 2
    val L15_TLB_CSM_WIDTH       = 33
  }

  val L15_WAY_WIDTH           = log2(L15_SET_ASSOC)
  val L1I_WAY_WIDTH           = log2(ariane_pkg::ICACHE_SET_ASSOC)
  val L1D_WAY_WIDTH           = log2(ariane_pkg::DCACHE_SET_ASSOC)

  // FIFO depths of L15 adapter
  val ADAPTER_REQ_FIFO_DEPTH  = 2
  val ADAPTER_RTRN_FIFO_DEPTH = 2


  // Calculated parameter
  val ICACHE_OFFSET_WIDTH     = log2(ariane_pkg::ICACHE_LINE_WIDTH/8)
  val ICACHE_NUM_WORDS        = 2**(ariane_pkg::ICACHE_INDEX_WIDTH-ICACHE_OFFSET_WIDTH)
  val ICACHE_CL_IDX_WIDTH     = log2(ICACHE_NUM_WORDS);// excluding byte offset

  val DCACHE_OFFSET_WIDTH     = log2(ariane_pkg::DCACHE_LINE_WIDTH/8)
  val DCACHE_NUM_WORDS        = 2**(ariane_pkg::DCACHE_INDEX_WIDTH-DCACHE_OFFSET_WIDTH)
  val DCACHE_CL_IDX_WIDTH     = log2(DCACHE_NUM_WORDS);// excluding byte offset

  val DCACHE_NUM_BANKS        = ariane_pkg::DCACHE_LINE_WIDTH/64

  // write buffer parameterization
  val DCACHE_WBUF_DEPTH       = 8
  val DCACHE_MAX_TX           = 2**L15_TID_WIDTH
  val CACHE_ID_WIDTH          = L15_TID_WIDTH


  class wbuffer_t extends Bundle {
    val [ariane_pkg::DCACHE_INDEX_WIDTH+ariane_pkg::DCACHE_TAG_WIDTH-1:0] wtag
    val [63:0]                                                            data
    val [7:0]                                                             dirty;   // byte is dirty
    val [7:0]                                                             valid;   // byte is valid
    val [7:0]                                                             txblock; // byte is part of transaction in-flight
    val                                                                   checked; // if cache state of this word has been checked
    val [ariane_pkg::DCACHE_SET_ASSOC-1:0]                                hit_oh;  // valid way in the cache
  }

  // TX status registers are indexed with the transaction ID
  // they basically store which bytes from which buffer entry are part
  // of that transaction
  class tx_stat_t extends Bundle {
    val vld = Bool()
    val be  = UInt(8.W)
    val ptr = UInt(log2(DCACHE_WBUF_DEPTH))
  }

  // local interfaces between caches and L15 adapter
  val s_even :: s_odd :: Nil = Enum(UInt(), 2)
  val DCACHE_STORE_REQ :: DCACHE_LOAD_REQ :: DCACHE_ATOMIC_REQ :: DCACHE_INT_REQ :: Nil = Enum(UInt(), 4)
  val DCACHE_INV_REQ /* no ack from the core required */ :: DCACHE_STORE_ACK /* note: this may contain an invalidation vector, too */ :: DCACHE_LOAD_ACK :: DCACHE_ATOMIC_ACK :: DCACHE_INT_ACK :: Nil = Enum(UInt(), 5)

  val ICACHE_INV_REQ /* no ack from the core required */ :: ICACHE_IFILL_ACK :: Nil = Enum(UInt(), 2)

  class cache_inval_t extends Bundle {
    val vld = Bool()                                 // invalidate only affected way
    val all = Bool()                                 // invalidate all ways
    val idx = UInt(ariane_pkg::ICACHE_INDEX_WIDTH.W) // physical address to invalidate
    val way = UInt(L15_WAY_WIDTH.W)                  // way to invalidate
  }

  // icache interface
  class icache_req_t extends Bundle {
    val way    = UInt(log2(ariane_pkg::ICACHE_SET_ASSOC)) // way to replace
    val paddr  = UInt(64.W)                               // physical address
    val nc     = Bool()                                   // noncacheable
    val tid    = UInt(CACHE_ID_WIDTH)                     // threadi id (used as transaction id in Ariane)
  }

  class icache_rtrn_t extends Bundle {
    val rtype = icache_in_t()                         // see definitions above
    val data  = UInt(ariane_pkg::ICACHE_LINE_WIDTH.W) // full cache line width
    val inv   = cache_inval_t()                       // invalidation vector
    val tid   = UInt(CACHE_ID_WIDTH.W)                // threadi id (used as transaction id in Ariane)
  }

  // dcache interface
  class dcache_req_t extends Bundle {
    val rtype  = dcache_out_t()         // see definitions above
    val size   = UInt(3.W)              // transaction size: 000=Byte 001=2Byte 010=4Byte 011=8Byte 111=Cache line (16/32Byte)
    val way    = UInt(L1D_WAY_WIDTH.W)  // way to replace
    val paddr  = UInt(64.W)             // physical address
    val data   = UInt(64.W)             // word width of processor (no block stores at the moment)
    val nc     = Bool()                 // noncacheable
    val tid    = UInt(CACHE_ID_WIDTH.W) // threadi id (used as transaction id in Ariane)
    val amo_op = ariane_pkg::amo_t()    // amo opcode
  }

  class dcache_rtrn_t extends Bundle {
    val rtype = dcache_in_t()                         // see definitions above
    val data  = UInt(ariane_pkg::DCACHE_LINE_WIDTH.W) // full cache line width
    val inv   = cache_inval_t()                       // invalidation vector
    val tid   = UInt(CACHE_ID_WIDTH.W)                // threadi id (used as transaction id in Ariane)
  }


  // taken from iop.h in openpiton
  // to l1.5 (only marked subset is used)
  object l15_reqtypes_t {
    val L15_LOAD_RQ     = Integer.parseInt("00000", 2) // load request
    val L15_IMISS_RQ    = Integer.parseInt("10000", 2) // instruction fill request
    val L15_STORE_RQ    = Integer.parseInt("00001", 2) // store request
    val L15_ATOMIC_RQ   = Integer.parseInt("00110", 2) // atomic op
    // val L15_CAS1_RQ     = Integer.parseInt("00010", 2) // compare and swap1 packet (OpenSparc atomics)
    // val L15_CAS2_RQ     = Integer.parseInt("00011", 2) // compare and swap2 packet (OpenSparc atomics)
    // val L15_SWAP_RQ     = Integer.parseInt("00110", 2) // swap packet (OpenSparc atomics)
    val L15_STRLOAD_RQ  = Integer.parseInt("00100", 2) // unused
    val L15_STRST_RQ    = Integer.parseInt("00101", 2) // unused
    val L15_STQ_RQ      = Integer.parseInt("00111", 2) // unused
    val L15_INT_RQ      = Integer.parseInt("01001", 2) // interrupt request
    val L15_FWD_RQ      = Integer.parseInt("01101", 2) // unused
    val L15_FWD_RPY     = Integer.parseInt("01110", 2) // unused
    val L15_RSVD_RQ     = Integer.parseInt("11111", 2) // unused
  }

  // from l1.5 (only marked subset is used)
  typedef enum val [3:0] {
    L15_LOAD_RET               = 4'b0000, // load packet
    // L15_INV_RET                = 4'b0011, // invalidate packet, not unique...
    L15_ST_ACK                 = 4'b0100, // store ack packet
    //L15_AT_ACK                 = 4'b0011, // unused, not unique...
    L15_INT_RET                = 4'b0111, // interrupt packet
    L15_TEST_RET               = 4'b0101, // unused
    L15_FP_RET                 = 4'b1000, // unused
    L15_IFILL_RET              = 4'b0001, // instruction fill packet
    L15_EVICT_REQ              = 4'b0011, // eviction request
    L15_ERR_RET                = 4'b1100, // unused
    L15_STRLOAD_RET            = 4'b0010, // unused
    L15_STRST_ACK              = 4'b0110, // unused
    L15_FWD_RQ_RET             = 4'b1010, // unused
    L15_FWD_RPY_RET            = 4'b1011, // unused
    L15_RSVD_RET               = 4'b1111, // unused
    L15_CPX_RESTYPE_ATOMIC_RES = 4'b1110  // custom type for atomic responses
  } l15_rtrntypes_t


  class l15_req_t extends Bundle {
    val                              l15_val                   // valid signal, asserted with request
    val                              l15_req_ack               // ack for response
    l15_reqtypes_t                     l15_rqtype                // see below for encoding
    val                              l15_nc                    // non-cacheable bit
    val [2:0]                        l15_size                  // transaction size: 000=Byte 001=2Byte 010=4Byte 011=8Byte 111=Cache line (16/32Byte)
    val [L15_TID_WIDTH-1:0]          l15_threadid              // currently 0 or 1
    val                              l15_prefetch              // unused in openpiton
    val                              l15_invalidate_cacheline  // unused by Ariane as L1 has no ECC at the moment
    val                              l15_blockstore            // unused in openpiton
    val                              l15_blockinitstore        // unused in openpiton
    val [L15_WAY_WIDTH-1:0]          l15_l1rplway              // way to replace
    val [39:0]                       l15_address               // physical address
    val [63:0]                       l15_data                  // word to write
    val [63:0]                       l15_data_next_entry       // unused in Ariane (only used for CAS atomic requests)
    val [L15_TLB_CSM_WIDTH-1:0]      l15_csm_data              // unused in Ariane
    val [3:0]                        l15_amo_op                // atomic operation type
  }

  class l15_rtrn_t extends Bundle {
    val                              l15_ack                   // ack for request struct
    val                              l15_header_ack            // ack for request struct
    val                              l15_val                   // valid signal for return struct
    l15_rtrntypes_t                    l15_returntype            // see below for encoding
    val                              l15_l2miss                // unused in Ariane
    val [1:0]                        l15_error                 // unused in openpiton
    val                              l15_noncacheable          // non-cacheable bit
    val                              l15_atomic                // asserted in load return and store ack packets of atomic tx
    val [L15_TID_WIDTH-1:0]          l15_threadid              // used as transaction ID
    val                              l15_prefetch              // unused in openpiton
    val                              l15_f4b                   // 4byte instruction fill from I/O space (nc).
    val [63:0]                       l15_data_0                // used for both caches
    val [63:0]                       l15_data_1                // used for both caches
    val [63:0]                       l15_data_2                // currently only used for I$
    val [63:0]                       l15_data_3                // currently only used for I$
    val                              l15_inval_icache_all_way  // invalidate all ways
    val                              l15_inval_dcache_all_way  // unused in openpiton
    val [15:4]                       l15_inval_address_15_4    // invalidate selected cacheline
    val                              l15_cross_invalidate      // unused in openpiton
    val [L15_WAY_WIDTH-1:0]          l15_cross_invalidate_way  // unused in openpiton
    val                              l15_inval_dcache_inval    // invalidate selected cacheline and way
    val                              l15_inval_icache_inval    // unused in openpiton
    val [L15_WAY_WIDTH-1:0]          l15_inval_way             // way to invalidate
    val                              l15_blockinitstore        // unused in openpiton
  }

  // swap endianess in a 64bit word
  function automatic val[63:0] swendian64(input val[63:0] in)
    automatic val[63:0] out
    for(int k=0 k<64k+=8)begin
        out[k +: 8] = in[63-k -: 8]
    end
    return out
  endfunction

  function automatic val [ariane_pkg::ICACHE_SET_ASSOC-1:0] icache_way_bin2oh (
    input val [log2(ariane_pkg::ICACHE_SET_ASSOC)-1:0] in
  )
    val [ariane_pkg::ICACHE_SET_ASSOC-1:0] out
    out     = '0
    out[in] = 1'b1
    return out
  endfunction

  function automatic val [ariane_pkg::DCACHE_SET_ASSOC-1:0] dcache_way_bin2oh (
    input val [log2(ariane_pkg::DCACHE_SET_ASSOC)-1:0] in
  )
    val [ariane_pkg::DCACHE_SET_ASSOC-1:0] out
    out     = '0
    out[in] = 1'b1
    return out
  endfunction

  function automatic val [DCACHE_NUM_BANKS-1:0] dcache_cl_bin2oh (
    input val [log2(DCACHE_NUM_BANKS)-1:0] in
  )
    val [DCACHE_NUM_BANKS-1:0] out
    out     = '0
    out[in] = 1'b1
    return out
  endfunction


  function automatic val [5:0] popcnt64 (
    input val [63:0] in
  )
    val [5:0] cnt= 0
    foreach (in[k]) begin
      cnt += 6'(in[k])
    end
    return cnt
  endfunction : popcnt64

  function automatic val [7:0] toByteEnable8(
    input val [2:0] offset,
    input val [1:0] size
  )
    val [7:0] be
    be = '0
    unique case(size)
      2'b00:   be[offset]       = '1 // byte
      2'b01:   be[offset +:2 ]  = '1 // hword
      2'b10:   be[offset +:4 ]  = '1 // word
      default: be               = '1 // dword
    endcase // size
    return be
  endfunction : toByteEnable8

  // openpiton requires the data to be replicated in case of smaller sizes than dwords
  function automatic val [63:0] repData64(
    input val [63:0] data,
    input val [2:0]  offset,
    input val [1:0]  size
  )
    val [63:0] out
    unique case(size)
      2'b00:   for(int k=0 k<8 k++) out[k*8  +: 8]    = data[offset*8 +: 8]  // byte
      2'b01:   for(int k=0 k<4 k++) out[k*16 +: 16]   = data[offset*8 +: 16] // hword
      2'b10:   for(int k=0 k<2 k++) out[k*32 +: 32]   = data[offset*8 +: 32] // word
      default: out   = data // dword
    endcase // size
    return out
  endfunction : repData64

  // note: this is openpiton specific. cannot transmit unaligned words.
  // hence we default to individual bytes in that case, and they have to be transmitted
  // one after the other
  function automatic val [1:0] toSize64(
    input val  [7:0] be
  )
    val [1:0] size
    unique case(be)
      8'b1111_1111:                                           size = 2'b11  // dword
      8'b0000_1111, 8'b1111_0000:                             size = 2'b10 // word
      8'b1100_0000, 8'b0011_0000, 8'b0000_1100, 8'b0000_0011: size = 2'b01 // hword
      default:                                                size = 2'b00 // individual bytes
    endcase // be
    return size
  endfunction : toSize64

  // align the physical address to the specified size:
  // 000: bytes
  // 001: hword
  // 010: word
  // 011: dword
  // 111: DCACHE line
  function automatic val [63:0] paddrSizeAlign(
    input val [63:0] paddr,
    input val [2:0]  size
  )
    val [63:0] out
    out = paddr
    unique case (size)
      3'b001: out[0:0]                     = '0
      3'b010: out[1:0]                     = '0
      3'b011: out[2:0]                     = '0
      3'b111: out[DCACHE_OFFSET_WIDTH-1:0] = '0
      default:
    endcase
    return out
  endfunction : paddrSizeAlign

endpackage
