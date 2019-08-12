package cache_subsystem

import chisel3._
import chisel3.util._
import chisel3.Bool

import scala.math.pow

import ariane_pkg._


object wt_cache_pkg
{

  def log2(x: Int): Int = (scala.math.log(x) / scala.math.log(2)).asInstanceOf[Int]

  // these parames need to coincide with the
  // L1.5 parameterization, do not change
  var L15_SET_ASSOC           = 0
  var L15_TID_WIDTH           = 0
  var L15_TLB_CSM_WIDTH       = 0

  var CONFIG_L15_ASSOCIATIVITY = 4  // this results in 8 pending tx slots in the writebuffer
  var L15_THREADID_WIDTH       = 3
  var TLB_CSM_WIDTH            = 33

  val PITON_ARIANE = false

  if (PITON_ARIANE) {
    CONFIG_L15_ASSOCIATIVITY = 4
    // this results in 8 pending tx slots in the writebuffer
    L15_THREADID_WIDTH = 3
    TLB_CSM_WIDTH = 33

    L15_SET_ASSOC           = CONFIG_L15_ASSOCIATIVITY
    L15_TID_WIDTH           = L15_THREADID_WIDTH
    L15_TLB_CSM_WIDTH       = TLB_CSM_WIDTH
  } else {
    L15_SET_ASSOC           = DCACHE_SET_ASSOC  // align with dcache for compatibility with the standard Ariane setup
    L15_TID_WIDTH           = 2
    L15_TLB_CSM_WIDTH       = 33
  }

  val L15_WAY_WIDTH           = log2(L15_SET_ASSOC)
  val L1I_WAY_WIDTH           = log2(ICACHE_SET_ASSOC)
  val L1D_WAY_WIDTH           = log2(DCACHE_SET_ASSOC)

  // FIFO depths of L15 adapter
  val ADAPTER_REQ_FIFO_DEPTH  = 2
  val ADAPTER_RTRN_FIFO_DEPTH = 2


  // Calculated parameter
  val ICACHE_OFFSET_WIDTH     = log2(ICACHE_LINE_WIDTH/8)
  val ICACHE_NUM_WORDS        = pow(2, ICACHE_INDEX_WIDTH-ICACHE_OFFSET_WIDTH).asInstanceOf[Int]
  val ICACHE_CL_IDX_WIDTH     = log2(ICACHE_NUM_WORDS)// excluding byte offset

  val DCACHE_OFFSET_WIDTH     = log2(DCACHE_LINE_WIDTH/8)
  val DCACHE_NUM_WORDS        = pow(2, DCACHE_INDEX_WIDTH-DCACHE_OFFSET_WIDTH).asInstanceOf[Int]
  val DCACHE_CL_IDX_WIDTH     = log2(DCACHE_NUM_WORDS)// excluding byte offset

  val DCACHE_NUM_BANKS        = DCACHE_LINE_WIDTH/64

  // write buffer parameterization
  val DCACHE_WBUF_DEPTH       = 8
  val DCACHE_MAX_TX           = pow(2, L15_TID_WIDTH).asInstanceOf[Int]
  val CACHE_ID_WIDTH          = L15_TID_WIDTH


  class wbuffer_t extends Bundle {
    val wtag    = UInt((DCACHE_INDEX_WIDTH+DCACHE_TAG_WIDTH).W)
    val data    = UInt(64.W)
    val dirty   = UInt(8.W)                            // byte is dirty
    val valid   = UInt(8.W)                            // byte is valid
    val txblock = UInt(8.W)                            // byte is part of transaction in-flight
    val checked = Bool()                               // if cache state of this word has been checked
    val hit_oh  = UInt(DCACHE_SET_ASSOC.W) // valid way in the cache
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
  val dcache_store_req :: dcache_load_req :: dcache_atomic_req :: dcache_int_req :: Nil = Enum(4) // dcache_out_t
  val dcache_inv_req /* no ack from the core required */ :: dcache_store_ack /* note: this may contain an invalidation vector, too */ :: dcache_load_ack :: dcache_atomic_ack :: dcache_int_ack :: Nil = Enum(5)  // dcache_in_t
  val icache_inv_req /* no ack from the core required */ :: icache_ifill_ack :: Nil = Enum(2)  // icache_in_t

  class cache_inval_t extends Bundle {
    val vld = Bool()                                 // invalidate only affected way
    val all = Bool()                                 // invalidate all ways
    val idx = UInt(ICACHE_INDEX_WIDTH.W) // physical address to invalidate
    val way = UInt(L15_WAY_WIDTH.W)                  // way to invalidate
  }

  // icache interface
  class icache_req_t extends Bundle {
    val way    = UInt(log2(ICACHE_SET_ASSOC)) // way to replace
    val paddr  = UInt(64.W)                               // physical address
    val nc     = Bool()                                   // noncacheable
    val tid    = UInt(CACHE_ID_WIDTH)                     // threadi id (used as transaction id in Ariane)
  }

  class icache_rtrn_t extends Bundle {
    val rtype = UInt()  // icache_in_t()                         // see definitions above
    val data  = UInt(ICACHE_LINE_WIDTH.W) // full cache line width
    val inv   = UInt()  // cache_inval_t()                       // invalidation vector
    val tid   = UInt(CACHE_ID_WIDTH.W)                // threadi id (used as transaction id in Ariane)
  }

  // dcache interface
  class dcache_req_t extends Bundle {
    val rtype  = UInt() // dcache_out_t()         // see definitions above
    val size   = UInt(3.W)              // transaction size: 000=Byte 001=2Byte 010=4Byte 011=8Byte 111=Cache line (16/32Byte)
    val way    = UInt(L1D_WAY_WIDTH.W)  // way to replace
    val paddr  = UInt(64.W)             // physical address
    val data   = UInt(64.W)             // word width of processor (no block stores at the moment)
    val nc     = Bool()                 // noncacheable
    val tid    = UInt(CACHE_ID_WIDTH.W) // threadi id (used as transaction id in Ariane)
    val amo_op = UInt()                 // amo opcode
  }

  class dcache_rtrn_t extends Bundle {
    val rtype = UInt() // dcache_in_t()                         // see definitions above
    val data  = UInt(DCACHE_LINE_WIDTH.W) // full cache line width
    val inv   = UInt() // cache_inval_t()                       // invalidation vector
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
  object l15_rtrntypes_t {
    val L15_LOAD_RET               = Integer.parseInt("0000", 2) // load packet
    // val L15_INV_RET                = Integer.parseInt("0011", 2) // invalidate packet, not unique...
    val L15_ST_ACK                 = Integer.parseInt("0100", 2) // store ack packet
    // val L15_AT_ACK                 = Integer.parseInt("0011", 2), // unused, not unique...
    val L15_INT_RET                = Integer.parseInt("0111", 2) // interrupt packet
    val L15_TEST_RET               = Integer.parseInt("0101", 2) // unused
    val L15_FP_RET                 = Integer.parseInt("1000", 2) // unused
    val L15_IFILL_RET              = Integer.parseInt("0001", 2) // instruction fill packet
    val L15_EVICT_REQ              = Integer.parseInt("0011", 2) // eviction request
    val L15_ERR_RET                = Integer.parseInt("1100", 2) // unused
    val L15_STRLOAD_RET            = Integer.parseInt("0010", 2) // unused
    val L15_STRST_ACK              = Integer.parseInt("0110", 2) // unused
    val L15_FWD_RQ_RET             = Integer.parseInt("1010", 2) // unused
    val L15_FWD_RPY_RET            = Integer.parseInt("1011", 2) // unused
    val L15_RSVD_RET               = Integer.parseInt("1111", 2) // unused
    val L15_CPX_RESTYPE_ATOMIC_RES = Integer.parseInt("1110", 2) // custom type for atomic responses
  }


  class l15_req_t extends Bundle {
    val l15_val                  = Bool()              // valid signal, asserted with request
    val l15_req_ack              = Bool()              // ack for response
    val l15_rqtype               = UInt()   // l15_reqtypes_t()                // see below for encoding
    val l15_nc                   = Bool()      // non-cacheable bit
    val l15_size                 = UInt(3.W)                  // transaction size: 000=Byte 001=2Byte 010=4Byte 011=8Byte 111=Cache line (16/32Byte)
    val l15_threadid             = UInt(L15_TID_WIDTH.W)              // currently 0 or 1
    val l15_prefetch             = Bool() // unused in openpiton
    val l15_invalidate_cacheline = Bool() // unused by Ariane as L1 has no ECC at the moment
    val l15_blockstore           = Bool() // unused in openpiton
    val l15_blockinitstore       = Bool() // unused in openpiton
    val l15_l1rplway             = UInt(L15_WAY_WIDTH.W)              // way to replace
    val l15_address              = UInt(40.W)               // physical address
    val l15_data                 = UInt(64.W)                 // word to write
    val l15_data_next_entry      = UInt(64.W)     // unused in Ariane (only used for CAS atomic requests)
    val l15_csm_data             = UInt(L15_TLB_CSM_WIDTH.W)              // unused in Ariane
    val l15_amo_op               = UInt(4.W)                // atomic operation type
  }

  class l15_rtrn_t extends Bundle {
    val l15_ack                  = Bool()           // ack for request struct
    val l15_header_ack           = Bool()           // ack for request struct
    val l15_val                  = Bool()           // valid signal for return struct
    val l15_returntype           = UInt() // l15_rtrntypes_t()            // see below for encoding
    val l15_l2miss               = Bool() // unused in Ariane
    val l15_error                = UInt(2.W) // unused in openpiton
    val l15_noncacheable         = Bool() // non-cacheable bit
    val l15_atomic               = Bool() // asserted in load return and store ack packets of atomic tx
    val l15_threadid             = UInt(L15_TID_WIDTH.W) // used as transaction ID
    val l15_prefetch             = Bool() // unused in openpiton
    val l15_f4b                  = Bool() // 4byte instruction fill from I/O space (nc).
    val l15_data_0               = UInt(64.W) // used for both caches
    val l15_data_1               = UInt(64.W) // used for both caches
    val l15_data_2               = UInt(64.W) // currently only used for I$
    val l15_data_3               = UInt(64.W) // currently only used for I$
    val l15_inval_icache_all_way = Bool() // invalidate all ways
    val l15_inval_dcache_all_way = Bool() // unused in openpiton
    val l15_inval_address_15_4   = UInt(12.W) // invalidate selected cacheline [15:4]
    val l15_cross_invalidate     = Bool() // unused in openpiton
    val l15_cross_invalidate_way = UInt(L15_WAY_WIDTH.W) // unused in openpiton
    val l15_inval_dcache_inval   = Bool() // invalidate selected cacheline and way
    val l15_inval_icache_inval   = Bool() // unused in openpiton
    val l15_inval_way            = UInt(L15_WAY_WIDTH.W) // way to invalidate
    val l15_blockinitstore       = Bool() // unused in openpiton
  }

  // swap endianess in a 64bit word
  // def swendian64(in: UInt(64.W)) : UInt = {
  def swendian64(in: UInt) : UInt = {
    val out = Vec(8, UInt(8.W))
    for (k <- 0 until 8) {
      out(k) := in(8-k)
    }
    return out.foldLeft(0.U(0.W))(Cat(_, _))
  }

  // def icache_way_bin2oh (in: UInt(log2(ICACHE_SET_ASSOC))) : UInt(ICACHE_SET_ASSOC) = {
  def icache_way_bin2oh (in: UInt) : UInt = {
    val out = UInt(ICACHE_SET_ASSOC.W)
    out     := 0.U
    out(in) := 1.U
    return out
  }

  // def dcache_way_bin2oh (in: UInt(log2(DCACHE_SET_ASSOC))) : UInt(DCACHE_SET_ASSOC) = {
  def dcache_way_bin2oh (in: UInt) : UInt = {
    val out = UInt(DCACHE_SET_ASSOC.W)
    out     := 0.U
    out(in) := 1.U
    return out
  }

  // def dcache_cl_bin2oh(in: UInt(log2(DCACHE_NUM_BANKS).W)) : UInt(DCACHE_NUM_BANKS) = {
  def dcache_cl_bin2oh(in: UInt) : UInt = {
    val out = UInt(DCACHE_NUM_BANKS)
    out     := 0.U
    out(in) := 1.U
    return out
  }

  // def popcnt64(in: UInt(64.W)) : UInt(5.W) = {
  def popcnt64(in: UInt) : UInt = {
    var cnt = UInt(6.W)
    for (k <- 0 until 64) {
      cnt = cnt + in(k)
    }
    return cnt
  }

  // def toByteEnable8(offset: UInt(3.W), size: UInt(2.W)) : UInt(8.W) = {
  def toByteEnable8(offset: UInt, size: UInt) : UInt = {
    var be = Vec(8, Bool())
    be := 0.U
    be := true.B // default dword
    switch (size) {
      is (0.U) { be(offset)                := true.B } // byte
      is (1.U) { for(i <- 0 until 2) { be(offset + i.U)  := true.B } } // hword
      is (2.U) { for(i <- 0 until 4) { be(offset + i.U)  := true.B } } // word
    }
    return be.asUInt
  }

  // openpiton requires the data to be replicated in case of smaller sizes than dwords
  // def repData64(data: UInt(64.W), offset: UInt(3.W), size: UInt(2.W)) : UInt(64.W) = {
  def repData64(data: UInt, offset: UInt, size: UInt) : UInt = {
    var out = Vec(8, UInt(8.W))
    out  := data // dword

    val dataVec = Vec(8, UInt(8.W))
    for(k <- 0 until 8) { dataVec(k) := data(8 * k + 7, 8 * k) }

    switch(size) {
      is (0.U) {
        for(k <- 0 until 8) { out(k) := dataVec(offset) }
      } // byte
      is (1.U) {
        for(k <- 0 until 8 by 2) {
          out(k + 0) := dataVec(offset + 0.U)
          out(k + 1) := dataVec(offset + 1.U)
        } // hword
      }
      is (2.U) {
        for(k <- 0 until 8 by 4) {
          out(k + 0)  := dataVec(offset + 0.U)
          out(k + 1)  := dataVec(offset + 1.U)
          out(k + 2)  := dataVec(offset + 2.U)
          out(k + 3)  := dataVec(offset + 3.U)
        }
      } // word
    }
    return out.asUInt
  }

  // note: this is openpiton specific. cannot transmit unaligned words.
  // hence we default to individual bytes in that case, and they have to be transmitted
  // one after the other
  // def toSize64 (be: UInt(8.W)) : UInt(2.W) = {
  def toSize64 (be: UInt) : UInt = {
    val size = UInt(2.W)
    size := 0.U
    switch(be) {
      is (0xff.U)                         { size := 3.U }  // dword
      is (0x0f.U, 0xf0.U)                 { size := 2.U } // word
      is (0xc0.U, 0x30.U, 0x0c.U, 0x03.U) { size := 1.U } // hword
    }
    return size
  }

  // align the physical address to the specified size:
  // 000: bytes
  // 001: hword
  // 010: word
  // 011: dword
  // 111: DCACHE line
  // def paddrSizeAlign(paddr: UInt(64.W), size: UInt(3.W)) : UInt(64.W) = {
  def paddrSizeAlign(paddr: UInt, size: UInt) : UInt = {
    val out = Vec(64, Bool())
    out := paddr
    switch (size) {
      is (1.U) { for (i <- 0 until 1) { out(i) := false.B } }
      is (2.U) { for (i <- 0 until 2) { out(i) := false.B } }
      is (3.U) { for (i <- 0 until 3) { out(i) := false.B } }
      is (7.U) { for (i <- 0 until DCACHE_OFFSET_WIDTH) { out(i) := false.B } }
    }
    return out.asUInt
  }
}
