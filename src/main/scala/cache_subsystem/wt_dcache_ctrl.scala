package cache_subsystem

import chisel3._
import chisel3.util._
import chisel3.Bool

import wt_cache_pkg._
import ariane_pkg._

class wt_dcache_ctrl(RdTxId:Int = 1,
  // ArianeCfg: ariane_cfg_t = ArianeDefaultConfig)
  ArianeCfg: ariane_cfg_t)
    extends Module {
  val io = IO(new Bundle {
    val cache_en_i      = Input(Bool())
    // core request ports
    val req_port_i      = Input(new dcache_req_i_t())
    val req_port_o      = Output(new dcache_req_o_t())
    // interface to miss handler
    val miss_req_o      = Output(Bool())
    val miss_ack_i      = Input(Bool())
    val miss_we_o       = Output(Bool())                        // unused (set to 0)
    val miss_wdata_o    = Output(UInt(64.W))                    // unused (set to 0)
    val miss_vld_bits_o = Output(Vec(DCACHE_SET_ASSOC, Bool())) // valid bits at the missed index
    val miss_paddr_o    = Output(UInt(64.W))
    val miss_nc_o       = Output(Bool())                        // request to I/O space
    val miss_size_o     = Output(UInt(3.W))                     // 00: 1byte, 01: 2byte, 10: 4byte, 11: 8byte, 111: cacheline
    val miss_id_o       = Output(UInt(CACHE_ID_WIDTH.W))        // set to constant ID
    val miss_replay_i   = Input(Bool())                         // request collided with pending miss - have to replay the request
    val miss_rtrn_vld_i = Input(Bool())                         // signals that the miss has been served, asserted in the same cycle as when the data returns from memory
                                                                // used to detect readout mux collisions
    val wr_cl_vld_i   = Input(Bool())
    // cache memory interface
    val rd_tag_o      = Output(UInt(DCACHE_TAG_WIDTH.W))       // tag in - comes one cycle later
    val rd_idx_o      = Output(UInt(DCACHE_CL_IDX_WIDTH.W))
    val rd_off_o      = Output(UInt(DCACHE_OFFSET_WIDTH.W))
    val rd_req_o      = Output(Bool())                   // read the word at offset off_i[:3] in all ways
    val rd_tag_only_o = Output(Bool())                   // set to zero here
    val rd_ack_i      = Input(Bool())
    val rd_data_i     = Input(UInt(64.W))
    val rd_vld_bits_i = Input(Vec(DCACHE_SET_ASSOC, Bool()))
    val rd_hit_oh_i   = Input(Vec(DCACHE_SET_ASSOC, Bool()))
  })

  // controller FSM
  val idle_s :: read_s :: missreq_s :: misswait_s :: killmiss_s :: killmissack_s :: replayreq_s :: replayread_s :: Nil = Enum(8)
  val state_q = RegInit (idle_s)
  val state_d = WireInit (idle_s)

  val address_tag_q = RegInit(0.U(DCACHE_TAG_WIDTH.W))
  val address_idx_q = RegInit(0.U(DCACHE_CL_IDX_WIDTH.W))
  val address_off_q = RegInit(0.U(DCACHE_OFFSET_WIDTH.W))
  val vld_data_q    = RegInit(VecInit(Seq.fill(DCACHE_SET_ASSOC)(false.B)))
  val address_tag_d = Wire(UInt(DCACHE_TAG_WIDTH.W))
  val address_idx_d = Wire(UInt(DCACHE_CL_IDX_WIDTH.W))
  val address_off_d = Wire(UInt(DCACHE_OFFSET_WIDTH.W))
  val vld_data_d    = Wire(Vec(DCACHE_SET_ASSOC, Bool()))

  val save_tag = Wire(Bool())
  val rd_req_d = Wire(Bool())
  val rd_req_q = Reg(Bool())
  val rd_ack_d = Reg(Bool())
  val rd_ack_q = Reg(Bool())
  val data_size_d = Wire(UInt(2.W))
  val data_size_q = Reg(UInt(2.W))

  ///////////////////////////////////////////////////////
  // misc
  ///////////////////////////////////////////////////////

  // map address to tag/idx/offset and save
  vld_data_d    := Mux(rd_req_q              , io.rd_vld_bits_i                                                     , vld_data_q   )
  address_tag_d := Mux(save_tag              , io.req_port_i.address_tag                                            , address_tag_q)
  address_idx_d := Mux(io.req_port_o.data_gnt, io.req_port_i.address_index(DCACHE_INDEX_WIDTH-1,DCACHE_OFFSET_WIDTH), address_idx_q)
  address_off_d := Mux(io.req_port_o.data_gnt, io.req_port_i.address_index(DCACHE_OFFSET_WIDTH-1,0)                 , address_off_q)
  data_size_d   := Mux(io.req_port_o.data_gnt, io.req_port_i.data_size                                              , data_size_q  )
  io.rd_tag_o   := address_tag_d
  io.rd_idx_o   := address_idx_d
  io.rd_off_o   := address_off_d

  io.req_port_o.data_rdata := io.rd_data_i

  // to miss unit
  io.miss_vld_bits_o := vld_data_q
  io.miss_paddr_o    := Cat(address_tag_q, address_idx_q, address_off_q)
  io.miss_size_o     := Mux(io.miss_nc_o, data_size_q, 7.U)

  // noncacheable if request goes to I/O space, or if cache is disabled
  io.miss_nc_o := (~io.cache_en_i) | (~is_inside_cacheable_regions(ArianeCfg, Cat(address_tag_q, Fill(DCACHE_INDEX_WIDTH, 0.U(1.W)))))

  io.miss_we_o     := 0.U
  io.miss_wdata_o  := 0.U
  io.miss_id_o     := RdTxId.asUInt
  rd_req_d         := io.rd_req_o
  rd_ack_d         := io.rd_ack_i
  io.rd_tag_only_o := 0.U

///////////////////////////////////////////////////////
// main control logic
///////////////////////////////////////////////////////

  // default assignment
  state_d                := state_q
  save_tag               := false.B
  io.rd_req_o               := false.B
  io.miss_req_o             := false.B
  io.req_port_o.data_rvalid := false.B
  io.req_port_o.data_gnt    := false.B

  // interfaces
  switch (state_q) {
    //////////////////////////////////
    // wait for an incoming request
    is (idle_s) {
      when (io.req_port_i.data_req) {
        io.rd_req_o := true.B
        when (io.rd_ack_i) {
          state_d := read_s
          io.req_port_o.data_gnt := true.B
        }
      }
    }
    //////////////////////////////////
    // check whether we have a hit
    // in case the cache is disabled,
    // or in case the address is NC, we
    // reuse the miss mechanism to handle
    // the request
    is (read_s, replayread_s) {
      // speculatively request cache line
      io.rd_req_o := true.B

      // kill -> go back to IDLE
      when (io.req_port_i.kill_req) {
        state_d := idle_s
        io.req_port_o.data_rvalid := true.B
      } .elsewhen (io.req_port_i.tag_valid || state_q === replayread_s) {
        save_tag := (state_q =/= replayread_s)
        when (io.wr_cl_vld_i || !rd_ack_q) {
          state_d := replayreq_s
          // we've got a hit
        } .elsewhen (io.rd_hit_oh_i.asUInt.orR && io.cache_en_i) {
          state_d := idle_s
          io.req_port_o.data_rvalid := true.B
          // we can handle another request
          when (io.rd_ack_i && io.req_port_i.data_req) {
            state_d := read_s
            io.req_port_o.data_gnt := true.B
          }
          // we've got a miss
        } .otherwise {
          state_d := missreq_s
        }
      }
    }
    //////////////////////////////////
    // issue request
    is (missreq_s) {
      io.miss_req_o := true.B

      when (io.req_port_i.kill_req) {
        io.req_port_o.data_rvalid := true.B
        when (io.miss_ack_i) {
          state_d := killmiss_s
        } .otherwise {
          state_d := killmissack_s
        }
      } .elsewhen (io.miss_replay_i) {
        state_d  := replayreq_s
      } .elsewhen (io.miss_ack_i) {
        state_d  := misswait_s
      }
    }
    //////////////////////////////////
    // wait until the memory transaction
    // returns.
    is (misswait_s) {
      when (io.req_port_i.kill_req) {
        io.req_port_o.data_rvalid := true.B
        when (io.miss_rtrn_vld_i) {
          state_d := idle_s
        } .otherwise {
          state_d := killmiss_s
        }
      } .elsewhen (io.miss_rtrn_vld_i) {
        state_d := idle_s
        io.req_port_o.data_rvalid := true.B
      }
    }
    //////////////////////////////////
    // replay read request
    is (replayreq_s) {
      io.rd_req_o := true.B
      when (io.req_port_i.kill_req) {
        io.req_port_o.data_rvalid := true.B
        state_d := idle_s
      } .elsewhen (io.rd_ack_i) {
        state_d := replayread_s
      }
    }
    //////////////////////////////////
    is (killmissack_s) {
      io.miss_req_o := true.B
      // in this case the miss handler did not issue
      // a transaction and we can safely go to idle
      when (io.miss_replay_i) {
        state_d := idle_s
      } .elsewhen (io.miss_ack_i) {
        state_d := killmiss_s
      }
    }
    //////////////////////////////////
    // killed miss,
    // wait until miss unit responds and
    // go back to idle
    is (killmiss_s) {
      when (io.miss_rtrn_vld_i) {
        state_d := idle_s
      }
    }
    // default: {
    //   // we should never get here
    //   state_d := idle_s
    // }
  }


  ///////////////////////////////////////////////////////
  // ff's
  ///////////////////////////////////////////////////////

  state_q       := state_d
  address_tag_q := address_tag_d
  address_idx_q := address_idx_d
  address_off_q := address_off_d
  vld_data_q    := vld_data_d
  data_size_q   := data_size_d
  rd_req_q      := rd_req_d
  rd_ack_q      := rd_ack_d

}


object wt_dcache_ctrl extends App {
  class ariane_default_cfg extends ariane_cfg_t {
    val RASDepth   = 2
    val BTBEntries = 32
    val BHTEntries = 128
    // PMAs
    val NrNonIdempotentRules  = 2                        // Number of non idempotent rules
    val NonIdempotentAddrBase = Array(0, 0) // base which needs to match
    val NonIdempotentLength   = Array(0, 0) // bit mask which bits to consider when matching the rule
    val NrExecuteRegionRules  = 2                        // Number of regions which have execute property
    val ExecuteRegionAddrBase = Array(0, 0) // base which needs to match
    val ExecuteRegionLength   = Array(0, 0) // bit mask which bits to consider when matching the rule
    val NrCachedRegionRules   = 2                        // Number of regions which have cached property
    val CachedRegionAddrBase  = Array(0, 0) // base which needs to match
    val CachedRegionLength    = Array(0, 0) // bit mask which bits to consider when matching the rule
                                            // cache config
    val Axi64BitCompliant = false // set to 1 when using in conjunction with 64bit AXI bus adapter
    val SwapEndianess     = 0     // set to 1 to swap endianess inside L1.5 openpiton adapter
                                  //
    val DmBaseAddress  = 0         // offset of the debug module
  }

  val cfg = new ariane_default_cfg

  chisel3.Driver.execute(args, () => new wt_dcache_ctrl(1, cfg))
}
