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
// Author: Florian Zaruba, ETH Zurich
// Date: 08.02.2018
// Description: Ariane Instruction Fetch Frontend
//
// This module interfaces with the instruction cache, handles control
// change request from the back-end and does branch prediction.

package ariane

import chisel3._
import chisel3.util._
import chisel3.Bool

import wt_cache_pkg._
import ariane_pkg._
import ariane_if_id._


class frontend (ArianeCfg:ariane_cfg_t) extends Module
{
  val io = IO(new Bundle {
    val flush_i      = Input(Bool())                      // flush request for PCGEN
    val flush_bp_i   = Input(Bool())                      // flush branch prediction
    val debug_mode_i = Input(Bool())
    // global input
    val boot_addr_i  = Input(UInt(64.W))
    // Set a new PC
    // mispredict
    val resolved_branch_i = Input(new bp_resolve_t())     // from controller signaling a branch_predict -> update BTB

    // from commit, when flushing the whole pipeline
    val set_pc_commit_i    = Input(Bool())                // Take the PC from commit stage
    val pc_commit_i        = Input(UInt(64.W))            // PC of instruction in commit stage
                                                          // CSR input
    val epc_i              = Input(UInt(64.W))            // exception PC which we need to return to
    val eret_i             = Input(Bool())                // return from exception
    val trap_vector_base_i = Input(UInt(64.W))            // base of trap vector
    val ex_valid_i         = Input(Bool())                // exception is valid - from commit
    val set_debug_pc_i     = Input(Bool())                // jump to debug address
                                                          // Instruction Fetch
    val icache_dreq_o = Output(new icache_dreq_i_t())
    val icache_dreq_i = Input (new icache_dreq_o_t())
    // instruction output port -> to processor back-end
    val fetch_entry_o = Output(new fetch_entry_t())              // fetch entry containing all relevant data for the ID stage
    val fetch_entry_valid_o = Output(Bool())                // instruction in IF is valid
    val fetch_entry_ready_i = Input(Bool())                 // ID acknowledged this instruction
  })


  // Instruction Cache Registers, from I$
  val icache_data_q = RegInit(0.U(FETCH_WIDTH.W))
  val icache_valid_q    = RegInit(false.B)
  val icache_ex_valid_q = RegInit(false.B)
  val icache_vaddr_q = RegInit(0.U(64.W))
  val instr_queue_ready = Wire(Bool())
  val instr_queue_consumed = Wire(Vec(INSTR_PER_FETCH, Bool()))
  // upper-most branch-prediction from last cycle
  val btb_q = Reg(new btb_prediction_t())
  val bht_q = Reg(new bht_prediction_t())
  // instruction fetch is ready
  val          if_ready = Wire(Bool())
  val npc_d = Wire(UInt(64.W))
  val npc_q = RegInit(0.U(64.W)) // next PC

  // indicates whether we come out of reset (then we need to load boot_addr_i)
  val npc_rst_load_q = Wire(Bool())

  val replay = Wire(Bool())
  val replay_addr = Wire(UInt(64.W))

  // shift amount
  val shamt = Wire(UInt(log2Ceil(INSTR_PER_FETCH).W))
  // address will always be 16 bit aligned, make this explicit here
  shamt := io.icache_dreq_i.vaddr(log2Ceil(INSTR_PER_FETCH),1)

  // -----------------------
  // Ctrl Flow Speculation
  // -----------------------
  // RVI ctrl flow prediction
  val rvi_return = Wire(Vec(INSTR_PER_FETCH, Bool()))
  val rvi_call = Wire(Vec(INSTR_PER_FETCH, Bool()))
  val rvi_branch = Wire(Vec(INSTR_PER_FETCH, Bool()))
  val rvi_jalr = Wire(Vec(INSTR_PER_FETCH, Bool()))
  val rvi_jump = Wire(Vec(INSTR_PER_FETCH, Bool()))
  val rvi_imm = Wire(Vec(INSTR_PER_FETCH, UInt(64.W)))
  // RVC branching
  val rvc_branch = Wire(Vec(INSTR_PER_FETCH, Bool()))
  val rvc_jump = Wire(Vec(INSTR_PER_FETCH, Bool()))
  val rvc_jr = Wire(Vec(INSTR_PER_FETCH, Bool()))
  val rvc_return = Wire(Vec(INSTR_PER_FETCH, Bool()))
  val rvc_jalr = Wire(Vec(INSTR_PER_FETCH, Bool()))
  val rvc_call = Wire(Vec(INSTR_PER_FETCH, Bool()))
  val rvc_imm  = Wire(Vec(INSTR_PER_FETCH, UInt(64.W)))
  // re-aligned instruction and address (coming from cache - combinationally)
  val instr = Wire(Vec(INSTR_PER_FETCH, UInt(32.W)))
  val addr  = Wire(Vec(INSTR_PER_FETCH, UInt(64.W)))
  val instruction_valid  = Wire(Vec(INSTR_PER_FETCH, Bool()))
  // BHT, BTB and RAS prediction
  val bht_prediction         = Wire(Vec(INSTR_PER_FETCH, new bht_prediction_t()))
  val btb_prediction         = Wire(Vec(INSTR_PER_FETCH, new btb_prediction_t()))
  val bht_prediction_shifted = Wire(Vec(INSTR_PER_FETCH, new bht_prediction_t()))
  val btb_prediction_shifted = Wire(Vec(INSTR_PER_FETCH, new btb_prediction_t()))
  val ras_predict = Wire(new ras_t())

  // branch-predict update
  val is_mispredict = Wire(Bool())
  val ras_push      = Wire(Bool())
  val ras_pop       = Wire(Bool())
  val ras_update    = Wire(UInt(64.W))

  // Instruction FIFO
  val predict_address = Wire(UInt(64.W))
  val cf_type      = Wire(Vec(INSTR_PER_FETCH, UInt(3.W))) // new cf_t()))
  val taken_rvi_cf = Wire(Vec(INSTR_PER_FETCH, Bool()))
  val taken_rvc_cf = Wire(Vec(INSTR_PER_FETCH, Bool()))

  val serving_unaligned = Wire(Bool())
  // Re-align instructions
  val i_instr_realign = Module(new instr_realign())
  i_instr_realign.io.flush_i             := io.icache_dreq_o.kill_s2
  i_instr_realign.io.valid_i             := icache_valid_q
  serving_unaligned := i_instr_realign.io.serving_unaligned_o
  i_instr_realign.io.address_i           := icache_vaddr_q
  i_instr_realign.io.data_i              := icache_data_q
  instruction_valid := i_instr_realign.io.valid_o
  addr              := i_instr_realign.io.addr_o
  instr             := i_instr_realign.io.instr_o

  // --------------------
  // Branch Prediction
  // --------------------
  // select the right branch prediction result
  // in case we are serving an unaligned instruction in instr[0] we need to take
  // the prediction we saved from the previous fetch
  bht_prediction_shifted(0) := Mux(serving_unaligned, bht_q, bht_prediction(0))
  btb_prediction_shifted(0) := Mux(serving_unaligned, btb_q, btb_prediction(0))
  // for all other predictions we can use the generated address to index
  // into the branch prediction data structures
  for (i <- 0 until INSTR_PER_FETCH) { // gen_prediction_address
    bht_prediction_shifted(i) := bht_prediction(addr(i)(log2Ceil(INSTR_PER_FETCH), 1))
    btb_prediction_shifted(i) := btb_prediction(addr(i)(log2Ceil(INSTR_PER_FETCH), 1))
  }

  // for the return address stack it doens't matter as we have the
  // address of the call/return already
  var bp_valid = Wire(Bool())

  val is_branch = Wire(Vec(INSTR_PER_FETCH, Bool()))
  val is_call   = Wire(Vec(INSTR_PER_FETCH, Bool()))
  val is_jump   = Wire(Vec(INSTR_PER_FETCH, Bool()))
  val is_return = Wire(Vec(INSTR_PER_FETCH, Bool()))
  val is_jalr   = Wire(Vec(INSTR_PER_FETCH, Bool()))

  for (i <- 0 until INSTR_PER_FETCH) {
    // branch history table -> BHT
    is_branch(i) :=  instruction_valid(i) & (rvi_branch(i) | rvc_branch(i))
    // function calls -> RAS
    is_call(i) := instruction_valid(i) & (rvi_call(i) | rvc_call(i))
    // function return -> RAS
    is_return(i) := instruction_valid(i) & (rvi_return(i) | rvc_return(i))
    // unconditional jumps with known target -> immediately resolved
    is_jump(i) := instruction_valid(i) & (rvi_jump(i) | rvc_jump(i))
    // unconditional jumps with unknown target -> BTB
    is_jalr(i) := instruction_valid(i) & ~is_return(i) & ~is_call(i) & (rvi_jalr(i) | rvc_jalr(i) | rvc_jr(i))
  }

  // taken/not taken
  taken_rvi_cf := Vec(Seq.fill(INSTR_PER_FETCH)(false.B))
  taken_rvc_cf := Vec(Seq.fill(INSTR_PER_FETCH)(false.B))
  predict_address := 0.U(64.W)

  for (i <- 0 until INSTR_PER_FETCH) { cf_type(i) := NoCF }

  ras_push := false.B
  ras_pop  := false.B
  ras_update := 0.U(64.W)

  // lower most prediction gets precedence
  for (i <- INSTR_PER_FETCH-1 until -1 by -1) {
    switch (Cat(is_branch(i), is_return(i), is_jump(i), is_jalr(i))) {
      is(Integer.parseInt("0000", 2).U,    // regular instruction e.g.: no branch
        Integer.parseInt("0001", 2).U) {  // unconditional jump to register, we need the BTB to resolve this
        ras_pop  := false.B
        ras_push := false.B
        when (btb_prediction_shifted(i).valid) {
          predict_address := btb_prediction_shifted(i).target_address
          cf_type(i) := JumpR
        }
      }
      // its an unconditional jump to an immediate
      is(Integer.parseInt("0010", 2).U) {
        ras_pop := false.B
        ras_push := false.B
        taken_rvi_cf(i) := rvi_jump(i)
        taken_rvc_cf(i) := rvc_jump(i)
        cf_type(i) := Jump
      }
      // return
      is(Integer.parseInt("0100", 2).U) {
        // make sure to only alter the RAS if we actually consumed the instruction
        ras_pop := ras_predict.valid & instr_queue_consumed(i)
        ras_push := false.B
        predict_address := ras_predict.ra
        cf_type(i) := Return
      }
      // branch prediction
      is(Integer.parseInt("1000", 2).U) {
        ras_pop := false.B
        ras_push := false.B
        // if we have a valid dynamic prediction use it
        when (bht_prediction_shifted(i).valid) {
          taken_rvi_cf(i) := rvi_branch(i) & bht_prediction_shifted(i).taken
          taken_rvc_cf(i) := rvc_branch(i) & bht_prediction_shifted(i).taken
          // otherwise default to static prediction
        } .otherwise {
          // set if immediate is negative - static prediction
          taken_rvi_cf(i) := rvi_branch(i) & rvi_imm(i)(63)
          taken_rvc_cf(i) := rvc_branch(i) & rvc_imm(i)(63)
        }
        when (taken_rvi_cf(i) || taken_rvc_cf(i)) { cf_type(i) := Branch }
      }
      // default: $error("Decoded more than one control flow")
    }
    // if this instruction, in addition, is a call, save the resulting address
    // but only if we actually consumed the address
    when (is_call(i)) {
      ras_push := instr_queue_consumed(i)
      ras_update := addr(i) + Mux(rvc_call(i), 2.U, 4.U)
    }
    // calculate the jump target address
    when (taken_rvc_cf(i) || taken_rvi_cf(i)) {
      predict_address := addr(i) + Mux(taken_rvc_cf(i), rvc_imm(i), rvi_imm(i))
    }
  }

  // or reduce struct
  bp_valid := false.B
  for (i <- 0 until INSTR_PER_FETCH) {
    bp_valid = bp_valid | (cf_type(i) != NoCF)
  }

  is_mispredict := io.resolved_branch_i.valid & io.resolved_branch_i.is_mispredict

  // Cache interface
  io.icache_dreq_o.req := instr_queue_ready
  if_ready := io.icache_dreq_i.ready & instr_queue_ready
  // We need to flush the cache pipeline if:
  // 1. We mispredicted
  // 2. Want to flush the whole processor front-}
  // 3. Need to replay an instruction because the fetch-fifo was full
  io.icache_dreq_o.kill_s1 := is_mispredict | io.flush_i | replay
  // if we have a valid branch-prediction we need to only kill the last cache request
  // also if we killed the first stage we also need to kill the second stage (inclusive flush)
  io.icache_dreq_o.kill_s2 := io.icache_dreq_o.kill_s1 | bp_valid

  // Update Control Flow Predictions
  val bht_update = Wire(new bht_update_t())
  val btb_update = Wire(new btb_update_t())

  bht_update.valid := io.resolved_branch_i.valid & (io.resolved_branch_i.cf_type === Branch)
  bht_update.pc    := io.resolved_branch_i.pc
  bht_update.taken := io.resolved_branch_i.is_taken
  // only update mispredicted branches e.g. no returns from the RAS
  btb_update.valid := io.resolved_branch_i.valid &
                      io.resolved_branch_i.is_mispredict &
                      (io.resolved_branch_i.cf_type === JumpR)
  btb_update.pc    := io.resolved_branch_i.pc
  btb_update.target_address := io.resolved_branch_i.target_address

  // -------------------
  // Next PC
  // -------------------
  // next PC (NPC) can come from (in order of precedence):
  // 0. Default assignment/replay instruction
  // 1. Branch Predict taken
  // 2. Control flow change request (misprediction)
  // 3. Return from environment call
  // 4. Exception/Interrupt
  // 5. Pipeline Flush because of CSR side effects
  // Mis-predict handling is a little bit different
  // select PC a.k.a PC Gen

  val fetch_address = Wire(UInt(64.W))
  // check whether we come out of reset
  // this is a workaround. some tools have issues
  // having boot_addr_i in the asynchronous
  // reset assignment to npc_q, even though
  // boot_addr_i will be assigned a constant
  // on the top-level.
  when (npc_rst_load_q) {
    npc_d         := io.boot_addr_i
    fetch_address := io.boot_addr_i
  } .otherwise {
    fetch_address    := npc_q
    // keep stable by default
    npc_d            := npc_q
  }
  // 0. Branch Prediction
  when (bp_valid) {
    fetch_address := predict_address
    npc_d := predict_address
  }
  // 1. Default assignment
  when (if_ready) { npc_d := Cat(fetch_address(63,2), 0.U(2.W))  + 4.U }
  // 2. Replay instruction fetch
  when (replay) { npc_d := replay_addr }
  // 3. Control flow change request
  when (is_mispredict) { npc_d := io.resolved_branch_i.target_address }
  // 4. Return from environment call
  when (io.eret_i) { npc_d := io.epc_i }
  // 5. Exception/Interrupt
  when (io.ex_valid_i) { npc_d := io.trap_vector_base_i }
  // 6. Pipeline Flush because of CSR side effects
  // On a pipeline flush start fetching from the next address
  // of the instruction in the commit stage
  // we came here from a flush request of a CSR instruction or AMO
  // as CSR or AMO instructions do not exist in a compressed form
  // we can unconditionally do PC + 4 here
  // TODO(zarubaf) This adder can at least be merged with the one in the csr_regfile stage
  when (io.set_pc_commit_i) { npc_d := io.pc_commit_i + 4.U(64.W) }
  // 7. Debug
  // enter debug on a hard-coded base-address
  // when (set_debug_pc_i) npc_d := DmBaseAddress + dm::HaltAddress
  when (io.set_debug_pc_i) { npc_d := ArianeCfg.DmBaseAddress.U + 0x800.U(64.W) }
  io.icache_dreq_o.vaddr := fetch_address

  val icache_data = Wire(UInt(FETCH_WIDTH.W))
  // re-align the cache line
  icache_data := io.icache_dreq_i.data >> Cat(shamt, 0.U(4.W))

  npc_rst_load_q    := false.B
  npc_q             := npc_d
  icache_valid_q    := io.icache_dreq_i.valid
  when (io.icache_dreq_i.valid) {
    icache_data_q        := icache_data
    icache_vaddr_q       := io.icache_dreq_i.vaddr
    // icache_ex_valid_q    := io.icache_dreq_i.ex
    icache_ex_valid_q    := io.icache_dreq_i.ex.valid
    // save the uppermost prediction
    btb_q                := btb_prediction(INSTR_PER_FETCH-1)
    bht_q                := bht_prediction(INSTR_PER_FETCH-1)
  }

  val i_ras = Module(new ras(ArianeCfg.RASDepth))
  i_ras.io.flush_i := io.flush_bp_i
  i_ras.io.push_i  := ras_push
  i_ras.io.pop_i   := ras_pop
  i_ras.io.data_i  := ras_update
  ras_predict := i_ras.io.data_o


  val i_btb = Module(new btb(ArianeCfg.BTBEntries))
  i_btb.io.flush_i          := io.flush_bp_i
  i_btb.io.debug_mode_i     := io.debug_mode_i
  i_btb.io.vpc_i            := icache_vaddr_q
  i_btb.io.btb_update_i     := btb_update
  btb_prediction := i_btb.io.btb_prediction_o

  val i_bht = Module(new bht(ArianeCfg.BHTEntries))
  i_bht.io.flush_i          := io.flush_bp_i
  i_bht.io.debug_mode_i     := io.debug_mode_i
  i_bht.io.vpc_i            := icache_vaddr_q
  i_bht.io.bht_update_i     := bht_update
  bht_prediction := i_bht.io.bht_prediction_o

  // we need to inspect up to INSTR_PER_FETCH instructions for branches
  // and jumps
  for (i <- 0 until INSTR_PER_FETCH) { // gen_instr_scan
    val i_instr_scan = Module(new instr_scan())
    i_instr_scan.io.instr_i      := instr(i)
    rvi_return(i) := i_instr_scan.io.rvi_return_o
    rvi_call(i)   := i_instr_scan.io.rvi_call_o
    rvi_branch(i) := i_instr_scan.io.rvi_branch_o
    rvi_jalr(i)   := i_instr_scan.io.rvi_jalr_o
    rvi_jump(i)   := i_instr_scan.io.rvi_jump_o
    rvi_imm(i)    := i_instr_scan.io.rvi_imm_o
    rvc_branch(i) := i_instr_scan.io.rvc_branch_o
    rvc_jump(i)   := i_instr_scan.io.rvc_jump_o
    rvc_jr(i)     := i_instr_scan.io.rvc_jr_o
    rvc_return(i) := i_instr_scan.io.rvc_return_o
    rvc_jalr(i)   := i_instr_scan.io.rvc_jalr_o
    rvc_call(i)   := i_instr_scan.io.rvc_call_o
    rvc_imm(i)    := i_instr_scan.io.rvc_imm_o
  }

  val i_instr_queue = Module(new instr_queue())
  i_instr_queue.io.flush_i             := io.flush_i
  i_instr_queue.io.instr_i             := instr                 // from re-aligner
  i_instr_queue.io.addr_i              := addr                  // from re-aligner
  i_instr_queue.io.exception_i         := icache_ex_valid_q     // from I$
  i_instr_queue.io.predict_address_i   := predict_address
  i_instr_queue.io.cf_type_i           := cf_type
  i_instr_queue.io.valid_i             := instruction_valid     // from re-aligner
  instr_queue_consumed   := i_instr_queue.io.consumed_o
  instr_queue_ready      := i_instr_queue.io.ready_o
  replay                 := i_instr_queue.io.replay_o
  replay_addr            := i_instr_queue.io.replay_addr_o
  io.fetch_entry_o       := i_instr_queue.io.fetch_entry_o         // to back-}
  io.fetch_entry_valid_o := i_instr_queue.io.fetch_entry_valid_o   // to back-}
  i_instr_queue.io.fetch_entry_ready_i := io.fetch_entry_ready_i   // to back-}

}


object frontend extends App {
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

  chisel3.Driver.execute(args, () => new frontend(cfg))
}
