// Copyright 2018 - 2019 ETH Zurich and University of Bologna.
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
// Date: 26.10.2018sim:/ariane_tb/dut/i_ariane/i_frontend/icache_ex_valid_q

// Description: Instruction Queue, separates instruction front-end from processor
//              back-end.
//
// This is an optimized instruction queue which supports the handling of
// compressed instructions (16 bit instructions). Internally it is organized as
// FETCH_ENTRY x 32 bit queues which are filled in a consecutive manner. Two pointers
// point into (`idx_is_q` and `idx_ds_q`) the fill port and the read port. The read port
// is designed so that it will easily allow for multiple issue implementation.
// The input supports arbitrary power of two instruction fetch widths.
//
// The queue supports handling of branch prediction and will take care of
// only saving a valid instruction stream.
//
// Furthermore it contains a replay interface in case the instruction queue
// is already full. As instructions are in general easily replayed this should
// increase the efficiency as I$ misses are potentially hidden. This stands in
// contrast to pessimistic actions (early stalling) or credit based approaches.
// Credit based systems might be difficult to implement with the current system
// as we do not exactly know how much space we are going to need in the fifos
// as each instruction can take either one or two slots.
//
// So the consumed/valid interface degenerates to a `information` interface. If the
// upstream circuits keeps pushing the queue will discard the information
// and start replaying from the point were it could last manage to accept instructions.
//
// The instruction front-end will stop issuing instructions as soon as the
// fifo is full. This will gate the logic if the processor is e.g.: halted
//
// TODO(zarubaf): The instruction queues can be reduced to 16 bit. Potentially
// the replay mechanism gets more complicated as it can be that a 32 bit instruction
// can not be pushed at once.

package ariane

import chisel3._
import chisel3.util._
import chisel3.Bool

import ariane_pkg._
import riscv_pkg._
import ariane_if_id._


class instr_queue extends Module {
  val io = IO(new Bundle {
    val flush_i = Input (Bool())
    val instr_i = Input (Vec(INSTR_PER_FETCH, UInt(32.W)))
    val addr_i  = Input (Vec(INSTR_PER_FETCH, UInt(64.W)))
    val valid_i = Input (Vec(INSTR_PER_FETCH, Bool()))
    val ready_o = Output(Bool())
    val consumed_o = Output(Vec(INSTR_PER_FETCH, Bool()))
    // we've encountered an exception, at this point the only possible exceptions are page-table faults
    val exception_i = Input (Bool())
    // branch predict
    val predict_address_i = Input (UInt(64.W))
    val cf_type_i = Input(Vec(INSTR_PER_FETCH, UInt(3.W)))  // new cf_t()))
    // replay instruction because one of the FIFO was already full
    val replay_o      = Output(Bool())
    val replay_addr_o = Output(UInt(64.W)) // address at which to replay this instruction

    // to processor backend
    val fetch_entry_o = Output(new fetch_entry_t())
    val fetch_entry_valid_o = Output(Bool())
    val fetch_entry_ready_i = Input (Bool())
  })

  class instr_data_t extends Bundle {
    val instr = UInt(32.W) // instruction word
    val cf    = UInt(3.W)  // new cf_t() // branch was taken
    val ex    = Bool()     // exception happened
  }

  val branch_index = Wire(UInt(log2Ceil(INSTR_PER_FETCH).W))
  // instruction queues
  val instr_queue_usage = Wire(Vec(INSTR_PER_FETCH, UInt(log2Ceil(FETCH_FIFO_DEPTH).W)))
  val instr_data_in  = Wire(Vec(INSTR_PER_FETCH, new instr_data_t()))
  val instr_data_out = Wire(Vec(INSTR_PER_FETCH, new instr_data_t()))
  val push_instr = Wire(Vec(INSTR_PER_FETCH, Bool()))
  val push_instr_fifo = Wire(Vec(INSTR_PER_FETCH, Bool()))
  val pop_instr         = Wire(Vec(INSTR_PER_FETCH, Bool()))
  val instr_queue_full  = Wire(Vec(INSTR_PER_FETCH, Bool()))
  val instr_queue_empty = Wire(Vec(INSTR_PER_FETCH, Bool()))
  val instr_overflow    = Wire(Bool())
  // address queue
  val address_queue_usage = Wire(UInt(log2Ceil(FETCH_FIFO_DEPTH).W))
  val address_out = Wire(UInt(64.W))
  val pop_address      = Wire(Bool())
  var push_address     = Wire(Bool())
  val full_address     = Wire(Bool())
  val empty_address    = Wire(Bool())
  val address_overflow = Wire(Bool())
  // input stream counter
  val idx_is_d = Wire(UInt(log2Ceil(INSTR_PER_FETCH).W))
  val idx_is_q = RegInit(0.U(log2Ceil(INSTR_PER_FETCH).W))
  // Registers
  // output FIFO select, one-hot
  val idx_ds_d = Wire(UInt(INSTR_PER_FETCH.W))
  val idx_ds_q = RegInit(0.U(INSTR_PER_FETCH.W))
  val pc_d = Wire(UInt(64.W))
  val pc_q = RegInit(0.U(64.W))  // current PC
  val reset_address_d = Wire(Bool())
  val reset_address_q = RegInit(false.B) // we need to re-set the address because of a flush

  val branch_mask_extended = Wire(UInt((INSTR_PER_FETCH*2-1).W))
  val branch_mask          = Wire(UInt(INSTR_PER_FETCH.W))
  val branch_empty         = Wire(Bool())
  val taken = Wire(Vec(INSTR_PER_FETCH, Bool()))
  // shift amount, e.g.: instructions we want to retire
  val popcount = Wire(UInt((log2Ceil(INSTR_PER_FETCH)+1).W))
  val shamt    = Wire(UInt(log2Ceil(INSTR_PER_FETCH).W))
  val valid = Wire(Vec(INSTR_PER_FETCH, Bool()))
  val consumed_extended = Wire(UInt((INSTR_PER_FETCH*2).W))
  // FIFO mask
  val fifo_pos_extended = Wire(UInt((INSTR_PER_FETCH*2).W))
  val fifo_pos = Wire(Vec(INSTR_PER_FETCH, Bool()))
  val instr = Wire(Vec(INSTR_PER_FETCH*2, UInt(32.W)))
  val cf = Wire(Vec(INSTR_PER_FETCH*2, UInt(3.W))) // new cf_t()))
  // replay interface
  val instr_overflow_fifo = Wire(Vec(INSTR_PER_FETCH, Bool()))

  io.ready_o := ~(instr_queue_full.reduce(_|_)) & ~full_address

  for (i <- 0 until INSTR_PER_FETCH) { // : gen_unpack_taken
    taken(i) := io.cf_type_i(i) =/= NoCF
  }

  // calculate a branch mask, e.g.: get the first taken branch
  val i_lzc_branch_index = Module(new lzc(INSTR_PER_FETCH, false)) // count trailing zeros
  i_lzc_branch_index.io.in_i    := taken         // we want to count trailing zeros
  branch_index := i_lzc_branch_index.io.cnt_o    // first branch on branch_index
  branch_empty := i_lzc_branch_index.io.empty_o

  // the first index is for sure valid
  // for example (64 bit fetch):
  // taken mask: 0 1 1 0
  // leading zero count := 1
  // 0 0 0 1, 1 1 1 << 1 := 0 0 1 1, 1 1 0
  // take the upper 4 bits: 0 0 1 1
  branch_mask_extended := Cat(VecInit(Seq.fill(INSTR_PER_FETCH-1)(false.B)).asUInt,
                              VecInit(Seq.fill(INSTR_PER_FETCH  )(false.B)).asUInt) << branch_index
  branch_mask := branch_mask_extended(INSTR_PER_FETCH * 2 - 2, INSTR_PER_FETCH - 1)

  // mask with taken branches to get the actual amount of instructions we want to push
  val branch_mask_bools = branch_mask.toBools
  for (i <- 0 until INSTR_PER_FETCH) { valid(i) := io.valid_i(i) & branch_mask_bools(i) }
  // rotate right again
  consumed_extended := Cat(push_instr_fifo.asUInt, push_instr_fifo.asUInt) >> idx_is_q
  for (i <- 0 until INSTR_PER_FETCH) {
    io.consumed_o(i) := consumed_extended(i)
  }

  // count the numbers of valid instructions we've pushed from this package
  val i_popcount = Module(new popcount(INSTR_PER_FETCH))
  i_popcount.io.data_i := push_instr_fifo.asUInt
  popcount := i_popcount.io.popcount_o

  shamt := popcount(shamt.getWidth-1,0)

  // save the shift amount for next cycle
  idx_is_d := idx_is_q + shamt

  // ----------------------
  // Input interface
  // ----------------------
  // rotate left by the current position
  fifo_pos_extended := Cat(valid.asUInt, valid.asUInt) << idx_is_q
  // we just care about the upper bits
  fifo_pos := fifo_pos_extended(INSTR_PER_FETCH*2-1, INSTR_PER_FETCH).toBools
  // the fifo_position signal can directly be used to guide the push signal of each FIFO
  // make sure it is not full
  for(i <- 0 until INSTR_PER_FETCH) { push_instr(i) := fifo_pos(i) & ~instr_queue_full(i) }

  // duplicate the entries for easier selection e.g.: 3 2 1 0 3 2 1 0
  for (i <- 0 until INSTR_PER_FETCH) { //gen_duplicate_instr_input
    instr(i)                   := io.instr_i(i)
    instr(i + INSTR_PER_FETCH) := io.instr_i(i)
    cf(i)                      := io.cf_type_i(i)
    cf(i + INSTR_PER_FETCH)    := io.cf_type_i(i)
  }

  // shift the inputs
  for (i <- 0 until INSTR_PER_FETCH) { // gen_fifo_input_select
    /* verilator lint_off WIDTH */
    instr_data_in(i).instr := instr(i.U + idx_is_q)
    instr_data_in(i).cf    := cf(i.U + idx_is_q)
    instr_data_in(i).ex    := io.exception_i; // exceptions hold for the whole fetch packet
  }

  // ----------------------
  // Replay Logic
  // ----------------------
  // We need to replay a instruction fetch iff:
  // 1. One of the instruction data FIFOs was full and we needed it
  // (e.g.: we pushed and it was full)
  // 2. The address/branch predict FIFO was full
  // if one of the FIFOs was full we need to replay the faulting instruction
  for(i <- 0 until INSTR_PER_FETCH) { instr_overflow_fifo(i) := instr_queue_full(i) & fifo_pos(i) }
  instr_overflow := instr_overflow_fifo.reduce(_|_) // at least one instruction overflowed
  address_overflow := full_address & push_address
  io.replay_o := instr_overflow | address_overflow

  // select the address, in the case of an address fifo overflow just
  // use the base of this package
  // if we successfully pushed some instructions we can output the next instruction
  // which we didn't manage to push
  io.replay_addr_o := Mux(address_overflow,  io.addr_i(0), io.addr_i(shamt))

  // ----------------------
  // Downstream interface
  // ----------------------
  // as long as there is at least one queue which can take the value we have a valid instruction
  io.fetch_entry_valid_o := ~(instr_queue_empty.reduce(_&_))

  idx_ds_d := idx_ds_q

  pop_instr := VecInit(Seq.fill(INSTR_PER_FETCH)(false.B))
  // assemble fetch entry
  io.fetch_entry_o.instruction := 0.U
  io.fetch_entry_o.address := pc_q
  io.fetch_entry_o.ex.valid := false.B
  // This is the only exception which can occur up to this point.
  io.fetch_entry_o.ex.cause := INSTR_PAGE_FAULT
  io.fetch_entry_o.ex.tval := 0.U
  io.fetch_entry_o.branch_predict.predict_address := address_out
  io.fetch_entry_o.branch_predict.cf := NoCF
  // output mux select
  for (i <- 0 until INSTR_PER_FETCH) {
    when (idx_ds_q(i)) {
      io.fetch_entry_o.instruction := instr_data_out(i).instr
      io.fetch_entry_o.ex.valid := instr_data_out(i).ex
      io.fetch_entry_o.ex.tval  := pc_q
      io.fetch_entry_o.branch_predict.cf := instr_data_out(i).cf
      pop_instr(i) := io.fetch_entry_valid_o & io.fetch_entry_ready_i
    }
    // rotate the pointer left
    when (io.fetch_entry_ready_i) {
      idx_ds_d := Cat(idx_ds_q(INSTR_PER_FETCH-2,0), idx_ds_q(INSTR_PER_FETCH-1))
    }
  }

  // TODO(zarubaf): This needs to change for dual-issue
  // if the handshaking is successful and we had a prediction pop one address entry
  pop_address := ((io.fetch_entry_o.branch_predict.cf =/= NoCF) & pop_instr.reduce(_|_))

  // ----------------------
  // Calculate (Next) PC
  // ----------------------
  pc_d := pc_q
  reset_address_d := Mux(io.flush_i, true.B, reset_address_q)

  when (io.fetch_entry_ready_i) {
    // TODO(zarubaf): This needs to change for a dual issue implementation
    // advance the PC
    pc_d :=  pc_q + Mux((io.fetch_entry_o.instruction(1, 0) =/= 3.U(2.W)), 2.U, 4.U)
  }

  when (pop_address) { pc_d := address_out }

  // we previously flushed so we need to reset the address
  when (io.valid_i(0) && reset_address_q) {
    // this is the base of the first instruction
    pc_d := io.addr_i(0)
    reset_address_d := false.B
  }

  // FIFOs
  for (i <- 0 until INSTR_PER_FETCH) { // : gen_instr_fifo

    // Make sure we don't save any instructions if we couldn't save the address
    push_instr_fifo(i) := push_instr(i) & ~address_overflow
    val i_fifo_instr_data = Module(new fifo_v3(new instr_data_t(), false, FETCH_FIFO_DEPTH))
    i_fifo_instr_data.io.flush_i    := io.flush_i
    i_fifo_instr_data.io.testmode_i := false.B
    instr_queue_full(i)  := i_fifo_instr_data.io.full_o
    instr_queue_empty(i) := i_fifo_instr_data.io.empty_o
    instr_queue_usage(i) := i_fifo_instr_data.io.usage_o
    i_fifo_instr_data.io.data_i     := instr_data_in(i)
    i_fifo_instr_data.io.push_i     := push_instr_fifo(i)
    instr_data_out(i) := i_fifo_instr_data.io.data_o
    i_fifo_instr_data.io.pop_i      := pop_instr(i)
  }

  // or reduce and check whether we are retiring a taken branch (might be that the corresponding)
  // fifo is full.
  push_address := false.B
  // check if we are pushing a ctrl flow change, if so save the address
  for (i <- 0 until INSTR_PER_FETCH) {
    push_address = push_address | (push_instr(i) & (instr_data_in(i).cf =/= NoCF))
  }

  val i_fifo_address = Module(new fifo_v3(UInt(64.W), false, FETCH_FIFO_DEPTH))
  i_fifo_address.io.flush_i    := io.flush_i
  i_fifo_address.io.testmode_i := false.B
  full_address        := i_fifo_address.io.full_o
  empty_address       := i_fifo_address.io.empty_o
  address_queue_usage := i_fifo_address.io.usage_o
  i_fifo_address.io.data_i     := io.predict_address_i
  i_fifo_address.io.push_i     := push_address & ~full_address
  address_out := i_fifo_address.io.data_o
  i_fifo_address.io.pop_i      := pop_address

  // unread i_unread_address_fifo (.d_i(|{empty_address, address_queue_usage}))
  // unread i_unread_branch_mask (.d_i(|branch_mask_extended))
  // unread i_unread_lzc (.d_i(|{branch_empty}))
  // unread i_unread_fifo_pos (.d_i(|fifo_pos_extended)); // we don't care about the lower signals
  // unread i_unread_instr_fifo (.d_i(|instr_queue_usage))

  pc_q := pc_d
  reset_address_q := reset_address_d
  when (io.flush_i) {
    // one-hot encoded
    idx_ds_q := true.B
    // binary encoded
    idx_is_q := 0.U
    reset_address_q := true.B
  } .otherwise {
    idx_ds_q := idx_ds_d
    idx_is_q := idx_is_d
  }
}


object instr_queue extends App {
  chisel3.Driver.execute(args, () => new instr_queue())
}
