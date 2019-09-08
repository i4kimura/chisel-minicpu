// Copyright 2018 - 2019 ETH Zurich and University of Bologna.
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 2.0 (the "License"); you may not use this file except in
// compliance with the License.  You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-2.0. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.
//
// Author: Florian Zaruba, ETH Zurich
// Date: 08.02.2018
// Migrated: Luis Vitorio Cargnini, IEEE
// Date: 09.06.2018

package ariane

import chisel3._
import chisel3.util._
import chisel3.Bool

import ariane_pkg._

// branch history table - 2 bit saturation counter
class bht(NR_ENTRIES:Int = 1024) extends Module {
  val io = IO(new Bundle {
    val flush_i      = Input(Bool())
    val debug_mode_i = Input(Bool())
    val vpc_i        = Input(UInt(64.W))
    val bht_update_i = Input(new bht_update_t())
    // we potentially need INSTR_PER_FETCH predictions/cycle
    val bht_prediction_o = Output(Vec(INSTR_PER_FETCH, new bht_prediction_t()))
  })

  // the last bit is always zero, we don't need it for indexing
  val OFFSET:Int = 1
  // re-shape the branch history table
  val NR_ROWS:Int = NR_ENTRIES / INSTR_PER_FETCH
  // number of bits needed to index the row
  val ROW_ADDR_BITS:Int = log2Ceil(INSTR_PER_FETCH)
  // number of bits we should use for prediction
  val PREDICTION_BITS:Int = log2Ceil(NR_ROWS) + OFFSET + ROW_ADDR_BITS
  // // we are not interested in all bits of the address
  // unread i_unread (.d_i(|vpc_i))

  class bht_d_t extends Bundle {
    val valid = Bool()
    val saturation_counter = UInt(2.W)
  }
  val bht_d = Wire(Vec(NR_ROWS, Vec(INSTR_PER_FETCH, new bht_d_t())))
  val bht_q = Reg (Vec(NR_ROWS, Vec(INSTR_PER_FETCH, new bht_d_t())))

  val index     = Wire(UInt(log2Ceil(NR_ROWS).W))
  val update_pc = Wire(UInt(log2Ceil(NR_ROWS).W))
  val update_row_index = Wire(UInt(ROW_ADDR_BITS.W))
  val saturation_counter = Wire(UInt(2.W))

  index            := io.vpc_i(PREDICTION_BITS - 1, ROW_ADDR_BITS + OFFSET)
  update_pc        := io.bht_update_i.pc(PREDICTION_BITS - 1,ROW_ADDR_BITS + OFFSET)
  update_row_index := io.bht_update_i.pc(ROW_ADDR_BITS + OFFSET - 1,OFFSET)

  // prediction assignment
  for (i <- 0 until INSTR_PER_FETCH) { // gen_bht_output
    io.bht_prediction_o(i).valid := bht_q(index)(i).valid
    io.bht_prediction_o(i).taken := bht_q(index)(i).saturation_counter(1) === 1.U(1.W)
  }

  bht_d := bht_q
  saturation_counter := bht_q(update_pc)(update_row_index).saturation_counter

  when (io.bht_update_i.valid && !io.debug_mode_i) {
    bht_d(update_pc)(update_row_index).valid := true.B

    when (saturation_counter === 3.U(2.W)) {
      // we can safely decrease it
      when (!io.bht_update_i.taken) {
        bht_d(update_pc)(update_row_index).saturation_counter := saturation_counter - 1.U
      }

      // then check if it saturated in the negative regime e.g., branch not taken
    } .elsewhen (saturation_counter === 0.U(2.W)) {
      // we can safely increase it
      when (io.bht_update_i.taken) {
        bht_d(update_pc)(update_row_index).saturation_counter := saturation_counter + 1.U
      }
    } .otherwise { // otherwise we are not in any boundaries and can decrease or increase it
      when (io.bht_update_i.taken) {
        bht_d(update_pc)(update_row_index).saturation_counter := saturation_counter + 1.U
      } .otherwise {
        bht_d(update_pc)(update_row_index).saturation_counter := saturation_counter - 1.U
      }
    }
  }
  // evict all entries
  when (io.flush_i) {
    for (i <- 0 until NR_ROWS) {
      for (j <- 0 until INSTR_PER_FETCH) {
        bht_q(i)(j).valid <=  false.B
        bht_q(i)(j).saturation_counter <= 2.U(2.W)
      }
    }
  } .otherwise {
    bht_q := bht_d
  }

}

object bht extends App {
  chisel3.Driver.execute(args, () => new bht(1024))
}
