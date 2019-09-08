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

// branch target buffer
class btb (NR_ENTRIES:Int = 8) extends Module {
  val io = IO(new Bundle {
    val flush_i      = Input(Bool())         // flush the btb
    val debug_mode_i = Input(Bool())

    val vpc_i  = Input(UInt(64.W))           // virtual PC from IF stage
    val btb_update_i     = Input(new btb_update_t())    // update btb with this information
    val btb_prediction_o = Output(Vec(INSTR_PER_FETCH, new btb_prediction_t()))  // prediction from btb
  })

  // the last bit is always zero, we don't need it for indexing
  val OFFSET:Int = 1
  // re-shape the branch history table
  val NR_ROWS:Int = NR_ENTRIES / INSTR_PER_FETCH
  // number of bits needed to index the row
  val ROW_ADDR_BITS:Int = log2Ceil(INSTR_PER_FETCH)
  // number of bits we should use for prediction
  val PREDICTION_BITS:Int = log2Ceil(NR_ROWS) + OFFSET + ROW_ADDR_BITS
  // prevent aliasing to degrade performance
  val ANTIALIAS_BITS:Int = 8

  // typedef for all branch target entries
  // we may want to try to put a tag field that fills the rest of the PC in-order to mitigate aliasing effects
  val btb_d = Wire(Vec(NR_ROWS, Vec(INSTR_PER_FETCH, new btb_prediction_t())))
  val btb_q = Reg (Vec(NR_ROWS, Vec(INSTR_PER_FETCH, new btb_prediction_t())))

  val index     = Wire(UInt(log2Ceil(NR_ROWS).W))
  val update_pc = Wire(UInt(log2Ceil(NR_ROWS).W))
  val update_row_index = Wire(UInt(ROW_ADDR_BITS.W))

  index     := io.vpc_i(PREDICTION_BITS - 1, ROW_ADDR_BITS + OFFSET)
  update_pc := io.btb_update_i.pc(PREDICTION_BITS - 1, ROW_ADDR_BITS + OFFSET)
  update_row_index := io.btb_update_i.pc(ROW_ADDR_BITS + OFFSET - 1, OFFSET)

  // output matching prediction
  for (i <- 0 until INSTR_PER_FETCH) { // : gen_btb_output
    io.btb_prediction_o(i) := btb_q(index)(i) // workaround
  }

  // -------------------------
  // Update Branch Prediction
  // -------------------------
  // update on a mis-predict
  btb_d := btb_q
  when (io.btb_update_i.valid && !io.debug_mode_i) {
    btb_d(update_pc)(update_row_index).valid := true.B
    // the target address is simply updated
    btb_d(update_pc)(update_row_index).target_address := io.btb_update_i.target_address
  }

  // evict all entries
  when (io.flush_i) {
    for (i <- 0 until NR_ROWS) {
      for (j <- 0 until INSTR_PER_FETCH) {
        btb_q(i)(j).valid := false.B
      }
    }
  } .otherwise {
    btb_q := btb_d
  }
}


object btb extends App {
  chisel3.Driver.execute(args, () => new btb(8))
}
