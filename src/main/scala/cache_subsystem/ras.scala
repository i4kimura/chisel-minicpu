//Copyright (C) 2018 to present,
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

import wt_cache_pkg._
import ariane_pkg._

// return address stack
class ras (DEPTH: Int = 2) extends Module
{
  val io = IO(new Bundle {
    val flush_i = Input(Bool())
    val push_i  = Input(Bool())
    val pop_i   = Input(Bool())
    val data_i  = Input(UInt(64.W))
    val data_o = Output(new ras_t())
  })

  val stack_d = Wire(Vec(DEPTH, new ras_t()))
  val stack_q = Reg (Vec(DEPTH, new ras_t()))

  io.data_o := stack_q(0)

  stack_d := stack_q

  // push on the stack
  when (io.push_i) {
    stack_d(0).ra := io.data_i
    // mark the new return address as valid
    stack_d(0).valid := true.B
    for (i <- 1 until DEPTH) {
      stack_d(i) := stack_q(i-1)
    }
  }

  when (io.pop_i) {
    for (i <- 1 until DEPTH) {
      stack_d(i-1) := stack_q(i)
    }
    // we popped the value so invalidate the end of the stack
    stack_d(DEPTH-1).valid := false.B
    stack_d(DEPTH-1).ra := 0.U
  }
  // leave everything untouched and just push the latest value to the
  // top of the stack
  when (io.pop_i && io.push_i) {
    stack_d := stack_q
    stack_d(0).ra := io.data_i
    stack_d(0).valid := true.B
  }

  when (io.flush_i) {
    for(i <- 0 until DEPTH) {
      stack_d(i).valid := false.B
      stack_d(i).ra    := 0.U
    }
  }

  stack_q := stack_d
}
