// Copyright (C) 2013-2018 ETH Zurich, University of Bologna
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the "License"); you may not use this file except in
// compliance with the License. You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.

// Author: Manuel Eggimann <meggimann@iis.ee.ethz.ch>

// Description: This module calculates the hamming weight (number of ones) in
// its input vector using a balanced binary adder tree. Recursive instantiation
// is used to build the tree.  Any unsigned INPUT_WIDTH larger or equal 2 is
// legal.  The module pads the signal internally to the next power of two.  The
// output result width is ceil(log2(INPUT_WIDTH))+1.

package ariane

import chisel3._
import chisel3.util._
import chisel3.Bool

class popcount(INPUT_WIDTH:Int = 256) extends Module
{
  val io = IO(new Bundle {
    val data_i     = Input(UInt(INPUT_WIDTH.W))
    val popcount_o = Output(UInt((log2Ceil(INPUT_WIDTH)+1).W))
  })

  val POPCOUNT_WIDTH:Int = log2Ceil(INPUT_WIDTH) + 1
  val PADDED_WIDTH  :Int = 1 << log2Ceil(INPUT_WIDTH)

  val padded_input = Wire(UInt(PADDED_WIDTH.W))
  val left_child_result  = Wire(UInt((POPCOUNT_WIDTH-1).W))
  val right_child_result = Wire(UInt((POPCOUNT_WIDTH-1).W))

  //Zero pad the input to next power of two
  padded_input := io.data_i

  //Recursive instantiation to build binary adder tree
  if (INPUT_WIDTH == 2) { // leaf_node
    left_child_result  := padded_input(1)
    right_child_result := padded_input(0)
  } else { // : non_leaf_node
    val left_child = Module(new popcount (PADDED_WIDTH / 2))
    left_child.io.data_i := padded_input(PADDED_WIDTH-1, PADDED_WIDTH/2)
    left_child_result    := left_child.io.popcount_o

    val right_child = Module(new popcount (PADDED_WIDTH / 2))
    right_child.io.data_i := padded_input(PADDED_WIDTH/2-1, 0)
    right_child_result    := right_child.io.popcount_o
  }

  //Output assignment
  io.popcount_o := left_child_result + right_child_result
}


object popcount extends App {
  chisel3.Driver.execute(args, () => new popcount(8))
}
