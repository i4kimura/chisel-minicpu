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
// Author: Florian Zaruba <zarubaf@iis.ee.ethz.ch>
// Description: Instruction Re-aligner
//
// This module takes 32-bit aligned cache blocks and extracts the instructions.
// As we are supporting the compressed instruction set extension in a 32 bit instruction word
// are up to 2 compressed instructions.
// Furthermore those instructions can be arbitrarily interleaved which makes it possible to fetch
// only the lower part of a 32 bit instruction.
// Furthermore we need to handle the case if we want to start fetching from an unaligned
// instruction e.g. a branch.

package cache_subsystem

import chisel3._
import chisel3.util._
import chisel3.Bool

import ariane_pkg._

class instr_realign extends Module {
  val io = IO(new Bundle {
    val flush_i = Input(Bool())
    val valid_i = Input(Bool())
    val serving_unaligned_o = Output(Bool()) // we have an unaligned instruction in [0]
    val address_i = Input(UInt(64.W))
    val data_i  = Input(UInt(FETCH_WIDTH.W))
    val valid_o = Output(Vec(INSTR_PER_FETCH, Bool()))
    val addr_o  = Output(Vec(INSTR_PER_FETCH, UInt(64.W)))
    val instr_o = Output(Vec(INSTR_PER_FETCH, UInt(32.W)))
  })

  // as a maximum we support a fetch width of 64-bit, hence there can be 4 compressed instructions
  val instr_is_compressed = Wire(Vec(INSTR_PER_FETCH, Bool()))
  for (i <- 0 until INSTR_PER_FETCH) {
    // LSB != 2'b11
    instr_is_compressed(i) := ~(io.data_i(i*16 + 1, i*16).andR)
  }

  // save the unaligned part of the instruction to this ff
  val unaligned_instr_d = Wire(UInt(16.W))
  val unaligned_instr_q = RegInit(0.U(16.W))
  // the last instruction was unaligned
  val unaligned_d = Wire(Bool())
  val unaligned_q = RegInit(false.B)

  // register to save the unaligned address
  val unaligned_address_d = Wire(UInt(64.W))
  val unaligned_address_q = RegInit(0.U(64.W))
  // we have an unaligned instruction
  io.serving_unaligned_o := unaligned_q;

  // Instruction re-alignment
  if (FETCH_WIDTH == 32) { // begin : realign_bp_32

    // always_comb begin : re_align
    unaligned_d := unaligned_q
    unaligned_address_d := Cat(io.address_i(63,2), 2.U(2.W))
    unaligned_instr_d := io.data_i(31,16)

    io.valid_o(0) := io.valid_i
    io.instr_o(0) := Mux(unaligned_q, Cat(io.data_i(15,0), unaligned_instr_q), io.data_i(31,0))
    io.addr_o (0) := Mux(unaligned_q, unaligned_address_q, io.address_i)

    io.valid_o(1) := false.B
    io.instr_o(1) := 0.U
    io.addr_o (1) := Cat(io.address_i(63,2), 2.U(2.W))

    // this instruction is compressed or the last instruction was unaligned
    when (instr_is_compressed(0) || unaligned_q) {
      // check if this is instruction is still unaligned e.g.: it is not compressed
      // if its compressed re-set unaligned flag
      // for 32 bit we can simply check the next instruction and whether it is compressed or not
      // if it is compressed the next fetch will contain an aligned instruction
      // is instruction 1 also compressed
      // yes? -> no problem, no -> we've got an unaligned instruction
      when (instr_is_compressed(1)) {
        unaligned_d := false.B
        io.valid_o(1) := io.valid_i
        io.instr_o(1) := Cat(0.U(16.W), io.data_i(31,16))
      } .otherwise {
        // save the upper bits for next cycle
        unaligned_d := true.B
        unaligned_instr_d := io.data_i(31,16)
        unaligned_address_d := Cat(io.address_i(63,2), 2.U(2.W))
      }
    } // else -> normal fetch

    // we started to fetch on a unaligned boundary with a whole instruction -> wait until we've
    // received the next instruction
    when (io.valid_i && io.address_i(1)) {
      // the instruction is not compressed so we can't do anything in this cycle
      when (!instr_is_compressed(0)) {
        io.valid_o          := VecInit(Seq.fill(INSTR_PER_FETCH)(false.B))
        unaligned_d         := true.B
        unaligned_address_d := Cat(io.address_i(63,2), 2.U(2.W))
        unaligned_instr_d   := io.data_i(15,0)
        // the instruction isn't compressed but only the lower is ready
      } .otherwise {
        io.valid_o(0) := true.B
      }
    }

    // TODO(zarubaf): Fix 64 bit FETCH_WIDTH, maybe generalize to arbitrary fetch width
  } else if (FETCH_WIDTH == 64) { // : realign_bp_64

    // initial {
    //   $error("Not propperly implemented")
    // }
    // always_comb begin : re_align
    unaligned_d         := unaligned_q
    unaligned_address_d := unaligned_address_q
    unaligned_instr_d   := unaligned_instr_q

    io.valid_o    := VecInit(Seq.fill(INSTR_PER_FETCH)(false.B))
    io.valid_o(0) := io.valid_i

    io.instr_o(0) := io.data_i(31, 0)
    io.addr_o(0)  := io.address_i

    io.instr_o(1) := 0.U
    io.addr_o(1)  := Cat(io.address_i(63,3), 2.U(3.W))

    io.instr_o(2) := Cat(0.U(16.W), io.data_i(47,32))
    io.addr_o(2)  := Cat(io.address_i(63,3), 4.U(3.W))

    io.instr_o(3) := Cat(0.U(16.W), io.data_i(63,48))
    io.addr_o(3)  := Cat(io.address_i(63,3), 6.U(3.W))

    // last instruction was unaligned
    when (unaligned_q) {
      io.instr_o(0) := Cat(io.data_i(15,0), unaligned_instr_q)
      io.addr_o (0) := unaligned_address_q
      // for 64 bit there exist the following options:
      //     64      32      0
      //     | 3 | 2 | 1 | 0 | <- instruction slot
      // |   I   |   I   |   U   | -> again unaligned
      // | * | C |   I   |   U   | -> aligned
      // | * |   I   | C |   U   | -> aligned
      // |   I   | C | C |   U   | -> again unaligned
      // | * | C | C | C |   U   | -> aligned
      // Legend: C := compressed, I := 32 bit instruction, U := unaligned upper half
      //         * := don't care
      when (instr_is_compressed(1)) {
        io.instr_o(1) := Cat(0.U(16.W), io.data_i(31,16))
        io.valid_o(1) := io.valid_i

        when (instr_is_compressed(2)) {
          when (instr_is_compressed(3)) {
            unaligned_d := false.B
            io.valid_o(3) := io.valid_i
          } .otherwise {
            // continues to be unaligned
          }
        } .otherwise {
          unaligned_d := false.B
          io.instr_o(2) := io.data_i(63,32)
          io.valid_o(2) := io.valid_i
        }
        // instruction 1 is not compressed
      } .otherwise {
        io.instr_o(1) := io.data_i(47,16)
        io.valid_o(1) := io.valid_i
        io.addr_o(2) := Cat(io.address_i(63,3), 6.U(3.W))
        when (instr_is_compressed(2)) {
          unaligned_d := false.B
          io.instr_o(2) := Cat(0.U(16.W), io.data_i(63,48))
          io.valid_o(2) := io.valid_i
        } .otherwise {
          // continues to be unaligned
        }
      }
    } .elsewhen (instr_is_compressed(0)) { // instruction zero is RVC

      //     64     32       0
      //     | 3 | 2 | 1 | 0 | <- instruction slot
      // |   I   |   I   | C | -> again unaligned
      // | * | C |   I   | C | -> aligned
      // | * |   I   | C | C | -> aligned
      // |   I   | C | C | C | -> again unaligned
      // | * | C | C | C | C | -> aligned
      when (instr_is_compressed(1)) {
        io.instr_o(1) := Cat(0.U(16.W), io.data_i(31,16))
        io.valid_o(1) := io.valid_i

        when (instr_is_compressed(2)) {
          io.valid_o(2) := io.valid_i
          when (instr_is_compressed(3)) {
            io.valid_o(3) := io.valid_i
          } .otherwise {
            // this instruction is unaligned
            unaligned_d := true.B
            unaligned_instr_d := io.data_i(63,48)
            unaligned_address_d := io.addr_o(3)
          }
        } .otherwise {
          io.instr_o(2) := io.data_i(63,32)
          io.valid_o(2) := io.valid_i
        }
        // instruction 1 is not compressed -> check slot 3
      } .otherwise {
        io.instr_o(1) := io.data_i(47,16)
        io.valid_o(1) := io.valid_i
        io.addr_o(2) := Cat(io.address_i(63,3), 6.U(3.W))
        when (instr_is_compressed(3)) {
          io.instr_o(2) := io.data_i(63,48)
          io.valid_o(2) := io.valid_i
        } .otherwise {
          unaligned_d := true.B
          unaligned_instr_d := io.data_i(63,48)
          unaligned_address_d := io.addr_o(2)
        }
      }

      // Full instruction in slot zero
      //     64     32       0
      //     | 3 | 2 | 1 | 0 | <- instruction slot
      // |   I   | C |   I   |
      // | * | C | C |   I   |
      // | * |   I   |   I   |
    } .otherwise {
      io.addr_o(1) := Cat(io.address_i(63,3), 4.U(3.W))

      when (instr_is_compressed(2)) {
        io.instr_o(1) := Cat(0.U(16.W), io.data_i(47,32))
        io.valid_o(1) := io.valid_i
        io.addr_o(2) := Cat(io.address_i(63,3), 6.U(3.W))
        when (instr_is_compressed(3)) {
          // | * | C | C |   I   |
          io.valid_o(2) := io.valid_i
          io.addr_o(2) := Cat(0.U(16.W), io.data_i(63,48))
        } .otherwise {
          // this instruction is unaligned
          unaligned_d := true.B
          unaligned_instr_d   := io.data_i(63,48)
          unaligned_address_d := io.addr_o(2)
        }
      } .otherwise {
        // two regular instructions back-to-back
        io.instr_o(1) := io.data_i(63,32)
        io.valid_o(1) := io.valid_i
      }
    }

    // --------------------------
    // Unaligned fetch
    // --------------------------
    // Address was not 64 bit aligned
    switch (io.address_i(2,1)) {
      // this means the previouse instruction was either compressed or unaligned
      // in any case we don't ccare
      is (1.U(2.W)) {
        //     64     32       0
        //     | 3 | 2 | 1 | 0 | <- instruction slot
        // |   I   |   I   | x  -> again unaligned
        // | * | C |   I   | x  -> aligned
        // | * |   I   | C | x  -> aligned
        // |   I   | C | C | x  -> again unaligned
        // | * | C | C | C | x  -> aligned
        io.addr_o(0) := Cat(io.address_i(63,3), 2.U(3.W))

        when (instr_is_compressed(1)) {
          io.instr_o(0) := Cat(0.U(16.W), io.data_i(31,16))
          io.valid_o(0) := io.valid_i

          when (instr_is_compressed(2)) {
            io.valid_o(1) := io.valid_i
            io.instr_o(1) := Cat(0.U(16.W), io.data_i(47,32))
            io.addr_o(1) := Cat(io.address_i(63,3), 4.U(3.W))
            when (instr_is_compressed(3)) {
              io.instr_o(2) := Cat(0.U(16.W), io.data_i(63,48))
              io.addr_o(2) := Cat(io.address_i(63,3), 6.U(3.W))
              io.valid_o(2) := io.valid_i
            } .otherwise {
              // this instruction is unaligned
              unaligned_d := true.B
              unaligned_instr_d := io.data_i(63,48)
              unaligned_address_d := io.addr_o(3)
            }
          } .otherwise {
            io.instr_o(1) := io.data_i(63,32)
            io.addr_o(1) := Cat(io.address_i(63,3), 4.U(3.W))
            io.valid_o(1) := io.valid_i
          }
          // instruction 1 is not compressed -> check slot 3
        } .otherwise {
          io.instr_o(0) := io.data_i(47,16)
          io.valid_o(0) := io.valid_i
          io.addr_o(1) := Cat(io.address_i(63,3), 6.U(3.W))
          when (instr_is_compressed(3)) {
            io.instr_o(1) := io.data_i(63,48)
            io.valid_o(1) := io.valid_i
          } .otherwise {
            unaligned_d := true.B
            unaligned_instr_d := io.data_i(63,48)
            unaligned_address_d := io.addr_o(1)
          }
        }
      }
      is (2.U(2.W)) {
        io.valid_o := VecInit(Seq.fill(INSTR_PER_FETCH)(false.B))
        //     64     32       0
        //     | 3 | 2 | 1 | 0 | <- instruction slot
        // |   I   | C |   *   | <- unaligned
        //     | C | C |   *   | <- aligned
        //     |   I   |   *   | <- aligned
        when (instr_is_compressed(2)) {
          io.valid_o(0) := io.valid_i
          io.instr_o(0) := io.data_i(47,32)
          // second instruction is also compressed
          when (instr_is_compressed(3)) {
            io.valid_o(1) := io.valid_i
            io.instr_o(1) := io.data_i(63,48)
            // regular instruction -> unaligned
          } .otherwise {
            unaligned_d := true.B
            unaligned_address_d := Cat(io.address_i(63,3), 6.U(3.W))
            unaligned_instr_d := io.data_i(63,48)
          }
          // instruction is a regular instruction
        } .otherwise {
          io.valid_o(0) := io.valid_i
          io.instr_o(0) := io.data_i(63,32)
          io.addr_o (0) := io.address_i
        }
      }
      // we started to fetch on a unaligned boundary with a whole instruction -> wait until we've
      // received the next instruction
      is (3.U(2.W)) {
        io.valid_o := VecInit(Seq.fill(INSTR_PER_FETCH)(false.B))
        when (!instr_is_compressed(3)) {
          unaligned_d := true.B
          unaligned_address_d := Cat(io.address_i(63,3), 6.U(3.W))
          unaligned_instr_d := io.data_i(63,48)
        } .otherwise {
          io.valid_o(3) := io.valid_i
        }
      }
    }
  }

  when (io.valid_i) {
    unaligned_address_q := unaligned_address_d
    unaligned_instr_q   := unaligned_instr_d
  }

  when (io.flush_i) {
    unaligned_q := false.B
  } .elsewhen (io.valid_i) {
    unaligned_q := unaligned_d
  }

}


object instr_realign extends App {
  chisel3.Driver.execute(args, () => new instr_realign())
}
