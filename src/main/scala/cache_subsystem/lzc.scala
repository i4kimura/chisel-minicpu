package cache_subsystem

import chisel3._
import chisel3.util._
import chisel3.Bool

import scala.math.pow

/// A trailing zero counter / leading zero counter.
/// Set MODE to 0 for trailing zero counter => cnt_o is the number of trailing zeros (from the LSB)
/// Set MODE to 1 for leading zero counter  => cnt_o is the number of leading zeros  (from the MSB)
/// If the input does not contain a zero, `empty_o` is asserted. Additionally `cnt_o` contains
/// the maximum number of zeros - 1. For example:
///   in_i = 000_0000, empty_o = 1, cnt_o = 6 (mode = 0)
///   in_i = 000_0001, empty_o = 0, cnt_o = 0 (mode = 0)
///   in_i = 000_1000, empty_o = 0, cnt_o = 3 (mode = 0)
/// Furthermore, this unit contains a more efficient implementation for Verilator (simulation only).
/// This speeds up simulation significantly.

class lzc (
  /// The width of the input vector.
  WIDTH: Int = 2,
  MODE: Boolean = false // 0 -> trailing zero, 1 -> leading zero
) extends Module {
  val io = IO(new Bundle {
    val in_i    = Input(Vec(WIDTH, Bool()))
    val cnt_o   = Output(UInt(log2Ceil(WIDTH).W))
    val empty_o = Output(Bool())   // asserted if all bits in in_i are zero
  })

  val NUM_LEVELS: Int = log2Ceil(WIDTH)

  // pragma translate_off
  assert(WIDTH > 0, "input must be at least one bit wide")
  // pragma translate_on

  val index_lut   = WireInit(VecInit(Seq.fill(WIDTH)(0.U(NUM_LEVELS.W))))
  val sel_nodes   = WireInit(VecInit(Seq.fill(pow(2, NUM_LEVELS).asInstanceOf[Int])(false.B)))
  val index_nodes = WireInit(VecInit(Seq.fill(pow(2, NUM_LEVELS).asInstanceOf[Int])(0.U(NUM_LEVELS.W))))

  val in_tmp = Wire(Vec(WIDTH, Bool()))

  // reverse vector if required
  for (i <-0 until WIDTH) {
    if (MODE) {
      in_tmp(i) := io.in_i(WIDTH-1-i)
    } else {
      in_tmp(i) := io.in_i(i)
    }
  }

  for (j <- 0 until WIDTH) { // : g_index_lut
    index_lut(j) := j.U
  }


  for (level <- 0 until NUM_LEVELS) { // : g_levels
    val level2pow  = pow(2, level).asInstanceOf[Int]
    val leveln2pow = pow(2,(level+1)).asInstanceOf[Int]
    if (level == NUM_LEVELS-1) { // : g_last_level
      for (k <- 0 until level2pow) { // : g_level
                                    // if two successive indices are still in the vector...
        if (k * 2 < WIDTH-1) {
          sel_nodes  (level2pow-1+k) := in_tmp(k*2) | in_tmp(k*2+1)
          index_nodes(level2pow-1+k) := Mux(in_tmp(k*2) === true.B, index_lut(k*2), index_lut(k*2+1))
        }
        // if only the first index is still in the vector...
        if (k * 2 == WIDTH-1) {
          sel_nodes  (level2pow-1+k) := in_tmp(k*2)
          index_nodes(level2pow-1+k) := index_lut(k*2)
        }
        // if index is out of range
        if (k * 2 > WIDTH-1) {
          sel_nodes(level2pow-1+k)   := false.B
          index_nodes(level2pow-1+k) := 0.U
        }
      }
    } else {
      for (l <- 0 until level2pow) { //  : g_level
        sel_nodes  (level2pow-1+l) := sel_nodes(leveln2pow-1+l*2) | sel_nodes(leveln2pow-1+l*2+1)
        index_nodes(level2pow-1+l) := Mux(sel_nodes(leveln2pow-1+l*2) === true.B,
                                             index_nodes(leveln2pow-1+l*2),
                                             index_nodes(leveln2pow-1+l*2+1))
      }
    }
  }

  if(NUM_LEVELS > 0) {
    io.cnt_o := index_nodes(0)
  } else {
    io.cnt_o := 0.U
  }
  if (NUM_LEVELS > 0) {
    io.empty_o := ~sel_nodes(0)
  } else {
    io.empty_o := ~(io.in_i.asUInt.orR)
  }

}


object lzc extends App {
  chisel3.Driver.execute(args, () => new lzc(8))
}
