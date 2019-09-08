package ariane

import chisel3._
import chisel3.util._
import chisel3.Bool

import wt_cache_pkg._
import ariane_pkg._

// --------------
// 8-bit LFSR
// --------------
//
// Description: Shift register
//
class lfsr_8bit (
  SEED : Int = 0,
  WIDTH: Int = 8
) extends Module {
  val io = IO(new Bundle {
    val en_i = Input(Bool())
    val refill_way_oh  = Output(UInt(WIDTH.W))
    val refill_way_bin = Output(UInt(log2Ceil(WIDTH).W))
  })

  val LOG_WIDTH = log2Ceil(WIDTH)

  val shift_d = Wire(UInt(8.W))
  val shift_q = RegInit(SEED.U(8.W))

  val shift_in = Wire(UInt(8.W))
  shift_in := !(shift_q(7) ^ shift_q(3) ^ shift_q(2) ^ shift_q(1))
  shift_d := shift_q;

  when (io.en_i) { shift_d := Cat(shift_q(6, 0), shift_in) }
  // output assignment
  val refill_way_oh_vec = WireInit(VecInit(Seq.fill(WIDTH)(false.B)))
  refill_way_oh_vec(shift_q(LOG_WIDTH-1,0)) := true.B

  io.refill_way_oh  := refill_way_oh_vec.asUInt
  io.refill_way_bin := shift_q

  shift_q := shift_d

}
