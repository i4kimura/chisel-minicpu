// For each failed trial (set_i pulsed), this unit exponentially increases the
// (average) backoff time by masking an LFSR with a shifted mask in order to
// create the backoff counter initial value.
//
// The shift register mask and the counter value are both reset to '0 in case of
// a successful trial (clr_i).
//

package cache_subsystem

import chisel3._
import chisel3.util._
import chisel3.Bool

import wt_cache_pkg._
import ariane_pkg._

class exp_backoff (
  Seed  :Int = 0xffff, // seed for 16bit lfsr
  MaxExp:Int = 16      // 2**MaxExp-1 determines the maximum range from which random wait counts are drawn
) extends Module {
  val io = IO(new Bundle {
    val set_i = Input(Bool())      // sets the backoff counter (pulse) -> use when trial did not succeed
    val clr_i = Input(Bool())      // clears the backoff counter (pulse) -> use when trial succeeded
    val is_zero_o = Output(Bool()) // indicates whether the backoff counter is equal to zero and a new trial can be launched
  })

  // leave this constant
  val WIDTH = 16;

  val lfsr_d = Wire(UInt(WIDTH.W))
  val lfsr_q = RegInit(Seed.U(WIDTH.W))
  val cnt_d = Wire(UInt(WIDTH.W))
  val cnt_q = RegInit(0.U(WIDTH.W))
  val mask_d = Wire(UInt(WIDTH.W))
  val mask_q = RegInit(0.U(WIDTH.W))
  val lfsr = Wire(Bool())

  // generate random wait counts
  // note: we use a flipped lfsr here to
  // avoid strange correlation effects between
  // the (left-shifted) mask and the lfsr
  lfsr := lfsr_q(15-15) ^
          lfsr_q(15-13) ^
          lfsr_q(15-12) ^
          lfsr_q(15-10)

  lfsr_d := Mux(io.set_i, Cat(lfsr, lfsr_q(WIDTH-1,1)), lfsr_q)

  // mask the wait counts with exponentially increasing mask (shift reg)
  if (WIDTH == MaxExp) {
    mask_d := Mux(io.clr_i, 0.U,
              Mux(io.set_i, Cat(mask_q(MaxExp-2,0), 0.U(1.W)),
                            mask_q))
  } else {
    mask_d := Mux(io.clr_i, 0.U,
              Mux(io.set_i, Cat(VecInit(Seq.fill(WIDTH-MaxExp)(false.B)).asUInt, mask_q(MaxExp-2,0), 0.U(1.W)),
                  mask_q))
  }

  cnt_d :=  Mux(io.clr_i, 0.U,
            Mux(io.set_i, mask_q & lfsr_q,
            Mux(!io.is_zero_o, cnt_q - 1.U, 0.U)))

  io.is_zero_o := (cnt_q === 0.U)

  lfsr_q := lfsr_d
  mask_q := mask_d
  cnt_q  := cnt_d

}
