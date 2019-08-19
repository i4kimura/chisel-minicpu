// Author: Michael Schaffner <schaffner@iis.ee.ethz.ch>, ETH Zurich
// Date: 02.04.2019
// Description: logarithmic arbitration tree with round robin arbitration scheme.
//
// The rr_arb_tree employs fair round robin arbitration - i.e. the priorities
// rotate each cycle.
//
// The `LockIn` option prevents the arbiter from changing the arbitration
// decision when the arbiter is disabled. I.e., the index of the first request
// that wins the arbitration will be locked in case the destination is not
// able to grant the request in the same cycle.
//
// The `ExtPrio` option allows to override the internal round robin counter via the
// `rr_i` signal. This can be useful in case multiple arbiters need to have
// rotating priorities that are operating in lock-step. If static priority arbitration
// is needed, just connect `rr_i` to '0.
//
// If `AxiVldRdy` is set, the req/gnt signals are compliant with the AXI style vld/rdy
// handshake. Namely, upstream vld (req) must not depend on rdy (gnt), as it can be deasserted
// again even though vld is asserted. Enabling `AxiVldRdy` leads to a reduction of arbiter
// delay and area.
//

package cache_subsystem

import chisel3._
import chisel3.util._
import chisel3.Bool

import scala.math.{abs, round, sin, cos, Pi, pow}

class rr_arb_tree[T<:Data](
  dtype    : T       = UInt,
  NumIn    : Int     = 64,
  ExtPrio  : Boolean = false, // set to 1'b1 to enable
  AxiVldRdy: Boolean = false, // treat req/gnt as vld/rdy
  LockIn   : Boolean = false  // set to 1'b1 to enable
) extends Module {
  val io = IO(new Bundle {
    val flush_i = Input (Bool())                       // clears the arbiter state
    val rr_i    = Input (Vec(log2Ceil(NumIn), Bool())) // external RR prio (needs to be enabled above)

    // input requests and data
    val req_i = Input (Vec(NumIn, Bool()))
    /* verilator lint_off UNOPTFLAT */
    val gnt_o = Output(Vec(NumIn, Bool()))
    /* verilator lint_on UNOPTFLAT */
    val data_i = Input(Vec(NumIn, dtype))
    // arbitrated output
    val gnt_i = Input (Bool())
    val req_o = Output(Bool())
    val data_o = Output(dtype)
    val idx_o = Output(UInt(log2Ceil(NumIn).W))
  })

  // just pass through in this corner case
  if (NumIn == 1) {
    io.req_o    := io.req_i(0)
    io.gnt_o(0) := io.gnt_i
    io.data_o   := io.data_i(0)
    io.idx_o    := 0.U
  // non-degenerate cases
  } else {
    val NumLevels = log2Ceil(NumIn)
    val NumLevel2Pow = pow(2,NumLevels).asInstanceOf[Int]

    /* verilator lint_off UNOPTFLAT */
    val index_nodes = WireInit(VecInit(Seq.fill(NumLevel2Pow  )(0.U(NumLevels.W)))) // used to propagate the indices
    val data_nodes  = Wire(Vec(NumLevel2Pow-1, dtype)) // used to propagate the data
    val gnt_nodes   = WireInit(VecInit(Seq.fill(NumLevel2Pow-1)(false.B)))          // used to propagate the grant to masters
    val req_nodes   = WireInit(VecInit(Seq.fill(NumLevel2Pow-1)(false.B)))          // used to propagate the requests to slave
    /* lint_off */
    val rr_q  = Wire(Vec(NumLevels, Bool()))
    val req_d = Wire(Vec(NumIn, Bool()))

    // the final arbitration decision can be taken from the root of the tree
    io.req_o  := req_nodes(0)
    io.data_o := data_nodes(0)
    io.idx_o  := index_nodes(0)

    if (ExtPrio) {
      rr_q  := io.rr_i
      req_d := io.req_i
    } else { // gen_int_rr
      val rr_d = Wire(UInt(NumLevels.W))

      // lock arbiter decision in case we got at least one req and no acknowledge
      if (LockIn) { // gen_lock
        val lock_d = Wire(Bool())
        val lock_q = RegInit(false.B)
        val req_q  = RegInit(VecInit(Seq.fill(NumIn)(false.B)))

        lock_d := io.req_o & ~io.gnt_i
        req_d  := Mux(lock_q, req_q, io.req_i)

        when (io.flush_i) {
          lock_q := false.B
        } .otherwise {
          lock_q := lock_d
        }

        // // pragma translate_off
        // `ifndef VERILATOR
        //   lock: assert property(
        //     @(posedge clk_i) disable iff (!rst_ni) LockIn |-> req_o && !gnt_i |=> idx_o == $past(idx_o))
        //       else $fatal (1, "Lock implies same arbiter decision in next cycle if output is not ready.")
        //
        //   logic [NumIn-1:0] req_tmp
        //   req_tmp = req_q & req_i
        //   lock_req: assert property(
        //     @(posedge clk_i) disable iff (!rst_ni) LockIn |-> lock_d |=> req_tmp == req_q)
        //       else $fatal (1, "It is disallowed to deassert unserved request signals when LockIn is enabled.")
        // `endif
        // // pragma translate_on

        when (io.flush_i) {
          req_q := VecInit(Seq.fill(NumIn)(false.B))
        } .otherwise {
          req_q := req_d
        }
      } else { // : gen_no_lock
        req_d := io.req_i
      }

      rr_d := Mux(io.gnt_i && io.req_o, Mux(rr_q.asUInt === (NumIn-1).U, 0.U, rr_q.asUInt + 1.U),
                  rr_q.asUInt)

      val rr_q_reg = Reg(Vec(NumLevels, Bool()))

      when (io.flush_i) {
        rr_q_reg := VecInit(Seq.fill(NumLevels)(false.B))
      } .otherwise {
        rr_q_reg := rr_d.toBools
      }
      rr_q := rr_q_reg
    }

    gnt_nodes(0) := io.gnt_i

    // arbiter tree
    for (level <- 0 until NumLevels) { // gen_levels
      for (l <- 0 until pow(2,level).asInstanceOf[Int]) { // gen_level
        // local select signal
        val sel = WireInit(false.B)
        // index calcs
        val idx0:Int = pow(2, level   ).asInstanceOf[Int]-1+l          // current node
        val idx1:Int = pow(2,(level+1)).asInstanceOf[Int]-1 + l*2
        //////////////////////////////////////////////////////////////
        // uppermost level where data is fed in from the inputs
        if (level == NumLevels-1) { // gen_first_level
                                    // if two successive indices are still in the vector...
          if (l * 2 < NumIn-1) {
            req_nodes(idx0) := req_d(l*2) | req_d(l*2+1)

            // arbitration: round robin
            sel :=  ~req_d(l*2) | req_d(l*2+1) & rr_q(NumLevels-1-level)

            index_nodes(idx0) := sel
            data_nodes(idx0)  := Mux(sel, io.data_i(l*2+1), io.data_i(l*2))
            io.gnt_o(l*2  )   := gnt_nodes(idx0) & (AxiVldRdy.asBool | req_d(l*2))   & ~sel
            io.gnt_o(l*2+1)   := gnt_nodes(idx0) & (AxiVldRdy.asBool | req_d(l*2+1)) & sel
          }

          // if only the first index is still in the vector...
          if (l*2 == NumIn-1) {
            req_nodes(idx0)   := req_d(l*2)
            index_nodes(idx0) := 0.U// always zero in this case
            data_nodes(idx0)  := io.data_i(l*2)
            io.gnt_o(l*2)     := gnt_nodes(idx0) & (AxiVldRdy.asBool | req_d(l*2))
          }
          // if index is out of range, fill up with zeros (will get pruned)
          if (l * 2 > NumIn-1) {
            req_nodes(idx0)   := false.B
            index_nodes(idx0) := 0.U
            data_nodes(idx0)  := 0.U
          }
        //////////////////////////////////////////////////////////////
        // general case for other levels within the tree
        } else { // : gen_other_levels
          req_nodes(idx0)   := req_nodes(idx1) | req_nodes(idx1+1)

          // arbitration: round robin
          sel :=  ~req_nodes(idx1) | req_nodes(idx1+1) & rr_q(NumLevels-1-level)

          index_nodes(idx0) := Mux(sel, Cat(true.B,  index_nodes(idx1+1)(NumLevels-level-2,0)),
                                        Cat(false.B, index_nodes(idx1)(NumLevels-level-2,0)))
          data_nodes(idx0)  := Mux(sel, data_nodes(idx1+1), data_nodes(idx1))
          gnt_nodes(idx1)   := gnt_nodes(idx0) & ~sel
          gnt_nodes(idx1+1) := gnt_nodes(idx0) &  sel
        }
        //////////////////////////////////////////////////////////////
      }
    }

  }
}
