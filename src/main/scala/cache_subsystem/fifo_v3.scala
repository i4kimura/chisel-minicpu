package cache_subsystem

import chisel3._
import chisel3.util._
import chisel3.Bool

import wt_cache_pkg._
import ariane_pkg._

class fifo_v3 [T <: Data](
    dtype       :T = UInt,
    FALL_THROUGH:Boolean = false, // fifo is in fall-through mode
    DEPTH       :Int     = 8      // depth can be arbitrary from 0 to 2**32
) extends Module {
  val io = IO(new Bundle {
    val flush_i    = Input(Bool())         // flush the queue
    val testmode_i = Input(Bool())         // test_mode to bypass clock gating
                                           // status flags
    val full_o     = Output(Bool())             // queue is full
    val empty_o    = Output(Bool())             // queue is empty
    val usage_o    = Output(UInt(ADDR_DEPTH.W)) // fill pointer
                                                // as long as the queue is not full we can push new data
    val data_i     = Input(dtype)              // data to push into the queue
    val push_i     = Input(Bool())             // data is valid and can be pushed to the queue
                                               // as long as the queue is not empty we can pop new elements
    val data_o     = Output(dtype)             // output data
    val pop_i      = Input(Bool())             // pop head from queue
  })
  val ADDR_DEPTH: Int = if (DEPTH > 1) { log2Ceil(DEPTH) } else { 1 }

  // local parameter
  // FIFO depth - handle the case of pass-through, synthesizer will do constant propagation
  val FIFO_DEPTH:Int = if (DEPTH > 0) { DEPTH } else { 1 }
  // clock gating control
  val gate_clock = Wire(Bool())
  // pointer to the read and write section of the queue
  val read_pointer_n  = Wire(UInt(ADDR_DEPTH.W))
  val read_pointer_q  = RegInit(0.U(ADDR_DEPTH.W))
  val write_pointer_n = Wire(UInt(ADDR_DEPTH.W))
  val write_pointer_q = RegInit(0.U(ADDR_DEPTH.W))
  // keep a counter to keep track of the current queue status
  val status_cnt_n = Wire(UInt(ADDR_DEPTH.W))
  val status_cnt_q = RegInit(0.U(ADDR_DEPTH.W)) // this integer will be truncated by the synthesis tool
                                                 // actual memory
  val mem_n = Wire(Vec(FIFO_DEPTH, dtype))
  val mem_q = Reg (Vec(FIFO_DEPTH, dtype))

  io.usage_o := status_cnt_q(ADDR_DEPTH-1, 0)

  if (DEPTH == 0) {
    io.empty_o := ~io.push_i
    io.full_o  := ~io.pop_i
  } else {
    io.full_o  := (status_cnt_q === FIFO_DEPTH.U(ADDR_DEPTH,0))
    io.empty_o := (status_cnt_q === 0.U) & ~(FALL_THROUGH.asBool & io.push_i)
  }
  // status flags

  // read and write queue logic

  // default assignment
  read_pointer_n  := read_pointer_q
  write_pointer_n := write_pointer_q
  status_cnt_n    := status_cnt_q
  if (DEPTH == 0) {
    io.data_o := io.data_i
  } else {
    io.data_o := mem_q(read_pointer_q)
  }
  mem_n           := mem_q
  gate_clock      := true.B

  // push a new element to the queue
  when (io.push_i && ~io.full_o) {
    // push the data onto the queue
    mem_n(write_pointer_q) := io.data_i
    // un-gate the clock, we want to write something
    gate_clock := false.B
    // increment the write counter
    when (write_pointer_q === FIFO_DEPTH.U(ADDR_DEPTH-1, 0) - 1.U) {
      write_pointer_n := 0.U
    } .otherwise {
      write_pointer_n := write_pointer_q + 1.U
      // increment the overall counter
      status_cnt_n    := status_cnt_q + 1.U
    }

    when (io.pop_i && ~io.empty_o) {
      // read from the queue is a default assignment
      // but increment the read pointer...
      when (read_pointer_q === FIFO_DEPTH.U(ADDR_DEPTH-1, 0) - 1.U) {
        read_pointer_n := 0.U
      } .otherwise {
        read_pointer_n := read_pointer_q + 1.U
        // ... and decrement the overall count
        status_cnt_n   := status_cnt_q - 1.U
      }

      // keep the count pointer stable if we push and pop at the same time
      when (io.push_i && io.pop_i &&  ~io.full_o && ~io.empty_o) {
        status_cnt_n   := status_cnt_q
      }
      // FIFO is in pass through mode -> do not change the pointers
      when (FALL_THROUGH.asBool && (status_cnt_q === 0.U) && io.push_i) {
        io.data_o := io.data_i
        when (io.pop_i) {
          status_cnt_n    := status_cnt_q
          read_pointer_n  := read_pointer_q
          write_pointer_n := write_pointer_q
        }
      }
    }

    // sequential process
    when (io.flush_i) {
      read_pointer_q  := 0.U
      write_pointer_q := 0.U
      status_cnt_q    := 0.U
    } .otherwise {
      read_pointer_q  := read_pointer_n
      write_pointer_q := write_pointer_n
      status_cnt_q    := status_cnt_n
    }
  }

  mem_q := mem_n

}
