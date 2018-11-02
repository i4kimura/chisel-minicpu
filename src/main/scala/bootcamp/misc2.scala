package misc

import chisel3._
import chisel3.util._

class PopCount2 extends Module {
  // Example circuit using Reverse
  val io = IO(new Bundle {
    val in = Input(UInt(8.W))
    val out = Output(UInt(8.W))
  })
  io.out := PopCount(io.in)
}


class Reverse2 extends Module {
  // Example circuit using Reverse
  val io = IO(new Bundle {
    val in = Input(UInt(8.W))
    val out = Output(UInt(8.W))
  })
  io.out := Reverse(io.in)
}


class OneHot2 extends Module {
  // Example circuit using UIntToOH
  val io = IO(new Bundle {
    val in = Input(UInt(4.W))
    val out = Output(UInt(16.W))
  })
  io.out := UIntToOH(io.in)
}


class OHToUInt2 extends Module {
  // Example circuit using OHToUInt
  val io = IO(new Bundle {
    val in = Input(UInt(16.W))
    val out = Output(UInt(4.W))
  })
  io.out := OHToUInt(io.in)
}


class Mux2 extends Module {
  // Example circuit using PriorityMux
  val io = IO(new Bundle {
    val in_sels = Input(Vec(2, Bool()))
    val in_bits = Input(Vec(2, UInt(8.W)))
    val out = Output(UInt(8.W))
  })
  io.out := PriorityMux(io.in_sels, io.in_bits)
}


class Mux1H_2 extends Module {
  // Example circuit using Mux1H
  val io = IO(new Bundle {
    val in_sels = Input(Vec(2, Bool()))
    val in_bits = Input(Vec(2, UInt(8.W)))
    val out = Output(UInt(8.W))
  })
  io.out := Mux1H(io.in_sels, io.in_bits)
}


class Counter2 extends Module {
  // Example circuit using Mux1H
  val io = IO(new Bundle {
    val count = Input(Bool())
    val out = Output(UInt(2.W))
  })
  val counter = Counter(3)  // 3-count Counter (outputs range [0...2])
    when(io.count) {
      counter.inc()
    }
  io.out := counter.value
}
