package ooo

import chisel3._
import chisel3.util._
import chisel3.Bool
import chisel3.experimental.{withClock, withReset, withClockAndReset}

//
// 32-bit Control Bus
//
class ExtBus [Conf <: RVConfig](conf: Conf) extends Bundle {
  override def cloneType: this.type =
    new ExtBus(conf).asInstanceOf[this.type]

  val req    = Output(Bool())
  val addr   = Output(UInt(conf.bus_width.W))
  val wrdata = Output(SInt(32.W))
  val rddata = Input(SInt(32.W))
}
