package cpu

import chisel3._
import chisel3.util._
import chisel3.Bool

import DecodeConsts._

class Bus [Conf <: RVConfig](conf: Conf) extends Bundle {
  val req  = Input(Bool())
  val addr = Input(UInt(conf.bus_width.W))
  val data = Input(SInt(32.W))
}


class InstBus [Conf <: RVConfig](conf: Conf) extends Bundle {
  override def cloneType: this.type =
    new InstBus(conf).asInstanceOf[this.type]

  val req    = Output(Bool())
  val addr   = Output(UInt(conf.xlen.W))

  val ack    = Input(Bool())
  val rddata = Input(SInt(32.W))
}


class DataBus [Conf <: RVConfig](conf: Conf) extends Bundle {
  override def cloneType: this.type =
    new DataBus(conf).asInstanceOf[this.type]

  val req    = Output(Bool())
  val cmd    = Output(UInt(2.W))
  val addr   = Output(UInt(conf.xlen.W))
  val size   = Output(UInt(3.W))
  val wrdata = Output(SInt(64.W))

  val ack    = Input(Bool())
  val rddata = Input(SInt(64.W))
}
