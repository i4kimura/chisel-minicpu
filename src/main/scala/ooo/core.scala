package ooo

import chisel3._
import chisel3.util._
import chisel3.Bool

import DecodeConsts._

class FrontEndReqIo [Conf <: RVConfig](conf: Conf) extends Bundle {
  override def cloneType: this.type =
    new FrontEndReqIo(conf).asInstanceOf[this.type]

  val addr = UInt(conf.bus_width.W)
}

class FrontEndRespIo [Conf <: RVConfig](conf: Conf) extends Bundle {
  override def cloneType: this.type =
    new FrontEndRespIo(conf).asInstanceOf[this.type]

  val data = UInt(64.W)
}


class OooCore [Conf <: RVConfig](conf: Conf) extends Module {
  val io = IO (new Bundle {
    val front_req = Decoupled(new FrontEndReqIo(conf))
    val front_resp = Flipped(Decoupled(new FrontEndReqIo(conf)))
  })

  io.front_req.valid := true.B
  io.front_resp.ready := true.B

  val decode_units = for (w <- 0 until conf.fetch_width) yield {
    val u_cpath = Module (new CtlPath);
    u_cpath
  }

  for (w <- 0 until conf.fetch_width) {
    decode_units(w).io.inst := io.front_resp.data(w*32+31, w*32)
  }

}


object OooCore extends App {
  chisel3.Driver.execute(args, () => new OooCore(new RV64IConfig))
}
