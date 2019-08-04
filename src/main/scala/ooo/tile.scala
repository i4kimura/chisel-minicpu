package ooo

import chisel3._
import chisel3.util._
import chisel3.Bool

class MemoryIo [Conf <: RVConfig](conf: Conf) extends Bundle {
  override def cloneType: this.type =
    new MemoryIo(conf).asInstanceOf[this.type]

  val l1i_rd_req  = Flipped(Decoupled(new FrontEndReqIo(conf)))
  val l1i_rd_resp = Decoupled(new FrontEndRespIo(conf))

  val l1d_rd_req  = Flipped(Decoupled(new LsuRdReqIo(conf)))
  val l1d_rd_resp = Decoupled(new LsuRdRespIo(conf))
  val l1d_wr_req  = Flipped(Decoupled(new LsuWrReqIo (conf)))
}

class OooTile [Conf <: RVConfig](conf: Conf) extends Module {
  val io = IO(Flipped(new MemoryIo(conf)))

  val ooo_core = Module (new OooCore(conf))

  ooo_core.io.front_req <> io.l1i_rd_req
  ooo_core.io.front_resp <> io.l1i_rd_resp

  ooo_core.io.lsu_rd_req  <> io.l1d_rd_req
  ooo_core.io.lsu_rd_resp <> io.l1d_rd_resp
  ooo_core.io.lsu_wr_req  <> io.l1d_wr_req

  // val l1icache = Module (new L1ICache(conf))
  // val l1dcache = Module (new L1DCache(conf))
  //
  // l1icache.io.cpu_rd_req <> ooo_core.io.front_req
  // ooo_core.io.front_resp <> l1icache.io.cpu_rd_resp
  //
  // l1dcache.io.cpu_rd_req  <> ooo_core.io.lsu_rd_req
  // l1dcache.io.cpu_rd_resp <> ooo_core.io.lsu_rd_resp
  // l1dcache.io.cpu_wr_req  <> ooo_core.io.lsu_wr_req
  //
  // // ooo_core.io.lsu_rd_req  <> l1dcache.io.cpu_rd_req
  // // ooo_core.io.lsu_rd_resp <> l1dcache.io.cpu_rd_resp
  //
  // l1icache.io.ext_rd_req  <> io.l1i_rd_req
  // l1icache.io.ext_rd_resp <> io.l1i_rd_resp
  // l1icache.io.ext_wr_req.ready := false.B
  //
  // l1icache.io.cpu_wr_req.bits.addr := 0.U
  // l1icache.io.cpu_wr_req.valid     := false.B
  //
  // l1dcache.io.ext_rd_req  <> io.l1d_rd_req
  // l1dcache.io.ext_rd_resp <> io.l1d_rd_resp
  // io.l1d_wr_req <> l1dcache.io.ext_wr_req

}

object OooTile extends App {
  chisel3.Driver.execute(args, () => new OooTile(new RV64IConfig))
}
