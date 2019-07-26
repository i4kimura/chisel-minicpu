package ooo

import chisel3._
import chisel3.util._
import chisel3.Bool

class OooTile [Conf <: RVConfig](conf: Conf) extends Module {
  val io = IO (new Bundle {
    val l1i_rd_req = Decoupled(new FrontEndReqIo(conf))
    val l1i_rd_resp = Flipped(Decoupled(new FrontEndRespIo(conf)))

    val l1d_rd_req  = Decoupled(new FrontEndReqIo(conf))
    val l1d_rd_resp = Flipped(Decoupled(new FrontEndRespIo(conf)))
    val l1d_wr_req  = Decoupled(new FrontEndReqIo (conf))
  })

  val ooo_core = Module (new OooCore(conf))

  val l1icache = Module (new L1Cache(conf))
  val l1dcache = Module (new L1Cache(conf))

  l1icache.io.cpu_rd_req <> ooo_core.io.front_req
  ooo_core.io.front_resp <> l1icache.io.cpu_rd_resp

  l1dcache.io.cpu_rd_req.valid     := ooo_core.io.lsu_req.valid
  l1dcache.io.cpu_rd_req.bits.addr := ooo_core.io.lsu_req.bits.addr
  l1dcache.io.cpu_wr_req.valid     := false.B
  l1dcache.io.cpu_wr_req.bits.addr := 0.U
  ooo_core.io.lsu_req.ready        := l1dcache.io.cpu_rd_req.ready

  ooo_core.io.lsu_resp.valid       := l1dcache.io.cpu_rd_resp.valid
  ooo_core.io.lsu_resp.bits.rddata := l1dcache.io.cpu_rd_resp.bits.data.asSInt()
  l1dcache.io.cpu_rd_resp.ready    := ooo_core.io.lsu_resp.ready

  // ooo_core.io.lsu_req  <> l1dcache.io.cpu_rd_req
  // ooo_core.io.lsu_resp <> l1dcache.io.cpu_rd_resp

  l1icache.io.ext_rd_req  <> io.l1i_rd_req
  l1icache.io.ext_rd_resp <> io.l1i_rd_resp
  l1icache.io.ext_wr_req.ready := false.B

  l1icache.io.cpu_wr_req.bits.addr := 0.U
  l1icache.io.cpu_wr_req.valid     := false.B

  l1dcache.io.ext_rd_req  <> io.l1d_rd_req
  l1dcache.io.ext_rd_resp <> io.l1d_rd_resp
  io.l1d_wr_req <> l1dcache.io.ext_wr_req

}

object OooTile extends App {
  chisel3.Driver.execute(args, () => new OooTile(new RV64IConfig))
}
