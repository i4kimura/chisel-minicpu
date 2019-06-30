package cpu

import chisel3._
import chisel3.util._
import chisel3.Bool

class icache_req_t (val VADDR_WIDTH: Int) extends Bundle
{
  val addr = UInt(VADDR_WIDTH.W)
}
class icache_resp_t (val DATA_WIDTH: Int) extends Bundle
{
  val data = UInt(DATA_WIDTH.W)
}
class icache_if_t (val VADDR_WIDTH: Int, val DATA_WIDTH: Int) extends Bundle {
  val req  = Valid(new icache_req_t(VADDR_WIDTH))
  val resp = Flipped(Valid(new icache_resp_t(DATA_WIDTH)))
}


class outer_req_t (val VADDR_WIDTH: Int) extends Bundle
{
  val addr = UInt(VADDR_WIDTH.W)
}
class outer_resp_t (val DATA_WIDTH: Int) extends Bundle
{
  val data = UInt(DATA_WIDTH.W)
}
class outer_if_t (val VADDR_WIDTH: Int, val DATA_WIDTH: Int) extends Bundle {
  val req  = Valid(new outer_req_t(VADDR_WIDTH))
  val resp = Valid(new outer_resp_t(DATA_WIDTH)).flip
}


class icache(val VADDR_WIDTH:Int, val DATA_WIDTH: Int) extends Module
{
  val io = IO (new Bundle {
    // val ic_req  = Flipped(Valid(new icache_req_t(VADDR_WIDTH)))
    // val ic_resp = Valid(new icache_resp_t(DATA_WIDTH))
    val ic = Flipped(new icache_if_t(VADDR_WIDTH, DATA_WIDTH))
    val outer = new outer_if_t (VADDR_WIDTH, DATA_WIDTH)
  })

  val UNTAG_WIDTH = 10
  val TAG_MSB = VADDR_WIDTH
  val TAG_LSB = UNTAG_WIDTH
  val TAG_WIDTH = TAG_MSB - TAG_LSB

  // val tag_array  = SyncReadMem(Math.pow(2, UNTAG_WIDTH), UInt(TAG_WIDTH.W))
  // val tag_valid  = Reg(Vec(Math.pow(2, UNTAG_WIDTH), Bool()))
  // val data_array = SeqMem(Math.pow(2, UNTAG_WIDTH), UInt(DATA_WIDTH.W))

  val tag_array  = SyncReadMem(1024, UInt(TAG_WIDTH.W))
  val tag_valid  = Reg(Vec(1024, Bool()))
  val data_array = SeqMem(1024, UInt(DATA_WIDTH.W))

  val f0_req_addr_untag = io.ic.req.bits.addr(UNTAG_WIDTH-1, 0)
  val f0_req_addr_tag   = io.ic.req.bits.addr(VADDR_WIDTH-1, UNTAG_WIDTH)
  val f1_req_addr_tag   = RegNext(f0_req_addr_tag)

  when (RegNext(io.ic.req.valid)) {
    when (RegNext(tag_valid(f0_req_addr_tag)) && RegNext(tag_array(f0_req_addr_tag)) === f1_req_addr_tag) {
      io.ic.resp.valid := true.B
    } .otherwise {
      io.ic.resp.valid := false.B
    }
  } .otherwise {
    io.ic.resp.valid := false.B
  }

  io.ic.resp.bits.data := data_array.read(f0_req_addr_tag)

  io.outer.req.valid     := false.B
  io.outer.req.bits.addr := 0.U

}


object icache extends App {
  chisel3.Driver.execute(args, () => new icache(48, 64))
}
