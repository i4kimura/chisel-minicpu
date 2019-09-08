// Description: SRAM Behavioral Model

package ariane

import chisel3._
import chisel3.util._
import chisel3.Bool

class sram(DATA_WIDTH: Int = 64, NUM_WORDS: Int = 1024) extends Module {
  val io = IO(new Bundle {
   val req_i   = Input(Bool())
   val we_i    = Input(Bool())
   val addr_i  = Input(UInt(log2Ceil(NUM_WORDS).W))
   val wdata_i = Input(UInt(DATA_WIDTH.W))
   val be_i    = Input(UInt(DATA_WIDTH.W))
   val rdata_o = Output(UInt(DATA_WIDTH.W))
  })

  val ADDR_WIDTH:Int = log2Ceil(NUM_WORDS);

  val ram = Mem(NUM_WORDS, UInt(DATA_WIDTH.W))
  val raddr_q = Reg(UInt(ADDR_WIDTH.W))

  // 1. randomize array
  // 2. randomize output when no request is active
  when (io.req_i) {
    when (!io.we_i) {
      raddr_q := io.addr_i
    } .otherwise {
      ram(io.addr_i) := (ram(io.addr_i) & (~io.be_i)) | (io.wdata_i & io.be_i)
    }
  }

  io.rdata_o := ram(raddr_q).asUInt

}


object sram extends App {
  chisel3.Driver.execute(args, () => new sram())
}
