package ariane

import chisel3._
import chisel3.util._
import chisel3.Bool
import ariane_pkg._

object ariane_if_id {
  // ---------------
  // IF/ID Stage
  // ---------------
  // store the decompressed instruction
  class fetch_entry_t extends Bundle {
    val address = UInt(64.W)                       // the address of the instructions from below
    val instruction = UInt(32.W)                   // instruction word
    val branch_predict = new branchpredict_sbe_t() // this field contains branch prediction information regarding the forward branch path
    val ex = new exception_t()                     // this field contains exceptions which might have happened earlier, e.g.: fetch exceptions
  }

  val REG_ADDR_SIZE:Int = 6
  val NR_WB_PORTS:Int   = 4
}
