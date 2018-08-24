package cpu

import chisel3._

class Memory extends Module {
  val io = IO(new Bundle {
    val i_wen    = Input(Bool())
    val i_wrAddr = Input(UInt(8.W))
    val i_wrData = Input(UInt(32.W))

    val i_ren    = Input(Bool())
    val i_rdAddr = Input(UInt(8.W))
    val o_rdData = Output(UInt(32.W))
    val o_rdEn   = Output(Bool())

    // External Memory Input
    val i_extWen  = Input(Bool())
    val i_extAddr = Input(UInt(8.W))
    val i_extData = Input(UInt(32.W))
  })

  val memory = Mem(256, UInt(32.W))
  val r_en   = Reg(Bool())

  when (io.i_extWen) {
    memory(io.i_extAddr) := io.i_extData
    printf(s"<Info : Address %x : Write %x>\n", io.i_extAddr, io.i_extData)
  } .otherwise {
    when (io.i_wen) {
      memory(io.i_wrAddr) := io.i_wrData
    }
  }

  r_en := io.i_ren

  io.o_rdData := memory(io.i_rdAddr)
  io.o_rdEn  := r_en

  // External Data Input

}
