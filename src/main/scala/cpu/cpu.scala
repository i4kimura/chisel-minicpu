package cpu

import chisel3._
import chisel3.Bool


class CpuTop extends Module {
  val io = IO (new Bundle {
    val i_run     = Input(Bool())

    val i_memReq  = Input(Bool())
    val i_memAddr = Input(UInt(8.W))
    val i_memData = Input(UInt(32.W))

    val o_DebugInstReq  = Output(Bool())
    val o_DebugInstAddr = Output(UInt(8.W))
    val o_DebugInstData = Output(UInt(32.W))
  })

  val memory = Module(new Memory)
  val cpu    = Module(new Cpu)

  val w_instReq  = cpu.io.o_instReq
  val w_instAddr = cpu.io.o_instAddr

  // Connect CPU and Memory
  memory.io.i_ren    := w_instReq
  memory.io.i_rdAddr := w_instAddr(7, 2)

  cpu.io.i_instAck   := memory.io.o_rdEn
  cpu.io.i_instData  := memory.io.o_rdData

  cpu.io.i_run       := io.i_run

  // Memory Load for External Debug
  memory.io.i_extWen  := io.i_memReq
  memory.io.i_extAddr := io.i_memAddr
  memory.io.i_extData := io.i_memData

  io.o_DebugInstReq  := w_instReq
  io.o_DebugInstAddr := w_instAddr
  io.o_DebugInstData := memory.io.o_rdData
}


class Cpu extends Module {
  val io = IO (new Bundle {
    val i_run      = Input(Bool())

    val o_instAddr = Output(UInt(8.W))
    val o_instReq  = Output(Bool())

    val i_instAck  = Input(Bool())
    val i_instData = Input(UInt(32.W))
  })

  val r_inst_addr = RegInit(0.U(8.W))
  val r_inst_en   = RegInit(false.B)

  r_inst_en := io.i_run

  when(r_inst_en & io.i_instAck) {
    r_inst_addr := r_inst_addr + 4.U
  }

  io.o_instAddr := r_inst_addr
  io.o_instReq  := r_inst_en
}
