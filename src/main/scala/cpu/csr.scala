package cpu

import chisel3._
import chisel3.util._
import collection.mutable.LinkedHashMap
import chisel3.Bool


class CsrFileIo extends Bundle {
  val rw = new Bundle {
    val cmd   = Input(UInt(2.W))
    val addr  = Input(UInt(12.W))
    val rdata = Output(UInt(64.W))
    val wdata = Input(UInt(64.W))
  }
  val ecall_inst = Input(Bool())
  val mepc  = Output(UInt(64.W))
  val mtvec = Output(UInt(64.W))
}


object CSR
{
  val X     = 0.U(3.W)
  val Exch  = 1.U(3.W)
  val Set   = 2.U(3.W)
  val Clear = 3.U(3.W)
  val Inst  = 4.U(3.W)
  val Mret  = 5.U(3.W)
}


object PRV
{
  val U = 0.U(2.W)
  val S = 1.U(2.W)
  val H = 2.U(2.W)
  val M = 3.U(2.W)
}


class MStatus extends Bundle {
    // not truly part of mstatus, but convenient
  val debug = Bool()
  val prv = UInt(2.W) // not truly part of mstatus, but convenient
  val sd = Bool()
  val zero1 = UInt(8.W)
  val tsr = Bool()
  val tw = Bool()
  val tvm = Bool()
  val mxr = Bool()
  val sum = Bool()
  val mprv = Bool()
  val xs = UInt(2.W)
  val fs = UInt(2.W)
  val mpp = UInt(2.W)
  val hpp = UInt(2.W)
  val spp = UInt(1.W)
  val mpie = Bool()
  val hpie = Bool()
  val spie = Bool()
  val upie = Bool()
  val mie = Bool()
  val hie = Bool()
  val sie = Bool()
  val uie = Bool()
}


class MIP extends Bundle {
  val zero2 = Bool()
  val debug = Bool() // keep in sync with CSR.debugIntCause
  val zero1 = Bool()
  val rocc = Bool()
  val meip = Bool()
  val heip = Bool()
  val seip = Bool()
  val ueip = Bool()
  val mtip = Bool()
  val htip = Bool()
  val stip = Bool()
  val utip = Bool()
  val msip = Bool()
  val hsip = Bool()
  val ssip = Bool()
  val usip = Bool()
}


class CsrFile extends Module
{
  val io = IO(new CsrFileIo)

  val reset_mstatus = WireInit(0.U.asTypeOf(new MStatus()))
  reset_mstatus.mpp := PRV.M
  reset_mstatus.prv := PRV.M
  val reg_mstatus = RegInit(reset_mstatus)
  val reg_mepc = Reg(UInt(64.W))
  val reg_mcause = Reg(UInt(64.W))
  val reg_mtval = Reg(UInt(64.W))
  val reg_mscratch = Reg(UInt(64.W))
  val reg_mtimecmp = Reg(UInt(64.W))
  val reg_medeleg = Reg(UInt(64.W))

  val reg_mip = RegInit(0.U.asTypeOf(new MIP()))
  val reg_mie = RegInit(0.U.asTypeOf(new MIP()))
  val reg_wfi = RegInit(false.B)
  val reg_mtvec = Reg(UInt(64.W))

  // val reg_time = WideCounter(64)
  // val reg_instret = WideCounter(64, io.retire)

  val reg_mcounteren = Reg(UInt(32.W))
  //val reg_hpmevent = io.counters.map(c => Reg(init = 0.asUInt(64.W)))
  //(io.counters zip reg_hpmevent) foreach { case (c, e) => c.eventSel := e }
  // val reg_hpmcounter = io.counters.map(c => WideCounter(CSR.hpmWidth, c.inc, reset = false))

  val new_prv = WireInit(reg_mstatus.prv)
  reg_mstatus.prv := new_prv

  val reg_debug = RegInit(false.B)
  val reg_dpc = Reg(UInt(64.W))
  val reg_dscratch = Reg(UInt(64.W))
  val reg_singleStepped = Reg(Bool())
  // val reset_dcsr = WireInit(0.U.asTypeOf(new DCSR()))
  // reset_dcsr.xdebugver := 1
  // reset_dcsr.prv := PRV.M
  // val reg_dcsr = RegInit(reset_dcsr)

  val system_insn = io.rw.cmd === CSR.Inst
  val cpu_ren = io.rw.cmd =/= CSR.X && !system_insn

  /* val read_mstatus = io.status.asUInt() */
  val read_mstatus = 0.U
  val isa_string = "I"
  val misa = BigInt(0) | isa_string.map(x => 1 << (x - 'A')).reduce(_|_)
  val impid = 0x8000 // indicates an anonymous source, which can be used
                     // during development before a Source ID is allocated.

  val read_mapping = collection.mutable.LinkedHashMap[Int,Bits](
    /* CsrAddr.mcycle    -> reg_time, */
    /* CsrAddr.minstret  -> reg_instret, */
    CsrAddr.mimpid    -> 0.U,
    CsrAddr.marchid   -> 0.U,
    CsrAddr.mvendorid -> 0.U,
    CsrAddr.misa      -> misa.U,
    CsrAddr.mimpid    -> impid.U,
    CsrAddr.mstatus   -> read_mstatus,
    CsrAddr.mtvec     -> reg_mtvec.asUInt(),
    CsrAddr.mip       -> reg_mip.asUInt(),
    CsrAddr.mie       -> reg_mie.asUInt(),
    CsrAddr.mscratch  -> reg_mscratch,
    CsrAddr.mepc      -> reg_mepc,
    CsrAddr.mtval     -> reg_mtval,
    CsrAddr.mcause    -> reg_mcause,
    /* CsrAddr.mhartid   -> io.hartid, */
    /* CsrAddr.dcsr      -> reg_dcsr.asUInt, */
    CsrAddr.dpc       -> reg_dpc,
    CsrAddr.dscratch  -> reg_dscratch,
    CsrAddr.medeleg   -> reg_medeleg)

  /*
   for (i <- 0 until CSR.nCtr)
   {
   read_mapping += (i + CSR.firstMHPC)  -> reg_hpmcounter(i)
   read_mapping += (i + CSR.firstMHPCH) -> reg_hpmcounter(i)
   }
   */

/*  for (((e, c), i) <- (reg_hpmevent.padTo(CSR.nHPM, 0.U)
                       zip reg_hpmcounter.map(x => x: UInt).padTo(CSR.nHPM, 0.U)) zipWithIndex) {
    read_mapping += (i + CSR.firstHPE) -> e // mhpmeventN
    read_mapping += (i + CSR.firstMHPC) -> c // mhpmcounterN
    if (conf.usingUser) read_mapping += (i + CSR.firstHPC) -> c // hpmcounterN
    if (conf.xprlen == 32) {
      read_mapping += (i + CSR.firstMHPCH) -> c // mhpmcounterNh
      if (conf.usingUser) read_mapping += (i + CSR.firstHPCH) -> c // hpmcounterNh
    }
  }
 */
  /*
  if (conf.usingUser) {
    read_mapping += CsrAddr.mcounteren -> reg_mcounteren
    read_mapping += CsrAddr.cycle -> reg_time
    read_mapping += CsrAddr.instret -> reg_instret
  }
   */
  /*
  if (conf.xprlen == 32) {
    read_mapping += CsrAddr.mcycleh -> 0.U //(reg_time >> 32)
    read_mapping += CsrAddr.minstreth -> 0.U //(reg_instret >> 32)
    if (conf.usingUser) {
      read_mapping += CsrAddr.cycleh -> 0.U //(reg_time >> 32)
      read_mapping += CsrAddr.instreth -> 0.U //(reg_instret >> 32)
    }
  }
   */

  val decoded_addr = read_mapping map { case (k, v) => k -> (io.rw.addr === k.U(12.W)) }

  val priv_sufficient = reg_mstatus.prv >= io.rw.addr(9,8)
  val read_only = io.rw.addr(11,10).andR
  val cpu_wen = cpu_ren && io.rw.cmd =/= CSR.X && priv_sufficient
  val wen = cpu_wen && !read_only
  val wdata = readModifyWriteCSR(io.rw.cmd, io.rw.rdata, io.rw.wdata)

  val opcode = 1.U << io.rw.addr(2,0)
  val insn_call = system_insn && opcode(0)
  val insn_break = system_insn && opcode(1)
  val insn_ret = system_insn && opcode(2) && priv_sufficient
  val insn_wfi = system_insn && opcode(5) && priv_sufficient

  when(io.ecall_inst) {
    reg_mcause := Causes.UserEcall.U
  } .elsewhen (decoded_addr(CsrAddr.mcause)) {
  	reg_mcause := wdata & ((BigInt(1) << (63)) + 31).U /* only implement 5 LSBs and MSB */
  }

  io.mepc  := reg_mepc
  io.mtvec := reg_mtvec

  when (decoded_addr(CsrAddr.dpc))      { reg_dpc := wdata }
  when (decoded_addr(CsrAddr.dscratch)) { reg_dscratch := wdata }

  when (decoded_addr(CsrAddr.mepc))     { reg_mepc := (wdata(63,0) >> 2.U) << 2.U }
  when (decoded_addr(CsrAddr.mscratch)) { reg_mscratch := wdata }
  when (decoded_addr(CsrAddr.mtval))    { reg_mtval := wdata(63,0) }
  when (decoded_addr(CsrAddr.mtvec))    {
    reg_mtvec := wdata(63,0)
  }
  when (decoded_addr(CsrAddr.medeleg))  { reg_medeleg := wdata(63,0) }

  private def decodeAny(m: LinkedHashMap[Int,Bits]): Bool = m.map { case(k: Int, _: Bits) => io.rw.addr === k.U(12.W) }.reduce(_||_)
  io.rw.rdata := Mux1H(for ((k, v) <- read_mapping) yield decoded_addr(k) -> v)

  def readModifyWriteCSR(cmd: UInt, rdata: UInt, wdata: UInt) =
    (Mux((cmd === CSR.Set |  cmd === CSR.Clear), rdata, 0.U) | wdata) & ~Mux(cmd === CSR.Clear, wdata, 0.U)
}
