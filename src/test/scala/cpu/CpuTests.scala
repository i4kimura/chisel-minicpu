package cpu

import chisel3.iotesters
import chisel3.iotesters.{ChiselFlatSpec, Driver, PeekPokeTester}


class CpuTopTests(c: CpuTop) extends PeekPokeTester(c) {

  var mem_init = Array (
    0x041b0010,
    0x141301f4,
    0x2573f140,
    0x05970000,
    0x85930745,
    0x84020000,
    0x00000000,
    0x00000000,
    0x00000000,
    0x00000000,
    0x00000000,
    0x00000000,
    0x00000000,
    0x00000000,
    0x00000000,
    0x00000000,
    0x2573f140,
    0x05970000,
    0x859303c5,
    0x00731050,
    0xbff50000,
    0x00000000,
    0x00000000,
    0x00000000,
    0x00000000,
    0x00000000,
    0x00000000,
    0x00000000
  )

  private val  cpu_tb = c

  poke (cpu_tb.io.i_run, 0)

  for (addr <- 0 to mem_init.length-1) {
    poke (cpu_tb.io.i_memReq, 1)
    poke (cpu_tb.io.i_memAddr, addr)
    poke (cpu_tb.io.i_memData, mem_init(addr) & 0x0000FFFFFFFFL)

    step(1)
  }

  poke (cpu_tb.io.i_memReq , 0)
  poke (cpu_tb.io.i_memAddr, 0)
  poke (cpu_tb.io.i_memData, 0)

  step(1)
  step(1)

  poke (cpu_tb.io.i_run, 1)
  step(1)
  expect(cpu_tb.io.o_DebugInstReq,  1)
  expect(cpu_tb.io.o_DebugInstAddr, 0)

  step(1)
  for (step_idx <- 0 to 10 by 1) {
    val hexwidth = 8

    expect(cpu_tb.io.o_DebugInstReq,  1)
    expect(cpu_tb.io.o_DebugInstAddr, step_idx * 4)

    printf(s"<Info: Step %02d Instruction %0${hexwidth}x is fetched>\n", step_idx, peek(cpu_tb.io.o_DebugInstData))

    step(1)
  }
}

class CpuTopTester extends ChiselFlatSpec {
  private val backendNames = if(firrtl.FileUtils.isCommandAvailable("verilator")) {
    Array("firrtl", "verilator")
  }
  else {
    Array("firrtl")
  }
  for ( backendName <- backendNames ) {
    "CPU" should s"calculate CPU core (with $backendName)" in {
      Driver(() => new CpuTop, backendName) {
        c => new CpuTopTests(c)
      } should be (true)
    }
  }

  "Basic test using Driver.execute" should "be used as an alternative way to run specification" in {
    iotesters.Driver.execute(Array(), () => new CpuTop) {
      c => new CpuTopTests(c)
    } should be (true)
  }
}
