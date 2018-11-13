package cpu

import chisel3.iotesters
import chisel3.iotesters.{ChiselFlatSpec, Driver, PeekPokeTester}
import scala.io.Source

class CpuTopTests(c: CpuTop) extends PeekPokeTester(c)
{

  val fp    = Source.fromFile("test.hex")
  val lines = fp.getLines

  val memory = lines.map{ line =>
    val split_line = line.split(" ")
    if (split_line.length == 2) {
      Array(Integer.parseInt(line.split(" ")(0).diff("@"), 16),
        Integer.parseUnsignedInt(line.split(" ")(1), 16))
    } else {
      Array(Integer.parseInt(line.split(" ")(0).diff("@"), 16), 0)
    }
  }

  private val  cpu_tb = c

  poke (cpu_tb.io.run, 0)

  memory.foreach{ mem =>
    poke (cpu_tb.io.ext_bus.req, 1)
    poke (cpu_tb.io.ext_bus.addr, mem(0))
    poke (cpu_tb.io.ext_bus.data, mem(1))

    step(1)
  }

  poke (cpu_tb.io.ext_bus.req , 0)
  poke (cpu_tb.io.ext_bus.addr, 0)
  poke (cpu_tb.io.ext_bus.data, 0)

  step(1)
  step(1)

  poke (cpu_tb.io.run, 1)
  step(1)
  expect(cpu_tb.io.debugpath.req,  1)
  expect(cpu_tb.io.debugpath.addr, 0)

  step(100)
  // for (step_idx <- 0 to 10 by 1) {
  //   val hexbus_width = 8
  //
  //   expect(cpu_tb.io.debugpath.req,  1)
  //   expect(cpu_tb.io.debugpath.addr, step_idx * 4)
  //
  //   printf(s"<Info: Step %02d Instruction %0${hexbus_width}x is fetched>\n", step_idx, peek(cpu_tb.io.debugpath.data))
  //
  //   step(1)
  // }
}

class Tester extends ChiselFlatSpec {
  private val backendNames = if(firrtl.FileUtils.isCommandAvailable("verilator")) {
    Array("firrtl", "verilator")
  }
  else {
    Array("firrtl")
  }
  for ( backendName <- backendNames ) {
    "CPU" should s"calculate CPU core (with $backendName)" in {
      Driver(() => new CpuTop(bus_width = 16), backendName) {
        c => new CpuTopTests(c)
      } should be (true)
    }
  }

  "Basic test using Driver.execute" should "be used as an alternative way to run specification" in {
    iotesters.Driver.execute(Array(), () => new CpuTop(bus_width = 16)) {
      c => new CpuTopTests(c)
    } should be (true)
  }

}
