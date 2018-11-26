package cpu

import chisel3.iotesters
import chisel3.iotesters.{ChiselFlatSpec, Driver, PeekPokeTester}
import scala.io.Source
import java.io._

import BusConsts._

// object PrintHex
// {
//   def apply(x: UInt, length: Int, fp: PrintWriter) =
//   {
//     require(length > 0)
//     for (i <- length-1 to 0 by -1) {
//       printf("%x", ((x >> (i * 4)) & 0x0f.U))
//     }
//     0
//   }
//
//   def apply(x: SInt, length: Int, fp: PrintWriter) =
//   {
//     require(length > 0)
//     for (i <- length-1 to 0 by -1) {
//       printf("%x", ((x >> (i * 4)) & 0x0f.S))
//     }
//     0
//   }
// }


class CpuTopTests(c: CpuTop) extends PeekPokeTester(c)
{
  val fp = Source.fromFile("test.hex")
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

  //
  // Monitor for Debug
  //
  val writer = new PrintWriter(new File("pipetrace.log"))

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

  for (cycle <- 0 to 512) {
    val inst_valid = peek(cpu_tb.io.dbg_monitor.inst_valid)
    if (inst_valid == 1) {
      writer.printf("%10d : ".format(cycle))
      val reg_wren   = peek(cpu_tb.io.dbg_monitor.reg_wren)
      val reg_wraddr : Long = peek(cpu_tb.io.dbg_monitor.reg_wraddr).toLong
      val reg_wrdata : Long = peek(cpu_tb.io.dbg_monitor.reg_wrdata).toLong
      if (reg_wren == 1 && reg_wraddr != 0) {
        writer.printf("x%02d<=0x%016x".format(reg_wraddr, reg_wrdata))
      } else {
        writer.printf("                       ")
      }

      val data_bus_req    = peek(cpu_tb.io.dbg_monitor.data_bus_req)
      val data_bus_cmd    = peek(cpu_tb.io.dbg_monitor.data_bus_cmd)
      val data_bus_addr   = peek(cpu_tb.io.dbg_monitor.data_bus_addr)
      val data_bus_wrdata = peek(cpu_tb.io.dbg_monitor.data_bus_wrdata)
      val data_bus_ack    = peek(cpu_tb.io.dbg_monitor.data_bus_ack)
      val data_bus_rddata = peek(cpu_tb.io.dbg_monitor.data_bus_rddata)

      if (data_bus_req == 1 && data_bus_cmd == peek(CMD_WR)) {
        writer.printf(" [%08x]<=0x%016x".format(data_bus_addr, data_bus_wrdata.toLong))
      } else if (data_bus_req == 1 && data_bus_cmd == peek(CMD_RD)) {
        writer.printf(" [%08x]=>0x%016x".format(data_bus_addr, data_bus_rddata.toLong))
      } else {
        writer.printf("                               ")
      }

      val inst_addr = peek(cpu_tb.io.dbg_monitor.inst_addr)
      val inst_hex  = peek(cpu_tb.io.dbg_monitor.inst_hex)
      writer.printf(" : 0x%08x : INST(0x%08x) : DASM(%08x)\n"format(inst_addr, inst_hex, inst_hex))
    }
    step(1)
  }

  writer.close()
}

class Tester extends ChiselFlatSpec {
  private val backendNames = if(firrtl.FileUtils.isCommandAvailable("verilator")) {
    Array("firrtl", "verilator")
  }
  else {
    Array("firrtl")
  }
  for ( backendName <- backendNames ) {
    implicit val conf = RV64IConf()
    "CPU" should s"calculate CPU core (with $backendName)" in {
      Driver(() => new CpuTop(), backendName) {
        c => new CpuTopTests(c)
      } should be (true)
    }
  }

  "Basic test using Driver.execute" should "be used as an alternative way to run specification" in {
    implicit val conf = RV64IConf()
    iotesters.Driver.execute(Array(), () => new CpuTop()) {
      c => new CpuTopTests(c)
    } should be (true)
  }

}
