package cpu

import chisel3.iotesters
import chisel3.iotesters.{ChiselFlatSpec, Driver, PeekPokeTester}
import scala.io.Source
import java.io._
import util.control.Breaks._

import BusConsts._

class CpuTopTests [Conf <: RVConfig](c: CpuTop[Conf], hexname: String, pipename: String) extends PeekPokeTester(c)
{
  val fp = Source.fromFile(hexname)
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
  val writer = new PrintWriter(new File(pipename))

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

  breakable {
    for (cycle <- 0 to 4096) {
      val inst_valid = peek(cpu_tb.io.dbg_monitor.inst_valid)

      writer.printf("%10d : ".format(cycle))

      val inst_fetch_req           = peek(cpu_tb.io.dbg_monitor.inst_fetch_req)
      val inst_fetch_addr : Long   = peek(cpu_tb.io.dbg_monitor.inst_fetch_addr).toLong
      val inst_fetch_ack           = peek(cpu_tb.io.dbg_monitor.inst_fetch_ack)
      val inst_fetch_rddata        = peek(cpu_tb.io.dbg_monitor.inst_fetch_rddata)

      if (inst_fetch_req == 1) { writer.printf("[%08x]".format(inst_fetch_addr)) }
      else                     { writer.printf("          ") }
      if (inst_fetch_ack == 1) { writer.printf("  %08x ".format(inst_fetch_rddata)) }
      else                     { writer.printf("           ") }
      writer.printf("|")

      val reg_wren   = peek(cpu_tb.io.dbg_monitor.reg_wren)
      val reg_wraddr : Long = peek(cpu_tb.io.dbg_monitor.reg_wraddr).toLong
      val reg_wrdata : Long = peek(cpu_tb.io.dbg_monitor.reg_wrdata).toLong

      val alu_rdata0 : Long = peek (cpu_tb.io.dbg_monitor.alu_rdata0).toLong
      val alu_rdata1 : Long = peek (cpu_tb.io.dbg_monitor.alu_rdata1).toLong
      val alu_func          = peek (cpu_tb.io.dbg_monitor.alu_func)

      if (alu_func != 0) {
        writer.printf("%2d, %016x, %016x".format(alu_func, alu_rdata0, alu_rdata1))
      } else {
        writer.printf("                                      ")
      }
      writer.printf("|")

      if (reg_wren == 1) {
        writer.printf("x%02d<=0x%016x ".format(reg_wraddr, reg_wrdata))
      } else {
        writer.printf("                        ")
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

      if (inst_valid == 1) {
        val inst_addr = peek(cpu_tb.io.dbg_monitor.inst_addr)
        val inst_hex  = peek(cpu_tb.io.dbg_monitor.inst_hex)
        writer.printf(" : 0x%08x : INST(0x%08x) : DASM(%08x)\n"format(inst_addr, inst_hex, inst_hex))
      } else {
        writer.printf("\n")
      }

  	  if (data_bus_req == 1 && data_bus_cmd == peek(CMD_WR) &&
  	    data_bus_addr == 0x1000) {
        if (data_bus_wrdata == 0x1) {
  	      writer.printf(" PASS : Simulation Finished\n")
  	    } else {
  	      writer.printf(" FAIL : Simulation Finished\n")
  	    }
  	    break
  	  }
      step(1)
    }
  }

  writer.close()
}

class Tester extends ChiselFlatSpec {
  "Basic test using Driver.execute" should "be used as an alternative way to run specification" in {
    iotesters.Driver.execute(Array(), () => new CpuTop(new RV64IConfig)) {
      c => new CpuTopTests(c, "test.hex", "pipetrace.log")
    } should be (true)
  }
}
