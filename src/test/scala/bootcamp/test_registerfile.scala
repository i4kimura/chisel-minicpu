package registerfile

import chisel3._
import chisel3.util._
import chisel3.iotesters.{ChiselFlatSpec, Driver, PeekPokeTester}

class Tester extends ChiselFlatSpec {

  "Basic test1 using Driver.execute" should "be used as an alternative way to run specification" in {
    chisel3.iotesters.Driver(() => new RegisterFile(2) ) { c => new PeekPokeTester(c) {
      def readExpect(addr: Int, value: Int, port: Int = 0): Unit = {
        poke(c.io.raddr(port), addr)
        expect(c.io.rdata(port), value)
      }
      def write(addr: Int, value: Int): Unit = {
        poke(c.io.wen, 1)
        poke(c.io.wdata, value)
        poke(c.io.waddr, addr)
        step(1)
        poke(c.io.wen, 0)
      }
      // everything should be 0 on init
      for (i <- 0 until 32) {
        readExpect(i, 0, port = 0)
        readExpect(i, 0, port = 1)
      }

      // write 5 * addr + 3
      for (i <- 0 until 32) {
        write(i, 5 * i + 3)
      }

      // check that the writes worked
      for (i <- 0 until 32) {
        readExpect(i, if (i == 0) 0 else 5 * i + 3, port = i % 2)
      }
    }} should be (true)
  }

  println("SUCCESS!!") // Scala Code: if we get here, our tests passed!
}
