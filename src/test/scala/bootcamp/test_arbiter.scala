package arbiter2

import chisel3._
import chisel3.util._
import chisel3.iotesters.{ChiselFlatSpec, Driver, PeekPokeTester}


class Tester extends ChiselFlatSpec {

  "Basic test1 using Driver.execute" should "be used as an alternative way to run specification" in {
    chisel3.iotesters.Driver(() => new Arbiter2 ) { c => new PeekPokeTester(c) {
      poke(c.io.in(0).valid, 0)
      poke(c.io.in(1).valid, 0)
      println(s"Start:")
      println(s"\tin(0).ready=${peek(c.io.in(0).ready)}, in(1).ready=${peek(c.io.in(1).ready)}")
      println(s"\tout.valid=${peek(c.io.out.valid)}, out.bits=${peek(c.io.out.bits)}")
      poke(c.io.in(1).valid, 1)  // Valid input 1
      poke(c.io.in(1).bits, 42)
      // What do you think the output will be?
      println(s"valid input 1:")
      println(s"\tin(0).ready=${peek(c.io.in(0).ready)}, in(1).ready=${peek(c.io.in(1).ready)}")
      println(s"\tout.valid=${peek(c.io.out.valid)}, out.bits=${peek(c.io.out.bits)}")
      poke(c.io.in(0).valid, 1)  // Valid inputs 0 and 1
      poke(c.io.in(0).bits, 43)
      // What do you think the output will be? Which inputs will be ready?
      println(s"valid inputs 0 and 1:")
      println(s"\tin(0).ready=${peek(c.io.in(0).ready)}, in(1).ready=${peek(c.io.in(1).ready)}")
      println(s"\tout.valid=${peek(c.io.out.valid)}, out.bits=${peek(c.io.out.bits)}")
      poke(c.io.in(1).valid, 0)  // Valid input 0
                                 // What do you think the output will be?
      println(s"valid input 0:")
      println(s"\tin(0).ready=${peek(c.io.in(0).ready)}, in(1).ready=${peek(c.io.in(1).ready)}")
      println(s"\tout.valid=${peek(c.io.out.valid)}, out.bits=${peek(c.io.out.bits)}")
    }} should be (true)
  }

  println("SUCCESS!!") // Scala Code: if we get here, our tests passed!
}
