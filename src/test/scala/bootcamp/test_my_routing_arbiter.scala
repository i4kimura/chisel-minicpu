package my_routing_arbiter

import chisel3._
import chisel3.util._
import chisel3.iotesters.{ChiselFlatSpec, Driver, PeekPokeTester}

class TestMyRoutingArbiter (c: MyRoutingArbiter) extends PeekPokeTester(c) {
  // Set input defaults
  for(i <- 0 until 4) {
    poke(c.io.in(i).valid, 0)
    poke(c.io.in(i).bits, i)
    poke(c.io.out.ready, 1)
  }

  expect(c.io.out.valid, 0)

  // Check single input valid behavior with backpressure
  for (i <- 0 until 4) {
    poke(c.io.in(i).valid, 1)
    expect(c.io.out.valid, 1)
    expect(c.io.out.bits, i)

    poke(c.io.out.ready, 0)
    expect(c.io.in(i).ready, 0)

    poke(c.io.out.ready, 1)
    poke(c.io.in(i).valid, 0)
  }

  // Basic check of multiple input ready behavior with backpressure
  poke(c.io.in(1).valid, 1)
  poke(c.io.in(2).valid, 1)
  expect(c.io.out.bits, 1)
  expect(c.io.in(1).ready, 1)
  expect(c.io.in(0).ready, 0)

  poke(c.io.out.ready, 0)
  expect(c.io.in(1).ready, 0)
}


class Tester extends ChiselFlatSpec {

  "Basic test1 using Driver.execute" should "be used as an alternative way to run specification" in {
    iotesters.Driver.execute(Array(), () => new MyRoutingArbiter(4)) {
      c => new TestMyRoutingArbiter(c)
    } should be (true)
  }
}
