package first_module

import chisel3._
import chisel3.util._
import chisel3.iotesters.{ChiselFlatSpec, Driver, PeekPokeTester}

class TestFirstModule0(c: Passthrough) extends PeekPokeTester(c) {
  poke(c.io.in, 0)     // Set our input to value 0
  expect(c.io.out, 0)  // Assert that the output correctly has 0
  poke(c.io.in, 1)     // Set our input to value 1
  expect(c.io.out, 1)  // Assert that the output correctly has 1
  poke(c.io.in, 2)     // Set our input to value 2
  expect(c.io.out, 2)  // Assert that the output correctly has 2
}


class TestFirstModule1(c: PassthroughGenerator) extends PeekPokeTester(c) {
  poke(c.io.in, 0)
  expect(c.io.out, 0)
  poke(c.io.in, 1023)
  expect(c.io.out, 1023)
}


class TestFirstModule2(c: PassthroughGenerator) extends PeekPokeTester(c) {
  poke(c.io.in, 0)
  expect(c.io.out, 0)
  poke(c.io.in, 1048575)
  expect(c.io.out, 1048575)
}


class FirstModuleTester extends ChiselFlatSpec {

  "Basic test1 using Driver.execute" should "be used as an alternative way to run specification" in {
    iotesters.Driver.execute(Array(), () => new Passthrough) {
      c => new TestFirstModule0(c)
    } should be (true)
  }

  "Basic test2 using Driver.execute" should "be used as an alternative way to run specification" in {
    iotesters.Driver.execute(Array(), () => new PassthroughGenerator(10)) {
      c1 => new TestFirstModule1(c1)
    } should be (true)
  }

  "Basic test3 using Driver.execute" should "be used as an alternative way to run specification" in {
    iotesters.Driver.execute(Array(), () => new PassthroughGenerator(20)) {
      c2 => new TestFirstModule2(c2)
    } should be (true)
  }

  println("SUCCESS!!") // Scala Code: if we get here, our tests passed!
}
