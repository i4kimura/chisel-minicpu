package fir

import chisel3._
import chisel3.util._
import chisel3.iotesters.{ChiselFlatSpec, Driver, PeekPokeTester}

class TestFir0 (c: My4ElementFir) extends PeekPokeTester(c) {
  poke(c.io.in, 0)
  expect(c.io.out, 0)
  step(1)
  poke(c.io.in, 4)
  expect(c.io.out, 0)
  step(1)
  poke(c.io.in, 5)
  expect(c.io.out, 0)
  step(1)
  poke(c.io.in, 2)
  expect(c.io.out, 0)
}


class TestFir1 (c: My4ElementFir) extends PeekPokeTester(c) {
  poke(c.io.in, 1)
  expect(c.io.out, 1)  // 1, 0, 0, 0
  step(1)
  poke(c.io.in, 4)
  expect(c.io.out, 5)  // 4, 1, 0, 0
  step(1)
  poke(c.io.in, 3)
  expect(c.io.out, 8)  // 3, 4, 1, 0
  step(1)
  poke(c.io.in, 2)
  expect(c.io.out, 10)  // 2, 3, 4, 1
  step(1)
  poke(c.io.in, 7)
  expect(c.io.out, 16)  // 7, 2, 3, 4
  step(1)
  poke(c.io.in, 0)
  expect(c.io.out, 12)  // 0, 7, 2, 3
}


class TestFir2 (c: My4ElementFir) extends PeekPokeTester(c) {
  poke(c.io.in, 1)
  expect(c.io.out, 1)  // 1*1, 0*2, 0*3, 0*4
  step(1)
  poke(c.io.in, 4)
  expect(c.io.out, 6)  // 4*1, 1*2, 0*3, 0*4
  step(1)
  poke(c.io.in, 3)
  expect(c.io.out, 14)  // 3*1, 4*2, 1*3, 0*4
  step(1)
  poke(c.io.in, 2)
  expect(c.io.out, 24)  // 2*1, 3*2, 4*3, 1*4
  step(1)
  poke(c.io.in, 7)
  expect(c.io.out, 36)  // 7*1, 2*2, 3*3, 4*4
  step(1)
  poke(c.io.in, 0)
  expect(c.io.out, 32)  // 0*1, 7*2, 2*3, 3*4
}


class TestFirGen4_0 (c: GenericFIR) extends PeekPokeTester(c) {
  poke(c.io.consts(0), 0)
  poke(c.io.consts(1), 0)
  poke(c.io.consts(2), 0)
  poke(c.io.consts(3), 0)

  poke(c.io.in, 0)
  expect(c.io.out, 0)
  step(1)
  poke(c.io.in, 4)
  expect(c.io.out, 0)
  step(1)
  poke(c.io.in, 5)
  expect(c.io.out, 0)
  step(1)
  poke(c.io.in, 2)
  expect(c.io.out, 0)
}


class TestFirGen4_1 (c: GenericFIR) extends PeekPokeTester(c) {
  poke(c.io.consts(0), 1)
  poke(c.io.consts(1), 1)
  poke(c.io.consts(2), 1)
  poke(c.io.consts(3), 1)

  step(1)

  poke(c.io.in, 1)
  step(1)
  expect(c.io.out, 1)  // 1, 0, 0, 0

  poke(c.io.in, 4)
  step(1)
  expect(c.io.out, 5)  // 4, 1, 0, 0

  poke(c.io.in, 3)
  step(1)
  expect(c.io.out, 8)  // 3, 4, 1, 0

  poke(c.io.in, 2)
  step(1)
  expect(c.io.out, 10)  // 2, 3, 4, 1

  poke(c.io.in, 7)
  step(1)
  expect(c.io.out, 16)  // 7, 2, 3, 4

  poke(c.io.in, 0)
  step(1)
  expect(c.io.out, 12)  // 0, 7, 2, 3
}


class TestFirGen4_2 (c: GenericFIR) extends PeekPokeTester(c) {
  poke(c.io.consts(0), 1)
  poke(c.io.consts(1), 2)
  poke(c.io.consts(2), 3)
  poke(c.io.consts(3), 4)

  poke(c.io.in, 1)
  step(1)
  expect(c.io.out, 1)  // 1*1, 0*2, 0*3, 0*4
  poke(c.io.in, 4)
  step(1)
  expect(c.io.out, 6)  // 4*1, 1*2, 0*3, 0*4
  poke(c.io.in, 3)
  step(1)
  expect(c.io.out, 14)  // 3*1, 4*2, 1*3, 0*4
  poke(c.io.in, 2)
  step(1)
  expect(c.io.out, 24)  // 2*1, 3*2, 4*3, 1*4
  poke(c.io.in, 7)
  step(1)
  expect(c.io.out, 36)  // 7*1, 2*2, 3*3, 4*4
  poke(c.io.in, 0)
  step(1)
  expect(c.io.out, 32)  // 0*1, 7*2, 2*3, 3*4
}


class Tester extends ChiselFlatSpec {

  "Basic test1 using Driver.execute" should "be used as an alternative way to run specification" in {
    iotesters.Driver.execute(Array(), () => new My4ElementFir(0, 0, 0, 0)) {
      c => new TestFir0(c)
    } should be (true)
  }

  "Basic test2 using Driver.execute" should "be used as an alternative way to run specification" in {
    iotesters.Driver.execute(Array(), () => new My4ElementFir(1, 1, 1, 1)) {
      c1 => new TestFir1(c1)
    } should be (true)
  }

  "Basic test3 using Driver.execute" should "be used as an alternative way to run specification" in {
    iotesters.Driver.execute(Array(), () => new My4ElementFir(1, 2, 3, 4)) {
      c2 => new TestFir2(c2)
    } should be (true)
  }


  "Basic gen_test1 using Driver.execute" should "be used as an alternative way to run specification" in {
    iotesters.Driver.execute(Array(), () => new GenericFIR(4)) {
      c_gen0 => new TestFirGen4_0(c_gen0)
    } should be (true)
  }

  "Basic gen_test2 using Driver.execute" should "be used as an alternative way to run specification" in {
    iotesters.Driver.execute(Array(), () => new GenericFIR(4)) {
      c_gen1 => new TestFirGen4_1(c_gen1)
    } should be (true)
  }

  "Basic gen_test3 using Driver.execute" should "be used as an alternative way to run specification" in {
    iotesters.Driver.execute(Array(), () => new GenericFIR(4)) {
      c_gen2 => new TestFirGen4_2(c_gen2)
    } should be (true)
  }

  println("SUCCESS!!") // Scala Code: if we get here, our tests passed!
}
