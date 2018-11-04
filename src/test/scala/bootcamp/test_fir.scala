package fir

import chisel3._
import chisel3.util._
import chisel3.iotesters.{ChiselFlatSpec, Driver, PeekPokeTester}
// get some math functions
import scala.math.{abs, round, sin, cos, Pi, pow}
import breeze.signal.{filter, OptOverhang}
import breeze.signal.support.{CanFilter, FIRKernel1D}
import breeze.linalg.DenseVector

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

class TestMyFir (c: MyFir, length: Int, bitwidth: Int, window: (Int, Int) => Seq[Int]) extends PeekPokeTester(c) {
  // test data
  val n = 100 // input length
  val sine_freq = 10
  val samp_freq = 100

  // sample data, scale to between 0 and 2^bitwidth
  val max_value = pow(2, bitwidth)-1
  val sine = (0 until n).map(i => (max_value/2 + max_value/2*sin(2*Pi*sine_freq/samp_freq*i)).toInt)
  //println(s"input = ${sine.toArray.deep.mkString(", ")}")

  // coefficients
  val coeffs = window(length, bitwidth)
  //println(s"coeffs = ${coeffs.toArray.deep.mkString(", ")}")

  // use breeze filter as golden model; need to reverse coefficients
  val expected = filter(DenseVector(sine.toArray),
    FIRKernel1D(DenseVector(coeffs.reverse.toArray), 1.0, ""),
    OptOverhang.None)
  //println(s"exp_out = ${expected.toArray.deep.mkString(", ")}")

  // push data through our FIR and check the result
  reset(5)
  for (i <- 0 until n) {
    poke(c.io.in, sine(i))
    if (i >= length-1) { // wait for all registers to be initialized since we didn't zero-pad the data
      expect(c.io.out, expected(i-length+1))
      //println(s"cycle $i, got ${peek(c.io.out)}, expect ${expected(i-length+1)}")
    }
    step(1)
  }
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

  // simple triangular window
  val TriangularWindow: (Int, Int) => Seq[Int] = (length, bitwidth) => {
    val raw_coeffs = (0 until length).map( (x:Int) => 1-abs((x.toDouble-(length-1)/2.0)/((length-1)/2.0)) )
    val scaled_coeffs = raw_coeffs.map( (x: Double) => round(x * pow(2, bitwidth)).toInt)
    scaled_coeffs
  }

  // Hamming window
  val HammingWindow: (Int, Int) => Seq[Int] = (length, bitwidth) => {
    val raw_coeffs = (0 until length).map( (x: Int) => 0.54 - 0.46*cos(2*Pi*x/(length-1)))
    val scaled_coeffs = raw_coeffs.map( (x: Double) => round(x * pow(2, bitwidth)).toInt)
    scaled_coeffs
  }

  "Basic test_myfir using Driver.execute" should "be used as an alternative way to run specification" in {
    iotesters.Driver.execute(Array(), () => new MyFir(length = 7, bitwidth = 12, window = TriangularWindow)) {
      c_gen2 => new TestMyFir(c = c_gen2, length = 7, bitwidth = 12, window = TriangularWindow)
    } should be (true)
  }

  println("SUCCESS!!") // Scala Code: if we get here, our tests passed!
}
