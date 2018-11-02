package misc

import chisel3._
import chisel3.util._
import chisel3.iotesters.{ChiselFlatSpec, Driver, PeekPokeTester}

class Tester extends ChiselFlatSpec {

  "Basic test_PopCount2 using Driver.execute" should "be used as an alternative way to run specification" in {
    chisel3.iotesters.Driver(() => new PopCount2 ) { c => new PeekPokeTester(c) {
      // Integer.parseInt is used create an Integer from a binary specification
      poke(c.io.in, Integer.parseInt("00000000", 2))
      println(s"in=0b${peek(c.io.in).toInt.toBinaryString}, out=${peek(c.io.out)}")

      poke(c.io.in, Integer.parseInt("00001111", 2))
      println(s"in=0b${peek(c.io.in).toInt.toBinaryString}, out=${peek(c.io.out)}")

      poke(c.io.in, Integer.parseInt("11001010", 2))
      println(s"in=0b${peek(c.io.in).toInt.toBinaryString}, out=${peek(c.io.out)}")

      poke(c.io.in, Integer.parseInt("11111111", 2))
      println(s"in=0b${peek(c.io.in).toInt.toBinaryString}, out=${peek(c.io.out)}")
    }} should be (true)
  }

  "Basic test_Reverse2 using Driver.execute" should "be used as an alternative way to run specification" in {
    chisel3.iotesters.Driver(() => new Reverse2) { c => new PeekPokeTester(c) {
      // Integer.parseInt is used create an Integer from a binary specification
      poke(c.io.in, Integer.parseInt("01010101", 2))
      println(s"in=0b${peek(c.io.in).toInt.toBinaryString}, out=0b${peek(c.io.out).toInt.toBinaryString}")

      poke(c.io.in, Integer.parseInt("00001111", 2))
      println(s"in=0b${peek(c.io.in).toInt.toBinaryString}, out=0b${peek(c.io.out).toInt.toBinaryString}")

      poke(c.io.in, Integer.parseInt("11110000", 2))
      println(s"in=0b${peek(c.io.in).toInt.toBinaryString}, out=0b${peek(c.io.out).toInt.toBinaryString}")

      poke(c.io.in, Integer.parseInt("11001010", 2))
      println(s"in=0b${peek(c.io.in).toInt.toBinaryString}, out=0b${peek(c.io.out).toInt.toBinaryString}")
    }} should be (true)
  }

  "Basic test_OneHot2 using Driver.execute" should "be used as an alternative way to run specification" in {
    chisel3.iotesters.Driver(() => new OneHot2) { c => new PeekPokeTester(c) {
      poke(c.io.in, 0)
      println(s"in=${peek(c.io.in)}, out=0b${peek(c.io.out).toInt.toBinaryString}")

      poke(c.io.in, 1)
      println(s"in=${peek(c.io.in)}, out=0b${peek(c.io.out).toInt.toBinaryString}")

      poke(c.io.in, 8)
      println(s"in=${peek(c.io.in)}, out=0b${peek(c.io.out).toInt.toBinaryString}")

      poke(c.io.in, 15)
      println(s"in=${peek(c.io.in)}, out=0b${peek(c.io.out).toInt.toBinaryString}")
    }} should be (true)
  }

  "Basic test_OHToUInt2 using Driver.execute" should "be used as an alternative way to run specification" in {
    chisel3.iotesters.Driver(() => new OHToUInt2) { c => new PeekPokeTester(c) {
      poke(c.io.in, Integer.parseInt("0000 0000 0000 0001".replace(" ", ""), 2))
      println(s"in=0b${peek(c.io.in).toInt.toBinaryString}, out=${peek(c.io.out)}")

      poke(c.io.in, Integer.parseInt("0000 0000 1000 0000".replace(" ", ""), 2))
      println(s"in=0b${peek(c.io.in).toInt.toBinaryString}, out=${peek(c.io.out)}")

      poke(c.io.in, Integer.parseInt("1000 0000 0000 0001".replace(" ", ""), 2))
      println(s"in=0b${peek(c.io.in).toInt.toBinaryString}, out=${peek(c.io.out)}")

      // Some invalid inputs:
      // None high
      poke(c.io.in, Integer.parseInt("0000 0000 0000 0000".replace(" ", ""), 2))
      println(s"in=0b${peek(c.io.in).toInt.toBinaryString}, out=${peek(c.io.out)}")

      // Multiple high
      poke(c.io.in, Integer.parseInt("0001 0100 0010 0000".replace(" ", ""), 2))
      println(s"in=0b${peek(c.io.in).toInt.toBinaryString}, out=${peek(c.io.out)}")
    }} should be (true)
  }

  "Basic test_Mux2_2 using Driver.execute" should "be used as an alternative way to run specification" in {
    chisel3.iotesters.Driver(() => new Mux2) { c => new PeekPokeTester(c) {
      poke(c.io.in_bits(0), 10)
      poke(c.io.in_bits(1), 20)

      // Select higher index only
      poke(c.io.in_sels(0), 0)
      poke(c.io.in_sels(1), 1)
      println(s"in_sels=${peek(c.io.in_sels)}, out=${peek(c.io.out)}")

      // Select both - arbitration needed
      poke(c.io.in_sels(0), 1)
      poke(c.io.in_sels(1), 1)
      println(s"in_sels=${peek(c.io.in_sels)}, out=${peek(c.io.out)}")

      // Select lower index only
      poke(c.io.in_sels(0), 1)
      poke(c.io.in_sels(1), 0)
      println(s"in_sels=${peek(c.io.in_sels)}, out=${peek(c.io.out)}")
    }} should be (true)
  }

  "Basic test_Mux1H_2 using Driver.execute" should "be used as an alternative way to run specification" in {
    chisel3.iotesters.Driver(() => new Mux1H_2) { c => new PeekPokeTester(c) {
      poke(c.io.in_bits(0), 10)
      poke(c.io.in_bits(1), 20)

      // Select index 1
      poke(c.io.in_sels(0), 0)
      poke(c.io.in_sels(1), 1)
      println(s"in_sels=${peek(c.io.in_sels)}, out=${peek(c.io.out)}")

      // Select index 0
      poke(c.io.in_sels(0), 1)
      poke(c.io.in_sels(1), 0)
      println(s"in_sels=${peek(c.io.in_sels)}, out=${peek(c.io.out)}")

      // Select none (invalid)
      poke(c.io.in_sels(0), 0)
      poke(c.io.in_sels(1), 0)
      println(s"in_sels=${peek(c.io.in_sels)}, out=${peek(c.io.out)}")

      // Select both (invalid)
      poke(c.io.in_sels(0), 1)
      poke(c.io.in_sels(1), 1)
      println(s"in_sels=${peek(c.io.in_sels)}, out=${peek(c.io.out)}")
    }} should be (true)
  }

  "Basic test_Counter2 using Driver.execute" should "be used as an alternative way to run specification" in {
    chisel3.iotesters.Driver(() => new Counter2) { c => new PeekPokeTester(c) {
      poke(c.io.count, 1)
      println(s"start: counter value=${peek(c.io.out)}")

      step(1)
      println(s"step 1: counter value=${peek(c.io.out)}")

      step(1)
      println(s"step 2: counter value=${peek(c.io.out)}")

      poke(c.io.count, 0)
      step(1)
      println(s"step without increment: counter value=${peek(c.io.out)}")

      poke(c.io.count, 1)
      step(1)
      println(s"step again: counter value=${peek(c.io.out)}")
    }} should be (true)
  }

  println("SUCCESS!!") // Scala Code: if we get here, our tests passed!
}
