package neuron

import chisel3._
import chisel3.util._
import chisel3.iotesters.{ChiselFlatSpec, Driver, PeekPokeTester}
import chisel3.core.FixedPoint

class TestNeuron (c: Neuron) extends PeekPokeTester(c) {
  val inputs = Seq(Seq(0, 0), Seq(0, 1), Seq(1, 0), Seq(1, 1))

  // make these a sequence of two values
  val and_weights = Seq(0.5, 0.5)
  val or_weights  = Seq(1.0, 1.0)

  // push data through our Neuron and check the result (AND gate)
  reset(5)
  for (i <- inputs) {
    pokeFixedPoint(c.io.in(0), i(0))
    pokeFixedPoint(c.io.in(1), i(1))
    pokeFixedPoint(c.io.weights(0), and_weights(0))
    pokeFixedPoint(c.io.weights(1), and_weights(1))
    expectFixedPoint(c.io.out, i(0) & i(1), "ERROR")
    step(1)
  }

  // push data through our Neuron and check the result (OR gate)
  reset(5)
  for (i <- inputs) {
    pokeFixedPoint(c.io.in(0), i(0))
    pokeFixedPoint(c.io.in(1), i(1))
    pokeFixedPoint(c.io.weights(0), or_weights(0))
    pokeFixedPoint(c.io.weights(1), or_weights(1))
    expectFixedPoint(c.io.out, i(0) | i(1), "ERROR")
    step(1)
  }
}


class Tester extends ChiselFlatSpec {

  val Step: FixedPoint => FixedPoint = x => Mux(x <= 0.5.F(8.BP), 0.F(8.BP), 1.F(8.BP))
  val ReLU: FixedPoint => FixedPoint = x => Mux(x <= 0.5.F(8.BP), 0.F(8.BP), x)

  "Basic test_step_neuron using Driver.execute" should "be used as an alternative way to run specification" in {
    iotesters.Driver.execute(Array(), () => new Neuron(inputs = 2, act = Step)) {
      c_gen2 => new TestNeuron(c_gen2)
    } should be (true)
  }

  // "Basic test_relu_neuron using Driver.execute" should "be used as an alternative way to run specification" in {
  //   iotesters.Driver.execute(Array(), () => new Neuron(inputs = 2, act = ReLU)) {
  //     c_gen2 => new TestNeuron(c_gen2)
  //   } should be (true)
  // }

  println("SUCCESS!!") // Scala Code: if we get here, our tests passed!
}
