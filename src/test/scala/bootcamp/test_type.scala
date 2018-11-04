package chisel_type

import chisel3._
import chisel3.util._
import chisel3.iotesters.{ChiselFlatSpec, Driver, PeekPokeTester}
import java.io.PrintWriter
import dsptools.numbers._
import chisel3.experimental._

class TestType (c: TypeConvertDemo) extends PeekPokeTester(c) {
  poke(c.io.in, 3)
  expect(c.io.out, 3)
  poke(c.io.in, 15)
  expect(c.io.out, -1)
}


class ShiftRegisterTester[T <: Bits](c: ShiftRegister[T]) extends PeekPokeTester(c) {
  println(s"Testing ShiftRegister of type ${c.io.in} and depth ${c.io.out.length}")
  for (i <- 0 until 10) {
    poke(c.io.in, i)
    println(s"$i: ${peek(c.io.out)}")
    step(1)
  }
}


class MacTestModule extends Module {
  val io = IO(new Bundle {
    val uin = Input(UInt(4.W))
    val uout = Output(UInt())
    val sin = Input(SInt(4.W))
    val sout = Output(SInt())
    //val fin = Input(FixedPoint(16.W, 12.BP))
    //val fout = Output(FixedPoint())
  })
  // for each IO pair, do out = in * in + in
  io.uout := Mac(io.uin, io.uin, io.uin)
  io.sout := Mac(io.sin, io.sin, io.sin)
  //io.fout := Mac(io.fin, io.fin, io.fin)
}


class Tester extends ChiselFlatSpec {
  "Basic test_chisel_type using Driver.execute" should "be used as an alternative way to run specification" in {
    iotesters.Driver.execute(Array(), () => new TypeConvertDemo) {
      c => new TestType(c)
    } should be (true)
  }

  class ConstantSum(in1: Data, in2: Data) extends Module {
    val io = IO(new Bundle {
      val out = Output(in1.cloneType)
    })
      (in1, in2) match {
      case (x: UInt, y: UInt) => io.out := x + y
      case (x: SInt, y: SInt) => io.out := x + y
      case _ => throw new Exception("I give up!")
    }
  }

  // Convenience function to invoke Chisel and grab emitted Verilog.
  def getVerilog(dut: => chisel3.core.UserModule): String = {
    import firrtl._
    return chisel3.Driver.execute(Array[String](), {() => dut}) match {
      case s:chisel3.ChiselExecutionSuccess => s.firrtlResultOption match {
        case Some(f:FirrtlExecutionSuccess) => f.emitted
      }
    }
  }

  val constantsum0_fp = new PrintWriter("constant_sum0.v")
  constantsum0_fp.write(getVerilog(dut = new ConstantSum(3.U, 4.U)))
  constantsum0_fp.close()

  val constantsum1_fp = new PrintWriter("constant_sum1.v")
  constantsum1_fp.write(getVerilog(dut = new ConstantSum(-3.S, 4.S)))
  constantsum1_fp.close()

  // val constantsum2_fp = new PrintWriter("constant_sum2.v")
  // constantsum2_fp.write(getVerilog(dut = new ConstantSum(3.U, 4.S)))
  // constantsum2_fp.close()

  class Bundle1 extends Bundle {
    val a = UInt(8.W)
    override def cloneType = (new Bundle1).asInstanceOf[this.type]
  }

  class Bundle2 extends Bundle1 {
    val b = UInt(16.W)
    override def cloneType = (new Bundle2).asInstanceOf[this.type]
  }

  class BadTypeModule extends Module {
    val io = IO(new Bundle {
      val c  = Input(Clock())
      val in = Input(UInt(2.W))
      val out = Output(Bool())

      val bundleIn = Input(new Bundle2)
      val bundleOut = Output(new Bundle1)
    })

    //io.out := io.c // won't work due to different types

    // Okay, but Chisel will truncate the input width to 1 to match the output.
    io.out := io.in

    // Compiles; Chisel will connect the common subelements of the two Bundles (in this case, 'a').
    io.bundleOut := io.bundleIn
  }

  val badtypemodule_fp = new PrintWriter("bad_type_module.v")
  badtypemodule_fp.write(getVerilog(new BadTypeModule))
  badtypemodule_fp.close()


  Driver(() => new ShiftRegister(UInt(4.W), 5)) { c => new ShiftRegisterTester(c) }
  Driver(() => new ShiftRegister(SInt(6.W), 3)) { c => new ShiftRegisterTester(c) }


  val mac0_fp = new PrintWriter("mac0.v")
  mac0_fp.write(getVerilog(new Mac(UInt(4.W), UInt(6.W)) ))
  mac0_fp.close()

  val mac1_fp = new PrintWriter("mac0.v")
  mac1_fp.write(getVerilog(new Mac(SInt(4.W), SInt(6.W)) ))
  mac1_fp.close()

  val mac2_fp = new PrintWriter("mac0.v")
  mac2_fp.write(getVerilog(new Mac(FixedPoint(4.W, 3.BP), FixedPoint(6.W, 4.BP))))
  mac2_fp.close()


  // Driver(() => new Mac) { c => new MacTestModule(c) }
  // Driver(() => new Mac) { c => new MacTestModule(c) }

  println(getVerilog(new MacTestModule))

}
