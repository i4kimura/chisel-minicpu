package counting_adders

import chisel3._
import chisel3.util._
import java.io._
import firrtl._

import chisel3.iotesters.{ChiselFlatSpec, Driver, PeekPokeTester}

class Tester extends ChiselFlatSpec {

  def compileFIRRTL(
    inputFirrtl: String,
    compiler: firrtl.Compiler,
    customTransforms: Seq[firrtl.Transform] = Seq.empty,
    infoMode: firrtl.Parser.InfoMode = firrtl.Parser.IgnoreInfo,
    annotations: firrtl.AnnotationSeq = firrtl.AnnotationSeq(Seq.empty)
  ): String = {
    import firrtl.{Compiler, AnnotationSeq, CircuitState, ChirrtlForm, FIRRTLException}
    import firrtl.Parser._
    import scala.io.Source
    import scala.util.control.ControlThrowable
    import firrtl.passes._
    val outputBuffer = new java.io.CharArrayWriter
    try {
      //val parsedInput = firrtl.Parser.parse(Source.fromFile(input).getLines(), infoMode)
      val parsedInput = firrtl.Parser.parse(inputFirrtl.split("\n").toIterator, infoMode)
      compiler.compile(
        CircuitState(parsedInput, ChirrtlForm, annotations),
        outputBuffer,
        customTransforms)
    }

    catch {
      // Rethrow the exceptions which are expected or due to the runtime environment (out of memory, stack overflow)
      case p: ControlThrowable => throw p
      case p: PassException  => throw p
      case p: FIRRTLException => throw p
      // Treat remaining exceptions as internal errors.
      case e: Exception => firrtl.Utils.throwInternalError(exception = Some(e))
    }

    val outputString = outputBuffer.toString
    outputString
  }

  val firrtlSerialization = chisel3.Driver.emit(() => new AddMe(8, 4))
  val verilog = compileFIRRTL(firrtlSerialization, new firrtl.VerilogCompiler(), Seq(new AnalyzeCircuit()))
  println(verilog)

}
