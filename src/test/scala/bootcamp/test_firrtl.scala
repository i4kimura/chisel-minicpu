package test_firrtl

import chisel3._
import chisel3.util._
import java.io._
import firrtl._

import chisel3.iotesters.{ChiselFlatSpec, Driver, PeekPokeTester}

class Tester extends ChiselFlatSpec {

  def stringifyAST(firrtlAST: firrtl.ir.Circuit): String = {
    var ntabs = 0
    val buf = new StringBuilder
    val string = firrtlAST.toString
    string.zipWithIndex.foreach { case (c, idx) =>
      c match {
        case ' ' =>
        case '(' =>
          ntabs += 1
          buf ++= "(\n" + "| " * ntabs
        case ')' =>
          ntabs -= 1
          buf ++= "\n" + "| " * ntabs + ")"
        case ','=> buf ++= ",\n" + "| " * ntabs
        case  c if idx > 0 && string(idx-1)==')' =>
          buf ++= "\n" + "| " * ntabs + c
        case c => buf += c
      }
    }
    buf.toString
  }

  val delay2_fp = new PrintWriter("delay2.fir")
  delay2_fp.write(chisel3.Driver.emit(() => new DelayBy2(4)))
  delay2_fp.close()

  val firrtlSerialization = chisel3.Driver.emit(() => new DelayBy2(4))
  val firrtlAST = firrtl.Parser.parse(firrtlSerialization.split("\n").toIterator, Parser.GenInfo("file.fir"))

  println(firrtlAST)
  println(stringifyAST(firrtlAST))
}
