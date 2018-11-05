package counting_adders

// Chisel stuff
import chisel3._
import chisel3.util._

// Compiler Infrastructure

// Firrtl IR classes

// Map functions

// Scala's mutable collections
import scala.collection.mutable

class Ledger {
  import firrtl.Utils
  private var moduleName: Option[String] = None
  private val modules = mutable.Set[String]()
  private val moduleAddMap = mutable.Map[String, Int]()
  def foundAdd(): Unit = moduleName match {
    case None => sys.error("Module name not defined in Ledger!")
    case Some(name) => moduleAddMap(name) = moduleAddMap.getOrElse(name, 0) + 1
  }
  def getModuleName: String = moduleName match {
    case None => Utils.error("Module name not defined in Ledger!")
    case Some(name) => name
  }
  def setModuleName(myName: String): Unit = {
    modules += myName
    moduleName = Some(myName)
  }
  def serialize: String = {
    modules map { myName =>
      s"$myName => ${moduleAddMap.getOrElse(myName, 0)} add ops!"
    } mkString "\n"
  }
}


class AnalyzeCircuit extends firrtl.Transform {
  import firrtl._
  import firrtl.ir._
  import firrtl.Mappers._
  import firrtl.Parser._
  import firrtl.annotations._
  import firrtl.PrimOps._

  // Requires the [[Circuit]] form to be "low"
  def inputForm = LowForm
  // Indicates the output [[Circuit]] form to be "low"
  def outputForm = LowForm

  // Called by [[Compiler]] to run your pass. [[CircuitState]] contains
  // the circuit and its form, as well as other related data.
  def execute(state: CircuitState): CircuitState = {
    val ledger = new Ledger()
    val circuit = state.circuit

    // Execute the function walkModule(ledger) on every [[DefModule]] in
    // circuit, returning a new [[Circuit]] with new [[Seq]] of [[DefModule]].
    //   - "higher order functions" - using a function as an object
    //   - "function currying" - partial argument notation
    //   - "infix notation" - fancy function calling syntax
    //   - "map" - classic functional programming concept
    //   - discard the returned new [[Circuit]] because circuit is unmodified
    circuit map walkModule(ledger)

    // Print our ledger
    println(ledger.serialize)

    // Return an unchanged [[CircuitState]]
    state
  }

  // Deeply visits every [[Statement]] in m.
  def walkModule(ledger: Ledger)(m: DefModule): DefModule = {
    // Set ledger to current module name
    ledger.setModuleName(m.name)

    // Execute the function walkStatement(ledger) on every [[Statement]] in m.
    //   - return the new [[DefModule]] (in this case, its identical to m)
    //   - if m does not contain [[Statement]], map returns m.
    m map walkStatement(ledger)
  }

  // Deeply visits every [[Statement]] and [[Expression]] in s.
  def walkStatement(ledger: Ledger)(s: Statement): Statement = {

    // Execute the function walkExpression(ledger) on every [[Expression]] in s.
    //   - discard the new [[Statement]] (in this case, its identical to s)
    //   - if s does not contain [[Expression]], map returns s.
    s map walkExpression(ledger)

    // Execute the function walkStatement(ledger) on every [[Statement]] in s.
    //   - return the new [[Statement]] (in this case, its identical to s)
    //   - if s does not contain [[Statement]], map returns s.
    s map walkStatement(ledger)
  }

  // Deeply visits every [[Expression]] in e.
  //   - "post-order traversal" - handle e's children [[Expression]] before e
  def walkExpression(ledger: Ledger)(e: Expression): Expression = {

    // Execute the function walkExpression(ledger) on every [[Expression]] in e.
    //   - return the new [[Expression]] (in this case, its identical to e)
    //   - if s does not contain [[Expression]], map returns e.
    val visited = e map walkExpression(ledger)

    visited match {
      // If e is an adder, increment our ledger and return e.
      case DoPrim(Add, _, _, _) =>
        ledger.foundAdd
        e
      // If e is not an adder, return e.
      case notadd => notadd
    }
  }
}


class AddMe(nInputs: Int, width: Int) extends Module {
  val io = IO(new Bundle {
    val in  = Input(Vec(nInputs, UInt(width.W)))
    val out = Output(UInt(width.W))
  })
  io.out := io.in.reduce(_ +& _)
}
