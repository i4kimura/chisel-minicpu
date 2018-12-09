package cpu

abstract class RVConfig()
{
  val xlen = 32
  val bus_width = 16
}

case class RV64IConfig() extends RVConfig
{
  override val xlen = 64
  override val bus_width = 16
}
