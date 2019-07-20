package ooo

abstract class RVConfig()
{
  val xlen        = 32
  val bus_width   = 16
  val debug       = false
  val fetch_width = 1
}

case class RV64IConfig() extends RVConfig
{
  override val xlen        = 64
  override val bus_width   = 16
  override val debug       = true
  override val fetch_width = 2
}


case class RV64ISynth() extends RVConfig
{
  override val xlen        = 64
  override val bus_width   = 16
  override val debug       = false
  override val fetch_width = 2
}
