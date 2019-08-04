package ooo

abstract class RVConfig()
{
  val xlen        = 32
  val bus_width   = 32
  val debug       = false
  val fetch_width = 1
  val memory_byte_w = 8  // 64-bit
}

case class RV64IConfig() extends RVConfig
{
  override val xlen        = 64
  override val bus_width   = 20
  override val debug       = true
  override val fetch_width = 2
  override val memory_byte_w = 8 // 64-bit
}


case class RV64ISynth() extends RVConfig
{
  override val xlen        = 64
  override val bus_width   = 20
  override val debug       = false
  override val fetch_width = 2
  override val memory_byte_w = 8 // 64-bit
}
