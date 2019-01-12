#!/bin/ruby

hex_list = Dir.glob("tests/riscv-tests/isa/rv64[usm]i-p-*.hex")

hex_patterns = hex_list.map{|hex_file| File.basename(hex_file, ".hex").gsub("-", "_")}

#
# Each Test Patterns
#
hex_list.each{|hex_file|
  fp = File.open("./src/test/scala/cpu/Test_" + File.basename(hex_file, ".hex") + ".scala", "w")

  pattern_name = File.basename(hex_file, ".hex").gsub("-", "_")

  fp.puts("package cpu\n\n")

  fp.puts("import chisel3.iotesters\n")
  fp.puts("import chisel3.iotesters.{ChiselFlatSpec, Driver, PeekPokeTester}\n")

  fp.puts("class Tester_" + pattern_name + " extends ChiselFlatSpec {\n")
  fp.puts("  \"Basic test using Driver.execute\" should \"be used as an alternative way to run specification\" in {\n")
  fp.puts("    iotesters.Driver.execute(Array(), () => new CpuTop(new RV64IConfig)) {\n")
  fp.puts("      c => new CpuTopTests(c, \"" + hex_file + "\", \"pipetrace." + pattern_name + ".log\")\n")
  fp.puts("    } should be (true)\n")
  fp.puts("  }\n")
  fp.puts("}\n")
  fp.close
}


#
# Test Pattern Makefile
#
fp = File.open("rv64ui_test.mk", "w")

fp.puts(".PHONY = \\")
hex_patterns.each{|pattern_name|
  fp.puts("test_run_" + pattern_name + " \\\n")
}
fp.puts("\n")

fp.puts("TEST_SETS = \\")
hex_patterns.each{|pattern_name|
  fp.puts("test_run_" + pattern_name + " \\\n")
}
fp.puts("\n")

fp.puts("run_all: $(TEST_SETS)\n\n")
hex_patterns.each{|pattern_name|
  fp.puts("test_run_" + pattern_name + ":\n")
  fp.puts("\tsbt \'testOnly cpu.Tester_" + pattern_name + " -- -z Basic\'")
}
fp.close


#
# Parallel
#
fp_all = File.open("./src/test/scala/cpu/Paralell_ScalaPatterns.scala", "w")

fp_all.puts("package cpu\n\n")
fp_all.puts("import chisel3.iotesters\n")
fp_all.puts("import chisel3.iotesters.{ChiselFlatSpec, Driver, PeekPokeTester}\n")

fp_all.puts("class Parallel_ScalaPattern extends ChiselFlatSpec {\n")
fp_all.puts("  val pattern_path = Array (\n")
hex_list.each_with_index{|hex_file, idx|
  if idx != hex_list.length-1 then
    fp_all.puts("    \"" + hex_file + "\",\n")
  else
    fp_all.puts("    \"" + hex_file + "\"\n")
  end
}
fp_all.puts("  )")

fp_all.puts("  val log_path = Array (\n")
hex_list.each_with_index{|hex_file, idx|
  pattern_name = File.basename(hex_file, ".hex").gsub("-", "_")
  if idx != hex_list.length-1 then
    fp_all.puts("    \"" + pattern_name + ".log\",\n")
  else
    fp_all.puts("    \"" + pattern_name + ".log\"\n")
  end
}
fp_all.puts("  )")

fp_all.puts("  (0 to " + (hex_list.length - 1).to_s  + ").par foreach { idx =>\n")
fp_all.puts("    iotesters.Driver.execute(Array(), () => new CpuTop(new RV64IConfig)) {\n")
fp_all.puts("      c => new CpuTopTests(c, pattern_path(idx), log_path(idx))\n")
fp_all.puts("    } should be (true)\n")
fp_all.puts("  }\n")

fp_all.puts("}\n")

fp_all.close


#
# RtlTestsAllPatterns
#
fp_all = File.open("./src/test/scala/cpu/Paralell_RtlPatterns.scala", "w")

fp_all.puts("package cpu\n\n")
fp_all.puts("import chisel3.iotesters\n")
fp_all.puts("import chisel3.iotesters.{ChiselFlatSpec, Driver, PeekPokeTester}\n")

fp_all.puts("class Parallel_RtlPattern extends ChiselFlatSpec {\n")
fp_all.puts("  val args = Array(\"--backend-name\", \"verilator\")")
fp_all.puts("  val pattern_path = Array (\n")
hex_list.each_with_index{|hex_file, idx|
  if idx != hex_list.length-1 then
    fp_all.puts("    \"" + hex_file + "\",\n")
  else
    fp_all.puts("    \"" + hex_file + "\"\n")
  end
}
fp_all.puts("  )")

fp_all.puts("  val log_path = Array (\n")
hex_list.each_with_index{|hex_file, idx|
  pattern_name = File.basename(hex_file, ".hex").gsub("-", "_")
  if idx != hex_list.length-1 then
    fp_all.puts("    \"" + pattern_name + ".log\",\n")
  else
    fp_all.puts("    \"" + pattern_name + ".log\"\n")
  end
}
fp_all.puts("  )")

fp_all.puts("  (0 to " + (hex_list.length - 1).to_s  + ").par foreach { idx =>\n")
fp_all.puts("    iotesters.Driver.execute(args, () => new CpuTop(new RV64IConfig)) {\n")
fp_all.puts("      c => new CpuTopTests(c, pattern_path(idx), log_path(idx))\n")
fp_all.puts("    } should be (true)\n")
fp_all.puts("  }\n")

fp_all.puts("}\n")

fp_all.close
