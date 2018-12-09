#!/bin/ruby

hex_list = Dir.glob("tests/riscv-tests/isa/rv64ui*-p-*.hex")

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
# CpuTestsAllPatterns
#
fp_all = File.open("./src/test/scala/cpu/Test_AllPatterns.scala", "w")

fp_all.puts("package cpu\n\n")
fp_all.puts("import chisel3.iotesters\n")
fp_all.puts("import chisel3.iotesters.{ChiselFlatSpec, Driver, PeekPokeTester}\n")

fp_all.puts("class Tester_AllPattern extends ChiselFlatSpec {\n")

hex_list.each{|hex_file|
  pattern_name = File.basename(hex_file, ".hex").gsub("-", "_")

  fp_all.puts("  \"" + pattern_name + " test using Driver.execute\" should \"be used as an alternative way to run specification\" in {\n")
  fp_all.puts("    iotesters.Driver.execute(Array(), () => new CpuTop(new RV64IConfig)) {\n")
  fp_all.puts("      c => new CpuTopTests(c, \"" + hex_file + "\", \"pipetrace." + pattern_name + ".log\")\n")
  fp_all.puts("    } should be (true)\n")
  fp_all.puts("  }\n")
}
fp_all.puts("}\n")

fp_all.close
