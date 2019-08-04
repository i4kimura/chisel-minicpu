.PHONY = regression gen_test_class invididual_tests cpu_verilog mod_verilog ooo_verilog gen_test_class

par_scala_regression: gen_test_class
	sbt 'testOnly cpu.Parallel_ScalaPattern'

par_rtl_regression: gen_test_class
	sbt 'testOnly cpu.Parallel_RtlPattern'

invididual_tests: gen_test_class
	$(MAKE) run_all

cpu_verilog:
	sbt 'runMain cpu.CpuTop'

mod_verilog:
	sbt 'runMain mod_test.ParamModTop'

ooo_verilog:
	sbt 'runMain ooo.OooTile'
ooo_test:
	sbt 'testOnly ooo.Tester'
ooo_rtltest:
	sbt 'testOnly ooo.RtlTester'

gen_test_class: compile_tests
	ruby ./gen_test_class.rb

compile_tests:
	$(MAKE) -C tests/riscv-tests/isa

-include rv64ui_test.mk

clean:
	rm -rf test_run_dir project target *.v *.fir *.json *.log
