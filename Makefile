.PHONY = regression gen_test_class invididual_tests cpu_verilog gen_test_class

regression: gen_test_class
	sbt 'testOnly cpu.Tester_AllPattern'

par_regression: gen_test_class
	sbt 'testOnly cpu.Parallel_AllPattern'

invididual_tests: gen_test_class
	$(MAKE) run_all

cpu_verilog:
	sbt 'runMain cpu.CpuTop'

verilator:
	sbt 'testOnly cpu.Rtl_AllPattern'

mod_verilog:
	sbt 'runMain mod_test.ParamModTop'

gen_test_class: compile_tests
	ruby ./gen_test_class.rb

compile_tests:
	$(MAKE) -C tests/riscv-tests/isa

include rv64ui_test.mk

clean:
	rm -rf test_run_dir project target *.v *.fir *.json *.log
