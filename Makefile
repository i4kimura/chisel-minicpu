.PHONY = regression gen_test_class invididual_tests cpu_verilog gen_test_class

regression: gen_test_class
	sbt 'testOnly cpu.Tester_AllPattern'

invididual_tests: gen_test_class
	$(MAKE) run_all

cpu_verilog:
	sbt 'runMain cpu.CpuTop'

gen_test_class:
	ruby ./gen_test_class.rb

include rv64ui_test.mk

clean:
	rm -rf test_run_dir project target *.v *.fir *.json *.log
