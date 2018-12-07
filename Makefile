cpu_run:
	sbt 'testOnly cpu.Tester -- -z Basic'

cpu_run2:
	sbt 'testOnly cpu.Tester -- -z Basic3'

cpu_verilog:
	sbt 'runMain cpu.CpuTop'

test:
	sbt 'testOnly counting_adders.Tester -- -z Basic'

include rv64ui_test.mk

clean:
	rm -rf test_run_dir project target *.v *.fir *.json
