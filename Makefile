cpu_run:
	sbt 'testOnly cpu.Tester -- -z Basic'

cpu_verilog:
	sbt 'runMain cpu.CpuTop'

test:
	sbt 'testOnly counting_adders.Tester -- -z Basic'

clean:
	rm -rf test_run_dir project target *.v *.fir *.json
