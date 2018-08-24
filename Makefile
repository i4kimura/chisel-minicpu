test:
	sbt 'testOnly cpu.CpuTopTester -- -z Basic'

clean:
	rm -rf test_run_dir project/target target

