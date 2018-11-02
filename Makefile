test:
	sbt 'testOnly misc.Tester -- -z Basic'

clean:
	rm -rf test_run_dir project/target target
