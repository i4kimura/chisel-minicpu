test:
	sbt 'testOnly fir.Tester -- -z Basic'

clean:
	rm -rf test_run_dir project/target target
