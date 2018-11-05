test:
	sbt 'testOnly counting_adders.Tester -- -z Basic'

clean:
	rm -rf test_run_dir project/target target
