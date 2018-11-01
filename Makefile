test:
	sbt 'testOnly registerfile.Tester -- -z Basic'

clean:
	rm -rf test_run_dir project/target target
