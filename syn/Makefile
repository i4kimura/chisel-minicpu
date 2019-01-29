all: cpu_verilog
	vivado -mode batch -source syn_vivado.tcl

cpu_verilog:
	$(MAKE) -C ../ cpu_verilog

clean:
	rm -rf *.jou *.log *.dcp *.syn.v *.rpt
