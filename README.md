Chisel-MiniCPU
=============

Mini CPU written by completely Chisel.

## Description

Chisel MiniCPU is written by Chisel and not use Verilog. Completely simulatable by Scala environment.

## Requirement

### Linux

Chisel-MiniCPU is tested on Ubuntu 18.04 LTS.

### RISC-V Tools

riscv-tests repository is used to run regression test using Chisel-MiniCPU.
It is recommended to install riscv-tools and $RISCV environment variable should be set.

## Usage

```sh
git clone https://github.com/msyksphinz/chisel-minicpu.git
cd chisel-minicpu
cd tests/riscv-tests/isa
make  # Generate hex test file.
cd ../../../../
ruby ./gen_test_class.rb
make regression
```

`pipetrace.[test pattern].log` is generated and can be confirm pipeline behavior.

### `pipetrace.rv64ui_p_add.log` result

```sh
make test_run_rv64ui_p_add
```

- `pipetrace.rv64ui_p_add.log`
```
spike-dasm <  pipetrace.rv64ui_p_add.log  > pipetrace.rv64ui_p_add.dasm.log
less pipetrace.rv64ui_p_add.dasm.log
```


## Contribution

## Author

[msyksphinz](https://github.com/msyksphinz)

Email : msyksphinz.dev@gmail.com
