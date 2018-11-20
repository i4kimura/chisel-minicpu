all: $(TARGET).hex

$(TARGET).hex : $(TARGET).bin
	od -tx4 -v -w4 -Ax $^ | sed 's/^/0x/g' | gawk -F ' ' '{printf "@%08x %s\n", rshift(strtonum($$1), 2), $$2}' > $@

$(TARGET).bin : $(TARGET).elf
	riscv64-unknown-elf-objcopy --gap-fill 0 -O binary $^ $@

$(TARGET).elf : $(TARGET).s
	riscv64-unknown-elf-as $^ -o $(TARGET).o
	riscv64-unknown-elf-ld $(TARGET).o -T ../common/riscv.ld -o $@
	riscv64-unknown-elf-objdump -D $@ > $(TARGET).dmp

clean:
	rm -rf $(TARGET).hex $(TARGET).dmp $(TARGET).elf $(TARGET).bin *.o
