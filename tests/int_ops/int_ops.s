_start:
    li  x10, 0xdf4ab835b72f5a1c
    li  x11, 0xb3fefd77eb062f44
    li  x12, 0x9b0c605e63aadd2b
    li  x13, 0x8cd5b53f23cedcc1
    li  x14, 0x48a2b40b10a2099a
    li  x15, 0xea85a28863d36dc0
    li  x16, 0xcdf0787616da5d02
    li  x17, 0x560e1563e6dadfbc
    li	x18, 0x35d5bba84a49bbf7
    li	x19, 0xd6c626eeca8ddd6d

	li	x20, 0x217f86df5a3cc3b8
	li	x21, 0xd6eba27110a854e8
	li	x22, 0x4abf77412b68193e
	li	x23, 0xeb25dbe5154abbf5
	li	x24, 0x7895be9bb5da4006
	li	x25, 0x9b554535849829af
	li	x26, 0x16759cf8f4440a9a
	li	x27, 0x6928861382952fc5
	li	x28, 0x42c8795b71c35101
	li	x29, 0x1c9ea62cf9927130

    add 	x1, x10, x20
    add 	x2, x11, x21
    add 	x3, x12, x22
    add 	x4, x13, x23
    add 	x5, x14, x24
    add 	x6, x15, x25
    add 	x7, x16, x26
    add 	x8, x17, x27

    sub 	x1, x10, x20
    sub 	x2, x11, x21
    sub 	x3, x12, x22
    sub 	x4, x13, x23
    sub 	x5, x14, x24
    sub 	x6, x15, x25
    sub 	x7, x16, x26
    sub 	x8, x17, x27

    addi    x1, x10, 0x610
    andi    x2, x11, 0x580
    ori     x3, x12, 0x0cb
    xori    x4, x13, 0x553
    slti    x5, x14, 0x292
    sltiu   x6, x15, 0x7f5
    slli    x7, x16, 0x6
    srai    x8, x17, 0x7
    srli    x9, x18, 0x2

target:

    sll     x1, x10, x20
    add     x2, x11, x21
    sub     x3, x12, x22
    slt     x4, x13, x23
    sltu    x5, x14, x24
    and     x6, x15, x25
    or      x7, x16, x26
    xor     x8, x17, x27
    sra     x9, x18, x28
    srl     x10, x19, x29

    addiw   x1, x10, 0x170
    slliw   x2, x11, 0xb
    srliw   x3, x12, 0xc
    sraiw   x4, x13, 0x8
    addw    x5, x14, x24
    subw    x6, x15, x25
    sllw    x7, x16, x26
    srlw    x8, x17, x27
    sraw    x9, x18, x28

    j       target
