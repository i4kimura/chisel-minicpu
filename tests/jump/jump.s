.section    .text.init

_start:
    j       target
    nop

    add 	x1, x10, x20
    add 	x2, x11, x21
    add 	x3, x12, x22
    add 	x4, x13, x23
    add 	x5, x14, x24
    add 	x6, x15, x25
    add 	x7, x16, x26
    add 	x8, x17, x27

Return:
    j       next_0
    nop

target:
    j       Return
    nop

next_0:
    la      x10, jalr_target
    jalr    x10
    nop

    j       next_1
    nop

    /* Dummy */
    li  x10, 0xdf4ab835b72f5a1c
    li  x11, 0xb3fefd77eb062f44
    li  x12, 0x9b0c605e63aadd2b
    li  x13, 0x8cd5b53f23cedcc1
    li  x14, 0x48a2b40b10a2099a

jalr_target:
    ret
    nop

next_1:
    la      x10, jr_target
    jr      x10
    nop

    /* Dummy */
    li  x10, 0xdf4ab835b72f5a1c
    li  x11, 0xb3fefd77eb062f44
    li  x12, 0x9b0c605e63aadd2b
    li  x13, 0x8cd5b53f23cedcc1
    li  x14, 0x48a2b40b10a2099a

jr_target:
    nop

End:
