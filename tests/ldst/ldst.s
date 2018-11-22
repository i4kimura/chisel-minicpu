_start:
    li  x21, 0xb3fefd77eb062f44
    li  x22, 0x9b0c605e63aadd2b
    li  x23, 0x8cd5b53f23cedcc1
    li  x24, 0x48a2b40b10a2099a
    li  x25, 0xea85a28863d36dc0
    li  x26, 0xcdf0787616da5d02
    li  x27, 0x560e1563e6dadfbc
    li  x28, 0xcbc0a96ce51fc370
    li  x29, 0x9261c0826773f9dc
    li  x30, 0xdf4ab835b72f5a1c

    li  x31, 0x2000

    sd  x21,  0(x31)
    sd  x22,  8(x31)
    sd  x23, 16(x31)
    sd  x24, 24(x31)

    ld  x12,  0(x31)
    ld  x13,  8(x31)
    ld  x14, 16(x31)
    ld  x15, 24(x31)

    sd  x21,  0(x31)
    sd  x22,  8(x31)
    sd  x23, 16(x31)
    sd  x24, 24(x31)

    ld  x12,  0(x31)
    ld  x13,  8(x31)
    ld  x14, 16(x31)
    ld  x15, 24(x31)

    lw  x1,  0(x31)
    lw  x2,  4(x31)
    lw  x3,  8(x31)
    lw  x4, 12(x31)

    lh  x5,  0(x31)
    lh  x6,  2(x31)
    lh  x7,  4(x31)
    lh  x8,  6(x31)

    lhu x1,  0(x31)
    lhu x2,  2(x31)
    lhu x3,  4(x31)
    lhu x4,  6(x31)

    lb  x5,  0(x31)
    lb  x6,  1(x31)
    lb  x7,  2(x31)
    lb  x8,  3(x31)

    lbu x9,  0(x31)
    lbu x10, 1(x31)
    lbu x11, 2(x31)
    lbu x12, 3(x31)
