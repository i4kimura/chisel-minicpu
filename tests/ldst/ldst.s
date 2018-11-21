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

    li  x10, 0x2000

    sd  x21,  0(x10)
    sd  x22,  4(x10)
    sd  x23,  8(x10)
    sd  x24, 12(x10)


    ld  x12,  0(x10)
    ld  x13,  4(x10)
    ld  x14,  8(x10)
    ld  x15, 12(x10)
