.section    .text.init

_start:

beq_test_0:
    li      x2, 0
    li      x3, 1

    beq     x2, x3, Fail_End
    nop

beq_test_1:
    li      x2, 1000
    li      x3, 1000

    beq     x2, x3, bne_test_0
    nop

    j       Fail_End
    nop

bne_test_0:
    li      x2, 2000
    li      x3, 2000

    bne     x2, x3, Fail_End
    nop

bne_test_1:
    li      x2, 1000
    li      x3, 2000

    bne     x2, x3, bge_test_0
    nop

    j       Fail_End
    nop

bge_test_0:
    li      x2, 999
    li      x3, 1000

    bge     x2, x3, Fail_End
    nop

bge_test_1:
    li      x2, 1000
    li      x3, 1000

    bge     x2, x3, bgeu_test_0
    nop

    j       Fail_End
    nop

bgeu_test_0:
    li      x2, 0
    li      x3, -1

    bgeu    x2, x3, Fail_End
    nop

bgeu_test_1:
    li      x2, -3
    li      x3, -5

    bgeu    x2, x3, blt_test_0
    nop

    j       Fail_End
    nop

blt_test_0:
    li      x2, 1000
    li      x3, 999

    blt     x2, x3, Fail_End
    nop

blt_test_1:
    li      x2, 1000
    li      x3, 1001

    blt     x2, x3, bltu_test_0
    nop

    j       Fail_End
    nop

bltu_test_0:
    li      x2, -1
    li      x3, -2

    bltu    x2, x3, Fail_End
    nop

bltu_test_1:
    li      x2, -5
    li      x3, -3

    bltu    x2, x3, goal
    nop

    j       Fail_End
    nop

goal:

    j       Pass
    nop

Fail_End:
    li      x1, 0xdeadbeef
    j       Fail_End
    nop

Pass:
