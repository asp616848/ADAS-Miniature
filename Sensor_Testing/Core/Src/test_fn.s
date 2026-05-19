.syntax unified
.cpu cortex-m3
.thumb
.thumb_func
.global test_fn_asm
.type test_fn_asm, %function
test_fn_asm:
    push {lr}
    movs r1, #3
    muls r0, r0, r1
    adds r0, r0, #7
    pop {pc}

.size test_fn_asm, .-test_fn_asm
