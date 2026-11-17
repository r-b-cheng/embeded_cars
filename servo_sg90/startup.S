.syntax unified
.cpu cortex-m4
.thumb

.section .isr_vector,"a",%progbits
.word _stack_top
.word Reset_Handler
/* The application polls SysTick and does not enable any interrupts. */
.rept 153
.word Default_Handler
.endr

.section .text.Reset_Handler,"ax",%progbits
.global Reset_Handler
.type Reset_Handler,%function
.thumb_func
Reset_Handler:
    ldr r0, =_data_load
    ldr r1, =_data_start
    ldr r2, =_data_end
1:  cmp r1, r2
    bhs 2f
    ldr r3, [r0], #4
    str r3, [r1], #4
    b 1b
2:  ldr r1, =_bss_start
    ldr r2, =_bss_end
    movs r3, #0
3:  cmp r1, r2
    bhs 4f
    str r3, [r1], #4
    b 3b
4:  bl main
    b Default_Handler
.size Reset_Handler, .-Reset_Handler

.type Default_Handler,%function
.thumb_func
Default_Handler:
    b .
.size Default_Handler, .-Default_Handler

.section .note.GNU-stack,"",%progbits
