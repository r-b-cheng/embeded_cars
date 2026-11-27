INCLUDE ../servo_sg90/tm4c123gh6pm.ld
SECTIONS {
    .ram_vectors (NOLOAD) : ALIGN(1024) { *(vtable) } > SRAM
    end = .;
}
ASSERT(end + 4096 <= _stack_top, "Insufficient stack space")
_event_log_start = ORIGIN(FLASH) + LENGTH(FLASH) - 2048;
ASSERT(LOADADDR(.data) + SIZEOF(.data) <= _event_log_start, "Firmware overlaps event journal")
