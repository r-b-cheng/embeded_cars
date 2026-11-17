CC := arm-none-eabi-gcc
OBJCOPY := arm-none-eabi-objcopy
NM := arm-none-eabi-nm
SIZE := arm-none-eabi-size
PYTHON ?= python3
FLAGS := -mcpu=cortex-m4 -mthumb -mfloat-abi=soft -std=c11 -Os -g3 \
         -ffreestanding -fno-unwind-tables -fno-asynchronous-unwind-tables \
         -ffunction-sections -fdata-sections -Wall -Wextra -Werror -nostdlib

.PHONY: all legacy
all: build/servo_sg90_hold.bin build/servo_sg90_hold.symbols.json
legacy: build/servo_sg90.bin

build/servo_sg90_hold.elf: hold.c main.c startup.S tm4c123gh6pm.ld
	mkdir -p build
	$(CC) $(FLAGS) -Wl,--gc-sections,-z,noexecstack,--fatal-warnings \
	  -Wl,-Map=build/servo_sg90_hold.map -T tm4c123gh6pm.ld startup.S hold.c -o $@
	$(SIZE) $@

build/servo_sg90_hold.bin: build/servo_sg90_hold.elf
	$(OBJCOPY) -O binary $< $@

build/servo_sg90_hold.symbols.json: build/servo_sg90_hold.elf build/servo_sg90_hold.bin tools/export_symbols.py
	$(PYTHON) tools/export_symbols.py --nm $(NM) --elf $< --binary build/servo_sg90_hold.bin --output $@

build/servo_sg90.elf: main.c startup.S tm4c123gh6pm.ld
	mkdir -p build
	$(CC) $(FLAGS) -Wl,--gc-sections,-z,noexecstack,--fatal-warnings \
	  -Wl,-Map=build/servo_sg90.map -T tm4c123gh6pm.ld startup.S main.c -o $@
	$(SIZE) $@

build/servo_sg90.bin: build/servo_sg90.elf
	$(OBJCOPY) -O binary $< $@
