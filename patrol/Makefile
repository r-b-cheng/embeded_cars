TIVAWARE ?= $(HOME)/ti/TivaWare_C_Series-2.2.0.295
ARM := arm-none-eabi
CFLAGS := -std=c11 -Wall -Wextra -Werror -Os -g -ffunction-sections -fdata-sections
ARCH := -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16
LIB := ../library
WHEEL := ../wheel_drive
BOARD := ../servo_sg90
SOURCES := $(wildcard firmware/*.c) $(WHEEL)/wheel.c $(WHEEL)/board_tm4c123g.c \
           $(addprefix $(LIB)/src/,clib_port.c clib_motor.c clib_ultrasonic.c)
.PHONY: all test
all: dist/patrol.bin
build/patrol.elf: $(SOURCES) $(wildcard firmware/*.h) firmware/vectors.ld $(BOARD)/startup.S $(BOARD)/tm4c123gh6pm.ld
	mkdir -p build
	$(ARM)-gcc $(CFLAGS) $(ARCH) -DPART_TM4C123GH6PM -DTARGET_IS_TM4C123_RB1 \
	  -I$(TIVAWARE) -I$(LIB)/include -I$(WHEEL) $(BOARD)/startup.S $(SOURCES) \
	  -nostartfiles --specs=nano.specs --specs=nosys.specs -Wl,--gc-sections,-z,noexecstack,--fatal-warnings \
	  -T firmware/vectors.ld -L$(TIVAWARE)/driverlib/gcc \
	  -ldriver -lm -o $@
	$(ARM)-size $@
dist/patrol.bin: build/patrol.elf
	mkdir -p dist
	$(ARM)-objcopy -O binary $< $@

.PHONY: sim
sim: build/sim_car.so
build/sim_car.so: tests/sim_car.c firmware/control.c firmware/protocol.c $(wildcard firmware/*.h)
	mkdir -p build
	$(CC) $(CFLAGS) -fPIC -shared -Ifirmware $(filter %.c,$^) -lm -o $@

JAVA_SRC := android/src/org/embeddedcars/patrol
test: build/control_test build/java/org/embeddedcars/patrol/NavigationTest.class sim
	./build/control_test
	java -ea -cp build/java org.embeddedcars.patrol.NavigationTest
	python3 tests/system_test.py

build/control_test: tests/control_test.c firmware/control.c firmware/radio.c firmware/protocol.c $(wildcard firmware/*.h)
	mkdir -p build
	$(CC) $(CFLAGS) -Ifirmware $(filter %.c,$^) -lm -o $@

build/java/org/embeddedcars/patrol/NavigationTest.class: tests/NavigationTest.java $(addprefix $(JAVA_SRC)/,Pilot.java Grid.java Frame.java)
	mkdir -p build/java
	javac -encoding UTF-8 -d build/java $^
