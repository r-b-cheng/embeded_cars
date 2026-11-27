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
all:
ELF := build/patrol.elf
ifdef PERSONAL
ELF := build/personal/patrol.elf
RADIO_FLAGS := -DPATROL_PERSONAL -Ibuild/personal
$(ELF): build/personal/hotspot.h
endif
$(ELF): $(SOURCES) $(wildcard firmware/*.h) firmware/vectors.ld $(BOARD)/startup.S $(BOARD)/tm4c123gh6pm.ld
	mkdir -p $(@D)
	$(ARM)-gcc $(RADIO_FLAGS) $(CFLAGS) $(ARCH) -DPART_TM4C123GH6PM -DTARGET_IS_TM4C123_RB1 \
	  -I$(TIVAWARE) -I$(LIB)/include -I$(WHEEL) $(BOARD)/startup.S $(SOURCES) \
	  -nostartfiles --specs=nano.specs --specs=nosys.specs -Wl,--gc-sections,-z,noexecstack,--fatal-warnings \
	  -T firmware/vectors.ld -L$(TIVAWARE)/driverlib/gcc \
	  -ldriver -lm -o $@
	$(ARM)-size $@
ifndef PERSONAL
all: dist/patrol.bin
dist/patrol.bin: $(ELF)
	mkdir -p dist
	$(ARM)-objcopy -O binary $< $@

else
all: build/personal/patrol.bin
build/personal/patrol.bin: $(ELF)
	$(ARM)-objcopy -O binary $< $@
endif
.PHONY: personal
personal:
	$(MAKE) PERSONAL=1 build/personal/patrol.bin

.PHONY: sim
sim: build/sim_car.so
build/sim_car.so: tests/sim_car.c firmware/control.c firmware/protocol.c $(wildcard firmware/*.h)
	mkdir -p build
	$(CC) $(CFLAGS) -fPIC -shared -Ifirmware $(filter %.c,$^) -lm -o $@

JAVA_SRC := android/src/org/embeddedcars/patrol
test: build/control_test build/station_test build/java/org/embeddedcars/patrol/NavigationTest.class build/java/org/embeddedcars/patrol/DriveLogTest.class sim
	./build/control_test
	./build/station_test
	java -ea -cp build/java org.embeddedcars.patrol.NavigationTest
	java -ea -cp build/java org.embeddedcars.patrol.DriveLogTest
	python3 tests/system_test.py

HOST_TEST_SRC := tests/control_test.c firmware/control.c firmware/radio.c firmware/journal.c firmware/protocol.c
build/station_test: $(HOST_TEST_SRC) $(wildcard firmware/*.h) tests/hotspot.h
	mkdir -p build
	$(CC) $(CFLAGS) -Ifirmware -Itests -DPATROL_PERSONAL $(HOST_TEST_SRC) -lm -o $@
build/control_test: $(HOST_TEST_SRC) $(wildcard firmware/*.h)
	mkdir -p build
	$(CC) $(CFLAGS) -Ifirmware $(HOST_TEST_SRC) -lm -o $@

build/java/org/embeddedcars/patrol/NavigationTest.class: tests/NavigationTest.java $(addprefix $(JAVA_SRC)/,Pilot.java Grid.java Frame.java)
	mkdir -p build/java
	javac -encoding UTF-8 -d build/java $^

.PHONY: renode-test
build/renode.elf: tests/renode/harness.c firmware/control.c firmware/protocol.c firmware/journal.c firmware/radio.c firmware/radio_uart.c $(wildcard firmware/*.h) tests/hotspot.h firmware/vectors.ld $(BOARD)/startup.S $(BOARD)/tm4c123gh6pm.ld
	mkdir -p build
	$(ARM)-gcc $(CFLAGS) $(ARCH) -DPART_TM4C123GH6PM -DTARGET_IS_TM4C123_RB1 -DPATROL_PERSONAL \
	  -Itests -Ifirmware -I$(TIVAWARE) $(BOARD)/startup.S $(filter %.c,$^) \
	  -nostartfiles --specs=nano.specs --specs=nosys.specs -Wl,--gc-sections,-z,noexecstack,--wrap=UARTCharPutNonBlocking \
	  -T firmware/vectors.ld -L$(TIVAWARE)/driverlib/gcc -ldriver -lm -o $@
renode-test: build/renode.elf
	python3 tests/renode/run.py

build/java/org/embeddedcars/patrol/DriveLogTest.class: tests/DriveLogTest.java $(JAVA_SRC)/DriveLog.java
	mkdir -p build/java
	javac -encoding UTF-8 -d build/java $^
