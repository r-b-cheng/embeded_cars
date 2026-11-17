# wheel_drive -- PC 端逻辑自测（不编译 board_tm4c123g.c / main.c，它们依赖 TivaWare）
# 用法：在本目录执行 make test
CC      := gcc
CFLAGS  := -std=c99 -Wall -Wextra -Werror -pedantic -O2
LIB     := ../library
INCS    := -I. -I$(LIB)/include -I$(LIB)/ports/mock
SRCS    := wheel.c test_wheel.c \
           $(LIB)/src/clib_common.c \
           $(LIB)/src/clib_port.c \
           $(LIB)/src/clib_motor.c \
           $(LIB)/ports/mock/clib_port_mock.c
TARGET  := build/test_wheel.exe

.PHONY: all test clean
all: test

$(TARGET): $(SRCS) wheel.h Makefile
	@mkdir -p build
	$(CC) $(CFLAGS) $(INCS) $(SRCS) -o $(TARGET) -lm

test: $(TARGET)
	./$(TARGET)

clean:
	rm -f $(TARGET)
