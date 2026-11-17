# clib -- 小车外设通信库构建文件
# 在 organized/library 目录下执行： make test
# 也支持从任意目录执行： make -f <本文件绝对路径> test
#
# 直接编译等价命令：
# gcc -std=c99 -Wall -Wextra -Werror -pedantic -O2 \
#     -Iinclude -Iports/mock src/*.c ports/mock/clib_port_mock.c tests/test_all.c -o build/test_all.exe -lm

# 用 := 强制覆盖 make 内建的 CC=cc（Windows/Git Bash 下通常没有 cc）
CC       := gcc
CFLAGS   ?= -std=c99 -Wall -Wextra -Werror -pedantic -O2
LDLIBS   ?= -lm

# 所有路径都基于本 Makefile 所在目录，避免在别的 CWD 下执行时找不到源文件
ROOT     := $(dir $(abspath $(lastword $(MAKEFILE_LIST))))
CPPFLAGS ?= -I$(ROOT)include -I$(ROOT)ports/mock

LIB_SRC  := $(wildcard $(ROOT)src/*.c)
MOCK_SRC := $(ROOT)ports/mock/clib_port_mock.c
TEST_SRC := $(ROOT)tests/test_all.c
TARGET   := $(ROOT)build/test_all.exe

.PHONY: all test clean

all: test

$(TARGET): $(LIB_SRC) $(MOCK_SRC) $(TEST_SRC) \
           $(wildcard $(ROOT)include/*.h) $(ROOT)ports/mock/clib_port_mock.h
	@mkdir -p $(ROOT)build
	@# MinGW 的 ld 无法写含非 ASCII 字符的绝对路径输出，故 cd 到库目录后用相对路径编译
	cd $(ROOT) && $(CC) $(CFLAGS) -Iinclude -Iports/mock \
	    src/*.c ports/mock/clib_port_mock.c tests/test_all.c \
	    -o build/test_all.exe $(LDLIBS)

test: $(TARGET)
	cd $(ROOT) && ./build/test_all.exe

# 只删除本 Makefile 生成的测试程序，不递归删整个 build/ 目录
# （避免误删他人临时产物 / 中文路径下的权限问题）
clean:
	rm -f $(ROOT)build/test_all.exe
