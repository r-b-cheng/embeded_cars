#ifndef BOARD_TM4C123G_H
#define BOARD_TM4C123G_H
/*
 * TM4C123G LaunchPad 板级初始化 + clib_port_t 实现。
 * 这是整个例程里唯一和芯片有关的文件。
 */
#include "clib_port.h"

void Board_Init(void);                     /* 在 main() 最开头调用一次 */
extern const clib_port_t g_board_port;     /* 传给 Wheel_Init(&g_board_port) */

#endif /* BOARD_TM4C123G_H */
