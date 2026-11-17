#ifndef WHEEL_H
#define WHEEL_H
/*
 * wheel.h -- 双轮差速驱动（基于 ../library 的 clib_motor）
 *
 * 只给左右轮各一个有符号速度，正数前进 / 负数后退 / 0 滑行：
 *     Wheel_Set(60,  60)   前进
 *     Wheel_Set(0,   60)   左转（左轮停、右轮转）
 *     Wheel_Set(-60, 60)   原地左转
 *     Wheel_Brake()        四路全高，L298N 动态刹车
 *
 * 本文件不含任何芯片依赖；物理管脚全部由 clib_port_t 决定。
 */
#include <stdint.h>
#include "clib_port.h"

#define WHEEL_SPEED_MAX 100

/* 注册平台 port，并让电机保持停止。TM4C 传 &g_board_port，PC 测试传 clib_mock_port() */
int Wheel_Init(const clib_port_t *port);

/* left / right ∈ [-100, 100]，超出自动钳位。持续输出，不会自动停车 */
int Wheel_Set(int16_t left, int16_t right);

int Wheel_Stop(void);    /* 四路 0% → 电机滑行 */
int Wheel_Brake(void);   /* 四路 100% → 电机两端短接，快速刹停 */

#endif /* WHEEL_H */
