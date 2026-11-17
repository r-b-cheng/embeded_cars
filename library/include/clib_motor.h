#ifndef CLIB_MOTOR_H
#define CLIB_MOTOR_H
/*
 * clib_motor.h -- L298N 四路 PWM 差速底盘（向外设发送控制消息）
 *
 * 来源：案例 1 的 robot.c / PWM.c
 *   原实现：TIM4 CH1~4 (PB6~PB9)，ARR=99、PSC=35 → 20kHz，
 *           robot_speed(l1,l2,r1,r2) 直接把 0~100 写进 4 个 CCR。
 *   抽象后：库只做「动作 + 速度 → 四路占空比」的映射，
 *           写 PWM 交给 port->motor_set_duty(ch, duty)。
 *
 * ★ 相对原厂代码的重要改动：
 *   原 makerobo_run(speed, time) 会在延时后自动停车（Delay_ms 阻塞），
 *   无法用于导航控制环。这里改成「持续输出速度指令」，
 *   停车必须显式调用 clib_motor_stop()，控制周期由调用方决定。
 */
#include "clib_common.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CLIB_MOTOR_CHANNELS   4u
#define CLIB_MOTOR_DUTY_MAX   100u

/* 纯函数：把动作映射成 4 路占空比 [L1, L2, R1, R2]，顺序与 L298N IN1~IN4 一致
 * speed 用 int16_t 而非 int8_t：原厂仅 makerobo_run() 在 robot.h 声明为 uint8_t
 * 而 robot.c 定义为 int8_t（其余 5 个动作两处都是 int8_t），类型不一致会让
 * >127 的速度在调用处就被截断，钳位逻辑形同虚设。 */
void clib_diff_map(clib_act_t act, int16_t speed, uint8_t out[CLIB_MOTOR_CHANNELS]);

/* 发送控制消息到外设 */
int clib_motor_set(clib_act_t act, int16_t speed);
int clib_motor_set_raw(const uint8_t duty[CLIB_MOTOR_CHANNELS]);
int clib_motor_stop(void);

#ifdef __cplusplus
}
#endif
#endif /* CLIB_MOTOR_H */
