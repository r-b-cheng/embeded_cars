#include "wheel.h"
#include "clib_motor.h"

/*
 * L298N 接法：IN1/IN2 管左轮，IN3/IN4 管右轮，两路互补 PWM。
 *   IN1=正转 / IN2=反转，IN3=正转 / IN4=反转。
 * ★ 不是「方向 GPIO + ENA/ENB 使能」的接法：ENA/ENB 跳线帽要一直插着。
 * 若实车方向相反，优先在 board_tm4c123g.c 里换 PWM 输出顺序，别改库。
 */
static uint8_t abs_clamp(int16_t v)
{
    if (v >  WHEEL_SPEED_MAX) { v =  WHEEL_SPEED_MAX; }
    if (v < -WHEEL_SPEED_MAX) { v = -WHEEL_SPEED_MAX; }
    return (uint8_t)((v < 0) ? -v : v);
}

int Wheel_Init(const clib_port_t *port)
{
    int ret = clib_init(port);
    if (ret != CLIB_OK) {
        return ret;
    }
    return Wheel_Stop();   /* 上电先确保轮子不动 */
}

int Wheel_Set(int16_t left, int16_t right)
{
    uint8_t duty[CLIB_MOTOR_CHANNELS];
    uint8_t l = abs_clamp(left);
    uint8_t r = abs_clamp(right);

    duty[0] = (left  > 0) ? l : 0u;   /* IN1 左轮正转 */
    duty[1] = (left  < 0) ? l : 0u;   /* IN2 左轮反转 */
    duty[2] = (right > 0) ? r : 0u;   /* IN3 右轮正转 */
    duty[3] = (right < 0) ? r : 0u;   /* IN4 右轮反转 */

    return clib_motor_set_raw(duty);
}

int Wheel_Stop(void)
{
    return clib_motor_set(CLIB_ACT_STOP, 0);
}

int Wheel_Brake(void)
{
    uint8_t duty[CLIB_MOTOR_CHANNELS] = { 100u, 100u, 100u, 100u };
    return clib_motor_set_raw(duty);
}
