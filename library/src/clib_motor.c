#include "clib_motor.h"
#include "clib_port.h"

/*
 * 映射表逐条对照 3.案例程序/.../1.机器人基本运动/Hardware/robot.c：
 *   run(s)       = (s, 0, s, 0)
 *   back(s)      = (0, s, 0, s)
 *   Left(s)      = (0, 0, s, 0)
 *   Right(s)     = (s, 0, 0, 0)
 *   Spin_Left(s) = (0, s, s, 0)
 *   Spin_Right(s)= (s, 0, 0, s)
 * 注意：这是「左右各两路互补 PWM」直接驱动 L298N 的 IN1~IN4，
 * 不是常见的「方向 GPIO + ENA/ENB 使能」接法，port 层别接错。
 */
void clib_diff_map(clib_act_t act, int16_t speed, uint8_t out[CLIB_MOTOR_CHANNELS])
{
    uint8_t s;
    uint8_t i;

    if (out == NULL) {
        return;
    }
    for (i = 0u; i < CLIB_MOTOR_CHANNELS; ++i) {
        out[i] = 0u;
    }
    /* 原代码对 speed 做 0~100 限幅；这里保留同样语义，但参数类型放宽到 int16_t，
       保证 >100 的值是在函数内部被钳位，而不是调用处隐式转换时被截断 */
    if (speed < 0) {
        speed = 0;
    }
    if (speed > (int16_t)CLIB_MOTOR_DUTY_MAX) {
        speed = (int16_t)CLIB_MOTOR_DUTY_MAX;
    }
    s = (uint8_t)speed;

    switch (act) {
    case CLIB_ACT_FORWARD:    out[0] = s; out[2] = s; break;
    case CLIB_ACT_BACKWARD:   out[1] = s; out[3] = s; break;
    case CLIB_ACT_LEFT:       out[2] = s; break;
    case CLIB_ACT_RIGHT:      out[0] = s; break;
    case CLIB_ACT_SPIN_LEFT:  out[1] = s; out[2] = s; break;
    case CLIB_ACT_SPIN_RIGHT: out[0] = s; out[3] = s; break;
    case CLIB_ACT_STOP:
    case CLIB_ACT_BRAKE:
    default:                  break;   /* 全 0 */
    }
}

int clib_motor_set_raw(const uint8_t duty[CLIB_MOTOR_CHANNELS])
{
    uint8_t i;
    uint8_t d;

    if (!clib_is_ready()) {
        return CLIB_ERR_NOT_INIT;
    }
    if (duty == NULL) {
        return CLIB_ERR_NULL;
    }
    for (i = 0u; i < CLIB_MOTOR_CHANNELS; ++i) {
        d = (duty[i] > CLIB_MOTOR_DUTY_MAX) ? (uint8_t)CLIB_MOTOR_DUTY_MAX : duty[i];
        clib_motor_hw_set_duty(i, d);
    }
    return CLIB_OK;
}

int clib_motor_set(clib_act_t act, int16_t speed)
{
    uint8_t duty[CLIB_MOTOR_CHANNELS];
    if ((int)act < 0 || (int)act >= (int)CLIB_ACT_COUNT) {
        return CLIB_ERR_PARAM;
    }
    clib_diff_map(act, speed, duty);
    return clib_motor_set_raw(duty);
}

int clib_motor_stop(void)
{
    return clib_motor_set(CLIB_ACT_STOP, 0);
}
