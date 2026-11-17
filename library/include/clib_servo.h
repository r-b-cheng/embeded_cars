#ifndef CLIB_SERVO_H
#define CLIB_SERVO_H
/*
 * clib_servo.h -- 舵机控制（向外设发送控制消息）
 *
 * 来源：案例 9 / 2-38 的 Servo.c
 *   原实现：TIM3_PWM_Init(19999,71) → 50Hz，计数周期 20000，1 计数 = 1us
 *           Servo_SetAngle(a) = TIM_SetCompare2(TIM3, a/180*2000+500)
 *   抽象后：库只负责「角度 → 目标脉宽(us)」这一条与芯片无关的换算，
 *           真正的 PWM 写寄存器交给 port->servo_set_pulse_us()。
 *           TM4C123G 上换 PWMGenPeriodSet(SysCtlClockGet()/50) 即可，库不用改。
 *
 * 附带一个非阻塞三方向扫描状态机，替代原案例里
 * 「Servo_SetAngle + Delay_ms(几百) + 测距」的阻塞写法（原写法会让导航控制环停摆）。
 */
#include "clib_common.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CLIB_SERVO_MIN_US      500u
#define CLIB_SERVO_MAX_US      2500u
#define CLIB_SERVO_PERIOD_US   20000u   /* 50Hz */
#define CLIB_SERVO_MIN_DEG     0.0f
#define CLIB_SERVO_MAX_DEG     180.0f
#define CLIB_SERVO_SCAN_MAX    8u
/* clib_servo_scan_tick() 的「扫描完毕」返回值，是流程状态不是错误码 */
#define CLIB_SERVO_SCAN_FINISHED (-1)

typedef struct {
    uint16_t min_us;      /* 0° 对应脉宽 */
    uint16_t max_us;      /* 180° 对应脉宽 */
    float    min_deg;
    float    max_deg;
} clib_servo_cal_t;

extern const clib_servo_cal_t CLIB_SERVO_CAL_DEFAULT;

/* ---- 纯函数：角度 <-> 脉宽，不碰硬件，可单独测试 ---- */
uint16_t clib_servo_angle_to_pulse(float angle);
uint16_t clib_servo_angle_to_pulse_cal(float angle, const clib_servo_cal_t *cal);
float    clib_servo_pulse_to_angle(uint16_t pulse_us, const clib_servo_cal_t *cal);
uint16_t clib_servo_clamp_pulse(uint16_t pulse_us);

/* ---- 发送控制消息到外设 ---- */
int clib_servo_set_pulse(uint16_t pulse_us);
int clib_servo_set_angle(float angle);
int clib_servo_set_angle_cal(float angle, const clib_servo_cal_t *cal);

/* ---- 非阻塞多方向扫描（比如 90°/175°/5° 三向测距）---- */
typedef struct {
    const float *angles;
    uint8_t  count;
    uint8_t  index;
    uint16_t settle_ms;
    uint32_t t_switch;
    bool     first_pending;
    bool     running;
    bool     done;
} clib_servo_scan_t;

/* angles 数组必须在扫描期间保持有效（通常是 static const） */
int  clib_servo_scan_begin(clib_servo_scan_t *s, const float *angles,
                           uint8_t count, uint16_t settle_ms);
/* 返回 1 = 刚切到新角度（*angle_out 为当前角度，可开始测距）；
 *       0 = 舵机还在稳定中；
 *      -1 = 全部角度扫描完毕 */
int  clib_servo_scan_tick(clib_servo_scan_t *s, float *angle_out);
bool clib_servo_scan_done(const clib_servo_scan_t *s);
void clib_servo_scan_abort(clib_servo_scan_t *s);

#ifdef __cplusplus
}
#endif
#endif /* CLIB_SERVO_H */
