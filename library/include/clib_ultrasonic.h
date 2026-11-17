#ifndef CLIB_ULTRASONIC_H
#define CLIB_ULTRASONIC_H
/*
 * clib_ultrasonic.h -- HC-SR04（发送触发脉冲 + 接收回波数据）
 *
 * 来源：案例 8/9 的 UltrasonicWave.c / timer.c
 *   原实现：PB15 输出 20us 高电平触发；PB14 上升沿外部中断，
 *           在中断里 while(GPIO_Read...) 忙等下降沿，用 TIM2 计数值换算距离。
 *   问题：中断内无超时忙等，没回波直接卡死；换算公式量纲混乱。
 *
 * 抽象后：触发 = 向外设发一条控制消息（clib_ultra_trigger）；
 *         回波 = 非阻塞状态机（clib_ultra_poll），带超时，永不卡死。
 *         强烈建议 TM4C 侧用 GPTM 输入捕获拿到脉宽后直接调
 *         clib_echo_us_to_cm()，精度远高于 GPIO 轮询。
 */
#include "clib_common.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CLIB_SOUND_CM_PER_US          0.0343f   /* 20°C 空气声速；往返 /2 在函数里处理 */
#define CLIB_ULTRA_TRIG_US            20u
#define CLIB_ULTRA_DEFAULT_TIMEOUT_US 30000u
#define CLIB_ULTRA_MAX_CM             400.0f
/* 原案例 TIM2：72MHz / (7199+1) = 10kHz → 1 计数 ≈ 100us
   （见 9.机器人超声波避障（带舵机摇头）/Hardware/timer.c 与 User/main.c 的 Timerx_Init(5000,7199)） */
#define CLIB_ULTRA_LEGACY_US_PER_CNT  100u

/* ---- 纯换算（可单独测试）---- */
float    clib_echo_us_to_cm(uint32_t pulse_us);
uint16_t clib_echo_us_to_mm(uint32_t pulse_us);
/* 参数化：count * us_per_count 得到脉宽微秒，再按标准公式换算成 cm。
   TM4C 上直接传捕获到的微秒数给 clib_echo_us_to_cm() 更好。 */
float    clib_echo_count_to_cm(uint32_t count, uint32_t us_per_count);
/* 原厂 UltrasonicWave_Distance = cnt*0.85（量纲不是 cm，仅为兼容保留） */
float    clib_echo_count_to_legacy(uint32_t count);
/* 原厂 UltrasonicWave_StartMeasure() 返回值 = Distance*10 = cnt*8.5。
   案例 9 里阈值 60 判的是「这个量」，不是 60cm。 */
float    clib_echo_legacy_measure(uint32_t count);

typedef enum {
    CLIB_ULTRA_IDLE = 0,
    CLIB_ULTRA_WAIT_RISE,
    CLIB_ULTRA_WAIT_FALL,
    CLIB_ULTRA_DONE,
    CLIB_ULTRA_TIMEOUT
} clib_ultra_state_t;

typedef struct {
    clib_ultra_state_t state;
    uint32_t t_start;
    uint32_t rise_us;
    uint32_t timeout_us;
    float    last_cm;
} clib_ultra_t;

/* 只发触发脉冲，不等待回波（含 20us 短延时，禁止在 ISR 内调用） */
int  clib_ultra_trigger(void);

void clib_ultra_start(clib_ultra_t *u, uint32_t timeout_us);
/* 返回 1 = 测到距离；0 = 进行中；负 = clib_err_t 错误码。
   已结束（DONE）返回 CLIB_ERR_BUSY，需重新 start；超时返回 CLIB_ERR_TIMEOUT。 */
int  clib_ultra_poll_at(clib_ultra_t *u, uint32_t now_us, float *cm_out);
int  clib_ultra_poll(clib_ultra_t *u, float *cm_out);

#ifdef __cplusplus
}
#endif
#endif /* CLIB_ULTRASONIC_H */
