#include "clib_ultrasonic.h"
#include "clib_port.h"

/*
 * 关于原厂换算公式（如实说明，不做臆断）：
 *   UltrasonicWave.c:65   UltrasonicWave_Distance = TIM_GetCounter(TIM2)*5*34/200.0
 *                        = cnt * 0.85
 *   UltrasonicWave.c:81   return UltrasonicWave_Distance * 10  = cnt * 8.5
 *   timer.c                TIM_Period=5000, TIM_Prescaler=7199
 *                        → TIM2 时钟 72MHz/7200 = 10kHz，1 计数 ≈ 100us
 *                        （案例 9 的 User/main.c 注释也写的是「10Khz 的计数频率」）
 *
 *   按这个时基，物理正确的距离应是 cnt*100*0.0343/2 ≈ cnt*1.715 cm，
 *   而原公式给 cnt*0.85，约为其一半；后面又 *10 变成 cnt*8.5，
 *   案例里阈值 60 判的其实是这个量纲不明的数，不是 60cm。
 *   公式里的常数 5 与 100us 的实际时基对不上，量纲混乱，无法可靠还原。
 *
 * 所以库按标准物理公式重算（cm = us * 0.0343 / 2），
 * 另保留两个 legacy 函数，供需要复现原案例行为时对照，但调用方须自行确认单位。
 */
float clib_echo_us_to_cm(uint32_t pulse_us)
{
    float cm = (float)pulse_us * CLIB_SOUND_CM_PER_US / 2.0f;
    if (cm > CLIB_ULTRA_MAX_CM) { cm = CLIB_ULTRA_MAX_CM; }
    return cm;
}

uint16_t clib_echo_us_to_mm(uint32_t pulse_us)
{
    float mm = clib_echo_us_to_cm(pulse_us) * 10.0f;
    if (mm > 65535.0f) { mm = 65535.0f; }
    return (uint16_t)(mm + 0.5f);
}

float clib_echo_count_to_cm(uint32_t count, uint32_t us_per_count)
{
    uint64_t pulse_us;
    if (us_per_count == 0u) {
        return 0.0f;
    }
    /* 用 64 位整数做乘法并显式饱和，避免 float->uint32 溢出时的未定义行为 */
    pulse_us = (uint64_t)count * (uint64_t)us_per_count;
    if (pulse_us > (uint64_t)UINT32_MAX) {
        pulse_us = (uint64_t)UINT32_MAX;
    }
    return clib_echo_us_to_cm((uint32_t)pulse_us);
}

float clib_echo_count_to_legacy(uint32_t count)
{
    /* UltrasonicWave_Distance = cnt*5*34/200.0 */
    return (float)count * 5.0f * 34.0f / 200.0f;
}

float clib_echo_legacy_measure(uint32_t count)
{
    /* UltrasonicWave_StartMeasure() 的返回值 = Distance*10 */
    return clib_echo_count_to_legacy(count) * 10.0f;
}

int clib_ultra_trigger(void)
{
    if (!clib_is_ready()) {
        return CLIB_ERR_NOT_INIT;
    }
    /* 发送控制消息：>10us 的高电平，原代码用 20us */
    clib_gpio_write(CLIB_PIN_ULTRA_TRIG, 1);
    clib_delay_us(CLIB_ULTRA_TRIG_US);
    clib_gpio_write(CLIB_PIN_ULTRA_TRIG, 0);
    return CLIB_OK;
}

void clib_ultra_start(clib_ultra_t *u, uint32_t timeout_us)
{
    if (u == NULL) {
        return;
    }
    u->state = CLIB_ULTRA_WAIT_RISE;
    u->t_start = clib_micros();
    u->rise_us = 0u;
    u->timeout_us = (timeout_us == 0u) ? CLIB_ULTRA_DEFAULT_TIMEOUT_US : timeout_us;
    u->last_cm = 0.0f;
    (void)clib_ultra_trigger();
}

int clib_ultra_poll_at(clib_ultra_t *u, uint32_t now_us, float *cm_out)
{
    int level;

    if (u == NULL) {
        return CLIB_ERR_NULL;
    }
    if (u->state == CLIB_ULTRA_IDLE) {
        return CLIB_ERR_NOT_INIT;      /* 还没 start */
    }
    if (u->state == CLIB_ULTRA_DONE) {
        return CLIB_ERR_BUSY;          /* 已测完，需重新 start */
    }
    if (u->state == CLIB_ULTRA_TIMEOUT) {
        return CLIB_ERR_TIMEOUT;
    }
    if ((uint32_t)(now_us - u->t_start) > u->timeout_us) {
        u->state = CLIB_ULTRA_TIMEOUT;
        return CLIB_ERR_TIMEOUT;
    }

    level = clib_gpio_read(CLIB_PIN_ULTRA_ECHO);
    if (level < 0) {
        return CLIB_ERR_IO;            /* 归一化 port 层的裸负错误码 */
    }

    if (u->state == CLIB_ULTRA_WAIT_RISE) {
        if (level == 1) {
            u->rise_us = now_us;
            u->state = CLIB_ULTRA_WAIT_FALL;
        }
        return 0;
    }

    /* WAIT_FALL */
    if (level == 0) {
        u->last_cm = clib_echo_us_to_cm((uint32_t)(now_us - u->rise_us));
        u->state = CLIB_ULTRA_DONE;
        if (cm_out != NULL) {
            *cm_out = u->last_cm;
        }
        return 1;
    }
    return 0;
}

int clib_ultra_poll(clib_ultra_t *u, float *cm_out)
{
    return clib_ultra_poll_at(u, clib_micros(), cm_out);
}
