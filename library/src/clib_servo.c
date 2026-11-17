#include "clib_servo.h"
#include "clib_port.h"

const clib_servo_cal_t CLIB_SERVO_CAL_DEFAULT = {
    CLIB_SERVO_MIN_US, CLIB_SERVO_MAX_US,
    CLIB_SERVO_MIN_DEG, CLIB_SERVO_MAX_DEG
};

uint16_t clib_servo_clamp_pulse(uint16_t pulse_us)
{
    if (pulse_us < CLIB_SERVO_MIN_US) { return CLIB_SERVO_MIN_US; }
    if (pulse_us > CLIB_SERVO_MAX_US) { return CLIB_SERVO_MAX_US; }
    return pulse_us;
}

/* 原厂公式：angle / 180 * 2000 + 500，这里参数化成标定表 */
uint16_t clib_servo_angle_to_pulse_cal(float angle, const clib_servo_cal_t *cal)
{
    float span_deg;
    float span_us;
    float pulse;

    if (cal == NULL) {
        cal = &CLIB_SERVO_CAL_DEFAULT;
    }
    if (angle < cal->min_deg) { angle = cal->min_deg; }
    if (angle > cal->max_deg) { angle = cal->max_deg; }
    span_deg = cal->max_deg - cal->min_deg;
    span_us  = (float)(cal->max_us - cal->min_us);
    if (span_deg <= 0.0f) {
        return clib_servo_clamp_pulse(cal->min_us);
    }
    pulse = (float)cal->min_us + (angle - cal->min_deg) * span_us / span_deg;
    /* 尊重自定义标定：钳到 cal 声明的行程范围（而不是固定的 500~2500）。
       注意 clib_servo_set_pulse() 仍会做一次 500~2500us 的全局安全钳位，
       所以自定义 cal 的 min_us/max_us 建议落在 500~2500 之内。 */
    if (pulse < (float)cal->min_us) { pulse = (float)cal->min_us; }
    if (pulse > (float)cal->max_us) { pulse = (float)cal->max_us; }
    /* 原厂 Servo_SetAngle() 是 float -> uint16_t 截断；这里用四舍五入，
       非整数角度最多与原实现差 1us（更精确，但不是逐位等价）。 */
    return (uint16_t)(pulse + 0.5f);
}

uint16_t clib_servo_angle_to_pulse(float angle)
{
    return clib_servo_angle_to_pulse_cal(angle, &CLIB_SERVO_CAL_DEFAULT);
}

float clib_servo_pulse_to_angle(uint16_t pulse_us, const clib_servo_cal_t *cal)
{
    float span_deg;
    float span_us;

    if (cal == NULL) {
        cal = &CLIB_SERVO_CAL_DEFAULT;
    }
    span_deg = cal->max_deg - cal->min_deg;
    span_us  = (float)(cal->max_us - cal->min_us);
    if (span_us <= 0.0f) {
        return cal->min_deg;
    }
    return cal->min_deg + (float)(pulse_us - cal->min_us) * span_deg / span_us;
}

int clib_servo_set_pulse(uint16_t pulse_us)
{
    if (!clib_is_ready()) {
        return CLIB_ERR_NOT_INIT;
    }
    clib_servo_hw_set_pulse(clib_servo_clamp_pulse(pulse_us));
    return CLIB_OK;
}

int clib_servo_set_angle(float angle)
{
    return clib_servo_set_pulse(clib_servo_angle_to_pulse(angle));
}

int clib_servo_set_angle_cal(float angle, const clib_servo_cal_t *cal)
{
    return clib_servo_set_pulse(clib_servo_angle_to_pulse_cal(angle, cal));
}

int clib_servo_scan_begin(clib_servo_scan_t *s, const float *angles,
                          uint8_t count, uint16_t settle_ms)
{
    int r;
    if (s == NULL || angles == NULL) {
        return CLIB_ERR_NULL;
    }
    if (count == 0u || count > CLIB_SERVO_SCAN_MAX || settle_ms == 0u) {
        return CLIB_ERR_PARAM;
    }
    if (!clib_is_ready()) {
        return CLIB_ERR_NOT_INIT;
    }
    s->angles = angles;
    s->count = count;
    s->index = 0u;
    s->settle_ms = settle_ms;
    s->first_pending = true;
    s->running = true;
    s->done = false;
    s->t_switch = clib_millis();
    r = clib_servo_set_angle(angles[0]);
    if (r != CLIB_OK) {
        s->running = false;
        return r;
    }
    return CLIB_OK;
}

int clib_servo_scan_tick(clib_servo_scan_t *s, float *angle_out)
{
    if (s == NULL || !s->running || s->done) {
        return CLIB_SERVO_SCAN_FINISHED;
    }
    if (s->first_pending) {
        s->first_pending = false;
        if (angle_out != NULL) {
            *angle_out = s->angles[0];
        }
        return 1;   /* 第一个角度已就位，调用方可以布置该方向的采集 */
    }
    if ((uint32_t)(clib_millis() - s->t_switch) < (uint32_t)s->settle_ms) {
        return 0;   /* 还在稳定 */
    }
    s->index++;
    if (s->index >= s->count) {
        s->done = true;
        s->running = false;
        return CLIB_SERVO_SCAN_FINISHED;
    }
    (void)clib_servo_set_angle(s->angles[s->index]);
    s->t_switch = clib_millis();
    if (angle_out != NULL) {
        *angle_out = s->angles[s->index];
    }
    return 1;
}

bool clib_servo_scan_done(const clib_servo_scan_t *s)
{
    return (s == NULL) ? true : s->done;
}

void clib_servo_scan_abort(clib_servo_scan_t *s)
{
    if (s != NULL) {
        s->running = false;
        s->done = true;
        s->first_pending = false;
    }
}
