#include "clib_ir_sensor.h"
#include "clib_port.h"

int clib_ir_sensor_init(clib_ir_sensor_t *s,
                        uint8_t pin_left, uint8_t pin_right,
                        bool active_low, uint16_t debounce_ms)
{
    if (s == NULL) {
        return CLIB_ERR_NULL;
    }
    if (pin_left >= (uint8_t)CLIB_PIN_COUNT || pin_right >= (uint8_t)CLIB_PIN_COUNT) {
        return CLIB_ERR_PARAM;
    }
    s->pin_left = pin_left;
    s->pin_right = pin_right;
    s->active_low = active_low;
    s->debounce_ms = debounce_ms;
    s->stable = CLIB_SENS_NONE;
    s->candidate = CLIB_SENS_NONE;
    s->t_change = 0u;
    s->has_candidate = false;
    return CLIB_OK;
}

static uint8_t read_one(const clib_ir_sensor_t *s, uint8_t pin)
{
    int v = clib_gpio_read((clib_pin_t)pin);
    if (v < 0) {
        return 0u;   /* 读失败按"未检测到"处理，安全优先 */
    }
    if (s->active_low) {
        return (v == 0) ? 1u : 0u;
    }
    return (v != 0) ? 1u : 0u;
}

uint8_t clib_ir_sensor_read_raw(clib_ir_sensor_t *s)
{
    uint8_t l;
    uint8_t r;

    if (s == NULL) {
        return CLIB_SENS_NONE;
    }
    l = read_one(s, s->pin_left);
    r = read_one(s, s->pin_right);
    return (uint8_t)((l << 1) | r);
}

uint8_t clib_ir_sensor_read_at(clib_ir_sensor_t *s, uint32_t now_ms)
{
    uint8_t raw;

    if (s == NULL) {
        return CLIB_SENS_NONE;
    }
    raw = clib_ir_sensor_read_raw(s);
    if (!s->has_candidate || raw != s->candidate) {
        s->candidate = raw;
        s->t_change = now_ms;
        s->has_candidate = true;
    } else if ((uint32_t)(now_ms - s->t_change) >= (uint32_t)s->debounce_ms) {
        s->stable = raw;
    }
    return s->stable;
}

uint8_t clib_ir_sensor_read(clib_ir_sensor_t *s)
{
    return clib_ir_sensor_read_at(s, clib_millis());
}

/* 案例 6 Robot_Traction()：
 *   (L,R)=(0,0) 前进；(1,0) 左修正；(0,1) 右修正；(1,1) 停止 */
clib_act_t clib_track_decide(uint8_t state)
{
    switch (state & CLIB_SENS_BOTH) {
    case CLIB_SENS_NONE:          return CLIB_ACT_FORWARD;
    case CLIB_SENS_LEFT_BIT:      return CLIB_ACT_LEFT;
    case CLIB_SENS_RIGHT_BIT:     return CLIB_ACT_RIGHT;
    case CLIB_SENS_BOTH:          return CLIB_ACT_STOP;
    default:                      return CLIB_ACT_STOP;
    }
}

/* 案例 6 Robot_Avoidance()：
 *   (L,R)=(1,1) 前进；(1,0) 左转；(0,1) 右转；(0,0) 刹车后原地右旋
 * 这里把最后一种合并成 SPIN_RIGHT（持续输出），由调用方决定持续多久，
 * 避免原代码 Delay_ms(400)+Delay_ms(900) 的阻塞。 */
clib_act_t clib_avoid_decide(uint8_t state)
{
    switch (state & CLIB_SENS_BOTH) {
    case CLIB_SENS_BOTH:          return CLIB_ACT_FORWARD;
    case CLIB_SENS_LEFT_BIT:      return CLIB_ACT_LEFT;
    case CLIB_SENS_RIGHT_BIT:     return CLIB_ACT_RIGHT;
    case CLIB_SENS_NONE:          return CLIB_ACT_SPIN_RIGHT;
    default:                      return CLIB_ACT_STOP;
    }
}
