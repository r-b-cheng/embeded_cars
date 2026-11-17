#include "clib_ir_remote.h"
#include "clib_port.h"

/* 案例 4 /User/main.c 的按键表：
 * buf[0]=data_code/16, buf[1]=data_code%16 */
const clib_ir_key_t CLIB_IR_KEYMAP_V1[] = {
    {  6u,  2u, CLIB_ACT_FORWARD,    "前进" },
    { 10u,  8u, CLIB_ACT_BACKWARD,   "后退" },
    {  2u,  2u, CLIB_ACT_SPIN_LEFT,  "左旋" },
    { 12u,  2u, CLIB_ACT_SPIN_RIGHT, "右旋" },
    {  0u,  2u, CLIB_ACT_STOP,       "停止" }
};
const uint8_t CLIB_IR_KEYMAP_V1_LEN =
    (uint8_t)(sizeof(CLIB_IR_KEYMAP_V1) / sizeof(CLIB_IR_KEYMAP_V1[0]));

/* 案例 6 /User/main.c 的按键表（同一套硬件、不同的遥控器/码表） */
const clib_ir_key_t CLIB_IR_KEYMAP_V2[] = {
    { 11u,  1u, CLIB_ACT_FORWARD,    "前进" },
    { 13u,  4u, CLIB_ACT_BACKWARD,   "后退" },
    {  9u,  1u, CLIB_ACT_SPIN_LEFT,  "左旋" },
    { 14u,  1u, CLIB_ACT_SPIN_RIGHT, "右旋" },
    {  8u,  1u, CLIB_ACT_STOP,       "停止" }
};
const uint8_t CLIB_IR_KEYMAP_V2_LEN =
    (uint8_t)(sizeof(CLIB_IR_KEYMAP_V2) / sizeof(CLIB_IR_KEYMAP_V2[0]));

void clib_irdec_reset(clib_irdec_t *d)
{
    if (d == NULL) {
        return;
    }
    d->state = CLIB_IRDEC_SEEK_LEADER;
    d->code = 0u;
    d->bits = 0u;
}

void clib_irdec_init(clib_irdec_t *d)
{
    if (d == NULL) {
        return;
    }
    d->frames_ok = 0u;
    d->frames_err = 0u;
    clib_irdec_reset(d);
}

int clib_irdec_feed_tick(clib_irdec_t *d, uint8_t ticks20us, uint32_t *code_out)
{
    uint8_t bit;

    if (d == NULL) {
        return CLIB_ERR_NULL;
    }
    if (ticks20us >= CLIB_IR_TIMEOUT_TICKS) {
        /* 长空闲 / 无效信号：复位状态机，不产生半帧 */
        d->frames_err++;
        clib_irdec_reset(d);
        return CLIB_ERR_TIMEOUT;
    }

    if (d->state == CLIB_IRDEC_SEEK_LEADER) {
        if (ticks20us >= CLIB_IR_LEADER_MIN && ticks20us < CLIB_IR_LEADER_MAX) {
            d->state = CLIB_IRDEC_DATA;
            d->code = 0u;
            d->bits = 0u;
        }
        return 0;
    }

    /* 数据位：只接受落在合法区间的脉宽，无效脉宽直接忽略（原厂会沿用上次 bit） */
    if (ticks20us >= CLIB_IR_BIT1_MIN && ticks20us < CLIB_IR_BIT1_MAX) {
        bit = 1u;
    } else if (ticks20us >= CLIB_IR_BIT0_MIN && ticks20us < CLIB_IR_BIT0_MAX) {
        bit = 0u;
    } else {
        return 0;
    }

    d->code = (d->code << 1) | bit;
    d->bits++;
    if (d->bits >= CLIB_IR_DATA_BITS) {
        if (code_out != NULL) {
            *code_out = d->code;
        }
        d->frames_ok++;
        d->state = CLIB_IRDEC_DONE;
        clib_irdec_reset(d);       /* 立刻准备好接收下一帧 */
        return 1;
    }
    return 0;
}

int clib_irdec_feed_pulse(clib_irdec_t *d, uint32_t high_us, uint32_t *code_out)
{
    uint32_t ticks = high_us / CLIB_IR_TICK_US;
    if (ticks > 255u) {
        ticks = 255u;
    }
    return clib_irdec_feed_tick(d, (uint8_t)ticks, code_out);
}

void clib_ir_sampler_init(clib_ir_sampler_t *s)
{
    if (s == NULL) {
        return;
    }
    clib_irdec_init(&s->dec);
    s->last_level = 0;
    s->rise_us = 0u;
    s->measuring = false;
}

int clib_ir_sampler_poll(clib_ir_sampler_t *s, uint32_t now_us, uint32_t *code_out)
{
    int level;
    uint32_t pulse;

    if (s == NULL) {
        return CLIB_ERR_NULL;
    }
    level = clib_gpio_read(CLIB_PIN_IR_REMOTE);
    if (level < 0) {
        return CLIB_ERR_IO;   /* 归一化 port 层的裸负错误码 */
    }
    if (level == 1) {
        if (!s->measuring) {
            s->rise_us = now_us;
            s->measuring = true;
        }
        s->last_level = level;
        return 0;
    }
    if (s->measuring) {
        s->measuring = false;
        pulse = now_us - s->rise_us;
        s->last_level = level;
        return clib_irdec_feed_pulse(&s->dec, pulse, code_out);
    }
    s->last_level = level;
    return 0;
}

int clib_ir_sampler_poll_now(clib_ir_sampler_t *s, uint32_t *code_out)
{
    return clib_ir_sampler_poll(s, clib_micros(), code_out);
}

uint8_t clib_ir_data_code(uint32_t raw_code)
{
    /* 原代码：data_code = IR_Receivecode >> 8; 赋给 uint8_t 后只保留低 8 位 */
    return (uint8_t)((raw_code >> 8) & 0xFFu);
}

void clib_ir_split_key(uint8_t data_code, uint8_t *hi, uint8_t *lo)
{
    if (hi != NULL) { *hi = (uint8_t)(data_code >> 4); }
    if (lo != NULL) { *lo = (uint8_t)(data_code & 0x0Fu); }
}

clib_act_t clib_ir_lookup(uint32_t raw_code, const clib_ir_key_t *map, uint8_t n)
{
    uint8_t dc;
    uint8_t hi;
    uint8_t lo;
    uint8_t i;

    if (map == NULL) {
        return CLIB_ACT_STOP;
    }
    dc = clib_ir_data_code(raw_code);
    clib_ir_split_key(dc, &hi, &lo);
    for (i = 0u; i < n; ++i) {
        if (map[i].hi == hi && map[i].lo == lo) {
            return map[i].act;
        }
    }
    return CLIB_ACT_STOP;   /* 未知键一律停车，安全优先 */
}
