#include "clib_vision_k210.h"
#include "clib_port.h"

void clib_k210_reset(clib_k210_t *k)
{
    uint8_t i;
    if (k == NULL) {
        return;
    }
    k->state = 0u;
    k->pos = 0u;
    for (i = 0u; i < CLIB_K210_FRAME_LEN; ++i) {
        k->buf[i] = 0u;
    }
}

void clib_k210_init(clib_k210_t *k)
{
    if (k == NULL) {
        return;
    }
    k->frames_ok = 0u;
    k->frames_err = 0u;
    clib_k210_reset(k);
}

int clib_k210_build(int16_t angle, uint8_t out[CLIB_K210_FRAME_LEN])
{
    uint16_t u;
    if (out == NULL) {
        return CLIB_ERR_NULL;
    }
    u = (uint16_t)angle;    /* 有符号转无符号，避免实现定义的移位行为 */
    out[0] = CLIB_K210_HEAD0;
    out[1] = CLIB_K210_HEAD1;
    out[2] = (uint8_t)(u & 0xFFu);
    out[3] = (uint8_t)((u >> 8) & 0xFFu);
    out[4] = CLIB_K210_TAIL;
    return (int)CLIB_K210_FRAME_LEN;
}

int clib_k210_feed(clib_k210_t *k, uint8_t byte, int16_t *angle_out)
{
    uint16_t u;

    if (k == NULL) {
        return CLIB_ERR_NULL;
    }

    if (k->state == 0u) {
        if (byte == CLIB_K210_HEAD0) {
            k->buf[0] = byte;
            k->pos = 1u;
            k->state = 1u;
        }
        return 0;
    }

    if (k->state == 1u) {
        if (byte == CLIB_K210_HEAD1) {
            k->buf[1] = byte;
            k->pos = 2u;
            k->state = 2u;
        } else if (byte == CLIB_K210_HEAD0) {
            /* AA AA AE 的情况：保持在找头1，但pos复位 */
            k->buf[0] = byte;
            k->pos = 1u;
        } else {
            k->frames_err++;
            clib_k210_reset(k);
        }
        return 0;
    }

    /* state == 2：严格按 5 字节定长收，杜绝原代码的数组越界 */
    if (k->pos >= CLIB_K210_FRAME_LEN) {
        k->frames_err++;
        clib_k210_reset(k);
        return CLIB_ERR_OVERFLOW;
    }
    k->buf[k->pos++] = byte;

    if (k->pos < CLIB_K210_FRAME_LEN) {
        return 0;
    }

    if (k->buf[4] != CLIB_K210_TAIL) {
        k->frames_err++;
        clib_k210_reset(k);
        return CLIB_ERR_FORMAT;
    }
    u = (uint16_t)k->buf[2] | (uint16_t)((uint16_t)k->buf[3] << 8);
    if (angle_out != NULL) {
        /* uint16 -> int16 的直接转换是实现定义行为（C99 6.3.1.3p3），
           这里显式做二补数解释，保证跨平台一致 */
        int32_t signed_val = (u & 0x8000u) ? ((int32_t)u - 65536) : (int32_t)u;
        *angle_out = (int16_t)signed_val;
    }
    k->frames_ok++;
    clib_k210_reset(k);
    return 1;
}

int clib_k210_poll(uint8_t uart_id, clib_k210_t *k, int16_t *angle_out)
{
    uint8_t  b = 0u;
    uint16_t budget = CLIB_K210_POLL_BUDGET;
    int n;
    int r;

    if (k == NULL) {
        return CLIB_ERR_NULL;
    }
    /* ★ 单次调用有字节预算，避免串口持续来数据（或全是噪声）时饿死控制环 */
    while (budget-- > 0u) {
        n = clib_uart_read(uart_id, &b, 1u);
        if (n < 0) {
            return n;       /* 不吞底层错误码 */
        }
        if (n == 0) {
            return 0;       /* 串口暂无数据 */
        }
        r = clib_k210_feed(k, b, angle_out);
        if (r != 0) {
            return r;
        }
    }
    return 0;
}

bool clib_k210_is_lost(int16_t angle)
{
    return (angle == CLIB_K210_LOST_ANGLE);
}

clib_act_t clib_k210_decide(int16_t angle, int16_t deadzone)
{
    if (clib_k210_is_lost(angle)) {
        return CLIB_ACT_BACKWARD;   /* 原代码 makerobo_back(50,10) */
    }
    if (angle == 0) {
        return CLIB_ACT_STOP;
    }
    if (angle > deadzone) {
        return CLIB_ACT_RIGHT;
    }
    if (angle < -deadzone) {
        return CLIB_ACT_LEFT;
    }
    return CLIB_ACT_FORWARD;
}
