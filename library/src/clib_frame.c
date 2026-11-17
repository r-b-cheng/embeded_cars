#include "clib_frame.h"
#include "clib_port.h"
#include <string.h>

static uint8_t cs_len(clib_checksum_t mode)
{
    return (mode == CLIB_CHECKSUM_NONE) ? 0u : 1u;
}

uint8_t clib_checksum_compute(clib_checksum_t mode, const uint8_t *data, uint8_t len)
{
    uint8_t i;
    uint8_t acc = 0u;

    if (data == NULL || mode == CLIB_CHECKSUM_NONE) {
        return 0u;
    }
    for (i = 0u; i < len; ++i) {
        if (mode == CLIB_CHECKSUM_XOR) {
            acc ^= data[i];
        } else {
            acc = (uint8_t)(acc + data[i]);
        }
    }
    return acc;
}

/* 注意：reset 不能清 cfg —— 收满一帧后还要靠它继续解析下一帧。
   调用方必须先 clib_frame_init()；未初始化的对象不要调用本函数。 */
void clib_frame_reset(clib_frame_t *f)
{
    if (f == NULL) {
        return;
    }
    f->state = 0u;
    f->head_pos = 0u;
    f->len = 0u;
    f->last_ms = clib_millis();
    memset(f->buf, 0, sizeof(f->buf));
}

int clib_frame_init(clib_frame_t *f, const clib_frame_cfg_t *cfg, uint32_t timeout_ms)
{
    if (f == NULL || cfg == NULL) {
        return CLIB_ERR_NULL;
    }
    if (cfg->head_len > CLIB_FRAME_HEAD_MAX || cfg->tail_len > CLIB_FRAME_TAIL_MAX ||
        cfg->max_payload == 0u || cfg->max_payload > CLIB_FRAME_PAYLOAD_MAX) {
        return CLIB_ERR_PARAM;
    }
    if (cfg->checksum != CLIB_CHECKSUM_NONE &&
        cfg->checksum != CLIB_CHECKSUM_XOR &&
        cfg->checksum != CLIB_CHECKSUM_SUM8) {
        return CLIB_ERR_PARAM;
    }
    /* 必须至少有一种判帧依据：定长 或 帧尾 */
    if (cfg->fixed_len == 0u && cfg->tail_len == 0u) {
        return CLIB_ERR_PARAM;
    }
    if (cfg->fixed_len > 0u) {
        uint16_t min_len = (uint16_t)cfg->head_len + (uint16_t)cfg->tail_len +
                           (uint16_t)cs_len(cfg->checksum) + 1u;
        if (cfg->fixed_len < min_len ||
            cfg->fixed_len > (uint16_t)(cfg->head_len + CLIB_FRAME_BUF_MAX)) {
            return CLIB_ERR_PARAM;
        }
    }

    memset(f, 0, sizeof(*f));
    f->cfg = cfg;
    f->timeout_ms = timeout_ms;
    clib_frame_reset(f);
    return CLIB_OK;
}

int clib_frame_build(const clib_frame_cfg_t *cfg,
                     const uint8_t *payload, uint8_t payload_len,
                     uint8_t *out, uint16_t out_cap)
{
    uint16_t total;
    uint16_t body;
    uint16_t pos = 0u;
    uint8_t  cl  = 0u;
    uint8_t  i;

    if (cfg == NULL || out == NULL || (payload == NULL && payload_len > 0u)) {
        return CLIB_ERR_NULL;
    }
    if (payload_len > cfg->max_payload) {
        return CLIB_ERR_PARAM;
    }
    cl = cs_len(cfg->checksum);
    /* buf 只存「帧头之后」的部分，比较容量时不能把 head 算进来 */
    body = (uint16_t)payload_len + (uint16_t)cl + (uint16_t)cfg->tail_len;
    total = (uint16_t)cfg->head_len + body;
    if (cfg->fixed_len > 0u && total != (uint16_t)cfg->fixed_len) {
        return CLIB_ERR_PARAM;
    }
    if (body > (uint16_t)CLIB_FRAME_BUF_MAX || total > out_cap) {
        return CLIB_ERR_OVERFLOW;
    }

    for (i = 0u; i < cfg->head_len; ++i) { out[pos++] = cfg->head[i]; }
    for (i = 0u; i < payload_len; ++i)   { out[pos++] = payload[i]; }
    if (cl > 0u) {
        out[pos++] = clib_checksum_compute(cfg->checksum, payload, payload_len);
    }
    for (i = 0u; i < cfg->tail_len; ++i) { out[pos++] = cfg->tail[i]; }
    return (int)pos;
}

int clib_frame_send(uint8_t uart_id, const clib_frame_cfg_t *cfg,
                    const uint8_t *payload, uint8_t payload_len)
{
    uint8_t frame[CLIB_FRAME_BUF_MAX];
    int built = clib_frame_build(cfg, payload, payload_len, frame, (uint16_t)sizeof(frame));
    int sent;

    if (built < 0) {
        return built;
    }
    sent = clib_uart_write(uart_id, frame, (uint16_t)built);
    if (sent < 0) {
        return sent;    /* 保留底层错误码，别一律吞成 IO 错误 */
    }
    return (sent == built) ? CLIB_OK : CLIB_ERR_IO;
}

/* 判断 buf 末尾是否恰好是 tail 序列 */
static bool tail_matched(const uint8_t *buf, uint8_t len,
                         const uint8_t *tail, uint8_t tail_len)
{
    uint8_t i;
    if (tail_len == 0u || len < tail_len) {
        return false;
    }
    for (i = 0u; i < tail_len; ++i) {
        if (buf[len - tail_len + i] != tail[i]) {
            return false;
        }
    }
    return true;
}

int clib_frame_feed(clib_frame_t *f, uint8_t byte,
                    uint8_t *payload_out, uint8_t payload_cap, uint8_t *payload_len_out)
{
    const clib_frame_cfg_t *cfg;
    uint8_t  cl;
    int      body_len;
    uint8_t  i;
    bool     active;

    if (f == NULL || f->cfg == NULL) {
        return CLIB_ERR_NOT_INIT;
    }
    cfg = f->cfg;
    cl  = cs_len(cfg->checksum);

    /* 超时自动复位：把「正在匹配帧头」也算作活动状态，避免半帧卡死 */
    active = (f->state != 0u || f->head_pos != 0u || f->len > 0u);
    if (f->timeout_ms > 0u && active &&
        (uint32_t)(clib_millis() - f->last_ms) > f->timeout_ms) {
        clib_frame_reset(f);
    }

    if (f->state == 0u) {
        if (cfg->head_len == 0u) {
            f->state = 1u;
            /* 落到下面的正文处理，不消耗掉这个字节 */
        } else {
            if (byte == cfg->head[f->head_pos]) {
                f->head_pos++;
                if (f->head_pos == cfg->head_len) {
                    f->state = 1u;
                    f->head_pos = 0u;
                    f->len = 0u;
                }
            } else {
                /* 支持 AA AA AE 这类重叠帧头：从头第一个字节重新尝试 */
                f->head_pos = (byte == cfg->head[0]) ? 1u : 0u;
            }
            f->last_ms = clib_millis();
            return 0;
        }
    }

    /* ---- 正文 ---- */
    if (f->len >= CLIB_FRAME_BUF_MAX) {
        f->frames_err++;
        clib_frame_reset(f);
        return CLIB_ERR_OVERFLOW;
    }
    f->buf[f->len++] = byte;
    f->last_ms = clib_millis();

    {
        bool complete = false;
        if (cfg->fixed_len > 0u) {
            complete = ((uint16_t)cfg->head_len + (uint16_t)f->len) >= (uint16_t)cfg->fixed_len;
        } else {
            complete = tail_matched(f->buf, f->len, cfg->tail, cfg->tail_len);
        }
        if (!complete) {
            /* 变长帧若正文已超过上限，及时复位，别无限攒 */
            if (cfg->fixed_len == 0u &&
                f->len > (uint8_t)(cfg->max_payload + cfg->tail_len + cl)) {
                f->frames_err++;
                clib_frame_reset(f);
                return CLIB_ERR_OVERFLOW;
            }
            return 0;
        }
    }

    /* 拆出 payload：buf = payload + [checksum] + [tail] */
    body_len = (int)f->len - (int)cfg->tail_len - (int)cl;
    if (body_len < 0 || body_len > (int)cfg->max_payload) {
        f->frames_err++;
        clib_frame_reset(f);
        return CLIB_ERR_FORMAT;
    }
    /* ★ 写调用方缓冲之前先检查容量，杜绝溢出。
       payload_out 允许为 NULL（只要帧到达通知、不取数据）。 */
    if (payload_out != NULL && (uint8_t)body_len > payload_cap) {
        f->frames_err++;
        clib_frame_reset(f);
        return CLIB_ERR_OVERFLOW;
    }
    if (cl > 0u) {
        uint8_t expect = clib_checksum_compute(cfg->checksum, f->buf, (uint8_t)body_len);
        if (f->buf[body_len] != expect) {
            f->frames_err++;
            clib_frame_reset(f);
            return CLIB_ERR_CHECKSUM;
        }
    }
    if (cfg->tail_len > 0u &&
        !tail_matched(f->buf, f->len, cfg->tail, cfg->tail_len)) {
        f->frames_err++;
        clib_frame_reset(f);
        return CLIB_ERR_FORMAT;
    }

    if (payload_out != NULL) {
        for (i = 0u; i < (uint8_t)body_len; ++i) {
            payload_out[i] = f->buf[i];
        }
    }
    if (payload_len_out != NULL) {
        *payload_len_out = (uint8_t)body_len;
    }
    f->frames_ok++;
    clib_frame_reset(f);
    return 1;
}

int clib_frame_feed_buf(clib_frame_t *f, const uint8_t *bytes, uint16_t n,
                        uint8_t *payload_out, uint8_t payload_cap, uint8_t *payload_len_out)
{
    uint16_t i;
    int r;

    if (f == NULL || bytes == NULL) {
        return CLIB_ERR_NULL;
    }
    for (i = 0u; i < n; ++i) {
        r = clib_frame_feed(f, bytes[i], payload_out, payload_cap, payload_len_out);
        /* 只返回第一个完整帧；剩余字节请分帧喂入，避免静默丢帧 */
        if (r != 0) {
            return r;
        }
    }
    return 0;
}

int clib_frame_poll(uint8_t uart_id, clib_frame_t *f,
                    uint8_t *payload_out, uint8_t payload_cap, uint8_t *payload_len_out)
{
    uint8_t  b = 0u;
    uint16_t budget = CLIB_FRAME_POLL_BUDGET;
    int n;
    int r;

    if (f == NULL || f->cfg == NULL) {
        return CLIB_ERR_NOT_INIT;
    }
    /* ★ 单次调用有字节预算：串口持续来噪声时也能按时把 CPU 还给控制环 */
    while (budget-- > 0u) {
        n = clib_uart_read(uart_id, &b, 1u);
        if (n < 0) {
            return n;       /* 不吞错误码 */
        }
        if (n == 0) {
            return 0;       /* 串口暂无数据 */
        }
        r = clib_frame_feed(f, b, payload_out, payload_cap, payload_len_out);
        if (r != 0) {
            return r;       /* 收满一帧或出错，剩余字节留到下次 poll */
        }
    }
    return 0;
}
