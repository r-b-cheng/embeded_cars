#ifndef CLIB_FRAME_H
#define CLIB_FRAME_H
/*
 * clib_frame.h -- 通用串口帧收发（跨平台）
 *
 * 覆盖两类「和外设通信」的通用需求：
 *   1) 向外设发送控制消息：clib_frame_build() 组帧 / clib_frame_send() 直接发到串口
 *   2) 接收外设送来的数据：clib_frame_feed() 逐字节喂状态机 / clib_frame_poll() 从串口批量灌入
 *
 * 支持：定长帧（如 K210 的 5 字节）、带帧尾的变长帧（如蓝牙 "\r\n" 结尾）、
 *      可选 XOR / SUM8 校验、超时自动复位、帧头重叠（AA AA AE）不丢帧。
 * 原厂 Uart3.c 依赖 STM32 的 IDLE 中断判帧，且 buff[100] 无越界保护；
 * 这里改成纯软件状态机，不依赖任何芯片中断，TM4C 上也能直接用。
 */
#include "clib_common.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CLIB_FRAME_HEAD_MAX      4u
#define CLIB_FRAME_TAIL_MAX      4u
#define CLIB_FRAME_PAYLOAD_MAX  64u
#define CLIB_FRAME_BUF_MAX      (CLIB_FRAME_PAYLOAD_MAX + CLIB_FRAME_TAIL_MAX + 1u)
#define CLIB_FRAME_POLL_BUDGET  16u   /* 单次 poll 最多处理的字节数，避免串口持续来数据饿死控制环 */

typedef enum {
    CLIB_CHECKSUM_NONE = 0,
    CLIB_CHECKSUM_XOR,
    CLIB_CHECKSUM_SUM8
} clib_checksum_t;

/* cfg 必须是 static const 或生命周期长于 frame 实例（frame 只保存指针，不拷贝） */
typedef struct {
    uint8_t head[CLIB_FRAME_HEAD_MAX];
    uint8_t head_len;        /* 0 = 无帧头 */
    uint8_t tail[CLIB_FRAME_TAIL_MAX];
    uint8_t tail_len;        /* 0 = 无帧尾 */
    uint8_t max_payload;     /* 负载上限，不含头/校验/尾 */
    uint8_t fixed_len;       /* >0 = 整帧定长（含头尾）；0 = 靠帧尾或超时判帧 */
    clib_checksum_t checksum;
} clib_frame_cfg_t;

typedef struct {
    const clib_frame_cfg_t *cfg;
    uint8_t  state;          /* 0 = 找帧头, 1 = 收正文 */
    uint8_t  head_pos;
    uint8_t  buf[CLIB_FRAME_BUF_MAX];
    uint8_t  len;
    uint32_t last_ms;
    uint32_t timeout_ms;     /* 0 = 不启用超时 */
    uint16_t frames_ok;
    uint16_t frames_err;
} clib_frame_t;

/* 常用校验算法，单独暴露便于测试与复用 */
uint8_t clib_checksum_compute(clib_checksum_t mode, const uint8_t *data, uint8_t len);

int  clib_frame_init(clib_frame_t *f, const clib_frame_cfg_t *cfg, uint32_t timeout_ms);
void clib_frame_reset(clib_frame_t *f);

/* 组帧：head + payload + [checksum] + tail，返回帧总长度或负错误码 */
int  clib_frame_build(const clib_frame_cfg_t *cfg,
                      const uint8_t *payload, uint8_t payload_len,
                      uint8_t *out, uint16_t out_cap);

/* 组帧并经 port 层串口发出 */
int  clib_frame_send(uint8_t uart_id, const clib_frame_cfg_t *cfg,
                     const uint8_t *payload, uint8_t payload_len);

/* ---- 下面三个函数的 payload_cap 是调用方 payload_out 缓冲的实际容量 ----
 * 库不会写超过 payload_cap 字节；容量不足返回 CLIB_ERR_OVERFLOW。
 * 建议直接传 CLIB_FRAME_PAYLOAD_MAX 或 sizeof(你的缓冲)。
 * 返回 1 = 收到完整帧；0 = 还没收完；负数 = clib_err_t 错误码。 */
int  clib_frame_feed(clib_frame_t *f, uint8_t byte,
                     uint8_t *payload_out, uint8_t payload_cap, uint8_t *payload_len_out);

int  clib_frame_feed_buf(clib_frame_t *f, const uint8_t *bytes, uint16_t n,
                         uint8_t *payload_out, uint8_t payload_cap, uint8_t *payload_len_out);

/* 从 port 层串口读字节并喂入（非阻塞，单次最多处理 CLIB_FRAME_POLL_BUDGET 字节） */
int  clib_frame_poll(uint8_t uart_id, clib_frame_t *f,
                     uint8_t *payload_out, uint8_t payload_cap, uint8_t *payload_len_out);

#ifdef __cplusplus
}
#endif
#endif /* CLIB_FRAME_H */
