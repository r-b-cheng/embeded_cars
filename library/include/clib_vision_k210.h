#ifndef CLIB_VISION_K210_H
#define CLIB_VISION_K210_H
/*
 * clib_vision_k210.h -- K210 / CanMV 视觉模块串口帧（从外设接收数据）
 *
 * 来源：K210 视觉循迹套餐资料
 *   K210 侧 robot_LinePatrol_4wd.py：
 *     struct.pack("<bbhb", 0xAA, 0xAE, int(偏转角), 0x5B)  → 5 字节
 *     UART1, IO6=RX / IO7=TX, 115200 8N1；QQVGA 灰度巡线，
 *     deflection_angle = degrees(-atan((center_pos-80)/60))，范围约 ±53°
 *   STM32 侧 main.c 的 canmv_Receive_Data()：状态机收帧，
 *     取 buff[n-3]、buff[n-2] 拼成 C = (高<<8) | 低。
 *
 * ★ 库里做的修正：
 *   1) 帧头 0xAA/0xAE 超过 127，原 py 用有符号 'b' 承载，标准 struct 会报错；
 *      clib_k210_build() 按无符号帧头 <BBhB 语义组帧（等价于 K210 实际发出的字节）。
 *   2) 原解析代码 RxBuffer1[5] 却循环写 10 字节（越界），这里固定 5 字节缓冲。
 *   3) 丢线时 center_pos=0 会算出恰好 +53°，原代码把它当"出界"标志；
 *      这是数值巧合不是协议字段，库里显式提供 clib_k210_is_lost()。
 *   4) 原帧无校验、无 ACK，库保留兼容，另建议升级到带校验的新帧（见 README）。
 */
#include "clib_common.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CLIB_K210_HEAD0       0xAAu
#define CLIB_K210_HEAD1       0xAEu
#define CLIB_K210_TAIL        0x5Bu
#define CLIB_K210_FRAME_LEN   5u
#define CLIB_K210_LOST_ANGLE  53
#define CLIB_K210_BAUD        115200u
#define CLIB_K210_POLL_BUDGET 16u    /* 单次 poll 最多处理字节数，避免持续噪声饿死控制环 */

typedef struct {
    uint8_t  state;                          /* 0 找头0 / 1 找头1 / 2 收数据 */
    uint8_t  buf[CLIB_K210_FRAME_LEN];
    uint8_t  pos;
    uint16_t frames_ok;
    uint16_t frames_err;
} clib_k210_t;

void clib_k210_init(clib_k210_t *k);
void clib_k210_reset(clib_k210_t *k);

/* 组帧（K210 侧参考实现 / host 测试造数据）：out = AA AE [低8] [高8] 5B */
int  clib_k210_build(int16_t angle, uint8_t out[CLIB_K210_FRAME_LEN]);

/* 逐字节喂入；返回 1 = 收到一帧，0 = 进行中，负 = 错误码 */
int  clib_k210_feed(clib_k210_t *k, uint8_t byte, int16_t *angle_out);

/* 从 port 层串口取字节并解析（非阻塞） */
int  clib_k210_poll(uint8_t uart_id, clib_k210_t *k, int16_t *angle_out);

bool       clib_k210_is_lost(int16_t angle);
/* 原 main.c 的阈值决策，deadzone 默认给 10 即可复现原行为 */
clib_act_t clib_k210_decide(int16_t angle, int16_t deadzone);

#ifdef __cplusplus
}
#endif
#endif /* CLIB_VISION_K210_H */
