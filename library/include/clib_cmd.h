#ifndef CLIB_CMD_H
#define CLIB_CMD_H
/*
 * clib_cmd.h -- 蓝牙 / 串口文本指令（从外设接收命令 + 向下位机/上位机发送应答）
 *
 * 来源：案例 10「机器人蓝牙控制」User/main.c
 *   "ONA" 前进 / "ONB" 后退 / "ONC" 左 / "OND" 右 / "ONE" "ONF" 停止
 *   USART3 9600（K210 套餐用 115200），原代码用 IDLE 中断收一帧后 strcmp。
 *
 * 抽象后：指令表数据化、大小写不敏感、自动去 \r\n 与首尾空白，
 *         收帧交由 clib_frame / port 层，发送走 clib_cmd_send()。
 *         额外加了导航用指令（GOTO x,y、MODE:*、?），方便点对点项目扩展。
 */
#include "clib_common.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    CLIB_CMD_NONE = 0,
    CLIB_CMD_FORWARD,
    CLIB_CMD_BACKWARD,
    CLIB_CMD_LEFT,
    CLIB_CMD_RIGHT,
    CLIB_CMD_SPIN_LEFT,
    CLIB_CMD_SPIN_RIGHT,
    CLIB_CMD_STOP,
    CLIB_CMD_MODE_TRACK,
    CLIB_CMD_MODE_AVOID,
    CLIB_CMD_MODE_REMOTE,
    CLIB_CMD_MODE_NAV,
    CLIB_CMD_GOTO,
    CLIB_CMD_QUERY,
    CLIB_CMD_UNKNOWN
} clib_cmd_t;

typedef struct {
    const char *text;
    clib_cmd_t  cmd;
    const char *note;
} clib_cmd_entry_t;

extern const clib_cmd_entry_t CLIB_CMD_TABLE[];
extern const uint8_t          CLIB_CMD_TABLE_LEN;

clib_cmd_t  clib_cmd_parse(const char *frame);
const char *clib_cmd_name(clib_cmd_t cmd);
clib_act_t  clib_cmd_to_act(clib_cmd_t cmd);

/* 发送文本控制消息 / 应答到任意逻辑串口 */
int  clib_cmd_send(uint8_t uart_id, const char *msg);
int  clib_cmd_send_ack(uint8_t uart_id, clib_cmd_t cmd, int result);

/* 解析 "GOTO 12,-3"；支持正负坐标，越界返回 false */
bool clib_cmd_parse_goto(const char *frame, int16_t *x, int16_t *y);

#ifdef __cplusplus
}
#endif
#endif /* CLIB_CMD_H */
