#ifndef CLIB_IR_REMOTE_H
#define CLIB_IR_REMOTE_H
/*
 * clib_ir_remote.h -- 红外遥控接收（从外设接收数据）
 *
 * 来源：案例 6「循迹、避障、遥控综合程序」User/main.c 的 Irscan() /
 *       ControlCar_Ircontrol()，以及 Hardware/Incontrol.c 的 IRremote_Counttime()；
 *       案例 4「机器人红外遥控」提供另一套遥控按键码表（其 Incontrol.c 的
 *       EXTI9_5_IRQHandler 解码体整段被注释，不可用）。
 *   原实现：PA8 上拉输入，忙等 Delay_us(20) 数高电平宽度，
 *           以 20us 为 1 tick 判定引导码 / bit1 / bit0，拼 32bit，
 *           再取 code>>8 拆成 (hi=code/16, lo=code%16) 查表得到动作。
 *
 * 抽象后：解码器只吃「高电平脉宽」，不关心脉宽是 GPIO 采样来的、
 *         定时器输入捕获来的，还是单元测试伪造的 —— 因此换 MCU 不用重写解码。
 *
 * ★ 修复原厂代码的 3 个问题：
 *   1) 局部变量 Data 未初始化，无效脉宽会沿用上一次的 bit 造成误码 → 忽略无效脉宽；
 *   2) 原移位在 Num>=32 判断之前，实际收了 33 位 → 这里严格 32 位；
 *   3) 原 Irscan() 是 while(1) 忙等且每轮都调 ControlCar_Ircontrol()（buf[2] 未初始化）
 *      → 这里全部改成非阻塞、显式状态、无副作用。
 *
 * ★ 两套遥控码表不一样（案例4 与案例6 完全不同），库里都保留，按实物选一套。
 */
#include "clib_common.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CLIB_IR_TICK_US         20u
#define CLIB_IR_TIMEOUT_TICKS   250u
#define CLIB_IR_LEADER_MIN      200u
#define CLIB_IR_LEADER_MAX      250u
#define CLIB_IR_BIT1_MIN        60u
#define CLIB_IR_BIT1_MAX        90u
#define CLIB_IR_BIT0_MIN        10u
#define CLIB_IR_BIT0_MAX        50u
#define CLIB_IR_DATA_BITS       32u

typedef enum {
    CLIB_IRDEC_SEEK_LEADER = 0,
    CLIB_IRDEC_DATA,
    CLIB_IRDEC_DONE,
    CLIB_IRDEC_ERROR
} clib_irdec_state_t;

typedef struct {
    clib_irdec_state_t state;
    uint32_t code;
    uint8_t  bits;
    uint16_t frames_ok;
    uint16_t frames_err;
} clib_irdec_t;

void clib_irdec_init(clib_irdec_t *d);
void clib_irdec_reset(clib_irdec_t *d);

/* 喂一个以 20us 为单位的脉宽计数（兼容原厂 IRremote_Counttime 的返回值）
 * 返回 1 = 收满 32bit 并把码写入 *code_out；0 = 进行中；
 *       CLIB_ERR_TIMEOUT / CLIB_ERR_NULL = 错误 */
int clib_irdec_feed_tick(clib_irdec_t *d, uint8_t ticks20us, uint32_t *code_out);

/* 喂微秒脉宽（推荐：由输入捕获中断给出，精度比 20us 轮询高） */
int clib_irdec_feed_pulse(clib_irdec_t *d, uint32_t high_us, uint32_t *code_out);

/* 降级方案：没有输入捕获时，用 GPIO 电平 + 微秒时基做非阻塞采样。
 * now_us 显式传入，便于无硬件环境下做确定性测试。 */
typedef struct {
    clib_irdec_t dec;
    int      last_level;
    uint32_t rise_us;
    bool     measuring;
} clib_ir_sampler_t;

void clib_ir_sampler_init(clib_ir_sampler_t *s);
int  clib_ir_sampler_poll(clib_ir_sampler_t *s, uint32_t now_us, uint32_t *code_out);
int  clib_ir_sampler_poll_now(clib_ir_sampler_t *s, uint32_t *code_out);

/* ---- 键码：把 32bit 原码映射成小车动作 ---- */
uint8_t clib_ir_data_code(uint32_t raw_code);
void    clib_ir_split_key(uint8_t data_code, uint8_t *hi, uint8_t *lo);

typedef struct {
    uint8_t     hi;
    uint8_t     lo;
    clib_act_t  act;
    const char *label;
} clib_ir_key_t;

/* 案例 4「机器人红外遥控」使用的码表 */
extern const clib_ir_key_t CLIB_IR_KEYMAP_V1[];
extern const uint8_t       CLIB_IR_KEYMAP_V1_LEN;
/* 案例 6「循迹、避障、遥控综合程序」使用的码表（与 V1 完全不同） */
extern const clib_ir_key_t CLIB_IR_KEYMAP_V2[];
extern const uint8_t       CLIB_IR_KEYMAP_V2_LEN;

clib_act_t clib_ir_lookup(uint32_t raw_code, const clib_ir_key_t *map, uint8_t n);

#ifdef __cplusplus
}
#endif
#endif /* CLIB_IR_REMOTE_H */
