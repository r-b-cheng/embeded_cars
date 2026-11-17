#ifndef CLIB_COMMON_H
#define CLIB_COMMON_H
/*
 * clib_common.h -- 小车外设通信库：公共类型与工具
 *
 * 设计原则：本层及以下所有代码只包含 C 标准库头文件，
 * 不含任何 STM32 / TivaWare / 芯片寄存器依赖，可在 PC 上直接编译测试。
 */
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CLIB_VERSION_MAJOR  1u
#define CLIB_VERSION_MINOR  0u
#define CLIB_VERSION_PATCH  0u
#define CLIB_VERSION_STRING "1.0.0"

/* ---------------- 返回值 ---------------- */
typedef enum {
    CLIB_OK            =   0,
    CLIB_ERR_NOT_INIT  =  -1,   /* 未调用 clib_init / 模块未初始化 */
    CLIB_ERR_NULL      =  -2,   /* 空指针 */
    CLIB_ERR_PARAM     =  -3,   /* 参数越界 */
    CLIB_ERR_NO_PORT   =  -4,   /* port 层未实现该回调 */
    CLIB_ERR_TIMEOUT   =  -5,
    CLIB_ERR_CHECKSUM  =  -6,
    CLIB_ERR_OVERFLOW  =  -7,
    CLIB_ERR_IO        =  -8,
    CLIB_ERR_BUSY      =  -9,
    CLIB_ERR_FORMAT    = -10
} clib_err_t;

/* ---------------- 逻辑引脚 ----------------
 * 库只认逻辑编号；物理管脚（PB6 / PE4 ...）由 port 层映射。
 * 换主控时库代码一行都不用改。
 */
typedef enum {
    CLIB_PIN_IR_REMOTE  = 0,    /* 红外接收头，输入，空闲为高 */
    CLIB_PIN_TRACK_L    = 1,    /* 红外循迹 左 */
    CLIB_PIN_TRACK_R    = 2,    /* 红外循迹 右 */
    CLIB_PIN_OBST_L     = 3,    /* 红外避障 左 */
    CLIB_PIN_OBST_R     = 4,    /* 红外避障 右 */
    CLIB_PIN_ULTRA_TRIG = 5,    /* HC-SR04 TRIG，输出 */
    CLIB_PIN_ULTRA_ECHO = 6,    /* HC-SR04 ECHO，输入 */
    CLIB_PIN_COUNT      = 7
} clib_pin_t;

/* ---------------- 逻辑串口 ---------------- */
typedef enum {
    CLIB_UART_DEBUG  = 0,
    CLIB_UART_BT     = 1,
    CLIB_UART_VISION = 2,
    CLIB_UART_COUNT  = 3
} clib_uart_t;

/* ---------------- 底盘动作 ---------------- */
typedef enum {
    CLIB_ACT_STOP = 0,
    CLIB_ACT_FORWARD,
    CLIB_ACT_BACKWARD,
    CLIB_ACT_LEFT,
    CLIB_ACT_RIGHT,
    CLIB_ACT_SPIN_LEFT,
    CLIB_ACT_SPIN_RIGHT,
    CLIB_ACT_BRAKE,
    CLIB_ACT_COUNT
} clib_act_t;

/* 传感器 2bit 状态：bit1=左，bit0=右 */
#define CLIB_SENS_LEFT_BIT   0x02u
#define CLIB_SENS_RIGHT_BIT  0x01u
#define CLIB_SENS_BOTH       0x03u
#define CLIB_SENS_NONE       0x00u

/* ---------------- 小工具 ---------------- */
static inline int16_t clib_clamp_i16(int16_t v, int16_t lo, int16_t hi)
{
    if (v < lo) { return lo; }
    if (v > hi) { return hi; }
    return v;
}

static inline uint8_t clib_clamp_u8(int32_t v, int32_t lo, int32_t hi)
{
    if (v < lo) { v = lo; }
    if (v > hi) { v = hi; }
    return (uint8_t)v;
}

static inline float clib_clamp_f(float v, float lo, float hi)
{
    if (v < lo) { return lo; }
    if (v > hi) { return hi; }
    return v;
}

const char *clib_version(void);
const char *clib_err_str(clib_err_t e);
const char *clib_act_name(clib_act_t a);

#ifdef __cplusplus
}
#endif
#endif /* CLIB_COMMON_H */
