#ifndef CLIB_IR_SENSOR_H
#define CLIB_IR_SENSOR_H
/*
 * clib_ir_sensor.h -- 红外循迹 / 红外避障传感器（从外设接收数据）
 *
 * 来源：案例 5/6/7 的 Irtracking.c、Irobstacle.c 与 main.c 里的
 *       Robot_Traction() / Robot_Avoidance()
 *   原实现：直接 GPIO_ReadInputDataBit，同一个循环里重复读 4 次、无防抖。
 *   抽象后：统一成 2bit 状态 (左<<1 | 右)，带回 configurable 极性与去抖，
 *           决策查表抽成纯函数，换板只改 port 层的 gpio_read 映射。
 */
#include "clib_common.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t  pin_left;
    uint8_t  pin_right;
    bool     active_low;        /* true = 检测到目标时模块输出低电平 */
    uint16_t debounce_ms;
    uint8_t  stable;            /* 稳定的 2bit 状态 */
    uint8_t  candidate;
    uint32_t t_change;
    bool     has_candidate;
} clib_ir_sensor_t;

int clib_ir_sensor_init(clib_ir_sensor_t *s,
                        uint8_t pin_left, uint8_t pin_right,
                        bool active_low, uint16_t debounce_ms);

/* 立即读，返回值 = (左 << 1) | 右，见 CLIB_SENS_* 宏 */
uint8_t clib_ir_sensor_read_raw(clib_ir_sensor_t *s);

/* 带去抖的稳定值；now_ms 显式传入，便于确定性测试 */
uint8_t clib_ir_sensor_read_at(clib_ir_sensor_t *s, uint32_t now_ms);
uint8_t clib_ir_sensor_read(clib_ir_sensor_t *s);

/* 纯函数查表：传感器状态 → 底盘动作 */
clib_act_t clib_track_decide(uint8_t state);
clib_act_t clib_avoid_decide(uint8_t state);

#ifdef __cplusplus
}
#endif
#endif /* CLIB_IR_SENSOR_H */
