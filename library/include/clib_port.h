#ifndef CLIB_PORT_H
#define CLIB_PORT_H
/*
 * clib_port.h -- 平台适配层（Porting Layer）
 *
 * ★ 这是整个库里唯一需要随主控芯片改动的地方。
 *   换到 TM4C123G 时，只需实现下面这一个函数指针表并调用 clib_init()，
 *   include/ 与 src/ 下的所有外设通信代码一行都不用改。
 *
 * 用法（TM4C123G 侧）：
 *   static const clib_port_t port = {
 *       .millis = my_millis, .micros = my_micros, .delay_us = ROM_SysCtlDelay...,
 *       .gpio_read = my_gpio_read, ...
 *   };
 *   clib_init(&port);
 *
 * 约定：
 *   - gpio_read / gpio_write 的 pin 参数是 clib_pin_t 逻辑编号，由本层映射到物理管脚。
 *   - motor_set_duty 的 ch 为 0..3，对应 L298N 的 IN1~IN4；duty 为 0..100（百分比）。
 *   - servo_set_pulse_us 的参数是目标高电平脉宽（微秒），500..2500。
 *     由本层负责把它换算成 PWM 比较值；不要在库里换算，因为各芯片 PWM 时基不同。
 *   - uart_read 必须非阻塞，没有数据就返回 0。
 *   - millis / micros 必须是单调递增的 uint32 计数（允许回绕，库内用减法差值）。
 *   - log_putc 可以为 NULL。
 */
#include "clib_common.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct clib_port {
    uint32_t (*millis)(void);
    uint32_t (*micros)(void);
    void     (*delay_us)(uint32_t us);

    int      (*gpio_read)(uint8_t logic_pin);      /* 返回 0 / 1；<0 表示错误 */
    void     (*gpio_write)(uint8_t logic_pin, int level);

    void     (*motor_set_duty)(uint8_t ch, uint8_t duty);
    void     (*servo_set_pulse_us)(uint16_t pulse_us);

    int      (*uart_write)(uint8_t uart_id, const uint8_t *buf, uint16_t len);
    int      (*uart_read)(uint8_t uart_id, uint8_t *buf, uint16_t cap);

    void     (*log_putc)(char c);                  /* 可选 */
} clib_port_t;

/* 注册平台实现。port 必须在整个运行期有效（通常是 static const）。 */
int  clib_init(const clib_port_t *port);
void clib_deinit(void);
bool clib_is_ready(void);
const clib_port_t *clib_port(void);

/* ---- 内部使用的安全包装：上层模块只调这些，不直接碰回调 ---- */
uint32_t clib_millis(void);
uint32_t clib_micros(void);
void     clib_delay_us(uint32_t us);
int      clib_gpio_read(clib_pin_t pin);
void     clib_gpio_write(clib_pin_t pin, int level);
void     clib_motor_hw_set_duty(uint8_t ch, uint8_t duty);
void     clib_servo_hw_set_pulse(uint16_t pulse_us);
int      clib_uart_write(uint8_t uart_id, const uint8_t *buf, uint16_t len);
int      clib_uart_read(uint8_t uart_id, uint8_t *buf, uint16_t cap);
void     clib_log_char(char c);

#ifdef __cplusplus
}
#endif
#endif /* CLIB_PORT_H */
