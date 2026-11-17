#ifndef CLIB_PORT_MOCK_H
#define CLIB_PORT_MOCK_H
/*
 * clib_port_mock.h -- PC 端假硬件（仅供单元测试，不进嵌入式工程）
 *
 * 让整套外设通信库在没有开发板的情况下也能验证：
 *   - 能断言「发给外设的控制消息」是否正确（4 路 PWM / 舵机脉宽 / 串口字节）
 *   - 能注入「外设送过来的数据」（GPIO 电平序列 / 串口字节流）
 */
#include "clib_common.h"
#include "clib_port.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CLIB_MOCK_FIFO_SIZE   512u
#define CLIB_MOCK_TXLOG_SIZE  2048u

typedef struct {
    uint8_t  buf[CLIB_MOCK_FIFO_SIZE];
    uint16_t head;
    uint16_t tail;
} clib_mock_fifo_t;

typedef struct {
    uint32_t now_ms;
    uint32_t now_us;
    int      gpio[CLIB_PIN_COUNT];
    uint8_t  motor_duty[4];
    uint16_t servo_pulse_us;
    uint32_t servo_write_count;
    uint32_t motor_write_count;
    uint32_t gpio_write_count;
    clib_mock_fifo_t rx[CLIB_UART_COUNT];
    uint8_t  tx[CLIB_UART_COUNT][CLIB_MOCK_TXLOG_SIZE];
    uint16_t tx_len[CLIB_UART_COUNT];
    char     log[CLIB_MOCK_TXLOG_SIZE];
    uint16_t log_len;
} clib_mock_t;

extern clib_mock_t g_mock;

void              clib_mock_reset(void);
const clib_port_t *clib_mock_port(void);   /* 返回静态 const 表，可直接传给 clib_init */
void              clib_mock_advance_ms(uint32_t ms);
void              clib_mock_advance_us(uint32_t us);
void              clib_mock_set_time_ms(uint32_t ms);
void              clib_mock_set_time_us(uint32_t us);
void              clib_mock_set_gpio(clib_pin_t pin, int level);
void              clib_mock_push_rx(uint8_t uart_id, const uint8_t *bytes, uint16_t n);
uint16_t          clib_mock_take_tx(uint8_t uart_id, uint8_t *out, uint16_t cap);
void              clib_mock_clear_tx(uint8_t uart_id);
const char       *clib_mock_log(void);

#ifdef __cplusplus
}
#endif
#endif /* CLIB_PORT_MOCK_H */
