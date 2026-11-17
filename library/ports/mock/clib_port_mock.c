#include "clib_port_mock.h"
#include <string.h>

clib_mock_t g_mock;

static uint32_t m_millis(void) { return g_mock.now_ms; }
static uint32_t m_micros(void) { return g_mock.now_us; }
static void     m_delay_us(uint32_t us) { g_mock.now_us += us; }

static int m_gpio_read(uint8_t pin)
{
    if (pin >= (uint8_t)CLIB_PIN_COUNT) {
        return -1;
    }
    return g_mock.gpio[pin];
}

static void m_gpio_write(uint8_t pin, int level)
{
    if (pin >= (uint8_t)CLIB_PIN_COUNT) {
        return;
    }
    g_mock.gpio[pin] = level ? 1 : 0;
    g_mock.gpio_write_count++;
}

static void m_motor_set_duty(uint8_t ch, uint8_t duty)
{
    if (ch < 4u) {
        g_mock.motor_duty[ch] = duty;
        g_mock.motor_write_count++;
    }
}

static void m_servo_set_pulse(uint16_t pulse_us)
{
    g_mock.servo_pulse_us = pulse_us;
    g_mock.servo_write_count++;
}

static int m_uart_write(uint8_t uart_id, const uint8_t *buf, uint16_t len)
{
    uint16_t i;
    if (uart_id >= (uint8_t)CLIB_UART_COUNT || buf == NULL) {
        return CLIB_ERR_PARAM;
    }
    for (i = 0u; i < len; ++i) {
        if (g_mock.tx_len[uart_id] < CLIB_MOCK_TXLOG_SIZE) {
            g_mock.tx[uart_id][g_mock.tx_len[uart_id]++] = buf[i];
        } else {
            return (int)i;
        }
    }
    return (int)len;
}

static int m_uart_read(uint8_t uart_id, uint8_t *buf, uint16_t cap)
{
    clib_mock_fifo_t *f;
    uint16_t n = 0u;

    if (uart_id >= (uint8_t)CLIB_UART_COUNT || buf == NULL) {
        return CLIB_ERR_PARAM;
    }
    f = &g_mock.rx[uart_id];
    while (n < cap && f->head != f->tail) {
        buf[n++] = f->buf[f->head];
        f->head = (uint16_t)((f->head + 1u) % CLIB_MOCK_FIFO_SIZE);
    }
    return (int)n;
}

static void m_log_putc(char c)
{
    if (g_mock.log_len < (uint16_t)(CLIB_MOCK_TXLOG_SIZE - 1u)) {
        g_mock.log[g_mock.log_len++] = c;
        g_mock.log[g_mock.log_len] = '\0';
    }
}

static const clib_port_t s_mock_port = {
    m_millis, m_micros, m_delay_us,
    m_gpio_read, m_gpio_write,
    m_motor_set_duty, m_servo_set_pulse,
    m_uart_write, m_uart_read,
    m_log_putc
};

void clib_mock_reset(void)
{
    memset(&g_mock, 0, sizeof(g_mock));
    /* 红外接收头空闲为高电平 */
    g_mock.gpio[CLIB_PIN_IR_REMOTE] = 1;
}

const clib_port_t *clib_mock_port(void)
{
    return &s_mock_port;
}

/* 饱和加法，避免 ms*1000u 在 uint32 上回绕（原实现 ms>4294967 会出错） */
static uint32_t sat_add(uint32_t a, uint32_t b)
{
    uint64_t s = (uint64_t)a + (uint64_t)b;
    return (s > (uint64_t)UINT32_MAX) ? UINT32_MAX : (uint32_t)s;
}

void clib_mock_advance_ms(uint32_t ms)
{
    uint64_t add_us = (uint64_t)ms * 1000u;
    uint64_t us = (uint64_t)g_mock.now_us + add_us;
    g_mock.now_ms = sat_add(g_mock.now_ms, ms);
    g_mock.now_us = (us > (uint64_t)UINT32_MAX) ? UINT32_MAX : (uint32_t)us;
}

void clib_mock_advance_us(uint32_t us)
{
    g_mock.now_us = sat_add(g_mock.now_us, us);
    g_mock.now_ms = sat_add(g_mock.now_ms, us / 1000u);
}

void clib_mock_set_time_ms(uint32_t ms)
{
    uint64_t us = (uint64_t)ms * 1000u;
    g_mock.now_ms = ms;
    g_mock.now_us = (us > (uint64_t)UINT32_MAX) ? UINT32_MAX : (uint32_t)us;
}

void clib_mock_set_time_us(uint32_t us)
{
    g_mock.now_us = us;
    g_mock.now_ms = us / 1000u;
}

void clib_mock_set_gpio(clib_pin_t pin, int level)
{
    if ((int)pin >= 0 && (int)pin < (int)CLIB_PIN_COUNT) {
        g_mock.gpio[(int)pin] = level ? 1 : 0;
    }
}

void clib_mock_push_rx(uint8_t uart_id, const uint8_t *bytes, uint16_t n)
{
    clib_mock_fifo_t *f;
    uint16_t i;

    if (uart_id >= (uint8_t)CLIB_UART_COUNT || bytes == NULL) {
        return;
    }
    f = &g_mock.rx[uart_id];
    for (i = 0u; i < n; ++i) {
        uint16_t next = (uint16_t)((f->tail + 1u) % CLIB_MOCK_FIFO_SIZE);
        if (next == f->head) {
            break;   /* FIFO 满，丢弃剩余 */
        }
        f->buf[f->tail] = bytes[i];
        f->tail = next;
    }
}

uint16_t clib_mock_take_tx(uint8_t uart_id, uint8_t *out, uint16_t cap)
{
    uint16_t n;
    if (uart_id >= (uint8_t)CLIB_UART_COUNT || out == NULL) {
        return 0u;
    }
    n = (g_mock.tx_len[uart_id] < cap) ? g_mock.tx_len[uart_id] : cap;
    memcpy(out, g_mock.tx[uart_id], n);
    return n;
}

void clib_mock_clear_tx(uint8_t uart_id)
{
    if (uart_id < (uint8_t)CLIB_UART_COUNT) {
        g_mock.tx_len[uart_id] = 0u;
    }
}

const char *clib_mock_log(void)
{
    return g_mock.log;
}
