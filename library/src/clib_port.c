#include "clib_port.h"

/* volatile：clib_init/deinit 可能在启动阶段由另一个上下文调用。
   约定：上电后单线程 init 一次，运行期不再改动（见 README 的并发约定）。 */
static const clib_port_t * volatile s_port = NULL;

/* port 回调允许直接返回 clib_err_t；其他裸负值统一归一化成 IO 错误 */
static int normalize_io(int r)
{
    if (r >= 0) {
        return r;
    }
    if (r >= CLIB_ERR_FORMAT) {   /* -1 .. -10 都是已定义的 clib_err_t */
        return r;
    }
    return CLIB_ERR_IO;
}

int clib_init(const clib_port_t *port)
{
    if (port == NULL) {
        return CLIB_ERR_NULL;
    }
    /* log_putc 允许为空，其余回调都是外设通信必需的 */
    if (port->millis == NULL || port->micros == NULL || port->delay_us == NULL ||
        port->gpio_read == NULL || port->gpio_write == NULL ||
        port->motor_set_duty == NULL || port->servo_set_pulse_us == NULL ||
        port->uart_write == NULL || port->uart_read == NULL) {
        return CLIB_ERR_NO_PORT;
    }
    s_port = port;
    return CLIB_OK;
}

void clib_deinit(void)
{
    s_port = NULL;
}

bool clib_is_ready(void)
{
    return (s_port != NULL);
}

const clib_port_t *clib_port(void)
{
    return s_port;
}

uint32_t clib_millis(void)
{
    return (s_port != NULL) ? s_port->millis() : 0u;
}

uint32_t clib_micros(void)
{
    return (s_port != NULL) ? s_port->micros() : 0u;
}

void clib_delay_us(uint32_t us)
{
    if (s_port != NULL) {
        s_port->delay_us(us);
    }
}

int clib_gpio_read(clib_pin_t pin)
{
    if (s_port == NULL) {
        return CLIB_ERR_NOT_INIT;
    }
    if ((int)pin < 0 || (int)pin >= (int)CLIB_PIN_COUNT) {
        return CLIB_ERR_PARAM;
    }
    return normalize_io(s_port->gpio_read((uint8_t)pin));
}

void clib_gpio_write(clib_pin_t pin, int level)
{
    if (s_port == NULL || (int)pin < 0 || (int)pin >= (int)CLIB_PIN_COUNT) {
        return;
    }
    s_port->gpio_write((uint8_t)pin, level);
}

void clib_motor_hw_set_duty(uint8_t ch, uint8_t duty)
{
    if (s_port == NULL || ch >= 4u) {
        return;
    }
    s_port->motor_set_duty(ch, duty);
}

void clib_servo_hw_set_pulse(uint16_t pulse_us)
{
    if (s_port == NULL) {
        return;
    }
    s_port->servo_set_pulse_us(pulse_us);
}

int clib_uart_write(uint8_t uart_id, const uint8_t *buf, uint16_t len)
{
    if (s_port == NULL) {
        return CLIB_ERR_NOT_INIT;
    }
    if (buf == NULL) {
        return CLIB_ERR_NULL;
    }
    return normalize_io(s_port->uart_write(uart_id, buf, len));
}

int clib_uart_read(uint8_t uart_id, uint8_t *buf, uint16_t cap)
{
    if (s_port == NULL) {
        return CLIB_ERR_NOT_INIT;
    }
    if (buf == NULL) {
        return CLIB_ERR_NULL;
    }
    if (cap == 0u) {
        return CLIB_ERR_PARAM;
    }
    return normalize_io(s_port->uart_read(uart_id, buf, cap));
}

void clib_log_char(char c)
{
    if (s_port != NULL && s_port->log_putc != NULL) {
        s_port->log_putc(c);
    }
}
