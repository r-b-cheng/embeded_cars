#include "radio_uart.h"
#include "radio.h"
#include "protocol.h"
#include "inc/hw_memmap.h"
#include "driverlib/uart.h"
#include <string.h>

/* An IMU burst can outlast the 16-byte UART FIFO at 115200 baud. */
#define RX_CAPACITY 512u
static volatile uint8_t rx[RX_CAPACITY];
static volatile unsigned rx_head, rx_tail;
static volatile bool rx_lost;

static void radio_rx(void) {
    UARTIntClear(UART1_BASE, UARTIntStatus(UART1_BASE, true));
    if (UARTRxErrorGet(UART1_BASE)) {
        UARTRxErrorClear(UART1_BASE);
        rx_lost = true;
    }
    while (UARTCharsAvail(UART1_BASE)) {
        uint8_t byte = (uint8_t)UARTCharGetNonBlocking(UART1_BASE);
        unsigned next = (rx_head + 1) % RX_CAPACITY;
        if (next == rx_tail) rx_lost = true;
        else {
            rx[rx_head] = byte;
            rx_head = next;
        }
    }
}
/* Queue AT commands and counted payloads in order. Drain only as much as
   the hardware FIFO accepts so telemetry never spins waiting for the UART. */
static uint8_t tx[2 * REPLY_SIZE];
static unsigned tx_head, tx_tail;
void radio_flush_tx(void) { tx_tail = tx_head; }

void radio_write(const char *s) {
    size_t len = strlen(s);
    if (len > (tx_tail + sizeof tx - tx_head - 1) % sizeof tx) {
        rx_lost = true;
        return;
    }
    while (*s) {
        tx[tx_head] = *s++;
        tx_head = (tx_head + 1) % sizeof tx;
    }
}
static void drain_tx(uint32_t now) {
    static bool fifo_full;
    static uint32_t last_attempt;
    if (fifo_full && now == last_attempt) return;
    fifo_full = false;
    while (tx_tail != tx_head) {
        if (!UARTCharPutNonBlocking(UART1_BASE, tx[tx_tail])) {
            fifo_full = true;
            last_attempt = now;
            break;
        }
        tx_tail = (tx_tail + 1) % sizeof tx;
    }
}

void radio_uart_init(uint32_t clock_hz) {
    UARTConfigSetExpClk(UART1_BASE, clock_hz, 115200,
                        UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE);
    UARTIntRegister(UART1_BASE, radio_rx);
    UARTIntEnable(UART1_BASE, UART_INT_RX | UART_INT_RT | UART_INT_OE);
}

void radio_uart_service(Car *car, uint32_t now) {
    if (rx_lost) {
        rx_lost = false;
        ++car->uart_errors;
        car_trace(car, now, TR_UART, 0, 0);
        rx_tail = rx_head;
        radio_reset(now);
    }
    for (int n = 0; n < 64 && rx_tail != rx_head; ++n) {
        char byte = (char)rx[rx_tail];
        rx_tail = (rx_tail + 1) % RX_CAPACITY;
        radio_byte(byte, now);
    }
    radio_tick(now);
    drain_tx(now);
}
