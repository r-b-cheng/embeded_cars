#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/fpu.h"
#include "driverlib/pin_map.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"
#include "board_tm4c123g.h"
#include "clib_ultrasonic.h"
#include "wheel.h"
#include "radio.h"
#include "sensors.h"

extern uint8_t _event_log_start[];
static Journal journal;

static Car car = {.command = 'S'};
static uint32_t ticks_us;
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
enum { DISARMED, RISE, FALL, COMPLETE };
static volatile unsigned echo_phase;
static volatile uint32_t rise_ticks, width_ticks;

static void echo_edge(void) {
    uint32_t t = TimerValueGet(TIMER0_BASE, TIMER_A);
    GPIOIntClear(GPIO_PORTB_BASE, GPIO_PIN_2);
    bool high = GPIOPinRead(GPIO_PORTB_BASE, GPIO_PIN_2) != 0;
    if (echo_phase == RISE && high) {
        rise_ticks = t;
        echo_phase = FALL;
    } else if (echo_phase == FALL && !high) {
        width_ticks = t - rise_ticks;
        echo_phase = COMPLETE;
    }
}

static void enable(uint32_t id) {
    SysCtlPeripheralEnable(id);
    while (!SysCtlPeripheralReady(id)) {
    }
}

static void io_init(void) {
    enable(SYSCTL_PERIPH_GPIOC);
    enable(SYSCTL_PERIPH_GPIOF);
    enable(SYSCTL_PERIPH_UART1);
    enable(SYSCTL_PERIPH_TIMER0);
    enable(SYSCTL_PERIPH_WTIMER1);
    ticks_us = SysCtlClockGet() / 1000000;
    GPIOPinConfigure(GPIO_PC4_U1RX);
    GPIOPinConfigure(GPIO_PC5_U1TX);
    GPIOPinTypeUART(GPIO_PORTC_BASE, GPIO_PIN_4 | GPIO_PIN_5);
    UARTConfigSetExpClk(UART1_BASE, SysCtlClockGet(), 115200,
                        UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE);
    UARTIntRegister(UART1_BASE, radio_rx);
    UARTIntEnable(UART1_BASE, UART_INT_RX | UART_INT_RT | UART_INT_OE);
    GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_4);
    GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC_UP);
    TimerLoadSet(TIMER0_BASE, TIMER_A, UINT32_MAX);
    TimerEnable(TIMER0_BASE, TIMER_A);
    GPIOPadConfigSet(GPIO_PORTB_BASE, GPIO_PIN_2, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);
    GPIOIntTypeSet(GPIO_PORTB_BASE, GPIO_PIN_2, GPIO_BOTH_EDGES);
    GPIOIntRegister(GPIO_PORTB_BASE, echo_edge);
    GPIOIntClear(GPIO_PORTB_BASE, GPIO_PIN_2);
    GPIOIntEnable(GPIO_PORTB_BASE, GPIO_PIN_2);

    GPIOPinConfigure(GPIO_PC7_WT1CCP1);
    GPIOPinTypeTimer(GPIO_PORTC_BASE, GPIO_PIN_7);
    TimerConfigure(WTIMER1_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_B_PWM);
    TimerLoadSet(WTIMER1_BASE, TIMER_B, 20000 * ticks_us - 1);
    TimerMatchSet(WTIMER1_BASE, TIMER_B, (20000 - 1500) * ticks_us - 1);
    TimerUpdateMode(WTIMER1_BASE, TIMER_B, TIMER_UP_MATCH_TIMEOUT);
    TimerEnable(WTIMER1_BASE, TIMER_B);
}

static void scan(uint32_t now) {
    static const int order[] = {FRONT, LEFT, FRONT, RIGHT};
    static const unsigned pulses[] = {1500, 1900, 1100};
    static unsigned index;
    static uint32_t changed;
    static bool waiting;
    if (!waiting && now - changed >= 140) {
        waiting = true;
        changed = now;
        if (!GPIOPinRead(GPIO_PORTB_BASE, GPIO_PIN_2)) {
            echo_phase = RISE;
            clib_ultra_trigger();
        }
    } else if (waiting && (echo_phase == COMPLETE || now - changed >= 30)) {
        int mm = -1;
        if (echo_phase == COMPLETE) {
            uint32_t us = width_ticks / ticks_us;
            if (us >= 116 && us <= 23320)
                mm = clib_echo_us_to_mm(us);
        }
        echo_phase = DISARMED;
        car_measure(&car, order[index], mm, now);
        index = (index + 1) % 4;
        TimerMatchSet(WTIMER1_BASE, TIMER_B, (20000 - pulses[order[index]]) * ticks_us - 1);
        changed = now;
        waiting = false;
    }
}

void radio_write(const char *s) {
    /* Echo, encoder edges and UART reception continue during transmission. */
    while (*s)
        UARTCharPut(UART1_BASE, *s++);
}

int main(void) {
    FPUEnable();
    Board_Init();
    Wheel_Init(&g_board_port);
    io_init();
    sensors_init();
    journal_open(&journal, (const volatile Event *)_event_log_start);
    journal_append(&journal, clib_millis(), EV_BOOT);
    radio_init(&car, &journal);
    uint32_t disconnects = 0, last_block = 0;
    int last_state = IDLE;
    int left = 0, right = 0;
    for (;;) {
        uint32_t now = clib_millis();
        scan(now);
        if (rx_lost) {
            rx_lost = false;
            rx_tail = rx_head;
            radio_reset(now);
        }
        for (int n = 0; n < 64 && rx_tail != rx_head; ++n) {
            char byte = (char)rx[rx_tail];
            rx_tail = (rx_tail + 1) % RX_CAPACITY;
            radio_byte(byte, now);
        }
        radio_tick(now);
        if (now - car.last_tick >= 10) {
            sensors_sample(&car, now);
            car_tick(&car, now, !GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_4));
            if (left != car.left || right != car.right) {
                Wheel_Set(car.left, car.right);
                left = car.left;
                right = car.right;
            }
            if (!left && !right) {
                const uint8_t motor_pins = GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
                if (GPIOPinRead(GPIO_PORTB_BASE, motor_pins)) {
                    Wheel_Stop();
                    if (GPIOPinRead(GPIO_PORTB_BASE, motor_pins)) {
                        car.state = OUTPUT_FAULT;
                        car.command = 'S';
                    }
                }
                if (car.disconnects != disconnects) {
                    journal_append(&journal, now, EV_DISCONNECT);
                    disconnects = car.disconnects;
                }
                if (car.state != last_state) {
                    if (car.state == LINK_LOST) journal_append(&journal, now, EV_LEASE);
                    if (car.state == OUTPUT_FAULT) journal_append(&journal, now, EV_OUTPUT);
                    if (car.state == BUTTON) journal_append(&journal, now, EV_BUTTON);
                    if (car.state == FEEDBACK_FAULT) journal_append(&journal, now, EV_FEEDBACK);
                    if (car.state == BLOCKED && (!last_block || now - last_block >= 30000)) {
                        journal_append(&journal, now, EV_BLOCKED);
                        last_block = now;
                    }
                }
            }
            last_state = car.state;
        }
    }
}
