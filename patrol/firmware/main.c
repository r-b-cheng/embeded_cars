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

static Car car = {.command = 'S'};
static uint32_t ticks_us;
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
    /* Bounded control/telemetry frames, < 22ms at 115200. Echo edges use IRQ. */
    while (*s)
        UARTCharPut(UART1_BASE, *s++);
}

int main(void) {
    FPUEnable();
    Board_Init();
    Wheel_Init(&g_board_port);
    io_init();
    radio_init(&car);
    int left = 0, right = 0;
    for (;;) {
        uint32_t now = clib_millis();
        scan(now);
        for (int n = 0; n < 32 && UARTCharsAvail(UART1_BASE); ++n)
            radio_byte((char)UARTCharGetNonBlocking(UART1_BASE), now);
        radio_tick(now);
        if (now - car.last_tick >= 10) {
            car_tick(&car, now, !GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_4));
            if (left != car.left || right != car.right) {
                Wheel_Set(car.left, car.right);
                left = car.left;
                right = car.right;
            }
        }
    }
}
