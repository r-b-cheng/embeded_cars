/* Real Cortex-M code, interrupt vectors, UART transport, AT parser and motion
   controller. Sonar, IMU and dual encoder inputs are deterministic fixtures. */
#include "control.h"
#include "radio.h"
#include "radio_uart.h"
#include <string.h>
#include "driverlib/fpu.h"
#include "driverlib/systick.h"
#include "driverlib/interrupt.h"

volatile uint32_t clock_ms;
Car probe;
/* Monitor-visible acceptance results. */
volatile uint32_t lease_delay, max_interval, moving_ticks, stopped_outputs;
volatile uint32_t block_tx, fifo_refusals;
bool __real_UARTCharPutNonBlocking(uint32_t base, unsigned char byte);
bool __wrap_UARTCharPutNonBlocking(uint32_t base, unsigned char byte) {
    if (block_tx) { ++fifo_refusals; return false; }
    return __real_UARTCharPutNonBlocking(base,byte);
}
static Event slots[LOG_SLOTS];
static Journal journal;
bool log_erase_page(unsigned slot) { memset(slots + slot, 0xff, 1024); return true; }
bool log_program(unsigned slot,const Event *e) { slots[slot] = *e; return true; }
static void tick(void) { ++clock_ms; }
int main(void) {
    FPUEnable();
    probe.command = 'S';
    probe.imu_ready = true;
    radio_uart_init(80000000);
    SysTickPeriodSet(80000);
    SysTickIntRegister(tick);
    SysTickIntEnable();
    SysTickEnable();
    IntMasterEnable();
    memset(slots, 0xff, sizeof slots);
    journal_open(&journal,slots);
    for (unsigned i=0; i<LOG_SLOTS; i++) journal_append(&journal,i,EV_BOOT);
    for (unsigned i=0; i<TRACE_CAPACITY+50; i++) car_trace(&probe,0,TR_CONTROL,0,0);
    radio_init(&probe, &journal);
    uint32_t l = 0, r = 0, next = 0;
    for (;;) {
        uint32_t now = clock_ms;
        radio_uart_service(&probe, now);
        if (now - probe.last_tick >= 10) {
            if (now - next >= 60) {
                if (probe.left) ++l;
                if (probe.right) ++r;
                next = now;
            }
            car_feedback(&probe,l,r,(probe.right - probe.left)*0.01,now);
            for (int i = 0; i < 3; ++i) car_measure(&probe,i,1000,now);
            probe.imu_stamp = now;
            car_tick(&probe,now,false);
            max_interval = probe.max_tick_ms;
            if (probe.left && probe.right) ++moving_ticks;
            if (probe.state == LINK_LOST && !lease_delay) {
                lease_delay = now - probe.last_command;
                stopped_outputs = !probe.left && !probe.right;
            }
        }
    }
}
