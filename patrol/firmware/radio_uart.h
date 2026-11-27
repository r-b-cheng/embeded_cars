#ifndef PATROL_RADIO_UART_H
#define PATROL_RADIO_UART_H
#include "control.h"
void radio_uart_init(uint32_t clock_hz);
void radio_uart_service(Car *car, uint32_t now);
#endif
