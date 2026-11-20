#ifndef PATROL_RADIO_H
#define PATROL_RADIO_H
#include "control.h"
void radio_init(Car *car);
void radio_byte(char c, uint32_t now);
void radio_tick(uint32_t now);
/* Board transport, also implemented by the transcript test. */
void radio_write(const char *data);
#endif
