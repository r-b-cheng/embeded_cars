#ifndef PATROL_RADIO_H
#define PATROL_RADIO_H
#include "control.h"
#include "journal.h"
void radio_init(Car *car, Journal *log);
void radio_byte(char c, uint32_t now);
void radio_reset(uint32_t now);
void radio_tick(uint32_t now);
/* Board transport, also implemented by the transcript test. */
void radio_flush_tx(void);
void radio_write(const char *data);
#endif
