#ifndef PATROL_SENSORS_H
#define PATROL_SENSORS_H
#include "control.h"
void sensors_init(void);
void sensors_sample(Car *car, uint32_t now);
#endif
