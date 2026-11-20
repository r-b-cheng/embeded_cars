#ifndef PATROL_CONTROL_H
#define PATROL_CONTROL_H
#include <stdbool.h>
#include <stdint.h>

enum { FRONT, LEFT, RIGHT, SECTORS };
enum { IDLE, DRIVING, BLOCKED, NO_RANGE, LINK_LOST, BUTTON };
typedef struct {
    int mm, x, y, heading; /* beam origin in mm, absolute angle in degrees */
    uint32_t stamp;
    bool measured;
} Ray;
typedef struct {
    double x, y, angle;
    Ray rays[SECTORS];
    uint32_t last_command, last_tick;
    char command;
    int state, left, right, goal_x, goal_y;
} Car;
void car_command(Car *car, char command, uint32_t now);
void car_tick(Car *car, uint32_t now, bool button);
void car_measure(Car *car, int sector, int mm, uint32_t now);
int car_degrees(const Car *car);
#endif
