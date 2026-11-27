#ifndef PATROL_CONTROL_H
#define PATROL_CONTROL_H
#include <stdbool.h>
#include <stdint.h>

enum { FRONT, LEFT, RIGHT, SECTORS };
enum { IDLE, DRIVING, BLOCKED, NO_RANGE, LINK_LOST, BUTTON, DONE, FEEDBACK_FAULT, OUTPUT_FAULT };
typedef struct {
    int mm, x, y, heading; /* beam origin in mm, absolute angle in degrees */
    uint32_t stamp;
    bool measured;
} Ray;
typedef struct {
    double x, y, angle;
    /* A single-channel encoder measures unsigned left-wheel travel. */
    bool feedback, imu_ready, task_ended;
    uint32_t pulses, speed_pulses, speed_stamp, silent_drive_ms;
    double wheel_mm, yaw_deg, task_origin;
    int speed_mm_s, task_id, target, imu[6];
    uint32_t task_start, disconnects;
    Ray rays[SECTORS];
    uint32_t last_command, last_tick;
    char command;
    int state, left, right, goal_x, goal_y;
} Car;
void car_feedback(Car *car, uint32_t pulses, double delta_yaw_deg, uint32_t now);
void car_task(Car *car, char action, int id, int target, uint32_t now);
bool car_command(Car *car, char command, uint32_t now);
void car_tick(Car *car, uint32_t now, bool button);
void car_measure(Car *car, int sector, int mm, uint32_t now);
int car_degrees(const Car *car);
#endif
