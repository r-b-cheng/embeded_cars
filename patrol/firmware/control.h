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
enum { WHEEL_LEFT, WHEEL_RIGHT, WHEELS };
typedef struct {
    uint32_t pulses, window_pulses, silent_ms, driven_at;
    double mm;
    int speed_mm_s, rpm10, sign;
} WheelReading;
enum { TRACE_CAPACITY = 128 };
enum { TR_COMMAND = 1, TR_CONTROL, TR_IMU, TR_RADIO_RESET, TR_INVALID, TR_UART, TR_LINK };
typedef struct { uint32_t ms; int kind, a, b; } Trace;
typedef struct {
    uint32_t trace_count;
    Trace trace[TRACE_CAPACITY];
    double x, y, angle;
    bool feedback, imu_ready, task_ended, button_held;
    WheelReading wheels[WHEELS];
    uint32_t speed_stamp, imu_stamp, uart_errors, radio_resets, invalid_requests, max_tick_ms;
    int center_speed, fault, scan_sector;
    double center_mm, window_mm, yaw_deg, task_origin;
    int task_id, target, imu[6];
    uint32_t task_start, disconnects;
    Ray rays[SECTORS];
    uint32_t last_command, last_tick;
    char command;
    int state, left, right, goal_x, goal_y;
} Car;
void car_trace(Car *car, uint32_t now, int kind, int a, int b);
void car_feedback(Car *car, uint32_t left_edges, uint32_t right_edges, double delta_yaw_deg, uint32_t now);
void car_task(Car *car, char action, int id, int target, uint32_t now);
bool car_command(Car *car, char command, uint32_t now);
void car_tick(Car *car, uint32_t now, bool button);
void car_measure(Car *car, int sector, int mm, uint32_t now);
int car_bearing(int sector);
int car_degrees(const Car *car);
#endif
