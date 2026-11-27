#include "control.h"
#include <math.h>
#include "calibration.h"

/* Open-loop estimates: calibrate under load on the actual floor. */
#define FULL_SPEED_MM_S 300.0

#define DUTY 40
#define PI 3.141592653589793
#define LEASE_MS 650u
#define FRONT_MAX_AGE 450u
#define SIDE_MAX_AGE 850u

static const int bearing[SECTORS] = {0, 35, -35}; /* measure on fitted horn */

int car_degrees(const Car *car) { return (int)lround(car->angle * 180 / PI); }

void car_measure(Car *car, int sector, int mm, uint32_t now) {
    car->rays[sector] = (Ray){
        mm,  (int)lround(car->x), (int)lround(car->y), car_degrees(car) + bearing[sector], now,
        true};
}

void car_feedback(Car *car, uint32_t pulses, double delta_yaw_deg, uint32_t now) {
    uint32_t edges = pulses - car->pulses;
    double travel = edges * (WHEEL_CIRCUMFERENCE_MM / PULSES_PER_WHEEL_REV);
    car->feedback = true;
    car->pulses = pulses;
    car->wheel_mm += travel;
    if (edges)
        car->silent_drive_ms = 0;
    uint32_t window = now - car->speed_stamp;
    if (window >= 250) {
        car->speed_mm_s = (int)lround((pulses - car->speed_pulses) *
            (WHEEL_CIRCUMFERENCE_MM / PULSES_PER_WHEEL_REV) * 1000 / window);
        car->speed_stamp = now;
        car->speed_pulses = pulses;
    }
    double rotation = delta_yaw_deg * PI / 180;
    /* Positive rotation is left. Recover center travel from the left wheel
       during forward drive. A pivot has nominally zero center translation. */
    if (car->left > 0 && car->right > 0) {
        double center = travel + TRACK_MM * rotation / 2;
        car->x += center * cos(car->angle + rotation / 2);
        car->y += center * sin(car->angle + rotation / 2);
    }
    car->yaw_deg += delta_yaw_deg;
    car->angle = remainder(car->angle + rotation, 2 * PI);
}

void car_task(Car *car, char action, int id, int target, uint32_t now) {
    bool same = car->command == action && car->task_id == id;
    if (!car_command(car, action, now) || same)
        return;
    car->state = IDLE;
    car->task_id = id;
    car->target = target;
    car->task_origin = action == 'D' ? car->wheel_mm : car->yaw_deg;
    car->task_start = now;
    car->task_ended = false;
}

bool car_command(Car *car, char command, uint32_t now) {
    if (car->state == OUTPUT_FAULT ||
        (command != 'S' && (car->state == BUTTON || car->state == LINK_LOST || car->state == FEEDBACK_FAULT)))
        return false;
    if (command == 'S') {
        car->state = IDLE;
        car->silent_drive_ms = 0;
    }
    car->command = command;
    car->last_command = now;
    return true;
}

static bool fresh(const Car *car, int sector, uint32_t now, unsigned limit) {
    const Ray *ray = &car->rays[sector];
    int angle = car_degrees(car) + bearing[sector] - ray->heading;
    double delta = remainder(angle, 360);
    if (fabs(delta) > (sector == FRONT ? 14 : 25) || fabs(car->x - ray->x) > 100 ||
        fabs(car->y - ray->y) > 100)
        return false;
    return ray->measured && ray->mm >= 20 && ray->mm <= 4000 && now - ray->stamp <= limit;
}

void car_tick(Car *car, uint32_t now, bool button) {
    uint32_t elapsed = now - car->last_tick;
    car->last_tick = now;
    if (!car->feedback) { /* Legacy open-loop simulation. */
        double scale = FULL_SPEED_MM_S * elapsed / 100000.0;
        double distance = (car->right + car->left) * scale / 2;
        double rotation = (car->right - car->left) * scale / TRACK_MM;
        car->x += distance * cos(car->angle + rotation / 2);
        car->y += distance * sin(car->angle + rotation / 2);
        car->angle = remainder(car->angle + rotation, 2 * PI);
    }
    bool was_driving = car->left || car->right;
    car->left = car->right = 0;
    if (car->state == OUTPUT_FAULT)
        return;
    if (button || (car->command != 'S' && now - car->last_command > LEASE_MS)) {
        car->state = button ? BUTTON : LINK_LOST;
        car->command = 'S';
        return;
    }
    if (car->state == BUTTON || car->state == LINK_LOST || car->state == FEEDBACK_FAULT)
        return;
    char motion = car->command;
    if (was_driving)
        car->silent_drive_ms += elapsed;
    if (motion != 'S' && car->feedback &&
        (!car->imu_ready || car->silent_drive_ms > 1000)) {
        car->state = FEEDBACK_FAULT;
        return;
    }
    if (motion == 'D' || motion == 'A') {
        double progress = (motion == 'D' ? car->wheel_mm : car->yaw_deg) - car->task_origin;
        if (car->target < 0)
            progress = -progress;
        if (car->task_ended || progress >= fabs((double)car->target)) {
            car->task_ended = true;
            car->state = DONE;
            return;
        }
        uint32_t limit = motion == 'A' ? 15000u : 5000u + (uint32_t)car->target * 50u;
        if (!car->feedback || now - car->task_start > limit || progress < -12) {
            car->state = FEEDBACK_FAULT;
            return;
        }
        motion = motion == 'D' ? 'F' : (car->target > 0 ? 'L' : 'R');
    }
    if (motion == 'G') {
        double gx = car->goal_x - car->x, gy = car->goal_y - car->y;
        if (hypot(gx, gy) < 50)
            motion = 'S';
        else {
            double error = remainder(atan2(gy, gx) - car->angle, 2 * PI);
            motion = fabs(error) > 17 * PI / 180 ? (error > 0 ? 'L' : 'R') : 'F';
        }
    }
    if (motion != 'F' && motion != 'L' && motion != 'R') {
        car->state = IDLE;
        return;
    }
    const Ray *r = car->rays;
    if (!fresh(car, FRONT, now, FRONT_MAX_AGE) || !fresh(car, LEFT, now, SIDE_MAX_AGE) ||
        !fresh(car, RIGHT, now, SIDE_MAX_AGE)) {
        car->state = NO_RANGE;
        return;
    }
    bool forward = motion == 'F';
    if (r[FRONT].mm < (forward ? 400 : 180) || r[LEFT].mm < 160 || r[RIGHT].mm < 160 ||
        (!forward && r[motion == 'L' ? LEFT : RIGHT].mm < 300)) {
        car->state = BLOCKED;
        return;
    }
    car->state = DRIVING;
    car->left = motion == 'L' ? -DUTY : DUTY;
    car->right = motion == 'R' ? -DUTY : DUTY;
    if (forward && r[LEFT].mm < 350 && r[LEFT].mm + 80 < r[RIGHT].mm)
        car->right -= 10;
    if (forward && r[RIGHT].mm < 350 && r[RIGHT].mm + 80 < r[LEFT].mm)
        car->left -= 10;
}
