#include "control.h"
#include <math.h>

/* Open-loop estimates: calibrate under load on the actual floor. */
#define FULL_SPEED_MM_S 300.0
#define TRACK_MM 135.0
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

void car_command(Car *car, char command, uint32_t now) {
    if (command != 'S' && (car->state == BUTTON || car->state == LINK_LOST))
        return;
    if (command == 'S')
        car->state = IDLE;
    car->command = command;
    car->last_command = now;
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
    double scale = FULL_SPEED_MM_S * elapsed / 100000.0;
    double distance = (car->right + car->left) * scale / 2;
    double rotation = (car->right - car->left) * scale / TRACK_MM;
    car->x += distance * cos(car->angle + rotation / 2);
    car->y += distance * sin(car->angle + rotation / 2);
    car->angle = remainder(car->angle + rotation, 2 * PI);

    car->left = car->right = 0;
    if (button || (car->command != 'S' && now - car->last_command > LEASE_MS)) {
        car->state = button ? BUTTON : LINK_LOST;
        car->command = 'S';
        return;
    }
    if (car->state == BUTTON || car->state == LINK_LOST)
        return;
    char motion = car->command;
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
