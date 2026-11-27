#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "protocol.h"

static bool integer(const char **at, long low, long high, int *out) {
    const char *p = *at;
    if (*p == '-' && low < 0)
        ++p;
    if (*p < '0' || *p > '9')
        return false;
    char *end;
    long value = strtol(*at, &end, 10);
    if (value < low || value > high)
        return false;
    *out = (int)value;
    *at = end;
    return true;
}

bool parse_request(const char *line, Request *r) {
    int seq;
    if (!integer(&line, 0, 65535, &seq) || *line++ != ' ')
        return false;
    r->seq = (unsigned)seq;
    r->extended = strncmp(line, "V ", 2) == 0;
    if (r->extended) line += 2;
    r->action = *line++;
    if (r->extended && (r->action == 'D' || r->action == 'A')) {
        bool distance = r->action == 'D';
        return *line++ == ' ' && integer(&line, 1, 65535, &r->x) &&
            *line++ == ' ' && integer(&line, distance ? 10 : -360, distance ? 10000 : 360, &r->y) &&
            r->y != 0 && *line == 0;
    }
    if (r->action == 'G') {
        return *line++ == ' ' && integer(&line, -1000000, 1000000, &r->x) && *line++ == ' ' &&
               integer(&line, -1000000, 1000000, &r->y) && *line == 0;
    }
    return r->action && strchr("SFLR", r->action) && *line == 0;
}

void apply_request(Car *car, const Request *r, uint32_t now) {
    if (r->action == 'D' || r->action == 'A') {
        car_task(car, r->action, r->x, r->y, now);
        return;
    }
    if (r->action == 'G') {
        car->goal_x = r->x;
        car->goal_y = r->y;
    }
    car_command(car, r->action, now);
}

int telemetry(char reply[REPLY_SIZE], const Car *v, const Request *request, Event e, bool log_failed) {
    int state = v->state;
    if (!request->extended && state > BUTTON) state = state == DONE ? IDLE : NO_RANGE;
    int n = snprintf(reply, REPLY_SIZE, "T %u %d %d %d %d", request->seq, state, (int)v->x, (int)v->y,
                     car_degrees(v));
    for (int i = 0; i < SECTORS; ++i) {
        const Ray *r = &v->rays[i];
        n += snprintf(reply + n, REPLY_SIZE - (size_t)n, " %lu %d %d %d %d",
                      (unsigned long)r->stamp, r->measured ? r->mm : -1, r->x, r->y, r->heading);
    }
    if (request->extended) {
        n += snprintf(reply + n, REPLY_SIZE - (size_t)n,
            " %lu %lu %lu %d %d %d %d %d %d %d %d %d %lu %lu %lu %d",
            (unsigned long)v->last_tick, (unsigned long)v->pulses, (unsigned long)(uint32_t)v->wheel_mm,
            v->speed_mm_s, v->imu_ready, v->task_id,
            v->imu[0], v->imu[1], v->imu[2], v->imu[3], v->imu[4], v->imu[5],
            (unsigned long)e.id, (unsigned long)e.ms, (unsigned long)e.code, log_failed);
    }
    reply[n++] = '\n';
    reply[n] = 0;
    return n;
}
