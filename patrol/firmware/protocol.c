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
    r->full = strncmp(line, "W ", 2) == 0;
    r->extended = r->full || strncmp(line, "V ", 2) == 0;
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

int telemetry(char reply[REPLY_SIZE], const Car *v, const Request *request, Event e, bool log_failed, uint32_t *trace_cursor) {
    _Static_assert(REPLY_SIZE >= 64 * 12 + 2, "64 bounded ASCII fields");
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
            (unsigned long)v->last_tick, (unsigned long)v->wheels[0].pulses, (unsigned long)(uint32_t)v->wheels[0].mm,
            v->wheels[0].speed_mm_s, v->imu_ready, v->task_id,
            v->imu[0], v->imu[1], v->imu[2], v->imu[3], v->imu[4], v->imu[5],
            (unsigned long)e.id, (unsigned long)e.ms, (unsigned long)e.code, log_failed);
    }
    if (request->full) {
        uint32_t pending = v->trace_count - *trace_cursor;
        uint32_t missed = pending > TRACE_CAPACITY ? pending - TRACE_CAPACITY : 0;
        uint32_t id = 0;
        Trace trace = {0};
        if (pending) {
            *trace_cursor += missed;
            trace = v->trace[*trace_cursor % TRACE_CAPACITY];
            id = ++*trace_cursor;
        }
        const WheelReading *right = &v->wheels[1];
        n += snprintf(reply + n, REPLY_SIZE - (size_t)n,
            " %lu %lu %d %d %d %d %lu %d %d %c %d %lu %d %d %d %d %d"
            " %lu %lu %lu %lu %lu %lu %d %d %d %lu",
            (unsigned long)right->pulses, (unsigned long)(uint32_t)right->mm,
            right->speed_mm_s, v->wheels[0].rpm10, right->rpm10, v->center_speed,
            (unsigned long)(uint32_t)v->center_mm, v->left, v->right, v->command,
            v->target, (unsigned long)v->imu_stamp, car_bearing(FRONT), car_bearing(LEFT),
            car_bearing(RIGHT), car_bearing(v->scan_sector), v->fault,
            (unsigned long)v->uart_errors, (unsigned long)v->radio_resets,
            (unsigned long)v->invalid_requests, (unsigned long)v->max_tick_ms,
            (unsigned long)id, (unsigned long)trace.ms, trace.kind, trace.a, trace.b,
            (unsigned long)missed);
    }
    reply[n++] = '\n';
    reply[n] = 0;
    return n;
}
