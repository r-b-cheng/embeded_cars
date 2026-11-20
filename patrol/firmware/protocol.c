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
    r->action = *line++;
    if (r->action == 'G') {
        return *line++ == ' ' && integer(&line, -1000000, 1000000, &r->x) && *line++ == ' ' &&
               integer(&line, -1000000, 1000000, &r->y) && *line == 0;
    }
    return r->action && strchr("SFLR", r->action) && *line == 0;
}

void apply_request(Car *car, const Request *r, uint32_t now) {
    if (r->action == 'G') {
        car->goal_x = r->x;
        car->goal_y = r->y;
    }
    car_command(car, r->action, now);
}

int telemetry(char reply[REPLY_SIZE], const Car *v, unsigned seq) {
    int n = snprintf(reply, REPLY_SIZE, "T %u %d %d %d %d", seq, v->state, (int)v->x, (int)v->y,
                     car_degrees(v));
    for (int i = 0; i < SECTORS; ++i) {
        const Ray *r = &v->rays[i];
        n += snprintf(reply + n, REPLY_SIZE - (size_t)n, " %lu %d %d %d %d",
                      (unsigned long)r->stamp, r->measured ? r->mm : -1, r->x, r->y, r->heading);
    }
    reply[n++] = '\n';
    reply[n] = 0;
    return n;
}
