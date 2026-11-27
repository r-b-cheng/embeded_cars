#ifndef PATROL_PROTOCOL_H
#define PATROL_PROTOCOL_H
#include "control.h"
#include "journal.h"
#define REPLY_SIZE 896
typedef struct {
    unsigned seq;
    char action;
    int x, y;
    bool extended, full;
} Request;
bool parse_request(const char *line, Request *request);
void apply_request(Car *car, const Request *request, uint32_t now);
int telemetry(char reply[REPLY_SIZE], const Car *car, const Request *request, Event event, bool log_failed, uint32_t *trace_cursor);
#endif
