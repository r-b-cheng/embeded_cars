#ifndef PATROL_PROTOCOL_H
#define PATROL_PROTOCOL_H
#include "control.h"
#define REPLY_SIZE 256
typedef struct {
    unsigned seq;
    char action;
    int x, y;
} Request;
bool parse_request(const char *line, Request *request);
void apply_request(Car *car, const Request *request, uint32_t now);
int telemetry(char reply[REPLY_SIZE], const Car *car, unsigned seq);
#endif
