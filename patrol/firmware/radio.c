#include "radio.h"
#include "protocol.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef PATROL_AP_PASSWORD
#define PATROL_AP_PASSWORD "SurveyCar01"
#endif
/* Stop-and-wait TCP. AT status lines and counted binary +IPD may interleave. */
static const char *const setup[] = {
    "AT+RST\r\n",        "ATE0\r\n",
    "AT+CWMODE=2\r\n",   "AT+CWSAP=\"Patrol-Car\",\"" PATROL_AP_PASSWORD "\",6,3\r\n",
    "AT+CIPMODE=0\r\n",  "AT+CIPMUX=1\r\n",
    "AT+CIPDINFO=0\r\n", "AT+CIPSERVER=1,7070\r\n"};
enum { SETUP, LISTEN, PROMPT, SENDING, RETRY };
static int phase, step, owner, source;
static uint32_t started, remaining;
static Car *vehicle;
static char line[96], request[40], reply[REPLY_SIZE];
static unsigned used, size;
static bool discard, handshaken;
static uint32_t payload_at;

static void release(void) {
    car_command(vehicle, 'S', vehicle->last_tick);
    owner = -1;
    handshaken = false;
    size = 0;
    discard = false;
}

static void reset_link(uint32_t now) {
    release();
    used = remaining = 0;
    phase = RETRY;
    started = now;
}

void radio_init(Car *car) {
    vehicle = car;
    reset_link(0);
}

static void configure(uint32_t now) {
    phase = SETUP;
    started = now;
    radio_write(setup[step]);
}

void radio_tick(uint32_t now) {
    if (remaining && now - payload_at > 500)
        reset_link(now);
    if (phase == RETRY && now - started >= 1000) {
        step = 0;
        configure(now);
    } else if ((phase == SETUP || phase == PROMPT || phase == SENDING) &&
               now - started > (phase == SETUP ? 7000u : 1000u)) {
        reset_link(now);
    }
}

static void respond(unsigned seq, uint32_t now) {
    int n = telemetry(reply, vehicle, seq);
    char send[32];
    snprintf(send, sizeof send, "AT+CIPSEND=%d,%d\r\n", owner, n);
    phase = PROMPT;
    started = now;
    radio_write(send);
}

static void command_byte(char c, uint32_t now) {
    if (source != owner || owner < 0)
        return;
    if (c != '\n') {
        if (c < 32 || c > 126 || size >= sizeof request - 1)
            discard = true;
        else
            request[size++] = c;
        return;
    }
    request[size] = 0;
    Request r = {0};
    bool valid = !discard && parse_request(request, &r);
    if (!valid)
        r.action = 'S';
    size = 0;
    discard = false;
    /* A new request can arrive just before the previous SEND OK. Accept STOP
       immediately; a conforming client retries by reconnecting on a lost reply. */
    if (phase != LISTEN && r.action != 'S')
        return;
    if (!handshaken && r.action != 'S')
        return;
    if (valid && r.action == 'S')
        handshaken = true;
    apply_request(vehicle, &r, now);
    if (phase == LISTEN && valid)
        respond(r.seq, now);
}

static void event(uint32_t now) {
    if (!strcmp(line, "ready")) {
        release();
        step = 1;
        configure(now);
    } else if (phase == SETUP && step && !strcmp(line, "OK")) {
        if (++step == (int)(sizeof setup / sizeof *setup))
            phase = LISTEN;
        else
            configure(now);
    } else if (strlen(line) == 9 && !strcmp(line + 1, ",CONNECT") && line[0] >= '0' &&
               line[0] <= '4' && phase == LISTEN && owner < 0) {
        owner = line[0] - '0';
    } else if (strlen(line) == 8 && !strcmp(line + 1, ",CLOSED") && line[0] - '0' == owner) {
        if (phase == LISTEN)
            release();
        else
            reset_link(now);
    } else if (phase == SENDING && !strcmp(line, "SEND OK")) {
        phase = LISTEN;
    } else if (!strcmp(line, "ERROR") || !strcmp(line, "FAIL") || !strcmp(line, "SEND FAIL")) {
        reset_link(now);
    }
}

void radio_byte(char c, uint32_t now) {
    if (remaining) {
        --remaining;
        command_byte(c, now);
        return;
    }
    if (c == '>' && phase == PROMPT) {
        phase = SENDING;
        started = now;
        used = 0;
        radio_write(reply);
        return;
    }
    if (c == '\r' || (!used && c == ' '))
        return;
    if (c == '\n') {
        line[used] = 0;
        event(now);
        used = 0;
        return;
    }
    if (c == ':' && used >= 8 && !strncmp(line, "+IPD,", 5)) {
        line[used] = 0;
        char *end;
        unsigned long length = strtoul(line + 7, &end, 10);
        if (line[5] < '0' || line[5] > '4' || line[6] != ',' || *end || length == 0 ||
            length > 65535) {
            reset_link(now);
            return;
        }
        remaining = length;
        payload_at = now;
        source = line[5] - '0';
        used = 0;
        return;
    }
    if (used == sizeof line - 1) {
        reset_link(now);
        return;
    }
    line[used++] = c;
}
