#include "radio.h"
#include "protocol.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef PATROL_PERSONAL
#include "hotspot.h"
#endif
#ifndef PATROL_AP_PASSWORD
#define PATROL_AP_PASSWORD "SurveyCar01"
#endif
/* Stop-and-wait TCP. AT status lines and counted binary +IPD may interleave. */
static const char *const setup[] = {
    "AT+RST\r\n",        "ATE0\r\n",
#ifdef PATROL_PERSONAL
    "AT+CWMODE=1\r\n", "AT+CWDHCP=1,1\r\n",
    "AT+CWJAP=" HOTSPOT_ARGUMENTS "\r\n",
    "AT+CIPSTA?\r\n",
#else
    "AT+CWMODE=2\r\n",   "AT+CWSAP=\"Patrol-Car\",\"" PATROL_AP_PASSWORD "\",6,3\r\n",
#endif
    "AT+CIPMODE=0\r\n",  "AT+CIPMUX=1\r\n",
    "AT+CIPDINFO=0\r\n", "AT+CIPSERVER=1,7070\r\n"};
enum { SETUP, LISTEN, PROMPT, SENDING, RETRY, DIAL };
static int phase, step, owner, source;
static uint32_t started, remaining;
static Car *vehicle;
static Journal *journal;
static uint32_t cursor, trace_cursor;
#ifdef PATROL_PERSONAL
static uint32_t last_dial;
static char gateway[16];
#endif
static char line[96], request[40], reply[REPLY_SIZE];
static unsigned used, size;
static Request deferred;
static bool has_deferred;
static uint32_t deferred_at;
static bool discard, handshaken;
static uint32_t payload_at;

static void release(void) {
    if (handshaken) ++vehicle->disconnects;
    cursor = trace_cursor = 0;
    car_command(vehicle, 'S', vehicle->last_tick);
    has_deferred = false;
    owner = -1;
    handshaken = false;
    size = 0;
    discard = false;
}

void radio_reset(uint32_t now) {
    radio_flush_tx();
    car_trace(vehicle, now, TR_RADIO_RESET, phase, step);
    ++vehicle->radio_resets;
    release();
    used = remaining = 0;
    phase = RETRY;
    started = now;
}

void radio_init(Car *car, Journal *log) {
    vehicle = car;
    journal = log;
    handshaken = false;
    radio_reset(0);
}

static void configure(uint32_t now) {
    phase = SETUP;
    started = now;
    radio_write(setup[step]);
}

void radio_tick(uint32_t now) {
#ifdef PATROL_PERSONAL
    if (phase == LISTEN && owner < 0 && gateway[0] && now - last_dial >= 3000) {
        char command[64];
        snprintf(command, sizeof command, "AT+CIPSTART=4,\"TCP\",\"%s\",7071\r\n", gateway);
        last_dial = started = now;
        phase = DIAL;
        radio_write(command);
    } else if (phase == DIAL && now - started > 12000)
        radio_reset(now);
#endif
    if (remaining && now - payload_at > 500)
        radio_reset(now);
    if (phase == RETRY && now - started >= 1000) {
#ifdef PATROL_PERSONAL
        gateway[0] = 0;
#endif
        step = 0;
        configure(now);
    } else if ((phase == SETUP || phase == PROMPT || phase == SENDING) &&
               now - started > (phase == SETUP ? 30000u : 1000u)) {
        radio_reset(now);
    }
}

static void respond(const Request *r, uint32_t now) {
    Event event = r->extended ? journal_after(journal, cursor) : (Event){0};
    if (event.id) cursor = event.id;
    int n = telemetry(reply, vehicle, r, event, journal && journal->failed, &trace_cursor);
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
    if (!valid) {
        ++vehicle->invalid_requests;
        car_trace(vehicle, now, TR_INVALID, size, source);
        r.action = 'S';
    }
    size = 0;
    discard = false;
    /* A phone can receive the frame before ESP emits its SEND OK.
       Keep one conforming stop-and-wait request across that boundary. */
    if (phase == SENDING && valid && handshaken) {
        if (has_deferred) { radio_reset(now); return; }
        if (r.action == 'S') apply_request(vehicle, &r, now);
        deferred = r;
        deferred_at = now;
        has_deferred = true;
        return;
    }
    if (phase != LISTEN && r.action != 'S') return;
    if (!handshaken && r.action != 'S')
        return;
    if (valid && r.action == 'S')
        handshaken = true;
    apply_request(vehicle, &r, now);
    if (phase == LISTEN && valid)
        respond(&r, now);
}

static void event(uint32_t now) {
#ifdef PATROL_PERSONAL
    if (phase == SETUP && step == 5 && !strncmp(line, "+CIPSTA:gateway:\"", 17)) {
        const char *ip = line + 17;
        size_t n = strspn(ip, "0123456789.");
        if (n > 0 && n < sizeof gateway && !strcmp(ip + n, "\"")) {
            memcpy(gateway, ip, n);
            gateway[n] = 0;
        }
        return;
    }
    if (!strcmp(line, "WIFI DISCONNECT") && (phase != SETUP || step > 4)) {
        radio_reset(now);
        return;
    }
    /* A refused dial can report CLOSED before its final ERROR. */
    if (phase == LISTEN && owner < 0 && !strcmp(line, "ERROR"))
        return;
    if (phase == DIAL && (!strcmp(line, "ERROR") || !strcmp(line, "FAIL") ||
                          !strcmp(line, "4,CLOSED"))) {
        phase = LISTEN;
        last_dial = now;
        return;
    }
#endif
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
               line[0] <= '4' && (phase == LISTEN || phase == DIAL) && owner < 0) {
        owner = line[0] - '0';
        car_trace(vehicle, now, TR_LINK, owner, 1);
        phase = LISTEN;
    } else if (strlen(line) == 8 && !strcmp(line + 1, ",CLOSED") && line[0] - '0' == owner) {
        car_trace(vehicle, now, TR_LINK, owner, 0);
        if (phase == LISTEN)
            release();
        else
            radio_reset(now);
    } else if (phase == SENDING && !strcmp(line, "SEND OK")) {
        phase = LISTEN;
        if (has_deferred) {
            has_deferred = false;
            if (now - deferred_at >= 650) { radio_reset(now); return; }
            apply_request(vehicle, &deferred, deferred_at);
            respond(&deferred, now);
        }
    } else if (!strcmp(line, "ERROR") || !strcmp(line, "FAIL") || !strcmp(line, "SEND FAIL")) {
        radio_reset(now);
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
            radio_reset(now);
            return;
        }
        remaining = length;
        payload_at = now;
        source = line[5] - '0';
        used = 0;
        return;
    }
    if (used == sizeof line - 1) {
        radio_reset(now);
        return;
    }
    line[used++] = c;
}
