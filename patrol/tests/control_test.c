#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "radio.h"
#include "protocol.h"

static char out[512];
void radio_flush_tx(void) {}
void radio_write(const char *data) { snprintf(out, sizeof out, "%s", data); }
static void bytes(const char *s, uint32_t now) {
    while (*s)
        radio_byte(*s++, now);
}
static void ranges(Car *c, uint32_t t, int f, int l, int r) {
    car_measure(c, FRONT, f, t);
    car_measure(c, LEFT, l, t);
    car_measure(c, RIGHT, r, t);
}
static void safety(void) {
    Car c = {.command = 'S'};
    car_command(&c, 'F', 0);
    car_tick(&c, 10, false);
    assert(c.state == NO_RANGE && !c.left);
    ranges(&c, 10, 600, 240, 900);
    car_tick(&c, 20, false);
    assert(c.state == DRIVING && c.left > c.right && c.right > 0);
    ranges(&c, 30, 300, 900, 800);
    car_tick(&c, 30, false);
    assert(c.state == BLOCKED && !c.right);
    car_command(&c, 'L', 40);
    car_tick(&c, 40, false);
    assert(c.left < 0 && c.right > 0);
    ranges(&c, 50, 150, 900, 800);
    car_tick(&c, 50, false);
    assert(c.state == BLOCKED);
    ranges(&c, 60, 2000, -1, 800);
    car_tick(&c, 60, false);
    assert(c.state == NO_RANGE);
    ranges(&c, 70, 2000, 900, 800);
    car_tick(&c, 521, false);
    assert(c.state == NO_RANGE);
    car_tick(&c, 800, false);
    assert(c.state == LINK_LOST && !c.left);
    car_command(&c, 'F', 805);
    car_tick(&c, 810, false);
    assert(c.state == LINK_LOST);
    car_command(&c, 'S', 815);
    ranges(&c, 815, 800, 900, 1000);
    car_command(&c, 'R', 820);
    car_tick(&c, 820, false);
    assert(c.right < 0);
    car_tick(&c, 830, true);
    car_command(&c, 'F', 840);
    car_tick(&c, 840, false);
    assert(c.state == BUTTON && !c.left);
    c = (Car){.last_tick = UINT32_MAX - 20, .command = 'S'};
    car_command(&c, 'F', UINT32_MAX - 20);
    ranges(&c, UINT32_MAX - 20, 900, 800, 700);
    car_tick(&c, 12, false);
    assert(c.left > 0);
}
static void api(void) {
    Request r = {0};
    assert(parse_request("65535 G -200 300", &r) && r.x == -200 && r.y == 300);
    const char *bad[] = {"",      "1 ",       "65536 F",        "-1 F",
                         "1 G 1", "1 G 1 2x", "1 G -1000001 0", "1 S extra",
                         "1  F",  "1 G +1 2"};
    for (unsigned i = 0; i < sizeof bad / sizeof *bad; i++)
        assert(!parse_request(bad[i], &r));
    Car c = {.command = 'S'};
    ranges(&c, 100, 900, 1000, 900);
    car_command(&c, 'F', 100);
    c.angle = .4;
    car_tick(&c, 100, false);
    assert(c.state == NO_RANGE && !c.left);
}

static void at_server(void) {
    Car c = {.command = 'S'};
    radio_init(&c, NULL);
    radio_tick(1000);
    assert(strstr(out, "AT+RST"));
    bytes("OK\r\nready\r\n", 1400);
    assert(strstr(out, "ATE0"));
#ifdef PATROL_PERSONAL
    bytes("OK\r\n", 1410);
    assert(strstr(out, "AT+CWMODE=1"));
    bytes("OK\r\n", 1420);
    assert(strstr(out, "AT+CWDHCP=1,1"));
    bytes("OK\r\n", 1425);
    assert(strstr(out, "AT+CWJAP="));
    bytes("WIFI CONNECTED\r\nWIFI GOT IP\r\nOK\r\n", 1430);
    assert(strstr(out, "AT+CIPSTA?"));
    bytes("+CIPSTA:gateway:\"192.168.43.1\"\r\n", 1440);
    for (int i = 0; i < 5; i++) bytes("OK\r\n", 1450);
    radio_tick(4500);
    assert(strstr(out, "AT+CIPSTART=4,\"TCP\",\"192.168.43.1\",7071"));
    bytes("4,CLOSED\r\nERROR\r\n", 4510);
    radio_tick(7510);
    bytes("4,CONNECT\r\nOK\r\n+IPD,4,6:1 V S\n", 7520);
    assert(strstr(out, "AT+CIPSEND=4,"));
    bytes("OK\r\n> ", 7521);
    assert(strstr(out, "T 1 "));
    bytes("SEND OK\r\n+IPD,4,6:2 V F\nWIFI DISCONNECT\r\n", 7530);
    assert(c.command == 'S' && c.disconnects == 1);
    return;
#endif
    for (int i = 0; i < 7; i++)
        bytes("OK\r\n", 1500);
    bytes("0,CONNECT\r\n+IPD,0,4:1 F\n", 1600);
    assert(c.command == 'S');
    bytes("+IPD,0,4:1 S\n", 1600);
    assert(strstr(out, "AT+CIPSEND=0,"));
    bytes("OK\r\n> ", 1601);
    assert(strstr(out, "T 1 "));
    bytes("\r\nSEND OK\r\n+IPD,1,4:2 F\n", 1610);
    assert(c.command == 'S');
    bytes("+IPD,0,2:2 +IPD,0,2:F\n", 1630);
    assert(c.command == 'F');
    bytes("OK\r\n> ", 1640);
    assert(strstr(out, "T 2 "));
    bytes("SEND OK\r\n+IPD,0,9:invalid!\n", 1650);
    assert(c.command == 'S');
    bytes("+IPD,0,4:3 L\n", 1700);
    assert(c.command == 'L');
    bytes("0,CLOSED\r\n", 1701);
    assert(c.command == 'S');
}
static Event slots[LOG_SLOTS];
bool log_erase_page(unsigned slot) {
    assert(slot == 0 || slot == PAGE_SLOTS);
    memset(slots + slot, 0xff, 1024);
    return true;
}
bool log_program(unsigned slot, const Event *e) {
    assert(slots[slot].id == UINT32_MAX);
    slots[slot] = *e;
    return true;
}
static void persistence(void) {
    Journal j;
    memset(slots, 0xff, sizeof slots);
    journal_open(&j, slots);
    for (unsigned i = 1; i <= 160; i++) journal_append(&j, i * 1000, EV_BOOT);
    slots[j.next].id = 161; /* interrupted program, checksum still erased */
    journal_open(&j, slots);
    assert(j.newest == 160 && journal_after(&j, 0).id == 65);
    journal_append(&j, 456, EV_BUTTON);
    assert(journal_after(&j, 160).code == EV_BUTTON);
    assert(!journal_after(&j, 161).id);
}
static void measured_motion(void) {
    Car c = {.command = 'S', .imu_ready = true};
    Request r;
    assert(!parse_request("1 V D 0 100", &r) && !parse_request("1 V A 1 0", &r));
    assert(parse_request("1 V D 1 100", &r));
    car_feedback(&c, 0, 0, 0, 0);
    for (unsigned i = 0; i <= 12; i++) {
        uint32_t now = i * 100;
        apply_request(&c, &r, now);
        car_feedback(&c, i, i, 0, now);
        ranges(&c, now, 900, 900, 900);
        car_tick(&c, now, false);
        assert(c.state == (i >= 10 ? DONE : DRIVING));
    }
    assert(c.wheels[0].speed_mm_s >= 100 && c.wheels[0].speed_mm_s < 105 && c.left == 0);
    assert(parse_request("2 V A 2 -90", &r));
    for (unsigned i = 0; i <= 10; ++i) {
        uint32_t now = 2000 + 10 * i;
        apply_request(&c, &r, now);
        car_feedback(&c, 12 + i, 12 + i, i ? -10 : 0, now);
        ranges(&c, now, 900, 900, 900);
        car_tick(&c, now, false);
        assert(c.state == (i >= 9 ? DONE : DRIVING));
    }
    car_command(&c, 'S', 2150);
    for (unsigned now = 2200; now <= 3400; now += 100) {
        car_command(&c, 'F', now);
        ranges(&c, now, 900, 900, 900);
        car_feedback(&c, c.wheels[0].pulses + 1, c.wheels[1].pulses, 0, now);
        car_tick(&c, now, false);
    }
    assert(c.state == FEEDBACK_FAULT && c.fault == 3 && !c.left);
    car_command(&c, 'S', 3500);
    c.imu_ready = false;
    car_command(&c, 'R', 3500);
    car_tick(&c, 3500, false);
    assert(c.state == FEEDBACK_FAULT && !c.right);
    c.command = 'D';
    car_task(&c, 'D', 33, 100, 4000);
    assert(c.state == FEEDBACK_FAULT);
    c.state = OUTPUT_FAULT;
    car_command(&c, 'S', 4010);
    car_task(&c, 'A', 1, 45, 4020);
    car_tick(&c, 4030, true);
    assert(c.state == OUTPUT_FAULT && !c.left && !c.right);
}

static void dual_wheels(void) {
    Car c = {.command='S', .imu_ready=true, .left=30, .right=30};
    car_feedback(&c, 10, 20, 0, 1000);
    assert(c.wheels[0].rpm10 == 300 && c.wheels[1].rpm10 == 600);
    assert(c.wheels[0].speed_mm_s == 102 && c.wheels[1].speed_mm_s == 204);
    assert(c.center_speed == 153 && fabs(c.center_mm - 153.15) < .1);
    c.left = -30;
    car_feedback(&c, 12, 22, 15, 1200);
    assert(fabs(c.center_mm - 153.15) < .1 && car_degrees(&c) == 15);
    car_tick(&c,1210,true);
    car_command(&c,'S',1211);
    assert(c.state == BUTTON && !car_command(&c,'F',1212));
    car_tick(&c,1220,false);
    assert(car_command(&c,'S',1221) && c.state == IDLE);
}
int main(void) {
    safety();
    api();
    at_server();
    persistence();
    measured_motion();
    dual_wheels();
    puts("C motion interlocks and ESP-AT transcript passed");
}
