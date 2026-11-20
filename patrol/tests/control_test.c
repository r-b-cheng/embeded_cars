#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "radio.h"
#include "protocol.h"

static char out[512];
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
    radio_init(&c);
    radio_tick(1000);
    assert(strstr(out, "AT+RST"));
    bytes("OK\r\nready\r\n", 1400);
    assert(strstr(out, "ATE0"));
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
int main(void) {
    safety();
    api();
    at_server();
    puts("C motion interlocks and ESP-AT transcript passed");
}
