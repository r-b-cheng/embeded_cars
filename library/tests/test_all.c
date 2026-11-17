/*
 * test_all.c -- 外设通信库的全套单元测试（PC 端，无硬件）
 * 编译：见库根目录 Makefile，执行 make test
 *
 * 重点验证用户最关心的两件事：
 *   1) 「如何向外设发送控制消息」——断言 port 层实际收到的 PWM / 脉宽 / 串口字节
 *   2) 「如何接收外设送来的数据」——注入 GPIO 电平序列与串口字节流，断言解析结果
 */
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "clib_common.h"
#include "clib_port.h"
#include "clib_frame.h"
#include "clib_servo.h"
#include "clib_motor.h"
#include "clib_ir_remote.h"
#include "clib_ir_sensor.h"
#include "clib_ultrasonic.h"
#include "clib_vision_k210.h"
#include "clib_cmd.h"
#include "clib_port_mock.h"

static int g_pass = 0;
static int g_fail = 0;

#define CHECK(cond, msg) do {                                                   \
    if (cond) { g_pass++; }                                                     \
    else { g_fail++; printf("  FAIL %-28s (line %d): %s\n", __func__, __LINE__, (msg)); } \
} while (0)

#define CHECK_EQ(a, b, msg) do {                                                \
    long _va = (long)(a); long _vb = (long)(b);                                 \
    if (_va == _vb) { g_pass++; }                                               \
    else { g_fail++; printf("  FAIL %-28s (line %d): %s | got %ld, want %ld\n", \
                            __func__, __LINE__, (msg), _va, _vb); }         \
} while (0)

static void section(const char *name)
{
    printf("[ %s ]\n", name);
}

/* ---------- 辅助：造一串红外脉冲 ---------- */
static int feed_ir_code(clib_irdec_t *d, uint32_t raw, uint32_t *code_out)
{
    int i;
    int r = 0;
    (void)clib_irdec_feed_tick(d, 220u, code_out);      /* 引导码 */
    for (i = 31; i >= 0; --i) {
        uint8_t tick = ((raw >> i) & 1u) ? 70u : 30u;
        r = clib_irdec_feed_tick(d, tick, code_out);
    }
    return r;
}

static void test_common_and_port(void)
{
    clib_port_t bad;
    section("common / port");
    CHECK(strcmp(clib_version(), "1.0.0") == 0, "version");
    CHECK(clib_err_str(CLIB_ERR_TIMEOUT) != NULL, "err_str");
    CHECK(strcmp(clib_act_name(CLIB_ACT_FORWARD), "FORWARD") == 0, "act_name");
    CHECK_EQ(clib_clamp_i16(200, 0, 100), 100, "clamp_i16 hi");
    CHECK_EQ(clib_clamp_i16(-5, 0, 100), 0, "clamp_i16 lo");
    CHECK_EQ(clib_clamp_u8(999, 0, 100), 100, "clamp_u8");

    clib_mock_reset();
    clib_deinit();
    CHECK(!clib_is_ready(), "not ready after deinit");
    CHECK_EQ(clib_init(NULL), CLIB_ERR_NULL, "init NULL");
    memset(&bad, 0, sizeof(bad));
    CHECK_EQ(clib_init(&bad), CLIB_ERR_NO_PORT, "init incomplete port");
    CHECK_EQ(clib_init(clib_mock_port()), CLIB_OK, "init mock port");
    CHECK(clib_is_ready(), "ready after init");
    CHECK_EQ(clib_gpio_read(CLIB_PIN_IR_REMOTE), 1, "IR idle high");
}

static void test_servo(void)
{
    clib_servo_cal_t cal = { 600u, 2400u, 0.0f, 180.0f };
    static const float scan_angles[3] = { 90.0f, 175.0f, 5.0f };
    clib_servo_scan_t scan;
    float a = -1.0f;

    section("servo (send control message)");
    CHECK_EQ(clib_servo_angle_to_pulse(0.0f), 500u, "0deg -> 500us");
    CHECK_EQ(clib_servo_angle_to_pulse(90.0f), 1500u, "90deg -> 1500us");
    CHECK_EQ(clib_servo_angle_to_pulse(180.0f), 2500u, "180deg -> 2500us");
    CHECK_EQ(clib_servo_angle_to_pulse(-20.0f), 500u, "negative clamped");
    CHECK_EQ(clib_servo_angle_to_pulse(999.0f), 2500u, ">180 clamped");
    CHECK_EQ(clib_servo_angle_to_pulse_cal(90.0f, &cal), 1500u, "custom cal mid");
    CHECK(fabsf(clib_servo_pulse_to_angle(1500u, NULL) - 90.0f) < 0.01f, "inverse");
    CHECK_EQ(clib_servo_clamp_pulse(100u), 500u, "clamp low");
    CHECK_EQ(clib_servo_clamp_pulse(9000u), 2500u, "clamp high");

    clib_mock_reset();
    (void)clib_init(clib_mock_port());
    CHECK_EQ(clib_servo_set_angle(90.0f), CLIB_OK, "set_angle ok");
    CHECK_EQ(g_mock.servo_pulse_us, 1500u, "port got 1500us");
    CHECK_EQ(g_mock.servo_write_count, 1u, "one servo write");
    CHECK_EQ(clib_servo_set_pulse(3000u), CLIB_OK, "set_pulse ok");
    CHECK_EQ(g_mock.servo_pulse_us, 2500u, "pulse clamped by lib");

    /* 非阻塞三向扫描：不阻塞、角度按序、结束后不重复触发 */
    CHECK_EQ(clib_servo_scan_begin(&scan, scan_angles, 3u, 100u), CLIB_OK, "scan begin");
    CHECK_EQ(g_mock.servo_pulse_us, 1500u, "scan first angle 90");
    CHECK_EQ(clib_servo_scan_tick(&scan, &a), 1, "scan tick#1");
    CHECK(fabsf(a - 90.0f) < 0.01f, "scan angle#1");
    clib_mock_advance_ms(50u);
    CHECK_EQ(clib_servo_scan_tick(&scan, &a), 0, "scan settling");
    clib_mock_advance_ms(60u);
    CHECK_EQ(clib_servo_scan_tick(&scan, &a), 1, "scan tick#2");
    CHECK(fabsf(a - 175.0f) < 0.01f, "scan angle#2");
    clib_mock_advance_ms(110u);
    CHECK_EQ(clib_servo_scan_tick(&scan, &a), 1, "scan tick#3");
    CHECK(fabsf(a - 5.0f) < 0.01f, "scan angle#3");
    clib_mock_advance_ms(110u);
    CHECK_EQ(clib_servo_scan_tick(&scan, &a), -1, "scan done");
    CHECK(clib_servo_scan_done(&scan), "scan done flag");
}

static void test_motor(void)
{
    uint8_t d[4];

    section("motor (send control message)");
    clib_diff_map(CLIB_ACT_FORWARD, 70, d);
    CHECK(d[0] == 70 && d[1] == 0 && d[2] == 70 && d[3] == 0, "forward map");
    clib_diff_map(CLIB_ACT_BACKWARD, 70, d);
    CHECK(d[0] == 0 && d[1] == 70 && d[2] == 0 && d[3] == 70, "backward map");
    clib_diff_map(CLIB_ACT_LEFT, 70, d);
    CHECK(d[0] == 0 && d[1] == 0 && d[2] == 70 && d[3] == 0, "left map");
    clib_diff_map(CLIB_ACT_RIGHT, 70, d);
    CHECK(d[0] == 70 && d[1] == 0 && d[2] == 0 && d[3] == 0, "right map");
    clib_diff_map(CLIB_ACT_SPIN_LEFT, 70, d);
    CHECK(d[0] == 0 && d[1] == 70 && d[2] == 70 && d[3] == 0, "spin_left map");
    clib_diff_map(CLIB_ACT_SPIN_RIGHT, 70, d);
    CHECK(d[0] == 70 && d[1] == 0 && d[2] == 0 && d[3] == 70, "spin_right map");
    clib_diff_map(CLIB_ACT_FORWARD, 150, d);
    CHECK_EQ(d[0], 100u, "speed clamped to 100");
    clib_diff_map(CLIB_ACT_FORWARD, -7, d);
    CHECK_EQ(d[0], 0u, "negative speed clamped to 0");

    clib_mock_reset();
    (void)clib_init(clib_mock_port());
    CHECK_EQ(clib_motor_set(CLIB_ACT_FORWARD, 70), CLIB_OK, "motor set ok");
    CHECK(g_mock.motor_duty[0] == 70 && g_mock.motor_duty[1] == 0 &&
          g_mock.motor_duty[2] == 70 && g_mock.motor_duty[3] == 0, "port got 4 duties");
    /* 关键改进：无自动停车、无阻塞 */
    clib_mock_advance_ms(5000u);
    CHECK_EQ(g_mock.motor_duty[0], 70u, "duty persists, no auto-stop");
    CHECK_EQ(g_mock.motor_write_count, 4u, "no extra writes while idle");
    CHECK_EQ(clib_motor_stop(), CLIB_OK, "explicit stop");
    CHECK(g_mock.motor_duty[0] == 0 && g_mock.motor_duty[2] == 0, "stop writes zeros");
}

static void test_ir_remote(void)
{
    clib_irdec_t d;
    clib_ir_sampler_t s;
    uint32_t code = 0u;
    uint32_t raw = 0x12345678u;
    uint8_t hi = 0u, lo = 0u;

    section("IR remote (receive data)");
    clib_irdec_init(&d);
    CHECK_EQ(feed_ir_code(&d, raw, &code), 1, "decode 32 bits");
    CHECK_EQ(code, (long)raw, "decoded code equals raw");
    CHECK_EQ(d.frames_ok, 1u, "frame counter");
    CHECK_EQ(d.state, (long)CLIB_IRDEC_SEEK_LEADER, "auto reset for next frame");

    /* 无效脉宽不得推进 bit（原厂 Data 未初始化的 bug） */
    clib_irdec_init(&d);
    (void)clib_irdec_feed_tick(&d, 220u, &code);
    (void)clib_irdec_feed_tick(&d, 100u, &code);   /* 落在 90~200 的空档 */
    CHECK_EQ(d.bits, 0u, "invalid pulse ignored");
    CHECK_EQ(feed_ir_code(&d, raw, &code), 1, "still decodes after noise");
    CHECK_EQ(code, (long)raw, "code after noise");

    /* 超时复位 */
    clib_irdec_init(&d);
    (void)clib_irdec_feed_tick(&d, 220u, &code);
    CHECK_EQ(clib_irdec_feed_tick(&d, 250u, &code), CLIB_ERR_TIMEOUT, "timeout");
    CHECK_EQ(d.state, (long)CLIB_IRDEC_SEEK_LEADER, "reset on timeout");

    /* 微秒接口 */
    clib_irdec_init(&d);
    (void)clib_irdec_feed_pulse(&d, 4400u, &code);   /* 220 ticks */
    CHECK_EQ(clib_irdec_feed_pulse(&d, 1400u, &code), 0, "70 ticks as 1400us");
    CHECK_EQ(clib_irdec_feed_pulse(&d, 600u, &code), 0, "30 ticks as 600us");

    /* 键码查表（两套码表都与原案例一致） */
    CHECK_EQ(clib_ir_data_code(0x0062u << 8), 0x62u, "data_code shift");
    clib_ir_split_key(0x62u, &hi, &lo);
    CHECK(hi == 6u && lo == 2u, "split 0x62 -> 6,2");
    CHECK_EQ(clib_ir_lookup(0x62u << 8, CLIB_IR_KEYMAP_V1, CLIB_IR_KEYMAP_V1_LEN),
             (long)CLIB_ACT_FORWARD, "V1 forward");
    CHECK_EQ(clib_ir_lookup(0xA8u << 8, CLIB_IR_KEYMAP_V1, CLIB_IR_KEYMAP_V1_LEN),
             (long)CLIB_ACT_BACKWARD, "V1 back");
    CHECK_EQ(clib_ir_lookup(0xB1u << 8, CLIB_IR_KEYMAP_V2, CLIB_IR_KEYMAP_V2_LEN),
             (long)CLIB_ACT_FORWARD, "V2 forward");
    CHECK_EQ(clib_ir_lookup(0xE1u << 8, CLIB_IR_KEYMAP_V2, CLIB_IR_KEYMAP_V2_LEN),
             (long)CLIB_ACT_SPIN_RIGHT, "V2 spin right");
    CHECK_EQ(clib_ir_lookup(0x00u, CLIB_IR_KEYMAP_V1, CLIB_IR_KEYMAP_V1_LEN),
             (long)CLIB_ACT_STOP, "unknown key stops");

    /* 无输入捕获时的 GPIO 采样降级方案 */
    clib_mock_reset();
    (void)clib_init(clib_mock_port());
    clib_ir_sampler_init(&s);
    code = 0u;
    clib_mock_set_time_us(1000u);
    clib_mock_set_gpio(CLIB_PIN_IR_REMOTE, 1);
    CHECK_EQ(clib_ir_sampler_poll(&s, 1000u, &code), 0, "sampler rise detected");
    clib_mock_set_gpio(CLIB_PIN_IR_REMOTE, 0);
    CHECK_EQ(clib_ir_sampler_poll(&s, 5400u, &code), 0, "sampler leader pulse 4400us");
    clib_mock_set_gpio(CLIB_PIN_IR_REMOTE, 1);
    (void)clib_ir_sampler_poll(&s, 5400u, &code);
    clib_mock_set_gpio(CLIB_PIN_IR_REMOTE, 0);
    CHECK_EQ(clib_ir_sampler_poll(&s, 6800u, &code), 0, "sampler bit1 1400us");
    CHECK_EQ(s.dec.bits, 1u, "sampler pushed 1 bit");
}

static void test_ir_sensor(void)
{
    clib_ir_sensor_t trk;
    clib_ir_sensor_t obs;

    section("IR sensor (receive data)");
    clib_mock_reset();
    (void)clib_init(clib_mock_port());
    CHECK_EQ(clib_ir_sensor_init(&trk, CLIB_PIN_TRACK_L, CLIB_PIN_TRACK_R, false, 20u),
             CLIB_OK, "track init");
    CHECK_EQ(clib_ir_sensor_init(&obs, CLIB_PIN_OBST_L, CLIB_PIN_OBST_R, false, 0u),
             CLIB_OK, "obstacle init");

    CHECK_EQ(clib_ir_sensor_read_raw(&trk), 0u, "raw none");
    clib_mock_set_gpio(CLIB_PIN_TRACK_L, 1);
    CHECK_EQ(clib_ir_sensor_read_raw(&trk), 2u, "raw left bit");
    clib_mock_set_gpio(CLIB_PIN_TRACK_R, 1);
    CHECK_EQ(clib_ir_sensor_read_raw(&trk), 3u, "raw both");

    /* 去抖：状态变了要等 debounce_ms 才生效 */
    clib_mock_set_gpio(CLIB_PIN_TRACK_L, 0);
    CHECK_EQ(clib_ir_sensor_read_at(&trk, 0u), 0u, "debounce pending");
    CHECK_EQ(clib_ir_sensor_read_at(&trk, 10u), 0u, "debounce not yet");
    CHECK_EQ(clib_ir_sensor_read_at(&trk, 25u), 1u, "debounce settled");

    CHECK_EQ(clib_track_decide(0u), (long)CLIB_ACT_FORWARD, "track 00");
    CHECK_EQ(clib_track_decide(2u), (long)CLIB_ACT_LEFT, "track 10");
    CHECK_EQ(clib_track_decide(1u), (long)CLIB_ACT_RIGHT, "track 01");
    CHECK_EQ(clib_track_decide(3u), (long)CLIB_ACT_STOP, "track 11");
    CHECK_EQ(clib_avoid_decide(3u), (long)CLIB_ACT_FORWARD, "avoid 11");
    CHECK_EQ(clib_avoid_decide(2u), (long)CLIB_ACT_LEFT, "avoid 10");
    CHECK_EQ(clib_avoid_decide(1u), (long)CLIB_ACT_RIGHT, "avoid 01");
    CHECK_EQ(clib_avoid_decide(0u), (long)CLIB_ACT_SPIN_RIGHT, "avoid 00");

    /* active_low 极性 */
    CHECK_EQ(clib_ir_sensor_init(&trk, CLIB_PIN_TRACK_L, CLIB_PIN_TRACK_R, true, 0u),
             CLIB_OK, "active_low init");
    clib_mock_set_gpio(CLIB_PIN_TRACK_L, 0);
    clib_mock_set_gpio(CLIB_PIN_TRACK_R, 1);
    CHECK_EQ(clib_ir_sensor_read_raw(&trk), 2u, "active_low polarity");
    (void)obs;
}

static void test_ultrasonic(void)
{
    clib_ultra_t u;
    float cm = 0.0f;

    section("ultrasonic (trigger + echo)");
    CHECK(fabsf(clib_echo_us_to_cm(583u) - 10.0f) < 0.05f, "583us ~ 10cm");
    CHECK_EQ(clib_echo_us_to_mm(583u), 100u, "583us ~ 100mm");
    CHECK(fabsf(clib_echo_count_to_cm(100u, 5u) - 8.575f) < 0.05f, "count->cm");
    CHECK(fabsf(clib_echo_count_to_legacy(100u) - 85.0f) < 0.01f, "legacy Distance");
    /* 原 StartMeasure() 返回 Distance*10，案例 9 的阈值 60 判的是这个量 */
    CHECK(fabsf(clib_echo_legacy_measure(100u) - 850.0f) < 0.05f, "legacy StartMeasure");
    CHECK_EQ(clib_echo_count_to_cm(10u, 0u), (long)0, "zero us_per_count guard");
    /* 极端入参不得触发 float->uint32 的未定义行为 */
    CHECK(fabsf(clib_echo_count_to_cm(UINT32_MAX, 1000u) - CLIB_ULTRA_MAX_CM) < 0.01f,
          "count overflow saturated");

    clib_mock_reset();
    (void)clib_init(clib_mock_port());
    CHECK_EQ(clib_ultra_trigger(), CLIB_OK, "trigger ok");
    CHECK_EQ(g_mock.gpio_write_count, 2u, "trigger wrote high+low");
    CHECK_EQ(g_mock.gpio[CLIB_PIN_ULTRA_TRIG], 0, "trig line back low");

    clib_mock_reset();
    (void)clib_init(clib_mock_port());
    clib_ultra_start(&u, 30000u);
    CHECK_EQ(u.state, (long)CLIB_ULTRA_WAIT_RISE, "state wait rise");
    clib_mock_set_time_us(1000u);
    clib_mock_set_gpio(CLIB_PIN_ULTRA_ECHO, 1);
    CHECK_EQ(clib_ultra_poll_at(&u, 1100u, &cm), 0, "echo rising");
    CHECK_EQ(u.state, (long)CLIB_ULTRA_WAIT_FALL, "state wait fall");
    clib_mock_set_gpio(CLIB_PIN_ULTRA_ECHO, 0);
    CHECK_EQ(clib_ultra_poll_at(&u, 1683u, &cm), 1, "echo falling -> distance");
    CHECK(fabsf(cm - 10.0f) < 0.05f, "measured ~10cm");
    CHECK_EQ(clib_ultra_poll_at(&u, 1700u, &cm), CLIB_ERR_BUSY, "done state");

    /* 超时保护（原厂代码会在中断里死等） */
    clib_ultra_start(&u, 30000u);
    clib_mock_set_gpio(CLIB_PIN_ULTRA_ECHO, 0);
    CHECK_EQ(clib_ultra_poll_at(&u, 40000u, &cm), CLIB_ERR_TIMEOUT, "timeout protected");
}

static void test_k210(void)
{
    clib_k210_t k;
    uint8_t frame[CLIB_K210_FRAME_LEN];
    uint8_t tx[16];
    uint16_t n;
    int16_t angle = 0;
    int i;
    int r = 0;

    section("K210 vision frame (receive data)");
    CHECK_EQ(clib_k210_build(53, frame), (int)CLIB_K210_FRAME_LEN, "build 53");
    CHECK(frame[0] == 0xAA && frame[1] == 0xAE && frame[2] == 53 &&
          frame[3] == 0x00 && frame[4] == 0x5B, "build 53 bytes");
    CHECK_EQ(clib_k210_build(-10, frame), (int)CLIB_K210_FRAME_LEN, "build -10");
    CHECK(frame[2] == 0xF6 && frame[3] == 0xFF, "negative little endian");

    clib_k210_init(&k);
    (void)clib_k210_build(-10, frame);
    for (i = 0; i < (int)CLIB_K210_FRAME_LEN; ++i) {
        r = clib_k210_feed(&k, frame[i], &angle);
    }
    CHECK_EQ(r, 1, "full frame parsed");
    CHECK_EQ(angle, -10, "angle -10");
    CHECK_EQ(k.frames_ok, 1u, "frame counter");

    /* 坏帧尾 */
    clib_k210_init(&k);
    (void)clib_k210_build(12, frame);
    frame[4] = 0x00;
    r = 0;
    for (i = 0; i < (int)CLIB_K210_FRAME_LEN; ++i) {
        r = clib_k210_feed(&k, frame[i], &angle);
    }
    CHECK_EQ(r, CLIB_ERR_FORMAT, "bad tail rejected");
    CHECK_EQ(k.frames_err, 1u, "err counter");

    /* 帧头重叠 AA AA AE */
    clib_k210_init(&k);
    (void)clib_k210_build(20, frame);
    (void)clib_k210_feed(&k, 0xAA, &angle);
    (void)clib_k210_feed(&k, 0xAA, &angle);
    r = 0;
    for (i = 1; i < (int)CLIB_K210_FRAME_LEN; ++i) {
        r = clib_k210_feed(&k, frame[i], &angle);
    }
    CHECK_EQ(r, 1, "overlapping head recovered");
    CHECK_EQ(angle, 20, "angle after overlap");

    /* 从串口收 */
    clib_mock_reset();
    (void)clib_init(clib_mock_port());
    clib_k210_init(&k);
    (void)clib_k210_build(-25, frame);
    clib_mock_push_rx(CLIB_UART_VISION, frame, CLIB_K210_FRAME_LEN);
    CHECK_EQ(clib_k210_poll(CLIB_UART_VISION, &k, &angle), 1, "poll from uart");
    CHECK_EQ(angle, -25, "polled angle");
    CHECK_EQ(clib_k210_poll(CLIB_UART_VISION, &k, &angle), 0, "uart empty");

    CHECK(clib_k210_is_lost(53), "lost flag 53");
    CHECK(!clib_k210_is_lost(52), "not lost at 52");
    CHECK_EQ(clib_k210_decide(0, 10), (long)CLIB_ACT_STOP, "decide stop");
    CHECK_EQ(clib_k210_decide(20, 10), (long)CLIB_ACT_RIGHT, "decide right");
    CHECK_EQ(clib_k210_decide(-20, 10), (long)CLIB_ACT_LEFT, "decide left");
    CHECK_EQ(clib_k210_decide(5, 10), (long)CLIB_ACT_FORWARD, "decide forward");
    CHECK_EQ(clib_k210_decide(53, 10), (long)CLIB_ACT_BACKWARD, "decide lost");

    /* 发送侧：构造帧写串口 */
    clib_mock_clear_tx(CLIB_UART_VISION);
    (void)clib_k210_build(7, frame);
    CHECK_EQ(clib_uart_write(CLIB_UART_VISION, frame, CLIB_K210_FRAME_LEN),
             (int)CLIB_K210_FRAME_LEN, "uart write");
    n = clib_mock_take_tx(CLIB_UART_VISION, tx, sizeof(tx));
    CHECK_EQ(n, (int)CLIB_K210_FRAME_LEN, "tx length");
    CHECK(memcmp(tx, frame, CLIB_K210_FRAME_LEN) == 0, "tx bytes match");

    /* 持续噪声下 poll 必须按预算返回，不能活锁 */
    {
        uint8_t noise[64];
        memset(noise, 0x00, sizeof(noise));
        clib_k210_init(&k);
        /* 清掉之前注入的 RX */
        while (clib_uart_read(CLIB_UART_VISION, tx, 1u) > 0) { }
        clib_mock_push_rx(CLIB_UART_VISION, noise, (uint16_t)sizeof(noise));
        CHECK_EQ(clib_k210_poll(CLIB_UART_VISION, &k, &angle), 0, "k210 noise poll bounded");
    }
}

static void test_frame(void)
{
    clib_frame_cfg_t fixed;
    clib_frame_cfg_t text;
    clib_frame_t f;
    uint8_t frame[16];
    uint8_t payload[16];
    uint8_t pl = 0u;
    uint8_t tx[32];
    uint16_t n;
    int built;
    int r = 0;
    int i;
    static const uint8_t pl2[2] = { 0x35u, 0x00u };

    section("generic frame (build / feed / poll)");
    memset(&fixed, 0, sizeof(fixed));
    fixed.head[0] = 0xAA; fixed.head[1] = 0xAE; fixed.head_len = 2u;
    fixed.tail[0] = 0x5B; fixed.tail_len = 1u;
    fixed.max_payload = 2u;
    fixed.fixed_len = 5u;
    fixed.checksum = CLIB_CHECKSUM_NONE;

    built = clib_frame_build(&fixed, pl2, 2u, frame, sizeof(frame));
    CHECK_EQ(built, 5, "fixed build len");
    CHECK(frame[0] == 0xAA && frame[1] == 0xAE && frame[2] == 0x35 &&
          frame[3] == 0x00 && frame[4] == 0x5B, "fixed build bytes");

    CHECK_EQ(clib_frame_init(&f, &fixed, 0u), CLIB_OK, "fixed init");
    for (i = 0; i < built; ++i) {
        r = clib_frame_feed(&f, frame[i], payload, (uint8_t)sizeof(payload), &pl);
    }
    CHECK_EQ(r, 1, "fixed feed complete");
    CHECK_EQ(pl, 2u, "fixed payload len");
    CHECK(payload[0] == 0x35 && payload[1] == 0x00, "fixed payload bytes");

    /* 变长 + 校验 */
    memset(&text, 0, sizeof(text));
    text.head[0] = 0xAA; text.head_len = 1u;
    text.tail[0] = '\r'; text.tail[1] = '\n'; text.tail_len = 2u;
    text.max_payload = 16u;
    text.fixed_len = 0u;
    text.checksum = CLIB_CHECKSUM_XOR;
    built = clib_frame_build(&text, (const uint8_t *)"ABC", 3u, frame, sizeof(frame));
    CHECK_EQ(built, 7, "text build len");
    CHECK_EQ(frame[4], (long)clib_checksum_compute(CLIB_CHECKSUM_XOR,
                                                   (const uint8_t *)"ABC", 3u),
             "checksum placed");
    CHECK_EQ(clib_frame_init(&f, &text, 1000u), CLIB_OK, "text init");
    pl = 0u;
    for (i = 0; i < built; ++i) {
        r = clib_frame_feed(&f, frame[i], payload, (uint8_t)sizeof(payload), &pl);
    }
    CHECK_EQ(r, 1, "text feed complete");
    CHECK_EQ(pl, 3u, "text payload len");
    CHECK(memcmp(payload, "ABC", 3) == 0, "text payload bytes");

    /* 校验错必须被拒绝 */
    frame[2] = 'Z';
    CHECK_EQ(clib_frame_init(&f, &text, 0u), CLIB_OK, "text re-init");
    r = 0;
    for (i = 0; i < built; ++i) {
        r = clib_frame_feed(&f, frame[i], payload, (uint8_t)sizeof(payload), &pl);
    }
    CHECK_EQ(r, CLIB_ERR_CHECKSUM, "bad checksum rejected");

    /* 组帧发送 / 串口收帧 */
    clib_mock_reset();
    (void)clib_init(clib_mock_port());
    clib_mock_clear_tx(CLIB_UART_BT);
    CHECK_EQ(clib_frame_send(CLIB_UART_BT, &fixed, pl2, 2u), CLIB_OK, "frame_send");
    n = clib_mock_take_tx(CLIB_UART_BT, tx, sizeof(tx));
    CHECK_EQ(n, 5u, "frame_send tx len");

    clib_mock_push_rx(CLIB_UART_BT, tx, n);
    CHECK_EQ(clib_frame_init(&f, &fixed, 0u), CLIB_OK, "poll init");
    r = 0;
    for (i = 0; i < (int)n; ++i) {
        uint8_t b = tx[i];
        r = clib_frame_feed(&f, b, payload, (uint8_t)sizeof(payload), &pl);
    }
    CHECK_EQ(r, 1, "roundtrip via feed");
    clib_mock_push_rx(CLIB_UART_BT, tx, n);
    CHECK_EQ(clib_frame_init(&f, &fixed, 0u), CLIB_OK, "poll init2");
    CHECK_EQ(clib_frame_poll(CLIB_UART_BT, &f, payload,
                             (uint8_t)sizeof(payload), &pl), 1, "poll from uart");
    CHECK_EQ(pl, 2u, "poll payload len");

    /* 输出缓冲容量不足必须报错，绝不越界写 */
    {
        uint8_t tiny[1];
        CHECK_EQ(clib_frame_init(&f, &fixed, 0u), CLIB_OK, "cap test init");
        r = 0;
        for (i = 0; i < (int)n; ++i) {
            r = clib_frame_feed(&f, tx[i], tiny, (uint8_t)sizeof(tiny), &pl);
        }
        CHECK_EQ(r, CLIB_ERR_OVERFLOW, "payload_cap respected");
    }

    /* 批量喂入（feed_buf） */
    {
        uint8_t out[16]; uint8_t olen = 0u;
        CHECK_EQ(clib_frame_init(&f, &fixed, 0u), CLIB_OK, "feed_buf init");
        CHECK_EQ(clib_frame_feed_buf(&f, tx, n, out, (uint8_t)sizeof(out), &olen),
                 1, "feed_buf complete");
        CHECK_EQ(olen, 2u, "feed_buf payload len");
    }

    /* 串口持续噪声：poll 必须有字节预算、按时返回（不能活锁） */
    {
        uint8_t noise[64];
        uint8_t out[16]; uint8_t olen = 0u;
        memset(noise, 0xAA, sizeof(noise));
        clib_mock_clear_tx(CLIB_UART_BT);
        /* 先把上一批残留的 RX 字节读空，保证本用例只面对噪声 */
        while (clib_uart_read(CLIB_UART_BT, out, 1u) > 0) { }
        clib_mock_push_rx(CLIB_UART_BT, noise, (uint16_t)sizeof(noise));
        CHECK_EQ(clib_frame_init(&f, &fixed, 0u), CLIB_OK, "noise init");
        CHECK_EQ(clib_frame_poll(CLIB_UART_BT, &f, out,
                                 (uint8_t)sizeof(out), &olen), 0, "noise poll bounded");
    }
}

static void test_cmd(void)
{
    uint8_t tx[64];
    uint16_t n;
    int16_t x = 0, y = 0;

    section("bluetooth / serial commands");
    CHECK_EQ(clib_cmd_parse("ONA"), (long)CLIB_CMD_FORWARD, "ONA");
    CHECK_EQ(clib_cmd_parse("ona"), (long)CLIB_CMD_FORWARD, "case insensitive");
    CHECK_EQ(clib_cmd_parse("  ONB\r\n"), (long)CLIB_CMD_BACKWARD, "trim CRLF");
    CHECK_EQ(clib_cmd_parse("ONC"), (long)CLIB_CMD_LEFT, "ONC");
    CHECK_EQ(clib_cmd_parse("OND"), (long)CLIB_CMD_RIGHT, "OND");
    CHECK_EQ(clib_cmd_parse("ONE"), (long)CLIB_CMD_STOP, "ONE");
    CHECK_EQ(clib_cmd_parse("ONF"), (long)CLIB_CMD_STOP, "ONF");
    CHECK_EQ(clib_cmd_parse("MODE:NAV"), (long)CLIB_CMD_MODE_NAV, "MODE:NAV");
    CHECK_EQ(clib_cmd_parse("GOTO 12,-3"), (long)CLIB_CMD_GOTO, "GOTO token");
    CHECK_EQ(clib_cmd_parse("?"), (long)CLIB_CMD_QUERY, "query");
    CHECK_EQ(clib_cmd_parse("WAT"), (long)CLIB_CMD_UNKNOWN, "unknown");
    CHECK_EQ(clib_cmd_parse(""), (long)CLIB_CMD_NONE, "empty");
    CHECK_EQ(clib_cmd_parse(NULL), (long)CLIB_CMD_NONE, "null");
    CHECK_EQ(clib_cmd_to_act(CLIB_CMD_SPIN_LEFT), (long)CLIB_ACT_SPIN_LEFT, "cmd->act");
    CHECK(strcmp(clib_cmd_name(CLIB_CMD_FORWARD), "FORWARD") == 0, "cmd name");

    CHECK(clib_cmd_parse_goto("GOTO 12,-3", &x, &y), "goto parse");
    CHECK(x == 12 && y == -3, "goto values");
    CHECK(clib_cmd_parse_goto("goto  100 , 200", &x, &y), "goto spaces");
    CHECK(x == 100 && y == 200, "goto values 2");
    CHECK(!clib_cmd_parse_goto("GOTO 40000,1", &x, &y), "goto overflow rejected");
    CHECK(!clib_cmd_parse_goto("STOP", &x, &y), "goto bad prefix");
    CHECK(!clib_cmd_parse_goto("GOTO 12,-3junk", &x, &y), "goto trailing junk rejected");
    CHECK(!clib_cmd_parse_goto("GOTO 99999999999999999999,1", &x, &y),
          "goto ERANGE rejected");

    clib_mock_reset();
    (void)clib_init(clib_mock_port());
    CHECK_EQ(clib_cmd_send(CLIB_UART_BT, "ONA"), CLIB_OK, "cmd_send");
    n = clib_mock_take_tx(CLIB_UART_BT, tx, sizeof(tx));
    CHECK_EQ(n, 3u, "cmd_send length");
    CHECK(memcmp(tx, "ONA", 3) == 0, "cmd_send bytes");
    CHECK_EQ(clib_cmd_send_ack(CLIB_UART_BT, CLIB_CMD_FORWARD, 0), CLIB_OK, "ack send");
    CHECK(strstr((const char *)g_mock.tx[CLIB_UART_BT], "ACK FORWARD 0") != NULL,
          "ack content");
}

static void test_end_to_end(void)
{
    uint32_t code = 0u;
    clib_irdec_t d;
    uint8_t frame[CLIB_K210_FRAME_LEN];
    int16_t angle = 0;
    clib_k210_t k;

    section("end-to-end: peripheral data -> control message");
    clib_mock_reset();
    (void)clib_init(clib_mock_port());

    /* 红外遥控：解码 -> 查表 -> 差速电机输出 */
    clib_irdec_init(&d);
    CHECK_EQ(feed_ir_code(&d, 0x62u << 8, &code), 1, "ir decoded");
    CHECK_EQ(clib_motor_set(clib_ir_lookup(code, CLIB_IR_KEYMAP_V1,
                                           CLIB_IR_KEYMAP_V1_LEN), 70),
             CLIB_OK, "motor set from ir");
    CHECK(g_mock.motor_duty[0] == 70 && g_mock.motor_duty[2] == 70, "ir -> forward PWM");

    /* K210：串口字节 -> 帧解析 -> 决策 -> 差速电机输出 */
    clib_k210_init(&k);
    (void)clib_k210_build(25, frame);
    clib_mock_push_rx(CLIB_UART_VISION, frame, CLIB_K210_FRAME_LEN);
    CHECK_EQ(clib_k210_poll(CLIB_UART_VISION, &k, &angle), 1, "vision frame");
    CHECK_EQ(clib_motor_set(clib_k210_decide(angle, 10), 60), CLIB_OK, "motor from vision");
    CHECK_EQ(g_mock.motor_duty[0], 60u, "vision -> right PWM ch0");
    CHECK_EQ(g_mock.motor_duty[2], 0u, "vision -> right PWM ch2 off");

    /* 舵机三向扫描 + 超声波测距联动 */
    CHECK_EQ(clib_servo_set_angle(90.0f), CLIB_OK, "servo front");
    CHECK_EQ(g_mock.servo_pulse_us, 1500u, "servo front pulse");
    CHECK_EQ(clib_ultra_trigger(), CLIB_OK, "ultra trigger");
    CHECK_EQ(g_mock.gpio_write_count, 2u, "ultra pulse emitted");
}

int main(void)
{
    printf("=== clib unit tests (build %s) ===\n", clib_version());
    test_common_and_port();
    test_servo();
    test_motor();
    test_ir_remote();
    test_ir_sensor();
    test_ultrasonic();
    test_k210();
    test_frame();
    test_cmd();
    test_end_to_end();
    printf("\n=== %d passed, %d failed ===\n", g_pass, g_fail);
    return (g_fail == 0) ? 0 : 1;
}
