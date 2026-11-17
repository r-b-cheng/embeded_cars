/*
 * test_wheel.c -- PC 端逻辑自测，不需要开发板
 * 用 ../library/ports/mock 的假硬件，断言 Wheel_Set() 发给 L298N 的 4 路占空比。
 * 顺序固定为 [IN1, IN2, IN3, IN4] = [左正, 左反, 右正, 右反]。
 */
#include <stdio.h>
#include "wheel.h"
#include "clib_motor.h"
#include "clib_port_mock.h"

static int g_fail = 0;

#define EXPECT(expr, want)                                                   \
    do {                                                                     \
        if ((expr) != (want)) {                                              \
            printf("FAIL line %d: %s\n", __LINE__, #expr);                   \
            g_fail++;                                                        \
        }                                                                    \
    } while (0)

static void expect_duty(const char *name,
                        uint8_t a, uint8_t b, uint8_t c, uint8_t d)
{
    const uint8_t want[4] = { a, b, c, d };
    int i;
    printf("%-24s -> IN1..IN4 = %3u %3u %3u %3u", name,
           g_mock.motor_duty[0], g_mock.motor_duty[1],
           g_mock.motor_duty[2], g_mock.motor_duty[3]);
    for (i = 0; i < 4; ++i) {
        if (g_mock.motor_duty[i] != want[i]) {
            printf("   FAIL (want %u %u %u %u)", a, b, c, d);
            g_fail++;
            break;
        }
    }
    printf("\n");
}

int main(void)
{
    clib_mock_reset();
    EXPECT(Wheel_Init(NULL), CLIB_ERR_NULL);      /* 空 port 必须报错 */
    EXPECT(Wheel_Init(clib_mock_port()), CLIB_OK);
    expect_duty("init (must be stopped)", 0, 0, 0, 0);

    EXPECT(Wheel_Set(60, 60), CLIB_OK);
    expect_duty("forward 60/60",       60,  0, 60,  0);

    EXPECT(Wheel_Set(-40, -40), CLIB_OK);
    expect_duty("backward -40/-40",     0, 40,  0, 40);

    EXPECT(Wheel_Set(0, 70), CLIB_OK);
    expect_duty("turn left 0/70",       0,  0, 70,  0);

    EXPECT(Wheel_Set(50, 0), CLIB_OK);
    expect_duty("turn right 50/0",     50,  0,  0,  0);

    EXPECT(Wheel_Set(-50, 50), CLIB_OK);
    expect_duty("spin left -50/50",     0, 50, 50,  0);

    EXPECT(Wheel_Set(50, -50), CLIB_OK);
    expect_duty("spin right 50/-50",   50,  0,  0, 50);

    EXPECT(Wheel_Set(999, -999), CLIB_OK);
    expect_duty("clamp 999/-999",     100,  0,  0,100);

    EXPECT(Wheel_Stop(), CLIB_OK);
    expect_duty("stop",                 0,  0,  0,  0);

    EXPECT(Wheel_Brake(), CLIB_OK);
    expect_duty("brake",             100,100,100,100);

    /* 与库原生动作映射必须一致，证明没有绕开 clib_motor 的语义 */
    EXPECT(clib_motor_set(CLIB_ACT_FORWARD, 70), CLIB_OK);
    expect_duty("clib FORWARD 70",     70,  0, 70,  0);
    EXPECT(clib_motor_set(CLIB_ACT_SPIN_LEFT, 35), CLIB_OK);
    expect_duty("clib SPIN_LEFT 35",    0, 35, 35,  0);

    printf("\n%s (%d failure%s)\n",
           (g_fail == 0) ? "ALL PASSED" : "FAILED",
           g_fail, (g_fail == 1) ? "" : "s");
    return (g_fail == 0) ? 0 : 1;
}
