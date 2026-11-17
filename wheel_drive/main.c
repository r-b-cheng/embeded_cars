/*
 * main.c -- L298N 双轮驱动演示（TM4C123G LaunchPad）
 *
 * 上电后按表依次做动作，循环播放；串口 115200 会打印当前动作。
 * ★ 调试时务必把小车架空，别让它带着线在地上跑。
 * 全程非阻塞：没有 Delay_ms，靠 clib_millis() 计时，控制环以后可以直接接管 Wheel_Set()。
 */
#include <stdint.h>
#include "clib_port.h"
#include "board_tm4c123g.h"
#include "wheel.h"

typedef struct {
    int16_t     left;
    int16_t     right;
    uint32_t    ms;
    const char *name;
} step_t;

static const step_t s_steps[] = {
    {  60,  60, 2000u, "FORWARD  60" },
    {   0,   0,  500u, "STOP"        },
    { -60, -60, 2000u, "BACKWARD 60" },
    {   0,   0,  500u, "STOP"        },
    {   0,  60, 1000u, "TURN LEFT"   },
    {  60,   0, 1000u, "TURN RIGHT"  },
    { -60,  60, 1000u, "SPIN LEFT"   },
    {  60, -60, 1000u, "SPIN RIGHT"  },
    {   0,   0, 2000u, "STOP"        }
};

#define STEP_COUNT (sizeof(s_steps) / sizeof(s_steps[0]))

static void log_str(const char *s)
{
    while (*s != '\0') { clib_log_char(*s++); }
}

int main(void)
{
    uint32_t idx = 0u;
    uint32_t t0;

    Board_Init();
    if (Wheel_Init(&g_board_port) != CLIB_OK) {
        return 1;                       /* port 注册失败，灯也不亮 */
    }

    log_str("wheel demo ready\r\n");
    Wheel_Set(s_steps[0].left, s_steps[0].right);
    log_str(s_steps[0].name);
    log_str("\r\n");
    t0 = clib_millis();

    for (;;) {
        if (clib_millis() - t0 >= s_steps[idx].ms) {
            t0 += s_steps[idx].ms;
            idx = (idx + 1u) % (uint32_t)STEP_COUNT;
            Wheel_Set(s_steps[idx].left, s_steps[idx].right);
            log_str(s_steps[idx].name);
            log_str("\r\n");
        }
        /* 以后把这里换成：读编码器 → Odom_Update() → 路径跟踪 → Wheel_Set() */
    }
}
