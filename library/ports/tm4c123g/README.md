# TM4C123G LaunchPad 的 port 层实现说明

> 状态：**代码骨架 / 未接真机验证**。里面的 TivaWare API 名是准确的，但引脚号、时钟配置、极性需要根据
> `organized/materials/01_底板与引脚图/` 的原理图和实物调试后确定。
> 这里刻意用 `.md` 而不是 `.c`，避免被 `make test` 误编进 PC 端构建。
> 本文已按四路代码审查的结论修正（PWM 时钟分频、16 位捕获溢出、阻塞串口、GPIO 解锁措辞）。

## 1. 要做什么

整个库只有 `clib_port_t` 这一张函数指针表需要你实现，填完调用 `clib_init(&port)` 就完事。
库里的 `include/` 和 `src/` 一个字都不用改。

## 2. 代码骨架（拷到 CCS 工程里补全）

```c
/* clib_port_tm4c123g.c —— 骨架，需按实际引脚与时钟补全 */
#include <stdint.h>
#include <stdbool.h>
#include "inc/tm4c123gh6pm.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "driverlib/pin_map.h"
#include "driverlib/uart.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"
#include "clib_port.h"

/* ---- 逻辑引脚 → 物理管脚映射表（★ 唯一需要按原理图核对的地方）---- */
typedef struct { uint32_t port_base; uint8_t pin_mask; } pin_hw_t;
static const pin_hw_t s_pins[CLIB_PIN_COUNT] = {
    /* CLIB_PIN_IR_REMOTE  */ { GPIO_PORTA_BASE, GPIO_PIN_3 },
    /* CLIB_PIN_TRACK_L    */ { GPIO_PORTE_BASE, GPIO_PIN_2 },
    /* CLIB_PIN_TRACK_R    */ { GPIO_PORTE_BASE, GPIO_PIN_3 },
    /* CLIB_PIN_OBST_L     */ { GPIO_PORTE_BASE, GPIO_PIN_0 },
    /* CLIB_PIN_OBST_R     */ { GPIO_PORTE_BASE, GPIO_PIN_1 },
    /* CLIB_PIN_ULTRA_TRIG */ { GPIO_PORTA_BASE, GPIO_PIN_2 },
    /* CLIB_PIN_ULTRA_ECHO */ { GPIO_PORTB_BASE, GPIO_PIN_2 }  /* 建议接 CCP 捕获脚 */
};

static volatile uint32_t s_ms = 0;   /* SysTick 中断每 1ms 递增 */

static uint32_t port_millis(void) { return s_ms; }

/* micros() 必须直读硬件定时器，不要开 1us 中断去累加：80MHz 下中断开销不可承受。
   示意：把 TIMER5 配成 1MHz 自由运行计数器（32 位模式 + 预分频 80），
   这里直接读它的值。具体配置见下面初始化步骤 6。 */
static uint32_t port_micros(void) {
    return TimerValueGet(TIMER5_BASE, TIMER_A);
}

static void port_delay_us(uint32_t us) {
    /* 80MHz：SysCtlDelay 一次循环 3 个时钟 */
    ROM_SysCtlDelay(us * (SysCtlClockGet() / 3000000UL));
}

static int port_gpio_read(uint8_t logic_pin) {
    if (logic_pin >= CLIB_PIN_COUNT) return CLIB_ERR_PARAM;
    return (GPIOPinRead(s_pins[logic_pin].port_base,
                        s_pins[logic_pin].pin_mask) != 0) ? 1 : 0;
}
static void port_gpio_write(uint8_t logic_pin, int level) {
    if (logic_pin >= CLIB_PIN_COUNT) return;
    GPIOPinWrite(s_pins[logic_pin].port_base, s_pins[logic_pin].pin_mask,
                 level ? s_pins[logic_pin].pin_mask : 0);
}

/* 4 路电机 PWM：PWM0 Gen0/Gen1，20kHz。duty 是 0~100 百分比
   前提：初始化时已调用 SysCtlPWMClockSet(SYSCTL_PWMDIV_1)，
   否则复位默认分频不是 /1，算出来的频率会偏（电机变 10kHz、舵机变 25Hz）。 */
static void port_motor_set_duty(uint8_t ch, uint8_t duty) {
    static const uint32_t out[4] = { PWM_OUT_0, PWM_OUT_1, PWM_OUT_2, PWM_OUT_3 };
    uint32_t period = SysCtlClockGet() / 20000UL;   /* 20kHz */
    if (ch >= 4) return;
    PWMPulseWidthSet(PWM0_BASE, out[ch], (uint32_t)duty * period / 100UL);
}

/* 舵机单独占 Gen2（与电机频率不同，绝不能共用发生器） */
static void port_servo_set_pulse(uint16_t pulse_us) {
    uint32_t period = SysCtlClockGet() / 50UL;      /* 50Hz */
    uint32_t width  = (uint32_t)pulse_us * SysCtlClockGet() / 1000000UL;
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_2, period);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4, width);   /* PE4 = M0PWM4 */
}

static uint32_t uart_base(uint8_t id) {
    switch (id) {
    case CLIB_UART_DEBUG:  return UART0_BASE;
    case CLIB_UART_BT:     return UART1_BASE;
    case CLIB_UART_VISION: return UART3_BASE;
    default:               return UART0_BASE;
    }
}

/* ★ 生产代码请用「TX 环形缓冲 + UART 发送中断」做非阻塞。
   下面这版 UARTCharPut 是阻塞的，仅用于裸机调试起步；
   9600 波特率下发 20 字节 ≈ 21ms，会直接卡死 1kHz 导航控制环。 */
static int port_uart_write(uint8_t id, const uint8_t *buf, uint16_t len) {
    uint32_t base = uart_base(id); uint16_t i;
    for (i = 0; i < len; ++i) UARTCharPut(base, buf[i]);
    return len;
}
static int port_uart_read(uint8_t id, uint8_t *buf, uint16_t cap) {
    uint32_t base = uart_base(id); uint16_t n = 0;
    /* 非阻塞：从 RX 环形缓冲取；没有数据立刻返回 0 */
    while (n < cap && UARTCharsAvail(base)) {
        buf[n++] = (uint8_t)UARTCharGetNonBlocking(base);
    }
    return n;
}
static void port_log_putc(char c) { UARTCharPut(UART0_BASE, (uint8_t)c); }

const clib_port_t g_tm4c123g_port = {
    port_millis, port_micros, port_delay_us,
    port_gpio_read, port_gpio_write,
    port_motor_set_duty, port_servo_set_pulse,
    port_uart_write, port_uart_read,
    port_log_putc
};

/* 初始化顺序（在 main 里，clib_init 之前调用）
 * 1) SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
 *                   SYSCTL_XTAL_16MHZ);                     → 80MHz
 * 2) SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOx / PWM0 / UART1 / UART3 / TIMER5 ...)
 *    每个使能后等 while(!SysCtlPeripheralReady(SYSCTL_PERIPH_xxx)); 再访问寄存器
 * 3) ★ SysCtlPWMClockSet(SYSCTL_PWMDIV_1);   ← 必须显式设，复位默认不是 /1
 * 4) GPIOPinConfigure(GPIO_PB6_M0PWM0) 等 PinMux；GPIOPinTypePWM / GPIOPinTypeUART
 * 5) PWMGenConfigure(PWM0_BASE, PWM_GEN_0/1,
 *                    PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC)
 *    PWMGenPeriodSet(... SysCtlClockGet()/20000) → 电机 20kHz
 *    PWMGenPeriodSet(... PWM_GEN_2, SysCtlClockGet()/50) → 舵机 50Hz
 *    PWMOutputState + PWMGenEnable
 * 6) TIMER5 配成 1MHz 自由计数给 port_micros()：
 *    TimerConfigure(TIMER5_BASE, TIMER_CFG_PERIODIC)
 *    TimerPrescaleSet(TIMER5_BASE, TIMER_A, 79)  /* 80MHz/80 = 1MHz */
 *    TimerLoadSet(TIMER5_BASE, TIMER_A, 0xFFFFFFFFu); TimerEnable(...)
 * 7) UARTConfigSetExpClk(base, SysCtlClockGet(), 115200,
 *        UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE)
 *    UARTFIFOLevelSet + UARTFIFOEnable
 *    UARTIntEnable(base, UART_INT_RX | UART_INT_RT)
 *    ★ UART_INT_RT 是「FIFO 接收超时中断」，需先使能 FIFO；作用近似 STM32 的
 *      IDLE 中断，但两者不是同一个机制（IDLE 判总线空闲，RT 判 FIFO 里有数据且超时）
 * 8) 红外 / 超声波输入捕获：★ 不要用 16 位拆分对去测 HC-SR04 回波
 *    TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_CAP_TIME 在 80MHz 无预分频时约 819us 就溢出，
 *    而 Echo 高电平最长可达 ~30ms，直接读必得回绕错值。
 *    改法二选一：
 *      a) 用 32 位 TIMER_CFG_A_CAP_TIME_UP（或 TIMER_CFG_PERIODIC_UP）计边沿时间；
 *      b) 保留 16 位拆分对，但加 TimerPrescaleSet 并统计溢出次数做补偿。
 *    在 ISR 里算出脉宽微秒后，喂 clib_irdec_feed_pulse() 或 clib_echo_us_to_cm()。
 * 9) clib_init(&g_tm4c123g_port);
 */
```

## 3. 必踩的坑（按严重程度排）

| 坑 | 说明 |
|---|---|
| **PWM 时钟分频** | 复位后 PWM 时钟分频不是 /1。漏了 `SysCtlPWMClockSet(SYSCTL_PWMDIV_1)`，按 `SysCtlClockGet()` 算 period 会得到电机 10kHz、舵机 25Hz。 |
| **16 位捕获溢出** | 见骨架步骤 8。HC-SR04 回波最长约 30ms，16 位计数器在 80MHz 下 819us 就回绕。 |
| **5V 电平** | 按具体芯片、引脚和输入模式核对 5 V 容限。TM4C123GH6PM 的 PB2 用作数字输入时，HC-SR04 Echo 可按整车指南分压或直连。5 V 红外输出仍按分压或电平转换接入。GPIO 输出高电平为 3.3 V，驱动 L298N IN 要核对其 `V_IH`。[Echo 接线与供电顺序](../../../patrol/WIRING.md#hc-sr04-echo-接线与供电顺序) |
| **PWM 共时基** | 同一个 PWM 发生器的两路输出共用一个计数器，频率必须相同。4 路电机 20kHz 用 Gen0/Gen1，舵机 50Hz 单独占 Gen2。 |
| **GPIO 解锁是两类事** | PD7 / PF0 是 **NMI 锁定脚**，作 GPIO 须 `HWREG(port_base + GPIO_O_LOCK) = GPIO_LOCK_KEY;` 再 `HWREG(port_base + GPIO_O_CR) |= pin;`，用完建议重新上锁；PC0~3 是 **JTAG 复用脚**，不是 NMI 锁定，别混为一谈（改错会把调试口锁死）。PF0/PF4 又是板载按键、PF1~3 是 RGB LED，**尽量整组避开**。 |
| **UART0 被占用** | PA0/PA1 硬连板载 ICDI 虚拟串口，正好当调试口，别再分给蓝牙/K210。 |
| **无 IDLE 中断** | 原 `Uart3.c` 靠 `USART_IT_IDLE` 判帧；TM4C 用 `UART_INT_RT`（需开 FIFO）+ 环形缓冲，或直接用本库 `clib_frame` 的软件状态机（不依赖中断）。 |
| **串口发送阻塞** | `UARTCharPut` 在 9600 下 20 字节 ≈ 21ms。生产代码必须改中断 + 环形缓冲。 |
| **NVIC 优先级** | TM4C 有效优先级只有 3 bit = 8 级，`IntPrioritySet` 仅高 3 位有效，**没有抢占/子优先级分组**，原 STM32 的 `NVIC_PriorityGroupConfig` 逻辑要整体重排。 |
| **中断使能** | `IntMasterEnable()` 别忘了；`GPIOIntStatus(port, true)` 要配合 `GPIOIntClear`。 |
| **ROM_ 前缀** | `ROM_SysCtlDelay` 等需匹配 `TARGET_IS_TM4C123_RA1` / `PART_TM4C123GH6PM` 与 TivaWare 版本；建议统一用 `MAP_` 前缀自动回退 Flash 版。 |
| **供电** | 舵机、多路传感器别从 LaunchPad 板载 3.3V LDO 取大电流；K210 建议独立供电并**共地**。 |
| **时钟常数** | 原 `Delay.c` 的 `72 * xus` 是 72MHz，TM4C 是 80MHz，必须改。 |

## 4. 落地验收顺序

1. 先只实现 `millis/micros/delay_us/gpio_read/gpio_write/log_putc`，跑通 ICDI 虚拟串口打印。
2. 接 `motor_set_duty`，用万用表/示波器确认 4 路 20kHz PWM 与 L298N IN1~IN4 对应关系（注意是**互补 PWM** 不是方向脚+使能）。
3. 接 `servo_set_pulse_us`，用示波器量 500/1500/2500us 是否对得上 0/90/180°，极性反了在 port 层修，别改库。
4. 接 `uart_write/read`，用 `clib_cmd_send()` + 串口助手验证收发。
5. 最后接红外输入捕获 / 超声波捕获，把脉宽喂给 `clib_irdec_feed_pulse()` 和 `clib_echo_us_to_cm()`。
6. 每接一步，回到 PC 跑一次 `make test` 确认库逻辑没被改坏。
