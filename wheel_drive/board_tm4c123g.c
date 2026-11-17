/*
 * board_tm4c123g.c -- TM4C123G LaunchPad 的 clib_port_t
 *
 * 依赖：TI TivaWare for C Series driverlib
 * 电机 PWM：PWM0 Gen0/Gen1，20kHz，PB6/PB7/PB4/PB5 → L298N IN1/IN2/IN3/IN4
 * 时基：SysTick 1ms 中断；micros() 由 SysTick 计数值换算，不另开 1us 中断
 */
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "inc/tm4c123gh6pm.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/pwm.h"
#include "driverlib/systick.h"
#include "driverlib/interrupt.h"
#include "driverlib/uart.h"
#include "clib_common.h"
#include "clib_port.h"
#include "board_tm4c123g.h"

#define SYS_CLK_HZ        80000000UL
#define MOTOR_PWM_HZ      20000UL
#define MOTOR_PERIOD      (SYS_CLK_HZ / MOTOR_PWM_HZ)          /* 4000 计数 = 20kHz */
#define SYSTICK_PER_MS    (SYS_CLK_HZ / 1000UL)                 /* 80000 计数 = 1ms */
#define US_PER_TICK_DIV   (SYS_CLK_HZ / 1000000UL)              /* 80 计数 = 1us */

/* 逻辑引脚 → 物理管脚（与 ../library/ports/tm4c123g/README.md 一致）。
   本例程只用电机，传感器脚先配成上拉输入，以后接超声波/循迹不用改代码。 */
typedef struct { uint32_t base; uint8_t pin; } pin_hw_t;
static const pin_hw_t s_pins[CLIB_PIN_COUNT] = {
    /* IR_REMOTE  */ { GPIO_PORTA_BASE, GPIO_PIN_3 },
    /* TRACK_L    */ { GPIO_PORTE_BASE, GPIO_PIN_2 },
    /* TRACK_R    */ { GPIO_PORTE_BASE, GPIO_PIN_3 },
    /* OBST_L     */ { GPIO_PORTE_BASE, GPIO_PIN_0 },
    /* OBST_R     */ { GPIO_PORTE_BASE, GPIO_PIN_1 },
    /* ULTRA_TRIG */ { GPIO_PORTA_BASE, GPIO_PIN_2 },
    /* ULTRA_ECHO */ { GPIO_PORTB_BASE, GPIO_PIN_2 }
};

/* ch 0..3 对应 L298N IN1~IN4 */
static const uint32_t s_pwm_out[4] = {
    PWM_OUT_0, PWM_OUT_1, PWM_OUT_2, PWM_OUT_3
};

static volatile uint32_t s_ms = 0u;

static void systick_isr(void) { s_ms++; }

/* ---------------- 时基 ---------------- */
static uint32_t board_millis(void) { return s_ms; }

/* 重读 s_ms 直到两次一致，避免「刚跨过 1ms 边界」时算错微秒 */
static uint32_t board_micros(void)
{
    uint32_t ms, frac;
    do {
        ms   = s_ms;
        frac = (SYSTICK_PER_MS - SysTickValueGet()) / US_PER_TICK_DIV;
    } while (ms != s_ms);
    return ms * 1000u + frac;
}

static void board_delay_us(uint32_t us)
{
    /* SysCtlDelay 每循环 3 个时钟；80MHz 而非原 STM32 的 72MHz */
    SysCtlDelay(us * (SysCtlClockGet() / 3000000UL));
}

/* ---------------- GPIO ---------------- */
static int board_gpio_read(uint8_t pin)
{
    if (pin >= (uint8_t)CLIB_PIN_COUNT) { return CLIB_ERR_PARAM; }
    return (GPIOPinRead(s_pins[pin].base, s_pins[pin].pin) != 0u) ? 1 : 0;
}

static void board_gpio_write(uint8_t pin, int level)
{
    if (pin >= (uint8_t)CLIB_PIN_COUNT) { return; }
    GPIOPinWrite(s_pins[pin].base, s_pins[pin].pin,
                 level ? s_pins[pin].pin : 0u);
}

/* ---------------- 电机 4 路 PWM ---------------- */
static void board_motor_set_duty(uint8_t ch, uint8_t duty)
{
    if (ch >= 4u) { return; }
    PWMPulseWidthSet(PWM0_BASE, s_pwm_out[ch],
                     (uint32_t)duty * MOTOR_PERIOD / 100u);
}

/* 本例程不接舵机，留空实现满足 clib_init() 的非空回调要求。
 * ★ 50Hz 舵机不能和 20kHz 电机共用 PWMDIV_1：PWM 周期寄存器只有 16 位，
 *   80MHz 下最长周期 819us（≈1.2kHz），做不出 20ms 周期。见 README。 */
static void board_servo_set_pulse(uint16_t pulse_us) { (void)pulse_us; }

/* ---------------- UART0（板载 ICDI 虚拟串口，调试用） ---------------- */
static int board_uart_write(uint8_t id, const uint8_t *buf, uint16_t len)
{
    uint16_t i;
    (void)id;                       /* 本例程把所有串口都指向 UART0 */
    for (i = 0u; i < len; ++i) { UARTCharPut(UART0_BASE, buf[i]); }
    return (int)len;
}

static int board_uart_read(uint8_t id, uint8_t *buf, uint16_t cap)
{
    uint16_t n = 0u;
    (void)id;
    while ((n < cap) && UARTCharsAvail(UART0_BASE)) {
        buf[n++] = (uint8_t)UARTCharGetNonBlocking(UART0_BASE);
    }
    return (int)n;                  /* 非阻塞：没数据立刻返回 0 */
}

static void board_log_putc(char c) { UARTCharPut(UART0_BASE, (uint8_t)c); }

const clib_port_t g_board_port = {
    board_millis, board_micros, board_delay_us,
    board_gpio_read, board_gpio_write,
    board_motor_set_duty, board_servo_set_pulse,
    board_uart_write, board_uart_read,
    board_log_putc
};

/* ---------------- 板级初始化 ---------------- */
static void enable_periph(uint32_t periph)
{
    SysCtlPeripheralEnable(periph);
    while (!SysCtlPeripheralReady(periph)) { }   /* 使能后需等 3 个时钟 */
}

void Board_Init(void)
{
    uint32_t i;

    /* 1) 16MHz 晶振 + PLL → 80MHz */
    SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
                   SYSCTL_XTAL_16MHZ);

    /* 2) GPIO / PWM / UART 时钟 */
    enable_periph(SYSCTL_PERIPH_GPIOA);
    enable_periph(SYSCTL_PERIPH_GPIOB);
    enable_periph(SYSCTL_PERIPH_GPIOE);
    enable_periph(SYSCTL_PERIPH_PWM0);
    enable_periph(SYSCTL_PERIPH_UART0);

    /* 3) ★ 必须显式设为 /1，复位默认分频不是 /1，漏了频率会错一倍 */
    SysCtlPWMClockSet(SYSCTL_PWMDIV_1);

    /* 4) 传感器脚：Trig 输出，其余带上拉输入 */
    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_2);
    GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_3);
    GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, GPIO_PIN_2);
    GPIOPinTypeGPIOInput(GPIO_PORTE_BASE,
                         GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
    GPIOPadConfigSet(GPIO_PORTA_BASE, GPIO_PIN_3,
                     GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
    GPIOPadConfigSet(GPIO_PORTB_BASE, GPIO_PIN_2,
                     GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
    GPIOPadConfigSet(GPIO_PORTE_BASE,
                     GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3,
                     GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    /* 5) UART0：PA0/PA1 已硬连板载 ICDI 虚拟串口，115200 8N1 */
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200,
                        UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                        UART_CONFIG_PAR_NONE);
    UARTFIFOEnable(UART0_BASE);

    /* 6) 四路电机 PWM：Gen0 = IN1/IN2，Gen1 = IN3/IN4，同为 20kHz */
    GPIOPinConfigure(GPIO_PB6_M0PWM0);
    GPIOPinConfigure(GPIO_PB7_M0PWM1);
    GPIOPinConfigure(GPIO_PB4_M0PWM2);
    GPIOPinConfigure(GPIO_PB5_M0PWM3);
    GPIOPinTypePWM(GPIO_PORTB_BASE,
                   GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);
    PWMGenConfigure(PWM0_BASE, PWM_GEN_0,
                    PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenConfigure(PWM0_BASE, PWM_GEN_1,
                    PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, MOTOR_PERIOD);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, MOTOR_PERIOD);
    for (i = 0u; i < 4u; ++i) {
        PWMPulseWidthSet(PWM0_BASE, s_pwm_out[i], 0u);   /* 上电全 0%，轮子不动 */
    }
    PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT | PWM_OUT_1_BIT |
                              PWM_OUT_2_BIT | PWM_OUT_3_BIT, true);
    PWMGenEnable(PWM0_BASE, PWM_GEN_0);
    PWMGenEnable(PWM0_BASE, PWM_GEN_1);

    /* 7) SysTick 1ms；IntRegister 会动态注册向量，不用改 startup_ccs.c */
    SysTickPeriodSet(SysCtlClockGet() / 1000UL);   /* 80000 计数 = 1ms */
    IntRegister(FAULT_SYSTICK, systick_isr);
    SysTickIntEnable();
    SysTickEnable();
    IntMasterEnable();
}
