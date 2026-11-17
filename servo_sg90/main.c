/* EK-TM4C123GXL + SG90 position servo. Signal: PC7 (J4.7 / WT1CCP1).
 * Reset clock: nominal 16 MHz PIOSC. No driverlib or motor PWM dependency.
 * Register reference: TI SPMS376E, chapters 3, 5, 10 and 11.
 */
#include <stdbool.h>
#include <stdint.h>

#define BIT(n)          (UINT32_C(1) << (n))
#define REG(base, off)  (*(volatile uint32_t *)((base) + (off)))
#define SYSCTL          UINT32_C(0x400FE000)
#define GPIO_C          UINT32_C(0x40006000)
#define GPIO_F          UINT32_C(0x40025000)
#define WT1             UINT32_C(0x40037000)
#define SYSTICK         UINT32_C(0xE000E010)

#define GPIO_DATA(p,m)  REG((p), ((m) << 2))
#define GPIO_DIR(p)     REG((p), 0x400u)
#define GPIO_AFSEL(p)   REG((p), 0x420u)
#define GPIO_PUR(p)     REG((p), 0x510u)
#define GPIO_DEN(p)     REG((p), 0x51Cu)
#define GPIO_AMSEL(p)   REG((p), 0x528u)
#define GPIO_PCTL(p)    REG((p), 0x52Cu)

#define GPIO_CLOCKS     (BIT(2) | BIT(5)) /* C, F */
#define WT1_CLOCK       BIT(1)
#define SIGNAL          BIT(7)
#define SW1             BIT(4)          /* PF4, active low */
#define LED_GREEN       BIT(3)          /* PF3 */
#define TIMER_B_ENABLE  BIT(8)
#define CENTER_US       1500u
#define TICKS_PER_US    16u
#define FRAME_LOAD      (20000u * TICKS_PER_US - 1u)

/* Near-center test; exact angles and left/right depend on the servo and horn. */
static const uint16_t sweep_us[] = {1300u, CENTER_US, 1700u, CENTER_US};
#define SWEEP_COUNT (sizeof(sweep_us) / sizeof(sweep_us[0]))

static void pulse_set_us(uint32_t pulse_us)
{
    /* WT1B counts down: high at reload, low at match.
       TBMRSU latches the new match at the next frame boundary. */
    REG(WT1, 0x034u) = FRAME_LOAD - pulse_us * TICKS_PER_US;
}

static void servo_enable(bool enable)
{
    if (enable) {
        pulse_set_us(CENTER_US);
        GPIO_AFSEL(GPIO_C) |= SIGNAL;
        REG(WT1, 0x00Cu) |= TIMER_B_ENABLE;
    } else {
        GPIO_AFSEL(GPIO_C) &= ~SIGNAL;   /* PC7 returns to GPIO, data latch low */
        REG(WT1, 0x00Cu) &= ~TIMER_B_ENABLE;
    }
    GPIO_DATA(GPIO_F, LED_GREEN) = enable ? LED_GREEN : 0u;
}

static void board_init(void)
{
    REG(SYSCTL, 0x608u) |= GPIO_CLOCKS; /* RCGCGPIO */
    REG(SYSCTL, 0x65Cu) |= WT1_CLOCK;   /* RCGCWTIMER */
    while ((REG(SYSCTL, 0xA08u) & GPIO_CLOCKS) != GPIO_CLOCKS) {}
    while ((REG(SYSCTL, 0xA5Cu) & WT1_CLOCK) == 0u) {}

    GPIO_AMSEL(GPIO_C) &= ~SIGNAL;
    GPIO_AFSEL(GPIO_C) &= ~SIGNAL;
    GPIO_PCTL(GPIO_C) = (GPIO_PCTL(GPIO_C) & ~(UINT32_C(0xF) << 28)) |
                        (UINT32_C(7) << 28); /* PC7 mux 7 = WT1CCP1 */
    GPIO_DATA(GPIO_C, SIGNAL) = 0u;
    GPIO_DIR(GPIO_C) |= SIGNAL;
    GPIO_DEN(GPIO_C) |= SIGNAL;

    GPIO_AMSEL(GPIO_F) &= ~(SW1 | LED_GREEN);
    GPIO_AFSEL(GPIO_F) &= ~(SW1 | LED_GREEN);
    GPIO_DIR(GPIO_F) = (GPIO_DIR(GPIO_F) & ~SW1) | LED_GREEN;
    GPIO_PUR(GPIO_F) |= SW1;
    GPIO_DATA(GPIO_F, LED_GREEN) = 0u;
    GPIO_DEN(GPIO_F) |= SW1 | LED_GREEN;

    REG(WT1, 0x00Cu) = 0u;              /* Disable; non-inverted PWM */
    REG(WT1, 0x000u) = 4u;              /* Split: 32-bit timers A and B */
    REG(WT1, 0x008u) = BIT(10) | BIT(3) | 2u; /* TBMRSU, TBAMS, periodic down */
    REG(WT1, 0x02Cu) = FRAME_LOAD;      /* TBILR */
    REG(WT1, 0x03Cu) = 0u;              /* TBPR */
    REG(WT1, 0x044u) = 0u;              /* TBPMR */
    pulse_set_us(CENTER_US);

    REG(SYSTICK, 0x04u) = 1000u * TICKS_PER_US - 1u;
    REG(SYSTICK, 0x08u) = 0u;
    REG(SYSTICK, 0x00u) = BIT(2) | BIT(0); /* Core clock, poll only */
}

int main(void)
{
    bool running = false;
    bool pressed = true;    /* Require a release first if SW1 is held at reset. */
    uint32_t debounce_ms = 0u;
    uint32_t hold_ms = 0u;
    uint32_t step = 0u;

    board_init();
    for (;;) {
        while ((REG(SYSTICK, 0x00u) & BIT(16)) == 0u) {}
        const bool sample = (GPIO_DATA(GPIO_F, SW1) == 0u);

        if (sample == pressed) {
            debounce_ms = 0u;
        } else if (++debounce_ms == 25u) {
            debounce_ms = 0u;
            pressed = sample;
            if (pressed) {
                running = !running;
                servo_enable(running);
                step = 0u;
                hold_ms = 1000u; /* Center first; then repeat the sweep. */
                continue;
            }
        }

        if (running && --hold_ms == 0u) {
            pulse_set_us(sweep_us[step]);
            step = (step + 1u) % SWEEP_COUNT;
            hold_ms = 800u;
        }
    }
}
