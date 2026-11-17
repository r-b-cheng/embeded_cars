/* PC7 position holding; no autonomous sweep table or sweep countdown.
 * Commands are volatile memory values consumed by the main loop.
 * Reset defaults to output disabled; SW1 toggles hold/stop.
 */
#define main repository_manual_main
#include "main.c"
#undef main

volatile uint32_t control_target_us = 1500u;
volatile uint32_t control_enable;
volatile uint32_t control_active;
volatile uint32_t control_applied_us = 1500u;
volatile uint32_t control_millis;

static uint32_t limit_pulse(uint32_t pulse)
{
    if (pulse < 1000u) return 1000u;
    if (pulse > 2200u) return 2200u;
    return pulse;
}

static void output_set(bool active)
{
    if (active) {
        pulse_set_us(control_applied_us);
        GPIO_AFSEL(GPIO_C) |= SIGNAL;
        REG(WT1, 0x00Cu) |= TIMER_B_ENABLE;
    } else {
        GPIO_AFSEL(GPIO_C) &= ~SIGNAL;
        GPIO_DATA(GPIO_C, SIGNAL) = 0u;
        REG(WT1, 0x00Cu) &= ~TIMER_B_ENABLE;
    }
    control_active = active ? 1u : 0u;
    GPIO_DATA(GPIO_F, LED_GREEN) = active ? LED_GREEN : 0u;
}

int main(void)
{
    bool pressed = true;
    uint32_t debounce_ms = 0u;
    uint32_t ramp_ms = 0u;
    board_init();
    output_set(false);

    for (;;) {
        while ((REG(SYSTICK, 0x00u) & BIT(16)) == 0u) {}
        ++control_millis;
        const bool sample = GPIO_DATA(GPIO_F, SW1) == 0u;
        if (sample == pressed) {
            debounce_ms = 0u;
        } else if (++debounce_ms == 25u) {
            debounce_ms = 0u;
            pressed = sample;
            if (pressed) control_enable = control_enable ? 0u : 1u;
        }

        const bool active = control_enable == 1u;
        if (active != (control_active != 0u)) output_set(active);
        if (++ramp_ms == 5u) {
            ramp_ms = 0u;
            if (active) {
                const uint32_t target = limit_pulse(control_target_us);
                uint32_t applied = control_applied_us;
                if (applied < target) applied += target - applied > 2u ? 2u : 1u;
                if (applied > target) applied -= applied - target > 2u ? 2u : 1u;
                if (applied != control_applied_us) {
                    control_applied_us = applied;
                    pulse_set_us(applied);
                }
            }
        }
    }
}
