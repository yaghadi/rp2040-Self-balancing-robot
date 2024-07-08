#include "pico_servo.h"
#include "pico/time.h"
#include "hardware/irq.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "hardware/structs/clocks.h"
#include "hardware/gpio.h"

#include <string.h>

#define KILO 1000
#define PWM_FREQ 50 // PWM frequency in hertz

static float clkdiv;
static uint min;
static uint max;

static int slice_map[30];
static uint slice_active[8];
static uint servo_pos[32];
static uint servo_pos_buf[16];
static pwm_config slice_cfg[8];

static uint min_us = 500;
static uint max_us = 2500;
static float us_per_unit = 0.0f;

static void wrap_cb(void)
{
    uint offset;

    for (int i = 0; i < 8; ++i)
    {
        if (slice_active[i] == 0)
            continue;

        pwm_clear_irq(i);
        offset = 16 * ((servo_pos_buf[i + 0] + 1) % 2);
        pwm_set_chan_level(i, 0, servo_pos[offset + i + 0]);

        offset = 16 * ((servo_pos_buf[i + 1] + 1) % 2);
        pwm_set_chan_level(i, 1, servo_pos[offset + i + 1]);
    }
}

void servo_set_bounds(uint a, uint b)
{
    min_us = a;
    max_us = b;
    if (us_per_unit > 0.0f)
    {
        min = min_us / us_per_unit;
        max = max_us / us_per_unit;
    }
}

int servo_init(void)
{
    for (int i = 0; i < 30; ++i)
    {
        slice_map[i] = -1;
    }
    memset(slice_active, 0, 8 * sizeof(uint));
    memset(servo_pos, 0, 32 * sizeof(uint));
    memset(servo_pos_buf, 0, 16 * sizeof(uint));

    irq_add_shared_handler(PWM_IRQ_WRAP, wrap_cb, PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);

    return 0;
}

int servo_clock_auto(void)
{
    return servo_clock_source(CLOCKS_FC0_SRC_VALUE_PLL_SYS_CLKSRC_PRIMARY);
}

int servo_clock_source(uint src)
{
    clkdiv = (float)frequency_count_khz(src) * (float)KILO / (PWM_FREQ * 10000);
    if (clkdiv == 0)
    {
        return 1;
    }
    us_per_unit = 1.0f / (PWM_FREQ * 10000) / 1e-6;

    min = min_us / us_per_unit;
    max = max_us / us_per_unit;

    return 0;
}

int servo_attach(uint pin)
{
    uint slice = pwm_gpio_to_slice_num(pin);
    if (slice_active[slice] >= 2)
    {
        return 1;
    }

    gpio_set_function(pin, GPIO_FUNC_PWM);
    slice_map[pin] = slice;

    if (slice_active[slice] == 0)
    {
        pwm_clear_irq(slice);
        pwm_set_irq_enabled(slice, true);

        slice_cfg[slice] = pwm_get_default_config();
        pwm_config_set_wrap(&slice_cfg[slice], 10000);
        pwm_config_set_clkdiv(&slice_cfg[slice], clkdiv);
        pwm_init(slice, &slice_cfg[slice], true);
        pwm_set_chan_level(slice, pin % 2, 450);
    }

    ++slice_active[slice];

    irq_set_enabled(PWM_IRQ_WRAP, true);

    return 0;
}

int servo_move_to(uint pin, uint angle)
{
    if (slice_map[pin] < 0)
    {
        return 1;
    }

    uint val = (uint)((float)angle / 180.0f * (max - min)) + min;

    uint pos = slice_map[pin] + (pin % 2);
    servo_pos[16 * servo_pos_buf[pos] + pos] = val;
    servo_pos_buf[pos] = (servo_pos_buf[pos] + 1) % 2;
    return 0;
}

int servo_microseconds(uint pin, uint us)
{
    if (slice_map[pin] < 0)
    {
        return 1;
    }
    
    uint pos = slice_map[pin] + (pin % 2);
    servo_pos[16 * servo_pos_buf[pos] + pos] = us;
    servo_pos_buf[pos] = (servo_pos_buf[pos] + 1) % 2;
    return 0;
}