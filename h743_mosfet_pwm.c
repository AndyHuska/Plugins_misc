/*

  h743_mosfet_pwm.c - STM32H743 specific multi-PWM plugin for MOSFET outputs

  Part of grblHAL

*/

#include "driver.h"

#if H743_MOSFET_PWM_ENABLE == 1 && defined(BOARD_WAVESHARE_OPENH743I)

#include <stdio.h>
#include <string.h>

#include "grbl/ioports.h"
#include "grbl/protocol.h"
#include "grbl/gcode.h"

#define MOSPWM_CHANNELS 8
#define MOSPWM_MCODE_SET_DUTY 1010
#define MOSPWM_MCODE_SET_FREQ 1011
#define MOSPWM_MCODE_REPORT   1012
#define MOSPWM_DEFAULT_FREQ_HZ 1000.0f

typedef struct {
    GPIO_TypeDef *gpio_port;
    uint8_t gpio_pin;
} mosfet_gpio_t;

typedef struct {
    bool active;
    uint8_t ioport;
    xbar_t xbar;
    float freq_hz;
    float duty;
    float period_us;
    float on_time_us;
} mosfet_pwm_t;

static mosfet_pwm_t mosfet[MOSPWM_CHANNELS] = {0};
static uint8_t n_mosfet = 0;

static user_mcode_ptrs_t user_mcode;
static on_report_options_ptr on_report_options;

static bool ensure_channel (uint8_t ch);
static void init_channel (uint8_t ch);

static const mosfet_gpio_t mosfet_gpio[MOSPWM_CHANNELS] = {
    { PWMOUTPUT0_PORT, PWMOUTPUT0_PIN },
    { PWMOUTPUT1_PORT, PWMOUTPUT1_PIN },
    { PWMOUTPUT2_PORT, PWMOUTPUT2_PIN },
    { PWMOUTPUT3_PORT, PWMOUTPUT3_PIN },
    { PWMOUTPUT4_PORT, PWMOUTPUT4_PIN },
    { PWMOUTPUT5_PORT, PWMOUTPUT5_PIN },
    { PWMOUTPUT6_PORT, PWMOUTPUT6_PIN },
    { PWMOUTPUT7_PORT, PWMOUTPUT7_PIN }
};

static int8_t find_output_port (GPIO_TypeDef *port, uint8_t pin)
{
    int8_t found = -1;
    uint8_t max = ioports_available(Port_Analog, Port_Output);

    for(uint8_t idx = 0; idx < max; idx++) {
        xbar_t *xbar = ioport_get_info(Port_Analog, Port_Output, idx);
        if(xbar && xbar->port == (void *)port && xbar->pin == pin) {
            found = (int8_t)idx;
            break;
        }
    }

    return found;
}

static bool mosfet_pwm_configure (uint8_t ch, float freq_hz)
{
    if(ch >= MOSPWM_CHANNELS || !mosfet[ch].active)
        return false;

    pwm_config_t cfg = {
        .freq_hz = freq_hz,
        .min = 0.0f,
        .max = 100.0f,
        .off_value = 0.0f,
        .min_value = 0.0f,
        .max_value = 100.0f,
        .invert = Off,
        .servo_mode = Off
    };

    if(!mosfet[ch].xbar.config || !mosfet[ch].xbar.config(&mosfet[ch].xbar, &cfg, false))
        return false;

    mosfet[ch].freq_hz = freq_hz;
    return true;
}

static bool mosfet_pwm_set_duty (uint8_t ch, float duty)
{
    if(ch >= MOSPWM_CHANNELS || !mosfet[ch].active)
        return false;

    if(duty < 0.0f)
        duty = 0.0f;
    if(duty > 100.0f)
        duty = 100.0f;

    if(!ioport_analog_out(mosfet[ch].ioport, duty))
        return false;
    mosfet[ch].duty = duty;

    return true;
}

static bool mosfet_pwm_set (uint8_t ch, float freq_hz, float duty, float period_us, float on_time_us)
{
    if(!mosfet_pwm_configure(ch, freq_hz) || !mosfet_pwm_set_duty(ch, duty))
        return false;

    mosfet[ch].period_us = period_us;
    mosfet[ch].on_time_us = on_time_us;

    return true;
}

static void mosfet_pwm_report (int8_t ch)
{
    char msg[112];

    if(ch >= 0 && ch < MOSPWM_CHANNELS) {
        if(mosfet[ch].active) {
            snprintf(msg, sizeof(msg), "[MOSPWM%u:F=%.1fHz,D=%.2f%%,Q=%.3fus,R=%.3fus]" ASCII_EOL,
                     (uint8_t)ch, mosfet[ch].freq_hz, mosfet[ch].duty, mosfet[ch].period_us, mosfet[ch].on_time_us);
            hal.stream.write(msg);
        } else {
            snprintf(msg, sizeof(msg), "[MOSPWM%u:inactive]" ASCII_EOL, (uint8_t)ch);
            hal.stream.write(msg);
        }
        return;
    }

    for(uint8_t i = 0; i < MOSPWM_CHANNELS; i++) {
        if(mosfet[i].active) {
            snprintf(msg, sizeof(msg), "[MOSPWM%u:F=%.1fHz,D=%.2f%%,Q=%.3fus,R=%.3fus]" ASCII_EOL,
                     i, mosfet[i].freq_hz, mosfet[i].duty, mosfet[i].period_us, mosfet[i].on_time_us);
            hal.stream.write(msg);
        } else {
            snprintf(msg, sizeof(msg), "[MOSPWM%u:inactive]" ASCII_EOL, i);
            hal.stream.write(msg);
        }
    }
}

uint16_t h743_mosfet_pwm_get_relay_state (void)
{
    uint16_t state = 0;

    // Encode each relay as 00 off, 01 constant, or 10 pulsing.
    for(uint8_t ch = 0; ch < MOSPWM_CHANNELS; ch++) {
        uint16_t relay_state = 0;

        if(mosfet[ch].active) {
            if(mosfet[ch].duty >= 100.0f)
                relay_state = 1;
            else if(mosfet[ch].duty > 0.0f)
                relay_state = 2;
        }

        state |= relay_state << (ch * 2);
    }

    return state;
}

static user_mcode_type_t mcode_check (user_mcode_t mcode)
{
    return ((uint32_t)mcode == MOSPWM_MCODE_SET_DUTY || (uint32_t)mcode == MOSPWM_MCODE_SET_FREQ || (uint32_t)mcode == MOSPWM_MCODE_REPORT)
             ? UserMCode_Normal
             : (user_mcode.check ? user_mcode.check(mcode) : UserMCode_Unsupported);
}

static status_code_t mcode_validate (parser_block_t *gc_block)
{
    if((uint32_t)gc_block->user_mcode == MOSPWM_MCODE_SET_DUTY) {

        if(gc_block->words.p) {
            if(!isintf(gc_block->values.p))
                return Status_BadNumberFormat;
            if(gc_block->values.p < 0.0f || gc_block->values.p >= MOSPWM_CHANNELS)
                return Status_GcodeValueOutOfRange;
        }

        if(gc_block->words.f != gc_block->words.d)
            return Status_GcodeValueWordMissing;

        if(gc_block->words.f && (gc_block->values.f < 1.0f || gc_block->values.f > 100000.0f))
            return Status_GcodeValueOutOfRange;

        if(gc_block->words.d && (gc_block->values.d < 0.0f || gc_block->values.d > 100.0f))
            return Status_GcodeValueOutOfRange;

        gc_block->words.p = gc_block->words.f = gc_block->words.d = Off;

        return Status_OK;
    }

    if((uint32_t)gc_block->user_mcode == MOSPWM_MCODE_SET_FREQ) {

        if(!gc_block->words.p)
            return Status_GcodeValueWordMissing;

        if(!isintf(gc_block->values.p))
            return Status_BadNumberFormat;

        if(gc_block->values.p < 0.0f || gc_block->values.p >= MOSPWM_CHANNELS)
            return Status_GcodeValueOutOfRange;

        if(gc_block->words.q != gc_block->words.r)
            return Status_GcodeValueWordMissing;

        if(gc_block->words.q && (gc_block->values.q < 10.0f || gc_block->values.q > 1000000.0f))
            return Status_GcodeValueOutOfRange;

        if(gc_block->words.r && (gc_block->values.r < 0.0f || gc_block->values.r > gc_block->values.q))
            return Status_GcodeValueOutOfRange;

        gc_block->words.p = gc_block->words.q = gc_block->words.r = Off;

        return Status_OK;
    }

    if((uint32_t)gc_block->user_mcode == MOSPWM_MCODE_REPORT) {
        if(gc_block->words.p && !isintf(gc_block->values.p))
            return Status_BadNumberFormat;
        if(gc_block->words.p && (gc_block->values.p < 0.0f || gc_block->values.p >= MOSPWM_CHANNELS))
            return Status_GcodeValueOutOfRange;
        gc_block->words.p = Off;
        return Status_OK;
    }

    return user_mcode.validate ? user_mcode.validate(gc_block) : Status_Unhandled;
}

static void mcode_execute (uint_fast16_t state, parser_block_t *gc_block)
{
    uint32_t mcode = (uint32_t)gc_block->user_mcode;

    if(mcode == MOSPWM_MCODE_SET_DUTY) {

        uint8_t ch = gc_block->words.p ? (uint8_t)gc_block->values.p : 0;

        if(gc_block->words.f && gc_block->words.d) {
            float period_us = 1000000.0f / gc_block->values.f;
            float on_time_us = period_us * gc_block->values.d / 100.0f;
            if(ensure_channel(ch))
                mosfet_pwm_set(ch, gc_block->values.f, gc_block->values.d, period_us, on_time_us);
        } else
            mosfet_pwm_report((int8_t)ch);

    } else if(mcode == MOSPWM_MCODE_SET_FREQ) {

        uint8_t ch = (uint8_t)gc_block->values.p;

        if(gc_block->words.q && gc_block->words.r) {
            float period_us = gc_block->values.q;
            float on_time_us = gc_block->values.r;
            float freq_hz = 1000000.0f / period_us;
            float duty = on_time_us * 100.0f / period_us;
            if(ensure_channel(ch))
                mosfet_pwm_set(ch, freq_hz, duty, period_us, on_time_us);
        } else
            mosfet_pwm_report((int8_t)ch);

    } else if(mcode == MOSPWM_MCODE_REPORT) {

        if(gc_block->words.p && isintf(gc_block->values.p))
            mosfet_pwm_report((int8_t)gc_block->values.p);
        else
            mosfet_pwm_report(-1);

    } else if(user_mcode.execute)
        user_mcode.execute(state, gc_block);
}

static void onReportOptions (bool newopt)
{
    if(on_report_options)
        on_report_options(newopt);

    if(!newopt)
        report_plugin("H743 8ch MOSFET PWM", "0.04");
}

static bool ensure_channel (uint8_t ch)
{
    if(ch >= MOSPWM_CHANNELS)
        return false;

    if(!mosfet[ch].active)
        init_channel(ch);

    return mosfet[ch].active;
}

static void init_channel (uint8_t ch)
{
    int8_t port = find_output_port(mosfet_gpio[ch].gpio_port, mosfet_gpio[ch].gpio_pin);

    if(port < 0)
        return;

    uint8_t p = (uint8_t)port;
    if(!ioport_claim(Port_Analog, Port_Output, &p, "MOSFET PWM"))
        return;

    xbar_t *xbar = ioport_get_info(Port_Analog, Port_Output, p);
    if(!xbar || !xbar->config)
        return;

    mosfet[ch].xbar = *xbar;
    mosfet[ch].ioport = p;
    mosfet[ch].active = true;
    mosfet[ch].freq_hz = MOSPWM_DEFAULT_FREQ_HZ;
    mosfet[ch].duty = 0.0f;
    mosfet[ch].period_us = 1000000.0f / MOSPWM_DEFAULT_FREQ_HZ;
    mosfet[ch].on_time_us = 0.0f;

    if(!mosfet_pwm_set(ch, MOSPWM_DEFAULT_FREQ_HZ, 0.0f, mosfet[ch].period_us, 0.0f)) {
        mosfet[ch].active = false;
        return;
    }

    ioport_set_description(Port_Analog, Port_Output, p, "MOSFET PWM");
    n_mosfet++;
}

void h743_mosfet_pwm_init (void)
{
    memset(mosfet, 0, sizeof(mosfet));


    memcpy(&user_mcode, &grbl.user_mcode, sizeof(user_mcode_ptrs_t));
    grbl.user_mcode.check = mcode_check;
    grbl.user_mcode.validate = mcode_validate;
    grbl.user_mcode.execute = mcode_execute;

    on_report_options = grbl.on_report_options;
    grbl.on_report_options = onReportOptions;
}

#endif // H743_MOSFET_PWM_ENABLE
