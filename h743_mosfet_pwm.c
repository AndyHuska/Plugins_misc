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
} mosfet_pwm_t;

static mosfet_pwm_t mosfet[MOSPWM_CHANNELS] = {0};
static uint8_t n_mosfet = 0;

static user_mcode_ptrs_t user_mcode;
static on_report_options_ptr on_report_options;

static bool ensure_channel (uint8_t ch);
static void init_channel (uint8_t ch);

static const mosfet_gpio_t mosfet_gpio[MOSPWM_CHANNELS] = {
    { AUXOUTPUT0_PORT, AUXOUTPUT0_PIN },
    { AUXOUTPUT1_PORT, AUXOUTPUT1_PIN },
    { AUXOUTPUT2_PORT, AUXOUTPUT2_PIN },
    { AUXOUTPUT3_PORT, AUXOUTPUT3_PIN },
    { AUXOUTPUT4_PORT, AUXOUTPUT4_PIN },
    { AUXOUTPUT5_PORT, AUXOUTPUT5_PIN },
    { AUXOUTPUT6_PORT, AUXOUTPUT6_PIN },
    { AUXOUTPUT7_PORT, AUXOUTPUT7_PIN }
};

static int8_t find_output_port (GPIO_TypeDef *port, uint8_t pin)
{
    int8_t found = -1;
    uint8_t max = ioports_available(Port_Digital, Port_Output);

    for(uint8_t idx = 0; idx < max; idx++) {
        xbar_t *xbar = ioport_get_info(Port_Digital, Port_Output, idx);
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
    if(ch >= MOSPWM_CHANNELS || !mosfet[ch].active || !mosfet[ch].xbar.set_value)
        return false;

    if(duty < 0.0f)
        duty = 0.0f;
    if(duty > 100.0f)
        duty = 100.0f;

    mosfet[ch].xbar.set_value(&mosfet[ch].xbar, duty);
    mosfet[ch].duty = duty;

    return true;
}

static void mosfet_pwm_report (int8_t ch)
{
    char msg[80];

    if(ch >= 0 && ch < MOSPWM_CHANNELS) {
        if(mosfet[ch].active) {
            snprintf(msg, sizeof(msg), "[MOSPWM%u:F=%.1fHz,D=%.1f%%]" ASCII_EOL, (uint8_t)ch, mosfet[ch].freq_hz, mosfet[ch].duty);
            hal.stream.write(msg);
        } else {
            snprintf(msg, sizeof(msg), "[MOSPWM%u:inactive]" ASCII_EOL, (uint8_t)ch);
            hal.stream.write(msg);
        }
        return;
    }

    for(uint8_t i = 0; i < MOSPWM_CHANNELS; i++) {
        if(mosfet[i].active) {
            snprintf(msg, sizeof(msg), "[MOSPWM%u:F=%.1fHz,D=%.1f%%]" ASCII_EOL, i, mosfet[i].freq_hz, mosfet[i].duty);
            hal.stream.write(msg);
        } else {
            snprintf(msg, sizeof(msg), "[MOSPWM%u:inactive]" ASCII_EOL, i);
            hal.stream.write(msg);
        }
    }
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

        uint8_t ch = gc_block->words.p ? (uint8_t)gc_block->values.p : 0;

        if(gc_block->words.p && !isintf(gc_block->values.p))
            return Status_BadNumberFormat;

        if(ch >= MOSPWM_CHANNELS)
            return Status_GcodeValueOutOfRange;

        if(gc_block->words.q && (gc_block->values.q < 0.0f || gc_block->values.q > 100.0f))
            return Status_GcodeValueOutOfRange;

        gc_block->words.p = gc_block->words.q = Off;

        return Status_OK;
    }

    if((uint32_t)gc_block->user_mcode == MOSPWM_MCODE_SET_FREQ) {

        if(!gc_block->words.p || !gc_block->words.f)
            return Status_GcodeValueWordMissing;

        if(!isintf(gc_block->values.p))
            return Status_BadNumberFormat;

        uint8_t ch = (uint8_t)gc_block->values.p;

        if(ch >= MOSPWM_CHANNELS)
            return Status_GcodeValueOutOfRange;

        if(gc_block->values.f < 1.0f || gc_block->values.f > 100000.0f)
            return Status_GcodeValueOutOfRange;

        gc_block->words.p = gc_block->words.f = Off;

        return Status_OK;
    }

    if((uint32_t)gc_block->user_mcode == MOSPWM_MCODE_REPORT) {
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

        if(gc_block->words.q) {
            if(ensure_channel(ch))
                mosfet_pwm_set_duty(ch, gc_block->values.q);
        }
        else
            mosfet_pwm_report((int8_t)ch);

    } else if(mcode == MOSPWM_MCODE_SET_FREQ) {

        uint8_t ch = (uint8_t)gc_block->values.p;

        if(ensure_channel(ch) && mosfet_pwm_configure(ch, gc_block->values.f))
            mosfet_pwm_set_duty(ch, mosfet[ch].duty);

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
        report_plugin("H743 8ch MOSFET PWM", "0.01");
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
    if(!ioport_claim(Port_Digital, Port_Output, &p, "MOSFET PWM"))
        return;

    xbar_t *xbar = ioport_get_info(Port_Digital, Port_Output, p);
    if(!xbar || !xbar->config)
        return;

    gpio_out_config_t gpio_cfg = {
        .inverted = Off,
        .open_drain = Off,
        .pwm = On
    };

    if(!xbar->config(xbar, &gpio_cfg, false))
        return;

    xbar = ioport_get_info(Port_Digital, Port_Output, p);
    if(!xbar || !xbar->config || !xbar->set_value)
        return;

    mosfet[ch].xbar = *xbar;
    mosfet[ch].ioport = p;
    mosfet[ch].active = true;
    mosfet[ch].freq_hz = MOSPWM_DEFAULT_FREQ_HZ;
    mosfet[ch].duty = 0.0f;

    if(!mosfet_pwm_configure(ch, MOSPWM_DEFAULT_FREQ_HZ)) {
        mosfet[ch].active = false;
        return;
    }

    ioport_set_description(Port_Digital, Port_Output, p, "MOSFET PWM");
    mosfet_pwm_set_duty(ch, 0.0f);
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
