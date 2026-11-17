/*

  velocity_axes.c - independent velocity controlled (reserved) axes

  Part of grblHAL

  Axes A and above can be reserved at runtime ($790) and are then driven at a commanded
  velocity, independent of G-code and Direct Motion, e.g. solder pumps and conveyors.
  See VELOCITY_AXES_PLAN.md for the design.

*/

#include "driver.h"

#if VELOCITY_AXES_ENABLE

#if N_AXIS < 4
#error "Velocity axes require N_AXIS > 3"
#endif

#if !STEP_INJECT_ENABLE
#error "Velocity axes require STEP_INJECT_ENABLE"
#endif

#include <math.h>
#include <string.h>

#include "grbl/hal.h"
#include "grbl/system.h"
#include "grbl/errors.h"
#include "grbl/protocol.h"
#include "grbl/state_machine.h"
#include "grbl/nvs_buffer.h"
#include "grbl/planner.h"
#include "grbl/stepper.h"
#include "grbl/motion_control.h"
#include "grbl/gcode.h"
#include "grbl/report.h"
#include "grbl/task.h"

#include "velocity_axes.h"

#if VELOCITY_JOG_ENABLE
#include "direct_motion.h"
#endif

#ifndef VELOCITY_AXES_TICK_HZ
#define VELOCITY_AXES_TICK_HZ 40000
#endif

#define VA_CTRL_HZ          1000
#define VA_FIRST_AXIS       3
#define VA_XYZ_MASK         0x07
#define VA_MCODE_SET        1020
#define VA_MCODE_REPORT     1021
#define VA_SETTING_RESERVED ((setting_id_t)790)
#define VA_SETTING_OPTIONS  ((setting_id_t)791)
#define VA_SETTING_ALARMS   ((setting_id_t)796)
#define VA_QUEUE_SIZE       32
#define VA_MAX_BACKENDS     2

#define VA_OPT_KEEP_ON_RESET bit(0)
#define VA_OPT_STOP_ON_HOLD  bit(1)

#define VA_ALARM_BIT(a) (1UL << ((a) - 1))
#define VA_ALARM_LABELS 22
#define VA_ALARM_DEFAULT (VA_ALARM_BIT(10)|VA_ALARM_BIT(14)|VA_ALARM_BIT(16)|VA_ALARM_BIT(17)|VA_ALARM_BIT(19)|VA_ALARM_BIT(20)|VA_ALARM_BIT(21)|VA_ALARM_BIT(22))

typedef struct {
    uint8_t reserved;
    uint8_t options[N_AXIS];
    uint32_t alarm_stop;
} va_settings_t;

typedef struct {
    const velocity_axes_backend_t *backend;
    float target;   // units/min, signed
    float accel;    // units/s^2, 0 = axis setting
    bool held;      // stopped by feed hold, target is kept for resume
} va_axis_t;

typedef struct {
    volatile bool pending;
    uint8_t axis;
    bool has_q, has_r;
    float q, r;
    int32_t seq;
    line_number_t id;
} va_cmd_t;

static va_settings_t cfg;
static nvs_address_t nvs_address;
static va_axis_t va_axis[N_AXIS];
static volatile uint8_t reserved = 0;
static uint8_t capable_last = 0;
static volatile uint_fast16_t last_alarm = 0;

static const velocity_axes_backend_t *backends[VA_MAX_BACKENDS];
static uint_fast8_t n_backends = 0;

static va_cmd_t queue[VA_QUEUE_SIZE];
static uint_fast8_t q_head = 0;
static volatile uint_fast8_t q_pending = 0;
static int32_t q_seq = 0;

static char reserved_detail[N_AXIS][40];

static user_mcode_ptrs_t user_mcode;
static on_report_options_ptr on_report_options;
static on_execute_realtime_ptr on_execute_realtime, on_execute_delay;
static on_state_change_ptr on_state_change;
static on_reset_ptr on_reset, on_soft_reset;
static on_motion_dispatch_ptr on_motion_dispatch;
static home_machine_ptr home_machine;

static inline uint32_t va_irq_save (void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static inline void va_irq_restore (uint32_t primask)
{
    __set_PRIMASK(primask);
}

/***********************
 *  Local step backend *
 ***********************/

typedef struct {
    volatile float target;  // steps/s, signed
    volatile float accel;   // steps/s^2
    volatile float decel;   // steps/s^2
    float v;                // steps/s, signed, ISR owned
    uint32_t phase;
    uint32_t inc;
} lm_axis_t;

static lm_axis_t lm[N_AXIS];
static hal_timer_t lm_timer = NULL;
static volatile uint8_t lm_active = 0, lm_kill = 0;
static volatile bool lm_running = false;
static uint_fast16_t lm_ctrl_cnt = 0;
static const float lm_inc_scale = 4294967296.0f / (float)VELOCITY_AXES_TICK_HZ;
static stepper_enable_ptr lm_enable_org = NULL;
static axes_signals_t lm_enable_req = {0};
static bool lm_enable_req_valid = false;
static uint8_t lm_enable_applied = 0;

static void va_alarm_poll (void);

static void lm_zero (lm_axis_t *m)
{
    m->target = 0.0f;
    m->v = 0.0f;
    m->inc = m->phase = 0;
}

static void lm_ramp (void)
{
    static const float dt = 1.0f / (float)VA_CTRL_HZ;
    uint_fast8_t idx;
    uint8_t active = lm_active;

    for(idx = VA_FIRST_AXIS; idx < N_AXIS; idx++) {

        if(!(active & bit(idx)))
            continue;

        lm_axis_t *m = &lm[idx];
        float t = m->target, v = m->v;
        bool slowing = (v > 0.0f && t < v) || (v < 0.0f && t > v);
        float dv = (slowing ? m->decel : m->accel) * dt;

        // Clamp at zero when braking so a reversal accelerates at the accel rate.
        if(v < t) {
            if((v += dv) > t)
                v = t;
            if(slowing && v > 0.0f)
                v = 0.0f;
        } else if(v > t) {
            if((v -= dv) < t)
                v = t;
            if(slowing && v < 0.0f)
                v = 0.0f;
        }

        m->v = v;
        v = fabsf(v);
        m->inc = v >= (float)VELOCITY_AXES_TICK_HZ ? UINT32_MAX : (uint32_t)(v * lm_inc_scale);

        if(m->v == 0.0f && t == 0.0f) {
            lm_active &= ~bit(idx);
            m->inc = m->phase = 0;
        }
    }
}

ISR_CODE static void ISR_FUNC(lm_isr)(void *context)
{
    uint_fast8_t idx;
    uint8_t step = 0, dir = 0, kill;

    (void)context;

    va_alarm_poll();

    if((kill = lm_kill)) {
        lm_kill = 0;
        for(idx = VA_FIRST_AXIS; idx < N_AXIS; idx++) {
            if(kill & bit(idx))
                lm_zero(&lm[idx]);
        }
        lm_active &= ~kill;
    }

    if(++lm_ctrl_cnt >= VELOCITY_AXES_TICK_HZ / VA_CTRL_HZ) {
        lm_ctrl_cnt = 0;
        lm_ramp();
    }

    uint8_t active = lm_active;

    for(idx = VA_FIRST_AXIS; idx < N_AXIS; idx++) {
        if(active & bit(idx)) {
            lm_axis_t *m = &lm[idx];
            uint32_t prev = m->phase;
            if((m->phase += m->inc) < prev) {
                step |= bit(idx);
                if(m->v < 0.0f)
                    dir |= bit(idx);
            }
        }
    }

    if(step)
        hal.stepper.output_step((axes_signals_t){ .bits = step }, (axes_signals_t){ .bits = dir });

    if(!lm_active) {
        lm_running = false;
        hal.timer.stop(lm_timer);
    }
}

static axes_signals_t lm_axes (void)
{
    axes_signals_t axes = { .mask = AXES_BITMASK & ~VA_XYZ_MASK };

    if(lm_timer == NULL || hal.stepper.claim_motor == NULL || hal.stepper.output_step == NULL)
        axes.mask = 0;
#if VELOCITY_JOG_ENABLE
    axes.mask &= ~direct_motion_remote_axes();
#endif

    return axes;
}

static void lm_claim (uint8_t axis, bool claim)
{
    if(!claim) {
        uint32_t irq = va_irq_save();
        lm_zero(&lm[axis]);
        lm_active &= ~bit(axis);
        lm_kill &= ~bit(axis);
        va_irq_restore(irq);
    }

    hal.stepper.claim_motor(axis, claim);
}

static void lm_set_target (uint8_t axis, float units_per_min, float accel)
{
    float spm = settings.axis[axis].steps_per_mm;

    if(spm <= 0.0f)
        return;

    // A decel setting equal to the axis acceleration means "not set": follow the effective accel.
    float decel = settings_get_deceleration(axis);
    decel = decel == settings.axis[axis].acceleration && accel > 0.0f ? accel : decel / 3600.0f;

    if(accel <= 0.0f)
        accel = settings.axis[axis].acceleration / 3600.0f;

    uint32_t irq = va_irq_save();

    lm_kill &= ~bit(axis);
    lm[axis].accel = accel * spm;
    lm[axis].decel = decel * spm;
    lm[axis].target = units_per_min * spm / 60.0f;

    if(lm[axis].target != 0.0f || lm[axis].v != 0.0f) {
        lm_active |= bit(axis);
        if(!lm_running) {
            lm_running = true;
            lm_ctrl_cnt = VELOCITY_AXES_TICK_HZ / VA_CTRL_HZ; // ramp on first tick
            hal.timer.start(lm_timer, 1000000UL / VELOCITY_AXES_TICK_HZ);
        }
    }

    va_irq_restore(irq);
}

static void lm_stop (axes_signals_t axes, bool immediate)
{
    uint_fast8_t idx;
    uint32_t irq = va_irq_save();

    for(idx = VA_FIRST_AXIS; idx < N_AXIS; idx++) {
        if(axes.mask & bit(idx)) {
            if(immediate || !lm_running) {
                lm_zero(&lm[idx]);
                lm_active &= ~bit(idx);
            } else
                lm[idx].target = 0.0f;
        }
    }

    va_irq_restore(irq);
}

static float lm_get_rate (uint8_t axis)
{
    float spm = settings.axis[axis].steps_per_mm;

    return spm > 0.0f ? lm[axis].v * 60.0f / spm : 0.0f;
}

// Keep running reserved motors energized when the core disables idle steppers.
static void lm_enable (axes_signals_t enable, bool hold)
{
    lm_enable_req = enable;
    lm_enable_req_valid = true;
    enable.mask |= lm_active;

    lm_enable_org(enable, hold);
}

static void lm_poll (void)
{
    uint8_t active = lm_active;

    if(active != lm_enable_applied) {
        // Before the core has requested an enable state assume all non-reserved motors are on.
        uint8_t base = lm_enable_req_valid ? lm_enable_req.mask : (AXES_BITMASK & ~velocity_axes_reserved().mask);
        lm_enable_applied = active;
        lm_enable_org((axes_signals_t){ .mask = base | active }, false);
    }
}

static const velocity_axes_backend_t local_backend = {
    .axes = lm_axes,
    .claim = lm_claim,
    .set_target = lm_set_target,
    .stop = lm_stop,
    .get_rate = lm_get_rate,
    .poll = lm_poll
};

static bool lm_init (void)
{
    if(hal.stepper.output_step == NULL || hal.stepper.claim_motor == NULL || hal.timer.claim == NULL)
        return false;

    if((lm_timer = hal.timer.claim((timer_cap_t){0}, 1000)) == NULL)
        return false;

    timer_cfg_t timer_cfg = {
        .single_shot = false,
        .timeout_callback = lm_isr
    };

    hal.timer.configure(lm_timer, &timer_cfg);

    lm_enable_org = hal.stepper.enable;
    hal.stepper.enable = lm_enable;

    return true;
}

/****************
 *  Front end   *
 ****************/

static axes_signals_t va_capable (void)
{
    axes_signals_t axes = {0};
    uint_fast8_t idx;

    for(idx = 0; idx < n_backends; idx++)
        axes.mask |= backends[idx]->axes().mask;

    axes.mask &= AXES_BITMASK & ~VA_XYZ_MASK;

    return axes;
}

static const velocity_axes_backend_t *va_backend_for (uint8_t axis)
{
    uint_fast8_t idx;

    for(idx = 0; idx < n_backends; idx++) {
        if(backends[idx]->axes().mask & bit(axis))
            return backends[idx];
    }

    return NULL;
}

static inline bool va_is_reserved (uint8_t axis)
{
    return axis < N_AXIS && (reserved & bit(axis));
}

static inline float va_effective (va_axis_t *a)
{
    return a->held ? 0.0f : a->target;
}

// May be called from interrupt context.
static void va_apply (uint8_t axis, bool has_q, float q, bool has_r, float r)
{
    va_axis_t *a = &va_axis[axis];

    if(!va_is_reserved(axis) || a->backend == NULL)
        return;

    if(has_r)
        a->accel = r;

    if(has_q) {
        float max = settings.axis[axis].max_rate;
        a->target = q > max ? max : (q < -max ? -max : q);
        a->held = false;
    }

    a->backend->set_target(axis, va_effective(a), a->accel);
}

// May be called from interrupt context.
static void va_stop (uint8_t axes, bool immediate)
{
    uint_fast8_t idx;

    for(idx = VA_FIRST_AXIS; idx < N_AXIS; idx++) {
        if((axes & reserved & bit(idx)) && va_axis[idx].backend) {
            va_axis[idx].target = 0.0f;
            va_axis[idx].held = false;
            va_axis[idx].backend->stop((axes_signals_t){ .mask = bit(idx) }, immediate);
        }
    }
}

static bool va_alarm_stops (uint_fast16_t code)
{
    return code == Alarm_EStop || code > VA_ALARM_LABELS || (cfg.alarm_stop & VA_ALARM_BIT(code));
}

// Called from the step ISR and the foreground; acts on new alarm codes only.
static void va_alarm_poll (void)
{
    uint_fast16_t alarm = sys.rt_exec_alarm;

    if(alarm != last_alarm) {
        last_alarm = alarm;
        if(alarm && va_alarm_stops(alarm))
            va_stop(reserved, true);
    }
}

static void va_queue_cancel_all (void)
{
    uint_fast8_t idx;
    uint32_t irq = va_irq_save();

    for(idx = 0; idx < VA_QUEUE_SIZE; idx++)
        queue[idx].pending = false;
    q_pending = 0;

    va_irq_restore(irq);
}

static bool va_cmd_exec (output_command_t *cmd, bool cancel)
{
    bool apply = false;
    va_cmd_t c;

    if(cmd->port >= VA_QUEUE_SIZE)
        return false;

    uint32_t irq = va_irq_save();

    if(queue[cmd->port].pending && queue[cmd->port].seq == cmd->value) {
        c = queue[cmd->port];
        queue[cmd->port].pending = false;
        q_pending--;
        apply = !cancel;
    }

    va_irq_restore(irq);

    if(apply)
        va_apply(c.axis, c.has_q, c.q, c.has_r, c.r);

    return apply;
}

static bool va_motion_idle (void)
{
    if(plan_get_current_block() || st_is_stepping())
        return false;

#if ENABLE_PATH_BLENDING
    uint32_t age;
    if(mc_path_blend_pending_age(&age))
        return false;
#endif

    return true;
}

// Apply queued commands that have no following motion block once all preceding motion is done.
static void va_queue_flush_idle (void)
{
    while(q_pending && va_motion_idle()) {

        int_fast8_t found = -1;
        uint_fast8_t idx;
        va_cmd_t c;

        uint32_t irq = va_irq_save();

        for(idx = 0; idx < VA_QUEUE_SIZE; idx++) {
            if(queue[idx].pending && (found < 0 || queue[idx].seq - queue[found].seq < 0))
                found = idx;
        }

        if(found >= 0) {
            c = queue[found];
            queue[found].pending = false;
            q_pending--;
        }

        va_irq_restore(irq);

        if(found < 0)
            break;

        va_apply(c.axis, c.has_q, c.q, c.has_r, c.r);
        if(c.id)
            st_execution_complete(c.id);
    }
}

static bool va_queue_has_room (void)
{
    return !queue[q_head].pending;
}

static void va_queue_command (uint8_t axis, bool has_q, float q, bool has_r, float r, line_number_t id)
{
    if(!q_pending && va_motion_idle()) {
        va_apply(axis, has_q, q, has_r, r);
        if(id) {
            st_execution_set_active(id);
            st_execution_effect_queued(id);
            st_execution_complete(id);
        }
        return;
    }

    uint_fast8_t slot = q_head;
    va_cmd_t *c = &queue[slot];

    *c = (va_cmd_t){ .axis = axis, .has_q = has_q, .q = q, .has_r = has_r, .r = r, .seq = ++q_seq, .id = id };

    output_command_t cmd = {
        .port = slot,
        .value = c->seq,
        .execution_id = id,
        .exec = va_cmd_exec
    };

    uint32_t irq = va_irq_save();
    c->pending = true;
    q_pending++;
    q_head = (q_head + 1) % VA_QUEUE_SIZE;
    va_irq_restore(irq);

    if(!gc_queue_output_command(&cmd)) {
        // Out of memory: apply now rather than lose the command.
        irq = va_irq_save();
        if(c->pending) {
            c->pending = false;
            q_pending--;
        }
        va_irq_restore(irq);
        va_apply(axis, has_q, q, has_r, r);
    }
}

static void va_update_homing_mask (void)
{
    sys.homing.mask &= ~reserved;
}

static void va_apply_reservation (void)
{
    uint_fast8_t idx;
    uint8_t want = cfg.reserved & va_capable().mask;

    capable_last = va_capable().mask;

    for(idx = VA_FIRST_AXIS; idx < N_AXIS; idx++) {

        va_axis_t *a = &va_axis[idx];

        if((reserved & bit(idx)) && !(want & bit(idx))) {
            va_stop(bit(idx), true);
            reserved &= ~bit(idx);
            if(a->backend)
                a->backend->claim(idx, false);
            a->backend = NULL;
            a->accel = 0.0f;
        } else if(!(reserved & bit(idx)) && (want & bit(idx))) {
            if((a->backend = va_backend_for(idx))) {
                a->target = a->accel = 0.0f;
                a->held = false;
                a->backend->claim(idx, true);
                reserved |= bit(idx);
            }
        }
    }

    va_update_homing_mask();
}

static void va_report_axis (uint8_t axis)
{
    va_axis_t *a = &va_axis[axis];

    hal.stream.write("[VEL:");
    hal.stream.write(uitoa(axis));
    hal.stream.write(",Q=");
    hal.stream.write(ftoa(a->target, 1));
    hal.stream.write(",V=");
    hal.stream.write(ftoa(a->backend ? a->backend->get_rate(axis) : 0.0f, 1));
    hal.stream.write(",R=");
    hal.stream.write(ftoa(a->accel > 0.0f ? a->accel : settings.axis[axis].acceleration / 3600.0f, 1));
    hal.stream.write(",F=");
    hal.stream.write(uitoa(cfg.options[axis]));
    hal.stream.write(a->held ? ",H]" ASCII_EOL : "]" ASCII_EOL);
}

static void va_report (int_fast8_t axis)
{
    uint_fast8_t idx;

    if(axis >= 0)
        va_report_axis((uint8_t)axis);
    else for(idx = VA_FIRST_AXIS; idx < N_AXIS; idx++) {
        if(reserved & bit(idx))
            va_report_axis(idx);
    }
}

static void va_poll (void)
{
    uint_fast8_t idx;

    va_alarm_poll();

    if(va_capable().mask != capable_last)
        va_apply_reservation();

    va_queue_flush_idle();

    for(idx = 0; idx < n_backends; idx++) {
        if(backends[idx]->poll)
            backends[idx]->poll();
    }
}

/*****************
 *  Public API   *
 *****************/

bool velocity_axes_register_backend (const velocity_axes_backend_t *backend)
{
    if(backend == NULL || n_backends >= VA_MAX_BACKENDS || !backend->axes || !backend->claim ||
        !backend->set_target || !backend->stop || !backend->get_rate)
        return false;

    backends[n_backends++] = backend;

    return true;
}

axes_signals_t velocity_axes_reserved (void)
{
    return (axes_signals_t){ .mask = reserved };
}

void velocity_axes_get_rates (float *rates)
{
    uint_fast8_t idx;

    for(idx = VA_FIRST_AXIS; idx < N_AXIS; idx++) {
        if((reserved & bit(idx)) && va_axis[idx].backend)
            rates[idx] = va_axis[idx].backend->get_rate(idx);
    }
}

/*****************
 *  Core hooks   *
 *****************/

static void onExecuteRealtime (sys_state_t state)
{
    va_poll();

    on_execute_realtime(state);
}

static void onExecuteDelay (sys_state_t state)
{
    va_poll();

    on_execute_delay(state);
}

static void onStateChange (sys_state_t state)
{
    uint_fast8_t idx;

    if(state & STATE_ESTOP)
        va_stop(reserved, true);
    else if((state & STATE_ALARM) && va_alarm_stops(sys.alarm))
        va_stop(reserved, true);

    for(idx = VA_FIRST_AXIS; idx < N_AXIS; idx++) {

        va_axis_t *a = &va_axis[idx];

        if(!(reserved & bit(idx)) || a->backend == NULL)
            continue;

        if(state & (STATE_HOLD|STATE_SAFETY_DOOR)) {
            if(!a->held && a->target != 0.0f && (cfg.options[idx] & VA_OPT_STOP_ON_HOLD)) {
                a->held = true;
                a->backend->set_target(idx, 0.0f, a->accel);
            }
        } else if(a->held) {
            a->held = false;
            if(state == STATE_CYCLE)
                a->backend->set_target(idx, a->target, a->accel);
            else
                a->target = 0.0f;
        }
    }

    if(on_state_change)
        on_state_change(state);
}

// Ctrl-X: called before mc_reset(), from interrupt context.
static void onSoftReset (void)
{
    uint_fast8_t idx;
    uint8_t stop = 0;

    for(idx = VA_FIRST_AXIS; idx < N_AXIS; idx++) {
        if(!(cfg.options[idx] & VA_OPT_KEEP_ON_RESET))
            stop |= bit(idx);
    }

    va_stop(stop, true);

    if(on_soft_reset)
        on_soft_reset();
}

static void onReset (void)
{
    uint_fast8_t idx;

    va_queue_cancel_all();

    for(idx = VA_FIRST_AXIS; idx < N_AXIS; idx++) {
        if(va_axis[idx].held) {
            va_axis[idx].held = false;
            va_axis[idx].target = 0.0f;
        }
    }

    if(on_reset)
        on_reset();
}

static uint_fast8_t va_first_axis (uint8_t mask)
{
    uint_fast8_t idx = 0;

    while(idx < N_AXIS && !(mask & bit(idx)))
        idx++;

    return idx;
}

static status_code_t onMotionDispatch (const motion_dispatch_request_t *request, plan_line_data_t *data, bool *handled)
{
    uint8_t hit = request->axes.mask & reserved;

    if(hit && (request->is_motion || request->jog || request->raw)) {
        *handled = false;
        gc_set_motion_dispatch_detail(reserved_detail[va_first_axis(hit)]);
        return VELOCITY_AXES_STATUS_RESERVED;
    }

    return on_motion_dispatch ? on_motion_dispatch(request, data, handled) : Status_OK;
}

static bool onHomeMachine (axes_signals_t cycle, axes_signals_t auto_square)
{
    va_update_homing_mask();

    cycle.mask &= ~reserved;
    auto_square.mask &= ~reserved;

    return cycle.mask == 0 || home_machine(cycle, auto_square);
}

/*************
 *  M-codes  *
 *************/

static user_mcode_type_t mcode_check (user_mcode_t mcode)
{
    return (uint32_t)mcode == VA_MCODE_SET || (uint32_t)mcode == VA_MCODE_REPORT
            ? UserMCode_Normal
            : (user_mcode.check ? user_mcode.check(mcode) : UserMCode_Unsupported);
}

static status_code_t va_validate_axis (float p)
{
    if(!isintf(p))
        return Status_BadNumberFormat;

    return p >= 0.0f && va_is_reserved((uint8_t)p) ? Status_OK : VELOCITY_AXES_STATUS_RESERVED;
}

static status_code_t mcode_validate (parser_block_t *gc_block)
{
    status_code_t status;

    switch((uint32_t)gc_block->user_mcode) {

        case VA_MCODE_SET:
            if(!gc_block->words.p || !(gc_block->words.q || gc_block->words.r))
                return Status_GcodeValueWordMissing;
            if((status = va_validate_axis(gc_block->values.p)) != Status_OK)
                return status;
            if(gc_block->words.r && !(gc_block->values.r >= 0.0f))
                return Status_GcodeValueOutOfRange;
            if(gc_block->words.q && isnan(gc_block->values.q))
                return Status_GcodeValueOutOfRange;
            if(!va_queue_has_room())
                return Status_Overflow;
            gc_block->words.p = gc_block->words.q = gc_block->words.r = Off;
            return Status_OK;

        case VA_MCODE_REPORT:
            if(gc_block->words.p && (status = va_validate_axis(gc_block->values.p)) != Status_OK)
                return status;
            gc_block->words.p = Off;
            return Status_OK;

        default:
            break;
    }

    return user_mcode.validate ? user_mcode.validate(gc_block) : Status_Unhandled;
}

static void mcode_execute (sys_state_t state, parser_block_t *gc_block)
{
    switch((uint32_t)gc_block->user_mcode) {

        case VA_MCODE_SET:
            va_queue_command((uint8_t)gc_block->values.p, gc_block->words.q, gc_block->values.q,
                              gc_block->words.r, gc_block->values.r, gc_block->output_command.execution_id);
            break;

        case VA_MCODE_REPORT:
            va_report(gc_block->words.p ? (int_fast8_t)gc_block->values.p : -1);
            break;

        default:
            if(user_mcode.execute)
                user_mcode.execute(state, gc_block);
            break;
    }
}

/**********************
 *  System commands   *
 **********************/

// $VEL=P<axis> [Q<speed>] [R<accel>] - immediate, any state.
static status_code_t cmd_vel (sys_state_t state, char *args)
{
    bool has_p = false, has_q = false, has_r = false;
    float p = 0.0f, q = 0.0f, r = 0.0f, value;
    uint_fast8_t pos = 0;
    status_code_t status;

    if(args == NULL) {
        va_report(-1);
        return Status_OK;
    }

    while(args[pos]) {

        char letter = CAPS(args[pos]);

        if(letter == ' ' || letter == ',') {
            pos++;
            continue;
        }

        pos++;
        if(!read_float(args, &pos, &value))
            return Status_BadNumberFormat;

        switch(letter) {
            case 'P': p = value; has_p = true; break;
            case 'Q': q = value; has_q = true; break;
            case 'R': r = value; has_r = true; break;
            default: return Status_InvalidStatement;
        }
    }

    if(!has_p)
        return Status_GcodeValueWordMissing;

    if((status = va_validate_axis(p)) != Status_OK)
        return status;

    if(!has_q && !has_r) {
        va_report((int_fast8_t)p);
        return Status_OK;
    }

    if(has_r && !(r >= 0.0f))
        return Status_GcodeValueOutOfRange;

    if(has_q && q != 0.0f && ((state & STATE_ESTOP) || hal.control.get_state().e_stop))
        return Status_EStop;

    va_apply((uint8_t)p, has_q, q, has_r, r);
    va_poll();

    return Status_OK;
}

static status_code_t home_check (uint8_t axis)
{
    return va_is_reserved(axis) ? VELOCITY_AXES_STATUS_RESERVED : Status_Unhandled;
}

static status_code_t home_3 (sys_state_t state, char *args) { return home_check(3); }
#if N_AXIS > 4
static status_code_t home_4 (sys_state_t state, char *args) { return home_check(4); }
#endif
#if N_AXIS > 5
static status_code_t home_5 (sys_state_t state, char *args) { return home_check(5); }
#endif
#if N_AXIS > 6
static status_code_t home_6 (sys_state_t state, char *args) { return home_check(6); }
#endif
#if N_AXIS > 7
static status_code_t home_7 (sys_state_t state, char *args) { return home_check(7); }
#endif

static char home_cmd_names[N_AXIS - VA_FIRST_AXIS][4];

static sys_command_t va_commands[] = {
    { "VEL", cmd_vel, { .allow_blocking = On }, { .str = "report or set velocity axis speed: $VEL=P<axis> Q<units/min> R<units/s^2>" } },
    { home_cmd_names[0], home_3, { .noargs = On } },
#if N_AXIS > 4
    { home_cmd_names[1], home_4, { .noargs = On } },
#endif
#if N_AXIS > 5
    { home_cmd_names[2], home_5, { .noargs = On } },
#endif
#if N_AXIS > 6
    { home_cmd_names[3], home_6, { .noargs = On } },
#endif
#if N_AXIS > 7
    { home_cmd_names[4], home_7, { .noargs = On } },
#endif
};

static sys_commands_t va_command_list = {
    .n_commands = sizeof(va_commands) / sizeof(sys_command_t),
    .commands = va_commands
};

/***************
 *  Settings   *
 ***************/

static status_code_t set_reserved (setting_id_t id, uint_fast16_t value)
{
    if((value & VA_XYZ_MASK) || (value & ~va_capable().mask))
        return Status_SettingValueOutOfRange;

    cfg.reserved = (uint8_t)value;
    va_apply_reservation();

    return Status_OK;
}

static uint32_t get_reserved (setting_id_t id)
{
    return cfg.reserved;
}

static status_code_t set_alarms (setting_id_t id, uint_fast16_t value)
{
    cfg.alarm_stop = (uint32_t)value | VA_ALARM_BIT(Alarm_EStop);

    return Status_OK;
}

static uint32_t get_alarms (setting_id_t id)
{
    return cfg.alarm_stop;
}

#define VA_OPTIONS_FORMAT "Keep running on soft reset,Stop on feed hold/door"

static const setting_detail_t va_settings[] = {
    { VA_SETTING_RESERVED, Group_Stepper, "Velocity axes", NULL, Format_AxisMask, NULL, NULL, NULL, Setting_NonCoreFn, set_reserved, get_reserved, NULL },
    { VA_SETTING_OPTIONS, Group_Stepper, "Velocity axis 3 options", NULL, Format_Bitfield, VA_OPTIONS_FORMAT, NULL, NULL, Setting_NonCore, &cfg.options[3], NULL, NULL },
#if N_AXIS > 4
    { VA_SETTING_OPTIONS + 1, Group_Stepper, "Velocity axis 4 options", NULL, Format_Bitfield, VA_OPTIONS_FORMAT, NULL, NULL, Setting_NonCore, &cfg.options[4], NULL, NULL },
#endif
#if N_AXIS > 5
    { VA_SETTING_OPTIONS + 2, Group_Stepper, "Velocity axis 5 options", NULL, Format_Bitfield, VA_OPTIONS_FORMAT, NULL, NULL, Setting_NonCore, &cfg.options[5], NULL, NULL },
#endif
#if N_AXIS > 6
    { VA_SETTING_OPTIONS + 3, Group_Stepper, "Velocity axis 6 options", NULL, Format_Bitfield, VA_OPTIONS_FORMAT, NULL, NULL, Setting_NonCore, &cfg.options[6], NULL, NULL },
#endif
#if N_AXIS > 7
    { VA_SETTING_OPTIONS + 4, Group_Stepper, "Velocity axis 7 options", NULL, Format_Bitfield, VA_OPTIONS_FORMAT, NULL, NULL, Setting_NonCore, &cfg.options[7], NULL, NULL },
#endif
    { VA_SETTING_ALARMS, Group_Stepper, "Velocity axes stop on alarm", NULL, Format_Bitfield,
      "Hard limit,Soft limit,Abort cycle,Probe fail initial,Probe fail contact,Homing fail reset,Homing fail door,"
      "Homing fail pulloff,Homing fail approach,E-stop,Homing required,Limits engaged,Probe protect,Spindle,"
      "Homing fail autosquare,Self-test failed,Motor fault,Homing fail,Modbus exception,Expander exception,NVS failed,Client timeout",
      NULL, NULL, Setting_NonCoreFn, set_alarms, get_alarms, NULL }
};

static const setting_descr_t va_settings_descr[] = {
    { VA_SETTING_RESERVED, "Axes (A and above) reserved for independent velocity control. Reserved axes are rejected by G-code, jogging, homing and Direct Motion." },
    { VA_SETTING_OPTIONS, "Options for velocity axis index 3 (A)." },
#if N_AXIS > 4
    { VA_SETTING_OPTIONS + 1, "Options for velocity axis index 4 (B)." },
#endif
#if N_AXIS > 5
    { VA_SETTING_OPTIONS + 2, "Options for velocity axis index 5 (C)." },
#endif
#if N_AXIS > 6
    { VA_SETTING_OPTIONS + 3, "Options for velocity axis index 6 (U)." },
#endif
#if N_AXIS > 7
    { VA_SETTING_OPTIONS + 4, "Options for velocity axis index 7 (V)." },
#endif
    { VA_SETTING_ALARMS, "Alarms that immediately stop all velocity axes. E-stop always stops." }
};

static void va_settings_save (void)
{
    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&cfg, sizeof(va_settings_t), true);
}

static void va_settings_restore (void)
{
    memset(&cfg, 0, sizeof(va_settings_t));
    cfg.alarm_stop = VA_ALARM_DEFAULT;

    va_settings_save();
}

static void va_settings_load (void)
{
    if(hal.nvs.memcpy_from_nvs((uint8_t *)&cfg, nvs_address, sizeof(va_settings_t), true) != NVS_TransferResult_OK)
        va_settings_restore();

    cfg.reserved &= AXES_BITMASK & ~VA_XYZ_MASK;
    cfg.alarm_stop |= VA_ALARM_BIT(Alarm_EStop);

    va_apply_reservation();
}

static const status_detail_t va_errors[] = {
    { VELOCITY_AXES_STATUS_RESERVED, "Axis is reserved for velocity control." }
};

static error_details_t va_error_details = {
    .n_errors = sizeof(va_errors) / sizeof(status_detail_t),
    .errors = va_errors
};

static void onReportOptions (bool newopt)
{
    on_report_options(newopt);

    if(!newopt)
        report_plugin("Velocity axes", "0.01");
}

void velocity_axes_init (void)
{
    static setting_details_t setting_details = {
        .settings = va_settings,
        .n_settings = sizeof(va_settings) / sizeof(setting_detail_t),
        .descriptions = va_settings_descr,
        .n_descriptions = sizeof(va_settings_descr) / sizeof(setting_descr_t),
        .save = va_settings_save,
        .load = va_settings_load,
        .restore = va_settings_restore
    };

    uint_fast8_t idx;

    if(!lm_init() || !velocity_axes_register_backend(&local_backend) || !(nvs_address = nvs_alloc(sizeof(va_settings_t)))) {
        task_run_on_startup(report_warning, "Velocity axes failed to initialize!");
        return;
    }

    for(idx = VA_FIRST_AXIS; idx < N_AXIS; idx++) {
        strcpy(reserved_detail[idx], "Axis ");
        strcat(reserved_detail[idx], axis_letter[idx]);
        strcat(reserved_detail[idx], " is reserved for velocity control");
        strcpy(home_cmd_names[idx - VA_FIRST_AXIS], "H");
        strcat(home_cmd_names[idx - VA_FIRST_AXIS], axis_letter[idx]);
    }

    errors_register(&va_error_details);
    system_register_commands(&va_command_list);

    memcpy(&user_mcode, &grbl.user_mcode, sizeof(user_mcode_ptrs_t));
    grbl.user_mcode.check = mcode_check;
    grbl.user_mcode.validate = mcode_validate;
    grbl.user_mcode.execute = mcode_execute;

    on_report_options = grbl.on_report_options;
    grbl.on_report_options = onReportOptions;

    on_execute_realtime = grbl.on_execute_realtime;
    grbl.on_execute_realtime = onExecuteRealtime;

    on_execute_delay = grbl.on_execute_delay;
    grbl.on_execute_delay = onExecuteDelay;

    on_state_change = grbl.on_state_change;
    grbl.on_state_change = onStateChange;

    on_reset = grbl.on_reset;
    grbl.on_reset = onReset;

    on_soft_reset = grbl.on_soft_reset;
    grbl.on_soft_reset = onSoftReset;

    on_motion_dispatch = grbl.on_motion_dispatch;
    grbl.on_motion_dispatch = onMotionDispatch;

    home_machine = grbl.home_machine;
    grbl.home_machine = onHomeMachine;

    settings_register(&setting_details);
}

#endif // VELOCITY_AXES_ENABLE
