#include "driver.h"

#if VELOCITY_JOG_ENABLE

#include <string.h>

#include "grbl/hal.h"
#include "grbl/system.h"
#include "grbl/stream.h"
#include "grbl/state_machine.h"
#include "grbl/stepper.h"
#include "grbl/motion_control.h"
#include "grbl/protocol.h"
#include "grbl/core_handlers.h"
#include "grbl/gcode.h"

#include "direct_motion.h"

#define DM_FRAME_SYNC0 0xA5u
#define DM_FRAME_SYNC1 0x5Au
#define DM_FRAME_VERSION 1u

#define DM_MSG_SET_VELOCITY_XYZ    0x01u  // Backwards compatible vector set.
#define DM_MSG_HEARTBEAT           0x02u
#define DM_MSG_STOP_LEGACY         0x03u

#define DM_MSG_SET_VELOCITY_MASKED 0x10u
#define DM_MSG_MOVE_ABSOLUTE       0x11u
#define DM_MSG_MOVE_RELATIVE       0x12u
#define DM_MSG_STOP_AXIS           0x13u
#define DM_MSG_STOP_ALL            0x14u

#define DM_FIXED_SHIFT 16
#define DM_FIXED_ONE   (1 << DM_FIXED_SHIFT)

#define DM_AXES 3u

// 1 kHz control loop.
#define DM_CTRL_HZ 1000u
// High-rate step emission ISR cadence for deterministic phase integration.
#define DM_ISR_HZ 40000u
#define DM_WATCHDOG_MS 1000u

#define DM_DEFAULT_VEL_MM_S 120.0f
#define DM_DEFAULT_ACC_MM_S2 600.0f

typedef enum {
    AxisMode_Inactive = 0,
    AxisMode_Velocity,
    AxisMode_Position
} axis_mode_t;

typedef enum {
    AxisPhase_Inactive = 0,
    AxisPhase_Accelerating,
    AxisPhase_Cruising,
    AxisPhase_Decelerating,
    AxisPhase_TargetReached
} axis_phase_t;

typedef enum {
    Rx_Sync0 = 0,
    Rx_Sync1,
    Rx_Version,
    Rx_Type,
    Rx_Len,
    Rx_Reserved,
    Rx_SeqLo,
    Rx_SeqHi,
    Rx_Payload,
    Rx_CrcLo,
    Rx_CrcHi
} rx_state_t;

typedef struct {
    rx_state_t state;
    uint8_t type;
    uint8_t len;
    uint8_t payload[40];
    uint8_t payload_idx;
    uint16_t sequence;
    uint16_t crc;
    uint16_t rx_crc;
} dm_rx_t;

typedef struct {
    uint8_t mask;
    int32_t mm_s_q16[DM_AXES];
} dm_velocity_masked_t;

typedef struct {
    uint8_t axis;
    int32_t target_mm_q16;
    int32_t vmax_mm_s_q16;
    int32_t acc_mm_s2_q16;
} dm_move_abs_t;

typedef struct {
    uint8_t axis;
    int32_t delta_mm_q16;
    int32_t vmax_mm_s_q16;
    int32_t acc_mm_s2_q16;
} dm_move_rel_t;

typedef struct {
    uint8_t axis;
} dm_stop_axis_t;

typedef struct {
    int32_t mm_s_q16[DM_AXES];
} dm_vel_xyz_t;

#define DM_LEN_SET_VELOCITY_XYZ     12u
#define DM_LEN_SET_VELOCITY_MASKED  13u
#define DM_LEN_MOVE_ABS             13u
#define DM_LEN_MOVE_REL             13u
#define DM_LEN_STOP_AXIS             1u

typedef struct {
    axis_mode_t mode;
    axis_phase_t phase;
    int64_t target_pos_steps_q16;
    int32_t target_vel_steps_s_q16;     // velocity mode target.
    int32_t commanded_vel_steps_s_q16;  // trajectory command.
    int32_t current_vel_steps_s_q16;    // ramped actual.
    int32_t vmax_steps_s_q16;
    int32_t acc_steps_s2_q16;
    int64_t phase_accum_num;
    bool target_valid;
} dm_axis_t;

typedef struct {
    dm_axis_t axis[DM_AXES];
    uint16_t last_sequence;
    uint32_t last_velocity_heartbeat_ms;
    uint32_t ctrl_time_ms;
    uint16_t ctrl_subtick;
    bool active;
    bool needs_idle_sync;
    bool isr_rate_forced;
    volatile uint8_t mode_snapshot[DM_AXES];
    volatile uint8_t phase_snapshot[DM_AXES];
    volatile bool active_snapshot;
    volatile int32_t target_vel_snapshot[DM_AXES];
    volatile int32_t commanded_vel_snapshot[DM_AXES];
    volatile int32_t current_vel_snapshot[DM_AXES];
} dm_state_t;

static dm_rx_t rx = {0};
static dm_state_t dm = {0};

static int32_t steps_per_mm_q16[DM_AXES] = {0};
static int32_t default_vmax_steps_s_q16[DM_AXES] = {0};
static int32_t default_acc_steps_s2_q16[DM_AXES] = {0};
static int32_t default_acc_step_q16[DM_AXES] = {0};

static stepper_interrupt_callback_ptr stepper_irq_org = NULL;
static on_unknown_realtime_cmd_ptr on_unknown_realtime_org = NULL;
static on_execute_realtime_ptr on_execute_realtime_org = NULL;
static bool dm_hooks_installed = false;

static int32_t accel_step_per_tick_q16 (int32_t acc_steps_s2_q16);

static int32_t q16_steps_round_to_int (int64_t steps_q16)
{
    if(steps_q16 >= 0)
        return (int32_t)((steps_q16 + (DM_FIXED_ONE / 2)) >> DM_FIXED_SHIFT);

    return (int32_t)((steps_q16 - (DM_FIXED_ONE / 2)) >> DM_FIXED_SHIFT);
}

static bool dm_stepper_cb_valid (stepper_interrupt_callback_ptr cb)
{
    uintptr_t p = (uintptr_t)cb;

    if(cb == NULL)
        return false;

    // Cortex-M Thumb pointers are odd. Reject clearly invalid low addresses.
    if((p & 0x1u) == 0 || p < 0x10001u)
        return false;

    // Internal flash text range for STM32H743II (2MB flash @ 0x08000000).
    if(p >= 0x08000001u && p <= 0x081FFFFFu)
        return true;

    // Allow SRAM/ITCM resident code pointers used by some ISR placement schemes.
    if((p >= 0x20000001u && p <= 0x200FFFFFu) ||
       (p >= 0x24000001u && p <= 0x2407FFFFu) ||
       (p >= 0x00000001u && p <= 0x0003FFFFu))
        return true;

    return false;
}

typedef enum {
    DM_Err_None = 0,
    DM_Err_StaleSequence = 1,
    DM_Err_UnknownType = 2,
    DM_Err_BadLength = 3,
    DM_Err_CommandRejected = 4,
    DM_Err_BadCrc = 5
} dm_error_t;

static inline int32_t iabs32 (int32_t v)
{
    return v < 0 ? -v : v;
}

static inline int32_t clamp_i32 (int32_t v, int32_t min_v, int32_t max_v)
{
    return v < min_v ? min_v : (v > max_v ? max_v : v);
}

static inline uint8_t axis_bit (uint8_t axis)
{
    return (uint8_t)(1u << axis);
}

static inline bool axis_valid (uint8_t axis)
{
    return axis < DM_AXES;
}

static bool dm_refresh_axis_scalars (uint8_t axis)
{
    if(!axis_valid(axis))
        return false;

    float spm = settings.axis[axis].steps_per_mm;
    float max_mm_s = settings.axis[axis].max_rate / 60.0f;
    float acc_mm_s2 = settings.axis[axis].acceleration / 3600.0f;

    if(spm <= 0.0f)
        return false;

    if(max_mm_s <= 0.0f)
        max_mm_s = DM_DEFAULT_VEL_MM_S;
    if(acc_mm_s2 <= 0.0f)
        acc_mm_s2 = DM_DEFAULT_ACC_MM_S2;

    steps_per_mm_q16[axis] = (int32_t)(spm * (float)DM_FIXED_ONE);
    default_vmax_steps_s_q16[axis] = (int32_t)((max_mm_s * spm) * (float)DM_FIXED_ONE);
    default_acc_steps_s2_q16[axis] = (int32_t)((acc_mm_s2 * spm) * (float)DM_FIXED_ONE);
    default_acc_step_q16[axis] = accel_step_per_tick_q16(default_acc_steps_s2_q16[axis]);

    return steps_per_mm_q16[axis] > 0;
}

static uint64_t isqrt_u64 (uint64_t x)
{
    uint64_t res = 0;
    uint64_t bit = (uint64_t)1 << 62;

    while(bit > x)
        bit >>= 2;

    while(bit) {
        if(x >= res + bit) {
            x -= res + bit;
            res = (res >> 1) + bit;
        } else
            res >>= 1;
        bit >>= 2;
    }

    return res;
}

static void crc16_ccitt_update (uint16_t *crc, uint8_t data)
{
    *crc ^= (uint16_t)data << 8;
    for(uint8_t b = 0; b < 8; b++)
        *crc = (*crc & 0x8000u) ? (uint16_t)((*crc << 1) ^ 0x1021u) : (uint16_t)(*crc << 1);
}

static bool seq_is_newer (uint16_t seq, uint16_t last)
{
    return (uint16_t)(seq - last) < 0x8000u;
}

static inline int32_t rd_i32_le (const uint8_t *p)
{
    return (int32_t)((uint32_t)p[0] |
                    ((uint32_t)p[1] << 8) |
                    ((uint32_t)p[2] << 16) |
                    ((uint32_t)p[3] << 24));
}

static void dm_report_error (uint16_t sequence, uint8_t type, dm_error_t err, uint8_t detail)
{
    char msg[56];

    strcpy(msg, "[DM:ERR,");
    strcat(msg, uitoa((uint32_t)sequence));
    strcat(msg, ",");
    strcat(msg, uitoa((uint32_t)type));
    strcat(msg, ",");
    strcat(msg, uitoa((uint32_t)err));
    strcat(msg, ",");
    strcat(msg, uitoa((uint32_t)detail));
    strcat(msg, "]" ASCII_EOL);

    hal.stream.write(msg);
}

static void dm_report_ok (uint16_t sequence, uint8_t type)
{
    char msg[32];

    strcpy(msg, "[DM:OK,");
    strcat(msg, uitoa((uint32_t)sequence));
    strcat(msg, ",");
    strcat(msg, uitoa((uint32_t)type));
    strcat(msg, "]" ASCII_EOL);

    hal.stream.write(msg);
}

static void dm_report_state (const char *state)
{
    char msg[40];

    strcpy(msg, "[DM:STATE,");
    strcat(msg, state);
    strcat(msg, "]" ASCII_EOL);

    hal.stream.write(msg);
}

static void dm_report_axis_phase (uint8_t axis, uint8_t mode, uint8_t phase)
{
    char msg[48];

    strcpy(msg, "[DM:PHASE,");
    strcat(msg, uitoa((uint32_t)axis));
    strcat(msg, ",");
    strcat(msg, uitoa((uint32_t)mode));
    strcat(msg, ",");
    strcat(msg, uitoa((uint32_t)phase));
    strcat(msg, "]" ASCII_EOL);

    hal.stream.write(msg);
}

static void dm_append_i32 (char *msg, int32_t v)
{
    if(v < 0) {
        strcat(msg, "-");
        v = -v;
    }
    strcat(msg, uitoa((uint32_t)v));
}

static void dm_report_move (uint8_t axis, int32_t cur_steps, int32_t target_steps)
{
    char msg[96];

    strcpy(msg, "[DM:MOVE,");
    strcat(msg, uitoa((uint32_t)axis));
    strcat(msg, ",cur=");
    dm_append_i32(msg, cur_steps);
    strcat(msg, ",target=");
    dm_append_i32(msg, target_steps);
    strcat(msg, ",delta=");
    dm_append_i32(msg, target_steps - cur_steps);
    strcat(msg, "]" ASCII_EOL);

    hal.stream.write(msg);
}

static void dm_report_rx (uint16_t sequence, uint8_t type, uint8_t len)
{
    char msg[56];

    strcpy(msg, "[DM:RX,");
    strcat(msg, uitoa((uint32_t)sequence));
    strcat(msg, ",");
    strcat(msg, uitoa((uint32_t)type));
    strcat(msg, ",len=");
    strcat(msg, uitoa((uint32_t)len));
    strcat(msg, "]" ASCII_EOL);

    hal.stream.write(msg);
}

static void dm_report_rx_move_abs (const dm_move_abs_t *cmd)
{
    char msg[96];

    strcpy(msg, "[DM:RXMOVE,");
    strcat(msg, uitoa((uint32_t)cmd->axis));
    strcat(msg, ",t=");
    dm_append_i32(msg, cmd->target_mm_q16);
    strcat(msg, ",v=");
    dm_append_i32(msg, cmd->vmax_mm_s_q16);
    strcat(msg, ",a=");
    dm_append_i32(msg, cmd->acc_mm_s2_q16);
    strcat(msg, "]" ASCII_EOL);

    hal.stream.write(msg);
}

static void dm_report_velocity (uint8_t axis, int32_t target_steps_s_q16, int32_t commanded_steps_s_q16, int32_t current_steps_s_q16)
{
    char msg[120];

    strcpy(msg, "[DM:VEL,");
    strcat(msg, uitoa((uint32_t)axis));
    strcat(msg, ",t=");
    dm_append_i32(msg, target_steps_s_q16);
    strcat(msg, ",cmd=");
    dm_append_i32(msg, commanded_steps_s_q16);
    strcat(msg, ",c=");
    dm_append_i32(msg, current_steps_s_q16);
    strcat(msg, "]" ASCII_EOL);

    hal.stream.write(msg);
}

static bool dm_can_start (void)
{
    sys_state_t state = state_get();

    if(state & (STATE_CYCLE|STATE_HOMING|STATE_CHECK_MODE|STATE_SLEEP|STATE_ESTOP|STATE_ALARM))
        return false;

    if(plan_get_current_block() != NULL)
        return false;

    return true;
}

static void dm_set_state_jog_if_needed (void)
{
    sys_state_t state = state_get();

    if((state == STATE_IDLE || state == STATE_TOOL_CHANGE) && !dm.active) {
        state_set(STATE_JOG);
        st_wake_up();
        dm.active = true;
        dm.isr_rate_forced = false;
        dm_report_state("JOG");
    }
}

static void dm_axis_set_defaults (uint8_t axis)
{
    dm.axis[axis].vmax_steps_s_q16 = default_vmax_steps_s_q16[axis];
    dm.axis[axis].acc_steps_s2_q16 = default_acc_steps_s2_q16[axis];
}

static void dm_axis_stop (uint8_t axis)
{
    if(!axis_valid(axis))
        return;

    dm.axis[axis].target_vel_steps_s_q16 = 0;
    dm.axis[axis].commanded_vel_steps_s_q16 = 0;
    dm.axis[axis].target_valid = false;

    if(dm.axis[axis].mode == AxisMode_Position)
        dm.axis[axis].mode = AxisMode_Velocity;

    dm.axis[axis].phase = AxisPhase_Decelerating;
    dm_report_axis_phase(axis, (uint8_t)dm.axis[axis].mode, (uint8_t)dm.axis[axis].phase);
}

static void dm_stop_all (void)
{
    for(uint8_t axis = 0; axis < DM_AXES; axis++)
        dm_axis_stop(axis);
}

static void dm_stop_immediate (void)
{
    dm.active = false;
    dm.needs_idle_sync = false;
    dm.isr_rate_forced = false;
    for(uint8_t axis = 0; axis < DM_AXES; axis++) {
        dm.axis[axis].mode = AxisMode_Inactive;
        dm.axis[axis].phase = AxisPhase_Inactive;
        dm.axis[axis].target_valid = false;
        dm.axis[axis].target_vel_steps_s_q16 = 0;
        dm.axis[axis].commanded_vel_steps_s_q16 = 0;
        dm.axis[axis].current_vel_steps_s_q16 = 0;
        dm.axis[axis].phase_accum_num = 0;
    }
}

static bool dm_check_target_in_limits (uint8_t axis, int64_t target_steps_q16)
{
    float target[N_AXIS] = {0};

    system_convert_array_steps_to_mpos(target, sys.position);
    target[axis] = (float)target_steps_q16 / (float)(DM_FIXED_ONE * steps_per_mm_q16[axis]) * (float)DM_FIXED_ONE;

    return grbl.check_travel_limits(target, (axes_signals_t){ .mask = axis_bit(axis) }, true, &sys.work_envelope);
}

static int64_t mm_q16_to_steps_q16 (uint8_t axis, int32_t mm_q16)
{
    if(steps_per_mm_q16[axis] == 0)
        return 0;
    return ((int64_t)mm_q16 * (int64_t)steps_per_mm_q16[axis]) >> DM_FIXED_SHIFT;
}

static int32_t mm_s_q16_to_steps_s_q16 (uint8_t axis, int32_t mm_s_q16)
{
    if(steps_per_mm_q16[axis] == 0)
        return 0;
    return (int32_t)(((int64_t)mm_s_q16 * (int64_t)steps_per_mm_q16[axis]) >> DM_FIXED_SHIFT);
}

static int32_t mm_s2_q16_to_steps_s2_q16 (uint8_t axis, int32_t mm_s2_q16)
{
    if(steps_per_mm_q16[axis] == 0)
        return 0;
    return (int32_t)(((int64_t)mm_s2_q16 * (int64_t)steps_per_mm_q16[axis]) >> DM_FIXED_SHIFT);
}

static int32_t accel_step_per_tick_q16 (int32_t acc_steps_s2_q16)
{
    return acc_steps_s2_q16 / (int32_t)DM_CTRL_HZ;
}

static void dm_set_velocity_axis (uint8_t axis, int32_t mm_s_q16)
{
    dm_axis_t *a = &dm.axis[axis];

    if(!dm_refresh_axis_scalars(axis))
        return;

    dm_axis_set_defaults(axis);

    // For direct jog velocity mode, follow host-requested speed directly.
    // Positional move limits remain enforced in move planning paths.
    int32_t new_target = mm_s_q16_to_steps_s_q16(axis, mm_s_q16);

    // Host key-repeat may resend the same jog vector frequently.
    // Ignore redundant target updates to avoid unnecessary phase churn and log load.
    if(a->mode == AxisMode_Velocity && a->target_vel_steps_s_q16 == new_target)
        return;

    if(a->mode != AxisMode_Velocity) {
        a->mode = AxisMode_Velocity;
        a->target_valid = false;
        a->phase = AxisPhase_Accelerating;
        dm_report_axis_phase(axis, (uint8_t)a->mode, (uint8_t)a->phase);
    } else if(new_target == 0 && a->target_vel_steps_s_q16 != 0) {
        a->phase = AxisPhase_Decelerating;
        dm_report_axis_phase(axis, (uint8_t)a->mode, (uint8_t)a->phase);
    }

    // If already jogging, treat repeated messages as target updates only.
    a->target_vel_steps_s_q16 = new_target;
}

static bool dm_move_absolute_axis (uint8_t axis, int32_t target_mm_q16, int32_t vmax_mm_s_q16, int32_t acc_mm_s2_q16)
{
    if(!axis_valid(axis) || (!dm_can_start() && !dm.active))
        return false;

    if(!dm_refresh_axis_scalars(axis))
        return false;

    dm_axis_t *a = &dm.axis[axis];

    int64_t target_steps_q16 = mm_q16_to_steps_q16(axis, target_mm_q16);
    int32_t target_steps_i = q16_steps_round_to_int(target_steps_q16);
    target_steps_q16 = (int64_t)target_steps_i << DM_FIXED_SHIFT;

    if(!dm_check_target_in_limits(axis, target_steps_q16))
        return false;

    int32_t vmax = vmax_mm_s_q16 <= 0 ? default_vmax_steps_s_q16[axis] : mm_s_q16_to_steps_s_q16(axis, vmax_mm_s_q16);
    int32_t acc = acc_mm_s2_q16 <= 0 ? default_acc_steps_s2_q16[axis] : mm_s2_q16_to_steps_s2_q16(axis, acc_mm_s2_q16);

    a->mode = AxisMode_Position;
    a->target_pos_steps_q16 = target_steps_q16;
    a->target_valid = true;
    a->vmax_steps_s_q16 = iabs32(vmax);
    a->acc_steps_s2_q16 = iabs32(acc);
    a->phase = AxisPhase_Accelerating;

    dm_report_move(axis, sys.position[axis], target_steps_i);

    dm_set_state_jog_if_needed();

    return true;
}

static bool dm_move_relative_axis (uint8_t axis, int32_t delta_mm_q16, int32_t vmax_mm_s_q16, int32_t acc_mm_s2_q16)
{
    if(!axis_valid(axis))
        return false;

    if(!dm_refresh_axis_scalars(axis))
        return false;

    int64_t cur_steps_q16 = (int64_t)sys.position[axis] << DM_FIXED_SHIFT;
    int64_t dst_steps_q16 = cur_steps_q16 + mm_q16_to_steps_q16(axis, delta_mm_q16);
    int32_t dst_mm_q16 = (int32_t)((dst_steps_q16 << DM_FIXED_SHIFT) / steps_per_mm_q16[axis]);

    return dm_move_absolute_axis(axis, dst_mm_q16, vmax_mm_s_q16, acc_mm_s2_q16);
}

static void dm_update_axis_profile (uint8_t axis)
{
    dm_axis_t *a = &dm.axis[axis];

    if(a->mode == AxisMode_Inactive) {
        a->phase = AxisPhase_Inactive;
        return;
    }

    if(a->mode == AxisMode_Velocity) {
        a->commanded_vel_steps_s_q16 = a->target_vel_steps_s_q16;
        if(a->current_vel_steps_s_q16 < a->commanded_vel_steps_s_q16)
            a->phase = AxisPhase_Accelerating;
        else if(a->current_vel_steps_s_q16 > a->commanded_vel_steps_s_q16)
            a->phase = AxisPhase_Decelerating;
        else {
            if(a->current_vel_steps_s_q16 == 0) {
                a->phase = AxisPhase_TargetReached;
                a->mode = AxisMode_Inactive;
            } else
                a->phase = AxisPhase_Cruising;
        }
        return;
    }

    int64_t cur_pos_q16 = (int64_t)sys.position[axis] << DM_FIXED_SHIFT;
    int64_t dist_q16 = a->target_pos_steps_q16 - cur_pos_q16;
    uint64_t abs_dist_q16 = (uint64_t)(dist_q16 >= 0 ? dist_q16 : -dist_q16);
    int32_t cur_steps = sys.position[axis];
    int32_t target_steps = (int32_t)(a->target_pos_steps_q16 >> DM_FIXED_SHIFT);
    int32_t step_err = target_steps - cur_steps;
    int32_t v = a->current_vel_steps_s_q16;
    int32_t abs_v = iabs32(v);
    int32_t acc = a->acc_steps_s2_q16 > 0 ? a->acc_steps_s2_q16 : default_acc_steps_s2_q16[axis];
    int32_t acc_step = accel_step_per_tick_q16(acc);

    // Deterministic completion: stop only when exactly on target step and
    // velocity has been reduced sufficiently.
    if(step_err == 0 && abs_v <= acc_step) {
        a->commanded_vel_steps_s_q16 = 0;
        a->current_vel_steps_s_q16 = 0;
        a->mode = AxisMode_Inactive;
        a->phase = AxisPhase_TargetReached;
        a->target_valid = false;
        return;
    }

    // If exactly on target but still moving, command a clean decel to zero.
    if(step_err == 0) {
        a->commanded_vel_steps_s_q16 = 0;
        a->phase = AxisPhase_Decelerating;
        return;
    }

    int32_t dir = dist_q16 >= 0 ? 1 : -1;
    uint64_t stop_dist_q16 = (uint64_t)(((int64_t)abs_v * (int64_t)abs_v) / (2 * (int64_t)acc));

    uint64_t vlim_sq = 2ULL * (uint64_t)acc * abs_dist_q16;
    int32_t vlim_dist = (int32_t)isqrt_u64(vlim_sq);
    int32_t desired_abs = vlim_dist < a->vmax_steps_s_q16 ? vlim_dist : a->vmax_steps_s_q16;

    if(abs_dist_q16 <= stop_dist_q16)
        a->phase = AxisPhase_Decelerating;
    else if(abs_v < desired_abs)
        a->phase = AxisPhase_Accelerating;
    else
        a->phase = AxisPhase_Cruising;

    a->commanded_vel_steps_s_q16 = dir * desired_abs;
}

static void dm_update_ramps_1khz (void)
{
    bool any_active = false;

    for(uint8_t axis = 0; axis < DM_AXES; axis++) {
        dm_axis_t *a = &dm.axis[axis];

        dm_update_axis_profile(axis);

        int32_t acc = a->acc_steps_s2_q16 > 0 ? a->acc_steps_s2_q16 : default_acc_steps_s2_q16[axis];
        int32_t dv = accel_step_per_tick_q16(acc);
        int32_t cur = a->current_vel_steps_s_q16;
        int32_t cmd = a->commanded_vel_steps_s_q16;

        if(cur < cmd) {
            cur += dv;
            if(cur > cmd)
                cur = cmd;
        } else if(cur > cmd) {
            cur -= dv;
            if(cur < cmd)
                cur = cmd;
        }

        a->current_vel_steps_s_q16 = cur;

        if(a->mode != AxisMode_Inactive || cur != 0)
            any_active = true;

        dm.mode_snapshot[axis] = (uint8_t)a->mode;
        dm.phase_snapshot[axis] = (uint8_t)a->phase;
        dm.target_vel_snapshot[axis] = a->target_vel_steps_s_q16;
        dm.commanded_vel_snapshot[axis] = a->commanded_vel_steps_s_q16;
        dm.current_vel_snapshot[axis] = a->current_vel_steps_s_q16;
    }

    dm.active = any_active;
    dm.active_snapshot = any_active;

    if(!any_active)
        dm.needs_idle_sync = true;
}

static void dm_watchdog_velocity_modes (void)
{
    bool any_nonzero_velocity_target = false;
    for(uint8_t axis = 0; axis < DM_AXES; axis++) {
        if(dm.axis[axis].mode == AxisMode_Velocity && dm.axis[axis].target_vel_steps_s_q16 != 0) {
            any_nonzero_velocity_target = true;
            break;
        }
    }

    // If no active velocity target exists, watchdog has nothing to police.
    if(!any_nonzero_velocity_target)
        return;

    uint32_t now = dm.ctrl_time_ms;
    if((now - dm.last_velocity_heartbeat_ms) <= DM_WATCHDOG_MS)
        return;

    for(uint8_t axis = 0; axis < DM_AXES; axis++) {
        if(dm.axis[axis].mode == AxisMode_Velocity)
            dm.axis[axis].target_vel_steps_s_q16 = 0;
    }
}

static void dm_generate_steps (void)
{
    stepper_t pulse = {0};
    axes_signals_t step = {0};
    axes_signals_t dir = {0};
    const int64_t step_threshold = (int64_t)DM_FIXED_ONE * (int64_t)DM_ISR_HZ;

    for(uint8_t axis = 0; axis < DM_AXES; axis++) {

        dm_axis_t *a = &dm.axis[axis];
        int32_t v = a->current_vel_steps_s_q16;

        if(v == 0)
            continue;

        // Exact phase integration in steps/s domain. This avoids per-tick
        // truncation artifacts that can make jog feel coarse.
        a->phase_accum_num += iabs32(v);

        if(a->phase_accum_num >= step_threshold) {
            a->phase_accum_num -= step_threshold;

            bool neg = v < 0;

            if(a->mode == AxisMode_Position && a->target_valid) {
                int32_t target_step = (int32_t)(a->target_pos_steps_q16 >> DM_FIXED_SHIFT);
                int32_t cur_step = sys.position[axis];
                int32_t next_step = cur_step + (neg ? -1 : 1);

                // Hard clamp: never emit a step that crosses past the target.
                if((!neg && next_step > target_step) || (neg && next_step < target_step)) {
                    a->current_vel_steps_s_q16 = 0;
                    a->commanded_vel_steps_s_q16 = 0;
                    a->target_vel_steps_s_q16 = 0;
                    a->phase_accum_num = 0;
                    a->mode = AxisMode_Inactive;
                    a->phase = AxisPhase_TargetReached;
                    a->target_valid = false;
                    continue;
                }
            }

            switch(axis) {
                case 0:
                    step.x = On;
                    dir.x = neg;
                    sys.position[X_AXIS] += neg ? -1 : 1;
                    break;
                case 1:
                    step.y = On;
                    dir.y = neg;
                    sys.position[Y_AXIS] += neg ? -1 : 1;
                    break;
                case 2:
                    step.z = On;
                    dir.z = neg;
                    sys.position[Z_AXIS] += neg ? -1 : 1;
                    break;
                default:
                    break;
            }

            if(a->mode == AxisMode_Position && a->target_valid) {
                int32_t target_step = (int32_t)(a->target_pos_steps_q16 >> DM_FIXED_SHIFT);
                int32_t acc = a->acc_steps_s2_q16 > 0 ? a->acc_steps_s2_q16 : default_acc_steps_s2_q16[axis];
                int32_t acc_step = accel_step_per_tick_q16(acc);

                if(sys.position[axis] == target_step && iabs32(a->current_vel_steps_s_q16) <= (acc_step * 2)) {
                    a->current_vel_steps_s_q16 = 0;
                    a->commanded_vel_steps_s_q16 = 0;
                    a->target_vel_steps_s_q16 = 0;
                    a->phase_accum_num = 0;
                    a->mode = AxisMode_Inactive;
                    a->phase = AxisPhase_TargetReached;
                    a->target_valid = false;
                }
            }
        }
    }

    if(step.bits) {
        pulse.step_out.bits = step.bits;
        pulse.dir_out.bits = dir.bits;
        pulse.dir_changed.bits = 0xFFu;
        hal.stepper.pulse_start(&pulse);
    }
}

static void dm_interrupt_callback (void)
{
    if(!dm.active) {
        // Delegate to core ISR whenever core motion states are active.
        // st_is_stepping() is too strict here because normal gcode motion
        // startup can have stepping=true while st.exec_block is still NULL
        // until the first ISR service call prepares a segment.
        sys_state_t state = state_get();
        // IMPORTANT: Do not delegate on STATE_JOG alone. Direct-motion can
        // transiently leave state in JOG while no core step segment exists,
        // and calling stepper_driver_interrupt_handler() in that window can
        // hit the segment-empty path and hardfault.
        if(st_is_stepping() || state == STATE_CYCLE || state == STATE_HOMING || state == STATE_HOLD)
            stepper_driver_interrupt_handler();
        return;
    }

    if(sys.rt_exec_alarm || (sys.rt_exec_state & EXEC_RESET)) {
        dm_stop_immediate();
        return;
    }

    // Force deterministic callback cadence while direct-motion is active.
    // Set once per active session to avoid unnecessary ISR overhead.
    if(!dm.isr_rate_forced) {
        uint32_t cpt = hal.f_step_timer / DM_ISR_HZ;
        if(cpt == 0)
            cpt = 1;
        hal.stepper.cycles_per_tick(cpt);
        dm.isr_rate_forced = true;
    }

    // Run 1kHz control loop from ISR cadence so motion does not depend on SysTick
    // scheduling under high interrupt load.
    dm.ctrl_subtick++;
    if(dm.ctrl_subtick >= (DM_ISR_HZ / DM_CTRL_HZ)) {
        dm.ctrl_subtick = 0;
        dm.ctrl_time_ms++;
        dm_watchdog_velocity_modes();
        dm_update_ramps_1khz();
    }

    if(dm.active)
        dm_generate_steps();
}

#if REPORT_REALTIME_AXIS_VELOCITY
bool direct_motion_get_realtime_axis_rates (float *rates)
{
    if(rates == NULL)
        return false;

    for(uint8_t axis = 0; axis < N_AXIS; axis++)
        rates[axis] = 0.0f;

    if(!dm.active_snapshot)
        return false;

    for(uint8_t axis = 0; axis < DM_AXES && axis < N_AXIS; axis++) {
        int32_t spm_q16 = steps_per_mm_q16[axis];
        int32_t v_steps_s_q16 = dm.current_vel_snapshot[axis];

        if(spm_q16 != 0 && v_steps_s_q16 != 0)
            rates[axis] = ((float)v_steps_s_q16 * 60.0f) / (float)spm_q16;
    }

    return true;
}
#endif

static void dm_apply_velocity_xyz_legacy (const dm_vel_xyz_t *vec)
{
    if(!dm_can_start() && !dm.active)
        return;

    for(uint8_t axis = 0; axis < DM_AXES; axis++)
        dm_set_velocity_axis(axis, vec->mm_s_q16[axis]);

    dm.last_velocity_heartbeat_ms = dm.ctrl_time_ms;

    dm_set_state_jog_if_needed();
}

static void dm_apply_velocity_masked (const dm_velocity_masked_t *cmd)
{
    if(!dm_can_start() && !dm.active)
        return;

    for(uint8_t axis = 0; axis < DM_AXES; axis++) {
        if(cmd->mask & axis_bit(axis))
            dm_set_velocity_axis(axis, cmd->mm_s_q16[axis]);
    }

    dm.last_velocity_heartbeat_ms = dm.ctrl_time_ms;

    dm_set_state_jog_if_needed();
}

static void dm_handle_frame (uint8_t type, uint16_t sequence, const uint8_t *payload, uint8_t len)
{
    const uint8_t *p = payload;
    uint8_t plen = len;

    if(!seq_is_newer(sequence, dm.last_sequence))
    {
        //dm_report_error(sequence, type, DM_Err_StaleSequence, plen);
        //return;
    }

    dm.last_sequence = sequence;

    switch(type) {

        case DM_MSG_SET_VELOCITY_XYZ:
            if(plen == DM_LEN_SET_VELOCITY_XYZ) {
                dm_vel_xyz_t vec = {
                    .mm_s_q16 = {
                        rd_i32_le(&p[0]),
                        rd_i32_le(&p[4]),
                        rd_i32_le(&p[8])
                    }
                };
                dm_apply_velocity_xyz_legacy(&vec);
            } else
                dm_report_error(sequence, type, DM_Err_BadLength, plen);
            break;

        case DM_MSG_SET_VELOCITY_MASKED:
            if(plen == DM_LEN_SET_VELOCITY_MASKED) {
                dm_velocity_masked_t cmd = {
                    .mask = p[0],
                    .mm_s_q16 = {
                        rd_i32_le(&p[1]),
                        rd_i32_le(&p[5]),
                        rd_i32_le(&p[9])
                    }
                };
                dm_apply_velocity_masked(&cmd);
            } else
                dm_report_error(sequence, type, DM_Err_BadLength, plen);
            break;

        case DM_MSG_MOVE_ABSOLUTE:
            dm_report_rx(sequence, type, plen);
            if(plen == DM_LEN_MOVE_ABS) {
                dm_move_abs_t cmd = {
                    .axis = p[0],
                    .target_mm_q16 = rd_i32_le(&p[1]),
                    .vmax_mm_s_q16 = rd_i32_le(&p[5]),
                    .acc_mm_s2_q16 = rd_i32_le(&p[9])
                };
                dm_report_rx_move_abs(&cmd);
                if(dm_move_absolute_axis(cmd.axis, cmd.target_mm_q16, cmd.vmax_mm_s_q16, cmd.acc_mm_s2_q16)) {
                    dm_set_state_jog_if_needed();
                    dm_report_ok(sequence, type);
                } else
                    dm_report_error(sequence, type, DM_Err_CommandRejected, cmd.axis);
            } else
                dm_report_error(sequence, type, DM_Err_BadLength, plen);
            break;

        case DM_MSG_MOVE_RELATIVE:
            dm_report_rx(sequence, type, plen);
            if(plen == DM_LEN_MOVE_REL) {
                dm_move_rel_t cmd = {
                    .axis = p[0],
                    .delta_mm_q16 = rd_i32_le(&p[1]),
                    .vmax_mm_s_q16 = rd_i32_le(&p[5]),
                    .acc_mm_s2_q16 = rd_i32_le(&p[9])
                };
                if(dm_move_relative_axis(cmd.axis, cmd.delta_mm_q16, cmd.vmax_mm_s_q16, cmd.acc_mm_s2_q16)) {
                    dm_set_state_jog_if_needed();
                    dm_report_ok(sequence, type);
                } else
                    dm_report_error(sequence, type, DM_Err_CommandRejected, cmd.axis);
            } else
                dm_report_error(sequence, type, DM_Err_BadLength, plen);
            break;

        case DM_MSG_STOP_AXIS:
            dm_report_rx(sequence, type, plen);
            if(plen == DM_LEN_STOP_AXIS) {
                dm_stop_axis_t cmd = { .axis = p[0] };
                dm_axis_stop(cmd.axis);
                dm_report_ok(sequence, type);
            } else
                dm_report_error(sequence, type, DM_Err_BadLength, plen);
            break;

        case DM_MSG_STOP_ALL:
        case DM_MSG_STOP_LEGACY:
            dm_report_rx(sequence, type, plen);
            dm_stop_all();
            dm_report_ok(sequence, type);
            break;

        case DM_MSG_HEARTBEAT:
            dm.last_velocity_heartbeat_ms = dm.ctrl_time_ms;
            break;

        default:
            dm_report_error(sequence, type, DM_Err_UnknownType, plen);
            break;
    }
}

bool direct_motion_telnet_rx_byte (uint8_t byte)
{
    switch(rx.state) {

        case Rx_Sync0:
            if(byte == DM_FRAME_SYNC0) {
                rx.state = Rx_Sync1;
                rx.crc = 0xFFFFu;
                crc16_ccitt_update(&rx.crc, byte);
                return true;
            }
            return false;

        case Rx_Sync1:
            if(byte != DM_FRAME_SYNC1) {
                rx.state = Rx_Sync0;
                return false;
            }
            crc16_ccitt_update(&rx.crc, byte);
            rx.state = Rx_Version;
            return true;

        case Rx_Version:
            if(byte != DM_FRAME_VERSION) {
                rx.state = Rx_Sync0;
                return true;
            }
            crc16_ccitt_update(&rx.crc, byte);
            rx.state = Rx_Type;
            return true;

        case Rx_Type:
            rx.type = byte;
            crc16_ccitt_update(&rx.crc, byte);
            rx.state = Rx_Len;
            return true;

        case Rx_Len:
            rx.len = byte;
            rx.payload_idx = 0;
            if(rx.len > sizeof(rx.payload)) {
                rx.state = Rx_Sync0;
                return true;
            }
            crc16_ccitt_update(&rx.crc, byte);
            rx.state = Rx_Reserved;
            return true;

        case Rx_Reserved:
            crc16_ccitt_update(&rx.crc, byte);
            rx.state = Rx_SeqLo;
            return true;

        case Rx_SeqLo:
            rx.sequence = byte;
            crc16_ccitt_update(&rx.crc, byte);
            rx.state = Rx_SeqHi;
            return true;

        case Rx_SeqHi:
            rx.sequence |= ((uint16_t)byte << 8);
            crc16_ccitt_update(&rx.crc, byte);
            rx.state = rx.len ? Rx_Payload : Rx_CrcLo;
            return true;

        case Rx_Payload:
            rx.payload[rx.payload_idx++] = byte;
            crc16_ccitt_update(&rx.crc, byte);
            if(rx.payload_idx >= rx.len)
                rx.state = Rx_CrcLo;
            return true;

        case Rx_CrcLo:
            rx.rx_crc = byte;
            rx.state = Rx_CrcHi;
            return true;

        case Rx_CrcHi:
            rx.rx_crc |= ((uint16_t)byte << 8);
            if(rx.rx_crc == rx.crc)
                dm_handle_frame(rx.type, rx.sequence, rx.payload, rx.len);
            else
                dm_report_error(rx.sequence, rx.type, DM_Err_BadCrc, rx.len);
            rx.state = Rx_Sync0;
            return true;

        default:
            rx.state = Rx_Sync0;
            return false;
    }
}

static bool on_unknown_realtime (char c)
{
    if(on_unknown_realtime_org && on_unknown_realtime_org(c))
        return true;

    // Reserved realtime byte for direct-motion activation hint.
    if((uint8_t)c == 0xB8u) {
        if(dm_can_start())
            dm_set_state_jog_if_needed();
        return true;
    }

    return false;
}

static void on_execute_realtime (sys_state_t state)
{
    static uint8_t mode_last[DM_AXES] = {0xFFu, 0xFFu, 0xFFu};
    static uint8_t phase_last[DM_AXES] = {0xFFu, 0xFFu, 0xFFu};
    static bool active_last = false;
    static uint32_t vel_last_report_ms = 0;

    if(on_execute_realtime_org)
        on_execute_realtime_org(state);

    if(dm.active_snapshot != active_last) {
        dm_report_state(dm.active_snapshot ? "ACTIVE" : "INACTIVE");
        active_last = dm.active_snapshot;
    }

    for(uint8_t axis = 0; axis < DM_AXES; axis++) {
        uint8_t mode = dm.mode_snapshot[axis];
        uint8_t phase = dm.phase_snapshot[axis];

        if(mode != mode_last[axis] || phase != phase_last[axis]) {
            dm_report_axis_phase(axis, mode, phase);
            mode_last[axis] = mode;
            phase_last[axis] = phase;
        }
    }

    if((uint32_t)(dm.ctrl_time_ms - vel_last_report_ms) >= 100u) {
        vel_last_report_ms = dm.ctrl_time_ms;

        for(uint8_t axis = 0; axis < DM_AXES; axis++) {
            int32_t t = dm.target_vel_snapshot[axis];
            int32_t cmd = dm.commanded_vel_snapshot[axis];
            int32_t cur = dm.current_vel_snapshot[axis];

            if(dm.mode_snapshot[axis] == (uint8_t)AxisMode_Velocity || t != 0 || cmd != 0 || cur != 0)
                dm_report_velocity(axis, t, cmd, cur);
        }
    }

    if(dm.needs_idle_sync && state_get() == STATE_JOG) {
        dm.needs_idle_sync = false;
        dm_report_state("IDLE_SYNC");
        state_set(STATE_IDLE);
        sync_position();
        dm_report_state("IDLE");
    }
}

void direct_motion_init (void)
{
    memset(&dm, 0, sizeof(dm));
    memset(&rx, 0, sizeof(rx));
    rx.state = Rx_Sync0;

    for(uint8_t axis = 0; axis < DM_AXES; axis++) {
        dm_refresh_axis_scalars(axis);

        dm_axis_set_defaults(axis);
        dm.axis[axis].mode = AxisMode_Inactive;
        dm.axis[axis].phase = AxisPhase_Inactive;
        dm.mode_snapshot[axis] = (uint8_t)AxisMode_Inactive;
        dm.phase_snapshot[axis] = (uint8_t)AxisPhase_Inactive;
    }

    dm.active_snapshot = false;

    dm.last_velocity_heartbeat_ms = 0;
    dm.ctrl_time_ms = 0;
    dm.ctrl_subtick = 0;

    if(!dm_hooks_installed) {
        dm_hooks_installed = true;

        // Default to the known core callback. This is robust if init runs before
        // the core has populated hal.stepper.interrupt_callback.
        stepper_irq_org = stepper_driver_interrupt_handler;

        if(dm_stepper_cb_valid(hal.stepper.interrupt_callback) && hal.stepper.interrupt_callback != dm_interrupt_callback)
            stepper_irq_org = hal.stepper.interrupt_callback;

        if(!dm_stepper_cb_valid(stepper_irq_org) || stepper_irq_org == dm_interrupt_callback)
            stepper_irq_org = stepper_driver_interrupt_handler;

        hal.stepper.interrupt_callback = dm_interrupt_callback;

        on_unknown_realtime_org = grbl.on_unknown_realtime_cmd;
        grbl.on_unknown_realtime_cmd = on_unknown_realtime;

        on_execute_realtime_org = grbl.on_execute_realtime;
        grbl.on_execute_realtime = on_execute_realtime;
    }
}

#endif // VELOCITY_JOG_ENABLE
