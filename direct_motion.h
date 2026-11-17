#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#define DIRECT_MOTION_MAX_AXES 8U

typedef struct {
	uint8_t mask;
	int32_t velocity_q16[DIRECT_MOTION_MAX_AXES];
} direct_motion_velocity_frame_t;

bool direct_motion_decode_velocity(uint8_t axes, const uint8_t *payload, size_t size,
								   direct_motion_velocity_frame_t *frame);

typedef struct {
	uint8_t mode;
	float rate_per_minute;
} direct_motion_remote_state_t;

typedef struct {
	uint8_t axes_mask;
	bool (*move)(uint8_t axis, bool relative, int32_t target_q16, int32_t velocity_q16, int32_t acceleration_q16, uint16_t sequence);
	bool (*velocity)(uint8_t axis, int32_t velocity_q16, uint16_t sequence);
	bool (*stop)(uint8_t axes_mask, bool immediate);
	bool (*state)(uint8_t axis, direct_motion_remote_state_t *state);
} direct_motion_remote_t;

bool direct_motion_bind_remote(const direct_motion_remote_t *backend);
void direct_motion_report_remote_error(uint16_t sequence, uint8_t type, uint8_t axis);
uint8_t direct_motion_remote_axes(void);

// Initialize velocity jog plugin hooks.
void direct_motion_init (void);

// Feed a raw Telnet byte to the velocity-jog frame parser.
// Returns true when the byte is consumed by velocity-jog transport.
bool direct_motion_telnet_rx_byte (uint8_t byte);

#if REPORT_REALTIME_AXIS_VELOCITY
// Returns true and fills `rates` with per-axis commanded rates when direct-motion
// is actively driving motion. Units are machine units/minute for each axis.
bool direct_motion_get_realtime_axis_rates (float *rates);
#endif
