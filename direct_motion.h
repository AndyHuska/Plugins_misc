#pragma once

#include <stdbool.h>
#include <stdint.h>

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
