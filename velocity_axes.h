/*

  velocity_axes.h - independent velocity controlled (reserved) axes

  Part of grblHAL

*/

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "../grbl/hal.h"

#define VELOCITY_AXES_STATUS_RESERVED ((status_code_t)200)

/*! \brief Motion provider for reserved axes.

set_target() and stop() may be called from interrupt context and must only latch values;
any I/O belongs in poll(), which is called from the foreground.
*/
typedef struct {
    axes_signals_t (*axes)(void);                                       //!< Axes this backend can currently drive.
    void (*claim)(uint8_t axis, bool claim);                            //!< Take axis from/return axis to normal motion control. Foreground.
    void (*set_target)(uint8_t axis, float units_per_min, float accel); //!< Signed speed, accel in units/s^2 (0 = axis setting).
    void (*stop)(axes_signals_t axes, bool immediate);
    float (*get_rate)(uint8_t axis);                                    //!< Actual signed speed in units/min.
    void (*poll)(void);                                                 //!< Optional.
} velocity_axes_backend_t;

bool velocity_axes_register_backend (const velocity_axes_backend_t *backend);
axes_signals_t velocity_axes_reserved (void);
void velocity_axes_get_rates (float *rates);
