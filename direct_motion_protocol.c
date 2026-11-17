#include "direct_motion.h"

#include <string.h>

bool direct_motion_decode_velocity(uint8_t axes, const uint8_t *payload, size_t size,
                                   direct_motion_velocity_frame_t *frame)
{
    if(!axes || axes > DIRECT_MOTION_MAX_AXES || !payload || !frame ||
       (size != 13U && size != 1U + 4U * axes))
        return false;
    uint8_t values = (uint8_t)((size - 1U) / 4U);
    uint8_t allowed = (uint8_t)((1U << (values < axes ? values : axes)) - 1U);
    if(payload[0] & ~allowed)
        return false;
    memset(frame, 0, sizeof(*frame));
    frame->mask = payload[0];
    for(uint8_t axis = 0; axis < values; axis++) {
        const uint8_t *bytes = payload + 1U + 4U * axis;
        uint32_t value = (uint32_t)bytes[0] | ((uint32_t)bytes[1] << 8) |
            ((uint32_t)bytes[2] << 16) | ((uint32_t)bytes[3] << 24);
        frame->velocity_q16[axis] = value <= INT32_MAX ? (int32_t)value : -1 - (int32_t)(UINT32_MAX - value);
    }
    return true;
}