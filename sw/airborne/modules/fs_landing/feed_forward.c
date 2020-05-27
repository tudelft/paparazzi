//
// Created by matteo on 26/05/2020.
//

#include "feed_forward.h"

void ff_actuator_values(struct fs_landing_t *actuator_values, float ff_start_time, uint8_t *has_ff_finished)
{
    if (get_sys_time_float() - ff_start_time < FF_DURATION)
    {

    } else {
        has_ff_finished = true;
    }
}
