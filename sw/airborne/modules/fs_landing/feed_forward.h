//
// Created by matteo on 26/05/2020.
//

#ifndef PAPARAZZI_FEED_FORWARD_H
#define PAPARAZZI_FEED_FORWARD_H

#include "mcu_periph/sys_time.h"
#include "fs_landing.h"

#define FF_DURATION 5

void ff_actuator_values(struct fs_landing_t *actuator_values, float ff_start_time, uint8_t *has_ff_finished);

#endif //PAPARAZZI_FEED_FORWARD_H
