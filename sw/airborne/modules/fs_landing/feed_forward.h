//
// Created by matteo on 26/05/2020.
//

#ifndef PAPARAZZI_FEED_FORWARD_H
#define PAPARAZZI_FEED_FORWARD_H

#include "mcu_periph/sys_time.h"
#include "fs_landing.h"

extern float ff_duration;
extern float thr_l_value;
extern float thr_r_value;
extern float elev_l_value;
extern float elev_r_value;

void ff_actuator_values(struct fs_landing_t *actuator_values, float ff_start_time, uint8_t *has_ff_finished);

// Handlers for changing gcs variables
extern void feed_forward_ff_duration_handler(float dur);
extern void feed_forward_thr_l_handler(float val);
extern void feed_forward_thr_r_handler(float val);
extern void feed_forward_elev_l_handler(float val);
extern void feed_forward_elev_r_handler(float val);

#endif //PAPARAZZI_FEED_FORWARD_H
