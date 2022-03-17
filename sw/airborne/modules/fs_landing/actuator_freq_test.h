//
// Created by matteo on 06/07/2020.
//

#ifndef PAPARAZZI_ACTUATOR_FREQ_TEST_H
#define PAPARAZZI_ACTUATOR_FREQ_TEST_H

#include "fs_landing.h"
#include <math.h>
#include "mcu_periph/sys_time.h"

extern float test_freq;
extern float test_min_value;
extern float test_max_value;
extern uint8_t actuator_idx;
extern uint8_t automatic_f;
extern uint8_t is_input_sin;
extern uint8_t cal_reverse;
extern float ramp_min_freq;
extern float ramp_max_freq;
extern float ramp_duration;

enum test_mode {CALIBRATE, DISCRETE, CONTINUOUS};
extern enum test_mode freq_test_mode;

void freq_test(struct fs_landing_t *actuator_values, float start_t);

#endif //PAPARAZZI_ACTUATOR_FREQ_TEST_H
