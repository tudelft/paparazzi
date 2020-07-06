//
// Created by matteo on 06/07/2020.
//

#ifndef PAPARAZZI_ACTUATOR_FREQ_TEST_H
#define PAPARAZZI_ACTUATOR_FREQ_TEST_H

#include "fs_landing.h"
#include <math.h>
#include "mcu_periph/sys_time.h"

extern float test_freq;

void freq_test(struct fs_landing_t *actuator_values, float start_t);

#endif //PAPARAZZI_ACTUATOR_FREQ_TEST_H
