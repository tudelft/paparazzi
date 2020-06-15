//
// Created by matteo on 15/06/2020.
//

#ifndef PAPARAZZI_ACTUATOR_ID_H
#define PAPARAZZI_ACTUATOR_ID_H

#include "fs_landing.h"
#include "state.h"
#include <math.h>

extern uint8_t selected_actuator;
extern float max_delta_amplitude;

void add_chirp(struct fs_landing_t *actuator_values);

#endif //PAPARAZZI_ACTUATOR_ID_H
