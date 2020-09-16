//
// Created by matteo on 15/09/2020.
//

#ifndef PAPARAZZI_CYCLIC_CONTROL_H
#define PAPARAZZI_CYCLIC_CONTROL_H

#include "state.h"
#include "fs_landing.h"

extern uint8_t both_actuators;
extern float kr;
extern float kp;
extern float phase_lead;

void cyclic_control_values(struct fs_landing_t *actuator_values);
void get_max_roll_yaw(void);
void get_max_pitch_yaw(void);

#endif //PAPARAZZI_CYCLIC_CONTROL_H
