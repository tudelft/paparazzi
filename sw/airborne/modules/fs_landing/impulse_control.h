//
// Created by matteo on 29/04/2021.
//

#ifndef PAPARAZZI_IMPULSE_CONTROL_H
#define PAPARAZZI_IMPULSE_CONTROL_H

#include "fs_landing.h"

extern uint8_t impulse_use_motor_l;
extern uint8_t impulse_use_motor_r;
extern uint8_t impulse_use_elevon_l;
extern uint8_t impulse_use_elevon_r;

extern float ml_value;
extern float mr_value;
extern float el_value;
extern float er_value;

extern uint16_t elevon_activate_at;
extern uint16_t motor_activate_at;

extern float elevon_half_arc;
extern float motor_half_arc;

void impulse_control_values(struct fs_landing_t *actuator_values);

#endif //PAPARAZZI_IMPULSE_CONTROL_H
