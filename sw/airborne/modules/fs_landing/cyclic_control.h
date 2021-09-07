//
// Created by matteo on 15/09/2020.
//

#ifndef PAPARAZZI_CYCLIC_CONTROL_H
#define PAPARAZZI_CYCLIC_CONTROL_H

#include "fs_landing.h"

extern uint8_t both_actuators;
extern float kr;
extern float kp;
extern float phase_lead;

extern uint8_t use_motor_l;
extern uint8_t use_motor_r;
extern uint8_t use_elevon_l;
extern uint8_t use_elevon_r;

extern float ml_avg;  // [-1, 1] multiplier for cosine average
extern float mr_avg;
extern float el_avg;
extern float er_avg;

extern float ml_delta;  // [0, 1] multiplier for cosine half amplitude
extern float mr_delta;
extern float el_delta;
extern float er_delta;

extern float mt_phase;  // phase offset in degrees
extern float el_phase;

extern uint8_t use_square_sig;
extern uint8_t phase_pilot_control;
extern uint8_t balance_motor_forces;
extern uint8_t use_controller;

void cyclic_control_values(struct fs_landing_t *actuator_values);
void get_max_roll_yaw(void);
void get_max_pitch_yaw(void);

extern void cyclic_control_use_controller_handler(uint8_t value);

#endif //PAPARAZZI_CYCLIC_CONTROL_H
