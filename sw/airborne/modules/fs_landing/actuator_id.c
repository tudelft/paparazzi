//
// Created by matteo on 15/06/2020.
//

#include "actuator_id.h"

uint8_t selected_actuator = 1;
float max_delta_amplitude = 0.f;

void add_chirp(struct fs_landing_t *actuator_values) {
    float current_yaw = stateGetNedToBodyEulers_f()->psi;
    actuator_values->commands[selected_actuator] += (int32_t) (max_delta_amplitude * sinf(current_yaw));
}
