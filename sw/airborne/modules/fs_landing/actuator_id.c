//
// Created by matteo on 15/06/2020.
//

#include "actuator_id.h"

uint8_t selected_actuator = 1;
float max_delta_amplitude = 0.f;

void add_chirp(struct fs_landing_t *actuator_values) {
    float current_yaw = stateGetNedToBodyEulers_f() -> psi;
    if (selected_actuator == 4) {
        // Assuming CCW spin
        actuator_values -> commands[SERVO_S_THROTTLE_LEFT] -= (int32_t) (max_delta_amplitude * sinf(current_yaw));
        actuator_values -> commands[SERVO_S_THROTTLE_RIGHT] += (int32_t) (max_delta_amplitude * sinf(current_yaw));
    } else if (selected_actuator == 5) {
        actuator_values -> commands[SERVO_S_THROTTLE_LEFT] += (int32_t) (max_delta_amplitude * sinf(current_yaw));
        actuator_values -> commands[SERVO_S_THROTTLE_RIGHT] += (int32_t) (max_delta_amplitude * sinf(current_yaw));
    } else {
        actuator_values -> commands[selected_actuator] += (int32_t)(max_delta_amplitude * sinf(current_yaw));
    }
}
