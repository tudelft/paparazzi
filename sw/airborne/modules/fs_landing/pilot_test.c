//
// Created by matteo on 27/05/2020.
//

#include "pilot_test.h"

void pilot_actuator_values(struct fs_landing_t *actuator_values) {
    // THROTTLE channel controls motor yaw
    // @THROTTLE 0 --> motors idle
    // @THROTTLE max --> Full thrust right, full reversal left
    float throttle = radio_control.values[RADIO_THROTTLE];

    // ROLL channel controls elevon deflection, left right opposite
    float elevon_alt = radio_control.values[RADIO_ROLL];

    // PITCH channel controls elevon deflection, left right together
    float elevon_tog = radio_control.values[RADIO_PITCH];

    actuator_values->commands[SERVO_S_THROTTLE_LEFT] = throttle;
    actuator_values->commands[SERVO_S_THROTTLE_RIGHT] = -throttle;
    actuator_values->commands[SERVO_S_ELEVON_LEFT] = elevon_tog + elevon_alt;
    actuator_values->commands[SERVO_S_ELEVON_RIGHT] = elevon_tog - elevon_alt;
}
