//
// Created by matteo on 15/09/2020.
//

#include "cyclic_control.h"

float old_yaw_roll;
float old_yaw_pitch;

float old_roll;
float old_pitch;

float new_roll;
float new_pitch;
float new_yaw;

bool max_roll_found;
bool max_pitch_found;
bool roll_increasing;
bool pitch_increasing;

float max_roll;
float max_pitch;
float yaw_roll;
float yaw_pitch;

uint8_t both_actuators = false;
float kr = 0.5;
float kp = 0.5;
float phase_lead = 90;

void cyclic_control_values(struct fs_landing_t *actuator_values) {
    new_yaw = stateGetNedToBodyEulers_f() -> psi;
    get_max_roll_yaw();
    get_max_pitch_yaw();

    int sign_roll = roll_increasing ? -1 : 1;
    int sign_pitch = pitch_increasing ? 1 : -1;

    float pl_rad = RadOfDeg(phase_lead);

    // Not sure this is the clearest way to do the sign
    int32_t roll_correction = (int32_t) (sign_roll * kr * 9600 * cosf(new_yaw - (pl_rad + yaw_roll)));
    int32_t pitch_correction = (int32_t) (sign_pitch * kp * 9600 * cosf(new_yaw - (pl_rad + yaw_pitch)));

    actuator_values -> commands[SERVO_S_ELEVON_RIGHT] += roll_correction + pitch_correction;
    if (both_actuators) {
        actuator_values -> commands[SERVO_S_ELEVON_LEFT] += -roll_correction + pitch_correction;
    }
}

void get_max_roll_yaw() {
    bool check_sign = false;
    new_roll = stateGetNedToBodyEulers_f() -> phi;
    if (old_yaw_roll < 0 && new_yaw > 0) {  // Only valid for CCW rotation
        max_roll = new_roll;
        max_roll_found = false;
        check_sign = true;
    }
    if (check_sign) {
        if (new_roll > old_roll) {
            roll_increasing = true;
        } else {
            roll_increasing = false;
        }
    }
    // New revolution
    if (!max_roll_found){
        if (roll_increasing) {
            if (new_roll > max_roll) {
                max_roll = new_roll;
            } else {
                yaw_roll = old_yaw_roll;
                max_roll_found = true;
            }
        } else {
            if (new_roll < max_roll) {
                max_roll = new_roll;
            } else {
                yaw_roll = old_yaw_roll;
                max_roll_found = true;
            }
        }
    }
    // Continue check
    old_yaw_roll = new_yaw;
    old_roll = new_roll;
}

void get_max_pitch_yaw() {
    bool check_sign = false;
    new_pitch = stateGetNedToBodyEulers_f() -> theta;
    if (old_yaw_pitch < 0 && new_yaw > 0) {
        max_pitch = new_pitch;
        max_pitch_found = false;
        check_sign = true;
    }
    if (check_sign) {
        if (new_pitch > old_pitch) {
            pitch_increasing = true;
        } else {
            pitch_increasing = false;
        }
    }
    // New revolution
    if (!max_pitch_found){
        if (pitch_increasing) {
            if (new_pitch > max_pitch) {
                max_pitch = new_pitch;
            } else {
                yaw_pitch = old_yaw_roll;
                max_pitch_found = true;
            }
        } else {
            if (new_pitch < max_pitch) {
                max_pitch = new_pitch;
            } else {
                yaw_pitch = old_yaw_roll;
                max_pitch_found = true;
            }
        }
    }
    // Continue check
    old_yaw_pitch = new_yaw;
    old_pitch = new_pitch;
}
