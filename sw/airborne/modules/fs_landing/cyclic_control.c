//
// Created by matteo on 15/09/2020.
//

#include <math.h>
#include "subsystems/radio_control.h"

#include "state.h"
#include "cyclic_control.h"

#include "subsystems/datalink/downlink.h"

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

uint8_t use_motor_l = false;
uint8_t use_motor_r = false;
uint8_t use_elevon_l = false;
uint8_t use_elevon_r = false;

float ml_avg = 0.;  // [-1, 1] multiplier for cosine average
float mr_avg = 0.;
float el_avg = 0.;
float er_avg = 0.;

float ml_delta = 0.;  // [0, 1] multiplier for cosine half amplitude
float mr_delta = 0.;
float el_delta = 0.;
float er_delta = 0.;

float mt_phase = 0.;  // phase offset in degrees
float el_phase = 0.;

uint8_t phase_pilot_control = false;

float dbg_msg[4];

void cyclic_control_values(struct fs_landing_t *actuator_values) {
  float current_yaw = stateGetNedToBodyEulers_f() -> psi;

  float mt_phase_rad = RadOfDeg(mt_phase);
  float el_phase_rad = RadOfDeg(el_phase);
  if (phase_pilot_control) {
    int64_t radio_roll = radio_control.values[RADIO_ROLL];
    if (abs(radio_roll) < 100) {
      radio_roll = 0;
    }
    int64_t radio_pitch = radio_control.values[RADIO_PITCH];
    if (abs(radio_pitch) < 100) {
      radio_pitch = 0;
    }
    float pilot_phase_rad = atan2f(radio_roll, radio_pitch);

    dbg_msg[0] = radio_roll;
    dbg_msg[1] = radio_pitch;
    dbg_msg[2] = DegOfRad(pilot_phase_rad);
    DOWNLINK_SEND_PAYLOAD_FLOAT(DefaultChannel, DefaultDevice, 4, dbg_msg);

    mt_phase_rad = pilot_phase_rad;
    el_phase_rad = pilot_phase_rad;
  }

  int32_t elevon_l = (int32_t) (el_avg * 9600 + el_delta * 9600 * cosf(current_yaw + el_phase_rad));
  int32_t elevon_r = (int32_t) (er_avg * 9600 + er_delta * 9600 * cosf(current_yaw + el_phase_rad));
  int32_t motor_l = (int32_t) (ml_avg * 9600 + ml_delta * 9600 * cosf(current_yaw + mt_phase_rad));
  int32_t motor_r = (int32_t) (mr_avg * 9600 + mr_delta * 9600 * cosf(current_yaw + mt_phase_rad));

  if (use_elevon_l) {
    actuator_values -> commands[SERVO_S_ELEVON_LEFT] = elevon_l;
  }
  if (use_elevon_r) {
    actuator_values -> commands[SERVO_S_ELEVON_RIGHT] = elevon_r;
  }
  if (use_motor_l) {
    actuator_values -> commands[SERVO_S_THROTTLE_LEFT] = motor_l;
  }
  if (use_motor_r) {
    actuator_values -> commands[SERVO_S_THROTTLE_RIGHT] = motor_r;
  }
}

//void cyclic_control_values(struct fs_landing_t *actuator_values) {
//    new_yaw = stateGetNedToBodyEulers_f() -> psi;
//    get_max_roll_yaw();
//    get_max_pitch_yaw();
//
//    int sign_roll = roll_increasing ? -1 : 1;
//    int sign_pitch = pitch_increasing ? 1 : -1;
//
//    float pl_rad = RadOfDeg(phase_lead);
//
//    // Not sure this is the clearest way to do the sign
//    int32_t roll_correction = (int32_t) (sign_roll * kr * 9600 * cosf(new_yaw - (pl_rad + yaw_roll)));
//    int32_t pitch_correction = (int32_t) (sign_pitch * kp * 9600 * cosf(new_yaw - (pl_rad + yaw_pitch)));
//
//    actuator_values -> commands[SERVO_S_ELEVON_RIGHT] += roll_correction + pitch_correction;
//    if (both_actuators) {
//        actuator_values -> commands[SERVO_S_ELEVON_LEFT] += -roll_correction + pitch_correction;
//    }
//}

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
