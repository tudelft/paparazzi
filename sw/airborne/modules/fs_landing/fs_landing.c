//
// Created by matteo on 09/05/2020.
//

#include "fs_landing.h"
#include "feed_forward.h"
#include "pilot_test.h"
#include "actuator_id.h"
#include "cyclic_control.h"

#include "subsystems/datalink/downlink.h"

struct fs_landing_t fs_landing;
struct fs_landing_t current_actuator_values;

uint8_t is_spinning = false;
uint8_t pilot_has_control = false;
uint8_t act_identification_active = false;
uint8_t cyclic_control_active = false;
uint8_t has_ff_started = false;
float ff_start_time = 0;

uint8_t use_pre_spin = true;
float pre_spin_pitch_coeff = 0.05;
float pre_spin_speed_setpoint = 8;
float pre_spin_trim_percentage = 0.20;
float err_test = 5;

// For debug use:
// float send_values[4];
// send_values[0] = (float)current_actuator_values.commands[1];
// DOWNLINK_SEND_PAYLOAD_FLOAT(DefaultChannel, DefaultDevice, 8, send_values);

void fs_landing_init()
{
    uint8_t i;
    for (i = 0; i < ACTUATORS_NB; i++) {
        fs_landing.commands[i] = 0;
        current_actuator_values.commands[i] = 0;
    }
}

// TODO Make sure all files agree on direction of spin (e.g. assume anti-clockwise rotation)
void fs_landing_run()
{
    if (is_fs_landing_active()) {
        if (is_spinning) {
            spin_actuator_values(&current_actuator_values);  // Constant actuator values to maintain spin
//            add_chirp(&current_actuator_values);
            if (pilot_has_control) {
                pilot_actuator_values(&current_actuator_values);  // RC Channels control actuator deflection
            } else if (act_identification_active){
                add_chirp(&current_actuator_values);  // +- sinusoidally varying delta to one of the actuators
            } else if (cyclic_control_active) {
                cyclic_control_values(&current_actuator_values);
            }
        } else {
            if (use_pre_spin) {
                is_spinning = pre_spin_actuator_values();
            } else {
                if (has_ff_started) {
                    is_spinning = ff_actuator_values(&current_actuator_values, ff_start_time);
                } else {
                    ff_start_time = get_sys_time_float();
                    has_ff_started = true;
                }
            }
        }
    } else {
        is_spinning = false;
        has_ff_started = false;
        act_identification_active = false;
    }
    return;
}

bool pre_spin_actuator_values() {
    float err = pre_spin_speed_setpoint - stateGetHorizontalSpeedNorm_f();
//    err = err_test;
    if (err > 0) {
        // Assuming max value is upward elevon deflection
        float elevon_l_range = 9600;
        float elevon_r_range = 9600;
        float elevon_l_trim = pre_spin_trim_percentage * elevon_l_range;
        float elevon_r_trim = pre_spin_trim_percentage * elevon_r_range;

        current_actuator_values.commands[SERVO_S_THROTTLE_LEFT] = 0;
        current_actuator_values.commands[SERVO_S_THROTTLE_RIGHT] = 0;
        current_actuator_values.commands[SERVO_S_ELEVON_LEFT] = elevon_l_trim + pre_spin_pitch_coeff * err * elevon_l_range;
        current_actuator_values.commands[SERVO_S_ELEVON_RIGHT] = elevon_r_trim + pre_spin_pitch_coeff * err * elevon_r_range;
        return false;
    } else {
        return true;
    }
}

bool is_fs_landing_active()
{
    bool is_active;
    // Map to command so if a different switch is used it will still work
    // if (radio_control.values[FS_LANDING] < 4500) { TODO Check
    if (radio_control.values[RADIO_AUX2] < 4500) {
        is_active = false;
    } else {
        is_active = true;
    }
    return is_active;
}

void fs_landing_pilot_control_handler(uint8_t active)
{
    pilot_has_control = active;
}

void fs_landing_actuator_id_handler(uint8_t active)
{
    if (is_spinning){
        act_identification_active = active;
    } else {
        act_identification_active = false;
    }
}

void fs_landing_set_actuator_values()
{
    if (is_fs_landing_active()) {
        fs_landing.commands[SERVO_S_THROTTLE_LEFT] = current_actuator_values.commands[SERVO_S_THROTTLE_LEFT];
        fs_landing.commands[SERVO_S_THROTTLE_RIGHT] = current_actuator_values.commands[SERVO_S_THROTTLE_RIGHT];
        fs_landing.commands[SERVO_S_ELEVON_LEFT] = current_actuator_values.commands[SERVO_S_ELEVON_LEFT];
        fs_landing.commands[SERVO_S_ELEVON_RIGHT] = current_actuator_values.commands[SERVO_S_ELEVON_RIGHT];

        uint8_t i;
        for (i = 0; i < ACTUATORS_NB; i++) {
            BoundAbs(fs_landing.commands[i], MAX_PPRZ);
        }
    }
}
