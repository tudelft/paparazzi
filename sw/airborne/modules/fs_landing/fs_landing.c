//
// Created by matteo on 09/05/2020.
//

#include "fs_landing.h"
#include "feed_forward.h"
#include "pilot_test.h"
#include "actuator_id.h"

struct fs_landing_t fs_landing;
struct fs_landing_t current_actuator_values;

uint8_t is_spinning = false;
uint8_t pilot_has_control = false;
uint8_t act_identification_active = false;
uint8_t has_ff_started = false;
float ff_start_time = 0;

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
            if (pilot_has_control) {
                pilot_actuator_values(&current_actuator_values);  // RC Channels control actuator deflection
            } else if (act_identification_active){
                add_chirp(&current_actuator_values);  // +- sinusoidally varying delta to one of the actuators
            }
        } else {
            if (has_ff_started) {
                is_spinning = ff_actuator_values(&current_actuator_values, ff_start_time);
            } else {
                ff_start_time = get_sys_time_float();
                has_ff_started = true;
            }
        }
    } else {
        is_spinning = false;
        has_ff_started = false;
    }
    return;
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
