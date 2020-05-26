//
// Created by matteo on 09/05/2020.
//
#include "subsystems/radio_control.h"

#include "fs_landing.h"

void fs_landing_init()
{
    uint8_t i;
    for (i = 0; i < NB_ACT_FS; i++) {
        fs_landing.commands[i] = 0;
    }
}

void fs_landing_run()
{
    // TODO set current_actuator_values here
    return;
}

void is_fs_landing_active()
{
    bool is_active;
    if (radio_control.values[RADIO_AUX2] < 4500) {
        is_active = false;
    } else {
        is_active = true;
    }
    return is_active;
}

void fs_landing_set_actuator_values()
{
    if (is_fs_landing_active()) {
        fs_landing.commands[S_THROTTLE_LEFT] = current_actuator_values[S_THROTTLE_LEFT];
        fs_landing.commands[S_THROTTLE_RIGHT] = current_actuator_values[S_THROTTLE_RIGHT];
        fs_landing.commands[S_ELEVON_LEFT] = current_actuator_values[S_ELEVON_LEFT];
        fs_landing.commands[S_ELEVON_RIGHT] = current_actuator_values[S_ELEVON_RIGHT];

        uint8_t i;
        for (i = 0; i < NB_ACT_FS; i++) {
            BoundAbs(fs_landing.commands[i], MAX_PPRZ);
        }
    }
}
