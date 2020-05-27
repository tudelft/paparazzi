//
// Created by matteo on 26/05/2020.
//

#include "feed_forward.h"

float ff_duration = 3.;
float thr_l_value = 0;
float thr_r_value = 0;
float elev_l_value = 0;
float elev_r_value = 0;

void ff_actuator_values(struct fs_landing_t *actuator_values, float ff_start_time, uint8_t *has_ff_finished)
{
    if (get_sys_time_float() - ff_start_time < ff_duration)
    {
        actuator_values->commands[SERVO_S_THROTTLE_LEFT] = thr_l_value;
        actuator_values->commands[SERVO_S_THROTTLE_RIGHT] = thr_r_value;
        actuator_values->commands[SERVO_S_ELEVON_LEFT] = elev_l_value;
        actuator_values->commands[SERVO_S_ELEVON_RIGHT] = elev_r_value;
    } else {
        has_ff_finished = true;
    }
}

void feed_forward_ff_duration_handler(float dur)
{
    ff_duration = dur;
}
void feed_forward_thr_l_handler(float val)
{
    thr_l_value = val;
}
void feed_forward_thr_r_handler(float val)
{
    thr_r_value = val;
}
void feed_forward_elev_l_handler(float val)
{
    elev_l_value = val;
}
void feed_forward_elev_r_handler(float val)
{
    elev_r_value = val;
}

