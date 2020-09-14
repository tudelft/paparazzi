//
// Created by matteo on 26/05/2020.
//

#include "feed_forward.h"
#include <stdio.h>

float ff_duration = 3.;

// TODO Determine
float thr_l_ff = 0; //-7000;
float thr_r_ff = 0; //7000;
float elev_l_ff = -9600;
float elev_r_ff = 9600;

// TODO Determine
float thr_l_spinning = 0; //-7000;
float thr_r_spinning = 0; //7000;
float elev_l_spinning = -9600;
float elev_r_spinning = 9600;

uint8_t ff_actuator_values(struct fs_landing_t *actuator_values, float ff_start_time)
{
    if (get_sys_time_float() - ff_start_time < ff_duration) {
        // Values to initiate spin
        actuator_values->commands[SERVO_S_THROTTLE_LEFT] = thr_l_ff;
        actuator_values->commands[SERVO_S_THROTTLE_RIGHT] = thr_r_ff;
        actuator_values->commands[SERVO_S_ELEVON_LEFT] = elev_l_ff;
        actuator_values->commands[SERVO_S_ELEVON_RIGHT] = elev_r_ff;
        return false;
    } else {
        return true;
    }
}

void spin_actuator_values(struct fs_landing_t *actuator_values)
{
    // These values should maintain the spin
    actuator_values->commands[SERVO_S_THROTTLE_LEFT] = thr_l_spinning;
    actuator_values->commands[SERVO_S_THROTTLE_RIGHT] = thr_r_spinning;
    actuator_values->commands[SERVO_S_ELEVON_LEFT] = elev_l_spinning;
    actuator_values->commands[SERVO_S_ELEVON_RIGHT] = elev_r_spinning;
}

//void feed_forward_ff_duration_handler(float dur)
//{
//    ff_duration = dur;
//}
//void feed_forward_thr_l_ff_handler(float val)
//{
//    thr_l_ff = val;
//}
//void feed_forward_thr_r_ff_handler(float val)
//{
//    thr_r_ff = val;
//}
//void feed_forward_elev_l_ff_handler(float val)
//{
//    elev_l_ff = val;
//}
//void feed_forward_elev_r_ff_handler(float val)
//{
//    elev_r_ff = val;
//}
//void feed_forward_thr_l_spin_handler(float val)
//{
//    thr_l_spinning = val;
//}
//void feed_forward_thr_r_spin_handler(float val)
//{
//    thr_r_spinning = val;
//}
//void feed_forward_elev_l_spin_handler(float val)
//{
//    elev_l_spinning = val;
//}
//void feed_forward_elev_r_spin_handler(float val)
//{
//    elev_r_spinning = val;
//}

