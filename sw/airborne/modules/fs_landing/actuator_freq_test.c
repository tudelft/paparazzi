//
// Created by matteo on 06/07/2020.
//

#include "actuator_freq_test.h"

#include "subsystems/datalink/downlink.h"

float test_freq = 0.;
float test_current_value = 0;

// Debug
float send_values[4];

void freq_test(struct fs_landing_t *actuator_values, float start_t) {
    actuator_values -> commands[SERVO_S_THROTTLE_LEFT] = 0;
    actuator_values -> commands[SERVO_S_THROTTLE_RIGHT] = 0;
    actuator_values -> commands[SERVO_S_ELEVON_RIGHT] = 0;

    float t = get_sys_time_float() - start_t;
    float test_sin = sinf(t * 2 * M_PI * test_freq);

    if (test_sin < 0) {
        test_current_value = -10000;
    } else {
        test_current_value = 10000;
    }
    actuator_values -> commands[SERVO_S_ELEVON_LEFT] = (int32_t) test_current_value;

    send_values[0] = test_freq;
    send_values[1] = test_current_value;
    send_values[2] = (int32_t) test_current_value;
    DOWNLINK_SEND_PAYLOAD_FLOAT(DefaultChannel, DefaultDevice, 8, send_values);
}
