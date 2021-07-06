//
// Created by matteo on 06/07/2020.
//

#include "actuator_freq_test.h"
#include "subsystems/datalink/downlink.h"

#define MAX_TEST_FREQ 15
#define TEST_STEP_DUR 3
#define FREQ_DELTA 0.25

#define CAL_STEP_DUR 2
#define CAL_STEPS 20

#define DEBUG FALSE

float test_freq = 0.;
float test_min_value = 1000;  // -9600 lowest val possible
float test_max_value = 10000 ;  // 9600 highest val possible
uint8_t actuator_idx = 1;  // Actuator being tested
uint8_t automatic_f = false;  // If true, freq will increase without user input
uint8_t is_input_sin = false;  // If true, chirp is sine wave instead of square
uint8_t calibrate = false;  // Calibration procedure (activate before turning on module)
uint8_t cal_reverse = false;  // To test reverse thrust (calibration)

#if DEBUG
float send_values[4];
#endif

// Spin the motors in steps to match % power to frequency with FFT
void motor_freq_calibration(struct fs_landing_t *actuator_values, float time_from_start);

void motor_freq_calibration(struct fs_landing_t *actuator_values, float time_from_start)
{
  // This test is only for the motors
  if (actuator_idx == 1 || actuator_idx == 2) {
    return;
  }
  float step_sign;
  step_sign =  cal_reverse ? -1 : 1;
  float calibration_steps = CAL_STEPS;
  float cs_time = CAL_STEP_DUR;
  float step_value_calib = step_sign * (9600 / calibration_steps);

  uint8_t cal_phase = (uint8_t) (time_from_start / cs_time);
  float test_current_value = cal_phase * step_value_calib;
  if (time_from_start > cs_time * (calibration_steps + 1)) { // Test over
    test_current_value = 0;
  }
  actuator_values->commands[actuator_idx] = (int32_t) test_current_value;
#if DEBUG
  send_values[1] = cal_phase;
  send_values[2] = test_current_value;
#endif
}

void freq_test(struct fs_landing_t *actuator_values, float start_t)
{
  actuator_values->commands[SERVO_S_ELEVON_LEFT] = 0;
  actuator_values->commands[SERVO_S_THROTTLE_LEFT] = 0;
  actuator_values->commands[SERVO_S_THROTTLE_RIGHT] = 0;
  actuator_values->commands[SERVO_S_ELEVON_RIGHT] = 0;

  double t = get_sys_time_float() - start_t;

  if (calibrate) {
    motor_freq_calibration(actuator_values, t);
  } else {
    uint8_t test_step = 0;
    if (automatic_f) {
      float freq_delta = FREQ_DELTA;
      float test_step_duration = TEST_STEP_DUR;
      test_step = (uint8_t)(t / test_step_duration);
      test_freq = test_step * freq_delta;
      if (test_freq > MAX_TEST_FREQ) {  // Test over
        actuator_values->commands[actuator_idx] = 0;
        return;
      }
    }
    float test_sin = sinf(t * 2 * M_PI * test_freq);

    float test_current_value = 0;
    if (is_input_sin) {
      float midpoint = (test_min_value + test_max_value) * 0.5;
      float half_amplitude = (test_min_value - test_max_value) * 0.5;
      test_current_value = midpoint + half_amplitude * test_sin;
    } else {
      if (test_sin < 0) {
        test_current_value = test_min_value;
      } else {
        test_current_value = test_max_value;
      }
    }
    actuator_values->commands[actuator_idx] = (int32_t) test_current_value;
#if DEBUG
    send_values[1] = test_step;
    send_values[2] = test_current_value;
#endif
  }
#if DEBUG
  send_values[0] = test_freq;
  DOWNLINK_SEND_PAYLOAD_FLOAT(DefaultChannel, DefaultDevice, 8, send_values);
#endif
}
