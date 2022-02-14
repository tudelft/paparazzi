//
// Created by matteo on 09/05/2020.
//

#include "fs_landing.h"
#include "feed_forward.h"
#include "cyclic_control.h"
#include "cyclic_controller.h"

#include "subsystems/datalink/downlink.h"

//#ifndef FS_LANDING_FREQ_TEST
//#define FS_LANDING_FREQ_TEST FALSE
//#endif

#ifndef FS_LANDING_DEBUG
#define FS_LANDING_DEBUG FALSE
#endif

struct fs_landing_t fs_landing;
struct fs_landing_t current_actuator_values;

uint8_t is_spinning = false;
//uint8_t pilot_has_control = false;
uint8_t cyclic_control_active = false;
//uint8_t impulse_control_active = false;
uint8_t has_ff_started = false;
//uint8_t freq_test_active = false;
float ff_start_time = 0;
float ft_start_time = 0;

uint8_t use_pre_spin = false;
float pre_spin_pitch_coeff = 0.05;
float pre_spin_speed_setpoint = 8;
float pre_spin_trim_percentage = 0.20;
float err_test = 5;

#if PERIODIC_TELEMETRY
float fs_landing_dbg_values[N_DBG_VALUES] = {0};

#include "subsystems/datalink/telemetry.h"
static void send_payload_float(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_PAYLOAD_FLOAT(trans, dev, AC_ID, (uint8_t)N_DBG_VALUES, fs_landing_dbg_values);
}
#endif

void fs_landing_init()
{
  uint8_t i;
  for (i = 0; i < ACTUATORS_NB; i++) {
    fs_landing.commands[i] = 0;
    current_actuator_values.commands[i] = 0;
  }
#if PERIODIC_TELEMETRY
  #include "subsystems/datalink/telemetry.h"
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_PAYLOAD_FLOAT, send_payload_float);
#endif
  cyclic_controller_init();
}

// TODO Make sure all files agree on direction of spin (e.g. assume anti-clockwise rotation)
void fs_landing_run()
{
  if (is_fs_landing_active()) {
//#if FS_LANDING_FREQ_TEST
//    if (freq_test_active) {
//    freq_test(&current_actuator_values, ft_start_time);
//    } else {
//      ft_start_time = get_sys_time_float();
//      freq_test_active = true;
//    }
//    return;
//#endif
    if (is_spinning) {
      spin_actuator_values(&current_actuator_values);  // Constant actuator values to maintain spin
//            add_chirp(&current_actuator_values);
//      if (pilot_has_control) {
//        pilot_actuator_values(&current_actuator_values);  // RC Channels control actuator deflection
//      } else if (cyclic_control_active) {
      if (cyclic_control_active) {
        cyclic_control_values(&current_actuator_values);
        cyclic_controller_run();
      }  // TODO Add controller here
//      if (impulse_control_active) {
//        impulse_control_values(&current_actuator_values);
//      }
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
//    freq_test_active = false;
  }
  return;
}

bool pre_spin_actuator_values()
{
  float err = pre_spin_speed_setpoint - stateGetHorizontalSpeedNorm_f();
#if FS_LANDING_DEBUG
  dbg_msg[0] = stateGetHorizontalSpeedNorm_f();
  dbg_msg[1] = err;
  DOWNLINK_SEND_PAYLOAD_FLOAT(DefaultChannel, DefaultDevice, 4, dbg_msg);
#endif
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

// Compute matching paparazzi actuator signal value for motor left
// given motor right value so that the rotation axis is at the center
int32_t get_matching_motl_val(int32_t val) {
  float a_rev = 0.1765237;
  float b_rev = 0.1014789;
  float c_rev = -0.0280731;

  float a_fwd = 0.2359004;
  float b_fwd = 0.2339600;
  float c_fwd = -0.0156445;

  // map pprz val to [0, 1]
  float k = val / 9600.;
  // approximate fwd thrust
  float thr = a_fwd * pow(k, 2) + b_fwd * k + c_fwd;
  // calculate [0, 1] value of reverse motor from thrust
  float m = (-b_rev + sqrtf(pow(b_rev,2 )- 4. * a_rev * (c_rev - thr))) / (2. * a_rev);
  // remap to pprz value
  float p = (int32_t) (m * -9600);
  return p;
}

//void fs_landing_pilot_control_handler(uint8_t active)
//{
//  pilot_has_control = active;
//}

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
