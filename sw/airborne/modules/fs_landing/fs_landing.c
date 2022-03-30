//
// Created by matteo on 09/05/2020.
//

#include "fs_landing.h"
#include "feed_forward.h"
#include "cyclic_control.h"
#include "cyclic_controller.h"
#include "actuator_freq_test.h"

#include "math/pprz_algebra_float.h"
#include "math/pprz_algebra_int.h"
#include "subsystems/abi.h"
#include "subsystems/datalink/downlink.h"

#include "generated/airframe.h"

#define MAX_PPRZ 9600

#ifndef FS_LANDING_FREQ_TEST
#define FS_LANDING_FREQ_TEST FALSE
#endif
uint8_t freq_test_active = false;

struct fs_landing_t fs_landing;
struct fs_landing_t current_actuator_values;

uint8_t is_spinning = false;
//uint8_t pilot_has_control = false;
uint8_t cyclic_control_active = false;
//uint8_t impulse_control_active = false;
uint8_t has_ff_started = false;

float ff_start_time = 0;
float ft_start_time = 0;

uint8_t use_pre_spin = false;
float pre_spin_pitch_coeff = 0.05;
float pre_spin_speed_setpoint = 8;
float pre_spin_trim_percentage = 0.20;
float err_test = 5;

// Horizontal velocity filter
bool is_horizontal_velocity_filter_initialized = false;
float prev_v_filt;
float v_filt, ins_v;

#define FS_V_FILT_ALPHA 0.0012
#define FS_V_FILT_BETA 0.07
#define FS_V_FILT_GAMMA 0.7
#define SEASONAL_L 120  // 2 * pi * module_frequency / (w_z average)
#define CT_DEFAULT 0.01
float seasonal_arr[SEASONAL_L] = {CT_DEFAULT};
int ctr = 0;
float bt = 0;
float filt_init_start_t = 0;
#define FILT_INIT_TIME 1.5

struct Int32Vect3 *_mag;
float my_psi = 0;

#ifndef MAG_RM3100_SENDER_ID
#define MAG_RM3100_SENDER_ID ABI_BROADCAST
#endif
static abi_event mag_rm3100_ev;
static void mag_rm3100_cb(uint8_t __attribute__((unused)) sender_id,
                          uint32_t __attribute__((unused)) stamp,
                          struct Int32Vect3 *mag)
{
  _mag = mag;
}

/*
 * <message name="FRISBEE_CONTROL" id="55">
 *   <field name="is_spinning" type="uint8">Is module currently active</field>
 *   <field name="v_filt" type="float" unit="m/s">Filtered horizontal velocity norm NED</field>
 *   <field name="ml_avg" type="float">Motor LEFT cyclic average</field>
 *   <field name="mr_avg" type="float">Motor RIGHT cyclic average</field>
 *   <field name="ml_delta" type="float">Motor LEFT cyclic half amplitude</field>
 *   <field name="mr_delta" type="float">Motor RIGHT cyclic half amplitude</field>
 *   <field name="el_avg" type="float">Elevon LEFT cyclic average</field>
 *   <field name="er_avg" type="float">Elevon RIGHT cyclic average</field>
 *   <field name="el_delta" type="float">Elevon LEFT cyclic half amplitude</field>
 *   <field name="er_delta" type="float">Elevon RIGHT cyclic half amplitude</field>
 *   <field name="phase_mot" type="float">Motor cyclic phase</field>
     <field name="phase_el" type="float">Elevon cyclic phase</field>
     <field name="my_psi" type="float">Psi from MAG</field>
 * </message>
 */
#include "subsystems/datalink/telemetry.h"
static void send_frisbee_control(struct transport_tx *trans, struct link_device *dev) {
  // Multiply with boolean to only send the value when variables are actually being used
  float fs_msg_ml_avg = cyclic_control_active * ml_avg;
  float fs_msg_ml_delta = motor_delta_active * ml_delta;
  float fs_msg_mr_avg = cyclic_control_active * mr_avg;
  float fs_msg_mr_delta = motor_delta_active * mr_delta;
  float fs_msg_el_avg = cyclic_control_active * el_avg;
  float fs_msg_el_delta = elevon_delta_active * el_delta;
  float fs_msg_er_avg = cyclic_control_active * er_avg;
  float fs_msg_er_delta = elevon_delta_active * er_delta;
  pprz_msg_send_FRISBEE_CONTROL(trans, dev, AC_ID,
                                &is_spinning,
                                &v_filt,
                                &fs_msg_ml_avg, &fs_msg_ml_delta,
                                &fs_msg_mr_avg, &fs_msg_mr_delta,
                                &fs_msg_el_avg, &fs_msg_el_delta,
                                &fs_msg_er_avg, &fs_msg_er_delta,
                                &mt_phase, &el_phase,
                                &my_psi);
}

void fs_landing_init()
{
  uint8_t i;
  for (i = 0; i < ACTUATORS_NB; i++) {
    fs_landing.commands[i] = 0;
    current_actuator_values.commands[i] = 0;
  }
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_FRISBEE_CONTROL, send_frisbee_control);
  AbiBindMsgIMU_MAG_INT32(MAG_RM3100_SENDER_ID, &mag_rm3100_ev, mag_rm3100_cb);
//  cyclic_controller_init();
}

// TODO Make sure all files agree on direction of spin (e.g. assume anti-clockwise rotation)
void fs_landing_run()
{
  if (is_fs_landing_active()) {
#if FS_LANDING_FREQ_TEST
    if (freq_test_active) {
    freq_test(&current_actuator_values, ft_start_time);
    } else {
      ft_start_time = get_sys_time_float();
      freq_test_active = true;
    }
    return;
#endif
    if (is_spinning) {
      horizontal_velocity_filter();
      my_psi_from_mag();
      spin_actuator_values(&current_actuator_values);  // Constant actuator values to maintain spin

      if (cyclic_control_active) {
        cyclic_control_values(&current_actuator_values);
//        cyclic_controller_run();
      }  // TODO Add controller here

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
    freq_test_active = false;

    // Reset horizontal velocity filter
    is_horizontal_velocity_filter_initialized = false;
    for (int i = 0; i < SEASONAL_L; i++) {
      seasonal_arr[i] = CT_DEFAULT;
    }
    bt = 0;
    ctr = 0;
  }
  return;
}

bool pre_spin_actuator_values()
{
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

// Compute matching paparazzi actuator signal value for motor right (forward)
// given motor left (reverse) pprz value so that the rotation axis is at the center
float get_matching_motl_val(int32_t val) {
  float a_rev = -1.967;    // y-intercept
  float b_rev = 0.001243;  // slope

  float a_fwd = -4.013;    // y-intercept
  float b_fwd = 0.002553;  // slope

  // pprz to pwm
  float pwm_r = SERVO_S_THROTTLE_LEFT_NEUTRAL + ((SERVO_S_THROTTLE_LEFT_MAX - SERVO_S_THROTTLE_LEFT_NEUTRAL) / (float)MAX_PPRZ) * abs(val);
  // approximate fwd thrust
  float thr = a_rev + b_rev * pwm_r;
  // calculate pwm of forward motor from thrust
  float pwm_f = (thr - a_fwd) / b_fwd;
  // remap to pprz value
  // motor value returned should always be positive (for forward motor)
  float pprz_f = (MAX_PPRZ / (SERVO_S_THROTTLE_RIGHT_MAX - SERVO_S_THROTTLE_RIGHT_NEUTRAL)) * (pwm_f - SERVO_S_THROTTLE_RIGHT_NEUTRAL);
  return pprz_f;
}

/*
 * Triple Exponential Smoothing (Holt-Winters) filter for horizontal velocity norm
 * Hyper-parameters ALPHA, BETA, GAMMA tuned by hand
 * Seasonal array initialized incorrectly to some value that seems to work
 */
void horizontal_velocity_filter() {
  float ins_vx = stateGetSpeedNed_f()->x;
  float ins_vy = stateGetSpeedNed_f()->y;
  ins_v = sqrtf(pow(ins_vx, 2) + pow(ins_vy, 2));

  // Prevent potential overflow
  if (ctr >= SEASONAL_L * 1000) {
    ctr = 0;
  }

  if (!is_horizontal_velocity_filter_initialized) {
    is_horizontal_velocity_filter_initialized = true;
    filt_init_start_t = get_sys_time_float();
  } else {
    if (get_sys_time_float() - filt_init_start_t < FILT_INIT_TIME) {
      prev_v_filt = ins_v;
    }
    else {
      float ct = seasonal_arr[ctr % SEASONAL_L];

      v_filt = FS_V_FILT_ALPHA * (ins_v - ct) + (1 - FS_V_FILT_ALPHA) * (prev_v_filt + bt);
      bt = FS_V_FILT_BETA * (v_filt - prev_v_filt) + (1 - FS_V_FILT_BETA) * bt;
      seasonal_arr[ctr % SEASONAL_L] = FS_V_FILT_GAMMA * (ins_v - v_filt - bt) + (1 - FS_V_FILT_GAMMA) * ct;

      prev_v_filt = v_filt;
      ctr++;
    }
  }
}

void my_psi_from_mag() {
  float phi = stateGetNedToBodyEulers_f()->phi;
  float theta = stateGetNedToBodyEulers_f()->theta;

  float sin_phi = sinf(phi);
  float cos_phi = cosf(phi);
  float sin_theta = sinf(theta);
  float cos_theta = cosf(theta);

  float m_00 = cos_theta;
  float m_01 = sin_phi * sin_theta;
  float m_02 = cos_phi * sin_theta;
  // float m_10 = 0;
  float m_11 = cos_phi;
  float m_12 = -sin_phi;

  float mx = _mag->x;
  float my = _mag->y;
  float mz = _mag->z;

  float mx_prime = m_00 * mx + m_01 * my + m_02 * mz;
  float my_prime = m_11 * my + m_12 * mz;

  my_psi = atan2(my_prime, mx_prime);
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
