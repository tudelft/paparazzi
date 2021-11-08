//
// Created by matteo on 15/09/2020.
//

#include <math.h>
#include <stdlib.h>
#include "subsystems/radio_control.h"
#include "state.h"

#include "cyclic_controller.h"
#include "cyclic_control.h"

#include "subsystems/datalink/downlink.h"

#define CYCLIC_CONTROL_DEBUG TRUE

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

uint8_t balance_motor_forces = true;
uint8_t use_square_sig = true;
uint8_t phase_pilot_control = false;
<<<<<<< Updated upstream
uint8_t cc_feed_forward = false;
uint8_t use_controller = false;

uint8_t has_cc_ff_started = false;
float cc_ff_start_time;
float cc_ff_side_time = 2;  // 2 seconds per square loop "side"

int cyclic_dbg_ctr = 0;  // For debug telemetry

void cyclic_control_values(struct fs_landing_t *actuator_values) {
  float current_yaw = stateGetNedToBodyEulers_f()->psi;
=======
uint8_t use_controller = false;

int cyclic_dbg_ctr = 0;  // For debug telemetry

void cyclic_control_values(struct fs_landing_t *actuator_values) {
  float current_yaw = stateGetNedToBodyEulers_f() -> psi;
>>>>>>> Stashed changes

  float mt_phase_rad = RadOfDeg(mt_phase);
  float el_phase_rad = RadOfDeg(el_phase);
  if (phase_pilot_control) {
    int64_t radio_roll = radio_control.values[RADIO_ROLL];
    if (labs(radio_roll) < 100) {
      radio_roll = 0;
    }
    int64_t radio_pitch = -radio_control.values[RADIO_PITCH];
    if (labs(radio_pitch) < 100) {
      radio_pitch = 0;
    }
    float pilot_phase_rad = atan2f(radio_pitch, radio_roll);
<<<<<<< Updated upstream
    mt_phase_rad += pilot_phase_rad;
    el_phase_rad += pilot_phase_rad;

#if CYCLIC_CONTROL_DEBUG
#if PERIODIC_TELEMETRY
=======
    mt_phase_rad = pilot_phase_rad;
    el_phase_rad = pilot_phase_rad;

#if CYCLIC_CONTROL_DEBUG
  #if PERIODIC_TELEMETRY
>>>>>>> Stashed changes
    fs_landing_dbg_values[0] = radio_roll;
    fs_landing_dbg_values[1] = radio_pitch;
    fs_landing_dbg_values[2] = DegOfRad(pilot_phase_rad);
    if (cyclic_dbg_ctr == 20) {
      cyclic_dbg_ctr = 0;
      DOWNLINK_SEND_PAYLOAD_FLOAT(DefaultChannel, DefaultDevice, N_DBG_VALUES, fs_landing_dbg_values);
    } else {
      cyclic_dbg_ctr++;
    }
#endif
#endif
  } else if (cc_feed_forward) {
    // Fly a feed-forward square loop by changing phase by 90deg every cc_ff_side_time seconds
    if (!has_cc_ff_started) {
      cc_ff_start_time = get_sys_time_float();
      has_cc_ff_started = true;
    } else {
      float cc_ff_dt = get_sys_time_float() - cc_ff_start_time;
      if (0 < cc_ff_dt & cc_ff_dt <= cc_ff_side_time) {
//        mt_phase_rad += 0;
        el_phase_rad += 0;
      } else if (cc_ff_side_time < cc_ff_dt & cc_ff_dt <= 2 * cc_ff_side_time) {
//        mt_phase_rad += M_PI_2;
        el_phase_rad += M_PI_2;
      } else if (2 * cc_ff_side_time < cc_ff_dt & cc_ff_dt <= 3 * cc_ff_side_time) {
//        mt_phase_rad += M_PI;
        el_phase_rad += M_PI;
      } else if (3 * cc_ff_side_time < cc_ff_dt & cc_ff_dt <= 4 * cc_ff_side_time) {
//        mt_phase_rad += M_PI + M_PI_2;
        el_phase_rad += M_PI + M_PI_2;
      } else {  // Test end, disable cyclic
        ml_delta = 0;
        mr_delta = 0;
      }
    }
  } else if (use_controller) {
    cyclic_controller_run();
    ml_delta = cyclic_controller.cyclic_amplitude;
    mr_delta = cyclic_controller.cyclic_amplitude;
    mt_phase_rad = cyclic_controller.cyclic_phase;
#if CYCLIC_CONTROL_DEBUG
  #if PERIODIC_TELEMETRY
    fs_landing_dbg_values[0] = cyclic_controller.cyclic_amplitude;
    fs_landing_dbg_values[1] = cyclic_controller.cyclic_phase;
    fs_landing_dbg_values[2] = cyclic_controller.d;
    fs_landing_dbg_values[3] = cyclic_controller.vel;
    if (cyclic_dbg_ctr == 20) {
      cyclic_dbg_ctr = 0;
      DOWNLINK_SEND_PAYLOAD_FLOAT(DefaultChannel, DefaultDevice, N_DBG_VALUES, fs_landing_dbg_values);
    } else {
      cyclic_dbg_ctr++;
    }
  #endif
#endif
  }

  float cos_val = cosf(current_yaw - el_phase_rad);
  int32_t elevon_l = (int32_t) (9600 * (el_avg + el_delta * cos_val));
  int32_t elevon_r = (int32_t) (9600 * (er_avg + er_delta * cos_val));

  if (balance_motor_forces) {
    ml_avg = get_matching_motl_val(abs(mr_avg) * 9600) / 9600;
  }
  cos_val = cosf(current_yaw + mt_phase_rad);
  int32_t motor_l = 0;
  int32_t motor_r = 0;
  if (use_square_sig) {
    if (cos_val > 0) {
      motor_l = (int32_t) (9600 * (ml_avg + ml_delta));
      motor_r = (int32_t) (9600 * (mr_avg + mr_delta));
    } else {
      motor_l = (int32_t) (9600 * (ml_avg - ml_delta));
      motor_r = (int32_t) (9600 * (mr_avg - mr_delta));
    }
  } else {
    motor_l = (int32_t) (9600 * (ml_avg + ml_delta * cos_val));
    motor_r = (int32_t) (9600 * (mr_avg + mr_delta * cos_val));
  }

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

void cyclic_control_cc_feed_forward_handler(uint8_t value) {
  if (value) {
    cc_feed_forward = true;
    has_cc_ff_started = false;
  } else {
    cc_feed_forward = false;
  }
}

void cyclic_control_use_controller_handler(uint8_t value) {
  if (!use_controller && value) {
    cyclic_controller_init();
    use_controller = true;
  } else {
    use_controller = value;
  }
}
