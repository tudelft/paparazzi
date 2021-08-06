//
// Created by matteo on 29/04/2021.
//

#include "subsystems/gps.h"
#include "stdlib.h"

#include "my_complementary_filter.h"
#include "impulse_control.h"

#define IMPULSE_CONTROL_DEBUG TRUE

uint8_t impulse_use_motor_l = false;
uint8_t impulse_use_motor_r = false;
uint8_t impulse_use_elevon_l = false;
uint8_t impulse_use_elevon_r = false;

float ml_value = 0.;  // [-9600, 9600] pprz actuator value
float mr_value = 0.;
float el_value = 0.;
float er_value = 0.;

// impulse at this azimuth value [degrees]
uint16_t elevon_activate_at = 0;
uint16_t motor_activate_at = 0;

// Turn impulse into step (activate within azimuth arc) [degrees]
float elevon_half_arc = 2;
float motor_half_arc = 2;

void impulse_control_values(struct fs_landing_t *actuator_values) {
  float current_yaw = stateGetNedToBodyEulers_f() -> psi;
  float filt_yaw = my_complementary_filter((double)gps.course / 1e7);

  float ele_h_arc = RadOfDeg(elevon_half_arc);
  float mot_h_arc = RadOfDeg(motor_half_arc);

  float ele_az = RadOfDeg(elevon_activate_at);
  float mot_az = RadOfDeg(motor_activate_at);
  FLOAT_ANGLE_NORMALIZE(ele_az);
  FLOAT_ANGLE_NORMALIZE(mot_az);

  float ele_arc_min = ele_az - ele_h_arc;
  float ele_arc_max = ele_az + ele_h_arc;
  float mot_arc_min = mot_az - mot_h_arc;
  float mot_arc_max = mot_az + mot_h_arc;
  FLOAT_ANGLE_NORMALIZE(ele_arc_min);
  FLOAT_ANGLE_NORMALIZE(ele_arc_max);
  FLOAT_ANGLE_NORMALIZE(mot_arc_min);
  FLOAT_ANGLE_NORMALIZE(mot_arc_max);

  if (impulse_use_elevon_l) {
    if ((current_yaw < ele_arc_min) || (current_yaw > ele_arc_max)) {
      int32_t elevon_l = (int32_t)(el_value);
      actuator_values->commands[SERVO_S_ELEVON_LEFT] = elevon_l;
    }
  }
  if (impulse_use_elevon_r) {
    if ((current_yaw < ele_arc_min) || (current_yaw > ele_arc_max)) {
      int32_t elevon_r = (int32_t)(er_value);
      actuator_values->commands[SERVO_S_ELEVON_RIGHT] = elevon_r;
    }
  }
  if (impulse_use_motor_l) {
    if ((current_yaw < mot_arc_min) || (current_yaw > mot_arc_max)) {
      int32_t motor_l = (int32_t)(ml_value);
      actuator_values->commands[SERVO_S_THROTTLE_LEFT] = motor_l;
    }
  }
  if (impulse_use_motor_r) {
    if ((current_yaw < mot_arc_min) || (current_yaw > mot_arc_max)) {
      int32_t motor_r = (int32_t)(mr_value);
      actuator_values->commands[SERVO_S_THROTTLE_RIGHT] = motor_r;
    }
  }
}