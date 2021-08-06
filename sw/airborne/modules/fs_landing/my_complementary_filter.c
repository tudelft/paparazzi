//
// Created by matteo on 26/07/2021.
//
#include "math/pprz_algebra_float.h"

#include "state.h"
#include "mcu_periph/sys_time.h"

#include "subsystems/datalink/downlink.h"

#include "my_complementary_filter.h"

#define IMPULSE_CONTROL_DEBUG TRUE

float old_t = 0;
double old_gps_yaw = 0;
float filtered_yaw = 0;

float heading_weight = 0.20;
int ctr = 0;

void is_measurement_delayed(float heading);

float my_complementary_filter(float heading) {
  FLOAT_ANGLE_NORMALIZE(heading);  // rad [-pi, pi]
  is_measurement_delayed(heading);
  // First check for optitrack update
  if (heading != old_gps_yaw) {
    old_t = get_sys_time_float();
    old_gps_yaw = heading;
    float angle_diff = heading - filtered_yaw;
    FLOAT_ANGLE_NORMALIZE(angle_diff);
    filtered_yaw += angle_diff * heading_weight;
  } else {
    float t = get_sys_time_float();
    float dt = t - old_t;
    filtered_yaw += dt * stateGetBodyRates_f()->r;
    old_t = t;
  }

  FLOAT_ANGLE_NORMALIZE(filtered_yaw);
#if IMPULSE_CONTROL_DEBUG
  #if PERIODIC_TELEMETRY
  fs_landing_dbg_values[0] = heading;
  fs_landing_dbg_values[1] = stateGetBodyRates_f()->r;
  fs_landing_dbg_values[2] = filtered_yaw;
  if (ctr == 20) {
    ctr = 0;
    DOWNLINK_SEND_PAYLOAD_FLOAT(DefaultChannel, DefaultDevice, N_DBG_VALUES, fs_landing_dbg_values);
  } else {
    ctr++;
  }
  #endif
#endif
  return filtered_yaw;
}

void is_measurement_delayed(float heading) {
  float dt = get_sys_time_float() - old_t;
  float gps_r = (heading - old_gps_yaw) / dt;
#if IMPULSE_CONTROL_DEBUG
  #if PERIODIC_TELEMETRY
  fs_landing_dbg_values[3] = gps_r;
  #endif
#endif
}