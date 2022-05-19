//
// Created by matteo on 06/08/2021.
//
#include <math.h>
#include "state.h"

//#include "modules/nav/common_nav.h"
#include "generated/flight_plan.h"
#include "generated/airframe.h"

#include "cyclic_controller.h"

struct cyclic_controller_t cyclic_controller;

float cc_x_ned, cc_y_ned, cc_x_ecef, cc_y_ecef;
float cc_vx_ned, cc_vy_ned, cc_vx_ecef, cc_vy_ecef;
float cc_hor_norm;
float cc_hor_dir;

// GCS settings
//float cc_kp_gain = 1;
//float cc_kd_gain = 1;
//float max_amplitude = 0.3;


void cyclic_controller_init() {
  NavSetWaypointHere(WP_STDBY);

  cc_x_ned = stateGetPositionNed_f() -> x;
  cc_y_ned = stateGetPositionNed_f() -> y;
  cc_x_ecef = stateGetPositionEcef_f() -> x;
  cc_y_ecef = stateGetPositionEcef_f() -> y;

  cc_vx_ned = stateGetSpeedNed_f() -> x;
  cc_vy_ned = stateGetSpeedNed_f() -> y;
  cc_vx_ecef = stateGetSpeedEcef_f() -> x;
  cc_vy_ecef = stateGetSpeedEcef_f() -> y;

  cc_hor_norm = stateGetHorizontalSpeedNorm_f();
  cc_hor_dir  = stateGetHorizontalSpeedDir_f();  // rad

}

void cyclic_controller_run() {

}

