//
// Created by matteo on 06/08/2021.
//
#include <math.h>
#include "state.h"

#include "subsystems/navigation/common_nav.h"
#include "generated/flight_plan.h"
#include "generated/airframe.h"

#include "cyclic_controller.h"

struct cyclic_controller_t cyclic_controller;

float psi;
float x, y;
float vx, vy;
float x_stb, y_stb;

// GCS settings
float cc_kp_gain = 1;
float cc_kd_gain = 1;
float max_amplitude = 0.3;

void cyclic_controller_init() {
  NavSetWaypointHere(WP_STDBY);
}

void cyclic_controller_run() {
  psi = stateGetNedToBodyEulers_f() -> psi;
  x = stateGetPositionNed_f() -> x;
  y = stateGetPositionNed_f() -> y;
  vx = stateGetSpeedNed_f() -> x;
  vy = stateGetSpeedNed_f() -> y;
  x_stb = WaypointX(WP_STDBY);
  y_stb = WaypointY(WP_STDBY);

  float x_err = x_stb - x;
  float y_err = y_stb - y;
  float desired_heading = atan2(y_err, x_err);

  // FIXME velocity angles etc
  float d_err = x_err * sqrtf(1 + (y_err / x_err) * (y_err / x_err));
  float vel = (vx * cosf(psi) + vy * sinf(psi)) * cosf(desired_heading) + (vx * sinf(psi) + vy * cosf(psi))* sinf(desired_heading);

  float cmd = 4 * (cc_kp_gain * d_err - cc_kd_gain * vel);
  Bound(cmd, 0, 1);
  float ampl = cmd * max_amplitude;

  cyclic_controller.cyclic_phase = desired_heading;
  cyclic_controller.cyclic_amplitude = ampl;
  cyclic_controller.d = d_err;
  cyclic_controller.vel = vel;
}

