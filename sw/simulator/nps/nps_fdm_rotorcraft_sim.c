/*
 * Copyright (C) 2026 OpenUAS
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file nps_fdm_rotorcraft_sim.c
 * Minimum complexity flight dynamic model for rotorcraft.
 *
 * Rotorcraft counterpart of nps_fdm_fixedwing_sim.c: a deliberately simple
 * hover-dynamics model whose only purpose is to allow testing of flight
 * plans, navigation and guidance without a full physics backend (JSBSim,
 * Gazebo, ...). It is NOT suitable for control-loop tuning.
 *
 * Model summary (flat Earth, local NED frame anchored at the flight plan
 * origin):
 *  - Attitude: first-order Euler-rate response to the normalized
 *    ROLL/PITCH/YAW stabilization commands (time constant
 *    NPS_ROTORCRAFT_SIM_ATT_TAU), with bank/pitch limited to
 *    NPS_ROTORCRAFT_SIM_MAX_TILT.
 *  - Thrust: specific thrust g * thrust_cmd / NPS_ROTORCRAFT_SIM_HOVER_THROTTLE
 *    along the body z axis, i.e. the vehicle hovers exactly at
 *    thrust_cmd == NPS_ROTORCRAFT_SIM_HOVER_THROTTLE.
 *  - Translation: thrust vector tilted by the attitude, gravity, and a
 *    linear drag -NPS_ROTORCRAFT_SIM_DRAG * (velocity - wind) which sets the
 *    terminal speed per bank angle and provides wind response.
 *  - Ground: flat ground at the LTP origin altitude; the vehicle rests
 *    level until the commanded thrust exceeds the weight.
 *
 * Requires NPS_USE_COMMANDS: the FDM consumes the autopilot commands
 * (COMMAND_ROLL/PITCH/YAW/THRUST) directly instead of per-motor actuator
 * values, so it works for any rotorcraft airframe regardless of the motor
 * layout. AHRS and INS are expected to be bypassed (see module
 * fdm_rotorcraft_sim.xml).
 */

#include "nps_fdm.h"

#include <math.h>

#include "std.h"
#include "math/pprz_algebra_double.h"
#include "math/pprz_geodetic_double.h"
#include "math/pprz_isa.h"

#include "generated/airframe.h"
#include "generated/flight_plan.h"

#ifndef ROTORCRAFT_FIRMWARE
#error "The module fdm_rotorcraft_sim is a basic flight model for rotorcraft only"
#endif

#ifndef NPS_USE_COMMANDS
#error "The module fdm_rotorcraft_sim requires NPS_USE_COMMANDS (loading the module should set it)"
#endif

/** Normalized thrust command that exactly balances gravity */
#ifndef NPS_ROTORCRAFT_SIM_HOVER_THROTTLE
#ifdef GUIDANCE_V_NOMINAL_HOVER_THROTTLE
#define NPS_ROTORCRAFT_SIM_HOVER_THROTTLE GUIDANCE_V_NOMINAL_HOVER_THROTTLE
#else
#define NPS_ROTORCRAFT_SIM_HOVER_THROTTLE 0.5
#endif
#endif

/** Roll/pitch rate at full stick in rad/s */
#ifndef NPS_ROTORCRAFT_SIM_MAX_TILT_RATE
#define NPS_ROTORCRAFT_SIM_MAX_TILT_RATE RadOfDeg(180.)
#endif

/** Yaw rate at full stick in rad/s */
#ifndef NPS_ROTORCRAFT_SIM_MAX_YAW_RATE
#define NPS_ROTORCRAFT_SIM_MAX_YAW_RATE RadOfDeg(120.)
#endif

/** Time constant of the angular rate response in s */
#ifndef NPS_ROTORCRAFT_SIM_ATT_TAU
#define NPS_ROTORCRAFT_SIM_ATT_TAU 0.05
#endif

/** Maximum bank/pitch angle in rad */
#ifndef NPS_ROTORCRAFT_SIM_MAX_TILT
#define NPS_ROTORCRAFT_SIM_MAX_TILT RadOfDeg(45.)
#endif

/** Linear drag coefficient in 1/s (terminal speed shaping) */
#ifndef NPS_ROTORCRAFT_SIM_DRAG
#define NPS_ROTORCRAFT_SIM_DRAG 0.35
#endif

// NpsFdm structure
struct NpsFdm fdm;

// Reference point
static struct LtpDef_d ltpdef;

struct rotorcraft_sim_state {
  struct NedCoor_d pos;         ///< position in LTP NED frame [m]
  struct NedCoor_d vel;         ///< velocity in LTP NED frame [m/s]
  struct NedCoor_d accel;       ///< acceleration in LTP NED frame [m/s2]
  struct DoubleEulers attitude; ///< LTP to body Euler angles [rad]
  struct DoubleEulers rates;    ///< Euler angle time derivatives [rad/s]
  struct DoubleVect3 wind;      ///< wind velocity in LTP NED frame [m/s]
};

static struct rotorcraft_sim_state sim_state;

static void init_ltp(void);

/** Rotate a NED vector to the body frame with the current attitude */
static void body_of_ned(struct DoubleVect3 *body, const struct DoubleVect3 *ned)
{
  const double cphi = cos(sim_state.attitude.phi), sphi = sin(sim_state.attitude.phi);
  const double cthe = cos(sim_state.attitude.theta), sthe = sin(sim_state.attitude.theta);
  const double cpsi = cos(sim_state.attitude.psi), spsi = sin(sim_state.attitude.psi);
  body->x = cthe * cpsi * ned->x + cthe * spsi * ned->y - sthe * ned->z;
  body->y = (sphi * sthe * cpsi - cphi * spsi) * ned->x
            + (sphi * sthe * spsi + cphi * cpsi) * ned->y
            + sphi * cthe * ned->z;
  body->z = (cphi * sthe * cpsi + sphi * spsi) * ned->x
            + (cphi * sthe * spsi - sphi * cpsi) * ned->y
            + cphi * cthe * ned->z;
}

void nps_fdm_init(double dt)
{
  fdm.init_dt = dt; // (1 / simulation freq)
  fdm.curr_dt = dt;
  fdm.time = dt;

  fdm.on_ground = TRUE;

  fdm.nan_count = 0;
  fdm.pressure = -1;
  fdm.pressure_sl = PPRZ_ISA_SEA_LEVEL_PRESSURE;
  fdm.total_pressure = -1;
  fdm.dynamic_pressure = -1;
  fdm.temperature = -1;

  VECT3_ASSIGN(sim_state.pos, 0., 0., 0.);
  VECT3_ASSIGN(sim_state.vel, 0., 0., 0.);
  VECT3_ASSIGN(sim_state.accel, 0., 0., 0.);
  EULERS_ASSIGN(sim_state.attitude, 0., 0., 0.);
  EULERS_ASSIGN(sim_state.rates, 0., 0., 0.);
  VECT3_ASSIGN(sim_state.wind, 0., 0., 0.);

  init_ltp();
}

void nps_fdm_run_step(bool launch __attribute__((unused)), double *commands,
                      int commands_nb __attribute__((unused)))
{
  const double dt = fdm.curr_dt;

  /* Normalized commands ([-1:1] for roll/pitch/yaw, [0:1] for thrust) */
  double cmd_roll = commands[COMMAND_ROLL];
  double cmd_pitch = commands[COMMAND_PITCH];
  double cmd_yaw = commands[COMMAND_YAW];
  double cmd_thrust = commands[COMMAND_THRUST];
  BoundAbs(cmd_roll, 1.);
  BoundAbs(cmd_pitch, 1.);
  BoundAbs(cmd_yaw, 1.);
  Bound(cmd_thrust, 0., 1.);

  /* Attitude: first-order (unconditionally stable) rate response */
  const double alpha = dt / (NPS_ROTORCRAFT_SIM_ATT_TAU + dt);
  sim_state.rates.phi += alpha * (NPS_ROTORCRAFT_SIM_MAX_TILT_RATE * cmd_roll - sim_state.rates.phi);
  sim_state.rates.theta += alpha * (NPS_ROTORCRAFT_SIM_MAX_TILT_RATE * cmd_pitch - sim_state.rates.theta);
  sim_state.rates.psi += alpha * (NPS_ROTORCRAFT_SIM_MAX_YAW_RATE * cmd_yaw - sim_state.rates.psi);

  sim_state.attitude.phi += sim_state.rates.phi * dt;
  BoundAbs(sim_state.attitude.phi, NPS_ROTORCRAFT_SIM_MAX_TILT);
  sim_state.attitude.theta += sim_state.rates.theta * dt;
  BoundAbs(sim_state.attitude.theta, NPS_ROTORCRAFT_SIM_MAX_TILT);
  sim_state.attitude.psi += sim_state.rates.psi * dt;
  NormRadAngle(sim_state.attitude.psi);

  /* Specific thrust along body -z, tilted into NED by the attitude */
  const double a_thrust = PPRZ_ISA_GRAVITY * cmd_thrust / NPS_ROTORCRAFT_SIM_HOVER_THROTTLE;
  const double cphi = cos(sim_state.attitude.phi), sphi = sin(sim_state.attitude.phi);
  const double cthe = cos(sim_state.attitude.theta), sthe = sin(sim_state.attitude.theta);
  const double cpsi = cos(sim_state.attitude.psi), spsi = sin(sim_state.attitude.psi);

  sim_state.accel.x = -a_thrust * (cphi * sthe * cpsi + sphi * spsi)
                      - NPS_ROTORCRAFT_SIM_DRAG * (sim_state.vel.x - sim_state.wind.x);
  sim_state.accel.y = -a_thrust * (cphi * sthe * spsi - sphi * cpsi)
                      - NPS_ROTORCRAFT_SIM_DRAG * (sim_state.vel.y - sim_state.wind.y);
  sim_state.accel.z = PPRZ_ISA_GRAVITY - a_thrust * cphi * cthe
                      - NPS_ROTORCRAFT_SIM_DRAG * (sim_state.vel.z - sim_state.wind.z);

  /* Semi-implicit Euler integration */
  VECT3_ADD_SCALED(sim_state.vel, sim_state.accel, dt);
  VECT3_ADD_SCALED(sim_state.pos, sim_state.vel, dt);

  /* Flat ground at LTP origin: rest level until thrust beats gravity */
  if (sim_state.pos.z >= 0.) {
    sim_state.pos.z = 0.;
    VECT3_ASSIGN(sim_state.vel, 0., 0., 0.);
    VECT3_ASSIGN(sim_state.accel, 0., 0., 0.);
    sim_state.attitude.phi = 0.;
    sim_state.attitude.theta = 0.;
    sim_state.rates.phi = 0.;
    sim_state.rates.theta = 0.;
    fdm.on_ground = TRUE;
  } else {
    fdm.on_ground = FALSE;
  }

  fdm.time += dt;

  /****************************************************************************/
  // Export the state to the FDM structure |
  // ---------------------------------------|

  /* Position, velocity and acceleration */
  ecef_of_ned_point_d(&fdm.ecef_pos, &ltpdef, &sim_state.pos);
  lla_of_ecef_d(&fdm.lla_pos, &fdm.ecef_pos);
  fdm.ltpprz_pos = sim_state.pos;
  fdm.hmsl = (double)NAV_ALT0 / 1000. - sim_state.pos.z;
  fdm.agl = -sim_state.pos.z; // flat Earth

  ecef_of_ned_vect_d(&fdm.ecef_ecef_vel, &ltpdef, &sim_state.vel);
  fdm.ltp_ecef_vel = sim_state.vel;
  fdm.ltpprz_ecef_vel = sim_state.vel;

  ecef_of_ned_vect_d(&fdm.ecef_ecef_accel, &ltpdef, &sim_state.accel);
  fdm.ltp_ecef_accel = sim_state.accel;
  fdm.ltpprz_ecef_accel = sim_state.accel;

  /* Attitude */
  fdm.ltp_to_body_eulers = sim_state.attitude;
  double_quat_of_eulers(&fdm.ltp_to_body_quat, &fdm.ltp_to_body_eulers);
  fdm.ltpprz_to_body_eulers = fdm.ltp_to_body_eulers;
  fdm.ltpprz_to_body_quat = fdm.ltp_to_body_quat;

  /* Body rates from Euler angle derivatives */
  fdm.body_ecef_rotvel.p = sim_state.rates.phi - sim_state.rates.psi * sthe;
  fdm.body_ecef_rotvel.q = sim_state.rates.theta * cphi + sim_state.rates.psi * sphi * cthe;
  fdm.body_ecef_rotvel.r = -sim_state.rates.theta * sphi + sim_state.rates.psi * cphi * cthe;
  fdm.body_inertial_rotvel = fdm.body_ecef_rotvel;

  /* Velocity and accelerometer measurement (specific force) in body frame */
  struct DoubleVect3 vel_ned = { sim_state.vel.x, sim_state.vel.y, sim_state.vel.z };
  body_of_ned(&fdm.body_ecef_vel, &vel_ned);
  struct DoubleVect3 accel_ned = { sim_state.accel.x, sim_state.accel.y, sim_state.accel.z };
  body_of_ned(&fdm.body_inertial_accel, &accel_ned);
  fdm.body_ecef_accel = fdm.body_inertial_accel;
  struct DoubleVect3 sforce_ned = accel_ned;
  sforce_ned.z -= PPRZ_ISA_GRAVITY;
  body_of_ned(&fdm.body_accel, &sforce_ned);

  /* Airspeed as norm of velocity relative to the air mass */
  struct DoubleVect3 airvel;
  VECT3_DIFF(airvel, vel_ned, sim_state.wind);
  fdm.airspeed = double_vect3_norm(&airvel);
}

/**************************
 ** Generating LTP plane **
 **************************/

static void init_ltp(void)
{

  struct LlaCoor_d llh_nav0; /* Height above the ellipsoid */
  llh_nav0.lat = RadOfDeg((double)NAV_LAT0 / 1e7);
  llh_nav0.lon = RadOfDeg((double)NAV_LON0 / 1e7);
  llh_nav0.alt = (double)(NAV_ALT0) / 1000.0;

  struct EcefCoor_d ecef_nav0;

  ecef_of_lla_d(&ecef_nav0, &llh_nav0);

  ltp_def_from_ecef_d(&ltpdef, &ecef_nav0);
  fdm.ecef_pos = ecef_nav0;

  // accel and mag data should not be used
  // AHRS and INS are bypassed
  fdm.ltp_g.x = 0.;
  fdm.ltp_g.y = 0.;
  fdm.ltp_g.z = 0.;

  fdm.ltp_h.x = 1.;
  fdm.ltp_h.y = 0.;
  fdm.ltp_h.z = 0.;

}


void nps_fdm_set_wind(double speed, double dir)
{
  sim_state.wind.x = speed * cos(dir);
  sim_state.wind.y = speed * sin(dir);
  sim_state.wind.z = 0.;
  VECT3_COPY(fdm.wind, sim_state.wind);
}

void nps_fdm_set_wind_ned(double wind_north, double wind_east, double wind_down)
{
  sim_state.wind.x = wind_north;
  sim_state.wind.y = wind_east;
  sim_state.wind.z = wind_down;
  VECT3_COPY(fdm.wind, sim_state.wind);
}

void nps_fdm_set_turbulence(double wind_speed __attribute__((unused)),
                            int turbulence_severity __attribute__((unused)))
{
}

void nps_fdm_set_temperature(double temp __attribute__((unused)),
                             double h __attribute__((unused)))
{
}

