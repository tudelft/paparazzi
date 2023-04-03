/*
 * Copyright (C) 2022
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

#include "nps_fdm.h"

#include <stdlib.h>
#include <stdio.h>

#include "math/pprz_geodetic.h"
#include "math/pprz_geodetic_double.h"
#include "math/pprz_geodetic_float.h"
#include "math/pprz_algebra.h"
#include "math/pprz_algebra_float.h"
#include "math/pprz_isa.h"


#include "generated/airframe.h"
#include "generated/flight_plan.h"

#include "simulink/darko/Darko/darko_ert_rtw/darko.h"

#include "state.h"

#ifndef NPS_SIM_TO_PPRZ_PHI
#define NPS_SIM_TO_PPRZ_PHI 0.
#endif

#ifndef NPS_SIM_TO_PPRZ_THETA
#define NPS_SIM_TO_PPRZ_THETA -90.
#endif

#ifndef NPS_SIM_TO_PPRZ_PSI
#define NPS_SIM_TO_PPRZ_PSI 0.
#endif

// NpsFdm structure
struct NpsFdm fdm;

// Reference point
static struct LtpDef_d ltpdef_d;

// rotation from simulink to pprz
static struct DoubleQuat quat_to_pprz;

// Static functions declaration
static void init_ltp(void);

static void feed_cmd(double *commands, int commands_nb);

static void fetch_pos(void);
static void fetch_vel(void);
static void fetch_accel(void);
static void fetch_orient(void);
static void fetch_angular_vel(void);
static void fetch_rotaccel(void);

//static void print_state(void);
static int check_for_nan(void);

void nps_fdm_init(double dt)
{
  fdm.init_dt = dt;
  fdm.time = dt;
  //fdm.on_ground = false;

  fdm.nan_count = 0;
  fdm.pressure = -1;
  fdm.pressure_sl = PPRZ_ISA_SEA_LEVEL_PRESSURE;
  fdm.total_pressure = -1;
  fdm.dynamic_pressure = -1;
  fdm.temperature = -1;

  struct DoubleEulers e2pprz = {
    RadOfDeg(NPS_SIM_TO_PPRZ_PHI),
    RadOfDeg(NPS_SIM_TO_PPRZ_THETA),
    RadOfDeg(NPS_SIM_TO_PPRZ_PSI)
  };
  double_quat_of_eulers(&quat_to_pprz, &e2pprz);

  init_ltp();

  rtU.u[0] = 0;
  rtU.u[1] = 0;
  rtU.u[2] = 1;
  rtU.u[3] = 1;

  for(int i=0; i<3; i++) {
    rtU.w[i] = 0;
  }
  darko_initialize();
  //print_state();
  darko_step();

  fetch_pos();
  fetch_vel();
  fetch_accel();
  fetch_orient();
  fetch_angular_vel();
  fetch_rotaccel();

  //print_state();

}

void nps_fdm_run_step(bool launch __attribute__((unused)), double *commands, int commands_nb __attribute__((unused)))
{
  feed_cmd(commands, commands_nb);

  //autopilot_in_flight()  autopilot.motors_on

  if (autopilot_in_flight()) {
    darko_step();
  }

    // fdm.time = rtY.time; //Really need ?

    // Transform ltp definition to double for accuracy

    ltpdef_d.ecef.x = state.ned_origin_f.ecef.x;
    ltpdef_d.ecef.y = state.ned_origin_f.ecef.y;
    ltpdef_d.ecef.z = state.ned_origin_f.ecef.z;
    ltpdef_d.lla.lat = state.ned_origin_f.lla.lat;
    ltpdef_d.lla.lon = state.ned_origin_f.lla.lon;
    ltpdef_d.lla.alt = state.ned_origin_f.lla.alt;
    for (int i = 0; i < 3 * 3; i++) {
      ltpdef_d.ltp_of_ecef.m[i] = state.ned_origin_f.ltp_of_ecef.m[i];
    }
    ltpdef_d.hmsl = state.ned_origin_f.hmsl;

    fetch_pos();
    fetch_vel();
    fetch_accel();
    fetch_orient();
    fetch_angular_vel();
    fetch_rotaccel();

    /* Check the current state to make sure it is valid (no NaNs) */
    if (check_for_nan()) {
      printf("Error: FDM simulation encountered a total of %i NaN values.\n", fdm.nan_count);
      printf("It is likely the simulation diverged and gave non-physical results. If you did\n");
      printf("not crash, check your model and/or initial conditions. Exiting with status 1.\n");
      exit(1);
    }
}

#define DBG_CMD 0


void feed_cmd(double *commands, int commands_nb __attribute__((unused))) {
  #if DBG_CMD
    printf("commands (%d), ", commands_nb);
  #endif

  if (commands_nb != 4) {exit(-45);}

  
  for (int i=0; i<commands_nb; i++) {
    rtU.u[i] = commands[i];
    #if DBG_CMD
        printf("%lf ", commands[i]);
    #endif
  }
  #if DBG_CMD
    printf("\n");
  #endif

  
}

static void fetch_pos() {
  // fetch NED pos
  fdm.ltpprz_pos.x = rtY.p[0];
  fdm.ltpprz_pos.y = rtY.p[1];
  fdm.ltpprz_pos.z = rtY.p[2];

  // convert to ECEF and LLA
  ecef_of_ned_point_d(&fdm.ecef_pos, &ltpdef_d, &fdm.ltpprz_pos);
  lla_of_ecef_d(&fdm.lla_pos, &fdm.ecef_pos);

  fdm.hmsl = -fdm.ltpprz_pos.z; // TODO check sign
  // lla_pos_pprz
  // lla_pos_geod
  // lla_pos_geoc
  fdm.agl = fdm.hmsl; // TODO

/*
printf(" pos %lf %lf %lf \n",
        fdm.ltpprz_pos.x, fdm.ltpprz_pos.y,fdm.ltpprz_pos.z);
*/
   
}

static void fetch_vel() {
  fdm.ltpprz_ecef_vel.x = rtY.v[0];
  fdm.ltpprz_ecef_vel.y = rtY.v[1];
  fdm.ltpprz_ecef_vel.z = rtY.v[2];

  ecef_of_ned_vect_d(&fdm.ecef_ecef_vel, &ltpdef_d, &fdm.ltpprz_ecef_vel);


}

static void fetch_accel() {
  fdm.ltpprz_ecef_accel.x = rtY.accel[0]; // - fdm.ltp_g.x;
  fdm.ltpprz_ecef_accel.y = rtY.accel[1]; // - fdm.ltp_g.y;
  fdm.ltpprz_ecef_accel.z = rtY.accel[2]; // - fdm.ltp_g.z;
  fdm.body_ecef_accel.x = rtY.accel[0]; // - fdm.ltp_g.x;
  fdm.body_ecef_accel.y = rtY.accel[1]; // - fdm.ltp_g.y;
  fdm.body_ecef_accel.z = rtY.accel[2]; // - fdm.ltp_g.z;
  struct DoubleVect3 tmp;
  tmp.x = rtY.accel[0];
  tmp.y = rtY.accel[1];
  tmp.z = rtY.accel[2];
  double_quat_vmult(&fdm.body_accel, &fdm.ltp_to_body_quat, &tmp);
}



static void fetch_orient() {
  struct DoubleQuat ltp_to_sim = { rtY.q[0], rtY.q[1], rtY.q[2], rtY.q[3] };
  double_quat_comp(&fdm.ltpprz_to_body_quat, &ltp_to_sim, &quat_to_pprz);
  double_eulers_of_quat(&fdm.ltpprz_to_body_eulers, &fdm.ltpprz_to_body_quat);
  QUAT_COPY(fdm.ltp_to_body_quat, fdm.ltpprz_to_body_quat);
  EULERS_COPY(fdm.ltp_to_body_eulers, fdm.ltpprz_to_body_eulers);

}

//TODO check inertial ECI frame ou ECEF frame  fichier nps_fdm.h

static void fetch_angular_vel() {
  struct DoubleVect3 sim_rates, pprz_rates;
  sim_rates.x = rtY.omega[0];
  sim_rates.y = rtY.omega[1];
  sim_rates.z = rtY.omega[2];
  double_quat_vmult(&pprz_rates, &quat_to_pprz, &sim_rates);
  fdm.body_inertial_rotvel.p = pprz_rates.x;
  fdm.body_inertial_rotvel.q = pprz_rates.y;
  fdm.body_inertial_rotvel.r = pprz_rates.z;
  fdm.body_ecef_rotvel = fdm.body_inertial_rotvel;
}

static void fetch_rotaccel() {
  struct DoubleVect3 sim_rotaccel, pprz_rotaccel;
  sim_rotaccel.x = rtY.rotaccel[0];
  sim_rotaccel.y = rtY.rotaccel[1];
  sim_rotaccel.z = rtY.rotaccel[2];
  double_quat_vmult(&pprz_rotaccel, &quat_to_pprz, &sim_rotaccel);
  fdm.body_inertial_rotaccel.p = pprz_rotaccel.x;
  fdm.body_inertial_rotaccel.q = pprz_rotaccel.y;
  fdm.body_inertial_rotaccel.r = pprz_rotaccel.z;
}



/**************************
 ** Generating LTP plane **
 **************************/

static void init_ltp(void)
{

  struct LlaCoor_d llh_nav0; /* Height above the ellipsoid */
  llh_nav0.lat = RadOfDeg((double)NAV_LAT0 / 1e7);
  llh_nav0.lon = RadOfDeg((double)NAV_LON0 / 1e7);

  struct EcefCoor_d ecef_nav0;

  ecef_of_lla_d(&ecef_nav0, &llh_nav0);

  ltp_def_from_ecef_d(&ltpdef_d, &ecef_nav0);
  fdm.ecef_pos = ecef_nav0;

  // ltp_g useless ????
  fdm.ltp_g.x = 0.;
  fdm.ltp_g.y = 0.;
  fdm.ltp_g.z = 9.81;


#ifdef AHRS_H_X
  fdm.ltp_h.x = AHRS_H_X;
  fdm.ltp_h.y = AHRS_H_Y;
  fdm.ltp_h.z = AHRS_H_Z;
  PRINT_CONFIG_MSG("Using magnetic field as defined in airframe file.")
#else
  fdm.ltp_h.x = 0.4912;
  fdm.ltp_h.y = 0.1225;
  fdm.ltp_h.z = 0.8624;
#endif

}


void nps_fdm_set_wind(double speed __attribute__((unused)),
    double dir __attribute__((unused)))
{
  printf("set_wind not implemented!\n");
}

void nps_fdm_set_wind_ned(double wind_north __attribute__((unused)),
    double wind_east __attribute__((unused)),
    double wind_down __attribute__((unused)))
{
  // printf("set_wind not implemented!\n");
  /*
     rtU.w[0] = wind_north;
     rtU.w[1] = wind_east;
     rtU.w[2] = wind_down;
     */
}

void nps_fdm_set_turbulence(double wind_speed __attribute__((unused)),
    int turbulence_severity __attribute__((unused)))
{
}

void nps_fdm_set_temperature(double temp __attribute__((unused)),
    double h __attribute__((unused)))
{
}

/*
static void print_state(void) {
  printf("state %lf\n", fdm.time);
  printf("  pos %lf %lf %lf, vel %lf %lf %lf, accel %lf %lf %lf\n",
      rtY.p[0], rtY.p[1], rtY.p[2],
      rtY.v[0], rtY.v[1], rtY.v[2],
      rtY.accel[0], rtY.accel[1], rtY.accel[2]);
  printf("  quat %lf %lf %lf %lf, rates %lf %lf %lf\n",
      rtY.q[0], rtY.q[1], rtY.q[2], rtY.q[3],
      rtY.omega[0], rtY.omega[1], rtY.omega[2]);
  struct DoubleEulers eulers;
  struct DoubleQuat quat = { rtY.q[0], rtY.q[1], rtY.q[2], rtY.q[3] };
  double_eulers_of_quat(&eulers, &quat);
  printf("  eulers %lf %lf %lf\n",
      DegOfRad(eulers.phi),
      DegOfRad(eulers.theta),
      DegOfRad(eulers.psi));
  printf("  quat nps %lf %lf %lf %lf, euler nps %lf %lf %lf\n",
      fdm.ltpprz_to_body_quat.qi, fdm.ltpprz_to_body_quat.qx, fdm.ltpprz_to_body_quat.qy, fdm.ltpprz_to_body_quat.qz,
      fdm.ltp_to_body_eulers.phi, fdm.ltp_to_body_eulers.theta, fdm.ltp_to_body_eulers.psi);
}
*/

/**
 * Checks NpsFdm struct for NaNs.
 *
 * Increments the NaN count on each new NaN
 *
 * @return Count of new NaNs. 0 for no new NaNs.
 */
static int check_for_nan(void)
{
  int orig_nan_count = fdm.nan_count;
  /* Check all elements for nans */
  if (isnan(fdm.ecef_pos.x)) { fdm.nan_count++; }
  if (isnan(fdm.ecef_pos.y)) { fdm.nan_count++; }
  if (isnan(fdm.ecef_pos.z)) { fdm.nan_count++; }
  if (isnan(fdm.ltpprz_pos.x)) { fdm.nan_count++; }
  if (isnan(fdm.ltpprz_pos.y)) { fdm.nan_count++; }
  if (isnan(fdm.ltpprz_pos.z)) { fdm.nan_count++; }
  if (isnan(fdm.lla_pos.lon)) { fdm.nan_count++; }
  if (isnan(fdm.lla_pos.lat)) { fdm.nan_count++; }
  if (isnan(fdm.lla_pos.alt)) { fdm.nan_count++; }
  if (isnan(fdm.hmsl)) { fdm.nan_count++; }
  // Skip debugging elements
  if (isnan(fdm.ecef_ecef_vel.x)) { fdm.nan_count++; }
  if (isnan(fdm.ecef_ecef_vel.y)) { fdm.nan_count++; }
  if (isnan(fdm.ecef_ecef_vel.z)) { fdm.nan_count++; }
  if (isnan(fdm.ecef_ecef_accel.x)) { fdm.nan_count++; }
  if (isnan(fdm.ecef_ecef_accel.y)) { fdm.nan_count++; }
  if (isnan(fdm.ecef_ecef_accel.z)) { fdm.nan_count++; }
  if (isnan(fdm.body_ecef_vel.x)) { fdm.nan_count++; }
  if (isnan(fdm.body_ecef_vel.y)) { fdm.nan_count++; }
  if (isnan(fdm.body_ecef_vel.z)) { fdm.nan_count++; }
  if (isnan(fdm.body_ecef_accel.x)) { fdm.nan_count++; }
  if (isnan(fdm.body_ecef_accel.y)) { fdm.nan_count++; }
  if (isnan(fdm.body_ecef_accel.z)) { fdm.nan_count++; }
  if (isnan(fdm.ltp_ecef_vel.x)) { fdm.nan_count++; }
  if (isnan(fdm.ltp_ecef_vel.y)) { fdm.nan_count++; }
  if (isnan(fdm.ltp_ecef_vel.z)) { fdm.nan_count++; }
  if (isnan(fdm.ltp_ecef_accel.x)) { fdm.nan_count++; }
  if (isnan(fdm.ltp_ecef_accel.y)) { fdm.nan_count++; }
  if (isnan(fdm.ltp_ecef_accel.z)) { fdm.nan_count++; }
  if (isnan(fdm.ltpprz_ecef_vel.x)) { fdm.nan_count++; }
  if (isnan(fdm.ltpprz_ecef_vel.y)) { fdm.nan_count++; }
  if (isnan(fdm.ltpprz_ecef_vel.z)) { fdm.nan_count++; }
  if (isnan(fdm.ltpprz_ecef_accel.x)) { fdm.nan_count++; }
  if (isnan(fdm.ltpprz_ecef_accel.y)) { fdm.nan_count++; }
  if (isnan(fdm.ltpprz_ecef_accel.z)) { fdm.nan_count++; }
  if (isnan(fdm.ecef_to_body_quat.qi)) { fdm.nan_count++; }
  if (isnan(fdm.ecef_to_body_quat.qx)) { fdm.nan_count++; }
  if (isnan(fdm.ecef_to_body_quat.qy)) { fdm.nan_count++; }
  if (isnan(fdm.ecef_to_body_quat.qz)) { fdm.nan_count++; }
  if (isnan(fdm.ltp_to_body_quat.qi)) { fdm.nan_count++; }
  if (isnan(fdm.ltp_to_body_quat.qx)) { fdm.nan_count++; }
  if (isnan(fdm.ltp_to_body_quat.qy)) { fdm.nan_count++; }
  if (isnan(fdm.ltp_to_body_quat.qz)) { fdm.nan_count++; }
  if (isnan(fdm.ltp_to_body_eulers.phi)) { fdm.nan_count++; }
  if (isnan(fdm.ltp_to_body_eulers.theta)) { fdm.nan_count++; }
  if (isnan(fdm.ltp_to_body_eulers.psi)) { fdm.nan_count++; }
  if (isnan(fdm.ltpprz_to_body_quat.qi)) { fdm.nan_count++; }
  if (isnan(fdm.ltpprz_to_body_quat.qx)) { fdm.nan_count++; }
  if (isnan(fdm.ltpprz_to_body_quat.qy)) { fdm.nan_count++; }
  if (isnan(fdm.ltpprz_to_body_quat.qz)) { fdm.nan_count++; }
  if (isnan(fdm.ltpprz_to_body_eulers.phi)) { fdm.nan_count++; }
  if (isnan(fdm.ltpprz_to_body_eulers.theta)) { fdm.nan_count++; }
  if (isnan(fdm.ltpprz_to_body_eulers.psi)) { fdm.nan_count++; }
  if (isnan(fdm.body_ecef_rotvel.p)) { fdm.nan_count++; }
  if (isnan(fdm.body_ecef_rotvel.q)) { fdm.nan_count++; }
  if (isnan(fdm.body_ecef_rotvel.r)) { fdm.nan_count++; }
  if (isnan(fdm.body_ecef_rotaccel.p)) { fdm.nan_count++; }
  if (isnan(fdm.body_ecef_rotaccel.q)) { fdm.nan_count++; }
  if (isnan(fdm.body_ecef_rotaccel.r)) { fdm.nan_count++; }
  if (isnan(fdm.ltp_g.x)) { fdm.nan_count++; }
  if (isnan(fdm.ltp_g.y)) { fdm.nan_count++; }
  if (isnan(fdm.ltp_g.z)) { fdm.nan_count++; }
  if (isnan(fdm.ltp_h.x)) { fdm.nan_count++; }
  if (isnan(fdm.ltp_h.y)) { fdm.nan_count++; }
  if (isnan(fdm.ltp_h.z)) { fdm.nan_count++; }

  return (fdm.nan_count - orig_nan_count);
}


