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


// NpsFdm structure
struct NpsFdm fdm;

// Reference point
static struct LtpDef_d ltpdef_d;

// Static functions declaration
static void init_ltp(void);

static void feed_cmd(double *commands, int commands_nb);

static void fetch_pos(void);
static void fetch_vel(void);
static void fetch_accel(void);
static void fetch_orient(void);
static void fetch_angular_vel(void);
static void fetch_rotaccel(void);

void nps_fdm_init(double dt)
{
  fdm.init_dt = dt;
  fdm.time = dt;
  //fdm.on_ground = TRUE;

  fdm.nan_count = 0;
  fdm.pressure = -1;
  fdm.pressure_sl = PPRZ_ISA_SEA_LEVEL_PRESSURE;
  fdm.total_pressure = -1;
  fdm.dynamic_pressure = -1;
  fdm.temperature = -1;

  init_ltp();

  
  rtU.u[0] = 1;
  rtU.u[1] = 1;
  rtU.u[2] = 0;
  rtU.u[3] = 0;
  

  for(int i=0; i<3; i++) {
    rtU.w[i] = 0;
  }
  darko_initialize();

/*
  darko_step(); 

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
*/
  //printf("init");
  
}

void nps_fdm_run_step(bool launch __attribute__((unused)), double *commands, int commands_nb __attribute__((unused)))
{ 
  feed_cmd(commands, commands_nb);

  //autopilot_in_flight()  autopilot.motors_on

  if(autopilot_in_flight()){
    darko_step();
  
  
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
  }

/*
  printf("run cmd: ");
  for(int i=0; i<commands_nb; i++) {
    printf("%lf  ", commands[i]);
  }
  printf("\n");
  
  printf("run pos: ");
  for(int i=0; i<3; i++) {
    printf("%lf  ", rtY.p[i]);
  }
  printf("\n");

  printf("run vel: ");
  for(int i=0; i<3; i++) {
    printf("%lf  ", rtY.v[i]);
  }
  printf("\n");


  printf("run quat: ");
  for(int i=0; i<4; i++) {
    printf("%lf  ", rtY.q[i]);
  }
  printf("\n");


  printf("run ome: ");
  for(int i=0; i<3; i++) {
    printf("%lf  ", rtY.omega[i]);
  }
  printf("\n");
  */

}

void feed_cmd(double *commands, int commands_nb __attribute__((unused))){
  if(commands_nb != 4) {exit(-45);}
  
  for(int i=0; i<commands_nb; i++) {
    rtU.u[i] = commands[i];
  }
  
}


static void fetch_pos() {
  // fetch NED pos
  fdm.ltpprz_pos.x = rtY.p[0];
  fdm.ltpprz_pos.y = rtY.p[1];
  fdm.ltpprz_pos.z = rtY.p[2];
  
  // convert to ECEF and LLA
  ecef_of_ned_point_d(&fdm.ecef_pos, &ltpdef_d, &fdm.ltpprz_pos);
  //lla_of_ecef_d(&fdm.lla_pos, &fdm.ecef_pos);
  
  fdm.hmsl = fdm.ltpprz_pos.z;
  // lla_pos_pprz
  // lla_pos_geod
  // lla_pos_geoc
  fdm.agl = fdm.hmsl; // TODO
}

static void fetch_vel() {
  fdm.ltpprz_ecef_vel.x = rtY.v[0];
  fdm.ltpprz_ecef_vel.y = rtY.v[1];
  fdm.ltpprz_ecef_vel.z = rtY.v[2];

  ecef_of_ned_vect_d(&fdm.ecef_ecef_vel, &ltpdef_d, &fdm.ltpprz_ecef_vel);
  //lla_of_ecef_d(&fdm.lla_pos, &fdm.ecef_ecef_vel);

}

static void fetch_accel(){
  fdm.ltpprz_ecef_accel.x = rtY.accel[0];
  fdm.ltpprz_ecef_accel.y = rtY.accel[1];
  fdm.ltpprz_ecef_accel.z = rtY.accel[2];
}

static void fetch_orient(){
  fdm.ltpprz_to_body_quat.qi = rtY.q[0];
  fdm.ltpprz_to_body_quat.qx = rtY.q[1];
  fdm.ltpprz_to_body_quat.qy = rtY.q[2];
  fdm.ltpprz_to_body_quat.qz = rtY.q[3];

}

//TODO check inertial ECI frame ou ECEF frame  fichier nps_fdm.h

static void fetch_angular_vel(){
  fdm.body_inertial_rotvel.p = rtY.omega[0];
  fdm.body_inertial_rotvel.q = rtY.omega[1];
  fdm.body_inertial_rotvel.r = rtY.omega[2];
}

static void fetch_rotaccel(){
  fdm.body_inertial_rotaccel.p = rtY.rotaccel[0];
  fdm.body_inertial_rotaccel.q = rtY.rotaccel[1];
  fdm.body_inertial_rotaccel.r = rtY.rotaccel[2];
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

