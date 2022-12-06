/*
 * Copyright (C) 2021 Jesús Bautista <jesusbautistavillar@gmail.com> 
 *                    Hector García  <noeth3r@gmail.com>
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

#include "simulink/darko/Darko/darko_grt_rtw/darko.h"

#include "state.h"


// NpsFdm structure
struct NpsFdm fdm;

// Reference point
static struct LtpDef_d ltpdef;

// Static functions declaration
static void init_ltp(void);

static void fetch_pos(void);
static void fetch_vel(void);

void nps_fdm_init(double dt)
{
  init_ltp();
  for(int i=0; i<4; i++) {
    rtU.u[i] = 0;
  }
  for(int i=0; i<3; i++) {
    rtU.w[i] = 0;
  }
  darko_initialize();

  darko_step();  
  fetch_pos();
}

void nps_fdm_run_step(bool launch __attribute__((unused)), double *commands, int commands_nb __attribute__((unused)))
{ 
  if(commands_nb != 4) {exit(-45);}
  
  for(int i=0; i<commands_nb; i++) {
    rtU.u[i] = commands[i];
  }
  
  darko_step();
  
  fetch_pos();
  
  printf("run: ");
  for(int i=0; i<commands_nb; i++) {
    printf("%lf  ", commands[i]);
  }
  printf("\n");

  

}

static void fetch_pos() {
  // fetch NED pos
  fdm.ltpprz_pos.x = rtY.p[0];
  fdm.ltpprz_pos.y = rtY.p[1];
  fdm.ltpprz_pos.z = rtY.p[2];
  
  // convert to ECEF and LLA
  ecef_of_ned_vect_d(&fdm.ecef_pos, &ltpdef, &fdm.ltpprz_pos);
  lla_of_ecef_d(&fdm.lla_pos, &fdm.ecef_pos);
  
  fdm.hmsl = fdm.lla_pos.alt; // TODO difference geoid/ellipsoid, terrain altitude ???
  // lla_pos_pprz
  // lla_pos_geod
  // lla_pos_geoc
  fdm.agl = fdm.hmsl; // TODO
}

static void fetch_vel() {

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

  ltp_def_from_ecef_d(&ltpdef, &ecef_nav0);
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
  rtU.w[0] = wind_north;
  rtU.w[1] = wind_east;
  rtU.w[2] = wind_down;
}

void nps_fdm_set_turbulence(double wind_speed __attribute__((unused)),
                            int turbulence_severity __attribute__((unused)))
{
}

void nps_fdm_set_temperature(double temp __attribute__((unused)),
                             double h __attribute__((unused)))
{
}

