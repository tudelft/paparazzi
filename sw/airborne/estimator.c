/*
 * Paparazzi autopilot $Id$
 *  
 * Copyright (C) 2004-2005 Pascal Brisset, Antoine Drouin
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
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA. 
 *
 */

/** \file estimator.c
 * \brief State estimate, fusioning sensors
 */

#include <inttypes.h>
#include <math.h>

#include "estimator.h"
#include "uart.h"
#include "ap_downlink.h"
#include "gps.h"
#include "nav.h"


/* position in meters */
float estimator_x;
float estimator_y;
float estimator_z;

float estimator_z_dot;

/* attitude in radian */
float estimator_phi;
float estimator_psi;
float estimator_theta;

/* rates in radians per second */
float estimator_p;
float estimator_q;

/* flight time in seconds */
uint16_t estimator_flight_time; 
/* flight time in seconds */
float estimator_t;

/* horizontal speed in module and dir */
float estimator_hspeed_mod;
float estimator_hspeed_dir;

/* wind */
float wind_east, wind_north;
float estimator_airspeed;

#define NORM_RAD_ANGLE2(x) { \
    while (x > 2 * M_PI) x -= 2 * M_PI; \
    while (x < 0 ) x += 2 * M_PI; \
  }


// FIXME maybe vz = -climb for NED??
#define EstimatorSetSpeedCart(vx, vy, vz) { \
  estimator_vx = vx; \
  estimator_vy = vy; \
  estimator_vz = vz; \
}
//  estimator_hspeed_mod = sqrt( estimator_vx * estimator_vx + estimator_vy * estimator_vy);
//  estimator_hspeed_dir = atan2(estimator_vy, estimator_vx);


//FIXME is this true ?? estimator_vx = estimator_hspeed_mod * cos(estimator_hspeed_dir);
//FIXME is this true ?? estimator_vy = estimator_hspeed_mod * sin(estimator_hspeed_dir);

void estimator_init( void ) {

  EstimatorSetPosXY(0., 0.);
  EstimatorSetAlt(0.);

  EstimatorSetAtt (0., 0., 0);

  EstimatorSetSpeedPol ( 0., 0., 0.);

  EstimatorSetRate(0., 0.);

  estimator_flight_time = 0;
}



void estimator_propagate_state( void ) {
  
}

bool_t alt_kalman_enabled;

#ifdef ALT_KALMAN

#ifndef ALT_KALMAN_ENABLED
#define ALT_KALMAN_ENABLED FALSE
#endif

#define GPS_SIGMA2 1.

#define GPS_DT 0.25
#define GPS_R 2.

#define BARO_DT 0.1

static float p[2][2];

void alt_kalman_reset( void ) {
  p[0][0] = 1.;
  p[0][1] = 0.;
  p[1][0] = 0.;
  p[1][1] = 1.;
}

void alt_kalman_init( void ) {
  alt_kalman_enabled = ALT_KALMAN_ENABLED;
  alt_kalman_reset();
}

void alt_kalman(float gps_z) {
  float DT;
  float R;
  float SIGMA2;

#ifdef USE_BARO_MS5534A
  if (alt_baro_enabled) {
    DT = BARO_DT;
    R = baro_MS5534A_r;
    SIGMA2 = baro_MS5534A_sigma2;
  } else
#endif
  {
    DT = GPS_DT;
    R = GPS_R;
    SIGMA2 = GPS_SIGMA2;
  }

  float q[2][2];
  q[0][0] = DT*DT*DT*DT/4.;
  q[0][1] = DT*DT*DT/2.;
  q[1][0] = DT*DT*DT/2.;
  q[1][1] = DT*DT;


  /* predict */
  estimator_z += estimator_z_dot * DT;
  p[0][0] = p[0][0]+p[1][0]*DT+DT*(p[0][1]+p[1][1]*DT) + SIGMA2*q[0][0];
  p[0][1] = p[0][1]+p[1][1]*DT + SIGMA2*q[0][1];
  p[1][0] = p[1][0]+p[1][1]*DT + SIGMA2*q[1][0];
  p[1][1] = p[1][1] + SIGMA2*q[1][1];

  /* error estimate */
  float e = p[0][0] + R;

  if (fabs(e) > 1e-5) {
    float k_0 = p[0][0] / e;
    float k_1 =  p[1][0] / e;
    e = gps_z - estimator_z;
    
    /* correction */
    estimator_z += k_0 * e;
    estimator_z_dot += k_1 * e;
    
    p[0][0] = p[0][0] * (1-k_0);
    p[0][1] = p[0][1] * (1-k_0);
    p[1][0] = -p[0][0]*k_1+p[1][0];
    p[1][1] = -p[0][1]*k_1+p[1][1];
  }

#ifdef DEBUG_ALT_KALMAN
  DOWNLINK_SEND_ALT_KALMAN(&(p[0][0]),&(p[0][1]),&(p[1][0]), &(p[1][1]));
#endif
}

#endif // ALT_KALMAN

void estimator_update_state_gps( void ) {
  float gps_east = gps_utm_east / 100.;
  float gps_north = gps_utm_north / 100.;

  /* Relative position to reference */
  gps_east -= nav_utm_east0;
  gps_north -= nav_utm_north0;

  float falt = gps_alt / 100.;
  EstimatorSetPosXY(gps_east, gps_north);
  EstimatorSetAlt(falt);
  float fspeed = gps_gspeed / 100.;
  float fclimb = gps_climb / 100.;
  float fcourse = RadOfDeg(gps_course / 10.);
  EstimatorSetSpeedPol(fspeed, fcourse, fclimb);

  // Heading estimator from wind-information, usually computed with -DWIND_INFO
  // wind_north and wind_east initialized to 0, so still correct if not updated
  float w_vn = cosf(estimator_hspeed_dir) * estimator_hspeed_mod - wind_north;
  float w_ve = sinf(estimator_hspeed_dir) * estimator_hspeed_mod - wind_east;
  estimator_psi = atan2f(w_ve, w_vn);
  if (estimator_psi < 0.)
    estimator_psi += 2 * M_PI;
}

