/*
 * Copyright (C) 2009 Antoine Drouin <poinix@gmail.com>, 2021 Dennis van Wijngaarden
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
 */

/**
 * @file test_alloc_hybrid.c
 *
 * Test routine for WLS (weighted least squares) control allocation
 *
 * Comparing a precomputed solution from Matlab with the one coming
 * from our PPRZ implementation.
 */

#include <stdio.h>
#include "std.h"
#include "firmwares/rotorcraft/guidance/wls/wls_alloc.h"

#define MAX_PPRZ 9600

void test_input_output_hybrid_indi(void);

int main(int argc, char **argv) {
  printf("Hello World\n");
  test_input_output_hybrid_indi();
}

void test_input_output_hybrid_indi(void) {
  printf("TEST INPUT/OUTPUT of WLS_ALLOC hybrid\n\n");

  // WLS implementation for Rot wing
  float Gmat[3][4];
  float du_min_hybrid[4];
  float du_max_hybrid[4];
  float du_pref_hybrid[4];
  float *Bwls_hybrid[3];
  float hybrid_du[4];
  float Wv_hybrid[3] = {1., 1., 1.};
  float Wu_hybrid[4] = {1000., 1000., 1., 1.};

  // Parameters to configure
  float thrust_bz_eff = -0.00136;
  float thrust_bx_eff = 0.0016;
  float hybrid_roll_limit = 0.785; // 45 deg
  float hybrid_pitch_limit = 0.349; // 20 deg

  // input data to configure
  float phi = 0.00000;
  float theta = 0.436;
  float psi = 0.00000;
  float thrust_bz = 4800.;
  float thrust_bx = 9600.;
  float sp_accel_ned[3] = {0.0, 0.0, -1.5};
  float accel_ned[3] = {0., 0., 0.};

  // Point BWls to Gmat
  uint8_t i;
  for (i = 0; i < 3; i++) {
    Bwls_hybrid[i] = Gmat[i];
  }

  /*
  Perform calculations
  */
  //Pre-calculate sines and cosines
  float sphi = sinf(phi);
  float cphi = cosf(phi);
  float stheta = sinf(theta);
  float ctheta = cosf(theta);
  float spsi = sinf(psi);
  float cpsi = cosf(psi);

  // Fill in Gmat array
  float lift_thrust_bz = -9.81;
  float accel_bx_sp = cpsi * sp_accel_ned[0] + spsi * sp_accel_ned[1];
  float accel_bx = cpsi * accel_ned[0] + spsi * accel_ned[1];
  float accel_bx_err = accel_bx_sp - accel_bx;

  Gmat[0][0] = cphi*spsi*lift_thrust_bz;
  Gmat[1][0] = -cphi*cpsi*lift_thrust_bz;
  Gmat[2][0] = -sphi*lift_thrust_bz;

  Gmat[0][1] = (ctheta*cpsi - sphi*stheta*spsi)*lift_thrust_bz;
  Gmat[1][1] = (ctheta*spsi + sphi*stheta*cpsi)*lift_thrust_bz;
  Gmat[2][1] = -cphi*stheta*lift_thrust_bz;

  Gmat[0][2] = stheta*cpsi + sphi*ctheta*spsi;
  Gmat[1][2] = stheta*spsi - sphi*ctheta*cpsi;
  Gmat[2][2] = cphi*ctheta;

  Gmat[0][3] = ctheta*cpsi - sphi*stheta*spsi;
  Gmat[1][3] = ctheta*spsi + sphi*stheta*cpsi;
  Gmat[2][3] = -cphi*stheta;

  // Set lower limits
  du_min_hybrid[0] = -hybrid_roll_limit - phi;
  du_min_hybrid[1] = -hybrid_pitch_limit - theta;
  du_min_hybrid[2] = (MAX_PPRZ - thrust_bz) * thrust_bz_eff; // PPRZ cmd (positive) * effectiveness (negative) ERROR!!!!!
  du_min_hybrid[3] = -thrust_bx * thrust_bx_eff;

  printf("du_min_roll: %f\ndu_min_pitch: %f\ndu_min_Tbz: %f\ndu_min_Tbx: %f\n\n", 
          du_min_hybrid[0], du_min_hybrid[1], du_min_hybrid[2], du_min_hybrid[3]);

  // Set upper limits
  du_max_hybrid[0] = hybrid_roll_limit - phi;
  du_max_hybrid[1] = hybrid_pitch_limit - theta;
  du_max_hybrid[2] = -thrust_bz * thrust_bz_eff;
  du_max_hybrid[3] = (MAX_PPRZ - thrust_bx) * thrust_bx_eff;

  printf("du_max_roll: %f\ndu_max_pitch: %f\ndu_max_Tbz: %f\ndu_max_Tbx: %f\n\n", 
          du_max_hybrid[0], du_max_hybrid[1], du_max_hybrid[2], du_max_hybrid[3]);

  // Set prefered states
  du_pref_hybrid[0] = -phi;
  du_pref_hybrid[1] = -theta;
  du_pref_hybrid[2] = 0.0;
  du_pref_hybrid[3] = 0.0;//9.81 * stheta; 

  printf("du_pref_roll: %f\ndu_pref_pitch: %f\ndu_pref_Tbz: %f\ndu_pref_Tbx: %f\n\n", 
          du_pref_hybrid[0], du_pref_hybrid[1], du_pref_hybrid[2], du_pref_hybrid[3]);

  // Perform WLS
  int num_iter = 
    wls_alloc_hybrid(hybrid_du, sp_accel_ned, du_min_hybrid, du_max_hybrid, Bwls_hybrid,
    0, 0, Wv_hybrid, Wu_hybrid, du_pref_hybrid, 10000, 10);

  printf("num_iter: %i\n\n", num_iter);

  float du_roll_out = hybrid_du[0];//; / 3.14159 * 180.; // Deg
  float du_pitch_out = hybrid_du[1];//; / 3.14159 * 180.; // Deg
  float du_Tbz_out = hybrid_du[2];
  float du_Tbx_out = hybrid_du[3];

  printf("du_roll_out: %f\ndu_pitch_out: %f\ndu_Tbz_out: %f\ndu_Tbx_out: %f\n",
              du_roll_out, du_pitch_out, du_Tbz_out, du_Tbx_out);
}

