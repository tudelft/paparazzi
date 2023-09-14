/*
 * Copyright (C) 2009 Antoine Drouin <poinix@gmail.com>
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
 * @file test_alloc.c
 *
 * Test routine for WLS (weighted least squares) control allocation
 *
 * Comparing a precomputed solution from Matlab with the one coming
 * from our PPRZ implementation.
 */

#include <stdio.h>
#include "std.h"
//#include "firmwares/rotorcraft/stabilization/wls/wls_alloc.h"
#include "firmwares/rotorcraft/oneloop/wls/wls_alloc.h"

#define INDI_OUTPUTS 6

void test_overdetermined(void);
void calc_nu_out(float** Bwls, float* du, float* nu_out);

int main(int argc, char **argv)
{
#define INDI_NUM_ACT 6
  test_overdetermined();
/*#define INDI_NUM_ACT 4*/
  /*test_four_by_four();*/
}

/*
 * function to test wls with 4x4 (outputs x inputs) system
 */
// void test_four_by_four(void)
// {
//   float u_min[INDI_NUM_ACT] = { -107, -19093, 0, -95, };
//   float u_max[INDI_NUM_ACT] = {19093, 107, 4600, 4505, };

//   float g1g2[INDI_OUTPUTS][INDI_NUM_ACT] = {
//     {      0,         0,  -0.0105,  0.0107016},
//     { -0.0030044, 0.0030044, 0.035, 0.035},
//     { -0.004856, -0.004856, 0, 0},
//     {       0,         0,   -0.0011,   -0.0011}
//   };

//   //State prioritization {W Roll, W pitch, W yaw, TOTAL THRUST}
//   static float Wv[INDI_OUTPUTS] = {100, 1000, 0.1, 10};
//   /*static float Wv[INDI_OUTPUTS] = {10, 10, 0.1, 1};*/

//   // The control objective in array format
//   float indi_v[INDI_OUTPUTS] = {10.8487,  -10.5658,    6.8383,    1.8532};
//   float indi_du[INDI_NUM_ACT];

//   // Initialize the array of pointers to the rows of g1g2
//   float *Bwls[INDI_OUTPUTS];
//   uint8_t i;
//   for (i = 0; i < INDI_OUTPUTS; i++) {
//     Bwls[i] = g1g2[i];
//   }

//   // WLS Control Allocator
//   int num_iter =
//     wls_alloc(indi_du, indi_v, u_min, u_max, Bwls, 0, 0, Wv, 0, 0, 10000, 10);

//   printf("finished in %d iterations\n", num_iter);
//   printf("du = %f, %f, %f, %f\n", indi_du[0], indi_du[1], indi_du[2], indi_du[3]);
// }


/*
 * function to test wls for an overdetermined 4x6 (outputs x inputs) system
 */
void test_overdetermined(void)
{
  //float u_min[INDI_NUM_ACT] = {0};
  //float u_max[INDI_NUM_ACT] = {0};

  float du_min[INDI_NUM_ACT] = { -0.5957,-0.5872,-0.6169,-0.5955,-1.6878,-1.1996};
  float du_max[INDI_NUM_ACT] = {0.4043,0.4128,0.3831,0.4045,0.3122,0.8004};
  float du_pref[INDI_NUM_ACT] = { -0.5957,-0.5872,-0.6169,-0.5955,-0.6878,-0.1996};

  //float u_p[INDI_NUM_ACT] = {0};

  float u_c[INDI_NUM_ACT] = {9600-5288.787109, 9600-1886.305664, 9600-5288.282227, 9600-1886.315430, 0.0, 0.0};

  printf("lower and upper bounds for du:\n");

  uint8_t k;
  for(k=0; k<INDI_NUM_ACT; k++) {
    //u_max[k] = 9600 - u_min[k];

    //du_min[k] = u_min[k] - u_c[k];
    //du_max[k] = u_max[k] - u_c[k];

    //u_p[k] = du_min[k];

    printf("%f ", du_min[k]);
    printf("%f \n", du_max[k]);
  }

  printf("\n");

  float g1g2[INDI_OUTPUTS][INDI_NUM_ACT] = {
    {-0.0261e3, -0.0261e3, -0.0261e3, -0.0261e3, 0.0013e3, -0.0151e3},
    {0.1027e3,  0.1027e3,  0.1027e3,  0.1027e3,  0.0153e3, -0.0001e3},
    {-0.1647e3, -0.1647e3, -0.1647e3, -0.1647e3, 0.0093e3, 0.0023e3},
    {4.7232e3,  -4.7232e3, -4.7232e3, 4.7232e3,  0.0,      0.0},
    {4.1472e3,  4.1472e3,  -4.1472e3, -4.1472e3, 0.0,      0.0},
    {0.2496e3,  -0.2496e3, 0.2496e3,  -0.2496e3, 0.0,      0.0}
  };

  //State prioritization {W Roll, W pitch, W yaw, TOTAL THRUST}
  static float Wv[INDI_OUTPUTS] = {1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 100.0};
  static float Wu[INDI_NUM_ACT] = {2.0,2.0,2.0,2.0,2.0,2.0};
  // The control objective in array format
  float indi_v[INDI_OUTPUTS] = {7.3244,9.0048,5.1520,6.2520,13.8888,-3.3358};
  float indi_du[INDI_NUM_ACT] = {95.290405,-570.446533,95.291260,-570.445801,0.0,0.0};

  // Initialize the array of pointers to the rows of g1g2
  float *Bwls[INDI_OUTPUTS];
  uint8_t i;
  for (i = 0; i < INDI_OUTPUTS; i++) {
    Bwls[i] = g1g2[i];
  }

  // WLS Control Allocator
  int num_iter =
    wls_alloc_oneloop(indi_du, indi_v, du_min, du_max, Bwls, 0, 0, Wv, Wu, du_pref, 1000.0, 10);

  printf("finished in %d iterations\n", num_iter);

  float nu_out[6] = {0.0f};
  calc_nu_out(Bwls, indi_du, nu_out);

  printf("du                 = %f, %f, %f, %f, %f, %f\n", indi_du[0], indi_du[1], indi_du[2], indi_du[3], indi_du[4], indi_du[5]);
  // Precomputed solution' in Matlab for this problem using lsqlin:
  printf("du (matlab_lsqlin) = %f, %f, %f, %f, %f, %f\n", -4614.0, 426.064612091305, 5390.0, -4614.0, -4210.0, 5390.0);
  printf("u = %f, %f, %f, %f, %f, %f\n", indi_du[0]+u_c[0], indi_du[1]+u_c[1], indi_du[2]+u_c[2], indi_du[3]+u_c[3], indi_du[4]+u_c[4], indi_du[5]+u_c[5]);
  printf("nu_in = %f, %f, %f, %f,  %f, %f\n", indi_v[0], indi_v[1], indi_v[2], indi_v[3], indi_v[4], indi_v[5]);
  printf("nu_out = %f, %f, %f, %f,  %f, %f\n", nu_out[0], nu_out[1], nu_out[2], nu_out[3], nu_out[4], nu_out[5]);
}

/*
 * Calculate the achieved control objective for some calculated control input
 */
void calc_nu_out(float** Bwls, float* du, float* nu_out) {

  for(int i=0; i<6; i++) {
    nu_out[i] = 0;
    for(int j=0; j<INDI_NUM_ACT; j++) {
      nu_out[i] += Bwls[i][j] * du[j];
    }
  }
}

