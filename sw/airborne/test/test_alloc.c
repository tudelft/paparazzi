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
#include "firmwares/rotorcraft/stabilization/wls/wls_alloc.h"
#include "math/qr_solve/qr_solve.h"
#include "math/qr_solve/r8lib_min.h"
#include "math/pprz_algebra_float.h"
#include "math/pprz_matrix_decomp_float.h"

#define INDI_OUTPUTS 6
#define INDI_NUM_ACT 12
void test_overdetermined(void);
void calc_nu_out(float** Bwls, float* du, float* nu_out);
void test_qr_solver_output(void);
void test_SVD_decomposition(void);
void test_pseudoinverse(void);
void Compute_commands_test(void);

int main(int argc, char **argv)
{
//  test_overdetermined();
/*#define INDI_NUM_ACT 4*/
  /*test_four_by_four();*/

 // test_qr_solver_output();
// test_SVD_decomposition();
//    test_pseudoinverse();
    Compute_commands_test();
}

/*
 * function to test wls with 4x4 (outputs x inputs) system
 */
void test_four_by_four(void)
{
  float u_min[INDI_NUM_ACT] = { -107, -19093, 0, -95, };
  float u_max[INDI_NUM_ACT] = {19093, 107, 4600, 4505, };

  float g1g2[INDI_OUTPUTS][INDI_NUM_ACT] = {
    {      0,         0,  -0.0105,  0.0107016},
    { -0.0030044, 0.0030044, 0.035, 0.035},
    { -0.004856, -0.004856, 0, 0},
    {       0,         0,   -0.0011,   -0.0011}
  };

  //State prioritization {W Roll, W pitch, W yaw, TOTAL THRUST}
  static float Wv[INDI_OUTPUTS] = {100, 1000, 0.1, 10};
  /*static float Wv[INDI_OUTPUTS] = {10, 10, 0.1, 1};*/

  // The control objective in array format
  float indi_v[INDI_OUTPUTS] = {10.8487,  -10.5658,    6.8383,    1.8532};
  float indi_du[INDI_NUM_ACT];

  // Initialize the array of pointers to the rows of g1g2
  float *Bwls[INDI_OUTPUTS];
  uint8_t i;
  for (i = 0; i < INDI_OUTPUTS; i++) {
    Bwls[i] = g1g2[i];
  }

  // WLS Control Allocator
  int num_iter =
    wls_alloc(indi_du, indi_v, u_min, u_max, Bwls, 0, 0, Wv, 0, 0, 10000, 10);

  printf("finished in %d iterations\n", num_iter);
  printf("du = %f, %f, %f, %f\n", indi_du[0], indi_du[1], indi_du[2], indi_du[3]);
}

void test_qr_solver_output(void)
{
    float B_matrix[INDI_OUTPUTS][INDI_NUM_ACT] = {
            {0, 0, 0, 0, -2.4525, -2.4525, -2.4525, -2.4525, 0, 0, 0, 0},
            {0, 0, 0, 0, 0, 0, 0, 0, 2.4525, 2.4525, 2.4525, 2.4525},
            {-0.0062, -0.0062, -0.0062, -0.0062, 0, 0, 0, 0, 0, 0, 0, 0},
            {0.0265, -0.0265, -0.0265, 0.0265, 0, 0, 0, 0, 0, 0, 0, 0},
            {0.0277, 0.0277, -0.0344, -0.0344, 0, 0, 0, 0, 0, 0, 0, 0},
            {0.0010, -0.0010, 0.0010, -0.0010, 0, 0, 0, 0, 8.1791, 8.1791, -10.1533, -10.1533},
    };


    // The qr solution

    float indi_v[INDI_OUTPUTS] = {0, 0, 0, 1, 0, 0};
    float indi_du[INDI_NUM_ACT];


    qr_solve(INDI_OUTPUTS,INDI_NUM_ACT,&B_matrix[0][0],&indi_v[0],&indi_du[0]);

    printf("du = %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n", indi_du[0], indi_du[1], indi_du[2], indi_du[3], indi_du[4], indi_du[5], indi_du[6], indi_du[7],indi_du[8], indi_du[9], indi_du[10], indi_du[11]);
}

void test_SVD_decomposition(void)
{
    float B_matrix_in[INDI_OUTPUTS][INDI_NUM_ACT] = {
            {0, 0, 0, 0, -2.4525, -2.4525, -2.4525, -2.4525, 0, 0, 0, 0},
            {0, 0, 0, 0, 0, 0, 0, 0, 2.4525, 2.4525, 2.4525, 2.4525},
            {-0.0062, -0.0062, -0.0062, -0.0062, 0, 0, 0, 0, 0, 0, 0, 0},
            {0.0265, -0.0265, -0.0265, 0.0265, 0, 0, 0, 0, 0, 0, 0, 0},
            {0.0277, 0.0277, -0.0344, -0.0344, 0, 0, 0, 0, 0, 0, 0, 0},
            {0.0010, -0.0010, 0.0010, -0.0010, 0, 0, 0, 0, 8.1791, 8.1791, -10.1533, -10.1533},
    };
    float B_matrix[INDI_OUTPUTS][INDI_NUM_ACT];
    memcpy(B_matrix, B_matrix_in, INDI_OUTPUTS * INDI_NUM_ACT * sizeof(float));

    float B_matrix_transposed[INDI_NUM_ACT][INDI_OUTPUTS];
    float * B_matrix_[INDI_NUM_ACT];

    //Transpose matrix B_in
    int i, j;
    for (i = 0; i < INDI_OUTPUTS; i++) {
        for (j = 0; j < INDI_NUM_ACT; j++) {
            B_matrix_transposed[j][i] = B_matrix[i][j];
            B_matrix_[j] = &B_matrix_transposed[j][0];
        }
    }

    float w_in[INDI_OUTPUTS];

    float v_in[INDI_OUTPUTS][INDI_OUTPUTS];
    float * v_in_[INDI_OUTPUTS];
    i = 0;
    for (i = 0; i < INDI_OUTPUTS; i++) {
        v_in_[i] = &v_in[i][0];
    }
    i = 0;
    j = 0;
//    MAKE_MATRIX_PTR(v_in_, v_in, INDI_OUTPUTS);

    pprz_svd_float(B_matrix_, &w_in[0], v_in_, INDI_NUM_ACT, INDI_OUTPUTS);

    //Print the U matrix
    printf("U = %f, %f, %f, %f, %f, %f\n", B_matrix_[0][0], B_matrix_[0][1],B_matrix_[0][2],B_matrix_[0][3],B_matrix_[0][4],B_matrix_[0][5]);
    for (i = 1; i < INDI_NUM_ACT; i++) {
        for (j = 0; j < INDI_OUTPUTS; j++) {
            printf("%f, ", B_matrix_[i][j]);
        }
        printf("\n");
    }
    i = 0;
    j = 0;

    //Print the diagonal matrix (/Sum)
    printf("diag = %f, %f, %f, %f, %f, %f\n", w_in[0], w_in[1], w_in[2], w_in[3], w_in[4], w_in[5]);

    //Print the V matrix
    printf("V = %f, %f, %f, %f, %f, %f\n", v_in_[0][0], v_in_[0][1],v_in_[0][2],v_in_[0][3],v_in_[0][4],v_in_[0][5]);
    for (i = 1; i < INDI_OUTPUTS; i++) {
        for (j = 0; j < INDI_OUTPUTS; j++) {
            printf("%f, ", v_in_[i][j]);
        }
        printf("\n");
    }
}

void test_pseudoinverse(void)
{
    float B_matrix_in[INDI_OUTPUTS][INDI_NUM_ACT] = {
            {0, 0, 0, 0, -2.4525, -2.4525, -2.4525, -2.4525, 0, 0, 0, 0},
            {0, 0, 0, 0, 0, 0, 0, 0, 2.4525, 2.4525, 2.4525, 2.4525},
            {-0.0062, -0.0062, -0.0062, -0.0062, 0, 0, 0, 0, 0, 0, 0, 0},
            {0.0265, -0.0265, -0.0265, 0.0265, 0, 0, 0, 0, 0, 0, 0, 0},
            {0.0277, 0.0277, -0.0344, -0.0344, 0, 0, 0, 0, 0, 0, 0, 0},
            {0.0010, -0.0010, 0.0010, -0.0010, 0, 0, 0, 0, 8.1791, 8.1791, -10.1533, -10.1533},
    };

    float B_matrix[INDI_OUTPUTS][INDI_NUM_ACT];
    memcpy(B_matrix, B_matrix_in, INDI_OUTPUTS * INDI_NUM_ACT * sizeof(float));

    float B_matrix_transposed[INDI_NUM_ACT][INDI_OUTPUTS];
    float * B_matrix_[INDI_NUM_ACT];

    //Transpose matrix B_in
    int i, j;
    for (i = 0; i < INDI_OUTPUTS; i++) {
        for (j = 0; j < INDI_NUM_ACT; j++) {
            B_matrix_transposed[j][i] = B_matrix[i][j];
            B_matrix_[j] = &B_matrix_transposed[j][0];
        }
    }

    float w_in[INDI_OUTPUTS];

    float v_in[INDI_OUTPUTS][INDI_OUTPUTS];
    float * v_in_[INDI_OUTPUTS];
    i = 0;
    for (i = 0; i < INDI_OUTPUTS; i++) {
        v_in_[i] = &v_in[i][0];
    }
    i = 0;
    j = 0;

    pprz_svd_float(B_matrix_, &w_in[0], v_in_, INDI_NUM_ACT, INDI_OUTPUTS);

    //Print the U matrix
    printf("U = \n");
    for (i = 0; i < INDI_NUM_ACT; i++) {
        for (j = 0; j < INDI_OUTPUTS; j++) {
            printf("%f, ", B_matrix_[i][j]);
        }
        printf("\n");
    }
    i = 0;
    j = 0;

    //Print the diagonal matrix (/Sum)
    printf("diag = \n%f, %f, %f, %f, %f, %f\n", w_in[0], w_in[1], w_in[2], w_in[3], w_in[4], w_in[5]);

    //Print the V matrix
    printf("V = \n");
    for (i = 0; i < INDI_OUTPUTS; i++) {
        for (j = 0; j < INDI_OUTPUTS; j++) {
            printf("%f, ", v_in_[i][j]);
        }
        printf("\n");
    }
    i = 0;
    j = 0;

    //Transpose matrix U
    float U_transposed[INDI_OUTPUTS][INDI_NUM_ACT];
    for (i = 0; i < INDI_NUM_ACT; i++) {
        for (j = 0; j < INDI_OUTPUTS; j++) {
            U_transposed[j][i] = B_matrix_[i][j];
        }
    }
    i = 0;
    j = 0;

    // Invert the diag values
    float w_inverted[INDI_OUTPUTS][INDI_OUTPUTS];
    for (i = 0; i < INDI_OUTPUTS; i++) {
        w_inverted[i][i] = 1/w_in[i];
    }
    i = 0;

    //Multiply the diagonal matrix with U_transposed
    float out_1[INDI_OUTPUTS][INDI_NUM_ACT];
    int k;
    for (i = 0; i < INDI_OUTPUTS; i++) {
        for (j = 0; j < INDI_NUM_ACT; j++) {
            out_1[i][j] = 0.;
            for (k = 0; k < INDI_OUTPUTS; k++) {
                out_1[i][j] += w_inverted[i][k] * U_transposed[k][j];
            }
        }
    }
    i = 0;
    j = 0;
    k = 0;

    //Multiply V with out_1
    float Pseudoinverse[INDI_OUTPUTS][INDI_NUM_ACT];
    for (i = 0; i < INDI_OUTPUTS; i++) {
        for (j = 0; j < INDI_NUM_ACT; j++) {
            Pseudoinverse[i][j] = 0.;
            for (k = 0; k < INDI_OUTPUTS; k++) {
                Pseudoinverse[i][j] += v_in_[i][k] * out_1[k][j];
            }
        }
    }
    i = 0;
    j = 0;
    k = 0;

    //Print the Pseudoinverse matrix
    printf("Pseudoinverse = \n");
    for (i = 0; i < INDI_OUTPUTS; i++) {
        for (j = 0; j < INDI_NUM_ACT; j++) {
            printf("%f, ", Pseudoinverse[i][j]);
        }
        printf("\n");
    }
    i = 0;
    j = 0;

}

//This function computes the simplified B matrix 6x12
void Compute_B_matrix_simple(float **B_matrix, float I_xx,float I_yy,float I_zz,float J_r,float l_1,float l_2,float l_3,float l_4,float l_z,float m,float K_p_T,float K_p_M,float *Omega_sens,float *b_sens,float *g_sens,float *Omega_dot_sens,float *b_dot_sens,float *g_dot_sens, float *Euler_rad, float *pqr_rad_s){
    //Transpose pointer to local variables:
    float Omega_1, Omega_2, Omega_3, Omega_4;
    Omega_1 = Omega_sens[0];
    Omega_2 = Omega_sens[1];
    Omega_3 = Omega_sens[2];
    Omega_4 = Omega_sens[3];
    float b_1, b_2, b_3, b_4, g_1, g_2, g_3, g_4;
    b_1 = b_sens[0];
    b_2 = b_sens[1];
    b_3 = b_sens[2];
    b_4 = b_sens[3];
    g_1 = g_sens[0];
    g_2 = g_sens[1];
    g_3 = g_sens[2];
    g_4 = g_sens[3];

    float Omega_1_dot, Omega_2_dot, Omega_3_dot, Omega_4_dot;
    Omega_1_dot = Omega_dot_sens[0];
    Omega_2_dot = Omega_dot_sens[1];
    Omega_3_dot = Omega_dot_sens[2];
    Omega_4_dot = Omega_dot_sens[3];
    float b_1_dot, b_2_dot, b_3_dot, b_4_dot, g_1_dot, g_2_dot, g_3_dot, g_4_dot;
    b_1_dot = b_dot_sens[0];
    b_2_dot = b_dot_sens[1];
    b_3_dot = b_dot_sens[2];
    b_4_dot = b_dot_sens[3];
    g_1_dot = g_dot_sens[0];
    g_2_dot = g_dot_sens[1];
    g_3_dot = g_dot_sens[2];
    g_4_dot = g_dot_sens[3];


    float Phi, Theta, Psi, p, q, r;
    Phi = Euler_rad[0];
    Theta = Euler_rad[1];
    Psi = Euler_rad[2];
    p = pqr_rad_s[0];
    q = pqr_rad_s[1];
    r = pqr_rad_s[2];

    //Motor disposition
    float l_1_x = -l_4;
    float l_2_x = -l_4;
    float l_3_x = l_3;
    float l_4_x = l_3;
    float l_1_y = l_1;
    float l_2_y = -l_1;
    float l_3_y = -l_2;
    float l_4_y = l_2;
    float l_1_z = l_z;
    float l_2_z = l_z;
    float l_3_z = l_z;
    float l_4_z = l_z;

    //Assumptions:
    float I_xx_tilt, I_yy_tilt;
    I_xx_tilt = 0;
    I_yy_tilt = 0;
    float b_1_ddot, b_2_ddot, b_3_ddot, b_4_ddot;
    float g_1_ddot, g_2_ddot, g_3_ddot, g_4_ddot;
    b_1_ddot = 0;
    b_2_ddot = 0;
    b_3_ddot = 0;
    b_4_ddot = 0;
    g_1_ddot = 0;
    g_2_ddot = 0;
    g_3_ddot = 0;
    g_4_ddot = 0;

    // First row
    B_matrix[0][0] = 0;
    B_matrix[0][1] = 0;
    B_matrix[0][2] = 0;
    B_matrix[0][3] = 0;

    B_matrix[0][4] = (K_p_T*Omega_1*Omega_1*cos(g_1)*sin(b_1)*(sin(Phi)*sin(Psi) + cos(Phi)*cos(Psi)*sin(Theta)) - K_p_T*Omega_1*Omega_1*cos(Psi)*cos(Theta)*cos(b_1) + K_p_T*Omega_1*Omega_1*sin(b_1)*sin(g_1)*(cos(Phi)*sin(Psi) - cos(Psi)*sin(Phi)*sin(Theta)))/m;
    B_matrix[0][5] = (K_p_T*Omega_2*Omega_2*cos(g_2)*sin(b_2)*(sin(Phi)*sin(Psi) + cos(Phi)*cos(Psi)*sin(Theta)) - K_p_T*Omega_2*Omega_2*cos(Psi)*cos(Theta)*cos(b_2) + K_p_T*Omega_2*Omega_2*sin(b_2)*sin(g_2)*(cos(Phi)*sin(Psi) - cos(Psi)*sin(Phi)*sin(Theta)))/m;
    B_matrix[0][6] = (K_p_T*Omega_3*Omega_3*cos(g_3)*sin(b_3)*(sin(Phi)*sin(Psi) + cos(Phi)*cos(Psi)*sin(Theta)) - K_p_T*Omega_3*Omega_3*cos(Psi)*cos(Theta)*cos(b_3) + K_p_T*Omega_3*Omega_3*sin(b_3)*sin(g_3)*(cos(Phi)*sin(Psi) - cos(Psi)*sin(Phi)*sin(Theta)))/m;
    B_matrix[0][7] = (K_p_T*Omega_4*Omega_4*cos(g_4)*sin(b_4)*(sin(Phi)*sin(Psi) + cos(Phi)*cos(Psi)*sin(Theta)) - K_p_T*Omega_4*Omega_4*cos(Psi)*cos(Theta)*cos(b_4) + K_p_T*Omega_4*Omega_4*sin(b_4)*sin(g_4)*(cos(Phi)*sin(Psi) - cos(Psi)*sin(Phi)*sin(Theta)))/m;

    B_matrix[0][8] = 0;
    B_matrix[0][9] = 0;
    B_matrix[0][10] = 0;
    B_matrix[0][11] = 0;

    // Second row
    B_matrix[1][0] = 0;
    B_matrix[1][1] = 0;
    B_matrix[1][2] = 0;
    B_matrix[1][3] = 0;

    B_matrix[1][4] = 0;
    B_matrix[1][5] = 0;
    B_matrix[1][6] = 0;
    B_matrix[1][7] = 0;

    B_matrix[1][8] = (K_p_T*Omega_1*Omega_1*cos(b_1)*cos(g_1)*(cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta)) - K_p_T*Omega_1*Omega_1*cos(b_1)*sin(g_1)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)))/m;
    B_matrix[1][9] = (K_p_T*Omega_2*Omega_2*cos(b_2)*cos(g_2)*(cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta)) - K_p_T*Omega_2*Omega_2*cos(b_2)*sin(g_2)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)))/m;
    B_matrix[1][10] = (K_p_T*Omega_3*Omega_3*cos(b_3)*cos(g_3)*(cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta)) - K_p_T*Omega_3*Omega_3*cos(b_3)*sin(g_3)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)))/m;
    B_matrix[1][11] = (K_p_T*Omega_4*Omega_4*cos(b_4)*cos(g_4)*(cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta)) - K_p_T*Omega_4*Omega_4*cos(b_4)*sin(g_4)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)))/m;

    // Third row
    B_matrix[2][0] = (2*K_p_T*Omega_1*sin(b_1)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)) + 2*K_p_T*Omega_1*cos(Theta)*sin(Phi)*cos(b_1)*sin(g_1) - 2*K_p_T*Omega_1*cos(Phi)*cos(Theta)*cos(b_1)*cos(g_1))/m;
    B_matrix[2][1] = (2*K_p_T*Omega_2*sin(b_2)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)) + 2*K_p_T*Omega_2*cos(Theta)*sin(Phi)*cos(b_2)*sin(g_2) - 2*K_p_T*Omega_2*cos(Phi)*cos(Theta)*cos(b_2)*cos(g_2))/m;
    B_matrix[2][2] = (2*K_p_T*Omega_3*sin(b_3)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)) + 2*K_p_T*Omega_3*cos(Theta)*sin(Phi)*cos(b_3)*sin(g_3) - 2*K_p_T*Omega_3*cos(Phi)*cos(Theta)*cos(b_3)*cos(g_3))/m;
    B_matrix[2][3] = (2*K_p_T*Omega_4*sin(b_4)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)) + 2*K_p_T*Omega_4*cos(Theta)*sin(Phi)*cos(b_4)*sin(g_4) - 2*K_p_T*Omega_4*cos(Phi)*cos(Theta)*cos(b_4)*cos(g_4))/m;

    B_matrix[2][4] = 0;
    B_matrix[2][5] = 0;
    B_matrix[2][6] = 0;
    B_matrix[2][7] = 0;

    B_matrix[2][8] = 0;
    B_matrix[2][9] = 0;
    B_matrix[2][10] = 0;
    B_matrix[2][11] = 0;

    // Fourth row
    B_matrix[3][0] = (2*K_p_M*Omega_1*sin(b_1) + J_r*b_1_dot*cos(b_1) + J_r*q*cos(b_1)*cos(g_1) + J_r*r*cos(b_1)*sin(g_1) + 2*K_p_T*Omega_1*l_1_z*cos(b_1)*sin(g_1) + 2*K_p_T*Omega_1*l_1_y*cos(b_1)*cos(g_1))/I_xx;
    B_matrix[3][1] = -(2*K_p_M*Omega_2*sin(b_2) + J_r*b_2_dot*cos(b_2) + J_r*q*cos(b_2)*cos(g_2) + J_r*r*cos(b_2)*sin(g_2) - 2*K_p_T*Omega_2*l_2_z*cos(b_2)*sin(g_2) - 2*K_p_T*Omega_2*l_2_y*cos(b_2)*cos(g_2))/I_xx;
    B_matrix[3][2] = (2*K_p_M*Omega_3*sin(b_3) + J_r*b_3_dot*cos(b_3) + J_r*q*cos(b_3)*cos(g_3) + J_r*r*cos(b_3)*sin(g_3) + 2*K_p_T*Omega_3*l_3_z*cos(b_3)*sin(g_3) + 2*K_p_T*Omega_3*l_3_y*cos(b_3)*cos(g_3))/I_xx;
    B_matrix[3][3] = -(2*K_p_M*Omega_4*sin(b_4) + J_r*b_4_dot*cos(b_4) + J_r*q*cos(b_4)*cos(g_4) + J_r*r*cos(b_4)*sin(g_4) - 2*K_p_T*Omega_4*l_4_z*cos(b_4)*sin(g_4) - 2*K_p_T*Omega_4*l_4_y*cos(b_4)*cos(g_4))/I_xx;

    B_matrix[3][4] = 0;
    B_matrix[3][5] = 0;
    B_matrix[3][6] = 0;
    B_matrix[3][7] = 0;

    B_matrix[3][8] = 0;
    B_matrix[3][9] = 0;
    B_matrix[3][10] = 0;
    B_matrix[3][11] = 0;

    // Fifth row
    B_matrix[4][0] = -(J_r*g_1_dot*cos(g_1) - J_r*r*sin(b_1) - 2*K_p_T*Omega_1*l_1_z*sin(b_1) + 2*K_p_M*Omega_1*cos(b_1)*sin(g_1) + J_r*p*cos(b_1)*cos(g_1) - J_r*b_1_dot*sin(b_1)*sin(g_1) + 2*K_p_T*Omega_1*l_1_x*cos(b_1)*cos(g_1))/I_yy;
    B_matrix[4][1] = (J_r*g_2_dot*cos(g_2) - J_r*r*sin(b_2) + 2*K_p_T*Omega_2*l_2_z*sin(b_2) + 2*K_p_M*Omega_2*cos(b_2)*sin(g_2) + J_r*p*cos(b_2)*cos(g_2) - J_r*b_2_dot*sin(b_2)*sin(g_2) - 2*K_p_T*Omega_2*l_2_x*cos(b_2)*cos(g_2))/I_yy;
    B_matrix[4][2] = -(J_r*g_3_dot*cos(g_3) - J_r*r*sin(b_3) - 2*K_p_T*Omega_3*l_3_z*sin(b_3) + 2*K_p_M*Omega_3*cos(b_3)*sin(g_3) + J_r*p*cos(b_3)*cos(g_3) - J_r*b_3_dot*sin(b_3)*sin(g_3) + 2*K_p_T*Omega_3*l_3_x*cos(b_3)*cos(g_3))/I_yy;
    B_matrix[4][3] = (J_r*g_4_dot*cos(g_4) - J_r*r*sin(b_4) + 2*K_p_T*Omega_4*l_4_z*sin(b_4) + 2*K_p_M*Omega_4*cos(b_4)*sin(g_4) + J_r*p*cos(b_4)*cos(g_4) - J_r*b_4_dot*sin(b_4)*sin(g_4) - 2*K_p_T*Omega_4*l_4_x*cos(b_4)*cos(g_4))/I_yy;

    B_matrix[4][4] = 0;
    B_matrix[4][5] = 0;
    B_matrix[4][6] = 0;
    B_matrix[4][7] = 0;

    B_matrix[4][8] = 0;
    B_matrix[4][9] = 0;
    B_matrix[4][10] = 0;
    B_matrix[4][11] = 0;

    // Sixth row
    B_matrix[5][0] = -(J_r*g_1_dot*sin(g_1) + J_r*q*sin(b_1) + 2*K_p_T*Omega_1*l_1_y*sin(b_1) - 2*K_p_M*Omega_1*cos(b_1)*cos(g_1) + J_r*b_1_dot*cos(g_1)*sin(b_1) + J_r*p*cos(b_1)*sin(g_1) + 2*K_p_T*Omega_1*l_1_x*cos(b_1)*sin(g_1))/I_zz;
    B_matrix[5][1] = (J_r*g_2_dot*sin(g_2) + J_r*q*sin(b_2) - 2*K_p_T*Omega_2*l_2_y*sin(b_2) - 2*K_p_M*Omega_2*cos(b_2)*cos(g_2) + J_r*b_2_dot*cos(g_2)*sin(b_2) + J_r*p*cos(b_2)*sin(g_2) - 2*K_p_T*Omega_2*l_2_x*cos(b_2)*sin(g_2))/I_zz;
    B_matrix[5][2] = -(J_r*g_3_dot*sin(g_3) + J_r*q*sin(b_3) + 2*K_p_T*Omega_3*l_3_y*sin(b_3) - 2*K_p_M*Omega_3*cos(b_3)*cos(g_3) + J_r*b_3_dot*cos(g_3)*sin(b_3) + J_r*p*cos(b_3)*sin(g_3) + 2*K_p_T*Omega_3*l_3_x*cos(b_3)*sin(g_3))/I_zz;
    B_matrix[5][3] = (J_r*g_4_dot*sin(g_4) + J_r*q*sin(b_4) - 2*K_p_T*Omega_4*l_4_y*sin(b_4) - 2*K_p_M*Omega_4*cos(b_4)*cos(g_4) + J_r*b_4_dot*cos(g_4)*sin(b_4) + J_r*p*cos(b_4)*sin(g_4) - 2*K_p_T*Omega_4*l_4_x*cos(b_4)*sin(g_4))/I_zz;

//    B_matrix[5][4] = -(J_r*Omega_1*q*cos(b_1) + K_p_T*Omega_1^2*l_1_y*cos(b_1) + J_r*Omega_1_dot*cos(g_1)*sin(b_1) + I_xx_tilt*g_1_ddot*cos(b_1)*cos(g_1) + K_p_M*Omega_1^2*cos(g_1)*sin(b_1) - J_r*Omega_1*p*sin(b_1)*sin(g_1) - K_p_T*Omega_1^2*l_1_x*sin(b_1)*sin(g_1) + J_r*Omega_1*b_1_dot*cos(b_1)*cos(g_1))/I_zz;
//    B_matrix[5][5] = (J_r*Omega_2*q*cos(b_2) - K_p_T*Omega_2^2*l_2_y*cos(b_2) - J_r*Omega_2_dot*cos(g_2)*sin(b_2) - I_xx_tilt*g_2_ddot*cos(b_2)*cos(g_2) + K_p_M*Omega_2^2*cos(g_2)*sin(b_2) - J_r*Omega_2*p*sin(b_2)*sin(g_2) + K_p_T*Omega_2^2*l_2_x*sin(b_2)*sin(g_2) + J_r*Omega_2*b_2_dot*cos(b_2)*cos(g_2))/I_zz;
//    B_matrix[5][6] = -(J_r*Omega_3*q*cos(b_3) + K_p_T*Omega_3^2*l_3_y*cos(b_3) + J_r*Omega_3_dot*cos(g_3)*sin(b_3) + I_xx_tilt*g_3_ddot*cos(b_3)*cos(g_3) + K_p_M*Omega_3^2*cos(g_3)*sin(b_3) - J_r*Omega_3*p*sin(b_3)*sin(g_3) - K_p_T*Omega_3^2*l_3_x*sin(b_3)*sin(g_3) + J_r*Omega_3*b_3_dot*cos(b_3)*cos(g_3))/I_zz;
//    B_matrix[5][7] = (J_r*Omega_4*q*cos(b_4) - K_p_T*Omega_4^2*l_4_y*cos(b_4) - J_r*Omega_4_dot*cos(g_4)*sin(b_4) - I_xx_tilt*g_4_ddot*cos(b_4)*cos(g_4) + K_p_M*Omega_4^2*cos(g_4)*sin(b_4) - J_r*Omega_4*p*sin(b_4)*sin(g_4) + K_p_T*Omega_4^2*l_4_x*sin(b_4)*sin(g_4) + J_r*Omega_4*b_4_dot*cos(b_4)*cos(g_4))/I_zz;

    B_matrix[5][4] = 0;
    B_matrix[5][5] = 0;
    B_matrix[5][6] = 0;
    B_matrix[5][7] = 0;

    B_matrix[5][8] = -(J_r*Omega_1*g_1_dot*cos(g_1) - I_yy_tilt*b_1_ddot*cos(g_1) + J_r*Omega_1_dot*cos(b_1)*sin(g_1) - I_xx_tilt*g_1_ddot*sin(b_1)*sin(g_1) + K_p_M*Omega_1*Omega_1*cos(b_1)*sin(g_1) - J_r*Omega_1*b_1_dot*sin(b_1)*sin(g_1) + K_p_T*Omega_1*Omega_1*l_1_x*cos(b_1)*cos(g_1) + J_r*Omega_1*p*cos(b_1)*cos(g_1))/I_zz;
    B_matrix[5][9] = (I_yy_tilt*b_2_ddot*cos(g_2) + J_r*Omega_2*g_2_dot*cos(g_2) - J_r*Omega_2_dot*cos(b_2)*sin(g_2) + I_xx_tilt*g_2_ddot*sin(b_2)*sin(g_2) + K_p_M*Omega_2*Omega_2*cos(b_2)*sin(g_2) - J_r*Omega_2*b_2_dot*sin(b_2)*sin(g_2) - K_p_T*Omega_2*Omega_2*l_2_x*cos(b_2)*cos(g_2) + J_r*Omega_2*p*cos(b_2)*cos(g_2))/I_zz;
    B_matrix[5][10] = -(J_r*Omega_3*g_3_dot*cos(g_3) - I_yy_tilt*b_3_ddot*cos(g_3) + J_r*Omega_3_dot*cos(b_3)*sin(g_3) - I_xx_tilt*g_3_ddot*sin(b_3)*sin(g_3) + K_p_M*Omega_3*Omega_3*cos(b_3)*sin(g_3) - J_r*Omega_3*b_3_dot*sin(b_3)*sin(g_3) + K_p_T*Omega_3*Omega_3*l_3_x*cos(b_3)*cos(g_3) + J_r*Omega_3*p*cos(b_3)*cos(g_3))/I_zz;
    B_matrix[5][11] = (I_yy_tilt*b_4_ddot*cos(g_4) + J_r*Omega_4*g_4_dot*cos(g_4) - J_r*Omega_4_dot*cos(b_4)*sin(g_4) + I_xx_tilt*g_4_ddot*sin(b_4)*sin(g_4) + K_p_M*Omega_4*Omega_4*cos(b_4)*sin(g_4) - J_r*Omega_4*b_4_dot*sin(b_4)*sin(g_4) - K_p_T*Omega_4*Omega_4*l_4_x*cos(b_4)*cos(g_4) + J_r*Omega_4*p*cos(b_4)*cos(g_4))/I_zz;

//    //Print the B matrix
//    printf("B_matrix = \n");
//    int i = 0;
//    int j = 0;
//    for (i = 0; i < INDI_OUTPUTS ; i++) {
//        for (j = 0; j < INDI_NUM_ACT; j++) {
//            printf("%f, ", B_matrix[i][j]);
//        }
//        printf("\n");
//    }

}

void Compute_commands_test(void)
{
//    float B_matrix_in[INDI_OUTPUTS][INDI_NUM_ACT] = {
//            {0, 0, 0, 0, -2.4525, -2.4525, -2.4525, -2.4525, 0, 0, 0, 0},
//            {0, 0, 0, 0, 0, 0, 0, 0, 2.4525, 2.4525, 2.4525, 2.4525},
//            {-0.0062, -0.0062, -0.0062, -0.0062, 0, 0, 0, 0, 0, 0, 0, 0},
//            {0.0265, -0.0265, -0.0265, 0.0265, 0, 0, 0, 0, 0, 0, 0, 0},
//            {0.0277, 0.0277, -0.0344, -0.0344, 0, 0, 0, 0, 0, 0, 0, 0},
//            {0.0010, -0.0010, 0.0010, -0.0010, 0, 0, 0, 0, 8.1791, 8.1791, -10.1533, -10.1533},
//    };

    float Desired_acceleration[INDI_OUTPUTS] = { 0, 0, 0, 0, 1, 0};

    float B_matrix_in[INDI_OUTPUTS][INDI_NUM_ACT];

    int i, j;
    float * B_matrix_in_[INDI_OUTPUTS];
    for (i = 0; i < INDI_OUTPUTS; i++) {
        B_matrix_in_[i] = &B_matrix_in[i][0];
    }
    i = 0;
    float Omega_sens[4] = {787,787,787,787};
    float Omega_dot_sens[4] = {0,0,0,0};
    float b_sens[4] = {0,0,0,0};
    float b_dot_sens[4] = {0,0,0,0};
    float g_sens[4] = {0,0,0,0};
    float g_dot_sens[4] = {0,0,0,0};

    float Euler_rad[3] = {0,0,0};
    float pqr_rad_s[3] = {0,0,0};

    Compute_B_matrix_simple(B_matrix_in_,0.1,.15,.2,1.984e-5,0.185,0.185,0.36,0.29,0,2.3,0.91e-5,1.3e-7,&Omega_sens[0],&b_sens[0],&g_sens[0],&Omega_dot_sens[0],&b_dot_sens[0],&g_dot_sens[0],&Euler_rad[0],&pqr_rad_s[0]);



    float B_matrix[INDI_OUTPUTS][INDI_NUM_ACT];
    memcpy(B_matrix, B_matrix_in, INDI_OUTPUTS * INDI_NUM_ACT * sizeof(float));

    float B_matrix_transposed[INDI_NUM_ACT][INDI_OUTPUTS];
    float * B_matrix_[INDI_NUM_ACT];

    //Transpose matrix B_in

    for (i = 0; i < INDI_OUTPUTS; i++) {
        for (j = 0; j < INDI_NUM_ACT; j++) {
            B_matrix_transposed[j][i] = B_matrix[i][j];
            B_matrix_[j] = &B_matrix_transposed[j][0];
        }
    }
    i = 0;
    j = 0;

    float w_in[INDI_OUTPUTS];

    float v_in[INDI_OUTPUTS][INDI_OUTPUTS];
    float * v_in_[INDI_OUTPUTS];
    for (i = 0; i < INDI_OUTPUTS; i++) {
        v_in_[i] = &v_in[i][0];
    }
    i = 0;

    pprz_svd_float(B_matrix_, &w_in[0], v_in_, INDI_NUM_ACT, INDI_OUTPUTS);

//    //Print the U matrix
//    printf("U = %f, %f, %f, %f, %f, %f\n", B_matrix_[0][0], B_matrix_[0][1],B_matrix_[0][2],B_matrix_[0][3],B_matrix_[0][4],B_matrix_[0][5]);
//    for (i = 1; i < INDI_NUM_ACT; i++) {
//        for (j = 0; j < INDI_OUTPUTS; j++) {
//            printf("%f, ", B_matrix_[i][j]);
//        }
//        printf("\n");
//    }
//    i = 0;
//    j = 0;
//
//    //Print the diagonal matrix (/Sum)
//    printf("diag = %f, %f, %f, %f, %f, %f\n", w_in[0], w_in[1], w_in[2], w_in[3], w_in[4], w_in[5]);
//
//    //Print the V matrix
//    printf("V = %f, %f, %f, %f, %f, %f\n", v_in_[0][0], v_in_[0][1],v_in_[0][2],v_in_[0][3],v_in_[0][4],v_in_[0][5]);
//    for (i = 1; i < INDI_OUTPUTS; i++) {
//        for (j = 0; j < INDI_OUTPUTS; j++) {
//            printf("%f, ", v_in_[i][j]);
//        }
//        printf("\n");
//    }
//    i = 0;
//    j = 0;

    //Transpose matrix U
    float U_transposed[INDI_OUTPUTS][INDI_NUM_ACT];
    for (i = 0; i < INDI_NUM_ACT; i++) {
        for (j = 0; j < INDI_OUTPUTS; j++) {
            U_transposed[j][i] = B_matrix_[i][j];
        }
    }
    i = 0;
    j = 0;

    // Invert the diag values
    float w_inverted[INDI_OUTPUTS][INDI_OUTPUTS];
    for (i = 0; i < INDI_OUTPUTS; i++) {
        w_inverted[i][i] = 1/w_in[i];
    }
    i = 0;

    //Multiply the diagonal matrix with U_transposed
    float out_1[INDI_OUTPUTS][INDI_NUM_ACT];
    int k;
    for (i = 0; i < INDI_OUTPUTS; i++) {
        for (j = 0; j < INDI_NUM_ACT; j++) {
            out_1[i][j] = 0.;
            for (k = 0; k < INDI_OUTPUTS; k++) {
                out_1[i][j] += w_inverted[i][k] * U_transposed[k][j];
            }
        }
    }
    i = 0;
    j = 0;
    k = 0;

    //Multiply V with out_1
    float Pseudoinverse[INDI_OUTPUTS][INDI_NUM_ACT];
    for (i = 0; i < INDI_OUTPUTS; i++) {
        for (j = 0; j < INDI_NUM_ACT; j++) {
            Pseudoinverse[i][j] = 0.;
            for (k = 0; k < INDI_OUTPUTS; k++) {
                Pseudoinverse[i][j] += v_in_[i][k] * out_1[k][j];
            }
        }
    }
    i = 0;
    j = 0;
    k = 0;

    //Get the final actuation command by multiplying desired acceleration with the out matrix
    float command[INDI_NUM_ACT];
    for (j = 0; j < INDI_NUM_ACT; j++) {
        command[j] = 0.;
        for (k = 0; k < INDI_OUTPUTS; k++) {
            command[j] += Desired_acceleration[k] * Pseudoinverse[k][j];
        }
    }
    i = 0;
    j = 0;
    k = 0;

    // Display the commands:
    printf("Commands = %f, ", command[0]);
    for (i = 1; i < INDI_NUM_ACT; i++) {
        printf("%f, ", command[i]);
    }
    i = 0;
    printf("\n");
}


/*
 * function to test wls for an overdetermined 4x6 (outputs x inputs) system
 */
void test_overdetermined(void)
{
  float u_min[INDI_NUM_ACT] = {0};
  float u_max[INDI_NUM_ACT] = {0};
  float du_min[INDI_NUM_ACT] = {0};
  float du_max[INDI_NUM_ACT] = {0};

  float u_p[INDI_NUM_ACT] = {0};

  float u_c[INDI_NUM_ACT] = {4614, 4210, 4210, 4614, 4210, 4210};

  printf("lower and upper bounds for du:\n");

  uint8_t k;
  for(k=0; k<INDI_NUM_ACT; k++) {
    u_max[k] = 9600 - u_min[k];

    du_min[k] = u_min[k] - u_c[k];
    du_max[k] = u_max[k] - u_c[k];

    u_p[k] = du_min[k];

    printf("%f ", du_min[k]);
    printf("%f \n", du_max[k]);
  }

  printf("\n");

  float g1g2[INDI_OUTPUTS][INDI_NUM_ACT] = {
    {  0.0,  -0.015,  0.015,  0.0,  -0.015,   0.015 },
    {  0.015,   -0.010, -0.010,   0.015,  -0.010,   -0.010 },
    {   0.103,   0.103,    0.103,   -0.103,    -0.103,    -0.103 },
    {-0.0009, -0.0009, -0.0009, -0.0009, -0.0009, -0.0009 }
  };

  //State prioritization {W Roll, W pitch, W yaw, TOTAL THRUST}
  static float Wv[INDI_OUTPUTS] = {100, 100, 1, 10};

  // The control objective in array format
  float indi_v[INDI_OUTPUTS] = {240,  -240.5658,    600.0,    1.8532};
  float indi_du[INDI_NUM_ACT];

  // Initialize the array of pointers to the rows of g1g2
  float *Bwls[INDI_OUTPUTS];
  uint8_t i;
  for (i = 0; i < INDI_OUTPUTS; i++) {
    Bwls[i] = g1g2[i];
  }

  // WLS Control Allocator
  int num_iter =
    wls_alloc(indi_du, indi_v, du_min, du_max, Bwls, 0, 0, Wv, 0, u_p, 0, 10);

  printf("finished in %d iterations\n", num_iter);

  float nu_out[4] = {0.0f};
  calc_nu_out(Bwls, indi_du, nu_out);

  printf("du                 = %f, %f, %f, %f, %f, %f\n", indi_du[0], indi_du[1], indi_du[2], indi_du[3], indi_du[4], indi_du[5]);
  // Precomputed solution' in Matlab for this problem using lsqlin:
  printf("du (matlab_lsqlin) = %f, %f, %f, %f, %f, %f\n", -4614.0, 426.064612091305, 5390.0, -4614.0, -4210.0, 5390.0);
  printf("u = %f, %f, %f, %f, %f, %f\n", indi_du[0]+u_c[0], indi_du[1]+u_c[1], indi_du[2]+u_c[2], indi_du[3]+u_c[3], indi_du[4]+u_c[4], indi_du[5]+u_c[5]);
  printf("nu_in = %f, %f, %f, %f\n", indi_v[0], indi_v[1], indi_v[2], indi_v[3]);
  printf("nu_out = %f, %f, %f, %f\n", nu_out[0], nu_out[1], nu_out[2], nu_out[3]);
}

/*
 * Calculate the achieved control objective for some calculated control input
 */
void calc_nu_out(float** Bwls, float* du, float* nu_out) {

  for(int i=0; i<4; i++) {
    nu_out[i] = 0;
    for(int j=0; j<INDI_NUM_ACT; j++) {
      nu_out[i] += Bwls[i][j] * du[j];
    }
  }
}

