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

#define INDI_INPUTS 6
#define INDI_NUM_ACT 14

void test_RSPI(void);
void compute_pseudoinverse(void);
void compute_B_matrix_extended(float * B_matrix);
void compute_pseudoinverse_fcn(int num_row, int num_column, float * B_matrix, float * B_inv);

// PARAMETERS FOR THE TEST :
float Omega_1 = 800;
float Omega_2 = 800;
float Omega_3 = 800;
float Omega_4 = 800;

float b_1 = 0;
float b_2 = 0;
float b_3 = 0;
float b_4 = 0;

float g_1 = 0;
float g_2 = 0;
float g_3 = 0;
float g_4 = 0;

float Phi = 0;
float Theta = 0;
float Psi = 0;

float l_1 = .185;
float l_2 = .185;
float l_3 = .36;
float l_4 = .29;
float l_z = 0.;

float I_xx = 0.15;
float I_yy = 0.115;
float I_zz = 0.2;
float m = 2.35;

float K_p_T = 0.9e-5;
float K_p_M = 1.31e-7;

float V = 0;

//Aerodynamic variables:
float Cl_alpha = 5.18;
float Cm_alpha = -.1;
float rho = 1.22;
float Cd_zero = 0.38;
float K_Cd = 0.2;
float S = 0.57;
float wing_chord = 0.3325;

//Max and min values:
float max_omega = 1000;
float min_omega = 200;
float max_b = 25 * M_PI/180;
float min_b = -100 * M_PI/180;
float max_g = 45 * M_PI/180;
float min_g = -45 * M_PI/180;
float max_theta_fwd = 15 * M_PI/180;
float min_theta_fwd = -15 * M_PI/180;
float max_phi_fwd = 30 * M_PI/180;

//RSPI properties
float sensibility_pseudo_control = 0.01;
float sensibility_locked_actuator = 0.01;
float max_iter_RSPI = 5;

float prioritized_actuator_states[INDI_NUM_ACT] = {0, 0, 0, 0,
                                                   0, 0, 0, 0,
                                                   0, 0, 0, 0,
                                                   0, 0 };

float INDI_acceleration_inputs[INDI_INPUTS];

float pseudo_control[INDI_INPUTS] = {0, 2, +16,
                                      0,4, 0 };

int conventional_analysis = 1;
int verbose = 1;

int main(int argc, char **argv)
{
    test_RSPI();

}

void test_RSPI(void)
{
    //Initialize the variables for the RSPI
    int locked_actuator[INDI_NUM_ACT] = {0,0,0,0,
                                         0,0,0,0,
                                         0,0,0,0,
                                         0,0};
    float scaling_array[INDI_NUM_ACT] = {1,1,1,1,
                                         1,1,1,1,
                                         1,1,1,1,
                                         1,1};
    int sum_locked_actuator = 0;
    float min_scaling_array = 1;
    float indi_u_scaled_iter[INDI_NUM_ACT];
    float indi_u_scaled_init[INDI_NUM_ACT];
    float achieved_dv[INDI_INPUTS];
    float residual_dv[INDI_INPUTS];
    float sum_residual_dv = 0;
    float max_du_scaled_iter[INDI_NUM_ACT];
    float min_du_scaled_iter[INDI_NUM_ACT];
    float max_u_scaled[INDI_NUM_ACT];
    float min_u_scaled[INDI_NUM_ACT];
    float indi_u_scaled[INDI_NUM_ACT];
    float indi_u[INDI_NUM_ACT];
    float indi_du[INDI_NUM_ACT];
    float indi_du_conv[INDI_NUM_ACT];

    float B_matrix[INDI_INPUTS][INDI_NUM_ACT];
    float B_matrix_inv[INDI_NUM_ACT][INDI_INPUTS];
    float B_matrix_scaled[INDI_INPUTS][INDI_NUM_ACT];
    float B_matrix_scaled_iter[INDI_INPUTS][INDI_NUM_ACT];

    //Evaluate B matrix :
    compute_B_matrix_extended(B_matrix[0]);

    //Print the computed effectiveness matrix
    if(verbose){
        printf("B_matrix not scaled= \n");
        for (int i = 0; i < INDI_INPUTS ; i++) {
            for (int j = 0; j < INDI_NUM_ACT; j++) {
                printf("%f, ", B_matrix[i][j]);
            }
            printf("\n");
        }
        printf("\n");
    }

    //Generate the control input status array
    float control_input_INDI_state[INDI_NUM_ACT];
    control_input_INDI_state[0] = Omega_1;
    control_input_INDI_state[1] = Omega_2;
    control_input_INDI_state[2] = Omega_3;
    control_input_INDI_state[3] = Omega_4;

    control_input_INDI_state[4] = b_1;
    control_input_INDI_state[5] = b_2;
    control_input_INDI_state[6] = b_3;
    control_input_INDI_state[7] = b_4;

    control_input_INDI_state[8] = g_1;
    control_input_INDI_state[9] = g_2;
    control_input_INDI_state[10] = g_3;
    control_input_INDI_state[11] = g_4;

    control_input_INDI_state[12] = Theta;
    control_input_INDI_state[13] = Phi;



    //Make a copy of the pseudo control array to add the increment inside the inversion:
    memcpy(& INDI_acceleration_inputs[0], & pseudo_control[0], INDI_INPUTS * sizeof(float));

    //Add the actual actuator position and the prioritized actuator states to the pseudo-control input through the effectiveness matrix:
    for (int k = 0; k < INDI_INPUTS; k++) {
        for (int i = 0; i < INDI_NUM_ACT; i++) {
            INDI_acceleration_inputs[k] += (control_input_INDI_state[i] - prioritized_actuator_states[i]) * B_matrix[k][i];
        }
    }

    //Print the new pseudo-control array
    if(verbose){
        printf("Pseudo-control array new = \n");
        for (int j = 0; j < INDI_INPUTS; j++) {
            printf("%f, ", INDI_acceleration_inputs[j]);
        }
        printf("\n\n");
    }

    //If requested, run a simple analysis on the control increment computation using the classic inversion method.
    if(conventional_analysis) {

        //Compute the pseudoinverse
        compute_pseudoinverse_fcn(INDI_INPUTS, INDI_NUM_ACT, B_matrix[0], B_matrix_inv[0]);

        //Print the Pseudoinverse
        if(verbose){
            printf("Pseudoinverse conventional = \n");
            for (int j = 0; j < INDI_NUM_ACT; j++) {
                for (int i = 0; i < INDI_INPUTS; i++) {
                    printf("%f, ", B_matrix_inv[j][i]);
                }
                printf("\n");
            }
            printf("\n");
        }

        //Compute the initial actuation increment command by multiplying desired acceleration with the pseudo-inverse matrix
        for (int j = 0; j < INDI_NUM_ACT; j++) {
            //Cleanup previous value
            indi_du_conv[j] = 0.;
            for (int k = 0; k < INDI_INPUTS; k++) {
                indi_du_conv[j] += INDI_acceleration_inputs[k] * B_matrix_inv[j][k];
            }
            indi_du_conv[j] += - control_input_INDI_state[j] + prioritized_actuator_states[j];
        }

        //Compute and print the residuals of the conventional inversion
        float residuals_conventional[INDI_NUM_ACT];
        for (int j = 0; j < INDI_INPUTS; j++) {
            residuals_conventional[j] = 0.f;
            for (int i = 0; i < INDI_NUM_ACT; i++) {
                residuals_conventional[j] += indi_du_conv[i] * B_matrix[j][i];
            }
            residuals_conventional[j] = pseudo_control[j] - residuals_conventional[j];
        }

        //Print the control increment and the residuals
        if(verbose){
            printf("Computed control increment with simple inversion= \n");
            for (int j = 0; j < INDI_NUM_ACT; j++) {
                printf("%f, ", indi_du_conv[j]);
                if (j == 3 || j == 7 || j == 11) {
                    printf("\n");
                }
            }
            printf("\n\n");
            printf("Computed residuals with simple inversion= \n");
            for (int j = 0; j < INDI_INPUTS; j++) {
                printf("%f, ", residuals_conventional[j]);
            }
            printf("\n\n");
        }
    }

    //Compute the gains for the B matrix with minimum and maximum actuator position
    float gain_motor = (max_omega - min_omega)/2;
    float gain_el = (max_b - min_b)/2;
    float gain_az = (max_g - min_g)/2;
    float gain_aoa = (max_theta_fwd - min_theta_fwd)/2;
    float gain_phi = max_phi_fwd;

    //Print gains
    if(verbose){
        printf("Gain motor = %f \n", gain_motor);
        printf("Gain el tilting = %f \n", gain_el);
        printf("Gain az tilting = %f \n", gain_az);
        printf("Gain aoa = %f \n", gain_aoa);
        printf("Gain phi = %f \n", gain_phi);
        printf("\n");
    }

    //Scale the effectiveness matrix with the actuator gains
    for (int j = 0; j < INDI_INPUTS; j++) {
        for (int i = 0; i < 4; i++) {
            B_matrix_scaled[j][i] = B_matrix[j][i] * gain_motor;
            B_matrix_scaled[j][i + 4] = B_matrix[j][i + 4] * gain_el;
            B_matrix_scaled[j][i + 8] = B_matrix[j][i + 8] * gain_az;
        }
        B_matrix_scaled[j][12] = B_matrix[j][12] * gain_aoa;
        B_matrix_scaled[j][13] = B_matrix[j][13] * gain_phi;
    }

    //Print the scaled B matrix
    if(verbose) {
        printf("Scaled B_matrix = \n");
        for (int i = 0; i < INDI_INPUTS; i++) {
            for (int j = 0; j < INDI_NUM_ACT; j++) {
                printf("%f, ", B_matrix[i][j]);
            }
            printf("\n");
        }
        printf("\n");
    }

    //Save a copy of the scaled effectiveness matrix, so then later we can modify it for the RSPI
    memcpy(&B_matrix_scaled_iter[0], &B_matrix_scaled[0], INDI_INPUTS * INDI_NUM_ACT * sizeof(float));

    //Compute the maximum and minimum actuator values:
    float max_u[INDI_NUM_ACT] = {max_omega, max_omega, max_omega, max_omega,
                                 max_b, max_b,max_b,max_b,
                                 max_g, max_g, max_g, max_g,
                                 max_theta_fwd, max_phi_fwd};
    float min_u[INDI_NUM_ACT] = {min_omega, min_omega, min_omega, min_omega,
                                 min_b, min_b,min_b,min_b,
                                 min_g, min_g, min_g, min_g,
                                 min_theta_fwd, - max_phi_fwd};

    //Compute the maximum and minimum scaled actuator values:
    for (int i = 0; i < 4; i++) {
        max_u_scaled[i] = max_omega / gain_motor;
        max_u_scaled[i + 4] = max_b / gain_el;
        max_u_scaled[i + 8] = max_g / gain_az;
        min_u_scaled[i] = min_omega / gain_motor;
        min_u_scaled[i + 4] = min_b / gain_el;
        min_u_scaled[i + 8] = min_g / gain_az;
    }
    max_u_scaled[12] = max_theta_fwd / gain_aoa;
    max_u_scaled[13] = max_phi_fwd / gain_phi;
    min_u_scaled[12] = min_theta_fwd / gain_aoa;
    min_u_scaled[13] = -max_phi_fwd / gain_phi;

    //print the maximum and minimum scaled and not scaled actuator saturation points:
    if(verbose) {
        printf("Maximum u = \n");
        for (int i = 0; i < INDI_NUM_ACT; i++) {
            printf("%f, ", max_u[i]);
        }
        printf("\n \n");

        printf("Scaled maximum u = \n");
        for (int i = 0; i < INDI_NUM_ACT; i++) {
            printf("%f, ", max_u_scaled[i]);
        }
        printf("\n \n");

        printf("Minimum u = \n");
        for (int i = 0; i < INDI_NUM_ACT; i++) {
            printf("%f, ", min_u[i]);
        }
        printf("\n \n");

        printf("Scaled minimum u = \n");
        for (int i = 0; i < INDI_NUM_ACT; i++) {
            printf("%f, ", min_u_scaled[i]);
        }
        printf("\n \n");

    }

    //Compute again the pseudoinverse for the scaled B matrix
    compute_pseudoinverse_fcn(INDI_INPUTS, INDI_NUM_ACT, B_matrix_scaled[0], B_matrix_inv[0]);

    //Print the scaled Pseudoinverse
    if(verbose) {
        printf("Pseudoinverse with scaled B matrix = \n");
        for (int j = 0; j < INDI_NUM_ACT; j++) {
            for (int i = 0; i < INDI_INPUTS; i++) {
                printf("%f, ", B_matrix_inv[j][i]);
            }
            printf("\n");
        }
        printf("\n");
    }

    //Compute the initial actuation increment scaled command by multiplying desired acceleration with the pseudo-inverse matrix
    for (int j = 0; j < INDI_NUM_ACT; j++) {
        //Cleanup previous value
        indi_u_scaled_init[j] = 0.;
        for (int k = 0; k < INDI_INPUTS; k++) {
            indi_u_scaled_init[j] += INDI_acceleration_inputs[k] * B_matrix_inv[j][k];
        }
    }

    //Print initial computed actuator input
    if(verbose) {
        printf("Initial computed control input scaled = \n");
        for (int i = 0; i < INDI_NUM_ACT; i++) {
            printf("%f, ", indi_u_scaled_init[i]);
        }
        printf("\n \n");
    }

    //Build the scaling array:
    for (int i = 0; i < INDI_NUM_ACT; i++) {
        if(indi_u_scaled_init[i] > 0){
            scaling_array[i] = fmin(1,max_u_scaled[i] / indi_u_scaled_init[i]);
        }
        else if(indi_u_scaled_init[i] < 0){
            scaling_array[i] = fmin(1,min_u_scaled[i] / indi_u_scaled_init[i]);
        }
        if(scaling_array[i] < 0){
            scaling_array[i] = 0;
        }
        //Find the minimum of the scaling_array to get the scaling parameter for the whole control input array:
        if (scaling_array[i] < min_scaling_array) {
            min_scaling_array = scaling_array[i];
        }
    }


    // Check for saturation and compute the scaling array accordingly
    for (int i = 0; i < INDI_NUM_ACT; i++) {
        if ( fabs(scaling_array[i] - min_scaling_array) < sensibility_locked_actuator) {
            locked_actuator[i] = 1;
            sum_locked_actuator ++;
            for (int j = 0; j < INDI_INPUTS; j++) {
                B_matrix_scaled_iter[j][i] = 0;
            }
        }
    }

    //Print the scaling_array and the saturated actuator
    if(verbose){
        printf("Scaling_array first computation = \n");
        for (int j = 0; j < INDI_NUM_ACT; j++) {
            printf("%f, ", scaling_array[j]);
        }
        printf("\n");
        printf("Minimum computed value of scaling array = %f \n",min_scaling_array);
        printf("\n");
    }

//    //Understand which actuators are locked(saturated) and register them into the locked_actuator array, also update the new B matrix:
//    for (int i = 0; i < INDI_NUM_ACT; i++) {
//        if (fabs(scaling_array[i] - min_scaling_array) < sensibility_locked_actuator) {
//            locked_actuator[i] = 1;
//            sum_locked_actuator ++;
//            for (int j = 0; j < INDI_INPUTS; j++) {
//                B_matrix_scaled_iter[j][i] = 0;
//            }
//        }
//    }

    //Print the locked actuator status and the new B matrix associated
    if(verbose){
        printf("Locked actuator array = \n");
        for (int j = 0; j < INDI_NUM_ACT; j++) {
            printf("%d, ", locked_actuator[j]);
        }
        printf("\n\n");
        printf("New Scaled B_matrix with saturated actuators = \n");
        for (int i = 0; i < INDI_INPUTS; i++) {
            for (int j = 0; j < INDI_NUM_ACT; j++) {
                printf("%f, ", B_matrix_scaled_iter[i][j]);
            }
            printf("\n");
        }
        printf("\n");
    }

    //If we do have saturation, scale the actuator position to the allowed one and recalculate the achieved pseudo-control:
    if (sum_locked_actuator > 0) {
        for (int i = 0; i < INDI_NUM_ACT; i++) {
            indi_u_scaled_iter[i] = indi_u_scaled_init[i] * min_scaling_array;
            indi_u_scaled[i] = indi_u_scaled_iter[i];
        }

        //Calculate the associated pseudo control achieved with the constrained control input:
        for (int j = 0; j < INDI_INPUTS; j++) {
            //Cleanup previous value
            achieved_dv[j] = 0.;
            for (int k = 0; k < INDI_NUM_ACT; k++) {
                achieved_dv[j] += B_matrix_scaled[j][k] * indi_u_scaled[k];
            }
            //Compute the residual with the main pseudo-control commanded:
            residual_dv[j] = INDI_acceleration_inputs[j] - achieved_dv[j];
            sum_residual_dv += fabs(residual_dv[j]);
        }

        //Print the first iteration residuals
        if(verbose){
            printf("First iteration pseudo-control residuals = \n");
            for (int j = 0; j < INDI_INPUTS; j++) {
                printf("%f, ", residual_dv[j]);
            }
            printf("\n");
            printf("Sum of absolute residuals = %f \n",sum_residual_dv);
            printf("\n\n");
        }

    }
    //If no saturation occurs just return the control inputs as they are.
    else {
        for (int i = 0; i < INDI_NUM_ACT; i++) {
            indi_u_scaled[i] = indi_u_scaled_init[i];
        }
    }

    //Print the first iteration scaled control input
    if(verbose){
        printf("First iteration computed scaled control input = \n");
        for (int j = 0; j < INDI_NUM_ACT; j++) {
            printf("%f, ", indi_u_scaled[j]);
        }
        printf("\n\n");
    }

    //Initialize the variable for the iterative part of the RSPI
    int iter_count = 1;

    //If we are saturated, and we didn't reach the pseudo-control target, begin the iterative process:
    while (sum_locked_actuator < INDI_NUM_ACT && iter_count < max_iter_RSPI &&
           sum_residual_dv > sensibility_pseudo_control) {

        //Increase the counter:
        iter_count++;

        //Compute the pseudoinverse with the modified B matrix
        compute_pseudoinverse_fcn(INDI_INPUTS, INDI_NUM_ACT, B_matrix_scaled_iter[0], B_matrix_inv[0]);

        //Print the new scaled Pseudoinverse
        if(verbose) {
            printf("Pseudoinverse with scaled B matrix, iteration number %d = \n", iter_count);
            for (int j = 0; j < INDI_NUM_ACT; j++) {
                for (int i = 0; i < INDI_INPUTS; i++) {
                    printf("%f, ", B_matrix_inv[j][i]);
                }
                printf("\n");
            }
            printf("\n");
        }

        //Compute the new control input associated with the residual of the pseudo-control of the previous iteration with the modified B matrix inverted
        for (int j = 0; j < INDI_NUM_ACT; j++) {
            //Cleanup previous value
            indi_u_scaled_init[j] = 0.;
            for (int k = 0; k < INDI_INPUTS; k++) {
                indi_u_scaled_init[j] += residual_dv[k] * B_matrix_inv[j][k];
            }
        }

        //Print initial actuator input increment
        if(verbose) {
            printf("Initial proposed scaled control input increment iteration number %d = \n", iter_count);
            for (int i = 0; i < INDI_NUM_ACT; i++) {
                printf("%f, ", indi_u_scaled_init[i]);
            }
            printf("\n \n");
        }

        //Reduce the new maximum control input position of the previously allocated value.
        //Notice that we pass from a global to an incremental problem.
        for (int i = 0; i < INDI_NUM_ACT; i++) {
            max_du_scaled_iter[i] = max_u_scaled[i] - indi_u_scaled[i];
            min_du_scaled_iter[i] = min_u_scaled[i] - indi_u_scaled[i];
            scaling_array[i] = 1;
        }
        min_scaling_array = 1;

        //print the maximum and minimum scaled increment at each iteration:
        if(verbose) {
            printf("Max scaled increment at iteration number %d = \n",iter_count);
            for (int i = 0; i < INDI_NUM_ACT; i++) {
                printf("%f, ", max_du_scaled_iter[i]);
            }
            printf("\n \n");

            printf("Min scaled increment at iteration number %d = \n",iter_count);
            for (int i = 0; i < INDI_NUM_ACT; i++) {
                printf("%f, ", min_du_scaled_iter[i]);
            }
            printf("\n \n");
        }

        //Build again the scaling array, this time with the new maximum actuator values:
        for (int i = 0; i < INDI_NUM_ACT; i++) {
            if (indi_u_scaled_init[i] > 0 && locked_actuator[i] == 0) {
                scaling_array[i] = fmin(1, max_du_scaled_iter[i] / indi_u_scaled_init[i]);
            } else if (indi_u_scaled_init[i] < 0 && locked_actuator[i] == 0) {
                scaling_array[i] = fmin(1, min_du_scaled_iter[i] / indi_u_scaled_init[i]);
            }
            if (scaling_array[i] < 0) {
                scaling_array[i] = 0;
            }
            //Find the minimum of the scaling_array to get the scaling parameter for the whole control input array:
            if (scaling_array[i] < min_scaling_array) {
                min_scaling_array = scaling_array[i];
            }
        }

        //Print the scaling_array at each iteration
        if(verbose){
            printf("Scaling_array at iteration number %d = \n", iter_count);
            for (int j = 0; j < INDI_NUM_ACT; j++) {
                printf("%f, ", scaling_array[j]);
            }
            printf("\n");
            printf("Minimum computed value of scaling array at iteration number %d = %f \n",iter_count,min_scaling_array);
            printf("\n");
        }

        //Understand which actuators are locked(saturated) and register them into the locked_actuator array, also update the new B matrix:
        for (int i = 0; i < INDI_NUM_ACT; i++) {
            if (fabs(scaling_array[i] - min_scaling_array) < sensibility_locked_actuator && min_scaling_array < 1) {
                locked_actuator[i] = 1;
                sum_locked_actuator ++;
                for (int j = 0; j < INDI_INPUTS; j++) {
                    B_matrix_scaled_iter[j][i] = 0;
                }
            }
        }

        //Print the locked actuator status and the new B matrix associated at each iteration
        if(verbose){
            printf("Locked actuator array at iteration number %d = \n",iter_count);
            for (int j = 0; j < INDI_NUM_ACT; j++) {
                printf("%d, ", locked_actuator[j]);
            }
            printf("\n\n");
            printf("New Scaled B_matrix with saturated actuators at iteration number %d = \n",iter_count);
            for (int i = 0; i < INDI_INPUTS; i++) {
                for (int j = 0; j < INDI_NUM_ACT; j++) {
                    printf("%f, ", B_matrix_scaled_iter[i][j]);
                }
                printf("\n");
            }
            printf("\n");
        }

        //Scale the actuator position to the allowed one and re-calculate the achieved pseudo-control:
        for (int i = 0; i < INDI_NUM_ACT; i++) {
            indi_u_scaled_iter[i] = indi_u_scaled_init[i] * min_scaling_array;
            indi_u_scaled[i] += indi_u_scaled_iter[i];
        }

        //Calculate the associated pseudo control achieved with the constrained control input:
        sum_residual_dv = 0.; //Cleanup previous value
        for (int j = 0; j < INDI_INPUTS; j++) {
            achieved_dv[j] = 0.; //Cleanup previous value
            for (int k = 0; k < INDI_NUM_ACT; k++) {
                achieved_dv[j] += B_matrix_scaled[j][k] * indi_u_scaled[k];
            }
            //Compute the residual with the main pseudo-control commanded:
            residual_dv[j] = INDI_acceleration_inputs[j] - achieved_dv[j];
            sum_residual_dv += fabs(residual_dv[j]);
        }

        //Print the residuals at each iteration
        if(verbose){
            printf("Pseudo-control residuals at iteration number %d = \n",iter_count);
            for (int j = 0; j < INDI_INPUTS; j++) {
                printf("%f, ", residual_dv[j]);
            }
            printf("\n");
            printf("Sum of absolute residuals at iteration number %d = %f \n",iter_count,sum_residual_dv);
            printf("\n\n");
        }

//        if (min_scaling_array < 1.f ) {
//            for (int i = 0; i < INDI_NUM_ACT; i++) {
//                indi_u_scaled_iter[i] = indi_u_scaled_init[i] * min_scaling_array;
//                indi_u_scaled[i] += indi_u_scaled_iter[i];
//            }
//
//            //Calculate the associated pseudo control achieved with the constrained control input:
//            sum_residual_dv = 0.; //Cleanup previous value
//            for (int j = 0; j < INDI_INPUTS; j++) {
//                achieved_dv[j] = 0.; //Cleanup previous value
//                for (int k = 0; k < INDI_NUM_ACT; k++) {
//                    achieved_dv[j] += B_matrix_scaled[j][k] * indi_u_scaled[k];
//                }
//                //Compute the residual with the main pseudo-control commanded:
//                residual_dv[j] = INDI_acceleration_inputs[j] - achieved_dv[j];
//                sum_residual_dv += fabs(residual_dv[j]);
//            }
//
//            //Print the residuals at each iteration
//            if(verbose){
//                printf("Pseudo-control residuals at iteration number %d = \n",iter_count);
//                for (int j = 0; j < INDI_INPUTS; j++) {
//                    printf("%f, ", residual_dv[j]);
//                }
//                printf("\n");
//                printf("Sum of absolute residuals at iteration number %d = %f \n",iter_count,sum_residual_dv);
//                printf("\n\n");
//            }
//        }
//        //If no saturation occurs just return the control inputs as they are.
//        else {
//            for (int i = 0; i < INDI_NUM_ACT; i++) {
//                indi_u_scaled[i] += indi_u_scaled_iter[i];
//            }
//        }

        //Print the computed scaled control input at each iteration
        if(verbose){
            printf("Scaled control input computed at iteration number %d = \n",iter_count);
            for (int j = 0; j < INDI_NUM_ACT; j++) {
                printf("%f, ", indi_u_scaled[j]);
            }
            printf("\n\n");
        }

    }

    //Multiply the scaled actuator position with the gains and get the real actuator commands, also subtract the prioritized position:
    for (int i = 0; i < 4; i++) {
        indi_u[i] = indi_u_scaled[i] * gain_motor;
        indi_u[i+4] = indi_u_scaled[i+4] * gain_el;
        indi_u[i+8] = indi_u_scaled[i+8] * gain_az;
    }
    indi_u[12] = indi_u_scaled[12] * gain_aoa;
    indi_u[13] = indi_u_scaled[13] * gain_phi;

    //Now subtract the prioritized actuator position to the computed actuator value and also compute the increment.
    for (int i = 0; i < INDI_NUM_ACT; i++) {
        indi_u[i] += prioritized_actuator_states[i];
        indi_du[i] = indi_u[i] - control_input_INDI_state[i];
    }

    //Print the output of the RSPI
    printf("Computed control increment with RSPI= \n");
    for (int j = 0; j < INDI_NUM_ACT; j++){
        printf("%f, ", indi_du[j]);
        if(j == 3 || j == 7 || j == 11){
            printf("\n");
        }
    }
    printf("\n\n");
    printf("Computed actuator position with RSPI= \n");
    for (int j = 0; j < INDI_NUM_ACT; j++){
        printf("%f, ", indi_u[j]);
        if(j == 3 || j == 7 || j == 11){
            printf("\n");
        }
    }
    printf("\n\n");

    printf("finished in %d out of %d iterations\n", iter_count, (int) max_iter_RSPI);
}

/** Moore–Penrose pseudo-inverse
 *
 * Given a matrix B(num_row,num_column), with num_row < num_column, this routine computes its Moore–Penrose inverse,
 * The Moore–Penrose inverse is based on the SVD decomposition.
 *
 * @param num_row number of rows of input the matrix B_matrix
 * @param num_column number of column of input the matrix B_matrix
 * @param B_matrix pointer to the Matrix to invert
 * @param B_inv pointer of the resulting inverted matrix. B_inv(num_column,num_row)
 */
void compute_pseudoinverse_fcn(int num_row, int num_column, float * B_matrix, float * B_inv){

    //Print the B matrix
    float B_matrix_local[num_row][num_column];
    //Make a local copy of the B matrix
    memcpy(&B_matrix_local[0], &B_matrix[0], num_row * num_column * sizeof(float));

    //Declare all the necessary variables:
    float W_diag_array[num_row];
    float V_matrix[num_row][num_row];
    float B_matrix_transposed[num_column][num_row];
    float * B_matrix_ptr[num_column];
    float * V_matrix_ptr[num_row];
    float out_local[num_row][num_row];
    float B_inv_local[num_column][num_row];
    float W_diag_inverted[num_row][num_row];
    float U_transposed[num_row][num_column];

    //Transpose matrix B_in
    for (int i = 0; i < num_row; i++) {
        for (int j = 0; j < num_column; j++) {
            B_matrix_transposed[j][i] = B_matrix_local[i][j];
        }
    }

    //Assign the pointer of B_matrix_transposed into B_matrix_ptr
    for (int j = 0; j < num_column; j++) {
        B_matrix_ptr[j] = &B_matrix_transposed[j][0];
    }

    // Pre-assign the matrices for the SVD decomposition
    for (int i = 0; i < num_row; i++) {
        V_matrix_ptr[i] = &V_matrix[i][0];
    }

    //Decompose the B matrix with the SVD decomposition module
    pprz_svd_float(B_matrix_ptr, & W_diag_array[0] , V_matrix_ptr, num_column, num_row);

    //Sort the eigenvalues and the V and U matrices with bubblesort algorithm:
    //Note that U is B_matrix_transposed;
    //FIXME need to improve speed!!!!
    float temp_W = 0;
    float temp_U = 0;
    float temp_V = 0;
    for (int j = 0; j < num_row; j++)
    {
        for (int i = 0; i < num_row - 1; i++)
        {
            if (W_diag_array[i] < W_diag_array[i + 1])
            {
                //Swap W
                temp_W = W_diag_array[i + 1];
                W_diag_array[i + 1] = W_diag_array[i];
                W_diag_array[i] = temp_W;
                //Swap U
                for(int k = 0; k < num_column ; k++){
                    temp_U = B_matrix_transposed[k][i + 1];
                    B_matrix_transposed[k][i + 1] = B_matrix_transposed[k][i];
                    B_matrix_transposed[k][i] = temp_U;
                }
                //Swap V
                for(int k = 0; k < num_row ; k++){
                    temp_V = V_matrix[k][i + 1];
                    V_matrix[k][i + 1] = V_matrix[k][i];
                    V_matrix[k][i] = temp_V;
                }
            }
        }
    }

    //Transpose matrix U from the SVD output
    for (int i = 0; i < num_column; i++) {
        for (int j = 0; j < num_row; j++) {
            U_transposed[j][i] = B_matrix_transposed[i][j];
        }
    }

    int num_row_mod = num_row;
    //Decrease the number of row if some eigenvalues are too small to avoid instabilities:
    for (int i = num_row - 1; i > 0; i--) {
        if(W_diag_array[i] < 0.00001 * W_diag_array[0]){
            num_row_mod--;
        }
    }

    // Invert the diag values and create the diagonal matrix
    for (int i = 0; i < num_row_mod; i++) {
        for (int j = 0; j < num_row_mod; j++) {
            if(i == j){
                W_diag_inverted[j][i] = 1/W_diag_array[i];
            }
            else{
                W_diag_inverted[j][i] = 0;
            }
        }
    }

    //Multiply V_matrix with the diagonal matrix W_diag_inverted
    for (int i = 0; i < num_row; i++) {
        for (int j = 0; j < num_row_mod; j++) {
            out_local[i][j] = 0.;
            for (int k = 0; k < num_row_mod; k++) {
                out_local[i][j] += V_matrix[i][k] * W_diag_inverted[j][k];
            }
        }
    }

    //Multiply out_local with the matrix U_transposed
    for (int i = 0; i < num_row; i++) {
        for (int j = 0; j < num_column; j++) {
            B_inv_local[j][i] = 0.;
            for (int k = 0; k < num_row_mod; k++) {
                B_inv_local[j][i] += out_local[i][k] * U_transposed[k][j];
            }
        }
    }

//    printf("U_transposed = \n");
//    for (int j = 0; j < num_row_mod; j++) {
//        for (int i = 0; i < num_column; i++) {
//            printf("%f, ", U_transposed[j][i]);
//        }
//        printf("\n");
//    }
//    printf("\n\n");
//
//    printf("V_matrix = \n");
//    for (int j = 0; j < num_row_mod; j++) {
//        for (int i = 0; i < num_row; i++) {
//            printf("%f, ", V_matrix[j][i]);
//        }
//        printf("\n");
//    }
//    printf("\n\n");
//
//    printf("W_diag_inverted = \n");
//    for (int j = 0; j < num_row_mod; j++) {
//        for (int i = 0; i < num_row_mod; i++) {
//            printf("%f, ", W_diag_inverted[j][i]);
//        }
//        printf("\n");
//    }
//    printf("\n\n");

//    printf("Computed pseudoinverse = \n");
//    for (int j = 0; j < num_column; j++) {
//        for (int i = 0; i < num_row_mod; i++) {
//            printf("%f, ", B_inv_local[j][i]);
//        }
//        printf("\n");
//    }
//    printf("\n\n");



//    //Multiply the diagonal matrix with U_transposed
//    for (int i = 0; i < num_row; i++) {
//        for (int j = 0; j < num_column; j++) {
//            out_local[i][j] = 0.;
//            for (int k = 0; k < num_row; k++) {
//                out_local[i][j] += W_diag_inverted[i][k] * U_transposed[k][j];
//            }
//        }
//    }
//
//    //Multiply V with out_1
//    for (int i = 0; i < num_row; i++) {
//        for (int j = 0; j < num_column; j++) {
//            B_inv_local[j][i] = 0.;
//            for (int k = 0; k < num_row; k++) {
//                B_inv_local[j][i] += V_matrix[i][k] * out_local[k][j];
//            }
//        }
//    }

    memcpy(B_inv, & B_inv_local[0], num_row * num_column * sizeof(float));
}

//This function computes the full B matrix extended 6x14
void compute_B_matrix_extended(float * B_matrix){

    float B_matrix_local[6][14];
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

    //First row
    B_matrix_local[0][0] = -(2*K_p_T*Omega_1*cos(Psi)*cos(Theta)*sin(b_1) + 2*K_p_T*Omega_1*cos(b_1)*cos(g_1)*(sin(Phi)*sin(Psi) + cos(Phi)*cos(Psi)*sin(Theta)) + 2*K_p_T*Omega_1*cos(b_1)*sin(g_1)*(cos(Phi)*sin(Psi) - cos(Psi)*sin(Phi)*sin(Theta)))/m;
    B_matrix_local[0][1] = -(2*K_p_T*Omega_2*cos(Psi)*cos(Theta)*sin(b_2) + 2*K_p_T*Omega_2*cos(b_2)*cos(g_2)*(sin(Phi)*sin(Psi) + cos(Phi)*cos(Psi)*sin(Theta)) + 2*K_p_T*Omega_2*cos(b_2)*sin(g_2)*(cos(Phi)*sin(Psi) - cos(Psi)*sin(Phi)*sin(Theta)))/m;
    B_matrix_local[0][2] = -(2*K_p_T*Omega_3*cos(Psi)*cos(Theta)*sin(b_3) + 2*K_p_T*Omega_3*cos(b_3)*cos(g_3)*(sin(Phi)*sin(Psi) + cos(Phi)*cos(Psi)*sin(Theta)) + 2*K_p_T*Omega_3*cos(b_3)*sin(g_3)*(cos(Phi)*sin(Psi) - cos(Psi)*sin(Phi)*sin(Theta)))/m;
    B_matrix_local[0][3] = -(2*K_p_T*Omega_4*cos(Psi)*cos(Theta)*sin(b_4) + 2*K_p_T*Omega_4*cos(b_4)*cos(g_4)*(sin(Phi)*sin(Psi) + cos(Phi)*cos(Psi)*sin(Theta)) + 2*K_p_T*Omega_4*cos(b_4)*sin(g_4)*(cos(Phi)*sin(Psi) - cos(Psi)*sin(Phi)*sin(Theta)))/m;

    B_matrix_local[0][4] = (K_p_T*Omega_1*Omega_1*cos(g_1)*sin(b_1)*(sin(Phi)*sin(Psi) + cos(Phi)*cos(Psi)*sin(Theta)) - K_p_T*Omega_1*Omega_1*cos(Psi)*cos(Theta)*cos(b_1) + K_p_T*Omega_1*Omega_1*sin(b_1)*sin(g_1)*(cos(Phi)*sin(Psi) - cos(Psi)*sin(Phi)*sin(Theta)))/m;
    B_matrix_local[0][5] = (K_p_T*Omega_2*Omega_2*cos(g_2)*sin(b_2)*(sin(Phi)*sin(Psi) + cos(Phi)*cos(Psi)*sin(Theta)) - K_p_T*Omega_2*Omega_2*cos(Psi)*cos(Theta)*cos(b_2) + K_p_T*Omega_2*Omega_2*sin(b_2)*sin(g_2)*(cos(Phi)*sin(Psi) - cos(Psi)*sin(Phi)*sin(Theta)))/m;
    B_matrix_local[0][6] = (K_p_T*Omega_3*Omega_3*cos(g_3)*sin(b_3)*(sin(Phi)*sin(Psi) + cos(Phi)*cos(Psi)*sin(Theta)) - K_p_T*Omega_3*Omega_3*cos(Psi)*cos(Theta)*cos(b_3) + K_p_T*Omega_3*Omega_3*sin(b_3)*sin(g_3)*(cos(Phi)*sin(Psi) - cos(Psi)*sin(Phi)*sin(Theta)))/m;
    B_matrix_local[0][7] = (K_p_T*Omega_4*Omega_4*cos(g_4)*sin(b_4)*(sin(Phi)*sin(Psi) + cos(Phi)*cos(Psi)*sin(Theta)) - K_p_T*Omega_4*Omega_4*cos(Psi)*cos(Theta)*cos(b_4) + K_p_T*Omega_4*Omega_4*sin(b_4)*sin(g_4)*(cos(Phi)*sin(Psi) - cos(Psi)*sin(Phi)*sin(Theta)))/m;

    B_matrix_local[0][8] = -(K_p_T*Omega_1*Omega_1*cos(b_1)*cos(g_1)*(cos(Phi)*sin(Psi) - cos(Psi)*sin(Phi)*sin(Theta)) - K_p_T*Omega_1*Omega_1*cos(b_1)*sin(g_1)*(sin(Phi)*sin(Psi) + cos(Phi)*cos(Psi)*sin(Theta)))/m;
    B_matrix_local[0][9] = -(K_p_T*Omega_2*Omega_2*cos(b_2)*cos(g_2)*(cos(Phi)*sin(Psi) - cos(Psi)*sin(Phi)*sin(Theta)) - K_p_T*Omega_2*Omega_2*cos(b_2)*sin(g_2)*(sin(Phi)*sin(Psi) + cos(Phi)*cos(Psi)*sin(Theta)))/m;
    B_matrix_local[0][10] = -(K_p_T*Omega_3*Omega_3*cos(b_3)*cos(g_3)*(cos(Phi)*sin(Psi) - cos(Psi)*sin(Phi)*sin(Theta)) - K_p_T*Omega_3*Omega_3*cos(b_3)*sin(g_3)*(sin(Phi)*sin(Psi) + cos(Phi)*cos(Psi)*sin(Theta)))/m;
    B_matrix_local[0][11] = -(K_p_T*Omega_4*Omega_4*cos(b_4)*cos(g_4)*(cos(Phi)*sin(Psi) - cos(Psi)*sin(Phi)*sin(Theta)) - K_p_T*Omega_4*Omega_4*cos(b_4)*sin(g_4)*(sin(Phi)*sin(Psi) + cos(Phi)*cos(Psi)*sin(Theta)))/m;

    B_matrix_local[0][12] = -(cos(Phi)*cos(Psi)*cos(Theta)*(K_p_T*cos(b_1)*cos(g_1)*Omega_1*Omega_1 + K_p_T*cos(b_2)*cos(g_2)*Omega_2*Omega_2 + K_p_T*cos(b_3)*cos(g_3)*Omega_3*Omega_3 + K_p_T*cos(b_4)*cos(g_4)*Omega_4*Omega_4) - cos(Psi)*sin(Theta)*(K_p_T*sin(b_1)*Omega_1*Omega_1 + K_p_T*sin(b_2)*Omega_2*Omega_2 + K_p_T*sin(b_3)*Omega_3*Omega_3 + K_p_T*sin(b_4)*Omega_4*Omega_4) - cos(Psi)*cos(Theta)*sin(Phi)*(K_p_T*cos(b_1)*sin(g_1)*Omega_1*Omega_1 + K_p_T*cos(b_2)*sin(g_2)*Omega_2*Omega_2 + K_p_T*cos(b_3)*sin(g_3)*Omega_3*Omega_3 + K_p_T*cos(b_4)*sin(g_4)*Omega_4*Omega_4) + (Cl_alpha*S*V*V*rho*(sin(Phi)*sin(Psi) + cos(Phi)*cos(Psi)*sin(Theta)))/2 - (S*V*V*rho*cos(Psi)*sin(Theta)*(K_Cd*Cl_alpha*Cl_alpha*Theta*Theta + Cd_zero))/2 + Cl_alpha*Cl_alpha*K_Cd*S*Theta*V*V*rho*cos(Psi)*cos(Theta) + (Cl_alpha*S*Theta*V*V*rho*cos(Phi)*cos(Psi)*cos(Theta))/2)/m;
    B_matrix_local[0][13] = -((Cl_alpha*S*Theta*rho*(cos(Phi)*sin(Psi) - cos(Psi)*sin(Phi)*sin(Theta))*V*V)/2 + (cos(Phi)*sin(Psi) - cos(Psi)*sin(Phi)*sin(Theta))*(K_p_T*cos(b_1)*cos(g_1)*Omega_1*Omega_1 + K_p_T*cos(b_2)*cos(g_2)*Omega_2*Omega_2 + K_p_T*cos(b_3)*cos(g_3)*Omega_3*Omega_3 + K_p_T*cos(b_4)*cos(g_4)*Omega_4*Omega_4) - (sin(Phi)*sin(Psi) + cos(Phi)*cos(Psi)*sin(Theta))*(K_p_T*cos(b_1)*sin(g_1)*Omega_1*Omega_1 + K_p_T*cos(b_2)*sin(g_2)*Omega_2*Omega_2 + K_p_T*cos(b_3)*sin(g_3)*Omega_3*Omega_3 + K_p_T*cos(b_4)*sin(g_4)*Omega_4*Omega_4))/m;


    //Second row
    B_matrix_local[1][0] = (2*K_p_T*Omega_1*cos(b_1)*cos(g_1)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)) - 2*K_p_T*Omega_1*cos(Theta)*sin(Psi)*sin(b_1) + 2*K_p_T*Omega_1*cos(b_1)*sin(g_1)*(cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta)))/m;
    B_matrix_local[1][1] = (2*K_p_T*Omega_2*cos(b_2)*cos(g_2)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)) - 2*K_p_T*Omega_2*cos(Theta)*sin(Psi)*sin(b_2) + 2*K_p_T*Omega_2*cos(b_2)*sin(g_2)*(cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta)))/m;
    B_matrix_local[1][2] = (2*K_p_T*Omega_3*cos(b_3)*cos(g_3)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)) - 2*K_p_T*Omega_3*cos(Theta)*sin(Psi)*sin(b_3) + 2*K_p_T*Omega_3*cos(b_3)*sin(g_3)*(cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta)))/m;
    B_matrix_local[1][3] = (2*K_p_T*Omega_4*cos(b_4)*cos(g_4)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)) - 2*K_p_T*Omega_4*cos(Theta)*sin(Psi)*sin(b_4) + 2*K_p_T*Omega_4*cos(b_4)*sin(g_4)*(cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta)))/m;

    B_matrix_local[1][4] = -(K_p_T*Omega_1*Omega_1*cos(Theta)*sin(Psi)*cos(b_1) + K_p_T*Omega_1*Omega_1*cos(g_1)*sin(b_1)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)) + K_p_T*Omega_1*Omega_1*sin(b_1)*sin(g_1)*(cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta)))/m;
    B_matrix_local[1][5] = -(K_p_T*Omega_2*Omega_2*cos(Theta)*sin(Psi)*cos(b_2) + K_p_T*Omega_2*Omega_2*cos(g_2)*sin(b_2)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)) + K_p_T*Omega_2*Omega_2*sin(b_2)*sin(g_2)*(cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta)))/m;
    B_matrix_local[1][6] = -(K_p_T*Omega_3*Omega_3*cos(Theta)*sin(Psi)*cos(b_3) + K_p_T*Omega_3*Omega_3*cos(g_3)*sin(b_3)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)) + K_p_T*Omega_3*Omega_3*sin(b_3)*sin(g_3)*(cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta)))/m;
    B_matrix_local[1][7] = -(K_p_T*Omega_4*Omega_4*cos(Theta)*sin(Psi)*cos(b_4) + K_p_T*Omega_4*Omega_4*cos(g_4)*sin(b_4)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)) + K_p_T*Omega_4*Omega_4*sin(b_4)*sin(g_4)*(cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta)))/m;

    B_matrix_local[1][8] = (K_p_T*Omega_1*Omega_1*cos(b_1)*cos(g_1)*(cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta)) - K_p_T*Omega_1*Omega_1*cos(b_1)*sin(g_1)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)))/m;
    B_matrix_local[1][9] = (K_p_T*Omega_2*Omega_2*cos(b_2)*cos(g_2)*(cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta)) - K_p_T*Omega_2*Omega_2*cos(b_2)*sin(g_2)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)))/m;
    B_matrix_local[1][10] = (K_p_T*Omega_3*Omega_3*cos(b_3)*cos(g_3)*(cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta)) - K_p_T*Omega_3*Omega_3*cos(b_3)*sin(g_3)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)))/m;
    B_matrix_local[1][11] = (K_p_T*Omega_4*Omega_4*cos(b_4)*cos(g_4)*(cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta)) - K_p_T*Omega_4*Omega_4*cos(b_4)*sin(g_4)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)))/m;

    B_matrix_local[1][12] = (sin(Psi)*sin(Theta)*(K_p_T*sin(b_1)*Omega_1*Omega_1 + K_p_T*sin(b_2)*Omega_2*Omega_2 + K_p_T*sin(b_3)*Omega_3*Omega_3 + K_p_T*sin(b_4)*Omega_4*Omega_4) - cos(Phi)*cos(Theta)*sin(Psi)*(K_p_T*cos(b_1)*cos(g_1)*Omega_1*Omega_1 + K_p_T*cos(b_2)*cos(g_2)*Omega_2*Omega_2 + K_p_T*cos(b_3)*cos(g_3)*Omega_3*Omega_3 + K_p_T*cos(b_4)*cos(g_4)*Omega_4*Omega_4) + cos(Theta)*sin(Phi)*sin(Psi)*(K_p_T*cos(b_1)*sin(g_1)*Omega_1*Omega_1 + K_p_T*cos(b_2)*sin(g_2)*Omega_2*Omega_2 + K_p_T*cos(b_3)*sin(g_3)*Omega_3*Omega_3 + K_p_T*cos(b_4)*sin(g_4)*Omega_4*Omega_4) + (Cl_alpha*S*V*V*rho*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)))/2 + (S*V*V*rho*sin(Psi)*sin(Theta)*(K_Cd*Cl_alpha*Cl_alpha*Theta*Theta + Cd_zero))/2 - Cl_alpha*Cl_alpha*K_Cd*S*Theta*V*V*rho*cos(Theta)*sin(Psi) - (Cl_alpha*S*Theta*V*V*rho*cos(Phi)*cos(Theta)*sin(Psi))/2)/m;
    B_matrix_local[1][13] = ((Cl_alpha*S*Theta*rho*(cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta))*V*V)/2 + (cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta))*(K_p_T*cos(b_1)*cos(g_1)*Omega_1*Omega_1 + K_p_T*cos(b_2)*cos(g_2)*Omega_2*Omega_2 + K_p_T*cos(b_3)*cos(g_3)*Omega_3*Omega_3 + K_p_T*cos(b_4)*cos(g_4)*Omega_4*Omega_4) - (cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta))*(K_p_T*cos(b_1)*sin(g_1)*Omega_1*Omega_1 + K_p_T*cos(b_2)*sin(g_2)*Omega_2*Omega_2 + K_p_T*cos(b_3)*sin(g_3)*Omega_3*Omega_3 + K_p_T*cos(b_4)*sin(g_4)*Omega_4*Omega_4))/m;

    //Third row
    B_matrix_local[2][0] = (2*K_p_T*Omega_1*sin(b_1)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)) + 2*K_p_T*Omega_1*cos(Theta)*sin(Phi)*cos(b_1)*sin(g_1) - 2*K_p_T*Omega_1*cos(Phi)*cos(Theta)*cos(b_1)*cos(g_1))/m;
    B_matrix_local[2][1] = (2*K_p_T*Omega_2*sin(b_2)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)) + 2*K_p_T*Omega_2*cos(Theta)*sin(Phi)*cos(b_2)*sin(g_2) - 2*K_p_T*Omega_2*cos(Phi)*cos(Theta)*cos(b_2)*cos(g_2))/m;
    B_matrix_local[2][2] = (2*K_p_T*Omega_3*sin(b_3)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)) + 2*K_p_T*Omega_3*cos(Theta)*sin(Phi)*cos(b_3)*sin(g_3) - 2*K_p_T*Omega_3*cos(Phi)*cos(Theta)*cos(b_3)*cos(g_3))/m;
    B_matrix_local[2][3] = (2*K_p_T*Omega_4*sin(b_4)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)) + 2*K_p_T*Omega_4*cos(Theta)*sin(Phi)*cos(b_4)*sin(g_4) - 2*K_p_T*Omega_4*cos(Phi)*cos(Theta)*cos(b_4)*cos(g_4))/m;

    B_matrix_local[2][4] = (K_p_T*Omega_1*Omega_1*cos(b_1)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)) + K_p_T*Omega_1*Omega_1*cos(Phi)*cos(Theta)*cos(g_1)*sin(b_1) - K_p_T*Omega_1*Omega_1*cos(Theta)*sin(Phi)*sin(b_1)*sin(g_1))/m;
    B_matrix_local[2][5] = (K_p_T*Omega_2*Omega_2*cos(b_2)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)) + K_p_T*Omega_2*Omega_2*cos(Phi)*cos(Theta)*cos(g_2)*sin(b_2) - K_p_T*Omega_2*Omega_2*cos(Theta)*sin(Phi)*sin(b_2)*sin(g_2))/m;
    B_matrix_local[2][6] = (K_p_T*Omega_3*Omega_3*cos(b_3)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)) + K_p_T*Omega_3*Omega_3*cos(Phi)*cos(Theta)*cos(g_3)*sin(b_3) - K_p_T*Omega_3*Omega_3*cos(Theta)*sin(Phi)*sin(b_3)*sin(g_3))/m;
    B_matrix_local[2][7] = (K_p_T*Omega_4*Omega_4*cos(b_4)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)) + K_p_T*Omega_4*Omega_4*cos(Phi)*cos(Theta)*cos(g_4)*sin(b_4) - K_p_T*Omega_4*Omega_4*cos(Theta)*sin(Phi)*sin(b_4)*sin(g_4))/m;

    B_matrix_local[2][8] = (K_p_T*Omega_1*Omega_1*cos(Phi)*cos(Theta)*cos(b_1)*sin(g_1) + K_p_T*Omega_1*Omega_1*cos(Theta)*sin(Phi)*cos(b_1)*cos(g_1))/m;
    B_matrix_local[2][9] = (K_p_T*Omega_2*Omega_2*cos(Phi)*cos(Theta)*cos(b_2)*sin(g_2) + K_p_T*Omega_2*Omega_2*cos(Theta)*sin(Phi)*cos(b_2)*cos(g_2))/m;
    B_matrix_local[2][10] = (K_p_T*Omega_3*Omega_3*cos(Phi)*cos(Theta)*cos(b_3)*sin(g_3) + K_p_T*Omega_3*Omega_3*cos(Theta)*sin(Phi)*cos(b_3)*cos(g_3))/m;
    B_matrix_local[2][11] = (K_p_T*Omega_4*Omega_4*cos(Phi)*cos(Theta)*cos(b_4)*sin(g_4) + K_p_T*Omega_4*Omega_4*cos(Theta)*sin(Phi)*cos(b_4)*cos(g_4))/m;

    B_matrix_local[2][12] = -(sin(Phi)*sin(Theta)*(K_p_T*cos(b_1)*sin(g_1)*Omega_1*Omega_1 + K_p_T*cos(b_2)*sin(g_2)*Omega_2*Omega_2 + K_p_T*cos(b_3)*sin(g_3)*Omega_3*Omega_3 + K_p_T*cos(b_4)*sin(g_4)*Omega_4*Omega_4) - cos(Phi)*sin(Theta)*(K_p_T*cos(b_1)*cos(g_1)*Omega_1*Omega_1 + K_p_T*cos(b_2)*cos(g_2)*Omega_2*Omega_2 + K_p_T*cos(b_3)*cos(g_3)*Omega_3*Omega_3 + K_p_T*cos(b_4)*cos(g_4)*Omega_4*Omega_4) + cos(Phi)*cos(Theta)*sin(Psi)*(K_p_T*sin(b_1)*Omega_1*Omega_1 + K_p_T*sin(b_2)*Omega_2*Omega_2 + K_p_T*sin(b_3)*Omega_3*Omega_3 + K_p_T*sin(b_4)*Omega_4*Omega_4) + (Cl_alpha*S*V*V*rho*cos(Phi)*cos(Theta))/2 + (S*V*V*rho*cos(Phi)*cos(Theta)*sin(Psi)*(K_Cd*Cl_alpha*Cl_alpha*Theta*Theta + Cd_zero))/2 - (Cl_alpha*S*Theta*V*V*rho*cos(Phi)*sin(Theta))/2 - Cl_alpha*Cl_alpha*K_Cd*S*Theta*V*V*rho*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)))/m;
    B_matrix_local[2][13] = ((cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta))*(K_p_T*sin(b_1)*Omega_1*Omega_1 + K_p_T*sin(b_2)*Omega_2*Omega_2 + K_p_T*sin(b_3)*Omega_3*Omega_3 + K_p_T*sin(b_4)*Omega_4*Omega_4) + cos(Phi)*cos(Theta)*(K_p_T*cos(b_1)*sin(g_1)*Omega_1*Omega_1 + K_p_T*cos(b_2)*sin(g_2)*Omega_2*Omega_2 + K_p_T*cos(b_3)*sin(g_3)*Omega_3*Omega_3 + K_p_T*cos(b_4)*sin(g_4)*Omega_4*Omega_4) + cos(Theta)*sin(Phi)*(K_p_T*cos(b_1)*cos(g_1)*Omega_1*Omega_1 + K_p_T*cos(b_2)*cos(g_2)*Omega_2*Omega_2 + K_p_T*cos(b_3)*cos(g_3)*Omega_3*Omega_3 + K_p_T*cos(b_4)*cos(g_4)*Omega_4*Omega_4) + (S*V*V*rho*(K_Cd*Cl_alpha*Cl_alpha*Theta*Theta + Cd_zero)*(cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta)))/2 + (Cl_alpha*S*Theta*V*V*rho*cos(Theta)*sin(Phi))/2)/m;


    //Fourth row
    B_matrix_local[3][0] = (2*K_p_M*Omega_1*sin(b_1) + 2*K_p_T*Omega_1*l_1_z*cos(b_1)*sin(g_1) + 2*K_p_T*Omega_1*l_1_y*cos(b_1)*cos(g_1))/I_xx;
    B_matrix_local[3][1] = (2*K_p_T*Omega_2*l_2_z*cos(b_2)*sin(g_2) - 2*K_p_M*Omega_2*sin(b_2) + 2*K_p_T*Omega_2*l_2_y*cos(b_2)*cos(g_2))/I_xx;
    B_matrix_local[3][2] = (2*K_p_M*Omega_3*sin(b_3) + 2*K_p_T*Omega_3*l_3_z*cos(b_3)*sin(g_3) + 2*K_p_T*Omega_3*l_3_y*cos(b_3)*cos(g_3))/I_xx;
    B_matrix_local[3][3] = (2*K_p_T*Omega_4*l_4_z*cos(b_4)*sin(g_4) - 2*K_p_M*Omega_4*sin(b_4) + 2*K_p_T*Omega_4*l_4_y*cos(b_4)*cos(g_4))/I_xx;

    B_matrix_local[3][4] = -(K_p_T*Omega_1*Omega_1*l_1_y*cos(g_1)*sin(b_1) - K_p_M*Omega_1*Omega_1*cos(b_1) + K_p_T*Omega_1*Omega_1*l_1_z*sin(b_1)*sin(g_1))/I_xx;
    B_matrix_local[3][5] = -(K_p_M*Omega_2*Omega_2*cos(b_2) + K_p_T*Omega_2*Omega_2*l_2_y*cos(g_2)*sin(b_2) + K_p_T*Omega_2*Omega_2*l_2_z*sin(b_2)*sin(g_2))/I_xx;
    B_matrix_local[3][6] = -(K_p_T*Omega_3*Omega_3*l_3_y*cos(g_3)*sin(b_3) - K_p_M*Omega_3*Omega_3*cos(b_3) + K_p_T*Omega_3*Omega_3*l_3_z*sin(b_3)*sin(g_3))/I_xx;
    B_matrix_local[3][7] = -(K_p_M*Omega_4*Omega_4*cos(b_4) + K_p_T*Omega_4*Omega_4*l_4_y*cos(g_4)*sin(b_4) + K_p_T*Omega_4*Omega_4*l_4_z*sin(b_4)*sin(g_4))/I_xx;

    B_matrix_local[3][8] = (K_p_T*Omega_1*Omega_1*l_1_z*cos(b_1)*cos(g_1) - K_p_T*Omega_1*Omega_1*l_1_y*cos(b_1)*sin(g_1))/I_xx;
    B_matrix_local[3][9] = (K_p_T*Omega_2*Omega_2*l_2_z*cos(b_2)*cos(g_2) - K_p_T*Omega_2*Omega_2*l_2_y*cos(b_2)*sin(g_2))/I_xx;
    B_matrix_local[3][10] = (K_p_T*Omega_3*Omega_3*l_3_z*cos(b_3)*cos(g_3) - K_p_T*Omega_3*Omega_3*l_3_y*cos(b_3)*sin(g_3))/I_xx;
    B_matrix_local[3][11] = (K_p_T*Omega_4*Omega_4*l_4_z*cos(b_4)*cos(g_4) - K_p_T*Omega_4*Omega_4*l_4_y*cos(b_4)*sin(g_4))/I_xx;

    B_matrix_local[3][12] = 0.f;
    B_matrix_local[3][13] = 0.f;

    //Fifth row
    B_matrix_local[4][0] = -(2*K_p_M*Omega_1*cos(b_1)*sin(g_1) - 2*K_p_T*Omega_1*l_1_z*sin(b_1) + 2*K_p_T*Omega_1*l_1_x*cos(b_1)*cos(g_1))/I_yy;
    B_matrix_local[4][1] = (2*K_p_T*Omega_2*l_2_z*sin(b_2) + 2*K_p_M*Omega_2*cos(b_2)*sin(g_2) - 2*K_p_T*Omega_2*l_2_x*cos(b_2)*cos(g_2))/I_yy;
    B_matrix_local[4][2] = -(2*K_p_M*Omega_3*cos(b_3)*sin(g_3) - 2*K_p_T*Omega_3*l_3_z*sin(b_3) + 2*K_p_T*Omega_3*l_3_x*cos(b_3)*cos(g_3))/I_yy;
    B_matrix_local[4][3] = (2*K_p_T*Omega_4*l_4_z*sin(b_4) + 2*K_p_M*Omega_4*cos(b_4)*sin(g_4) - 2*K_p_T*Omega_4*l_4_x*cos(b_4)*cos(g_4))/I_yy;

    B_matrix_local[4][4] = (K_p_M*Omega_1*Omega_1*sin(b_1)*sin(g_1) + K_p_T*Omega_1*Omega_1*l_1_z*cos(b_1) + K_p_T*Omega_1*Omega_1*l_1_x*cos(g_1)*sin(b_1))/I_yy;
    B_matrix_local[4][5] = (K_p_T*Omega_2*Omega_2*l_2_z*cos(b_2) - K_p_M*Omega_2*Omega_2*sin(b_2)*sin(g_2) + K_p_T*Omega_2*Omega_2*l_2_x*cos(g_2)*sin(b_2))/I_yy;
    B_matrix_local[4][6] = (K_p_M*Omega_3*Omega_3*sin(b_3)*sin(g_3) + K_p_T*Omega_3*Omega_3*l_3_z*cos(b_3) + K_p_T*Omega_3*Omega_3*l_3_x*cos(g_3)*sin(b_3))/I_yy;
    B_matrix_local[4][7] = (K_p_T*Omega_4*Omega_4*l_4_z*cos(b_4) - K_p_M*Omega_4*Omega_4*sin(b_4)*sin(g_4) + K_p_T*Omega_4*Omega_4*l_4_x*cos(g_4)*sin(b_4))/I_yy;

    B_matrix_local[4][8] = -(K_p_M*Omega_1*Omega_1*cos(b_1)*cos(g_1) - K_p_T*Omega_1*Omega_1*l_1_x*cos(b_1)*sin(g_1))/I_yy;
    B_matrix_local[4][9] = (K_p_M*Omega_2*Omega_2*cos(b_2)*cos(g_2) + K_p_T*Omega_2*Omega_2*l_2_x*cos(b_2)*sin(g_2))/I_yy;
    B_matrix_local[4][10] = -(K_p_M*Omega_3*Omega_3*cos(b_3)*cos(g_3) - K_p_T*Omega_3*Omega_3*l_3_x*cos(b_3)*sin(g_3))/I_yy;
    B_matrix_local[4][11] = (K_p_M*Omega_4*Omega_4*cos(b_4)*cos(g_4) + K_p_T*Omega_4*Omega_4*l_4_x*cos(b_4)*sin(g_4))/I_yy;

    B_matrix_local[4][12] = (Cm_alpha * S * V*V* rho * wing_chord) / (2 * I_yy);
    B_matrix_local[4][13] = 0.f;


    // Sixth row
    B_matrix_local[5][0] = -(2*K_p_T*Omega_1*l_1_y*sin(b_1) - 2*K_p_M*Omega_1*cos(b_1)*cos(g_1) + 2*K_p_T*Omega_1*l_1_x*cos(b_1)*sin(g_1))/I_zz;
    B_matrix_local[5][1] = -(2*K_p_T*Omega_2*l_2_y*sin(b_2) + 2*K_p_M*Omega_2*cos(b_2)*cos(g_2) + 2*K_p_T*Omega_2*l_2_x*cos(b_2)*sin(g_2))/I_zz;
    B_matrix_local[5][2] = -(2*K_p_T*Omega_3*l_3_y*sin(b_3) - 2*K_p_M*Omega_3*cos(b_3)*cos(g_3) + 2*K_p_T*Omega_3*l_3_x*cos(b_3)*sin(g_3))/I_zz;
    B_matrix_local[5][3] = -(2*K_p_T*Omega_4*l_4_y*sin(b_4) + 2*K_p_M*Omega_4*cos(b_4)*cos(g_4) + 2*K_p_T*Omega_4*l_4_x*cos(b_4)*sin(g_4))/I_zz;

    B_matrix_local[5][4] = -(K_p_T*Omega_1*Omega_1*l_1_y*cos(b_1) + K_p_M*Omega_1*Omega_1*cos(g_1)*sin(b_1) - K_p_T*Omega_1*Omega_1*l_1_x*sin(b_1)*sin(g_1))/I_zz;
    B_matrix_local[5][5] = (K_p_M*Omega_2*Omega_2*cos(g_2)*sin(b_2) - K_p_T*Omega_2*Omega_2*l_2_y*cos(b_2) + K_p_T*Omega_2*Omega_2*l_2_x*sin(b_2)*sin(g_2))/I_zz;
    B_matrix_local[5][6] = -(K_p_T*Omega_3*Omega_3*l_3_y*cos(b_3) + K_p_M*Omega_3*Omega_3*cos(g_3)*sin(b_3) - K_p_T*Omega_3*Omega_3*l_3_x*sin(b_3)*sin(g_3))/I_zz;
    B_matrix_local[5][7] = (K_p_M*Omega_4*Omega_4*cos(g_4)*sin(b_4) - K_p_T*Omega_4*Omega_4*l_4_y*cos(b_4) + K_p_T*Omega_4*Omega_4*l_4_x*sin(b_4)*sin(g_4))/I_zz;

    B_matrix_local[5][8] = -(K_p_M*Omega_1*Omega_1*cos(b_1)*sin(g_1) + K_p_T*Omega_1*Omega_1*l_1_x*cos(b_1)*cos(g_1))/I_zz;
    B_matrix_local[5][9] = (K_p_M*Omega_2*Omega_2*cos(b_2)*sin(g_2) - K_p_T*Omega_2*Omega_2*l_2_x*cos(b_2)*cos(g_2))/I_zz;
    B_matrix_local[5][10] = -(K_p_M*Omega_3*Omega_3*cos(b_3)*sin(g_3) + K_p_T*Omega_3*Omega_3*l_3_x*cos(b_3)*cos(g_3))/I_zz;
    B_matrix_local[5][11] = (K_p_M*Omega_4*Omega_4*cos(b_4)*sin(g_4) - K_p_T*Omega_4*Omega_4*l_4_x*cos(b_4)*cos(g_4))/I_zz;

    B_matrix_local[5][12] = 0.f;
    B_matrix_local[5][13] = 0.f;

    memcpy(& B_matrix[0], & B_matrix_local[0], 14*6*sizeof(float) );
}

