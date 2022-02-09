/*
 * Copyright (C) 2021 Gervase Lovell-Prescod <gervase.prescod@gmail.com>
 */

/** @file modules/ctrl/ctrl_effectiveness_calculator.c
 * Module that calculates control effectiveness matrix for a tiltprop tailsitter
 */

#include "modules/ctrl/ctrl_effectiveness_calculator.h"
#include "firmwares/rotorcraft/stabilization/stabilization_indi.h"
#include "generated/airframe.h"
#include "state.h"
#include "math/pprz_algebra_float.h"

static float ab_to_cd(float x);


/**
 * Function that calculates the control effectiveness matrix
 */
void ctrl_eff(void)
{
    /**
     * Some definitions for ease of reading. In the future I want to use angular acceleration of motor so labeled as omega already
     */
    float theta_l0 = ab_to_cd(actuator_state_filt_vect[0]);
    float theta_r0 = ab_to_cd(actuator_state_filt_vect[1]);
    float omega_r0 = actuator_state_filt_vect[2];
    float omega_l0 = actuator_state_filt_vect[3];
//    float omega_r0 = actuator_state_filt_vect[2] < 3000 ? 3000 : actuator_state_filt_vect[2];
//    float omega_l0 = actuator_state_filt_vect[3] < 3000 ? 3000 : actuator_state_filt_vect[3];

    float ctrl_deriv_00 = -Y_DIST * sinf(theta_l0) * (K1 * omega_l0 * omega_l0 + K2 * omega_l0 + K3) * (1 / I_XX) * (1/(float)MAX_PPRZ);
    float ctrl_deriv_01 =  Y_DIST * sinf(theta_r0) * (K1 * omega_r0 * omega_r0 + K2 * omega_r0 + K3) * (1 / I_XX) * (1/(float)MAX_PPRZ);
//    float ctrl_deriv_00 = 0;
//	float ctrl_deriv_01 = 0;
    float ctrl_deriv_02 = -Y_DIST * cosf(theta_r0) * (2 * K1 * omega_r0 + K2) * (1 / I_XX);
    float ctrl_deriv_03 =  Y_DIST * cosf(theta_l0) * (2 * K1 * omega_l0 + K2) * (1 / I_XX);
    float ctrl_deriv_10 =  X_DIST * cosf(theta_l0) * (K1 * omega_l0 * omega_l0 + K2 * omega_l0 + K3) * (1 / I_YY) * (1/(float)MAX_PPRZ);
    float ctrl_deriv_11 =  X_DIST * cosf(theta_r0) * (K1 * omega_r0 * omega_r0 + K2 * omega_r0 + K3) * (1 / I_YY) * (1/(float)MAX_PPRZ);
    float ctrl_deriv_12 =  X_DIST * sinf(theta_r0) * (2 * K1 * omega_r0 + K2) * (1 / I_YY);
    float ctrl_deriv_13 =  X_DIST * sinf(theta_l0) * (2 * K1 * omega_l0 + K2) * (1 / I_YY);
    float ctrl_deriv_20 = -Y_DIST * cosf(theta_l0) * (K1 * omega_l0 * omega_l0 + K2 * omega_l0 + K3) * (1 / I_ZZ) * (1/(float)MAX_PPRZ);
    float ctrl_deriv_21 =  Y_DIST * cosf(theta_r0) * (K1 * omega_r0 * omega_r0 + K2 * omega_r0 + K3) * (1 / I_ZZ) * (1/(float)MAX_PPRZ);
    float ctrl_deriv_22 =  Y_DIST * sinf(theta_r0) * (2 * K1 * omega_r0 + K2) * (1 / I_ZZ);
    float ctrl_deriv_23 = -Y_DIST * sinf(theta_l0) * (2 * K1 * omega_l0 + K2) * (1 / I_ZZ);
    float ctrl_deriv_30 = (K1 * omega_l0 * omega_l0 + K2 * omega_l0 + K3) * sinf(theta_l0) * (1/MASS) * (1/(float)MAX_PPRZ);
    float ctrl_deriv_31 = (K1 * omega_r0 * omega_r0 + K2 * omega_r0 + K3) * sinf(theta_r0) * (1/MASS) * (1/(float)MAX_PPRZ);
    float ctrl_deriv_32 =  -(2 * K1 * omega_r0 + K2) * cosf(theta_r0) * (1/MASS);
    float ctrl_deriv_33 =  -(2 * K1 * omega_l0 + K2) * cosf(theta_l0) * (1/MASS);


    g1g2[0][0] = ctrl_deriv_00;
    g1g2[0][1] = ctrl_deriv_01;
    g1g2[0][2] = ctrl_deriv_02;
    g1g2[0][3] = ctrl_deriv_03;
    g1g2[1][0] = ctrl_deriv_10;
    g1g2[1][1] = ctrl_deriv_11;
    g1g2[1][2] = ctrl_deriv_12;
    g1g2[1][3] = ctrl_deriv_13;
    g1g2[2][0] = ctrl_deriv_20;
    g1g2[2][1] = ctrl_deriv_21;
    g1g2[2][2] = ctrl_deriv_22;
    g1g2[2][3] = ctrl_deriv_23;
    g1g2[3][0] = ctrl_deriv_30;
    g1g2[3][1] = ctrl_deriv_31;
    g1g2[3][2] = ctrl_deriv_32;
    g1g2[3][3] = ctrl_deriv_33;


//    printf("\n");
//    printf("\n%f\t%f\t%f\t%f", ctrl_deriv_00, ctrl_deriv_01, ctrl_deriv_02, ctrl_deriv_03);
//    printf("\n%f\t%f\t%f\t%f", ctrl_deriv_10, ctrl_deriv_11, ctrl_deriv_12, ctrl_deriv_13);
//    printf("\n%f\t%f\t%f\t%f", ctrl_deriv_20, ctrl_deriv_21, ctrl_deriv_22, ctrl_deriv_23);
//    printf("\n%f\t%f\t%f\t%f", ctrl_deriv_30, ctrl_deriv_31, ctrl_deriv_32, ctrl_deriv_33);

}



/*
 * Function that maps [a, b] to [c, d]
 */
float ab_to_cd(float x)
{
    float a = -MAX_PPRZ;
    float b = MAX_PPRZ;
    float c = -1 * M_PI / 2; // Min tilt
    float d = M_PI / 2;		 // Max tilt
    return c + ((d - c) / (b - a)) * (x - a);
}

