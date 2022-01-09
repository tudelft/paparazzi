/*
 * Copyright (C) 2021 Gervase Lovell-Prescod <gervase.prescod@gmail.com>
 */

/** @file modules/ctrl/ctrl_effectiveness_calculator.c
 * Module that calculates ctrl effectiveness matrix for tiltprop tailsitter
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
	 * Motor coefficients. Thrust as a function of ESC command (on range [0, 9600]) T = k1 x^2 + k2 x + k3
	 */
	float k1 =  0.00000002;
	float k2 = 0.0003;
	float k3 = -0.2157;

	/**
	 * Drone specific metrics
	 */
	float b = 0.3;
	float l = 0.1;
    float m = 1.3; // kg
    float I_XX = 0.017897; // kg m2
    float I_YY = 0.003390; // kg m2
    float I_ZZ = 0.020337; // kg m2

    /**
     * Some definitions for ease of reading. In the future I want to use angular acceleration of motor so labeled as omega already
     */
    float theta_l0 = ab_to_cd(actuator_state_filt_vect[0]);
    float theta_r0 = ab_to_cd(actuator_state_filt_vect[1]);
    float omega_r0 = actuator_state_filt_vect[2];
    float omega_l0 = actuator_state_filt_vect[3];

    float ctrl_deriv_00 = -b * sinf(theta_l0) * (k1 * omega_l0 * omega_l0 + k2 * omega_l0 + k3) * (1 / I_XX) * (1/9600.0);
    float ctrl_deriv_01 =  b * sinf(theta_r0) * (k1 * omega_r0 * omega_r0 + k2 * omega_r0 + k3) * (1 / I_XX) * (1/9600.0);
    float ctrl_deriv_02 = -b * cosf(theta_r0) * (2 * k1 * omega_r0 + k2) * (1 / I_XX);
    float ctrl_deriv_03 =  b * cosf(theta_l0) * (2 * k1 * omega_l0 + k2) * (1 / I_XX);
    float ctrl_deriv_10 =  l * cosf(theta_l0) * (k1 * omega_l0 * omega_l0 + k2 * omega_l0 + k3) * (1 / I_YY) * (1/9600.0);
    float ctrl_deriv_11 =  l * cosf(theta_r0) * (k1 * omega_r0 * omega_r0 + k2 * omega_r0 + k3) * (1 / I_YY) * (1/9600.0);
    float ctrl_deriv_12 =  l * sinf(theta_r0) * (2 * k1 * omega_r0 + k2) * (1 / I_YY);
    float ctrl_deriv_13 =  l * sinf(theta_l0) * (2 * k1 * omega_l0 + k2) * (1 / I_YY);
    float ctrl_deriv_20 = -b * cosf(theta_l0) * (k1 * omega_l0 * omega_l0 + k2 * omega_l0 + k3) * (1 / I_ZZ) * (1/9600.0);
    float ctrl_deriv_21 =  b * cosf(theta_r0) * (k1 * omega_r0 * omega_r0 + k2 * omega_r0 + k3) * (1 / I_ZZ) * (1/9600.0);
    float ctrl_deriv_22 =  b * sinf(theta_r0) * (2 * k1 * omega_r0 + k2) * (1 / I_ZZ);
    float ctrl_deriv_23 = -b * sinf(theta_l0) * (2 * k1 * omega_l0 + k2) * (1 / I_ZZ);
    float ctrl_deriv_30 = (k1 * omega_l0 * omega_l0 + k2 * omega_l0 + k3) * sinf(theta_l0) * (1/m) * (1/9600.0);
    float ctrl_deriv_31 = (k1 * omega_r0 * omega_r0 + k2 * omega_r0 + k3) * sinf(theta_r0) * (1/m) * (1/9600.0);
    float ctrl_deriv_32 =  -(2 * k1 * omega_r0 + k2) * cosf(theta_r0) * (1/m);
    float ctrl_deriv_33 =  -(2 * k1 * omega_l0 + k2) * cosf(theta_l0) * (1/m);


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
    float c = -1 * M_PI / 4; // Min tilt
    float d = M_PI / 4;		 // Max tilt
    return c + ((d - c) / (b - a)) * (x - a);
}

