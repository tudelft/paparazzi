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
//#include "modules/nav/common_flight_plan.h"
//#include "modules/nav/nav_pivot_takeoff_landing.h"

struct MassProperties mass_property = {CTRL_EFF_CALC_MASS, CTRL_EFF_CALC_I_XX, CTRL_EFF_CALC_I_YY, CTRL_EFF_CALC_I_ZZ};
struct MotorCoefficients mot_coef = {CTRL_EFF_CALC_K1, CTRL_EFF_CALC_K2, CTRL_EFF_CALC_K3};

float y_dist = CTRL_EFF_CALC_Y_DIST;
float z_dist = CTRL_EFF_CALC_Z_DIST;
float mapping = CTRL_EFF_CALC_MAPPING;

float min_thrust = 1000;				// [pprz] this is mapped to PWM range
#ifdef CTRL_EFF_CALC_THRUST_LOWER_LIM
float thrust_lower_lim = CTRL_EFF_CALC_THRUST_LOWER_LIM;
#else
float thrust_lower_lim = min_thrust;
#endif

float airspeed_scaling = 0.6;
float thrust_loss_r;
float thrust_loss_l;


static float pprz_to_rad_left(float x);
static float pprz_to_rad_right(float x);
static float pprz_to_omega(float x);
static float thrust_correcting_ratio(float x, float delta);

#if PERIODIC_TELEMETRY
// #include "modules/datalink/telemetry.h"
// static void send_ctrl_eff_module(struct transport_tx *trans, struct link_device *dev)
// {
//   pprz_msg_send_CTRL_EFF_MODULE(trans, dev, AC_ID, &thrust_loss_r, &thrust_loss_l);
// }
#endif

/*
 * Function which initialises
 */
void ctrl_eff_calc_init(void)
{
#if PERIODIC_TELEMETRY
    // register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_CTRL_EFF_MODULE, send_ctrl_eff_module);
#endif
}

/**
 * Periodic function which calls either the main control effectiveness
 * calculator function or in the case of ground contact, the ground
 * contact one.
 */
void ctrl_eff_periodic(void)
{
	if(CTRL_EFF_CALC_GROUND_CONTACT == 0) {
		ctrl_eff();
	} else {
		ctrl_eff_ground_contact();
	}
}

/**
 * Function that calculates the control effectiveness matrix based on
 * the current actuator states.
 */
void ctrl_eff(void)
{
    /**
     * Some definitions for ease of reading.
     */

    float delta_l0 = pprz_to_rad_left(actuator_state_filt_vect[0]);
    float delta_r0 = pprz_to_rad_right(actuator_state_filt_vect[1]);
#if STABILIZATION_INDI_COUNTER_TORQUE_INDIFFERENCE
    float motor_r0 = actuator_state_filt_vect[2] < min_thrust ? min_thrust : actuator_state_filt_vect[2];
    float motor_l0 = actuator_state_filt_vect[3] < min_thrust ? min_thrust : actuator_state_filt_vect[3];
#else
    float motor_r0 = actuator_state_filt_vect[2] < thrust_lower_lim ? thrust_lower_lim : actuator_state_filt_vect[2]; // [pprz]
    float motor_l0 = actuator_state_filt_vect[3] < thrust_lower_lim ? thrust_lower_lim : actuator_state_filt_vect[3]; // [pprz]
#endif
    // thrust_loss_r = thrust_correcting_ratio(motor_r0, delta_r0);
    // thrust_loss_l = thrust_correcting_ratio(motor_l0, delta_l0);
    thrust_loss_r = 1.0;
    thrust_loss_l = 1.0;

    float ctrl_deriv_00 = -y_dist * sinf(delta_l0) * (mot_coef.k1 * motor_l0 * motor_l0 + mot_coef.k2 * motor_l0 + mot_coef.k3) * thrust_loss_l * (mapping / mass_property.I_xx);
    float ctrl_deriv_01 =  y_dist * sinf(delta_r0) * (mot_coef.k1 * motor_r0 * motor_r0 + mot_coef.k2 * motor_r0 + mot_coef.k3) * thrust_loss_r* (mapping / mass_property.I_xx);
    float ctrl_deriv_02 = -y_dist * cosf(delta_r0) * (2 * mot_coef.k1 * motor_r0 + mot_coef.k2) * thrust_loss_r* (1 / mass_property.I_xx);
    float ctrl_deriv_03 =  y_dist * cosf(delta_l0) * (2 * mot_coef.k1 * motor_l0 + mot_coef.k2) * thrust_loss_l* (1 / mass_property.I_xx);
    float ctrl_deriv_10 =  z_dist * cosf(delta_l0) * (mot_coef.k1 * motor_l0 * motor_l0 + mot_coef.k2 * motor_l0 + mot_coef.k3) * thrust_loss_l* (mapping / mass_property.I_yy);
    float ctrl_deriv_11 =  z_dist * cosf(delta_r0) * (mot_coef.k1 * motor_r0 * motor_r0 + mot_coef.k2 * motor_r0 + mot_coef.k3) * thrust_loss_r* (mapping / mass_property.I_yy);
    float ctrl_deriv_12 =  z_dist * sinf(delta_r0) * (2 * mot_coef.k1 * motor_r0 + mot_coef.k2) * thrust_loss_r* (1 / mass_property.I_yy);
    float ctrl_deriv_13 =  z_dist * sinf(delta_l0) * (2 * mot_coef.k1 * motor_l0 + mot_coef.k2) * thrust_loss_l* (1 / mass_property.I_yy);
    float ctrl_deriv_20 = -y_dist * cosf(delta_l0) * (mot_coef.k1 * motor_l0 * motor_l0 + mot_coef.k2 * motor_l0 + mot_coef.k3) * thrust_loss_l* (mapping / mass_property.I_zz);
    float ctrl_deriv_21 =  y_dist * cosf(delta_r0) * (mot_coef.k1 * motor_r0 * motor_r0 + mot_coef.k2 * motor_r0 + mot_coef.k3) * thrust_loss_r* (mapping / mass_property.I_zz);
    float ctrl_deriv_22 =  y_dist * sinf(delta_r0) * (2 * mot_coef.k1 * motor_r0 + mot_coef.k2) * thrust_loss_r* (1 / mass_property.I_zz);
    float ctrl_deriv_23 = -y_dist * sinf(delta_l0) * (2 * mot_coef.k1 * motor_l0 + mot_coef.k2) * thrust_loss_l* (1 / mass_property.I_zz);
    float ctrl_deriv_30 = (mot_coef.k1 * motor_l0 * motor_l0 + mot_coef.k2 * motor_l0 + mot_coef.k3) * thrust_loss_l* sinf(delta_l0) * (mapping/mass_property.mass);
    float ctrl_deriv_31 = (mot_coef.k1 * motor_r0 * motor_r0 + mot_coef.k2 * motor_r0 + mot_coef.k3) * thrust_loss_r* sinf(delta_r0) * (mapping/mass_property.mass);
    float ctrl_deriv_32 =  -(2 * mot_coef.k1 * actuator_state_filt_vect[2] + mot_coef.k2) * thrust_loss_r* cosf(delta_r0) * (1/mass_property.mass);
    float ctrl_deriv_33 =  -(2 * mot_coef.k1 * actuator_state_filt_vect[3] + mot_coef.k2) * thrust_loss_l* cosf(delta_l0) * (1/mass_property.mass);



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
}

void ctrl_eff_ground_contact(void)
{
    float delta_l0 = pprz_to_rad_left(actuator_state_filt_vect[0]);
    float delta_r0 = pprz_to_rad_right(actuator_state_filt_vect[1]);
    float motor_r0 = actuator_state_filt_vect[2];
    float motor_l0 = actuator_state_filt_vect[3];

    float Z_DIST_GC = 0.275; 		//[m]
    float C = Z_DIST_GC + z_dist; 	//[m]
    float I_YY_GC = mass_property.I_yy + mass_property.mass * Z_DIST_GC * Z_DIST_GC;

    float ctrl_deriv_gc_00 =  0;
    float ctrl_deriv_gc_01 =  0;
    float ctrl_deriv_gc_02 =  0;
    float ctrl_deriv_gc_03 =  0;
    float ctrl_deriv_gc_10 =  C * cosf(delta_l0) * (mot_coef.k1 * motor_l0 * motor_l0 + mot_coef.k2 * motor_l0 + mot_coef.k3) * (mapping/ I_YY_GC);
    float ctrl_deriv_gc_11 =  C * cosf(delta_r0) * (mot_coef.k1 * motor_r0 * motor_r0 + mot_coef.k2 * motor_r0 + mot_coef.k3) * (mapping/ I_YY_GC);
    float ctrl_deriv_gc_12 =  C * sinf(delta_r0) * (2 * mot_coef.k1 * motor_r0 + mot_coef.k2) * (1 / I_YY_GC);
    float ctrl_deriv_gc_13 =  C * sinf(delta_l0) * (2 * mot_coef.k1 * motor_l0 + mot_coef.k2) * (1 / I_YY_GC);
    float ctrl_deriv_gc_20 =  0;
    float ctrl_deriv_gc_21 =  0;
    float ctrl_deriv_gc_22 =  0;
    float ctrl_deriv_gc_23 =  0;
//    float ctrl_deriv_gc_30 =  0;
//    float ctrl_deriv_gc_31 =  0;
//    float ctrl_deriv_gc_32 =  0;
//    float ctrl_deriv_gc_33 =  0;
    float ctrl_deriv_gc_30 = (mot_coef.k1 * motor_l0 * motor_l0 + mot_coef.k2 * motor_l0 + mot_coef.k3) * sinf(delta_l0) * (mapping/mass_property.mass);
    float ctrl_deriv_gc_31 = (mot_coef.k1 * motor_r0 * motor_r0 + mot_coef.k2 * motor_r0 + mot_coef.k3) * sinf(delta_r0) * (mapping/mass_property.mass);
    float ctrl_deriv_gc_32 =  -(2 * mot_coef.k1 * motor_r0 + mot_coef.k2) * cosf(delta_r0) * (1/mass_property.mass);
    float ctrl_deriv_gc_33 =  -(2 * mot_coef.k1 * motor_l0 + mot_coef.k2) * cosf(delta_l0) * (1/mass_property.mass);

	g1g2[0][0] = ctrl_deriv_gc_00;
	g1g2[0][1] = ctrl_deriv_gc_01;
	g1g2[0][2] = ctrl_deriv_gc_02;
	g1g2[0][3] = ctrl_deriv_gc_03;
	g1g2[1][0] = ctrl_deriv_gc_10;
	g1g2[1][1] = ctrl_deriv_gc_11;
	g1g2[1][2] = ctrl_deriv_gc_12;
	g1g2[1][3] = ctrl_deriv_gc_13;
	g1g2[2][0] = ctrl_deriv_gc_20;
	g1g2[2][1] = ctrl_deriv_gc_21;
	g1g2[2][2] = ctrl_deriv_gc_22;
	g1g2[2][3] = ctrl_deriv_gc_23;
	g1g2[3][0] = ctrl_deriv_gc_30;
	g1g2[3][1] = ctrl_deriv_gc_31;
	g1g2[3][2] = ctrl_deriv_gc_32;
	g1g2[3][3] = ctrl_deriv_gc_33;
}


/*
 * Linear function of form y = mx which maps command in pprz units
 * to an angle based on servo mapping
 */
float pprz_to_rad_left(float x)
{
	// Below gradient corresponds to 55 degrees which comes from putting
	// the max value of servo from airframe file into linear equation
	// for servo: y = -0.0909x + 136.35 (left)	[deg]
	//			  y = -0.0901x + 135.15 (right)	[deg]
//	float gradient = 0.000099157;	// [rad]
//	return gradient * x;
	float m = 0.0001;
	float c = 0.0;
	return m * x + c;
}

/*
 * Linear function of form y = mx which maps command in pprz units
 * to an angle based on servo mapping
 */
float pprz_to_rad_right(float x)
{
	// Below gradient corresponds to 55 degrees which comes from putting
	// the max value of servo from airframe file into linear equation
	// for servo: y = -0.0909x + 136.35 (left)	[deg]
	//			  y = -0.0901x + 135.15 (right)	[deg]
//	float gradient = 0.000099157;	// [rad]
//	return gradient * x;
	float m = 0.0001;
	float c = 0.0;
	return m * x + c;
}

/*
 * Function which maps PPRZ to rad/s, from static thrust tests
 */
float pprz_to_omega(float x)
{
    float k1 = -0.000006;
    float k2 = 0.2033;
    float k3 = -57.266;

    return k1*x*x + k2*x + k3;
}

/*
 * Function which calculates thrust loss ratio using conservation of
 * momentum. 'airspeed_scaling' is to prevent the loss factor getting
 * too small too quickly. A better approach would be to (TODO:) alter
 * motor angular rate 'w' w.r.t measured airspeed. The higher the
 * measured airspeed, the higher the angular rate 'w' (less drag with
 * higher induced angle of attack of the propellers). If there is an
 * RPM sensor the measured RPM would be used here.
 */
float thrust_correcting_ratio(float x, float delta)
{
    float w = pprz_to_omega(x);
    float v = stateGetAirspeed_f();
    float pp = 0.1270;  // [m] Propeller pitch, 5 inch prop pitch
    float v_in = v*airspeed_scaling*cosf(delta);
    float v_e;
    float dv;   // Difference between exhaust velocity and measured airspeed

    v_e = (w/(2*M_PI)) * pp;
    dv = v_e - v_in;
    Bound(dv, 0.5*v_e, v_e);

    return dv/v_e;
}
