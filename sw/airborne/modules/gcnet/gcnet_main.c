/*
 * Copyright (C) 2015
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file "modules/gcnet/gcnet_main.c"
 * @brief example empty controller
 *
 */

#include <time.h>

#include "modules/gcnet/gcnet_main.h"
#include "state.h"
#include "subsystems/radio_control.h"
#include "autopilot.h"
#include "subsystems/imu.h"
#include "subsystems/gps.h"

/** For waypoints */
#include "firmwares/rotorcraft/navigation.h"
#include "generated/flight_plan.h"
#define WP_EQUALS(_wp1, _wp2) WaypointX(_wp1)==WaypointX(_wp2) && WaypointY(_wp1)==WaypointY(_wp2)

/** For Filtering */
#include "filters/low_pass_filter.h"

/** ABI Messaging */
#ifndef RPM_SENSOR_ID
#define RPM_SENSOR_ID ABI_BROADCAST
#endif

#include "boards/bebop/actuators.h"
#include "subsystems/abi.h"

uint16_t nn_rpm_obs[4] = {0,0,0,0};
uint16_t nn_rpm_ref[4] = {0,0,0,0};
static abi_event rpm_read_ev;
static void rpm_read_cb(uint8_t __attribute__((unused)) sender_id, uint16_t *rpm_obs_read, uint16_t *rpm_ref_read, uint8_t __attribute__((unused)) num_act)
{
   nn_rpm_obs[0] = rpm_obs_read[0];
   nn_rpm_obs[1] = rpm_obs_read[1];
   nn_rpm_obs[2] = rpm_obs_read[2];
   nn_rpm_obs[3] = rpm_obs_read[3];

   nn_rpm_ref[0] = rpm_ref_read[0];
   nn_rpm_ref[1] = rpm_ref_read[1];
   nn_rpm_ref[2] = rpm_ref_read[2];
   nn_rpm_ref[3] = rpm_ref_read[3];
}

float state_nn[NUM_STATES];
float control_nn[NUM_CONTROLS];

float Mx_measured;
float My_measured;
float Mz_measured;
float az_measured;

float Mx_modeled;
float My_modeled;
float Mz_modeled;
float az_modeled;

float t0;
float t1;

bool active = false;

// filter angular velocities and rpms
Butterworth2LowPass filter_p;
Butterworth2LowPass filter_q;
Butterworth2LowPass filter_r;
Butterworth2LowPass filter_w1;
Butterworth2LowPass filter_w2;
Butterworth2LowPass filter_w3;
Butterworth2LowPass filter_w4;
Butterworth2LowPass filter_az;

// parameters
float Ixx = 0.00090600;
float Iyy = 0.00124200;
float Izz = 0.00205400;
float k_p  = 1.83796309e-09;
float k_q  = 1.30258126e-09;
float k_r1 = 2.24736889e-10;
float k_r2 = 3.51480672e-07;
float k_rr = 1.95277752e-03;
float k_omega = 4.33399301e-08;
float tau = 0.028;

// sample_time = 1/sample_frequency
float sample_time = 1.0/512.0;


void gcnet_init(void)
{
	// keep track of time
	// t0 = get_sys_time_float();

	// ABI messaging for reading rpm
	AbiBindMsgRPM(RPM_SENSOR_ID, &rpm_read_ev, rpm_read_cb);

	// Initialize filters (cutoff frequency=4.0)
  	float tau_ = 1.0 / (2.0 * M_PI * 4.0);				// tau = 1/(2*pi*cutoff_frequency)

	init_butterworth_2_low_pass(&filter_p, tau_, sample_time, 0.0);
	init_butterworth_2_low_pass(&filter_q, tau_, sample_time, 0.0);
	init_butterworth_2_low_pass(&filter_r, tau_, sample_time, 0.0);
	init_butterworth_2_low_pass(&filter_w1, tau_, sample_time, 0.0);
	init_butterworth_2_low_pass(&filter_w2, tau_, sample_time, 0.0);
	init_butterworth_2_low_pass(&filter_w3, tau_, sample_time, 0.0);
	init_butterworth_2_low_pass(&filter_w4, tau_, sample_time, 0.0);
	init_butterworth_2_low_pass(&filter_az, tau_, sample_time, 0.0);
}
	

void gcnet_run(void)
{
	// keep track of time
	// t1 = get_sys_time_float();
	// float dt = t1-t0;
	// t0 = t1;

	// set goal to next waypoint once activated
	if (!active && autopilot_get_mode() == AP_MODE_ATTITUDE_DIRECT) {
		if (WP_EQUALS(WP_GOAL, WP_WP1)) {
			waypoint_copy(WP_GOAL, WP_WP2);
		}
		else if (WP_EQUALS(WP_GOAL, WP_WP2)) {
			waypoint_copy(WP_GOAL, WP_WP3);
		}
		else if (WP_EQUALS(WP_GOAL, WP_WP3)) {
			waypoint_copy(WP_GOAL, WP_WP4);
		}
		else if (WP_EQUALS(WP_GOAL, WP_WP4)) {
			waypoint_copy(WP_GOAL, WP_WP1);
		}
	}

	// only active when in ATT mode
	active = autopilot_get_mode() == AP_MODE_ATTITUDE_DIRECT;

	struct FloatEulers *att   = stateGetNedToBodyEulers_f();
	struct FloatQuat   *quat  = stateGetNedToBodyQuat_f();
	struct FloatRates  *rates = stateGetBodyRates_f();
	
	//struct Int32Vect3 *body_accel_i;
	//body_accel_i = stateGetAccelBody_i();

	// get acceleration in body frame
	struct NedCoor_f *acc   = stateGetAccelNed_f();
	struct FloatVect3 acc_ned = {acc->x, acc->y, acc->z-9.81};
	struct FloatVect3 acc_body;
	float_quat_vmult(&acc_body, quat, &acc_ned);
	
	
	// get waypoint position in body frame
	struct NedCoor_f *pos   = stateGetPositionNed_f();
	struct NedCoor_f waypoint_ned;
	ENU_OF_TO_NED(waypoint_ned, waypoints[WP_GOAL].enu_f);

	struct FloatVect3 delta_pos_ned = {
		waypoint_ned.x - pos->x,
		waypoint_ned.y - pos->y,
		waypoint_ned.z - pos->z
	};

	struct FloatVect3 waypoint_body;
	float_quat_vmult(&waypoint_body, quat, &delta_pos_ned);

	// get velocity in body frame
	struct NedCoor_f *vel = stateGetSpeedNed_f();
	struct FloatVect3 vel_ned = {vel->x, vel->y, vel->z};
	struct FloatVect3 vel_body;
	float_quat_vmult(&vel_body, quat, &vel_ned);

	// update filters
	update_butterworth_2_low_pass(&filter_p, rates->p);
	update_butterworth_2_low_pass(&filter_q, rates->q);
	update_butterworth_2_low_pass(&filter_r, rates->r);
	update_butterworth_2_low_pass(&filter_w1, nn_rpm_obs[0]);
	update_butterworth_2_low_pass(&filter_w2, nn_rpm_obs[1]);
	update_butterworth_2_low_pass(&filter_w3, nn_rpm_obs[2]);
	update_butterworth_2_low_pass(&filter_w4, nn_rpm_obs[3]);
	update_butterworth_2_low_pass(&filter_az, acc_body.z);

	// calculate moments and forces
	float d_p = (filter_p.o[0] - filter_p.o[1])/sample_time;
	float d_q = (filter_q.o[0] - filter_q.o[1])/sample_time;
	float d_r = (filter_r.o[0] - filter_r.o[1])/sample_time;

	float d_w1 = (filter_w1.o[0] - filter_w1.o[1])/sample_time;
	float d_w2 = (filter_w2.o[0] - filter_w2.o[1])/sample_time;
	float d_w3 = (filter_w3.o[0] - filter_w3.o[1])/sample_time;
	float d_w4 = (filter_w4.o[0] - filter_w4.o[1])/sample_time;

	float w1 = filter_w1.o[0], w2 = filter_w2.o[0], w3 = filter_w3.o[0], w4 = filter_w4.o[0];
	float p_ = filter_p.o[0], q_ = filter_q.o[0], r_ = filter_r.o[0];

	Mx_measured = Ixx*d_p - q_*r_*(Iyy-Izz);
	My_measured = Iyy*d_q - p_*r_*(Izz-Ixx);
	Mz_measured = Izz*d_r - p_*q_*(Ixx-Iyy);
	az_measured = filter_az.o[0];

	Mx_modeled = k_p*( w1*w1 - w2*w2 - w3*w3 + w4*w4);
	My_modeled = k_q*( w1*w1 + w2*w2 - w3*w3 - w4*w4);
	Mz_modeled = k_r1*(-w1*w1 + w2*w2 - w3*w3 + w4*w4) + k_r2*(-d_w1 + d_w2 - d_w3 + d_w4) - k_rr*r_;
	az_modeled = -k_omega*(w1*w1 + w2*w2 + w3*w3 + w4*w4);

/*	
	// set goal to next waypoint once waypoint is reached (within 0.1 meter)
	float dist_to_waypoint = sqrtf(delta_pos_ned.x*delta_pos_ned.x + delta_pos_ned.y*delta_pos_ned.y + delta_pos_ned.z*delta_pos_ned.z);
	if (dist_to_waypoint < 0.1 && autopilot_get_mode() == AP_MODE_ATTITUDE_DIRECT) {
		if (WP_EQUALS(WP_GOAL, WP_WP1)) {
			waypoint_copy(WP_GOAL, WP_WP2);
		}
		else if (WP_EQUALS(WP_GOAL, WP_WP2)) {
			waypoint_copy(WP_GOAL, WP_WP3);
		}
		else if (WP_EQUALS(WP_GOAL, WP_WP3)) {
			waypoint_copy(WP_GOAL, WP_WP4);
		}
		else if (WP_EQUALS(WP_GOAL, WP_WP4)) {
			waypoint_copy(WP_GOAL, WP_WP1);
		}
	}
*/


	// neural network input 
	state_nn[0] = waypoint_body.x;
	state_nn[1] = waypoint_body.y;
	state_nn[2] = waypoint_body.z;
	
	state_nn[3] = vel_body.x;
	state_nn[4] = vel_body.y;
	state_nn[5] = vel_body.z;
	
	state_nn[6] = att->phi;
	state_nn[7] = att->theta;

	state_nn[8] = rates->p;
	state_nn[9] = rates->q;
	state_nn[10] = rates->r;

	state_nn[11] = nn_rpm_obs[0];
	state_nn[12] = nn_rpm_obs[1];
	state_nn[13] = nn_rpm_obs[2];
	state_nn[14] = nn_rpm_obs[3];

	state_nn[15] = Mx_measured - Mx_modeled;     //  9.00431696e-03;
	state_nn[16] = My_measured - My_modeled;     // -8.49446691e-03;
	state_nn[17] = Mz_measured - Mz_modeled;     // -2.41474717e-03;
	state_nn[18] = 0.0; //az_measured - az_modeled;
	
	// calcuate neural network output
	nn_control(state_nn, control_nn);
}
