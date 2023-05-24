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
#include "modules/radio_control/radio_control.h"
#include "autopilot.h"
#include "modules/imu/imu.h"
#include "modules/gps/gps.h"

/** For waypoints */
#include "firmwares/rotorcraft/navigation.h"
#include "generated/flight_plan.h"
#define WP_EQUALS(_wp1, _wp2) WaypointX(_wp1)==WaypointX(_wp2) && WaypointY(_wp1)==WaypointY(_wp2) && WaypointAlt(_wp1)==WaypointAlt(_wp2)

/** For Filtering */
#include "filters/low_pass_filter.h"

/** ABI Messaging */
#ifndef RPM_SENSOR_ID
#define RPM_SENSOR_ID ABI_BROADCAST
#endif

#include "boards/bebop/actuators.h"
#include "modules/core/abi.h"

/** EKF robin */
#include "modules/ins/ins_ext_pose.h"

uint16_t nn_rpm_obs[4] = {0,0,0,0};
//uint16_t nn_rpm_ref[4] = {0,0,0,0};
static abi_event rpm_read_ev;
static void rpm_read_cb(uint8_t __attribute__((unused)) sender_id, uint16_t *rpm_obs_read, uint8_t __attribute__((unused)) num_act)
{
   nn_rpm_obs[0] = rpm_obs_read[0];
   nn_rpm_obs[1] = rpm_obs_read[1];
   nn_rpm_obs[2] = rpm_obs_read[2];
   nn_rpm_obs[3] = rpm_obs_read[3];
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
float t0_;
float t1_;

bool active = false;
bool waiting = false;
float dist_to_waypoint = 10;
float psi_ref = 0;

//float ev_pos[3] = {0.f,0.f,0.f};
//float ev_att[3] = {0.f,0.f,0.f};

// filter angular velocities and rpms
Butterworth2LowPass filter_p;
Butterworth2LowPass filter_q;
Butterworth2LowPass filter_r;
Butterworth2LowPass filter_w1;
Butterworth2LowPass filter_w2;
Butterworth2LowPass filter_w3;
Butterworth2LowPass filter_w4;
Butterworth2LowPass filter_vx;
Butterworth2LowPass filter_vy;
Butterworth2LowPass filter_az;

// parameters
float Ixx = 0.00090600;
float Iyy = 0.00124200;
float Izz = 0.00205400;
float k_p  = 1.41193310e-09; //1.83796309e-09;
float k_pv = -7.97101848e-03;
float k_q  = 1.21601884e-09; //1.30258126e-09;
float k_qv = 1.29263739e-02;
float k_r1 = 2.57035545e-06; //2.24736889e-10;
float k_r2 = 4.10923364e-07; //3.51480672e-07;
float k_rr = 8.12932607e-04; //1.95277752e-03;
float k_omega = 4.36301076e-08; //4.33399301e-08;
float tau = 0.03; //0.028;

// sample_time = 1/sample_frequency
float sample_time = 1.0/512.0;

int count = 0;

float switching_distance = 3.0; // m

// Two consecutive waypoint flight
bool set_first_wp_goal = true;


void gcnet_init(void)
{
	// keep track of time
	//t0 = get_sys_time_float();

	// ABI messaging for reading rpm
	AbiBindMsgRPM(RPM_SENSOR_ID, &rpm_read_ev, rpm_read_cb);

	// Initialize filters (cutoff frequency=8.0)
  	float tau_ = 1.0 / (2.0 * M_PI * 9.0);				// tau = 1/(2*pi*cutoff_frequency)

	init_butterworth_2_low_pass(&filter_p, tau_, sample_time, 0.0);
	init_butterworth_2_low_pass(&filter_q, tau_, sample_time, 0.0);
	init_butterworth_2_low_pass(&filter_r, tau_, sample_time, 0.0);
	init_butterworth_2_low_pass(&filter_w1, tau_, sample_time, 0.0);
	init_butterworth_2_low_pass(&filter_w2, tau_, sample_time, 0.0);
	init_butterworth_2_low_pass(&filter_w3, tau_, sample_time, 0.0);
	init_butterworth_2_low_pass(&filter_w4, tau_, sample_time, 0.0);
	init_butterworth_2_low_pass(&filter_vx, tau_, sample_time, 0.0);
	init_butterworth_2_low_pass(&filter_vy, tau_, sample_time, 0.0);
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

		// Set first waypoint goal to WP3 (for two consecutive waypoint flight)
		if (set_first_wp_goal == true){
			// Single waypoint flight
			//waypoint_copy(WP_GOAL, WP_WP2);

			// Consecutive waypoint flight 
			waypoint_copy(WP_GOAL, WP_WP3);

			// Consecutive waypoint flight and randomnly positioned 4x3m track
			//waypoint_copy(WP_GOAL, WP_WP3_rand3);

			set_first_wp_goal = false;
		     }
		/*else {
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
		      }*/
	}


	// only active when in ATT mode
	active = autopilot_get_mode() == AP_MODE_ATTITUDE_DIRECT;

	struct FloatEulers att; //   = stateGetNedToBodyEulers_f();
	//substitute ekf values
	att.phi   = ekf_X[6];
	att.theta = ekf_X[7];
	att.psi   = ekf_X[8];

	struct FloatQuat   quat; //  = stateGetNedToBodyQuat_f();
	//substitute ekf values
	float_quat_of_eulers(&quat, &att);

	struct FloatRates  *rates = stateGetBodyRates_f();
	float unbiased_p = rates->p - ekf_X[12];
	float unbiased_q = rates->q - ekf_X[13];
	float unbiased_r = rates->r - ekf_X[14];
	
	//struct Int32Vect3 *body_accel_i;
	//body_accel_i = stateGetAccelBody_i();

	// get acceleration in body frame
	struct NedCoor_f *acc   = stateGetAccelNed_f();
	struct FloatVect3 acc_ned = {acc->x, acc->y, acc->z-9.81};
	struct FloatVect3 acc_body;
	float_quat_vmult(&acc_body, &quat, &acc_ned);
	
	
	// get waypoint position in body frame
	struct NedCoor_f *pos   = stateGetPositionNed_f();
	struct NedCoor_f waypoint_ned;
	ENU_OF_TO_NED(waypoint_ned, waypoints[WP_GOAL].enu_f);

	struct FloatVect3 delta_pos_ned = {
		waypoint_ned.x - ekf_X[0],
		waypoint_ned.y - ekf_X[1],
		waypoint_ned.z - ekf_X[2]
	};

	struct FloatVect3 waypoint_body;
	float_quat_vmult(&waypoint_body, &quat, &delta_pos_ned);

	// get velocity in body frame
	struct NedCoor_f *vel = stateGetSpeedNed_f();
	struct FloatVect3 vel_ned = {ekf_X[3], ekf_X[4], ekf_X[5]};
	struct FloatVect3 vel_body;
	float_quat_vmult(&vel_body, &quat, &vel_ned);

	// update filters
	update_butterworth_2_low_pass(&filter_p, rates->p);
	update_butterworth_2_low_pass(&filter_q, rates->q);
	update_butterworth_2_low_pass(&filter_r, rates->r);
	update_butterworth_2_low_pass(&filter_w1, nn_rpm_obs[0]);
	update_butterworth_2_low_pass(&filter_w2, nn_rpm_obs[1]);
	update_butterworth_2_low_pass(&filter_w3, nn_rpm_obs[2]);
	update_butterworth_2_low_pass(&filter_w4, nn_rpm_obs[3]);
	update_butterworth_2_low_pass(&filter_vx, vel_body.x);
	update_butterworth_2_low_pass(&filter_vy, vel_body.y);
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
	float vbx = filter_vx.o[0], vby = filter_vy.o[0];

	Mx_measured = Ixx*d_p - q_*r_*(Iyy-Izz);
	My_measured = Iyy*d_q - p_*r_*(Izz-Ixx);
	Mz_measured = Izz*d_r - p_*q_*(Ixx-Iyy);
	//az_measured = filter_az.o[0];

	Mx_modeled = k_p*( w1*w1 - w2*w2 - w3*w3 + w4*w4) + k_pv*vby;
	My_modeled = k_q*( w1*w1 + w2*w2 - w3*w3 - w4*w4) + k_qv*vbx;
	Mz_modeled = k_r1*(-w1 + w2 - w3 + w4) + k_r2*(-d_w1 + d_w2 - d_w3 + d_w4) - k_rr*r_;
	//az_modeled = -k_omega*(w1*w1 + w2*w2 + w3*w3 + w4*w4);

/*
	//TEMPORARY FOR TEST
	psi_ref = atan2(waypoints[WP_WP2].enu_f.x-waypoints[WP_WP1].enu_f.x, waypoints[WP_WP2].enu_f.y-waypoints[WP_WP1].enu_f.y);
	dist_to_waypoint = sqrtf(delta_pos_ned.x*delta_pos_ned.x + delta_pos_ned.y*delta_pos_ned.y + delta_pos_ned.z*delta_pos_ned.z);
	//dist_to_waypoint < .3
	if (active && count<40) {
		if (waiting == false) {
			waiting = true;
			t0_ = get_sys_time_float();
		}
		t1_ = get_sys_time_float();
		if (t1_-t0_ > 4) {
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
			waiting=false;
			count += 1;
		}
	}
*/
/*
	// set goal to next waypoint once waypoint is reached (within 0.3 meter)
	if (dist_to_waypoint < 0.3 && dist_to_waypoint < sqrtf(delta_pos_ned.x*delta_pos_ned.x + delta_pos_ned.y*delta_pos_ned.y + delta_pos_ned.z*delta_pos_ned.z) && autopilot_get_mode() == AP_MODE_ATTITUDE_DIRECT) {
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
		printf("dist_to_waypoint = %f \n", dist_to_waypoint);
	}
	ENU_OF_TO_NED(waypoint_ned, waypoints[WP_GOAL].enu_f);

	struct FloatVect3 delta_pos_ned2 = {
		waypoint_ned.x - pos->x,
		waypoint_ned.y - pos->y,
		waypoint_ned.z - pos->z
	};
	dist_to_waypoint = sqrtf(delta_pos_ned2.x*delta_pos_ned2.x + delta_pos_ned2.y*delta_pos_ned2.y + delta_pos_ned2.z*delta_pos_ned2.z);
*/

	if (autopilot_get_mode() == AP_MODE_ATTITUDE_DIRECT) {
		// Added following condition: two consecutive waypoint changes need to be separated in time by at least .1 seconds
		// This is to prevent hysteresis due to position errors at high speeds.
		if (waiting == false) {
			waiting = true;
			t0_ = get_sys_time_float();
		}
		t1_ = get_sys_time_float();

		if (WP_EQUALS(WP_GOAL, WP_WP1)) {

			// Hover-to-Hover			
			//psi_ref = atan2(waypoints[WP_WP2].enu_f.x-waypoints[WP_WP1].enu_f.x, waypoints[WP_WP2].enu_f.y-waypoints[WP_WP1].enu_f.y);
			

			//// Single WP
			//psi_ref = atan2(waypoints[WP_WP1].enu_f.x-waypoints[WP_WP4].enu_f.x, waypoints[WP_WP1].enu_f.y-waypoints[WP_WP4].enu_f.y);

			//// Two consecutive WPs	
			psi_ref = atan2(waypoints[WP_WP1].enu_f.x-waypoints[WP_WP4].enu_f.x, waypoints[WP_WP1].enu_f.y-waypoints[WP_WP4].enu_f.y) - M_PI/2;
			// One extra input (for consecutive WP flight) - Distance between WPs
			state_nn[19] = -3.0;
			switching_distance = 3.0;
			// Two extra inputs (for consecutive WP flight) - XY
			//state_nn[20] = 0.0; // WP1_x
			//state_nn[19] = -3.0; // WP1_y
			//switching_distance = 3.0;

			// set next waypoint once drone passes the plane perpendicular to the waypoint (psi_ref + pi/4 direction)
			//float normal_x = cos(psi_ref + M_PI/4);
			//float normal_y = sin(psi_ref + M_PI/4);		
			//if (-delta_pos_ned.x*normal_x - delta_pos_ned.y*normal_y > -2.0 && t1_-t0_ > 0.1) {

			// Use euclidean distance instead
			if ( delta_pos_ned.x*delta_pos_ned.x + delta_pos_ned.y*delta_pos_ned.y + delta_pos_ned.z*delta_pos_ned.z < switching_distance*switching_distance && t1_-t0_ > 0.1) {
				waypoint_copy(WP_GOAL, WP_WP2);
				waiting=false;
			}
		}
		else if (WP_EQUALS(WP_GOAL, WP_WP2)) {

			// Hover-to-Hover			
			//psi_ref = atan2(waypoints[WP_WP2].enu_f.x-waypoints[WP_WP1].enu_f.x, waypoints[WP_WP2].enu_f.y-waypoints[WP_WP1].enu_f.y);
			

			// Single WP			
			//psi_ref = atan2(waypoints[WP_WP2].enu_f.x-waypoints[WP_WP1].enu_f.x, waypoints[WP_WP2].enu_f.y-waypoints[WP_WP1].enu_f.y);
			
			// Two consecutive WPs	
			psi_ref = atan2(waypoints[WP_WP2].enu_f.x-waypoints[WP_WP1].enu_f.x, waypoints[WP_WP2].enu_f.y-waypoints[WP_WP1].enu_f.y) - M_PI/2;
			// One extra input (for consecutive WP flight) - Distance between WPs
			state_nn[19] = -4.0;
			switching_distance = 4.0;
			// Two extra inputs (for consecutive WP flight) - XY
			//state_nn[20] = 0.0; // WP1_x
			//state_nn[19] = -4.0; // WP1_y
			//switching_distance = 4.0;

			// set next waypoint once drone passes the plane perpendicular to the waypoint (psi_ref + pi/4 direction)
			//float normal_x = cos(psi_ref + M_PI/4);
			//float normal_y = sin(psi_ref + M_PI/4);		
			// if (-delta_pos_ned.x*normal_x - delta_pos_ned.y*normal_y > -2.0 && t1_-t0_ > 0.1) {

			// Use euclidean distance instead
			if ( delta_pos_ned.x*delta_pos_ned.x + delta_pos_ned.y*delta_pos_ned.y + delta_pos_ned.z*delta_pos_ned.z < switching_distance*switching_distance && t1_-t0_ > 0.1) {
				waypoint_copy(WP_GOAL, WP_WP3);
				waiting=false;
			}
		}
		else if (WP_EQUALS(WP_GOAL, WP_WP3)) {
			// Single WP			
			//psi_ref = atan2(waypoints[WP_WP3].enu_f.x-waypoints[WP_WP2].enu_f.x, waypoints[WP_WP3].enu_f.y-waypoints[WP_WP2].enu_f.y);
			
			// Two consecutive WPs	
			psi_ref = atan2(waypoints[WP_WP3].enu_f.x-waypoints[WP_WP2].enu_f.x, waypoints[WP_WP3].enu_f.y-waypoints[WP_WP2].enu_f.y) - M_PI/2;
			// One extra input (for consecutive WP flight) - Distance between WPs
			state_nn[19] = -3.0;
			switching_distance = 3.0;
			// Two extra inputs (for consecutive WP flight) - XY
			//state_nn[20] = 0.0; // WP1_x
			//state_nn[19] = -3.0; // WP1_y
			//switching_distance = 3.0;

			// set next waypoint once drone passes the plane perpendicular to the waypoint (psi_ref + pi/4 direction)
			//float normal_x = cos(psi_ref + M_PI/4);
			//float normal_y = sin(psi_ref + M_PI/4);		
			//if (-delta_pos_ned.x*normal_x - delta_pos_ned.y*normal_y > -2.0 && t1_-t0_ > 0.1) {
		

			// Use euclidean distance instead
			if ( delta_pos_ned.x*delta_pos_ned.x + delta_pos_ned.y*delta_pos_ned.y + delta_pos_ned.z*delta_pos_ned.z < switching_distance*switching_distance && t1_-t0_ > 0.1) {
				waypoint_copy(WP_GOAL, WP_WP4);
				waiting=false;
			}
		}
		else if (WP_EQUALS(WP_GOAL, WP_WP4)) {
			// Single WP			
			//psi_ref = atan2(waypoints[WP_WP4].enu_f.x-waypoints[WP_WP3].enu_f.x, waypoints[WP_WP4].enu_f.y-waypoints[WP_WP3].enu_f.y);
			
			// Two consecutive WPs	
			psi_ref = atan2(waypoints[WP_WP4].enu_f.x-waypoints[WP_WP3].enu_f.x, waypoints[WP_WP4].enu_f.y-waypoints[WP_WP3].enu_f.y) - M_PI/2;
			// One extra input (for consecutive WP flight) - Distance between WPs
			state_nn[19] = -4.0;
			switching_distance = 4.0;
			// Two extra inputs (for consecutive WP flight) - XY
			//state_nn[20] = 0.0; // WP1_x
			//state_nn[19] = -4.0; // WP1_y
			//switching_distance = 4.0;

			// set next waypoint once drone passes the plane perpendicular to the waypoint (psi_ref + pi/4 direction)
			// float normal_x = cos(psi_ref + M_PI/4);
			// float normal_y = sin(psi_ref + M_PI/4);		
			//if (-delta_pos_ned.x*normal_x - delta_pos_ned.y*normal_y > -2.0 && t1_-t0_ > 0.1) {

			// Use euclidean distance instead
			if ( delta_pos_ned.x*delta_pos_ned.x + delta_pos_ned.y*delta_pos_ned.y + delta_pos_ned.z*delta_pos_ned.z < switching_distance*switching_distance && t1_-t0_ > 0.1) {
				waypoint_copy(WP_GOAL, WP_WP1);
				waiting=false;
			}
		}

	// 4x3m track with Y and X relative position between WPs as GCNET input
	// "Randomly" positioned waypoints for 4x3m track

		else if (WP_EQUALS(WP_GOAL, WP_WP1_rand1)) {
			// Single WP
			//psi_ref = atan2(waypoints[WP_WP1].enu_f.x-waypoints[WP_WP4].enu_f.x, waypoints[WP_WP1].enu_f.y-waypoints[WP_WP4].enu_f.y);

			// Two consecutive WPs	
			psi_ref = atan2(waypoints[WP_WP1].enu_f.x-waypoints[WP_WP4].enu_f.x, waypoints[WP_WP1].enu_f.y-waypoints[WP_WP4].enu_f.y) - M_PI/2;
			// Two extra inputs (for consecutive WP flight) - Relative Y and X position in rotated frame
			// WP_WP1_rand1
			state_nn[20] = 0.222; // WP1_x
			state_nn[19] = -3.0; // WP1_y

			switching_distance = 3.008;

			// Use euclidean distance instead
			if ( delta_pos_ned.x*delta_pos_ned.x + delta_pos_ned.y*delta_pos_ned.y + delta_pos_ned.z*delta_pos_ned.z < switching_distance*switching_distance && t1_-t0_ > 0.1) {
				waypoint_copy(WP_GOAL, WP_WP2_rand2);
				waiting=false;
			}
		}
		else if (WP_EQUALS(WP_GOAL, WP_WP2_rand2)) {
			// Single WP			
			//psi_ref = atan2(waypoints[WP_WP2].enu_f.x-waypoints[WP_WP1].enu_f.x, waypoints[WP_WP2].enu_f.y-waypoints[WP_WP1].enu_f.y);
			
			// Two consecutive WPs	
			psi_ref = atan2(waypoints[WP_WP2].enu_f.x-waypoints[WP_WP1].enu_f.x, waypoints[WP_WP2].enu_f.y-waypoints[WP_WP1].enu_f.y) - M_PI/2;
			// Two extra inputs (for consecutive WP flight) - Relative Y and X position in rotated frame
			// WP_WP2_rand2
			state_nn[20] = 0.0; // WP1_x
			state_nn[19] = -4.0; // WP1_y

			switching_distance = 4.0;

			// Use euclidean distance instead
			if ( delta_pos_ned.x*delta_pos_ned.x + delta_pos_ned.y*delta_pos_ned.y + delta_pos_ned.z*delta_pos_ned.z < switching_distance*switching_distance && t1_-t0_ > 0.1) {
				waypoint_copy(WP_GOAL, WP_WP3_rand3);
				waiting=false;
			}
		}
		else if (WP_EQUALS(WP_GOAL, WP_WP3_rand3)) {
			// Single WP			
			//psi_ref = atan2(waypoints[WP_WP3].enu_f.x-waypoints[WP_WP2].enu_f.x, waypoints[WP_WP3].enu_f.y-waypoints[WP_WP2].enu_f.y);
			
			// Two consecutive WPs	
			psi_ref = atan2(waypoints[WP_WP3].enu_f.x-waypoints[WP_WP2].enu_f.x, waypoints[WP_WP3].enu_f.y-waypoints[WP_WP2].enu_f.y) - M_PI/2;
			// Two extra inputs (for consecutive WP flight) - Relative Y and X position in rotated frame
			// WP_WP3_rand3
			state_nn[20] = 0.0; // WP1_x
			state_nn[19] = -3.0; // WP1_y

			switching_distance = 3.0;

			// Use euclidean distance instead
			if ( delta_pos_ned.x*delta_pos_ned.x + delta_pos_ned.y*delta_pos_ned.y + delta_pos_ned.z*delta_pos_ned.z < switching_distance*switching_distance && t1_-t0_ > 0.1) {
				waypoint_copy(WP_GOAL, WP_WP4_rand4);
				waiting=false;
			}
		}
		else if (WP_EQUALS(WP_GOAL, WP_WP4_rand4)) {
			// Single WP			
			//psi_ref = atan2(waypoints[WP_WP4].enu_f.x-waypoints[WP_WP3].enu_f.x, waypoints[WP_WP4].enu_f.y-waypoints[WP_WP3].enu_f.y);
			
			// Two consecutive WPs	
			psi_ref = atan2(waypoints[WP_WP4].enu_f.x-waypoints[WP_WP3].enu_f.x, waypoints[WP_WP4].enu_f.y-waypoints[WP_WP3].enu_f.y) - M_PI/2;
			// Two extra inputs (for consecutive WP flight) - Relative Y and X position in rotated frame
			// WP_WP4_rand4
			state_nn[20] = 0.0; // WP1_x
			state_nn[19] = -4.0; // WP1_y

			switching_distance = 4.0;

			// Use euclidean distance instead
			if ( delta_pos_ned.x*delta_pos_ned.x + delta_pos_ned.y*delta_pos_ned.y + delta_pos_ned.z*delta_pos_ned.z < switching_distance*switching_distance && t1_-t0_ > 0.1) {
				waypoint_copy(WP_GOAL, WP_WP1_rand5);
				waiting=false;
			}
		}

			///////////////////

		else if (WP_EQUALS(WP_GOAL, WP_WP1_rand5)) {
			// Single WP
			//psi_ref = atan2(waypoints[WP_WP1].enu_f.x-waypoints[WP_WP4].enu_f.x, waypoints[WP_WP1].enu_f.y-waypoints[WP_WP4].enu_f.y);

			// Two consecutive WPs	
			psi_ref = atan2(waypoints[WP_WP1].enu_f.x-waypoints[WP_WP4].enu_f.x, waypoints[WP_WP1].enu_f.y-waypoints[WP_WP4].enu_f.y) - M_PI/2;
			// Two extra inputs (for consecutive WP flight) - Relative Y and X position in rotated frame
			// WP_WP1_rand5
			state_nn[20] = 0.45; // WP1_x
			state_nn[19] = -3.45; // WP1_y

			switching_distance = 3.479;

			// Use euclidean distance instead
			if ( delta_pos_ned.x*delta_pos_ned.x + delta_pos_ned.y*delta_pos_ned.y + delta_pos_ned.z*delta_pos_ned.z < switching_distance*switching_distance && t1_-t0_ > 0.1) {
				waypoint_copy(WP_GOAL, WP_WP2_rand6);
				waiting=false;
			}
		}
		else if (WP_EQUALS(WP_GOAL, WP_WP2_rand6)) {
			// Single WP			
			//psi_ref = atan2(waypoints[WP_WP2].enu_f.x-waypoints[WP_WP1].enu_f.x, waypoints[WP_WP2].enu_f.y-waypoints[WP_WP1].enu_f.y);
			
			// Two consecutive WPs	
			psi_ref = atan2(waypoints[WP_WP2].enu_f.x-waypoints[WP_WP1].enu_f.x, waypoints[WP_WP2].enu_f.y-waypoints[WP_WP1].enu_f.y) - M_PI/2;
			// Two extra inputs (for consecutive WP flight) - Relative Y and X position in rotated frame
			// WP_WP2_rand6
			state_nn[20] = 0.0; // WP1_x
			state_nn[19] = -4.0; // WP1_y

			switching_distance = 4.0;

			// Use euclidean distance instead
			if ( delta_pos_ned.x*delta_pos_ned.x + delta_pos_ned.y*delta_pos_ned.y + delta_pos_ned.z*delta_pos_ned.z < switching_distance*switching_distance && t1_-t0_ > 0.1) {
				waypoint_copy(WP_GOAL, WP_WP3_rand7);
				waiting=false;
			}
		}
		else if (WP_EQUALS(WP_GOAL, WP_WP3_rand7)) {
			// Single WP			
			//psi_ref = atan2(waypoints[WP_WP3].enu_f.x-waypoints[WP_WP2].enu_f.x, waypoints[WP_WP3].enu_f.y-waypoints[WP_WP2].enu_f.y);
			
			// Two consecutive WPs	
			psi_ref = atan2(waypoints[WP_WP3].enu_f.x-waypoints[WP_WP2].enu_f.x, waypoints[WP_WP3].enu_f.y-waypoints[WP_WP2].enu_f.y) - M_PI/2;
			// Two extra inputs (for consecutive WP flight) - Relative Y and X position in rotated frame
			// WP_WP3_rand7
			state_nn[20] = 0.0; // WP1_x
			state_nn[19] = -3.0; // WP1_y

			switching_distance = 3.0;

			// Use euclidean distance instead
			if ( delta_pos_ned.x*delta_pos_ned.x + delta_pos_ned.y*delta_pos_ned.y + delta_pos_ned.z*delta_pos_ned.z < switching_distance*switching_distance && t1_-t0_ > 0.1) {
				waypoint_copy(WP_GOAL, WP_WP4_rand8);
				waiting=false;
			}
		}
		else if (WP_EQUALS(WP_GOAL, WP_WP4_rand8)) {
			// Single WP			
			//psi_ref = atan2(waypoints[WP_WP4].enu_f.x-waypoints[WP_WP3].enu_f.x, waypoints[WP_WP4].enu_f.y-waypoints[WP_WP3].enu_f.y);
			
			// Two consecutive WPs	
			psi_ref = atan2(waypoints[WP_WP4].enu_f.x-waypoints[WP_WP3].enu_f.x, waypoints[WP_WP4].enu_f.y-waypoints[WP_WP3].enu_f.y) - M_PI/2;
			// Two extra inputs (for consecutive WP flight) - Relative Y and X position in rotated frame
			// WP_WP4_rand8
			state_nn[20] = 0.0; // WP1_x
			state_nn[19] = -4.0; // WP1_y

			switching_distance = 4.0;

			// Use euclidean distance instead
			if ( delta_pos_ned.x*delta_pos_ned.x + delta_pos_ned.y*delta_pos_ned.y + delta_pos_ned.z*delta_pos_ned.z < switching_distance*switching_distance && t1_-t0_ > 0.1) {
				waypoint_copy(WP_GOAL, WP_WP1_rand9);
				waiting=false;
			}
		}

			///////////////////

			///////////////////

		else if (WP_EQUALS(WP_GOAL, WP_WP1_rand9)) {
			// Single WP
			//psi_ref = atan2(waypoints[WP_WP1].enu_f.x-waypoints[WP_WP4].enu_f.x, waypoints[WP_WP1].enu_f.y-waypoints[WP_WP4].enu_f.y);

			// Two consecutive WPs	
			psi_ref = atan2(waypoints[WP_WP1].enu_f.x-waypoints[WP_WP4].enu_f.x, waypoints[WP_WP1].enu_f.y-waypoints[WP_WP4].enu_f.y) - M_PI/2;
			// Two extra inputs (for consecutive WP flight) - Relative Y and X position in rotated frame
			// WP_WP1_rand9
			state_nn[20] = -0.4611; // WP1_x
			state_nn[19] = -3.0466; // WP1_y

			switching_distance = 3.081;

			// Use euclidean distance instead
			if ( delta_pos_ned.x*delta_pos_ned.x + delta_pos_ned.y*delta_pos_ned.y + delta_pos_ned.z*delta_pos_ned.z < switching_distance*switching_distance && t1_-t0_ > 0.1) {
				waypoint_copy(WP_GOAL, WP_WP2_rand10);
				waiting=false;
			}
		}
		else if (WP_EQUALS(WP_GOAL, WP_WP2_rand10)) {
			// Single WP			
			//psi_ref = atan2(waypoints[WP_WP2].enu_f.x-waypoints[WP_WP1].enu_f.x, waypoints[WP_WP2].enu_f.y-waypoints[WP_WP1].enu_f.y);
			
			// Two consecutive WPs	
			psi_ref = atan2(waypoints[WP_WP2].enu_f.x-waypoints[WP_WP1].enu_f.x, waypoints[WP_WP2].enu_f.y-waypoints[WP_WP1].enu_f.y) - M_PI/2;
			// Two extra inputs (for consecutive WP flight) - Relative Y and X position in rotated frame
			// WP_WP2_rand10
			state_nn[20] = 0.0056; // WP1_x
			state_nn[19] = -3.5454; // WP1_y

			switching_distance = 3.545;

			// Use euclidean distance instead
			if ( delta_pos_ned.x*delta_pos_ned.x + delta_pos_ned.y*delta_pos_ned.y + delta_pos_ned.z*delta_pos_ned.z < switching_distance*switching_distance && t1_-t0_ > 0.1) {
				waypoint_copy(WP_GOAL, WP_WP3_rand11);
				waiting=false;
			}
		}
		else if (WP_EQUALS(WP_GOAL, WP_WP3_rand11)) {
			// Single WP			
			//psi_ref = atan2(waypoints[WP_WP3].enu_f.x-waypoints[WP_WP2].enu_f.x, waypoints[WP_WP3].enu_f.y-waypoints[WP_WP2].enu_f.y);
			
			// Two consecutive WPs	
			psi_ref = atan2(waypoints[WP_WP3].enu_f.x-waypoints[WP_WP2].enu_f.x, waypoints[WP_WP3].enu_f.y-waypoints[WP_WP2].enu_f.y) - M_PI/2;
			// Two extra inputs (for consecutive WP flight) - Relative Y and X position in rotated frame
			// WP_WP3_rand11
			state_nn[20] = -0.0079; // WP1_x
			state_nn[19] = -3.9611; // WP1_y

			switching_distance = 3.961;

			// Use euclidean distance instead
			if ( delta_pos_ned.x*delta_pos_ned.x + delta_pos_ned.y*delta_pos_ned.y + delta_pos_ned.z*delta_pos_ned.z < switching_distance*switching_distance && t1_-t0_ > 0.1) {
				waypoint_copy(WP_GOAL, WP_WP4_rand12);
				waiting=false;
			}
		}
		else if (WP_EQUALS(WP_GOAL, WP_WP4_rand12)) {
			// Single WP			
			//psi_ref = atan2(waypoints[WP_WP4].enu_f.x-waypoints[WP_WP3].enu_f.x, waypoints[WP_WP4].enu_f.y-waypoints[WP_WP3].enu_f.y);
			
			// Two consecutive WPs	
			psi_ref = atan2(waypoints[WP_WP4].enu_f.x-waypoints[WP_WP3].enu_f.x, waypoints[WP_WP4].enu_f.y-waypoints[WP_WP3].enu_f.y) - M_PI/2;
			// Two extra inputs (for consecutive WP flight) - Relative Y and X position in rotated frame
			// WP_WP4_rand12
			state_nn[20] = -0.0176; // WP1_x
			state_nn[19] = -3.0872; // WP1_y

			switching_distance = 3.087;

			// Use euclidean distance instead
			if ( delta_pos_ned.x*delta_pos_ned.x + delta_pos_ned.y*delta_pos_ned.y + delta_pos_ned.z*delta_pos_ned.z < switching_distance*switching_distance && t1_-t0_ > 0.1) {
				waypoint_copy(WP_GOAL, WP_WP1_rand13);
				waiting=false;
			}
		}

			///////////////////


		else if (WP_EQUALS(WP_GOAL, WP_WP1_rand13)) {
			// Single WP
			//psi_ref = atan2(waypoints[WP_WP1].enu_f.x-waypoints[WP_WP4].enu_f.x, waypoints[WP_WP1].enu_f.y-waypoints[WP_WP4].enu_f.y);

			// Two consecutive WPs	
			psi_ref = atan2(waypoints[WP_WP1].enu_f.x-waypoints[WP_WP4].enu_f.x, waypoints[WP_WP1].enu_f.y-waypoints[WP_WP4].enu_f.y) - M_PI/2;
			// Two extra inputs (for consecutive WP flight) - Relative Y and X position in rotated frame
			// WP_WP1_rand13
			state_nn[20] = 0.0127; // WP1_x
			state_nn[19] = -3.02; // WP1_y

			switching_distance = 3.02;

			// Use euclidean distance instead
			if ( delta_pos_ned.x*delta_pos_ned.x + delta_pos_ned.y*delta_pos_ned.y + delta_pos_ned.z*delta_pos_ned.z < switching_distance*switching_distance && t1_-t0_ > 0.1) {
				waypoint_copy(WP_GOAL, WP_WP2_rand14);
				waiting=false;
			}
		}
		else if (WP_EQUALS(WP_GOAL, WP_WP2_rand14)) {
			// Single WP			
			//psi_ref = atan2(waypoints[WP_WP2].enu_f.x-waypoints[WP_WP1].enu_f.x, waypoints[WP_WP2].enu_f.y-waypoints[WP_WP1].enu_f.y);
			
			// Two consecutive WPs	
			psi_ref = atan2(waypoints[WP_WP2].enu_f.x-waypoints[WP_WP1].enu_f.x, waypoints[WP_WP2].enu_f.y-waypoints[WP_WP1].enu_f.y) - M_PI/2;
			// Two extra inputs (for consecutive WP flight) - Relative Y and X position in rotated frame
			// WP_WP2_rand14
			state_nn[20] = -0.1778; // WP1_x
			state_nn[19] = -3.2543; // WP1_y

			switching_distance = 3.259;

			// Use euclidean distance instead
			if ( delta_pos_ned.x*delta_pos_ned.x + delta_pos_ned.y*delta_pos_ned.y + delta_pos_ned.z*delta_pos_ned.z < switching_distance*switching_distance && t1_-t0_ > 0.1) {
				waypoint_copy(WP_GOAL, WP_WP3_rand15);
				waiting=false;
			}
		}
		else if (WP_EQUALS(WP_GOAL, WP_WP3_rand15)) {
			// Single WP			
			//psi_ref = atan2(waypoints[WP_WP3].enu_f.x-waypoints[WP_WP2].enu_f.x, waypoints[WP_WP3].enu_f.y-waypoints[WP_WP2].enu_f.y);
			
			// Two consecutive WPs	
			psi_ref = atan2(waypoints[WP_WP3].enu_f.x-waypoints[WP_WP2].enu_f.x, waypoints[WP_WP3].enu_f.y-waypoints[WP_WP2].enu_f.y) - M_PI/2;
			// Two extra inputs (for consecutive WP flight) - Relative Y and X position in rotated frame
			// WP_WP3_rand15
			state_nn[20] = 0.1553; // WP1_x
			state_nn[19] = -3.0323; // WP1_y

			switching_distance = 3.036;

			// Use euclidean distance instead
			if ( delta_pos_ned.x*delta_pos_ned.x + delta_pos_ned.y*delta_pos_ned.y + delta_pos_ned.z*delta_pos_ned.z < switching_distance*switching_distance && t1_-t0_ > 0.1) {
				waypoint_copy(WP_GOAL, WP_WP4_rand16);
				waiting=false;
			}
		}
		else if (WP_EQUALS(WP_GOAL, WP_WP4_rand16)) {
			// Single WP			
			//psi_ref = atan2(waypoints[WP_WP4].enu_f.x-waypoints[WP_WP3].enu_f.x, waypoints[WP_WP4].enu_f.y-waypoints[WP_WP3].enu_f.y);
			
			// Two consecutive WPs	
			psi_ref = atan2(waypoints[WP_WP4].enu_f.x-waypoints[WP_WP3].enu_f.x, waypoints[WP_WP4].enu_f.y-waypoints[WP_WP3].enu_f.y) - M_PI/2;
			// Two extra inputs (for consecutive WP flight) - Relative Y and X position in rotated frame
			// WP_WP4_rand16
			state_nn[20] = 0.3222; // WP1_x
			state_nn[19] = -3.7887; // WP1_y

			switching_distance = 3.802;

			// Use euclidean distance instead
			if ( delta_pos_ned.x*delta_pos_ned.x + delta_pos_ned.y*delta_pos_ned.y + delta_pos_ned.z*delta_pos_ned.z < switching_distance*switching_distance && t1_-t0_ > 0.1) {
				waypoint_copy(WP_GOAL, WP_WP1_rand1);
				waiting=false;
			}
		}


	}


	state_nn[8] = att.psi-psi_ref;
	
	// ensure that -pi < psi < pi
	while (state_nn[8] > M_PI) {
		state_nn[8] -= 2*M_PI;
	}
	while (state_nn[8] < -M_PI) {
		state_nn[8] += 2*M_PI;
	}

	// neural network input 
	state_nn[0] = waypoint_body.x;
	state_nn[1] = waypoint_body.y;
	state_nn[2] = waypoint_body.z;
	
	state_nn[3] = vel_body.x;
	state_nn[4] = vel_body.y;
	state_nn[5] = vel_body.z;
	
	state_nn[6] = att.phi;
	state_nn[7] = att.theta;
	//state_nn[8] = att.psi;

	state_nn[9] = unbiased_p;
	state_nn[10] = unbiased_q;
	state_nn[11] = unbiased_r;

	state_nn[12] = nn_rpm_obs[0];
	state_nn[13] = nn_rpm_obs[1];
	state_nn[14] = nn_rpm_obs[2];
	state_nn[15] = nn_rpm_obs[3];

	state_nn[16] = Mx_measured - Mx_modeled;
	state_nn[17] = My_measured - My_modeled;
	state_nn[18] = Mz_measured - Mz_modeled;
	
	// calcuate neural network output
	nn_control(state_nn, control_nn);

	// Artificially limit the RPM commands
	/*float omega_max_artificial_limit = 11200;
 	for (int i = 1; i < 4; i++)
        {if (control_nn[i] > omega_max_artificial_limit){
	control_nn[i] = omega_max_artificial_limit;
	}
	}*/
}


/*
void external_vision_update(uint8_t *buf)
{
  if (DL_EXTERNAL_VISION_ac_id(buf) != AC_ID) { return; } // not for this aircraft
  
  uint32_t stamp = get_sys_time_usec();
  
  float enu_x = DL_EXTERNAL_VISION_enu_x(buf);
  float enu_y = DL_EXTERNAL_VISION_enu_y(buf);
  float enu_z = DL_EXTERNAL_VISION_enu_z(buf);

  float quat_i = DL_EXTERNAL_VISION_quat_i(buf);
  float quat_x = DL_EXTERNAL_VISION_quat_x(buf);
  float quat_y = DL_EXTERNAL_VISION_quat_y(buf);
  float quat_z = DL_EXTERNAL_VISION_quat_z(buf);

  struct FloatQuat orient;
  struct FloatEulers orient_eulers;

  orient.qi = quat_i;
  orient.qx = quat_y;   //north
  orient.qy = -quat_x;  //east
  orient.qz = -quat_z;  //down

  float_eulers_of_quat(&orient_eulers, &orient);
  orient_eulers.psi += 90.0/57.6 - 33/57.6;

  ev_pos[0] = enu_y;
  ev_pos[1] = enu_x;
  ev_pos[2] = -enu_z;
  ev_att[0] = orient_eulers.phi;
  ev_att[1] = orient_eulers.theta;
  ev_att[2] = orient_eulers.psi;

  //printf("EV UPDATE \n");
  //printf("time = %d ", stamp);
  //printf("pos = %f, %f, %f ", enu_y, enu_x, -enu_z);
  //printf("att = %f, %f, %f \n", orient_eulers.phi*57.297795f, orient_eulers.theta*57.297795f, orient_eulers.psi*57.297795f);
}
*/



