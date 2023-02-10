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
#define WP_EQUALS(_wp1, _wp2) WaypointX(_wp1)==WaypointX(_wp2) && WaypointY(_wp1)==WaypointY(_wp2)

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

// sample_time = 1/sample_frequency
float sample_time = 1.0/512.0;

int count = 0;


void gcnet_init(void)
{
	// keep track of time
	//t0 = get_sys_time_float();

	// ABI messaging for reading rpm
	AbiBindMsgRPM(RPM_SENSOR_ID, &rpm_read_ev, rpm_read_cb);

	// Initialize filters (cutoff frequency=8.0)
  	float tau_ = 1.0 / (2.0 * M_PI * 8.0);				// tau = 1/(2*pi*cutoff_frequency)

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

struct NedCoor_f waypoint_ned;


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
	//struct NedCoor_f *pos   = stateGetPositionNed_f();
	ENU_OF_TO_NED(waypoint_ned, waypoints[WP_GOAL].enu_f);

	struct FloatVect3 delta_pos_ned = {
		waypoint_ned.x - ekf_X[0],
		waypoint_ned.y - ekf_X[1],
		waypoint_ned.z - ekf_X[2]
	};

	struct FloatVect3 waypoint_body;
	float_quat_vmult(&waypoint_body, &quat, &delta_pos_ned);

	// get velocity in body frame
	//struct NedCoor_f *vel = stateGetSpeedNed_f();
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

	Mx_modeled = k_p*( w1*w1 - w2*w2 - w3*w3 + w4*w4) + k_pv*vby;
	My_modeled = k_q*( w1*w1 + w2*w2 - w3*w3 - w4*w4) + k_qv*vbx;
	Mz_modeled = k_r1*(-w1 + w2 - w3 + w4) + k_r2*(-d_w1 + d_w2 - d_w3 + d_w4) - k_rr*r_;

/*
	//TEMPORARY FOR TEST
	psi_ref = atan2(waypoints[WP_WP2].enu_f.x-waypoints[WP_WP1].enu_f.x, waypoints[WP_WP2].enu_f.y-waypoints[WP_WP1].enu_f.y);
	dist_to_waypoint = sqrtf(delta_pos_ned.x*delta_pos_ned.x + delta_pos_ned.y*delta_pos_ned.y + delta_pos_ned.z*delta_pos_ned.z);
	//dist_to_waypoint < .3
	if (active && count<8) {
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

#define SWITCHING_DISTANCE  1.2

	if (autopilot_get_mode() == AP_MODE_ATTITUDE_DIRECT) {
		if (WP_EQUALS(WP_GOAL, WP_WP1)) {
			psi_ref = atan2(waypoints[WP_WP1].enu_f.x-waypoints[WP_WP4].enu_f.x, waypoints[WP_WP1].enu_f.y-waypoints[WP_WP4].enu_f.y);
			
			// set next waypoint once drone passes the plane perpendicular to the waypoint (psi_ref + pi/4 direction)
			float normal_x = cos(psi_ref + M_PI/4);
			float normal_y = sin(psi_ref + M_PI/4);		
			if (-delta_pos_ned.x*normal_x - delta_pos_ned.y*normal_y > -SWITCHING_DISTANCE) {
				waypoint_copy(WP_GOAL, WP_WP2);
			}
		}
		else if (WP_EQUALS(WP_GOAL, WP_WP2)) {
			psi_ref = atan2(waypoints[WP_WP2].enu_f.x-waypoints[WP_WP1].enu_f.x, waypoints[WP_WP2].enu_f.y-waypoints[WP_WP1].enu_f.y);
			
			// set next waypoint once drone passes the plane perpendicular to the waypoint (psi_ref + pi/4 direction)
			float normal_x = cos(psi_ref + M_PI/4);
			float normal_y = sin(psi_ref + M_PI/4);		
			if (-delta_pos_ned.x*normal_x - delta_pos_ned.y*normal_y > -SWITCHING_DISTANCE) {
				waypoint_copy(WP_GOAL, WP_WP3);
			}
		}
		else if (WP_EQUALS(WP_GOAL, WP_WP3)) {
			psi_ref = atan2(waypoints[WP_WP3].enu_f.x-waypoints[WP_WP2].enu_f.x, waypoints[WP_WP3].enu_f.y-waypoints[WP_WP2].enu_f.y);
			
			// set next waypoint once drone passes the plane perpendicular to the waypoint (psi_ref + pi/4 direction)
			float normal_x = cos(psi_ref + M_PI/4);
			float normal_y = sin(psi_ref + M_PI/4);		
			if (-delta_pos_ned.x*normal_x - delta_pos_ned.y*normal_y > -SWITCHING_DISTANCE) {
				waypoint_copy(WP_GOAL, WP_WP4);
			}
			
		}
		else if (WP_EQUALS(WP_GOAL, WP_WP4)) {
			psi_ref = atan2(waypoints[WP_WP4].enu_f.x-waypoints[WP_WP3].enu_f.x, waypoints[WP_WP4].enu_f.y-waypoints[WP_WP3].enu_f.y);
			
			// set next waypoint once drone passes the plane perpendicular to the waypoint (psi_ref + pi/4 direction)
			float normal_x = cos(psi_ref + M_PI/4);
			float normal_y = sin(psi_ref + M_PI/4);		
			if (-delta_pos_ned.x*normal_x - delta_pos_ned.y*normal_y > -SWITCHING_DISTANCE) {
				waypoint_copy(WP_GOAL, WP_WP1);
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


// Logging
extern void gnc_net_log_header(FILE *file) {
  fprintf(file, "autopilot_mode,");
  fprintf(file, "wp_goal_x,wp_goal_y,wp_goal_z,");
  fprintf(file, "nn_out_1,nn_out_2,nn_out_3,nn_out_4,");
  fprintf(file, "nn_in_1,nn_in_2,nn_in_3,nn_in_4,nn_in_5,nn_in_6,nn_in_7,nn_in_8,nn_in_9,nn_in_10,nn_in_11,nn_in_12,nn_in_13,nn_in_14,nn_in_15,nn_in_16,nn_in_17,nn_in_18,nn_in_19,");
  fprintf(file, "Mx_measured,My_measured,Mz_measured,az_measured,");
  fprintf(file, "Mx_modeled,My_modeled,Mz_modeled,az_modeled,");
}


extern void gnc_net_log_data(FILE *file) {
  //fprintf(file, "%d,%d,%d,%d,", motor_mixing.commands[0], motor_mixing.commands[1], motor_mixing.commands[2], motor_mixing.commands[3]);
  fprintf(file, "%d,", (autopilot_get_mode()==AP_MODE_ATTITUDE_DIRECT)?(1):(0));
  fprintf(file, "%f,%f,%f,", waypoint_ned.x, waypoint_ned.y, waypoint_ned.z);
  fprintf(file, "%f,%f,%f,%f,",control_nn[0],control_nn[1],control_nn[2],control_nn[3]);
  fprintf(file, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,",state_nn[0], state_nn[1], state_nn[2], state_nn[3], state_nn[4], state_nn[5], state_nn[6], state_nn[7], state_nn[8], state_nn[9], state_nn[10], state_nn[11], state_nn[12], state_nn[13], state_nn[14], state_nn[15], state_nn[16], state_nn[17], state_nn[18]);
  fprintf(file, "%f,%f,%f,%f,", Mx_measured, My_measured, Mz_measured, az_measured);
  fprintf(file, "%f,%f,%f,%f,", Mx_modeled, My_modeled, Mz_modeled, az_modeled);
}
