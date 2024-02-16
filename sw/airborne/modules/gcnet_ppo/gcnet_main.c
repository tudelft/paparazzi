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
 * @file "modules/gcnet_ppo/gcnet_main.c"
 * @brief 
 *
 */

#include "gcnet_main.h"

#include <time.h>
//#include <stdio.h>
#include "state.h"
#include "modules/radio_control/radio_control.h"
#include "autopilot.h"
#include "modules/imu/imu.h"
#include "modules/gps/gps.h"

/** For waypoints */
#include "firmwares/rotorcraft/navigation.h"
#include "generated/flight_plan.h"

/** For Filtering */
#include "filters/low_pass_filter.h"

/** ABI Messaging */
#ifndef RPM_SENSOR_ID
#define RPM_SENSOR_ID ABI_BROADCAST
#endif

#include "boards/bebop/actuators.h"
#include "modules/core/abi.h"

/** EKF */
#include "modules/ins/ins_ext_pose.h"

// nn thrust and moment model
#include "modules/gcnet_ppo/nn_thrust.h"
#include "modules/gcnet_ppo/nn_moment.h"

#define PRINT(string,...) fprintf(stderr, "[orange_avoider->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)

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

struct NedCoor_f waypoint_ned;
bool active = false;
bool at_start = false;

float control_nn[4] = {0, 0, 0, 0};

// measured/modeled moments and thrust
float Mx_measured;
float My_measured;
float Mz_measured;
float Fz_measured;

float Mx_modeled;
float My_modeled;
float Mz_modeled;
float Fz_modeled;

float Mx_nn_model;
float My_nn_model;
float Mz_nn_model;
float Fz_nn_model;

// low pass filtering
float sample_time = 1.0/512.0;
Butterworth2LowPass filter_p;
Butterworth2LowPass filter_q;
Butterworth2LowPass filter_r;
Butterworth2LowPass filter_w1;
Butterworth2LowPass filter_w2;
Butterworth2LowPass filter_w3;
Butterworth2LowPass filter_w4;
Butterworth2LowPass filter_vx;
Butterworth2LowPass filter_vy;
Butterworth2LowPass filter_vz;
Butterworth2LowPass filter_az;

// parameters from https://arxiv.org/pdf/2304.13460.pdf
float Ixx = 0.000906;
float Iyy = 0.001242;
float Izz = 0.002054;

float k_x  = 1.07933887e-05;
float k_y  = 9.65250793e-06;
float k_z  = 2.7862899e-05;
float k_w  = 4.36301076e-08;
float k_h  = 0.0625501332;
float k_p  = 1.4119331e-09;
float k_pv = -0.00797101848;
float k_q  = 1.21601884e-09;
float k_qv = 0.0129263739;
float k_r1 = 2.57035545e-06;
float k_r2 = 4.10923364e-07;
float k_rr = 0.000812932607;

bool gcnet_fake_att_mode = false;

void gcnet_activate() {
	if (gcnet_fake_att_mode) {
		gcnet_fake_att_mode=false;
	} else {
		gcnet_fake_att_mode=true;
	}
}

void go_to_start()
{
	// Sets WP_GOAL to [start_pos] defined in the nn_controller code
	struct EnuCoor_f pos;
	// start_pos is NORTH, EAST, DOWN
	pos.x = start_pos[1]; //EAST
	pos.y = start_pos[0]; //NORTH
	pos.z =-start_pos[2]; //UP
	waypoint_set_enu(WP_GOAL, &pos);
	
	// Sets Heading to align with the first gate
	nav_set_heading_rad(gate_yaw[0]);
	
	// activate at_start
	at_start = true;
	
	// set target gate to zero
	target_gate_index = 0;
}

void go_to_next_gate()
{
	at_start = false;
	
	// Sets WP_GOAL to [gate_pos[target_gate_index]] defined in the nn_controller code
	struct EnuCoor_f pos;
	// start_pos is NORTH, EAST, DOWN
	pos.x = gate_pos[target_gate_index][1]; //EAST
	pos.y = gate_pos[target_gate_index][0]; //NORTH
	pos.z =-gate_pos[target_gate_index][2]; //UP
	waypoint_set_enu(WP_GOAL, &pos);
	
	// Sets Heading to align with the first gate
	nav_set_heading_rad(gate_yaw[target_gate_index]);
	
	target_gate_index++;
	// loop back to the first gate if we reach the end
        if (target_gate_index >= NUM_GATES) {target_gate_index -= NUM_GATES;}	
}

void gcnet_init(void)
{
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
	init_butterworth_2_low_pass(&filter_vz, tau_, sample_time, 0.0);
	init_butterworth_2_low_pass(&filter_az, tau_, sample_time, 0.0);
}


void gcnet_run(void)
{
	// 1: call nn_reset once ATT mode is activated
	if (!active && autopilot_get_mode() == AP_MODE_ATTITUDE_DIRECT) {
		nn_reset();
		active = true;
	}
	if (active && autopilot_get_mode() != AP_MODE_ATTITUDE_DIRECT) {
		active = false;
	}

	if (gcnet_fake_att_mode) {active=true;}

	// 2: read the states from the EKF
	// position
	struct NedCoor_f *pos = stateGetPositionNed_f();
	// velocity
	struct NedCoor_f *vel = stateGetSpeedNed_f();
	// euler angles
	struct FloatEulers *att = stateGetNedToBodyEulers_f();
	// body rates
	struct FloatRates  *rates = stateGetBodyRates_f();
	// rpm measurements are in nn_rpm_obs
	
	// 3: get the disturbances
	// get body velocity:
	struct FloatQuat   quat;
	float_quat_of_eulers(&quat, att);
	struct FloatVect3 vel_ned = {vel->x, vel->y, vel->z};
	struct FloatVect3 vel_body;
	float_quat_vmult(&vel_body, &quat, &vel_ned);
	
	// get body acceleration (unbiased)
	float az = ekf_U[2]-ekf_X[11];
	
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
	update_butterworth_2_low_pass(&filter_vz, vel_body.z);
	update_butterworth_2_low_pass(&filter_az, az);
	
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
	float vbx = filter_vx.o[0], vby = filter_vy.o[0], vbz = filter_vz.o[0];
	float abz = filter_az.o[0];

	Mx_measured = Ixx*d_p - q_*r_*(Iyy-Izz);
	My_measured = Iyy*d_q - p_*r_*(Izz-Ixx);
	Mz_measured = Izz*d_r - p_*q_*(Ixx-Iyy);
	Fz_measured = abz;

	Mx_modeled = k_p*( w1*w1 - w2*w2 - w3*w3 + w4*w4) + k_pv*vby;
	My_modeled = k_q*( w1*w1 + w2*w2 - w3*w3 - w4*w4) + k_qv*vbx;
	Mz_modeled = k_r1*(-w1 + w2 - w3 + w4) + k_r2*(-d_w1 + d_w2 - d_w3 + d_w4) - k_rr*r_;
	Fz_modeled = -k_w*(w1*w1+w2*w2+w3*w3+w4*w4) - k_h*(vbx*vbx+vby*vby) - k_z*vbz*(w1+w2+w3+w4);
	
	// NN moment and thrust model
	float nn_moment_input[10];
	nn_moment_input[0] = 2*(w1-3000)/(12000-3000)-1;
	nn_moment_input[1] = 2*(w2-3000)/(12000-3000)-1;
	nn_moment_input[2] = 2*(w3-3000)/(12000-3000)-1;
	nn_moment_input[3] = 2*(w4-3000)/(12000-3000)-1;
	nn_moment_input[4] = vbx;
	nn_moment_input[5] = vby;
	nn_moment_input[6] = vbz;
	nn_moment_input[7] = p_;
	nn_moment_input[8] = q_;
	nn_moment_input[9] = r_;
	float nn_moment_output[3];
	nn_moment_forward(nn_moment_input, nn_moment_output);
	
	float nn_thrust_input[7];
	nn_thrust_input[0] = 2*(w1-3000)/(12000-3000)-1;
	nn_thrust_input[1] = 2*(w2-3000)/(12000-3000)-1;
	nn_thrust_input[2] = 2*(w3-3000)/(12000-3000)-1;
	nn_thrust_input[3] = 2*(w4-3000)/(12000-3000)-1;
	nn_thrust_input[4] = vbx;
	nn_thrust_input[5] = vby;
	nn_thrust_input[6] = vbz;
	float nn_thrust_output[1];
	nn_thrust_forward(nn_thrust_input, nn_thrust_output);
	
	// write to global vars
	Mx_nn_model = nn_moment_output[0];
	My_nn_model = nn_moment_output[1];
	Mz_nn_model = nn_moment_output[2];
	Fz_nn_model = nn_thrust_output[0];	
	
	float disturbances[4] = {
	 	Mx_measured - Mx_modeled - Mx_nn_model,
	 	My_measured - My_modeled - My_nn_model,
	 	Mz_measured - Mz_modeled - Mz_nn_model,
	 	Fz_measured - Fz_modeled - Fz_nn_model,
	};

	// DONT USE NN DRONE MODEL
	// float disturbances[4] = {
	//	Mx_measured - Mx_modeled,
	//	My_measured - My_modeled,
	//	Mz_measured - Mz_modeled,
	//	Fz_measured - Fz_modeled,
	//};	

	#ifndef NN_INDI_CMDS
	// 4: send state to nn_controller to obtain the rpm commands
	float world_state[16] = {
		pos->x, pos->y, pos->z,
		vel->x, vel->y, vel->z,
		att->phi, att->theta, att->psi,
		rates->p, rates->q, rates->r,
		nn_rpm_obs[0], nn_rpm_obs[1], nn_rpm_obs[2], nn_rpm_obs[3]
	};

	// compute the control command (rpms)
	if (active) {
		nn_control(world_state, disturbances, control_nn);
	}
	#endif

	#ifdef NN_INDI_CMDS
	// 4: send state to nn_controller to obtain the INDI commands
	float world_state[13] = {
		pos->x, pos->y, pos->z,
		vel->x, vel->y, vel->z,
		att->phi, att->theta, att->psi,
		rates->p, rates->q, rates->r,
		-Fz_measured
	};

	// compute the control command (rate, Thrust)
	if (active) {
		nn_control(world_state, control_nn);
		// 5: send the control command (control_nn) to the INDI controller
		float thrust_cmd = control_nn[3];

		// send thrust command to INDI
		struct FloatVect3 acc_sp_body = {0., 0., -thrust_cmd};
		// struct FloatVect3 acc_sp_ned;
		// struct FloatRMat *ned_to_body_rmat_f = stateGetNedToBodyRMat_f();
		// float_rmat_transp_vmult(&acc_sp_ned, ned_to_body_rmat_f, &acc_sp_body);

		// acc_sp_ned.x = (-thrust_cmd)*(sin(att->phi)*sin(att->psi) + sin(att->theta)*cos(att->phi)*cos(att->psi));
		// acc_sp_ned.y = (-thrust_cmd)*(-sin(att->phi)*cos(att->psi) + sin(att->psi)*sin(att->theta)*cos(att->phi));
		// acc_sp_ned.z = (-thrust_cmd)*cos(att->phi)*cos(att->theta);
	
		// acc_sp_ned.z += 9.81;

		// export to ABI
		uint8_t flag = 1;
		AbiSendMsgACCEL_SP(ACCEL_SP_FCR_ID, flag, &acc_sp_body); //USES UGLY FIX IN 

		// send angular rate command to INDI
		// --> UGLY FIX: hard coded in stabilization_indi_simple.c
	}
	#endif
}

// Logging
extern void gnc_net_log_header(FILE *file) {
  fprintf(file, "autopilot_mode,");
  fprintf(file, "target_gate_x,target_gate_y,target_gate_z,");
  fprintf(file, "Mx_measured,My_measured,Mz_measured,Fz_measured,");
  fprintf(file, "Mx_modeled,My_modeled,Mz_modeled,Fz_modeled,");
  fprintf(file, "Mx_nn_model,My_nn_model,Mz_nn_model,Fz_nn_model,");
	#ifdef NN_INDI_CMDS
	fprintf(file, "indi_p,indi_q,indi_r,indi_thrust,");
	#endif
}

extern void gnc_net_log_data(FILE *file) {
  fprintf(file, "%d,", ((autopilot_get_mode()==AP_MODE_ATTITUDE_DIRECT) || active)?(1):(0));
  fprintf(file, "%f,%f,%f,", gate_pos[target_gate_index][0], gate_pos[target_gate_index][1], gate_pos[target_gate_index][2]);
  fprintf(file, "%f,%f,%f,%f,", Mx_measured, My_measured, Mz_measured, Fz_measured);
  fprintf(file, "%f,%f,%f,%f,", Mx_modeled, My_modeled, Mz_modeled, Fz_modeled);
  fprintf(file, "%f,%f,%f,%f,", Mx_nn_model,My_nn_model,Mz_nn_model,Fz_nn_model);
	#ifdef NN_INDI_CMDS
	fprintf(file, "%f,%f,%f,%f,", control_nn[0], control_nn[1], control_nn[2], control_nn[3]);
	#endif
} 
