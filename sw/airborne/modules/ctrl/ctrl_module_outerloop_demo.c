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
 * @file modules/ctrl/ctrl_module_outerloop_demo.h
 * @brief example empty controller
 *
 */

#include "modules/ctrl/ctrl_module_outerloop_demo.h"
#include "state.h"
#include "autopilot.h"
#include <stdio.h>
#include "./dronerace/dronerace.h"
#include "./dronerace/filter.h"
// Own Variables
inline float z_i =0;
struct ctrl_module_demo_struct dr_ctrl = {0};

// Settings
float comode_time = 0;


/** The file pointer */
FILE *file_logger_t2 = NULL;

static void open_log(void) 
{
  char filename[512];
  // Check for available files
  sprintf(filename, "%s/%s.csv", STRINGIFY(FILE_LOGGER_PATH), "altitude_log");
  printf("\n\n*** chosen filename log drone race: %s ***\n\n", filename);
  file_logger_t2 = fopen(filename, "w+"); 
}


////////////////////////////////////////////////////////////////////
// Call our controller
// Implement own Horizontal loops
void guidance_h_module_init(void)
{
  dronerace_init();
  float psi_init = stateGetNedToBodyEulers_f()->psi;
} 



void guidance_h_module_enter(void)
{ 
  // Store current heading
  #ifdef LOG
  open_log();
  #endif
  dr_ctrl.cmd.psi = stateGetNedToBodyEulers_i()->psi;

  // Convert RC to setpoint
  //stabilization_attitude_read_rc_setpoint_eulers(&ctrl.rc_sp, autopilot.in_flight, false, false);
  dronerace_enter();
}

void guidance_h_module_read_rc(void)
{
  //stabilization_attitude_read_rc_setpoint_eulers(&ctrl.rc_sp, autopilot.in_flight, false, false);
}


float alt = 0;
float roll = 0;
float pitch = 0;
float yaw = 0;
void guidance_h_module_run(bool in_flight)
{
  // YOUR NEW HORIZONTAL OUTERLOOP CONTROLLER GOES HERE
  // ctrl.cmd = CallMyNewHorizontalOuterloopControl(ctrl);

  dronerace_get_cmd(&alt, &roll, &pitch, &yaw);

  dr_ctrl.cmd.phi   = ANGLE_BFP_OF_REAL(roll);
  dr_ctrl.cmd.theta = ANGLE_BFP_OF_REAL(pitch); //-ANGLE_BFP_OF_REAL(5*3.142/180); //pitch is calculated to control altitude below 
  dr_ctrl.cmd.psi   = ANGLE_BFP_OF_REAL(yaw);   // stateGetNedToBodyEulers_f()->psi;//

  stabilization_attitude_set_rpy_setpoint_i(&(dr_ctrl.cmd));
  stabilization_attitude_run(in_flight);

  // Alternatively, use the indi_guidance and send AbiMsgACCEL_SP to it instead of setting pitch and roll
}

#if 1
void guidance_v_module_init(void)
{
  // initialization of your custom vertical controller goes here
}

// Implement own Vertical loops
void guidance_v_module_enter(void)
{
    z_i=0;
  // your code that should be executed when entering this vertical mode goes here
}



// Altitude control gains
#define Z_ALPHA 0.9
#define KP_ALT 0.45
#define KD_ALT 0.04
#define KI_ALT 0//0.01

#define KP_Z 3.0
#define KP_VZ 3.0
#define KP_VZDOT 0.1

#define HOVERTHRUST 0.55
#define FIXEDTHRUST 0.57
#define MAXPITCH 20* PI/180.0
float prev_meas_z = 0; 
float z_cmd;
float z_measured;
float zv_measured;
float zv_dot_measured;
float prev_meas_zv = 0;
float zv_command; 
float theta_cmd;
float zv_dot_command; 
float est_state_vz= 0;
float est_state_z;
float thrust_cmd = 0; 
float m = 0.420; //mass of bebop 


void guidance_v_module_run(bool in_flight)
{ 

// Altitude control old
  z_cmd = -1.75; 
  z_measured = dr_state.z;//stateGetPositionUtm_f()->alt; //TODO check sign (may be MSL)
  zv_measured = (z_measured -prev_meas_z)*512.; 
  
  est_state_vz = zv_measured;//Z_ALPHA * est_state_vz + (1-Z_ALPHA) * zv_measured;
  est_state_z = z_measured;// Z_ALPHA * est_state_z + (1-Z_ALPHA) * z_measured;
  prev_meas_z = z_measured;
  z_i+=(z_cmd-est_state_z)/512.;
  
  thrust_cmd = -(KP_ALT *(z_cmd -est_state_z) - KD_ALT * est_state_vz + KI_ALT*z_i) + HOVERTHRUST /  (cosf(dr_state.phi)*cosf(dr_state.theta));

  if(thrust_cmd>0.8){
    thrust_cmd=0.8;
  }
  float nominal = radio_control.values[RADIO_THROTTLE];
  float flap = 0.85;
  stabilization_cmd[COMMAND_THRUST] = thrust_cmd*9125.;// nominal / (cosf(dr_state.phi * flap) * cosf(dr_state.theta * flap));
  // printf("z_measured: %f, est_state_z:%f, zv_measured: %f,nominal: %f,thrust_cmd: %f\n",z_measured,est_state_z,zv_measured,nominal,thrust_cmd);
  #ifdef LOG
  fprintf(file_logger_t2, "%f, %f, %f,%f, %f\n",z_measured,est_state_z,zv_measured,est_state_vz,thrust_cmd);
  #endif
}
#endif