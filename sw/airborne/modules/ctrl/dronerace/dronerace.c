/*
 * Copyright (C) MAVLab
 *
 * This file is part of paparazzi
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/ctrl/dronerace//dronerace.c"
 * @author MAVLab
 * Autonomous Drone Race
 */


#include "modules/ctrl/dronerace/dronerace.h"
#include "firmwares/rotorcraft/guidance/guidance_v.h"
#include "modules/sonar/sonar_bebop.h"
#include "generated/flight_plan.h"
#include "subsystems/abi.h"
#include "state.h"
#include "filter.h"
#include "control.h"
#include "ransac.h"
#include "flightplan.h"
#include "subsystems/datalink/telemetry.h"

// to know if we are simulating:
#include "generated/airframe.h"

float dt = 1.0f / 512.f;

float input_phi;
float input_theta;
float input_psi;

volatile int input_cnt = 0;
volatile float input_dx = 0;
volatile float input_dy = 0;
volatile float input_dz = 0;



///////////////////////////////////////////////////////////////////////////////////////////////////
// LOGGING

#include <stdio.h>

/** Set the default File logger path to the USB drive */
#ifndef FILE_LOGGER_PATH
  #if SIMULATE
    #define FILE_LOGGER_PATH .
  #else
    #define FILE_LOGGER_PATH /data/ftp/internal_000
  #endif
#endif

// What type of log to make during flight:
#define CHRISTOPHE_LOG 0
#define OLD_LOG 1
#define FULL_LOG 2
#define TYPE_LOG FULL_LOG


/** The file pointer */
static FILE *file_logger = NULL;

static void open_log(void) {
  #if 0 
  // Get current date/time, format is YYYY-MM-DD.HH:mm:ss
  /*
  char date_time[80];
  time_t now = time(0);
  struct tm  tstruct;
  tstruct = *localtime(&now);
  strftime(date_time, sizeof(date_time), "%Y-%m-%d_%X", &tstruct);
  */

  uint32_t counter = 0;
  char filename[512];
  // Check for available files
  sprintf(filename, "%s/%s.csv", STRINGIFY(FILE_LOGGER_PATH), "log_file");
  while ((file_logger = fopen(filename, "r"))) {
    fclose(file_logger);
    sprintf(filename, "%s/%s_%05d.csv", STRINGIFY(FILE_LOGGER_PATH), "log_file", counter);
    counter++;
  }

  printf("\n\n*** chosen filename log drone race: %s ***\n\n", filename);

  file_logger = fopen(filename, "w");

  if (file_logger != NULL) {
    if(TYPE_LOG == CHRISTOPHE_LOG) {
      fprintf(file_logger,"phi,theta,psi,vision_cnt,dx,dy,dz\n");
    }
    else if(TYPE_LOG == OLD_LOG) {
      fprintf(file_logger,"dr_state_x,dr_state_y,dr_state_vx,dr_state_vy,vision_cnt,vision_dx,vision_dy\n");
    }
    else {
      fprintf(file_logger,"phi,theta,psi,vision_cnt,ransac_buf_size,vision_dx,vision_dy,vision_dz,dr_state_x,dr_state_y,dr_state_vx,dr_state_vy,corr_x,corr_y,real_x,real_y\n");
    }
  }
  #endif
}

static void write_log(void)
{
  #if 0
  if (file_logger != 0) {

    if(TYPE_LOG == OLD_LOG) {
      fprintf(file_logger, "%f,%f,%f,%f,%d,%f,%f\n",dr_state.x, dr_state.y, dr_state.vx, dr_state.vy,
          dr_vision.cnt,dr_vision.dx,dr_vision.dy);
    }
    else if(TYPE_LOG == CHRISTOPHE_LOG) {
      fprintf(file_logger, "%f,%f,%f,%d,%f,%f,%f\n",input_phi, input_theta, input_psi, input_cnt, input_dx, input_dy, input_dz);
    }
    else {
      /*fprintf(file_logger, "%f,%f,%f,%d,%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",stateGetNedToBodyEulers_f()->phi, stateGetNedToBodyEulers_f()->theta, stateGetNedToBodyEulers_f()->psi,
                      dr_vision.cnt, dr_ransac.buf_size, dr_vision.dx, dr_vision.dy, dr_vision.dz, dr_state.x, dr_state.y, dr_state.vx, dr_state.vy,
                      dr_state.x+dr_ransac.corr_x, dr_state.y+dr_ransac.corr_y, stateGetPositionNed_f()->x, stateGetPositionNed_f()->y);
    */}
  }
  #endif
}

///////////////////////////////////////////////////////////////////////////////////////////////////
// TELEMETRY


// sending the divergence message to the ground station:
static void send_dronerace(struct transport_tx *trans, struct link_device *dev)
{
  float fx = dr_state.time;
  float fy = dr_state.x; // Flows
  float fz = dr_state.y;

  float cx = dr_state.vx; // Covariance
  float cy = dr_state.vy;
  float cz;
  
  float ex = 0;
  float ey = 0;
  float ez = 0;

  int32_t gc = dr_vision.cnt; // Error
  float gx = dr_vision.dx; // Vision
  float gy = dr_vision.dy;
  float gz = dr_vision.dz;

  //float ez = POS_FLOAT_OF_BFP(guidance_v_z_sp);

  float ix = dr_state.assigned_gate_index; // Error
  int32_t iy =  0; //dr_ransac.buf_size; // Error
  int32_t iz = 0;

  
  pprz_msg_send_OPTICAL_FLOW_HOVER(trans, dev, AC_ID, &fx, &fy, &fz,
                                   &cx, &cy, &cz,
                                   &gx, &gy, &gz,
                                   &ex, &ey, &ez,
                                   &ix, &iy, &iz); 
/*
  pprz_msg_send_DRONE_RACE(trans, dev, AC_ID,
		  &fx, &fy, &fz, &cx, &cy,
		  &gc, &gx, &gy, &gz,
		  &ix, &iy);
*/
}


// ABI receive gates!
#ifndef DRONE_RACE_ABI_ID
// Receive from: 33 (=onboard vision) 34 (=jevois) or 255=any
#define DRONE_RACE_ABI_ID ABI_BROADCAST
#endif

static abi_event gate_detected_ev;


static void gate_detected_cb(uint8_t sender_id __attribute__((unused)), int32_t cnt, float dx, float dy, float dz, float vx __attribute__((unused)), float vy __attribute__((unused)), float vz __attribute__((unused)))
{
  // Logging
  input_cnt = cnt;
  input_dx = dx;
  input_dy = dy;
  input_dz = dz;

}

void dronerace_init(void)
{
  // Receive vision
  //AbiBindMsgRELATIVE_LOCALIZATION(DRONE_RACE_ABI_ID, &gate_detected_ev, gate_detected_cb);

  // Send telemetry
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_OPTICAL_FLOW_HOVER, send_dronerace);

  // Start Logging
  open_log();

  // Compute waypoints
  dronerace_enter();
  /*
  float phi0 = 0;
  float phi1 = 0;
  float invert_time = 0;

  float x0[2] = {-3.0, -3.0}; //GPS_take;
  float v0[2] = {2.0, 0.0}; // needs non zero initial velocity, not sure why

  float xd[2] = {-0.0, 0.0};
  float vd[2] = {3.0, 0};

  float xt[2] = {0.0, 0.0};
  float vt[2] = {0.0, 0.0};

  float psi_init = stateGetNedToBodyEulers_f()->psi;
  
  find_optimal(x0, v0, xd, vd, xt, vt, &phi0, &phi1, &invert_time, psi_init);

  printf("init check: phi0: %f, phi1: %f, dt: %f\n", phi0, phi1, invert_time);
  printf("init reaching set: x: %f, y: %f, vx: %f, vy: %f\n", xt[0], xt[1], vt[0], vt[1]); // TODO: fix ptr
  */

}


float psi0 = 0;
void dronerace_enter(void)
{
  psi0 = stateGetNedToBodyEulers_f()->psi;
  filter_reset();
  control_reset();

  for (int i=0;i<MAX_GATES;i++)
  {
    struct EnuCoor_f enu_g = {.x=gates[i].y, .y=gates[i].x, .z=-gates[i].z};
    struct EnuCoor_f enu_w = {.x=waypoints_dr[i].y, .y=waypoints_dr[i].x, .z=-waypoints_dr[i].z};
    if (gates[i].type == VIRTUAL) {
      enu_g.x = -10;
      enu_g.y = 0;
    }
    waypoint_set_enu( WP_G1+i, &enu_g);
    waypoint_set_enu( WP_p1+i, &enu_w);
    //printf("Moved %f %f \n", enu_g.x, enu_g.y);
  }
}

#ifndef PREDICTION_BIAS_PHI
#define PREDICTION_BIAS_PHI        0.0f
#endif

#ifndef PREDICTION_BIAS_THETA
#define PREDICTION_BIAS_THETA      0.0f
#endif

void dronerace_periodic(void)
{

  float phi_bias = RadOfDeg(PREDICTION_BIAS_PHI);
  float theta_bias = RadOfDeg(PREDICTION_BIAS_THETA);

  input_phi = stateGetNedToBodyEulers_f()->phi - phi_bias;
  input_theta = stateGetNedToBodyEulers_f()->theta - theta_bias;
  input_psi = stateGetNedToBodyEulers_f()->psi - psi0;

  dr_state.phi = input_phi;
  dr_state.psi = input_psi;
  dr_state.theta = input_theta;
  
  filter_predict(input_phi, input_theta, input_psi, dt);
  

  // Vision update
  // printf("input count, vision count: %d, %d\n", input_cnt, dr_vision.cnt);
  /*
  if (input_cnt > dr_vision.cnt) {
    dr_vision.cnt = input_cnt;
    dr_vision.dx = input_dx;
    dr_vision.dy = input_dy;
    dr_vision.dz = input_dz;

    filter_correct();

    flightplan_list();
  }
  */
  //printf("before write log\n");
  write_log();
  //printf("after write log\n");

  {
    struct NedCoor_f target_ned;
    target_ned.x = dr_fp.gate_y;
    target_ned.y = dr_fp.gate_x;
    target_ned.z = -dr_fp.gate_z;

    if (autopilot.mode_auto2 == AP_MODE_MODULE) {
      ENU_BFP_OF_REAL(navigation_carrot, target_ned);
      ENU_BFP_OF_REAL(navigation_target, target_ned);
    }
  }


  // Show position on the map
  /*
  struct NedCoor_f pos;
  pos.x = dr_state.x;
  pos.y = dr_state.y;
  pos.z = sonar_bebop.distance; // Hardcoded push of sonar to the altitude
  //stateSetPositionNed_f(&pos);
*/
}

void dronerace_set_rc(UNUSED float rt, UNUSED float rx, UNUSED float ry, UNUSED float rz)
{
}

void dronerace_get_cmd(float* alt, float* phi, float* theta, float* psi_cmd, float *var_time, float *vel)
{

  control_run(dt);
  
  *phi = dr_control.phi_cmd;
  *theta = dr_control.theta_cmd;
  *psi_cmd = dr_control.psi_cmd + psi0;
  *alt = - dr_control.z_cmd;
  *var_time = dr_state.time;
  *vel = dr_state.vy;
  // guidance_v_z_sp = POS_BFP_OF_REAL(dr_control.z_cmd);
}


float K_ff_theta = 14.0/57 / 5;   // rad to fly at (e.g. 10 deg = 5 m/s)
float K_p_theta = 6.0 / 57;       // m/s to radians


#define OPT_ITER 1000
void find_optimal(float *x0, float *v0, 
                  float *xd, float *vd,
                  float *xt, float *vt,
                  float *phi0, float *phi1, 
                  float *switch_time, float psi_init)
{
  // flip the drone at 55% of total time
  float t1 = (xd[0]-x0[0]) / ((0.0*v0[0] + 1.0*vd[0])) * 0.55;
  *switch_time = t1;

  // set initial step size as 4 degrees
  float dphi0 = 4.0/57; 
  float dphi1 = 4.0/57;

  float maxbank = 25.0/57.0;

  for (int i = 0; i < OPT_ITER; i++) {
    
    float c1pstep = (*phi0 + dphi0);
    float c2pstep = (*phi1 + dphi1);

    float c0  = pathPredict(x0, v0, xd, vd, *phi0,   *phi1, t1, xt, vt, psi_init);
    float c1p = pathPredict(x0, v0, xd, vd, c1pstep, *phi1, t1, xt, vt, psi_init);  // phi0 + gradient0
    float c2p = pathPredict(x0, v0, xd, vd, *phi0, c2pstep, t1, xt, vt, psi_init);  // phi1 + gradient1

    // printf("iter: %d, c1step: %f,c1p: %f, c2pstep: %f, c2p: %f\n", i, c1pstep, c1p, c2pstep, c2p);
    
    if (c0 < c1p) {
      // calc minus step size
      float c1mstep = (*phi0 - dphi0);
      float c1m = pathPredict(x0, v0, xd, vd, c1mstep, *phi1, t1, xt, vt, psi_init);

      if (c1m < c0) {
        // reverse sign
        dphi0 = -dphi0;
      }
      else {
        // no need of reversing gradient, just reduce it
        dphi0 = dphi0 / 2;
      }

    }
    if (c0 < c2p) {
      // calc minus step size
      float c2mstep = (*phi1 - dphi1);
      float c2m = pathPredict(x0, v0, xd, vd, *phi0, c2mstep, t1, xt, vt, psi_init);
      
      if (c2m < c0) {
        // reverse sign
        dphi1 = -dphi1;
      }
      else {
        // no need of reversing gradient, just reduce it
        dphi1 = dphi1 / 2;
      }
    }

    if (c1p < c2p) {
      // see if c1p can become better
      *phi0 = *phi0 + dphi0;
    }
    else {
      *phi1 = *phi1 + dphi1;
    }

    
    // saturate @ 57
    if (*phi0 > maxbank) {
      *phi0 = maxbank;
    }
    if (*phi0 < -maxbank) {
      *phi0 = -maxbank;
    }

    // saturate @ 57
    if (*phi1 > maxbank) {
      *phi1 = maxbank;
    }
    if (*phi1 < -maxbank) {
      *phi1 = -maxbank;
    }
  
  }

  // send final phi0 and phi1

}

float pathPredict(float x0[2], float v0[2],
                  float xd[2], float vd[2], 
                  float phi0, float phi1, float t1, 
                  float *xt, float *vt, float psi_init)
{
  float deltat = 0.01;
  float simtime = 0;
  float phi = 0;
  // float psi = psi_init; // fix yaw to nothing at the moment
  float flapping = 0.85;

  float x[2];
  x[0] = x0[0];
  x[1] = x0[1];
  
  float v[2]; 
  v[0] = v0[0];
  v[1] = v0[1];

  // float theta =  20 * 3.142 / 180; // +ve is correct in their frame of ref
  float maxbank = 25.0/57.0;
  
  // Predict until passing the gate
  while ((x[0] - xd[0]) < 0.1) {

    float theta = K_p_theta * (vd[0] - v[0]) +  K_ff_theta * vd[0];
    // printf("theta: %f\n", theta);
    if (theta > maxbank) {
      theta = maxbank;
    }
    if (theta < -maxbank) {
      theta = -maxbank; 
    }
    
    if (simtime < t1) {
      phi = phi0;
    }
    if (simtime >= t1) {
      phi = phi1;
    }

    float thrust = 9.81/ (cos(flapping * phi) * cos(flapping * theta)); 
    
    float ax = (cos(phi) * sin(theta) * cos(psi_init) + sin(phi) * sin(psi_init)) * thrust - v[0] * KDX;
    float ay = (cos(phi) * sin(theta) * sin(psi_init) - sin(phi) * cos(psi_init)) * thrust - v[1] * KDY;

    // Simulation
    simtime = simtime + deltat;
    v[0] = v[0] + ax * deltat;
    x[0] = x[0] + v[0] * deltat; 

    v[1] = v[1] + ay * deltat;
    x[1] = x[1] + v[1] * deltat; 
  }
  // TODO: does it also care about x positions? this is only y
  // Position at the Gate
  xt[0] = x[0];
  xt[1] = x[1];

  vt[0] = v[0];
  vt[1] = v[1];
  // printf("reaching set: x: %f, y: %f, vx: %f, vy: %f\n", xt[0], xt[1], vt[0], vt[1]);
  // penalize x more than y
  float cost = fabsf(x[1] - xd[1])*fabsf(x[1] - xd[1]) * 10 + fabsf(v[1]- vd[1])*fabsf(v[1] - vd[1]);

  return cost; 
}

// void get_state(dronerace_state_struct *var) 
// {
//   *var = dr_state;
// }