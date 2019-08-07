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
#include "subsystems/imu.h"
#include "subsystems/datalink/telemetry.h"
#include "firmwares/rotorcraft/stabilization.h"
#include "boards/bebop/actuators.h"

// to know if we are simulating:
#include "generated/airframe.h"

#define MAXTIME 8.0
#define FILE_LOGGER_PATH /data/ftp/internal_000

float dt = 1.0f / 512.f;

float input_phi;
float input_theta;
float input_psi;

volatile int input_cnt = 0;
volatile float input_dx = 0;
volatile float input_dy = 0;
volatile float input_dz = 0;

#include <stdio.h>
// Variables
struct dronerace_control_struct dr_control;
/** The file pointer */
static FILE *file_logger_t = NULL;

static void open_log(void) 
{
  char filename[512];
  // Check for available files
  sprintf(filename, "%s/%s.csv", STRINGIFY(FILE_LOGGER_PATH), "lllllog_file");
  printf("\n\n*** chosen filename log drone race: %s ***\n\n", filename);
  file_logger_t = fopen(filename, "w+"); 
}


// float est_state_roll = 0;
// float est_state_pitch = 0;
// float est_state_yaw = 0;

struct FloatEulers *rot; 
struct NedCoor_f *pos;   
struct FloatRates *rates;
static void write_log(void)
{ 
  static uint32_t counter = 0; 
  if (file_logger_t != 0) {

    rot = stateGetNedToBodyEulers_f();
    pos = stateGetPositionNed_f();
    rates = stateGetBodyRates_f();
    
    /*
    fprintf(file_logger_t, "%d,%f,%f,%f,%f,%f,%f,%f,%f,%f\n", counter, 
    imu.accel.x/1024.0, imu.accel.y/1024.0, imu.accel.z/1024.0,
    imu.gyro.p/4096.0, imu.gyro.q/4096.0, imu.gyro.r/4096.0,
    est_state_roll, est_state_pitch, est_state_yaw);*/
    counter++;
  }  
  
}

///////////////////////////////////////////////////////////////////////////////////////////////////
// TELEMETRY


// sending the divergence message to the ground station:
static void send_dronerace(struct transport_tx *trans, struct link_device *dev)
{
  
  // float est_roll = est_state_roll*(180./3.1416);
  // float est_pitch = est_state_pitch*(180./3.1416);
  // float est_yaw = est_state_yaw*(180./3.1416);

  //float ez = POS_FLOAT_OF_BFP(guidance_v_z_sp);

 

  
  // pprz_msg_send_OPTICAL_FLOW_HOVER(trans, dev, AC_ID,&est_roll,&est_pitch,&est_yaw);
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

bool start_log = 0;
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
  start_log = 1;
}

#ifndef PREDICTION_BIAS_PHI
#define PREDICTION_BIAS_PHI        0.0f
#endif

#ifndef PREDICTION_BIAS_THETA
#define PREDICTION_BIAS_THETA      0.0f
#endif

void dronerace_periodic(void)
{

  float phi_bias   = RadOfDeg(PREDICTION_BIAS_PHI);
  float theta_bias = RadOfDeg(PREDICTION_BIAS_THETA);

  input_phi   = stateGetNedToBodyEulers_f()->phi - phi_bias;
  input_theta = stateGetNedToBodyEulers_f()->theta - theta_bias;
  input_psi   = stateGetNedToBodyEulers_f()->psi - psi0;

  dr_state.phi   = input_phi;
  dr_state.psi   = input_psi;
  dr_state.theta = input_theta;
  
  filter_predict(input_phi, input_theta, input_psi, dt);
  // ahrsblah();
  if(dr_state.time < MAXTIME && start_log == 1) {
    write_log();
    
  }
  if(dr_state.time > MAXTIME) {
    start_log = 0;
    //fclose(file_logger_t);
  }
  
  struct NedCoor_f target_ned;
  target_ned.x = dr_fp.gate_y;
  target_ned.y = dr_fp.gate_x;
  target_ned.z = -dr_fp.gate_z;

  if (autopilot.mode_auto2 == AP_MODE_MODULE) {
    ENU_BFP_OF_REAL(navigation_carrot, target_ned);
    ENU_BFP_OF_REAL(navigation_target, target_ned);
  }
}

void dronerace_set_rc(UNUSED float rt, UNUSED float rx, UNUSED float ry, UNUSED float rz)
{
}

void dronerace_get_cmd(float* alt, float* phi, float* theta, float* psi_cmd)
{

  control_run(dt);
  
  *phi     = dr_control.phi_cmd;
  *theta   = dr_control.theta_cmd;
  *psi_cmd = dr_control.psi_cmd + psi0;
  *alt     = - dr_control.z_cmd;
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
  float t1 = (xd[0]-x0[0]) / ((v0[0] + vd[0])/2.0) * 0.45;
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
  float prev_theta = 0.0;
  float prev_phi = 0.0;
  float d_theta = 0.0;
  float d_phi = 0.0;

  float p_max = (400.0 / 57.0) * 0.001;
  
  // Predict until passing the gate //TODO * -sign(x0[0] - xd[0])
  while ((x[0] - xd[0]) < 0.1 ) {

    float theta = K_p_theta * (vd[0] - v[0]) +  K_ff_theta * vd[0];
    // printf("theta: %f\n", theta);
    if (theta > maxbank) {
      theta = maxbank;
    }
    if (theta < -maxbank) {
      theta = -maxbank; 
    }

    d_theta = theta - prev_theta;
    if (d_theta > p_max) {
        d_theta = p_max;
    }
    if (d_theta < -p_max) {
        d_theta = -p_max;
    }
    theta = prev_theta + d_theta;
        
    
    if (simtime < t1) {
      phi = phi0;
    }
    if (simtime >= t1) {
      phi = phi1;
    }

    d_phi = phi - prev_phi;
    if (d_phi > p_max) {
        d_phi = p_max;
    }
    if (d_phi < -p_max) {
        d_phi = -p_max;
    }
    phi = prev_phi + d_phi;


    float thrust = 9.81/ (cosf(flapping * phi) * cosf(flapping * theta));   
    float ax = (cosf(phi) * sinf(theta) * cosf(psi_init) + sinf(phi) * sinf(psi_init)) * thrust - v[0] * KDX;
    float ay = (cosf(phi) * sinf(theta) * sinf(psi_init) - sinf(phi) * cosf(psi_init)) * thrust - v[1] * KDY;

    // Simulation
    simtime = simtime + deltat;
    v[0] = v[0] + ax * deltat;
    x[0] = x[0] + v[0] * deltat; 

    v[1] = v[1] + ay * deltat;
    x[1] = x[1] + v[1] * deltat; 

    prev_theta = theta;
    prev_phi = phi;


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

#if 0
void ahrsblah() {
  float p,q,r, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z;
  float dt1 = 1.0/512.0;
  float GRAVITY = -9.81;
  float KP_AHRS = 0.2;
  float KI_AHRS = 0.004;
  
  accel_x = (double)imu.accel.x / 1024.0;
  accel_y = (double)imu.accel.y / 1024.0;
  accel_z = (double)imu.accel.z / 1024.0;

  gyro_x = (double)imu.gyro.p / 4096.0;
  gyro_y = (double)imu.gyro.q / 4096.0;
  gyro_z = (double)imu.gyro.r / 4096.0;

  float acc[3] = {accel_x, accel_y, accel_z};
  float imu_pqr[3] = {gyro_x, gyro_y, gyro_z};
  float att[3] = {est_state_roll, est_state_pitch, est_state_yaw};

  // gravity in body frame
  float gB[3] = {-sinf(att[1]) * GRAVITY, sinf(att[0]) * cosf(att[1]) * GRAVITY, cosf(att[0]) * cosf(att[1]) * GRAVITY};
  float norm_gB = sqrtf(gB[0] * gB[0] + gB[1] * gB[1] + gB[2] * gB[2]); //gB.dot(gB);
  if(norm_gB < 1.0) {
    norm_gB = fabs(GRAVITY);
  }
  float gB_scaled[3] = {gB[0] / norm_gB, gB[1] / norm_gB, gB[2] / norm_gB};  // When gravity is downwards


  // acceleration in body frame
  float norm_acc = sqrtf(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2]);  //acc.dot(acc);
  if(norm_acc < 1.0) {
    norm_acc = fabs(GRAVITY);
  }
  float acc_scaled[3] = {acc[0] / norm_acc, acc[1] / norm_acc, acc[2] / norm_acc};
  

  // error between gravity and acceleration
  float error[3] = {0, 0, 0};   // acc_scaled.cross(gB_scaled);
  error[0] = acc_scaled[1] * gB_scaled[2] - acc_scaled[2] * gB_scaled[1];
  error[1] = acc_scaled[2] * gB_scaled[0] - acc_scaled[0] * gB_scaled[2];
  error[2] = acc_scaled[0] * gB_scaled[1] - acc_scaled[1] * gB_scaled[0];
  // printf("acc_scaled[0]: %f,acc_scaled[1]:%f,acc_scaled[2]:%f,norm_acc: %f\n",acc_scaled[0],acc_scaled[1],acc_scaled[2],norm_acc);

  static float sum_error_ahrs[3] = {0, 0, 0};
  sum_error_ahrs[0] = sum_error_ahrs[0] + error[0] * dt1;
  sum_error_ahrs[1] = sum_error_ahrs[1] + error[1] * dt1;
  sum_error_ahrs[2] = sum_error_ahrs[2] + error[2] * dt1;

  // ideal KP_AHRS 0.001 KI_AHRS 0.0000001
  // complementary filter
  float filt_pqr[3] = {0,0,0};
  filt_pqr[0] = KP_AHRS * error[0] + KI_AHRS * sum_error_ahrs[0] + imu_pqr[0];
  filt_pqr[1] = KP_AHRS * error[1] + KI_AHRS * sum_error_ahrs[1] + imu_pqr[1];
  filt_pqr[2] = KP_AHRS * error[2] + KI_AHRS * sum_error_ahrs[2] + imu_pqr[2];

  // attitude estimation
  float Rmat_pqr[3] = {filt_pqr[0] + filt_pqr[1] * tanf(att[1]) * sinf(att[0]) + filt_pqr[2] * tanf(att[1]) * cosf(att[0]),
                        filt_pqr[1] * cosf(att[0]) - filt_pqr[2] * sinf(att[0]),
                        filt_pqr[1] * sinf(att[0]) / cosf(att[1]) + filt_pqr[2] * cosf(att[0]) / cosf(att[1])};

  att[0] = att[0] + Rmat_pqr[0] * dt1;
  att[1] = att[1] + Rmat_pqr[1] * dt1;
  att[2] = att[2] + Rmat_pqr[2] * dt1;

  est_state_roll  = wrapAngle(att[0]);
  est_state_pitch = wrapAngle(att[1]);
  est_state_yaw   = wrapAngle(att[2]); 
}
#endif