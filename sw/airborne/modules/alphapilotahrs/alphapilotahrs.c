/*
 * Copyright (C) Jelle Westenberger
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
 * @file "modules/alphapilotahrs/alphapilotahrs.c"
 * @author Jelle Westenberger
 * AHRS used in ALPHAPILOT2019
 */

#include "modules/alphapilotahrs/alphapilotahrs.h"
#include "subsystems/imu.h"
#include "subsystems/datalink/telemetry.h"
// #include <Eigen/Dense>
float v_est[3]={0};
float pos_est[3]={0};
float grav_body[3]={};
float accel_earth[3]={};
float accel_corr[3]={};
float est_state_roll = 0;
float est_state_pitch = 0;
float est_state_yaw = 0;
float GRAVITY = -9.81;;
double counter = 1;
float est_euler[3] = {};
float p,q,r, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z;
static void send_alphapahrs(struct transport_tx *trans, struct link_device *dev)
{
  
  float est_roll = est_state_roll*(180./3.1416);
  float est_pitch = est_state_pitch*(180./3.1416);
  float est_yaw = est_state_yaw*(180./3.1416);
  

 pprz_msg_send_AHRS_ALPHAPILOT(trans, dev, AC_ID,&est_roll,&est_pitch,&est_yaw,&accel_x,&accel_y,&accel_z,&accel_corr[0],&accel_corr[1],&accel_corr[2],&pos_est[0],&pos_est[1],&pos_est[2]);

}


void alphapilot_ahrs_init() {
register_periodic_telemetry(DefaultPeriodic,  PPRZ_MSG_ID_AHRS_ALPHAPILOT, send_alphapahrs);
  
}



void alphapilot_ahrs_periodic() {


  float dt1 = 1.0/512.0;
  
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
  est_euler[0] = est_state_roll;
  est_euler[1] = est_state_pitch;
  est_euler[2] = est_state_yaw;

  // grav_body[0]=0;
  // grav_body[1]=0;
  // grav_body[2]=GRAVITY;
  // transform2Body(est_euler,grav_body);
  // printf("grav_body[0]: %f, grav_body[1]: %f, grav_body[2]: %f\n",grav_body[0],grav_body[1],grav_body[2]);
  // double normG = sqrtf(grav_body[0]*grav_body[0]+grav_body[1]*grav_body[1]+grav_body[2]*grav_body[2]);
  accel_earth[0]=accel_x;
  accel_earth[1]=accel_y;
  accel_earth[2]=accel_z; 
  transform2Earth(est_euler,accel_earth);

  counter+=1;
  accel_corr[0]=accel_earth[0];
  accel_corr[1]=accel_earth[1];
  accel_corr[2]=accel_earth[2]-GRAVITY;


  v_est[0]+=accel_corr[0]*dt1;
  v_est[1]+=accel_corr[1]*dt1;
  v_est[2]+=accel_corr[2]*dt1;
  printf("gx: %f, gy:%f, gz:%f\n",accel_earth[0],accel_earth[1],accel_earth[2]);

  pos_est[0]+=v_est[0]*dt1;
  pos_est[1]+=v_est[1]*dt1;
  pos_est[2]+=v_est[2]*dt1;

}



void alphapilot_ahrs_event() {}



 void alphapilot_datalink_call() {}


