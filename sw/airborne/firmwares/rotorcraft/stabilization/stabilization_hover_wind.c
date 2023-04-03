/*
 * Copyright (C) Florian Sansou <florian.sansou@enac.fr>
 *  *
 * This file is part of paparazzi.
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
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */


#include "coef.h"

#include "math/pprz_algebra_float.h"
#include "state.h"
#include "generated/airframe.h"
#include "modules/radio_control/radio_control.h"
#include "modules/actuators/actuators.h"
#include "modules/core/abi.h"
#include "filters/low_pass_filter.h"
#include "wls/wls_alloc.h"
#include "math/pprz_simple_matrix.h"
#include <stdio.h>

#include "generated/flight_plan.h"

#define MAX(a,b) ((a) > (b) ? (a) : (b))

#ifndef SMEUR_TO_BARTH_PHI
#define SMEUR_TO_BARTH_PHI 0.
#endif

#ifndef SMEUR_TO_BARTH_THETA
#define SMEUR_TO_BARTH_THETA 90.
#endif

#ifndef SMEUR_TO_BARTH_PSI
#define SMEUR_TO_BARTH_PSI 0.
#endif


/*#include "modules/loggers/sdlog_chibios.h"
#include "mcu_periph/sys_time.h"
*/

float x_e[CTRL_HOVER_WIND_INPUT][1] = {{0}, {0}};
float state_vector_redu[CTRL_HOVER_WIND_INPUT][1];

float eps[CTRL_HOVER_WIND_INPUT][1];
float dot_x_e_dt[CTRL_HOVER_WIND_NUM_INTEGRATOR_STATE][1]; 
float u_integrator[CTRL_HOVER_WIND_NUM_ACT][1];
float u_prop[CTRL_HOVER_WIND_NUM_ACT][1];
float u_filter[CTRL_HOVER_WIND_NUM_ACT][1];
float u_sub[CTRL_HOVER_WIND_NUM_ACT][1];
float u[CTRL_HOVER_WIND_NUM_ACT][1];
float u_scale[CTRL_HOVER_WIND_NUM_ACT][1];

// rotation from Smeur frame to Barth frame 
static struct FloatQuat quat_smeur_2_barth;

struct FloatQuat quat_att_barth_frame;

float tf_state1[2] = {0, 0};
float tf_state2[2] = {0, 0};
float tf_state3[2] = {0, 0};
float tf_state4[2] = {0, 0};




// static inline void log_hoverwind_periodic(void);
// static inline void log_hoverwind_start(void);


void stabilization_hover_wind_init(void){

  //log_hoverwind_start();
  struct FloatEulers eul2smeurbarth = {
    RadOfDeg(SMEUR_TO_BARTH_PHI),
    RadOfDeg(SMEUR_TO_BARTH_THETA),
    RadOfDeg(SMEUR_TO_BARTH_PSI)
  };
  float_quat_of_eulers(&quat_smeur_2_barth, &eul2smeurbarth);


  x_e[0][0] = 0;
  x_e[1][0] = 0;

}

void stabilization_hover_wind_run(bool in_flight){

  #define DBG_CMD 0

  struct FloatVect3 barth_rate, smeur_rate;
  smeur_rate.x = stateGetBodyRates_f()->p;
  smeur_rate.y = stateGetBodyRates_f()->q;
  smeur_rate.z = stateGetBodyRates_f()->r;
    
  float_quat_comp(&quat_att_barth_frame, stateGetNedToBodyQuat_f(), &quat_smeur_2_barth);
  float_quat_vmult(&barth_rate, &quat_smeur_2_barth, &smeur_rate);


  #if DBG_CMD
      printf("nav target z = %f \n", -POS_FLOAT_OF_BFP(nav.target.z));
      printf("pos z = %f \n", stateGetPositionNed_f()->z);
  #endif

  struct FloatVect3 nav_target_ned;
  ENU_OF_TO_NED(nav_target_ned, nav.target) //nav.target ENU
  printf("nav target z = %f \n", nav_target_ned.z);
  eps[0][0] = stateGetPositionNed_f()->x - nav_target_ned.x; 
  eps[1][0] = stateGetPositionNed_f()->y - nav_target_ned.y;
  eps[2][0] = stateGetPositionNed_f()->z - nav_target_ned.z;
  eps[3][0] = stateGetSpeedNed_f()->x;
  eps[4][0] = stateGetSpeedNed_f()->y;
  eps[5][0] = stateGetSpeedNed_f()->z;
  eps[6][0] = quat_att_barth_frame.qx;
  eps[7][0] = quat_att_barth_frame.qz;
  eps[8][0] = barth_rate.x;
  eps[9][0] = barth_rate.y;
  eps[10][0] = barth_rate.z;

  #if DBG_CMD
    /*
    for (int i=0; i<11; i++) {
      printf("eps(%d) = %f ", i, eps[i][0]);
    }
    */
    printf("dpx = %f ,", eps[0][0]);
    printf("dpy = %f ,", eps[1][0]);
    printf("dpz = %f ,", eps[2][0]);
    printf("dvx = %f ,", eps[3][0]);
    printf("dvy = %f ,", eps[4][0]);
    printf("dvz = %f ,", eps[5][0]);
    printf("depsx = %f ,", eps[6][0]);
    printf("depsz = %f ,", eps[7][0]);
    printf("dwx = %f ,", eps[8][0]);
    printf("dwy = %f ,", eps[9][0]);
    printf("dwz = %f ", eps[10][0]);
    printf("\n");
    
  #endif

  
  MAT_MUL_c(CTRL_HOVER_WIND_NUM_INTEGRATOR_STATE, CTRL_HOVER_WIND_INPUT, 1, dot_x_e_dt, H, eps, 1./PERIODIC_FREQUENCY);
  x_e[0][0] +=  dot_x_e_dt[0][0];
  x_e[1][0] +=  dot_x_e_dt[1][0];
  u_integrator[0][0] = x_e[0][0];
  u_integrator[1][0] = x_e[1][0];
  /*
    u_integrator[0][0] = 0;
  u_integrator[1][0] = 0;
  */

  #if DBG_CMD
    for (int i=0; i<2; i++) {
      printf("u integrator(%d) = %f ", i, u_integrator[i][0]);
    }
    printf("\n");
  #endif

  MAT_MUL(CTRL_HOVER_WIND_NUM_ACT, CTRL_HOVER_WIND_INPUT, 1, u_prop, K, eps);

  #if DBG_CMD
    for (int i=0; i<CTRL_HOVER_WIND_NUM_ACT; i++) {
      printf("u prop(%d) = %f ", i, u_prop[i][0]);
    }
    printf("\n");
  #endif

  float tf1_tmp;
  float tf2_tmp;
  float tf3_tmp;
  float tf4_tmp;

  tf1_tmp = u_prop[0][0] - den[1]*tf_state1[0] - den[2]* tf_state1[1];
  u_filter[0][0] = num[0]*tf1_tmp + num[1]*tf_state1[0] + num[2]*tf_state1[1];
  tf_state1[1] = tf_state1[0];
  tf_state1[0] = tf1_tmp;

  tf2_tmp = u_prop[1][0] - den[1]*tf_state2[0] - den[2]* tf_state2[1];
  u_filter[1][0] = num[0]*tf2_tmp + num[1]*tf_state2[0] + num[2]*tf_state2[1];
  tf_state2[1] = tf_state2[0];
  tf_state2[0] = tf2_tmp;

  tf3_tmp = u_prop[2][0] - den[1]*tf_state3[0] - den[2]* tf_state3[1];
  u_filter[2][0] = num[0]*tf3_tmp + num[1]*tf_state3[0] + num[2]*tf_state3[1];
  tf_state3[1] = tf_state3[0];
  tf_state3[0] = tf3_tmp;

  tf4_tmp = u_prop[3][0] - den[1]*tf_state4[0] - den[2]* tf_state4[1];
  u_filter[3][0] = num[0]*tf4_tmp + num[1]*tf_state4[0] + num[2]*tf_state4[1];
  tf_state4[1] = tf_state4[0];
  tf_state4[0] = tf4_tmp;


  #if DBG_CMD
    for (int i=0; i<CTRL_HOVER_WIND_NUM_ACT; i++) {
      printf("u filter(%d) = %f ", i, u_filter[i][0]);
    }
    printf("\n");
  #endif

  float integrator_repart[CTRL_HOVER_WIND_NUM_ACT][1] = {{u_integrator[0][0]}, {u_integrator[0][0]}, {u_integrator[1][0]}, {u_integrator[1][0]} };
  MAT_SUB(CTRL_HOVER_WIND_NUM_ACT, 1, u_sub, integrator_repart, u_filter);
  //MAT_SUB(CTRL_HOVER_WIND_NUM_ACT, 1, u_sub, integrator_repart, u_prop);
  MAT_SUM(CTRL_HOVER_WIND_NUM_ACT, 1, u, ueq, u_sub);



 
  if (u[0][0]>0){
    u_scale[0][0] = (sqrtf(MAX(u[0][0],0)/kf) / mot_max_speed)*MAX_PPRZ; 
  }
  else{
    u_scale[0][0] = 0; 
  }

  if (u[1][0]>0){
    u_scale[1][0] = (sqrtf(MAX(u[1][0],0)/kf) / mot_max_speed)*MAX_PPRZ; 
  }
  else{
    u_scale[1][0] = 0; 
  }
 
  u_scale[2][0] = (u[2][0]*6/M_PI)*MAX_PPRZ;
  u_scale[3][0] = (u[3][0]*6/M_PI)*MAX_PPRZ;

  #if DBG_CMD
    printf("u: %f %f %f %f  \n", u_scale[0][0], u_scale[1][0], u_scale[2][0], u_scale[3][0]);
  #endif


  actuators_pprz[0]=TRIM_PPRZ(u_scale[3][0]); //ELEVON_LEFT  -
  actuators_pprz[1]=TRIM_PPRZ(-u_scale[2][0]); // ELEVON_RIGHT  

  //actuators_pprz[0]=TRIM_PPRZ(0); //ELEVON_LEFT  
  //actuators_pprz[1]=TRIM_PPRZ(0); // ELEVON_RIGHT  

  actuators_pprz[2]=TRIM_UPPRZ(u_scale[0][0]); // RIGHT_MOTOR
  actuators_pprz[3]=TRIM_UPPRZ(u_scale[1][0]); // LEFT_MOTOR

  //actuators_pprz[2]=TRIM_UPPRZ(5000); // RIGHT_MOTOR
  //actuators_pprz[3]=TRIM_UPPRZ(5000); // LEFT_MOTOR
  
  
  #if DBG_CMD
    printf("actuators: %d %d %d %d\n", actuators_pprz[0], actuators_pprz[1], actuators_pprz[2], actuators_pprz[3]);
    float f[17] = {stateGetPositionNed_f()->x, stateGetPositionNed_f()->y, stateGetPositionNed_f()->z, stateGetSpeedNed_f()->x, stateGetSpeedNed_f()->y, stateGetSpeedNed_f()->z, quat_att_barth_frame.qi, quat_att_barth_frame.qx, quat_att_barth_frame.qy, quat_att_barth_frame.qz, stateGetBodyRates_f()->p, stateGetBodyRates_f()->q, stateGetBodyRates_f()->r, actuators_pprz[0], actuators_pprz[1], actuators_pprz[2], actuators_pprz[3]};
    DOWNLINK_SEND_PAYLOAD_FLOAT(DefaultChannel, DefaultDevice, 17, f);
  #endif

  if (in_flight) {
    stabilization_cmd[COMMAND_THRUST] = (actuators_pprz[2]+actuators_pprz[3]); // for in_flight detection
    //printf("in_flight\n");
  } else {
    stabilization_cmd[COMMAND_THRUST] = 1000;
    //printf("Not in_flight\n");
  };
  

  //log_hoverwind_periodic();
}

// Report function

void data_report()
{
  float msg[] = {
    nav.target.x,
    nav.target.y,
    nav.target.z,
    stateGetNedToBodyQuat_f()->qi, 
    stateGetNedToBodyQuat_f()->qx,
    stateGetNedToBodyQuat_f()->qy,
    stateGetNedToBodyQuat_f()->qz,
    quat_att_barth_frame.qi, 
    quat_att_barth_frame.qx,
    quat_att_barth_frame.qy,
    quat_att_barth_frame.qz    
  };
  DOWNLINK_SEND_PAYLOAD_FLOAT(DefaultChannel, DefaultDevice, 11, msg);
}


/*static inline void log_hoverwind_start(void) {
  // Check that log file has been created correctly
  if (pprzLogFile != -1) {
    //header
    sdLogWriteLog(pprzLogFile, "'time';'eps1';'eps2';'eps3';'eps7';'eps8';'qi';'qx';'qy';'qz';'u_scale1';'u_scale2';'u_scale3';'u_scale4'\n");
  }
}

static inline void log_hoverwind_periodic(void) {
  // Check that log file has been created correctly
  if (pprzLogFile != -1) {
    //Data
    sdLogWriteLog(pprzLogFile, "%.5f;", get_sys_time_float());
    sdLogWriteLog(pprzLogFile, "%.4f;%.4f;%.4f;%.4f;%.4f;", eps[0][0], eps[1][0], eps[2][0], eps[6][0], eps[7][0]);
    sdLogWriteLog(pprzLogFile, "%.4f;%.4f;%.4f;%.4f;", stateGetNedToBodyQuat_f()->qi, stateGetNedToBodyQuat_f()->qx, stateGetNedToBodyQuat_f()->qy,stateGetNedToBodyQuat_f()->qz);
    sdLogWriteLog(pprzLogFile, "%f;%f;%f;%f\n", u_scale[0][0], u_scale[1][0], u_scale[2][0], u_scale[3][0]);
  }
}*/
