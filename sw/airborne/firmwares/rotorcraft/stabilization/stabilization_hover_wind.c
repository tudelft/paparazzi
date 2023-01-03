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
#include "generated/flight_plan.h"
#include <stdio.h>

/*#include "modules/loggers/sdlog_chibios.h"
#include "mcu_periph/sys_time.h"
*/

float x_e[CTRL_HOVER_WIND_INPUT][1];
float state_vector_redu[CTRL_HOVER_WIND_INPUT][1];

float eps[CTRL_HOVER_WIND_INPUT][1];
float dot_x_e_dt[CTRL_HOVER_WIND_NUM_INTEGRATOR_STATE][1]; 
float u_integrator[CTRL_HOVER_WIND_NUM_ACT][1];
float u_prop[CTRL_HOVER_WIND_NUM_ACT][1];
float u_filter[CTRL_HOVER_WIND_NUM_ACT][1];
float u_sub[CTRL_HOVER_WIND_NUM_ACT][1];
float u[CTRL_HOVER_WIND_NUM_ACT][1];
float u_scale[CTRL_HOVER_WIND_NUM_ACT][1];

struct FloatQuat quat_roty_90 = {0.707106781186548,   0,    0.707106781186548,    0};
struct FloatQuat quat_att_barth_frame;

float tf_state1[2];
float tf_state2[2];


// static inline void log_hoverwind_periodic(void);
// static inline void log_hoverwind_start(void);


void stabilization_hover_wind_init(void){

  //log_hoverwind_start();

  x_e[0][0] = 0;
  x_e[1][0] = 0;

}

void stabilization_hover_wind_run(bool in_flight){
    
  float_quat_comp(&quat_att_barth_frame, &quat_roty_90, stateGetNedToBodyQuat_f());
  
  eps[0][0] = stateGetPositionEnu_f()->x - POS_FLOAT_OF_BFP(navigation_target.x); //navigation_target ENU
  eps[1][0] = stateGetPositionEnu_f()->y - POS_FLOAT_OF_BFP(navigation_target.y);
  eps[2][0] = stateGetPositionEnu_f()->z - POS_FLOAT_OF_BFP(navigation_target.z);
  eps[3][0] = stateGetSpeedEnu_f()->x;
  eps[4][0] = stateGetSpeedEnu_f()->y;
  eps[5][0] = stateGetSpeedEnu_f()->z;
  eps[6][0] = quat_att_barth_frame.qx;
  eps[7][0] = quat_att_barth_frame.qz;
  eps[8][0] = stateGetBodyRates_f()->p;
  eps[9][0] = stateGetBodyRates_f()->q;
  eps[10][0] = stateGetBodyRates_f()->p;

  
  MAT_MUL_c(CTRL_HOVER_WIND_NUM_INTEGRATOR_STATE, CTRL_HOVER_WIND_INPUT, 1, dot_x_e_dt, H, eps, 1./PERIODIC_FREQUENCY);
  x_e[0][0] +=  dot_x_e_dt[0][0];
  x_e[1][0] +=  dot_x_e_dt[1][0];
  u_integrator[0][0] = x_e[0][0];
  u_integrator[1][0] = x_e[1][0];
  MAT_MUL(CTRL_HOVER_WIND_NUM_ACT, CTRL_HOVER_WIND_INPUT, 1, u_prop, K, eps);

  float tf1_tmp;
  float tf2_tmp;

  tf1_tmp = (u_prop[0][0] - den[1]*tf_state1[0] - den[2]* tf_state1[1])/ den[0];
  u_filter[0][0] = num[0]*tf1_tmp + num[1]*tf_state1[0] + num[2]*tf_state1[1];

  tf2_tmp = (u_prop[1][0] - den[1]*tf_state2[0] - den[2]* tf_state2[1])/ den[0];
  u_filter[1][0] = num[0]*tf2_tmp + num[1]*tf_state2[0] + num[2]*tf_state2[1];


  MAT_SUB(CTRL_HOVER_WIND_NUM_ACT, 1, u_sub, u_integrator, u_filter);
  MAT_SUM(CTRL_HOVER_WIND_NUM_ACT, 1, u, ueq, u_sub);

  
  u_scale[0][0] = (sqrtf(u[0][0]/kf) / mot_max_speed)*MAX_PPRZ; 
  u_scale[1][0] = (u[1][0]*6/M_PI)*MAX_PPRZ;


  actuators_pprz[0]=TRIM_UPPRZ(u_scale[0][0]);
  actuators_pprz[1]=TRIM_UPPRZ(u_scale[0][0]);
  actuators_pprz[2]=TRIM_PPRZ(u_scale[1][0]);
  actuators_pprz[3]=TRIM_PPRZ(u_scale[1][0]);

  //log_hoverwind_periodic();
}

// Report function
void data_report()
{
  float msg[] = {
    POS_FLOAT_OF_BFP(navigation_target.x),
    POS_FLOAT_OF_BFP(navigation_target.y),
    POS_FLOAT_OF_BFP(navigation_target.z),
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