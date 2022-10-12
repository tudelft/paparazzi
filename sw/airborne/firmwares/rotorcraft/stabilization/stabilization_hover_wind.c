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


float K[CTRL_HOVER_WIND_NUM_ACT][CTRL_HOVER_WIND_INPUT] = {{1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}, 
                                                           {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
                                                           {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
                                                           {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}
                                                          };

float H[CTRL_HOVER_WIND_NUM_INTEGRATOR_STATE][CTRL_HOVER_WIND_INPUT] = {{1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}, 
                                                                        {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}
                                                                       };                                                          

float target[CTRL_HOVER_WIND_INPUT][1] = {{1}, {0}, {0}, {0}, {0}, {0}, {0}, {0}, {0}, {0}, {0}, {0}};  

float ueq[CTRL_HOVER_WIND_NUM_ACT][1] = {{1}, {1}, {0}, {0}};


float x_e[CTRL_HOVER_WIND_INPUT][1];
float state_vector_redu[CTRL_HOVER_WIND_INPUT][1];

static void get_state_vector(void);

void stabilization_hover_wind_init(void){
  //get_state_vector();
  x_e[0][0] = 0;
  x_e[1][0] = 0;
}

void stabilization_hover_wind_run(bool in_flight){
  if (in_flight) {
    get_state_vector();
    float eps[CTRL_HOVER_WIND_INPUT][1];
    MAT_SUB(CTRL_HOVER_WIND_INPUT,1,eps,state_vector_redu,target);
    float dot_x_e_dt[CTRL_HOVER_WIND_NUM_INTEGRATOR_STATE][1]; 
    MAT_MUL_c(CTRL_HOVER_WIND_NUM_INTEGRATOR_STATE, CTRL_HOVER_WIND_INPUT, 1, dot_x_e_dt, H, eps, 1./PERIODIC_FREQUENCY);
    x_e[0][0] +=  dot_x_e_dt[0][0];
    x_e[1][0] +=  dot_x_e_dt[1][0];
    float u_integrator[CTRL_HOVER_WIND_NUM_ACT][1] = {{x_e[0][0]}, {x_e[0][0]}, {x_e[1][0]}, {x_e[1][0]}};
    float u_prop[CTRL_HOVER_WIND_NUM_ACT][1];
    MAT_MUL(CTRL_HOVER_WIND_NUM_ACT, CTRL_HOVER_WIND_INPUT, 1, u_prop, K, eps);
    float u_sub[CTRL_HOVER_WIND_NUM_ACT][1];
    MAT_SUB(CTRL_HOVER_WIND_NUM_ACT, 1, u_sub, u_integrator, u_prop);
    float u[CTRL_HOVER_WIND_NUM_ACT][1];
    MAT_SUM(CTRL_HOVER_WIND_NUM_ACT, 1, u, ueq, u_sub);


    actuators_pprz[0]=1;
    actuators_pprz[1]=1;
    actuators_pprz[2]=0;
    actuators_pprz[3]=0;

    /*Commit the actuator command
    for (i = 0; i < CTRL_HOVER_WIND_NUM_ACT; i++) {
      actuators_pprz[i] = (int16_t) indi_u[i];
    }*/

    // Set the stab_cmd to 42 to indicate that it is not used
    /*stabilization_cmd[COMMAND_ROLL] = 42;
    stabilization_cmd[COMMAND_PITCH] = 42;
    stabilization_cmd[COMMAND_YAW] = 42;*/
  } else {
    
  }
}


static void get_state_vector(void){
  state_vector_redu[0][0] = stateGetPositionEnu_f()->x;
  state_vector_redu[1][0] = stateGetPositionEnu_f()->y ;
  state_vector_redu[2][0] =stateGetPositionEnu_f()->z;
  state_vector_redu[3][0] =stateGetSpeedEnu_f()->x;
  state_vector_redu[4][0] =stateGetSpeedEnu_f()->y;
  state_vector_redu[5][0] =stateGetSpeedEnu_f()->z;
  state_vector_redu[6][0] =stateGetNedToBodyQuat_f()->qx;
  state_vector_redu[7][0] =stateGetNedToBodyQuat_f()->qy;
  state_vector_redu[8][0] =stateGetNedToBodyQuat_f()->qz;
  state_vector_redu[9][0] =stateGetBodyRates_f()->p;
  state_vector_redu[10][0] =stateGetBodyRates_f()->q;
  state_vector_redu[11][0] =stateGetBodyRates_f()->p;
}