/*
 * Copyright (C) 2022 Tomaso De Ponti <tomasodp@gmail.com>
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

/** @file "modules/ctrl/ctrl_windtunnel_2022.c"
 * @author Tomaso De Ponti <tomasodp@gmail.com>
 * Module to test actuator effectiveness of rotwingdrone
 */

#include "modules/ctrl/ctrl_windtunnel_2022.h"
#include "modules/system_identification/extended_eff_message.h"
#include "firmwares/rotorcraft/stabilization/stabilization_indi.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_rc_setpoint.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_quat_transformations.h"
#include "subsystems/actuators.h"
#include "modules/system_identification/sys_id_doublet.h"
#include "modules/system_identification/sys_id_chirp.h"
#include "modules/rot_wing_drone/wing_rotation_controller.h"
#include "mcu_periph/sys_time.h"

int16_t count = 0;
int16_t mot_status[3][4] = {{0,0,0,0},{6400,6400,6400,6400},{6400,6400,8533,6400}};
int16_t as_static[5] = {-9600,-4800,1920,4800,9600};
float dt_s = 1;//3;
float dt_m = 3;//6;
float dt_l = 5;//10;
float wing_sp[7] = {0,10,30,45,60,75,90};
static float t_excitation = 0;
static float t_as = 0;
static float t_mot_status = 0;
static float t_wing = 0;
bool done_wing = true;
bool done_mot_status = true;
bool done_as = true;
bool done_excitation = true;
int8_t i = 0;
int8_t j = 0;
int8_t m = 0;
int8_t k = 0;
int8_t n = 0;


bool windtunnel_control(void)
{
 static_test = true;
 if (i < 7){
   if(done_wing){
     t_wing = get_sys_time_float();
     wing_rotation.wing_angle_deg_sp = wing_sp[i];
     printf("Wing SP = %f \n",wing_rotation.wing_angle_deg_sp);
     done_wing = false;}
   else{
     if((get_sys_time_float() - t_wing) > dt_m){
     if(!mot_status_control()){
       done_wing = true;
       i += 1;}}} 
   return true;    
   }else{
     i = 0;
     wing_rotation.wing_angle_deg_sp = 0;
     return false;}     
   }

bool mot_status_control(void)
{
 static_test = true;
 if (j < 3){
   if(done_mot_status){
     t_mot_status = get_sys_time_float();
     for ( int8_t m = 0; m < 4; m++){
      actuators_pprz_static[m] = (int16_t) mot_status[j][m];
     }
     printf("Motor State = %i %i %i %i \n",actuators_pprz_static[0],actuators_pprz_static[1],actuators_pprz_static[2],actuators_pprz_static[3]);
     done_mot_status = false;}
   else{
     if((get_sys_time_float() - t_mot_status) > dt_l){
     if(!as_control()){
       done_mot_status = true;
       j += 1;}}} 
   return true;    
   }else{
     j = 0;
     for ( int8_t m = 0; m < 4; m++){
      actuators_pprz_static[m] = (int16_t) 0;
     }
     return false;}     
   }   

bool as_control(void)
{
 static_test = true;
 if (k < 4){
   if(done_as){
     printf("AS = %i \n",k);
     done_as = false;}
   else{
     if(!excitation_control()){
       done_as = true;
       k += 1;}} 
   return true;    
   }else{
     k = 0;
     return false;}     
   }

bool excitation_control(void)
{
 if (n < 5 ){
   if(done_excitation){     
     t_excitation = get_sys_time_float();
     actuators_pprz_static[k+4] = (int16_t) as_static[n];
     printf("Excitation = %i \n",as_static[n]);
     done_excitation = false;}
   else{
     if((get_sys_time_float() - t_excitation) > dt_m){
       done_excitation = true;
       n += 1;}}
   return true;
   }else{
     actuators_pprz_static[k+4] = (int16_t) 0;
     n = 0;
     return false;}
}
// void windtunnel_control(void)
// {
//  static_test = true;
//  for ( int8_t i = 0; i < 7; i++){
//    //for every wing set point
//    wing_rotation.wing_angle_deg_sp = wing_sp[i];
//    printf("Wing SP = %f \n",wing_rotation.wing_angle_deg_sp);
//    sleep(10);
//    for ( int8_t j = 0; j < 3; j++){
//     //for every motor status
//     for ( int8_t m = 0; m < 4; m++){
//       actuators_pprz_static[m] = (int16_t) mot_status[j][m];
//     }
//     printf("Motor State = %i %i %i %i \n",actuators_pprz_static[0],actuators_pprz_static[1],actuators_pprz_static[2],actuators_pprz_static[3]);
//     sleep(5);
//     for ( int8_t k = 0; k < 4; k++){
//       //for every aerodynamic surface
//       printf("AS = %i \n",k);
//       for ( int8_t n = 0; n < 5; n++){
//        // for every excitation setting
//        actuators_pprz_static[k+4] = (int16_t) as_static[n];
//        printf("Excitation = %i \n",as_static[n]);
//        sleep(5);
//        }
//     }
//    } 
//   }
// }

// void extended_eff_message_init(void)
// {

// }
