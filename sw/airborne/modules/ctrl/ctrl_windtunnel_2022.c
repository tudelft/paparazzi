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
#include "firmwares/rotorcraft/stabilization/stabilization_indi.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_rc_setpoint.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_quat_transformations.h"
//#include "subsystems/actuators.h"
#include "modules/actuators/actuators.h"
#include "modules/system_identification/sys_id_doublet.h"
#include "modules/system_identification/sys_id_chirp.h"
#include "modules/rot_wing_drone/wing_rotation_controller.h"
#include "mcu_periph/sys_time.h"
#include "modules/wind_tunnel/wind_tunnel_rot_wing.h"

float dt_s = 1;//1-3;
float dt_m = 5;//4 3-6;
float dt_l = 6;//7 5-10;
#define imax 7 // Wing set point counter
#define jmax 1 // 5 Motor status counter
#define mmax 4 // Motor counter
#define kmax 8 // 4 Aerodynamic Surface + Motor counter
#define nmax 5 // Excitation signal counter

bool manual_test;
int16_t mot0_static = 0;
int16_t mot1_static = 0;
int16_t mot2_static = 0;
int16_t mot3_static = 0;
int16_t ailL_static = 0;
int16_t ailR_static = 0;
int16_t ele_static  = 0;
int16_t rud_static  = 0;
int16_t push_static = 0;

//int16_t mot_status[jmax][mmax] = {{0,0,0,0},{6400,6400,6400,6400},{6400,6400,8533,6400},{8000,8000,8000,8000},{0,0,0,0}};
int16_t mot_status[jmax][mmax] = {{0,0,0,0},{1000,1000,1000,1000},{2000,2000,2000,2000},{3000,3000,3000,3000}};
int16_t as_static[nmax] = {-9600,-4800,1920,4800,9600};
int16_t mot_static[nmax] = {2000,4000,5000,6000,8000};
int16_t sync_cmd[6] = {800,0,1000,0,1200,0};
int16_t push_cmd[2] = {2000,4000};
int16_t cmd_0 = 0;
float wing_sp[imax] = {0,10,30,45,60,75,90};
float rotation_rate_sp[6] = {0.15,0.15,0.20,0.20,0.25,0.25};
float boa[6] =  {90,0,90,0,90,0};
static float t_excitation = 0;
static float t_as = 0;
static float t_mot_status = 0;
static float t_wing = 0;
static float t_sync = 0;
static float t_skew = 0;
bool done_wing = true;
bool done_mot_status = true;
bool done_as = true;
bool done_excitation = true;
bool test_active = false;
bool test_skew_active = false;
bool done_sync = true;
bool done_skew = true;
bool shut_off = true;
int8_t i = 0;   // Wing set point counter
int8_t j = 0;   // Motor status counter
int8_t m = 0;   // Motor counter
int8_t k = 0;   // Aerodynamic Surface counter
int8_t n = 0;   // Excitation signal counter
int32_t tp = 0; // Test point counter
int8_t p = 0;   // Test number counter (Equal to number of windspeeds)
int8_t w = 0;   //Sync command counter
int8_t o = 0;   //Counter rot test
int8_t p2 = 0;  // Test number counter for the skew moment test
bool static_test = false; // defining now because do not rember where it has to be defined
float max_rotation_rate=3.14/2; // same
float stopwatch = 0;
float ratio_excitation = 1;
float ramp_ratio = 1./3.;
// Function that pubishes selected cmd at highest freq possible
void event_manual_test(void)
{
  if (manual_test) {
        actuators_wt[0]  = (int16_t) mot0_static;
        actuators_wt[1]  = (int16_t) mot1_static;
        actuators_wt[2]  = (int16_t) mot2_static;
        actuators_wt[3]  = (int16_t) mot3_static;
        actuators_wt[7]  = (int16_t) ailL_static;
        actuators_wt[8]  = (int16_t) ailR_static;
        actuators_wt[9]  = (int16_t) ele_static;
        actuators_wt[10] = (int16_t) rud_static;
        actuators_wt[4]  = (int16_t) push_static; 
  }
}


bool skew_moment(void)
{
 static_test = true;
 test_skew_active = true;
   if (w < 6){
   sync_procedure();
   return true;    
   }else{
     if (o < 6){
        if(done_skew){
          t_skew = get_sys_time_float();
          max_rotation_rate = rotation_rate_sp[o];
          wing_rotation.wing_angle_deg_sp = boa[o];
          printf("Wing SP = %f \n",wing_rotation.wing_angle_deg_sp);
          done_skew = false;}
        else{
          if((get_sys_time_float() - t_skew) > (2.0*3.14*0.5/max_rotation_rate)){
            done_skew = true;
            o += 1;}}
        return true;    
        }else{
          o = 0;
          wing_rotation.wing_angle_deg_sp = 0;
          test_skew_active = false;
          p2 += 1;
          return false;}  
    }
}
bool windtunnel_control(void)
{
 static_test = true;
 test_active = true;
 motors_on_wt = true;
  if (w < 6){
   sync_procedure();
   return true;    
   }else{
     if(wing_skew_control()){return true;}
     else{
        //w = 0;
        test_active = false;
        motors_on_wt = false;
        p += 1;
        return false;
     }
    }
}
void sync_procedure(void)
{
  if(done_sync){
      t_sync = get_sys_time_float();
      actuators_wt[4]= (int16_t) sync_cmd[w];
      printf("Sync CMD = %i \n",actuators_wt[4]);
      done_sync = false;}
    else{
        if((get_sys_time_float() - t_sync) > dt_s){
        done_sync = true;
        w += 1;}}
}
bool wing_skew_control(void)
{
 //static_test = true;
 //test_active = true;
 if (i < imax){
   if(done_wing){
     t_wing = get_sys_time_float();
     wing_rotation.wing_angle_deg_sp = wing_sp[i];
     printf("Wing SP = %f \n",wing_rotation.wing_angle_deg_sp);
     done_wing = false;}
   else{
     if((get_sys_time_float() - t_wing) > dt_l){
     if(!mot_status_control()){
       done_wing = true;
       i += 1;}}} 
   return true;    
   }else{
     i = 0;
     wing_rotation.wing_angle_deg_sp = 0;
     //test_active = false;
     //p += 1;
     return false;}     
   }

bool mot_status_control(void)
{
 if (j < jmax){
   if(done_mot_status){
     t_mot_status = get_sys_time_float();
     for ( int8_t m = 0; m < mmax; m++){
      actuators_wt[m] = (int16_t) mot_status[j][m];
     }
     printf("Motor State = %i %i %i %i \n",actuators_wt[0],actuators_wt[1],actuators_wt[2],actuators_wt[3]);
     printf("Test Point = %i \n",tp);
     //if (j>2){actuators_wt[8]= (int16_t) push_cmd[j-3];}
     done_mot_status = false;
     tp += 1;}
   else{
     if((get_sys_time_float() - t_mot_status) > dt_m){
     if(!as_control()){
       done_mot_status = true;
       j += 1;}}} 
   return true;    
   }else{
     j = 0;
     for ( int8_t m = 0; m < 4; m++){
      actuators_wt[m] = (int16_t) 0;
     }
     actuators_wt[4]= (int16_t) 0;
     return false;}     
   }   

bool as_control(void)
{
 if (k < kmax){
   if (k < 2 && j>2){k += 1;}
   else{
   if(done_as){
     printf("AS = %i \n",k);
     done_as = false;}
   else{
     if(!excitation_control()){
       done_as = true;
       k += 1;}}} 
   return true;    
   }else{
     k = 0;
     return false;}     
   }

bool excitation_control(void)
{
 if (n < nmax ){
   if(done_excitation){     
     t_excitation = get_sys_time_float();
     if(k<4){cmd_0 = actuators_wt[k+7];}
     else{cmd_0 = actuators_wt[k-4];}
     printf("Excitation = %i \n",as_static[n]);
     done_excitation = false;
     stopwatch = 0;
     tp += 1;}
   else{
     stopwatch = get_sys_time_float() - t_excitation;
     if(stopwatch > dt_m){
       done_excitation = true;
       n += 1;}
     else{
       ratio_excitation = stopwatch / dt_m / ramp_ratio;
       Bound(ratio_excitation, 0, 1);
       if(k<4){actuators_wt[k+7] = (int16_t) cmd_0 + (as_static[n] - cmd_0) * ratio_excitation;}
       else{actuators_wt[k-4] = (int16_t) cmd_0 + (mot_static[n] - cmd_0) * ratio_excitation;}
         }
       }
   return true;
   }else{
     if (shut_off){
      t_excitation = get_sys_time_float();
      shut_off = false;
      if(k<4){cmd_0 = actuators_wt[k+7];}
      else{cmd_0 = actuators_wt[k-4];}
      return true;
     }
     else {
      stopwatch = get_sys_time_float() - t_excitation;
      ratio_excitation = stopwatch / dt_m / ramp_ratio;
      Bound(ratio_excitation, 0, 1);
      if(k<4){actuators_wt[k+7] = (int16_t) cmd_0 - cmd_0 * ratio_excitation;}
      else{actuators_wt[k-4] = (int16_t) cmd_0 - cmd_0 * ratio_excitation;}
      if (ratio_excitation == 1) {
        shut_off = true;
        n = 0;
        cmd_0 = 0;
        return false;}
      else {return true;}
     }
    }
}

#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"
// #include "subsystems/datalink/telemetry.h"
static void send_windtunnel_static(struct transport_tx *trans, struct link_device *dev)
{
float airspeed = stateGetAirspeed_f();

pprz_msg_send_WINDTUNNEL_STATIC(trans, dev, AC_ID,
                                        &test_active,
                                        &airspeed,
                                        &wing_rotation.wing_angle_deg,
                                        &wing_rotation.wing_angle_deg_sp,
                                        &(stateGetNedToBodyEulers_i()->theta),
                                        &p,
                                        &i,
                                        &j,
                                        &k,
                                        &n,
                                        &tp,
                                        &o,
                                        &p2,
                                        &max_rotation_rate,
                                        11, actuators_wt
                                        );
}
#endif
//ACTUATORS_NB, actuators_wt
void windtunnel_message_init(void)
{
  #if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_WINDTUNNEL_STATIC, send_windtunnel_static);
  #endif 
}
