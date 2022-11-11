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
 * Module to charecterize drone in Windtunnel
 */

// INCLUDES #############################################################################################################################################################################
#include "modules/ctrl/ctrl_windtunnel_2022.h"
#include "firmwares/rotorcraft/stabilization/stabilization_indi.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_rc_setpoint.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_quat_transformations.h"
#include "modules/actuators/actuators.h"
#include "modules/system_identification/sys_id_doublet.h"
#include "modules/system_identification/sys_id_chirp.h"
#include "modules/rot_wing_drone/wing_rotation_controller.h"
#include "mcu_periph/sys_time.h"
#include "modules/wind_tunnel/wind_tunnel_rot_wing.h"
//#######################################################################################################################################################################################

// Defines of time variables ##########################################################################################################################################################
float dt_s = 1;// [s] Short test time interval. Used for procedures which do not require much time. (e.g. 1s)
float dt_m = 3;// [s] Medium test time interval. Used for procedures which involve transitional states (e.g. actuator test, 5s)
float dt_l = 3;// [s] Long test time interval. Used for procedure which involve the use of slow dynamics (e.g. rotation wing, 6s)
//#######################################################################################################################################################################################

// Defines of nested loop iteration limits ##############################################################################################################################################
#define imax 7 // Number of Wing set point 
#define jmax 3 // Number of Motor status (e.g. 5)
#define mmax 5 // Number of Motors used in the Motor status (e.g. 4) 
#define kmax 8 // Number of Aerodynamic Surfaces + Motors tested  (e.g. 4)
#define nmax 5 // Number of Excitation steps 
#define asel 3 // Number of selected actuators
//#######################################################################################################################################################################################

// Defines of variables to be used in manual slider test of actuators ###################################################################################################################
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
//#######################################################################################################################################################################################

// Defines of tested excitation signals #################################################################################################################################################
//int16_t mot_status[jmax][mmax] = {{0,0,0,0},{1000,1000,1000,1000},{2000,2000,2000,2000},{3000,3000,3000,3000}}; // [pprz] Motor status define [Status][Motor]
int16_t mot_status[jmax][mmax] = {{0,0,0,0,0},{3000,3000,3000,3000,0},{0,0,0,0,3000}}; // [pprz] Motor status define [Status][Motor]
int16_t as_static[nmax] = {-9600,-4800,1920,4800,9600}; // [pprz] Excitation signals Aerodynamic Surface 
int16_t mot_static[nmax] = {2000,4000,5000,6000,8000};  // [pprz] Excitation signals Motors
int16_t sync_cmd[6] = {800,0,1000,0,1200,0};            // [pprz] Excitation signals pusher motor during syncing procedure
int16_t push_cmd[2] = {2000,4000};
int16_t selected_act_idx[asel] = {3,9,10};              // Array of indeces of selected actuators
float wing_sp[imax] = {0,10,30,45,60,75,90};            // [deg] Tested skew angles 
float rotation_rate_sp[6] = {0.15,0.15,0.20,0.20,0.25,0.25};
float boa[6] =  {90,0,90,0,90,0};
float ramp_ratio = 1./3.;                               // [%] Percentage of test time dedicated to gradual sweep. FLOAT!
float max_rotation_rate=3.14/2.; // same                 // [rad/s] Maximum rotational velocity of wing
//#######################################################################################################################################################################################

// Initialisation of variables used in the nested functions

// FLOATS ------------------------------------------------------------------------------------------
static float t_excitation = 0;  // [s]Time of start of excitation of actuator
static float t_mot_status = 0;  // [s]
static float t_wing = 0;        // [s]
static float t_sync = 0;        // [s]
static float t_skew = 0;        // [s]     
float stopwatch = 0;            // [s] Elapsed time from start of excitation
float ratio_excitation = 1;     // [%] Percentage of completion of sweep for excitation of actuator

// BOOLs --------------------------------------------------------------------------------------------
bool done_wing = true;          // Skew test completed. Go to next skew.
bool done_mot_status = true;    // Mot Status test completed. Go to next mot status.
bool done_as = true;            // Actuator test completed. Go to next actuator.
bool done_excitation = true;    // Excitation completed. Go to next excitation signal.
bool test_active = false;       // Automatic test is active.
bool done_sync = true;          // Sync signal completed. Go to next sync signal.
bool shut_off = true;           // Shutoff of actuator completed. Go to next actuator. 
bool static_test = false;       // 
bool test_skew_active = false;  // Automatic skew test is active. [experimental skew test]
bool done_skew = true;          // Done skew. Go to next skew [experimetnal skew test]
bool single_act = true;         // Turn on to select only certain actuators
bool verbose_test_ID = false;   // When true, record test ID
// INTEGERs ------------------------------------------------------------------------------------------
int8_t i = 0;                                     // Wing set point counter
int8_t j = 0;                                     // Motor status counter
int8_t m = 0;                                     // Motor counter
int8_t k = 0;                                     // Actuator counter
int8_t k_conv = 20;                               // Actuator counter converted. 20 is the index of the wing.
int8_t n = 0;                                     // Excitation signal counter
int32_t tp = 0;                                   // Test point counter
int8_t p = 0;                                     // Test number counter (Equal to number of windspeeds)
int8_t w = 0;                                     // Sync command counter
int8_t o = 0;                                     // Counter rot test
int8_t p2 = 0;                                    // Test number counter for the skew moment test
int16_t cmd_0 = 0;                                // [pprz] Initial CMD of actuator
int16_t cmd_target = 0;                           // [pprz] Excitation cmd target
int16_t cmd_0_mot_status[mmax] = {0,0,0,0};       // [pprz] Initial CMD of mot status
// STRINGs --------------------------------------------------------------------------------------------
char test_id[5] = "AE1";
char point_id[20] = "Blank";
//#######################################################################################################################################################################################

// Graphical representation of nested functions #########################################################################################################################################

//  -- windtunnel_control:            
//    -- sync_procedure:    done? no (stay, next signal) | yes (Proceed) <== 
//         |                                                                |
//         v                                                                |
//    -- wing_skew_control: done? no (stay, next skew angle) | yes (Test Finished) <=======
//                                 |                                                       |
//                                 v                                                       |
//                             -- mot_status_control: done? no (stay, next mot status) | yes (next skew angle) <=====
//                                                           |                                                       |
//                                                           v                                                       |
//                                                       -- as_control: done? no (stay, next actuator) | yes (next mot status) <===
//                                                                             |                                                   |
//                                                                             v                                                   |
//                                                                         --excitation_control: done? no (stay, next signal) | yes (next actuator)

//#######################################################################################################################################################################################

// Function that pubishes selected slider cmd at highest freq possible#########################################################################################################################
void event_manual_test(void){
  if (manual_test) {
        actuators_wt[0]  = (int16_t) mot0_static;
        actuators_wt[1]  = (int16_t) mot1_static;
        actuators_wt[2]  = (int16_t) mot2_static;
        actuators_wt[3]  = (int16_t) mot3_static;
        actuators_wt[7]  = (int16_t) ailL_static;
        actuators_wt[8]  = (int16_t) ailR_static;
        actuators_wt[9]  = (int16_t) ele_static;
        actuators_wt[10] = (int16_t) rud_static;
        actuators_wt[4]  = (int16_t) push_static;}}
// ####################################################################################################################################################################################

// Skew test to understand reactionary moment generated by rotation of the wing #######################################################################################################
bool skew_moment(void){
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
          return false;}}}
//#######################################################################################################################################################################################

// Function to start a windtunnel test ##################################################################################################################################################
bool windtunnel_control(void){
 static_test = true;                            
 test_active = true;
 motors_on_wt = true;                           // Arm motors
  if (w < 6){                                   // if synching not done re-enter function
   sync_procedure();
   return true;    
   }else{                                       // else enter the skewing function
     if(wing_skew_control()){return true;}      // if not done skewing stay in the loop
     else{                                      
        //w = 0;
        test_active = false;                    // if done skewing terminate the test
        motors_on_wt = false;
        p += 1;
        return false;}}}
//#######################################################################################################################################################################################

// Function to perform a synching sequence for windtunnel data ##########################################################################################################################
void sync_procedure(void){
  motors_on_wt = true;                                // Arm motors
  if(done_sync){                                      // If synching sequence still has to start
      t_sync = get_sys_time_float();                  // Record current time of initialization
      actuators_wt[4]= (int16_t) sync_cmd[w];         // Send a cmd to the pusher motor
      printf("Sync CMD = %i \n",actuators_wt[4]);
      done_sync = false;}                             // Register that the signal has been set
    else{
        if((get_sys_time_float() - t_sync) > dt_s){   // If the signal has been maintained long enough
        done_sync = true;                             // terminate synch step and go to next signal
        w += 1;}}}
//#######################################################################################################################################################################################

// Function to command different skew settings ##########################################################################################################################################
bool wing_skew_control(void){
 if (i < imax){                                                   // If not all skew have been explored stay in the function
   if(done_wing){                                                 // If current skew test is done go to next skew setting
     t_wing = get_sys_time_float();                               // Record current time of initialization
     wing_rotation.wing_angle_deg_sp = wing_sp[i];                // Command the next skew set point
     printf("Wing SP = %f \n",wing_rotation.wing_angle_deg_sp);
     done_wing = false;}                                          // Register that the signal has been set 
   else{
     if((get_sys_time_float() - t_wing) > dt_l){                  // If enough time has passed for the wing to rotate
     if(!mot_status_control()){                                   // If all motor status have been explored
       done_wing = true;                                          // go to next skew setting
       i += 1;}}} 
   return true;    
   }else{                                                         // If all skews explored go back to QUAD and terminate test
     i = 0;
     wing_rotation.wing_angle_deg_sp = 0;
     return false;}}
//#######################################################################################################################################################################################

// Function to command different motor status (thrust commands) #########################################################################################################################
bool mot_status_control(void){
 if (j < jmax){
   if(done_mot_status){
     t_mot_status = get_sys_time_float();
     done_mot_status = false;
     for ( int8_t m = 0; m < mmax; m++){cmd_0_mot_status[m] = (int16_t) actuators_wt[m];}}
   else{
    stopwatch = get_sys_time_float() - t_mot_status;       // Register elapsed time
    //  printf("Motor State = %i %i %i %i \n",actuators_wt[0],actuators_wt[1],actuators_wt[2],actuators_wt[3]);
    //  printf("Test Point = %i \n",tp);
     if(stopwatch > dt_m){
     verbose_test_ID = false;
      if(!as_control()){
        done_mot_status = true;
        j += 1;}}
     else{
      ratio_excitation = stopwatch / dt_m / ramp_ratio;    // Calclate the percentage of completion of the initial linear sweep
      Bound(ratio_excitation, 0, 1);
      if (ratio_excitation==1){
        verbose_test_ID=true;
        tp += 1;}
      for ( int8_t m = 0; m < mmax; m++){actuators_wt[m] = (int16_t)cmd_0_mot_status[m] + ( mot_status[j][m] - cmd_0_mot_status[m]) * ratio_excitation;}}} 
   return true;    
   }else{
    if (shut_off){                                                             // If shutoff not yet initialized, initialize it
      t_mot_status = get_sys_time_float();
      shut_off = false;
      for ( int8_t m = 0; m < mmax; m++){cmd_0_mot_status[m] = (int16_t) actuators_wt[m];}
      return true;
     }else {
      stopwatch = get_sys_time_float() - t_mot_status;
      ratio_excitation = stopwatch / dt_m / ramp_ratio;
      Bound(ratio_excitation, 0, 1);
      for ( int8_t m = 0; m < mmax; m++){actuators_wt[m] = (int16_t) cmd_0_mot_status[m] + (0- cmd_0_mot_status[m]) * ratio_excitation;}
      if (ratio_excitation == 1) {                                              // If shutoff completed move to next actuator
        shut_off = true;
        j = 0;
        return false;}
      else {return true;}}
     return false;}}   
//#######################################################################################################################################################################################

// Function to iterate inbetween the different actuators ################################################################################################################################
bool as_control(void){
 if (k < kmax){
   if(k<4){k_conv = k+7;}                    // If aerodynamic surfaces index is 7-10 
   else{k_conv = k-4;}                       // If motor index is 0-3
   //if (k < 2 && j>2){k += 1;} // From 3rd mot status only activate the tail surfaces (k>2)
   if (single_act && !act_selected(k_conv, selected_act_idx)){k += 1;}
   else{
      if(done_as){
        printf("Actuator = %i \n",k_conv);
        done_as = false;}
      else{
        if(!excitation_control()){
          done_as = true;
          k += 1;}}} 
   return true;    
   }else{
     k = 0;
     k_conv = 20;                           // Reset to WING idx
     return false;}}
//#######################################################################################################################################################################################

// Function to command different excitation signals #####################################################################################################################################
bool excitation_control(void){
 if (n < nmax ){                                            // If not all excitation signal explored stay in the function
   if(done_excitation){                                     // If excitation done go to next signal
     t_excitation = get_sys_time_float();                   // Register current initialization time
     cmd_0 = actuators_wt[k_conv];                          // Register initial value
     printf("Excitation = %i \n",as_static[n]);
     done_excitation = false;
     stopwatch = 0;
     tp += 1;
     if(k<4){cmd_target = as_static[n];}                  // Command the aerodynamic surfaces
     else{cmd_target = mot_static[n];}}                    // Command the motors}
   else{
     stopwatch = get_sys_time_float() - t_excitation;       // Register elapsed time
     if(stopwatch > dt_m){                                  // If Elapsed time bigger than test time, terminate the excitation test
       done_excitation = true;
       verbose_test_ID = false;    
       n += 1;}
     else{                                                  // Else calculate the cmd to the actuator
       ratio_excitation = stopwatch / dt_m / ramp_ratio;    // Calclate the percentage of completion of the initial linear sweep
       Bound(ratio_excitation, 0, 1);
       if (ratio_excitation==1){verbose_test_ID=true;}
       actuators_wt[k_conv] = (int16_t) cmd_0 + (cmd_target - cmd_0) * ratio_excitation;}} 
   return true;
   }else{                                                                       // If all excitation explored, initiate a shutoff sequence of the actuator
     if (shut_off){                                                             // If shutoff not yet initialized, initialize it
      t_excitation = get_sys_time_float();
      shut_off = false;
      cmd_0 = actuators_wt[k_conv];
      return true;
     }else {
      stopwatch = get_sys_time_float() - t_excitation;
      ratio_excitation = stopwatch / dt_m / ramp_ratio;
      Bound(ratio_excitation, 0, 1);
      if(k<4){cmd_target = 0;}                             // Command the aerodynamic surfaces
      else{cmd_target = mot_status[j][k_conv];}            // CMD target might not be zero if we are testing a mot status. Logic might not work if m is different from k_conv
      actuators_wt[k_conv] = (int16_t) cmd_0 + (cmd_target- cmd_0) * ratio_excitation;
      if (ratio_excitation == 1) {                                              // If shutoff completed move to next actuator
        shut_off = true;
        n = 0;
        cmd_0 = 0;
        return false;}
      else {return true;}}}}
//#######################################################################################################################################################################################

// Message to register the wanted telemetry #############################################################################################################################################
#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"
static void send_windtunnel_static(struct transport_tx *trans, struct link_device *dev)
{
  float airspeed = stateGetAirspeed_f();
  uint8_t test_active_conv = (uint8_t) test_active;
  if (!verbose_test_ID){
       point_id[0] = '\0';
       strcat(point_id, "Blank");}
  else{ID_gen();}
  pprz_msg_send_WINDTUNNEL_STATIC(trans, dev, AC_ID,
                                        strlen(point_id), point_id,
                                        &test_active_conv,
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
                                        11, actuators_wt,
                                        11, actuators
                                        );
}
#endif
//ACTUATORS_NB
void windtunnel_message_init(void){
#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_WINDTUNNEL_STATIC, send_windtunnel_static);
#endif 
}
//#######################################################################################################################################################################################

// Function to check if index is in the selected array #######################################################################################################
bool act_selected(int8_t val, int16_t sel_val[]){
  int8_t i_sel;
  for(i_sel = 0; i_sel < asel; i_sel++){
    if(sel_val[i_sel]==val){return true;}}
  return false;}
//#######################################################################################################################################################################################

// Function to generate ID of data point #######################################################################################################
void ID_gen(void){
  char k_ch[3];
  char i_ch[3];
  char u_ch[3];
  char cmd_ch[7];
  char j_ch[3];
  int8_t airspeed_i = (int8_t) stateGetAirspeed_f();
  sprintf(k_ch,"%i",k_conv);
  sprintf(i_ch,"%i",(int8_t) wing_sp[i]);
  sprintf(u_ch,"%i",airspeed_i);
  sprintf(cmd_ch,"%i",cmd_target);
  sprintf(j_ch,"%i",j);
  point_id[0] = '\0';
  strcat(point_id, test_id);
  strcat(point_id,"_");
  strcat(point_id, k_ch);
  strcat(point_id,"_");
  strcat(point_id, cmd_ch);
  strcat(point_id,"_");
  strcat(point_id, i_ch);
  strcat(point_id,"_");
  strcat(point_id, j_ch);
  strcat(point_id,"_");
  strcat(point_id, u_ch);
  //printf("String ID = %s \n", point_id);
  //printf("test test \n");
}
//#######################################################################################################################################################################################
