/*
 * Copyright (C) 2021 A. Mancinelli
 *
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
 * @file "modules/overactuated_vehicle/overactuated_vehicle.c"
 * @author Alessandro Mancinelli (a.mancinelli@tudelft.nl)
 * Control laws for Overactuated Vehicle
 */
#include "generated/airframe.h"
#include "state.h"
#include "paparazzi.h"
#include "overactuated_mixing.h"
#include <math.h>
#include "modules/radio_control/radio_control.h"
#include "modules/datalink/telemetry.h"
#include "modules/nav/waypoints.h"
#include "generated/flight_plan.h"
#include "math/pprz_algebra_float.h"
#include "math/pprz_matrix_decomp_float.c"
#include "modules/sensors/ca_am7.h"
#include "modules/sensors/serial_act_t4.h"
#include "modules/core/abi.h"
#include "mcu_periph/sys_time.h"
#include "modules/adcs/adc_generic.h"
#include "modules/energy/electrical.h"
#include "modules/core/sys_mon_rtos.h"
#include "firmwares/rotorcraft/navigation.h"
#include "modules/nav/nav_rotorcraft_hybrid.h"
#include "firmwares/rotorcraft/navigation.h"
#include "modules/ground_detect/ground_detect_am7.h"
#include "modules/ahrs/ahrs_float_cmpl.h"

/**
 * Variables declaration
 */
#define USE_NAV_HYBRID_MODULE
#define RECTIFY_LAT_AND_FWD_SPEED

float fpa_off_deg = 0.0; 

struct ship_info_msg ship_info_receive;

//Filter of lateral acceleration for turn correction
Butterworth2LowPass accel_body_y_filter; Butterworth2LowPass accel_control_rf_filters[3]; 
Butterworth2LowPass body_rates_dot_filters[3];

//General state variables:
float euler_vect[3], rate_vect[3], rate_vect_filt[3], rate_vect_dot[3], rate_vect_dot_filt[3];
float speed_vect[3], speed_vect_control_rf[3], acc_vect[3], accel_vect_filt_control_rf[3];
float accel_vect_control_rf[3], speed_vect_control_rf[3];
float speed_vect[3], pos_vect[3], airspeed = 0, beta_deg = 0, flight_path_angle = 0, total_V = 0;

float euler_error[3], euler_error_integrated[3];
float pos_error[3];

float euler_cmd_PID[3];

//Sideslip gains
float K_beta = AM7_SETTINGS_K_BETA;
float extra_lat_gain = 0.15; 
float overestimation_coeff; 

//Variables for the NONLINEAR_CA_DEBUG message: 
float feed_fwd_term_yaw, feed_back_term_yaw;

// PID and general settings from slider
int deadband_stick_yaw = 300, deadband_stick_throttle = 300;
float stick_gain_yaw = 0.05; //Stick to yaw and throttle gain (for the integral part)

//Setpoints and pseudocontrol
float pos_setpoint[3], speed_setpoint_control_rf[3];
float speed_error_vect[3], speed_error_vect_control_rf[3];
float euler_setpoint[3], rate_setpoint[3], acc_setpoint[6];

// serial_act_t4 variables:
struct ActCmd_t act_cmd_to_t4;

//Struct to am7: 
struct am7_data_t data_to_am7_module;

//Variable to keep track of the control mode used (1 is failsafe PID, 2 is Nonlinear controller): 
uint8_t control_mode_ovc_vehicle = 0; 

//Variables for the sysmon file write: 
// #define PRINT_CPU_LOAD_ON_SD
#ifdef PRINT_CPU_LOAD_ON_SD
    float time_old_sys_mon = 0;
#endif

//Variables for the ewoud approach module: 
static abi_event vel_sp_ev;
float des_speed_approach_control_rf[3];
float time_of_speed_setpoint_approach = 0; 

//Gains and limits:
int approach_state = 1; 
struct PID_over pid_gains_over = {
    .p = { OVERACTUATED_GAINS_PID_P_GAIN_PHI,
        OVERACTUATED_GAINS_PID_P_GAIN_THETA,
        OVERACTUATED_GAINS_PID_P_GAIN_PSI_AZ,
    },
    .i = { OVERACTUATED_GAINS_PID_I_GAIN_PHI,
        OVERACTUATED_GAINS_PID_I_GAIN_THETA,
        OVERACTUATED_GAINS_PID_I_GAIN_PSI_AZ,
    },
    .d = { OVERACTUATED_GAINS_PID_D_GAIN_PHI,
        OVERACTUATED_GAINS_PID_D_GAIN_THETA,
        OVERACTUATED_GAINS_PID_D_GAIN_PSI_AZ,
    } 
};
struct PD_indi_over cruise_gains = {
    .p = { OVERACTUATED_GAINS_CRUISE_GAIN_P,
        OVERACTUATED_GAINS_CRUISE_GAIN_Q,
        OVERACTUATED_GAINS_CRUISE_GAIN_R,
        OVERACTUATED_GAINS_CRUISE_GAIN_X,
        OVERACTUATED_GAINS_CRUISE_GAIN_Y,
        OVERACTUATED_GAINS_CRUISE_GAIN_Z
    },
    .d = { OVERACTUATED_GAINS_CRUISE_GAIN_P_DOT,
        OVERACTUATED_GAINS_CRUISE_GAIN_Q_DOT,
        OVERACTUATED_GAINS_CRUISE_GAIN_R_DOT,
        OVERACTUATED_GAINS_CRUISE_GAIN_X_DOT,
        OVERACTUATED_GAINS_CRUISE_GAIN_Y_DOT,
        OVERACTUATED_GAINS_CRUISE_GAIN_Z_DOT
    } 
};
struct PD_indi_over app_gains = {
    .p = { OVERACTUATED_GAINS_APP_GAIN_P,
        OVERACTUATED_GAINS_APP_GAIN_Q,
        OVERACTUATED_GAINS_APP_GAIN_R,
        OVERACTUATED_GAINS_APP_GAIN_X,
        OVERACTUATED_GAINS_APP_GAIN_Y,
        OVERACTUATED_GAINS_APP_GAIN_Z
    },
    .d = { OVERACTUATED_GAINS_APP_GAIN_P_DOT,
        OVERACTUATED_GAINS_APP_GAIN_Q_DOT,
        OVERACTUATED_GAINS_APP_GAIN_R_DOT,
        OVERACTUATED_GAINS_APP_GAIN_X_DOT,
        OVERACTUATED_GAINS_APP_GAIN_Y_DOT,
        OVERACTUATED_GAINS_APP_GAIN_Z_DOT
    } 
};
struct PD_indi_over active_gains;
static bool gains_changed_app = false, gains_changed_cruise = false;
float max_fwd_speed, max_airspeed_am7, min_fwd_speed, max_lat_speed, max_vert_speed;
float max_fwd_acc, min_fwd_acc, max_lat_acc, max_vert_acc;
       

void overactuated_mixing_parse_SHIP_INFO_MSG(uint8_t *buf) {
    if(DL_SHIP_INFO_MSG_ac_id(buf) != AC_ID)
    return;
    ship_info_receive.phi = DL_SHIP_INFO_MSG_phi(buf);  
    ship_info_receive.theta = DL_SHIP_INFO_MSG_theta(buf);  
    ship_info_receive.psi = DL_SHIP_INFO_MSG_psi(buf);  
    ship_info_receive.phi_dot = DL_SHIP_INFO_MSG_phi_dot(buf);  
    ship_info_receive.theta_dot = DL_SHIP_INFO_MSG_theta_dot(buf);  
    ship_info_receive.psi_dot = DL_SHIP_INFO_MSG_psi_dot(buf);  
    ship_info_receive.x = DL_SHIP_INFO_MSG_x(buf);  
    ship_info_receive.y = DL_SHIP_INFO_MSG_y(buf); 
    ship_info_receive.z = DL_SHIP_INFO_MSG_z(buf);  
    ship_info_receive.lat = DL_SHIP_INFO_MSG_lat_ship(buf);  
    ship_info_receive.lon = DL_SHIP_INFO_MSG_long_ship(buf); 
    ship_info_receive.alt = DL_SHIP_INFO_MSG_alt_ship(buf);      
    ship_info_receive.x_dot = DL_SHIP_INFO_MSG_x_dot(buf);  
    ship_info_receive.y_dot = DL_SHIP_INFO_MSG_y_dot(buf); 
    ship_info_receive.z_dot = DL_SHIP_INFO_MSG_z_dot(buf);  
    ship_info_receive.x_ddot = DL_SHIP_INFO_MSG_x_ddot(buf);  
    ship_info_receive.y_ddot = DL_SHIP_INFO_MSG_y_ddot(buf); 
    ship_info_receive.z_ddot = DL_SHIP_INFO_MSG_z_ddot(buf); 
}

/**
 * Function for the message SHIP_INFO_MSG_GROUND
 */
static void send_ship_info_msg_ground( struct transport_tx *trans , struct link_device * dev ) {
    // Send telemetry message
    pprz_msg_send_SHIP_INFO_MSG_GROUND(trans , dev , AC_ID ,
                & ship_info_receive.phi,& ship_info_receive.theta,& ship_info_receive.psi, & ship_info_receive.psi, & ship_info_receive.psi,
                & ship_info_receive.phi_dot,& ship_info_receive.theta_dot,& ship_info_receive.psi_dot,
                & ship_info_receive.x,& ship_info_receive.y,& ship_info_receive.z,
                & ship_info_receive.lat,& ship_info_receive.lon,& ship_info_receive.alt,
                & ship_info_receive.x_dot,& ship_info_receive.y_dot,& ship_info_receive.z_dot, 
                & ship_info_receive.x_ddot,& ship_info_receive.y_ddot,& ship_info_receive.z_ddot);
}

/**
 * Function for the message OVERACTUATED_VARIABLES
 */
static void send_overactuated_variables( struct transport_tx *trans , struct link_device * dev ) {
    //Recall the function to detect the ground on landing
    uint8_t ground_detected_am_telemetry = detect_ground_on_landing(); 
    pprz_msg_send_OVERACTUATED_VARIABLES(trans , dev , AC_ID ,
                                         & airspeed, 
                                         & control_mode_ovc_vehicle , 
                                         & ground_detected_am_telemetry,
                                         & beta_deg,
                                         & pos_vect[0], & pos_vect[1], & pos_vect[2],
                                         & speed_vect_control_rf[0], & speed_vect_control_rf[1], & speed_vect_control_rf[2],
                                         & accel_vect_filt_control_rf[0], & accel_vect_filt_control_rf[1], & accel_vect_filt_control_rf[2],
                                         & rate_vect_dot_filt[0], & rate_vect_dot_filt[1], & rate_vect_dot_filt[2],
                                         & rate_vect_filt[0], & rate_vect_filt[1], & rate_vect_filt[2],
                                         & euler_vect[0], & euler_vect[1], & euler_vect[2],
                                         & euler_setpoint[0], & euler_setpoint[1], & euler_setpoint[2],
                                         & rate_setpoint[0], & rate_setpoint[1], & rate_setpoint[2],
                                         & acc_setpoint[3], & acc_setpoint[4], & acc_setpoint[5],
                                         & pos_setpoint[0], & pos_setpoint[1], & pos_setpoint[2], 
                                         & speed_setpoint_control_rf[0], & speed_setpoint_control_rf[1], & speed_setpoint_control_rf[2],
                                         & acc_setpoint[0], & acc_setpoint[1], & acc_setpoint[2],
                                         & feed_fwd_term_yaw, & feed_back_term_yaw);
}

/**
 * Function that computes a value linearly passing from 0 to 1. 
 * if current_speed < start_speed the output is 0
 * if start_speed < current_speed < end_speed the output is linearly increasing from 0 to 1
 * if current_speed > end_speed the output is 1
 */
float compute_lat_speed_multiplier(float start_speed, float end_speed, float current_speed){
    float lat_speed_multiplier = (current_speed - start_speed) / (end_speed - start_speed);
    Bound(lat_speed_multiplier , 0, 1);
    return lat_speed_multiplier;
}

/**
 * Transpose an array from earth reference frame to control reference frame
 */
void from_earth_to_control(float * out_array, float * in_array, float Psi){
    float R_gc_matrix[3][3];
    R_gc_matrix[0][0] = cos(Psi);
    R_gc_matrix[0][1] = -sin(Psi);
    R_gc_matrix[0][2] = 0;
    R_gc_matrix[1][0] = sin(Psi) ;
    R_gc_matrix[1][1] = cos(Psi) ;
    R_gc_matrix[1][2] = 0 ;
    R_gc_matrix[2][0] = 0 ;
    R_gc_matrix[2][1] = 0 ;
    R_gc_matrix[2][2] = 1 ;

    //Do the multiplication between the income array and the transposition matrix:
    for (int j = 0; j < 3; j++) {
        //Initialize value to zero:
        out_array[j] = 0.;
        for (int k = 0; k < 3; k++) {
            out_array[j] += in_array[k] * R_gc_matrix[k][j];
        }
    }
}

/**
 * Function that computes the yaw rate for the coordinate turn:
 */
float compute_yaw_rate_turn(void){
            //Compute the yaw rate for the coordinate turn:
        float yaw_rate_setpoint_turn = 0;
        float airspeed_turn = airspeed;
        //We are dividing by the airspeed, so a lower bound is important
        Bound(airspeed_turn,10.0,30.0);

        float accel_y_filt_corrected = 0;

        float local_gain_K_T = 1 - airspeed*K_T_airspeed ;
        Bound( local_gain_K_T, 0.1, 1);
        float K_T_airspeed_corrected = local_gain_K_T * AM7_SETTINGS_VEHICLE_MOTOR_K_T_OMEGASQ;

        float side_thrust_motor_1 = get_act_states_T4()->motor_1_rad_s_filt * get_act_states_T4()->motor_1_rad_s_filt * K_T_airspeed_corrected * sin(get_act_states_T4()->az_1_angle_deg_corrected * M_PI/180) * cos(get_act_states_T4()->el_1_angle_deg_corrected * M_PI/180);
        float side_thrust_motor_2 = get_act_states_T4()->motor_2_rad_s_filt * get_act_states_T4()->motor_2_rad_s_filt * K_T_airspeed_corrected * sin(get_act_states_T4()->az_2_angle_deg_corrected * M_PI/180) * cos(get_act_states_T4()->el_2_angle_deg_corrected * M_PI/180);
        float side_thrust_motor_3 = get_act_states_T4()->motor_3_rad_s_filt * get_act_states_T4()->motor_3_rad_s_filt * K_T_airspeed_corrected * sin(get_act_states_T4()->az_3_angle_deg_corrected * M_PI/180) * cos(get_act_states_T4()->el_3_angle_deg_corrected * M_PI/180);
        float side_thrust_motor_4 = get_act_states_T4()->motor_4_rad_s_filt * get_act_states_T4()->motor_4_rad_s_filt * K_T_airspeed_corrected * sin(get_act_states_T4()->az_4_angle_deg_corrected * M_PI/180) * cos(get_act_states_T4()->el_4_angle_deg_corrected * M_PI/180);
        //Overestimation of the lateral acceleration, to minimize sideslip in hovering:              
        accel_y_filt_corrected = accel_body_y_filter.o[0] 
                                - overestimation_coeff * side_thrust_motor_1 /AM7_SETTINGS_VEHICLE_MASS
                                - overestimation_coeff * side_thrust_motor_2 /AM7_SETTINGS_VEHICLE_MASS
                                - overestimation_coeff * side_thrust_motor_3 /AM7_SETTINGS_VEHICLE_MASS
                                - overestimation_coeff * side_thrust_motor_4 /AM7_SETTINGS_VEHICLE_MASS;

        yaw_rate_setpoint_turn = accel_vect_filt_control_rf[1]/airspeed_turn - K_beta * accel_y_filt_corrected;
        feed_fwd_term_yaw = accel_vect_filt_control_rf[1]/airspeed_turn;
        feed_back_term_yaw = - K_beta * accel_y_filt_corrected;
        
        return yaw_rate_setpoint_turn;
}

/**
 * ABI callback that obtains the velocity setpoint from a module and makes it in the control reference frame
  */
static void vel_sp_cb(uint8_t sender_id __attribute__((unused)), struct FloatVect3 *vel_sp)
{
    float des_speed_approach_earth_rf[3] = {vel_sp->x , vel_sp->y, vel_sp->z};
    time_of_speed_setpoint_approach = get_sys_time_float();
    from_earth_to_control( des_speed_approach_control_rf, des_speed_approach_earth_rf, euler_vect[2]);
}

/**
 * Function that computes the speed reference in the control reference frame, taking as input 
 * the current vehicle position and the desired position in the ground reference frame:
 */
void compute_speed_ref_from_waypoint(float * speed_reference_control_rf, float * dest_pos_ground_rf, float * current_pos_ground_rf, float airspeed_vehicle, float Psi){
    float pos_error_earth_rf[3], pos_error_control_rf[3];
    //Compute position error in ground reference frame 
    for( int i=0; i<3; i++){
        pos_error_earth_rf[i] = dest_pos_ground_rf[i] - current_pos_ground_rf[i];
    }
    //Transpose position error from ground rf to control rf: 
    from_earth_to_control(pos_error_control_rf, pos_error_earth_rf, Psi);

    //Compute the heading angle needed to get to the desired waypoint:
    float track_heading = atan2f(pos_error_control_rf[1],pos_error_control_rf[0]);

    //Apply saturation block to the maximum position error: 
    BoundAbs(pos_error_control_rf[0],WP_CONTROL_MAX_POS_XY_ERROR);
    BoundAbs(pos_error_control_rf[1],WP_CONTROL_MAX_POS_XY_ERROR);
    BoundAbs(pos_error_control_rf[2],WP_CONTROL_MAX_POS_Z_ERROR);

    //Compute the the constarined waypoint distance in the x-y plane:
    float pos_error_xy_norm = sqrt(pos_error_control_rf[0] * pos_error_control_rf[0] + pos_error_control_rf[1] * pos_error_control_rf[1]);

    //Let's compute the dynamic saturation point for the fwd speed, based on the control-x distance of the WP: 
    float max_fwd_speed_approach_wp = sqrt( pos_error_xy_norm * 2 * WP_CONTROL_MAX_DECEL_WP_APPROACH );

    //Now rescale the position error using the track heading and constrained waypoint distance:
    pos_error_control_rf[0] = pos_error_xy_norm * cosf(track_heading);
    pos_error_control_rf[1] = pos_error_xy_norm * sinf(track_heading);

    //Now apply static gains to the position error to generate the speed references in the control rf: 
    speed_reference_control_rf[0] = pos_error_control_rf[0] * WP_CONTROL_VX_CONTROL_STATIC_GAIN;
    speed_reference_control_rf[1] = pos_error_control_rf[1] * WP_CONTROL_VY_CONTROL_STATIC_GAIN;
    speed_reference_control_rf[2] = pos_error_control_rf[2] * WP_CONTROL_VZ_CONTROL_STATIC_GAIN;

    //Now compute and apply the Vy and Vz gains with the airspeed dependency: 
    float Vy_dyn_gain = 1 + WP_CONTROL_VY_AIRSPEED_GAIN_COEFF * airspeed_vehicle;
    float Vz_dyn_gain = 1 + WP_CONTROL_VZ_AIRSPEED_GAIN_COEFF * airspeed_vehicle;
    Bound(Vy_dyn_gain,WP_CONTROL_VY_GAIN_MIN_VAL,WP_CONTROL_VY_GAIN_MAX_VAL);
    Bound(Vz_dyn_gain,WP_CONTROL_VZ_GAIN_MIN_VAL,WP_CONTROL_VZ_GAIN_MAX_VAL);

    speed_reference_control_rf[1] = speed_reference_control_rf[1] * Vy_dyn_gain;
    speed_reference_control_rf[2] = speed_reference_control_rf[2] * Vz_dyn_gain;

    //Apply approach constrain to fwd speed: 
    Bound(speed_reference_control_rf[0],min_fwd_speed,max_fwd_speed_approach_wp);

    #ifdef USE_NAV_HYBRID_MODULE
        //Horizontal part:
        float nav_hybrid_des_speed[3]; 
        nav_hybrid_des_speed[0] = nav.speed.y;
        nav_hybrid_des_speed[1] = nav.speed.x;

        //Vertical part:
        if(nav.vertical_mode == NAV_VERTICAL_MODE_CLIMB){
            nav_hybrid_des_speed[2] = -nav.climb;
        }
        else if(nav.vertical_mode == NAV_VERTICAL_MODE_ALT){
            float pos_error_z = (-nav.nav_altitude) - stateGetPositionNed_f()->z;
            nav_hybrid_des_speed[2] = pos_error_z * WP_CONTROL_VZ_CONTROL_STATIC_GAIN;
        }

        //Transpose in control rf: 
        from_earth_to_control( speed_reference_control_rf, nav_hybrid_des_speed, euler_vect[2]);

    #endif

    //If we are in the ewoud approach mode, then use these speed references: 
    if( get_sys_time_float() - time_of_speed_setpoint_approach < 0.05){ //50 mS 
        speed_reference_control_rf[0] = des_speed_approach_control_rf[0];
        speed_reference_control_rf[1] = des_speed_approach_control_rf[1];
        speed_reference_control_rf[2] = des_speed_approach_control_rf[2];
    }

}

/**
 * Initialize the filters
 */
void init_filters(void){
    float sample_time = 1.0 / OVERACTUATED_MIXING_FREQUENCY;
    init_butterworth_2_low_pass(&accel_body_y_filter, 1.0f/AM7_SETTINGS_BODY_Y_ACC_SECOND_ORDER_CUTOFF_RAD_S, sample_time, 0.0);
    for (int i = 0; i < 3; i++) {
        init_butterworth_2_low_pass(&accel_control_rf_filters[i], 1.0f/AM7_SETTINGS_ACCELERATIONS_SECOND_ORDER_CUTOFF_RAD_S, sample_time, 0.0);
        init_butterworth_2_low_pass(&body_rates_dot_filters[i], 1.0f/AM7_SETTINGS_BODY_RATES_DOT_SECOND_ORDER_CUTOFF_RAD_S, sample_time, 0.0);
        rate_vect_filt[i] = 0.0f;
    }

}

#ifdef PRINT_CPU_LOAD_ON_SD
    /**
    * @brief Check the system performance
    * 
    */
    static void status_nederdrone_sysmon(void) {

    static uint8_t cnt = 0;

    if(rtos_mon.cpu_load > 85 || ( cnt++ > 10)) {
        sdLogWriteLog(pprzLogFile, "Data reported in the RTOS_MON message:\r\n");
        sdLogWriteLog(pprzLogFile, " core free mem: %lu\r\n", rtos_mon.core_free_memory);
        sdLogWriteLog(pprzLogFile, " heap free mem: %lu\r\n", rtos_mon.heap_free_memory);
        sdLogWriteLog(pprzLogFile, " heap fragments: %lu\r\n", rtos_mon.heap_fragments);
        sdLogWriteLog(pprzLogFile, " heap largest: %lu\r\n", rtos_mon.heap_largest);
        sdLogWriteLog(pprzLogFile, " CPU load: %d %%\r\n", rtos_mon.cpu_load);
        sdLogWriteLog(pprzLogFile, " number of threads: %d\r\n", rtos_mon.thread_counter);
        sdLogWriteLog(pprzLogFile, " thread names: %s\r\n", rtos_mon.thread_names);
        for (int i = 0; i < rtos_mon.thread_counter; i++) {
        sdLogWriteLog(pprzLogFile, " thread %d load: %0.1f, free stack: %d\r\n", i,
                (float)rtos_mon.thread_load[i] / 10.f, rtos_mon.thread_free_stack[i]);
        }
        sdLogWriteLog(pprzLogFile, " CPU time: %.2f\r\n", rtos_mon.cpu_time);

        cnt = 0;
    }
    }
#endif

/**
 * This function outputs the maximum equivalent ground speed in the control reference to have a maximum desired airspeed.
 */
float max_V_control_from_max_airspeed(float current_airspeed, float current_Vx_control_rf, float max_desired_airspeed){
    float estimated_wind_fwd = current_Vx_control_rf - current_airspeed;
    float Vx_ground_max_for_airspeed = max_desired_airspeed + estimated_wind_fwd;
    return Vx_ground_max_for_airspeed;
}

/**
 * Initialize the overactuated mixing module
 */
void overactuated_mixing_init(void) {

    register_periodic_telemetry ( DefaultPeriodic , PPRZ_MSG_ID_OVERACTUATED_VARIABLES , send_overactuated_variables );
    register_periodic_telemetry ( DefaultPeriodic , PPRZ_MSG_ID_SHIP_INFO_MSG_GROUND , send_ship_info_msg_ground );
    
    //Startup the init variables of the INDI
    init_filters();

    //Init abi for the approach module: 
    AbiBindMsgVEL_SP(ABI_BROADCAST, &vel_sp_ev, vel_sp_cb);
}


/**
 * Ad each iteration upload global variables
 */
void assign_variables(void){
    //Rates and rates derivatives: 
    rate_vect_dot[0] = (stateGetBodyRates_f()->p - rate_vect[0])*OVERACTUATED_MIXING_FREQUENCY;
    rate_vect_dot[1] = (stateGetBodyRates_f()->q - rate_vect[1])*OVERACTUATED_MIXING_FREQUENCY;
    rate_vect_dot[2] = (stateGetBodyRates_f()->r - rate_vect[2])*OVERACTUATED_MIXING_FREQUENCY;
    rate_vect[0] = stateGetBodyRates_f()->p;
    rate_vect[1] = stateGetBodyRates_f()->q;
    rate_vect[2] = stateGetBodyRates_f()->r;
    euler_vect[0] = stateGetNedToBodyEulers_f()->phi;
    euler_vect[1] = stateGetNedToBodyEulers_f()->theta;
    euler_vect[2] = stateGetNedToBodyEulers_f()->psi;
    acc_vect[0] = stateGetAccelNed_f()->x;
    acc_vect[1] = stateGetAccelNed_f()->y;
    acc_vect[2] = stateGetAccelNed_f()->z;
    speed_vect[0] = stateGetSpeedNed_f()->x;
    speed_vect[1] = stateGetSpeedNed_f()->y;
    speed_vect[2] = stateGetSpeedNed_f()->z;
    pos_vect[0] = stateGetPositionNed_f()->x;
    pos_vect[1] = stateGetPositionNed_f()->y;
    pos_vect[2] = stateGetPositionNed_f()->z;
    from_earth_to_control(accel_vect_control_rf, acc_vect, euler_vect[2]);
    from_earth_to_control(speed_vect_control_rf, speed_vect, euler_vect[2]);
    beta_deg = 0;

    #if NO_AIRSPEED_NONLINEAR_CA
        airspeed = 0.1; 
    #else
        airspeed = fmax(AM7_SETTINGS_MIN_AIRSPEED_READING,ms45xx.airspeed);
    #endif

    // Propagate the filters:
    for (int i = 0; i < 3; i++) {
        update_butterworth_2_low_pass(&body_rates_dot_filters[i], rate_vect_dot[i]);
        update_butterworth_2_low_pass(&accel_control_rf_filters[i], accel_vect_control_rf[i]);
        rate_vect_dot_filt[i] = body_rates_dot_filters[i].o[0]; 
        accel_vect_filt_control_rf[i] = accel_control_rf_filters[i].o[0];
    }
    update_butterworth_2_low_pass(&accel_body_y_filter, ACCEL_FLOAT_OF_BFP(stateGetAccelBody_i()->y));
    //Filter body rates with first order dedicated filter
    float tau_first_order_body_rates = 1.0f - exp(-AM7_SETTINGS_BODY_RATES_FIRST_ORDER_CUTOFF_RAD_S/(OVERACTUATED_MIXING_FREQUENCY*1.0f));
    for(int i = 0; i < 3; i++){
        rate_vect_filt[i] = rate_vect_filt[i] + tau_first_order_body_rates * (rate_vect[i] - rate_vect_filt[i]);
    }

    //Assign gains according to the approach state: 
    if(approach_state == 1 && gains_changed_app == false){
        //Booleans to avoid changing the gains multiple times, to allow the sliders to be used:
        gains_changed_app = true;
        gains_changed_cruise = false;
        active_gains = app_gains;
        max_fwd_speed = AM7_SETTINGS_LIMITS_APP_MAX_FWD_SPEED;
        max_airspeed_am7 = AM7_SETTINGS_LIMITS_APP_MAX_AIRSPEED;
        min_fwd_speed = AM7_SETTINGS_LIMITS_APP_MIN_FWD_SPEED;
        max_lat_speed = AM7_SETTINGS_LIMITS_APP_MAX_LAT_SPEED;
        max_vert_speed = AM7_SETTINGS_LIMITS_APP_MAX_VERT_SPEED;
        max_fwd_acc = AM7_SETTINGS_LIMITS_APP_MAX_FWD_ACC;
        min_fwd_acc = AM7_SETTINGS_LIMITS_APP_MIN_FWD_ACC;
        max_lat_acc = AM7_SETTINGS_LIMITS_APP_MAX_LAT_ACC;
        max_vert_acc = AM7_SETTINGS_LIMITS_APP_MAX_VERT_ACC;
        overestimation_coeff = AM7_SETTINGS_OVERESTIMATION_COEFF_APP;
    }
    if(approach_state == 0 && gains_changed_cruise == false){
        //Booleans to avoid changing the gains multiple times, to allow the sliders to be used:
        gains_changed_app = false;
        gains_changed_cruise = true;
        active_gains = cruise_gains;
        max_fwd_speed = AM7_SETTINGS_LIMITS_CRUISE_MAX_FWD_SPEED;
        max_airspeed_am7 = AM7_SETTINGS_LIMITS_CRUISE_MAX_AIRSPEED;
        min_fwd_speed = AM7_SETTINGS_LIMITS_CRUISE_MIN_FWD_SPEED;
        max_lat_speed = AM7_SETTINGS_LIMITS_CRUISE_MAX_LAT_SPEED;
        max_vert_speed = AM7_SETTINGS_LIMITS_CRUISE_MAX_VERT_SPEED;
        max_fwd_acc = AM7_SETTINGS_LIMITS_CRUISE_MAX_FWD_ACC;
        min_fwd_acc = AM7_SETTINGS_LIMITS_CRUISE_MIN_FWD_ACC;
        max_lat_acc = AM7_SETTINGS_LIMITS_CRUISE_MAX_LAT_ACC;
        max_vert_acc = AM7_SETTINGS_LIMITS_CRUISE_MAX_VERT_ACC;
        overestimation_coeff = AM7_SETTINGS_OVERESTIMATION_COEFF_CRUISE;
    }

    //Compute an estimation of the flight path angle:
    float smooth_gain_gamma = (airspeed - AM7_SETTINGS_MIN_AOA_ESTIMATION_AIRSPEED) / (AM7_SETTINGS_AOA_ESTIMATION_AIRSPEED - AM7_SETTINGS_MIN_AOA_ESTIMATION_AIRSPEED);
    Bound(smooth_gain_gamma , 0, 1); // 0 until min_speed and 1 above ref_speed
    float flight_path_angle_offset = fpa_off_deg*M_PI/180;
    float flight_path_angle_airspeed = flight_path_angle_offset;
    float projected_airspeed_on_x_control = 0.0f;
    if(fabs(cosf(euler_vect[1])) > 0.001f){
        projected_airspeed_on_x_control = fabs(airspeed/cosf(euler_vect[1]));
    }
    if(projected_airspeed_on_x_control > 1 && projected_airspeed_on_x_control > fabs(speed_vect[2])){
        flight_path_angle_airspeed = flight_path_angle_airspeed + asin(-speed_vect[2]/projected_airspeed_on_x_control);
        BoundAbs(flight_path_angle_airspeed, M_PI/2);
    }
    //Mix the two values: 
    flight_path_angle = smooth_gain_gamma * flight_path_angle_airspeed + (1-smooth_gain_gamma)*flight_path_angle_offset;

    //Append the values to the 
    data_to_am7_module.packet_timestamp = get_sys_time_float();
    data_to_am7_module.est_flight_path_angle_rad = flight_path_angle;
    data_to_am7_module.est_airspeed = airspeed;
    data_to_am7_module.est_beta_rad = beta_deg * M_PI / 180;
}

/**
 * Run the overactuated mixing
 */
void overactuated_mixing_run(void)
{
    //Assign variables
    assign_variables();

    #ifdef PRINT_CPU_LOAD_ON_SD
        //Write to sysmon every 1 second if required by debug enable
        if(get_sys_time_float() - time_old_sys_mon >= 1 ){
            status_nederdrone_sysmon();
            time_old_sys_mon = get_sys_time_float();
        }
    #endif

    // Manual PID control [FAILSAFE]
    if(autopilot.mode == AP_MODE_RC_DIRECT) {
        if(!autopilot.motors_on || control_mode_ovc_vehicle != 1){
            //INITIALIZATION OF THE RC DIRECT MODE:  
            for (int i = 0; i < 3; i++) {
                euler_error_integrated[i] = 0;
            }
            euler_setpoint[2] = euler_vect[2];    
        }
        control_mode_ovc_vehicle = 1;

        ////Angular error computation
        euler_setpoint[0] = 0;
        euler_setpoint[1] = 0;
        if (abs(radio_control.values[RADIO_YAW]) > deadband_stick_yaw ) {
            euler_setpoint[2] =
                    euler_setpoint[2] + stick_gain_yaw * radio_control.values[RADIO_YAW] * M_PI / 180 * .001;
            //Correct the setpoint in order to always be within -pi and pi
            if (euler_setpoint[2] > M_PI) {
                euler_setpoint[2] -= 2 * M_PI;
            }
            else if (euler_setpoint[2] < -M_PI) {
                euler_setpoint[2] += 2 * M_PI;
            }
        }

        //Bound the setpoints within maximum angular values
        BoundAbs(euler_setpoint[0], OVERACTUATED_GAINS_MAX_PHI);
        BoundAbs(euler_setpoint[1], OVERACTUATED_GAINS_MAX_THETA);

        euler_error[0] = euler_setpoint[0] - euler_vect[0];
        euler_error[1] = euler_setpoint[1] - euler_vect[1];
        euler_error[2] = euler_setpoint[2] - euler_vect[2];

        //Add logic for the psi control:
        if (euler_error[2] > M_PI) {
            euler_error[2] -= 2 * M_PI;
        }
        else if (euler_error[2] < -M_PI) {
            euler_error[2] += 2 * M_PI;
        }

        //Calculate and bound the angular error integration term for the PID
        for (int i = 0; i < 3; i++) {
            euler_error_integrated[i] += euler_error[i] / OVERACTUATED_MIXING_FREQUENCY;
            BoundAbs(euler_error_integrated[i], OVERACTUATED_GAINS_PID_MAX_EULER_ERR_INTEGRATIVE);
        }

        euler_cmd_PID[0] = pid_gains_over.p.phi * euler_error[0] + pid_gains_over.i.phi * euler_error_integrated[0] -
                         pid_gains_over.d.phi * rate_vect_filt[0];
        euler_cmd_PID[1] = pid_gains_over.p.theta * euler_error[1] + pid_gains_over.i.theta * euler_error_integrated[1] -
                         pid_gains_over.d.theta * rate_vect_filt[1];
        euler_cmd_PID[2] = pid_gains_over.p.psi * euler_error[2] + pid_gains_over.i.psi * euler_error_integrated[2] -
                         pid_gains_over.d.psi * rate_vect_filt[2];

        //Bound euler angle orders:
        BoundAbs(euler_cmd_PID[0], OVERACTUATED_GAINS_PID_MAX_ROLL_ORDER_DSHOT);
        BoundAbs(euler_cmd_PID[1], OVERACTUATED_GAINS_PID_MAX_PITCH_ORDER_DSHOT);
        BoundAbs(euler_cmd_PID[2], OVERACTUATED_GAINS_PID_MAX_YAW_ORDER_AZ_DEG);
        

        act_cmd_to_t4.cmd_timestamp = get_sys_time_float();
        act_cmd_to_t4.motor_control_mode = 1;
        //Fill the motor commands:
        float K_ppz_to_dshot = (float) FBW_T4_MAX_DSHOT_CMD/MAX_PPRZ;
        act_cmd_to_t4.motor_1_cmd = (float) (euler_cmd_PID[0] + euler_cmd_PID[1]) + radio_control.values[RADIO_THROTTLE] * K_ppz_to_dshot;
        act_cmd_to_t4.motor_2_cmd = (float) (-euler_cmd_PID[0] + euler_cmd_PID[1]) + radio_control.values[RADIO_THROTTLE] * K_ppz_to_dshot;
        act_cmd_to_t4.motor_3_cmd = (float) (-euler_cmd_PID[0] - euler_cmd_PID[1]) + radio_control.values[RADIO_THROTTLE] * K_ppz_to_dshot;
        act_cmd_to_t4.motor_4_cmd = (float) (euler_cmd_PID[0] - euler_cmd_PID[1]) + radio_control.values[RADIO_THROTTLE] * K_ppz_to_dshot;
        //FIll the longitudinal servo commands:
        act_cmd_to_t4.servo_el_1_angle_deg = (radio_control.values[RADIO_PITCH]*1.0f/MAX_PPRZ*1.0f) * OVERACTUATED_GAINS_PID_MAX_EL_ORDER_DEG;
        act_cmd_to_t4.servo_el_2_angle_deg = (radio_control.values[RADIO_PITCH]*1.0f/MAX_PPRZ*1.0f) * OVERACTUATED_GAINS_PID_MAX_EL_ORDER_DEG;
        act_cmd_to_t4.servo_el_3_angle_deg = (radio_control.values[RADIO_PITCH]*1.0f/MAX_PPRZ*1.0f) * OVERACTUATED_GAINS_PID_MAX_EL_ORDER_DEG;
        act_cmd_to_t4.servo_el_4_angle_deg = (radio_control.values[RADIO_PITCH]*1.0f/MAX_PPRZ*1.0f) * OVERACTUATED_GAINS_PID_MAX_EL_ORDER_DEG;
        //Fill the lateral servo commands:
        act_cmd_to_t4.servo_az_1_angle_deg = (radio_control.values[RADIO_ROLL]*1.0f/MAX_PPRZ*1.0f) * OVERACTUATED_GAINS_PID_MAX_AZ_ORDER_DEG;
        act_cmd_to_t4.servo_az_2_angle_deg = (radio_control.values[RADIO_ROLL]*1.0f/MAX_PPRZ*1.0f) * OVERACTUATED_GAINS_PID_MAX_AZ_ORDER_DEG;
        act_cmd_to_t4.servo_az_3_angle_deg = (radio_control.values[RADIO_ROLL]*1.0f/MAX_PPRZ*1.0f) * OVERACTUATED_GAINS_PID_MAX_AZ_ORDER_DEG;
        act_cmd_to_t4.servo_az_4_angle_deg = (radio_control.values[RADIO_ROLL]*1.0f/MAX_PPRZ*1.0f) * OVERACTUATED_GAINS_PID_MAX_AZ_ORDER_DEG;
        //Add the yaw commands on top of the azimuth commands:
        act_cmd_to_t4.servo_az_1_angle_deg += euler_cmd_PID[2]; 
        act_cmd_to_t4.servo_az_2_angle_deg += euler_cmd_PID[2];
        act_cmd_to_t4.servo_az_3_angle_deg -= euler_cmd_PID[2];
        act_cmd_to_t4.servo_az_4_angle_deg -= euler_cmd_PID[2];
        //Fill the flaperon commands:
        act_cmd_to_t4.flaperon_right_angle_deg = 0;
        act_cmd_to_t4.flaperon_left_angle_deg = 0;

        if(autopilot.motors_on){
            act_cmd_to_t4.motor_arm = 1;
            act_cmd_to_t4.servo_arm = 1;
        }
        else{
            act_cmd_to_t4.motor_arm = 0;
            act_cmd_to_t4.servo_arm = 0;
            //Kill motors and initialize a spiral mode with the flaperons:
            act_cmd_to_t4.flaperon_right_angle_deg = 15; 
            act_cmd_to_t4.flaperon_left_angle_deg = 15;
        }
    }
    // Manual INDI nonlinear control
    else if(autopilot.mode == AP_MODE_HOVER_DIRECT){
        if(control_mode_ovc_vehicle != 2){
            //INITIALIZATION OF THE HOVER DIRECT MODE:  
        }
        control_mode_ovc_vehicle = 2;

        //Calculate the desired values in this mode:
        data_to_am7_module.desired_phi_rad = AM7_SETTINGS_MAX_CMD_ROLL_ANGLE * radio_control.values[RADIO_MANUAL_ROLL_CMD] / MAX_PPRZ;
        data_to_am7_module.desired_theta_rad = AM7_SETTINGS_MAX_CMD_PITCH_ANGLE * radio_control.values[RADIO_MANUAL_PITCH_CMD] / MAX_PPRZ;
        data_to_am7_module.desired_motor_rad_s = 0; data_to_am7_module.desired_el_rad = 0; data_to_am7_module.desired_az_rad = 0;
        data_to_am7_module.desired_ail_rad = 0;

        //Generate the pseudocontrol array: 
        euler_setpoint[0] = get_am7_data_in()->phi_cmd_int * 0.01f * M_PI/180;
        euler_setpoint[1] = get_am7_data_in()->theta_cmd_int * 0.01f * M_PI/180;
        BoundAbs(euler_setpoint[0],OVERACTUATED_GAINS_MAX_PHI);
        BoundAbs(euler_setpoint[1],OVERACTUATED_GAINS_MAX_THETA);
        euler_error[0] = euler_setpoint[0] - euler_vect[0];
        euler_error[1] = euler_setpoint[1] - euler_vect[1];
        // For the yaw, we can directly control the rates:
        float yaw_rate_setpoint_manual = 0;
        if(abs(radio_control.values[RADIO_YAW]) >= 100){
            yaw_rate_setpoint_manual = AM7_SETTINGS_MAX_CMD_YAW_RATE * radio_control.values[RADIO_YAW] / MAX_PPRZ;
        }
        euler_error[2] = yaw_rate_setpoint_manual;
        #if !FULLY_MANUAL_HEADING
            euler_error[2] += compute_yaw_rate_turn();
        #endif

        //Update the value to the structure to be sent to the AM7 module: 
        data_to_am7_module.psi_dot_cmd_rad_s = euler_error[2];

        //Apply euler angle gains: 
        float gain_to_speed_constant = 1 - airspeed * k_gain_airspeed; 
        Bound(gain_to_speed_constant, 0.1, 1);
        float phi_dot = euler_error[0]  * active_gains.p.phi * gain_to_speed_constant;
        float theta_dot = euler_error[1]  * active_gains.p.theta * gain_to_speed_constant;
        float psi_dot = euler_error[2];
        float phi_value = euler_vect[0];
        float theta_value = euler_vect[1];

        //Calculate the body error using the body to euler conversion: 
        rate_setpoint[0] = phi_dot - sin(theta_value) * psi_dot;
        rate_setpoint[1] = cos(phi_value) * theta_dot + sin(phi_value) * cos(theta_value) * psi_dot;
        rate_setpoint[2] = -sin(phi_value) * theta_dot + cos(phi_value) * cos(theta_value) * psi_dot;

        //Compute the angular acceleration setpoint using the filtered rates:
        acc_setpoint[3] = (rate_setpoint[0] - rate_vect_filt[0]) * active_gains.d.phi * gain_to_speed_constant;
        acc_setpoint[4] = (rate_setpoint[1] - rate_vect_filt[1]) * active_gains.d.theta * gain_to_speed_constant;
        acc_setpoint[5] = (rate_setpoint[2] - rate_vect_filt[2]) * active_gains.d.psi * gain_to_speed_constant;

        //Compute the pseudocontrol angular array:
        data_to_am7_module.pseudocontrol_p_dot = acc_setpoint[3] - rate_vect_dot_filt[0];
        data_to_am7_module.pseudocontrol_q_dot = acc_setpoint[4] - rate_vect_dot_filt[1];
        data_to_am7_module.pseudocontrol_r_dot = acc_setpoint[5] - rate_vect_dot_filt[2];

        //Compute the speed setpoints in the control reference frame using the right sticks:
        speed_setpoint_control_rf[0] = - AM7_SETTINGS_MAX_CMD_FWD_SPEED * radio_control.values[RADIO_PITCH]/MAX_PPRZ;
        speed_setpoint_control_rf[1] = AM7_SETTINGS_MAX_CMD_LAT_SPEED * radio_control.values[RADIO_ROLL]/MAX_PPRZ;
        speed_setpoint_control_rf[2] = 0;
        //Compute the vertical speed setpoint using the throttle stick:
        if( abs(radio_control.values[RADIO_THROTTLE] - MAX_PPRZ/2) > deadband_stick_throttle ) {
            speed_setpoint_control_rf[2] = -AM7_SETTINGS_MAX_CMD_VERT_SPEED * (radio_control.values[RADIO_THROTTLE] - MAX_PPRZ/2) / MAX_PPRZ/2;
        }
        
        //Bound speeds based on the maximum airspeed or maximum ground speed:
        float max_Vx_airspeed = max_V_control_from_max_airspeed(airspeed, speed_vect_control_rf[0], max_airspeed_am7);
        Bound(speed_setpoint_control_rf[0],min_fwd_speed,Min(max_fwd_speed,max_Vx_airspeed));
        BoundAbs(speed_setpoint_control_rf[1],max_lat_speed);
        BoundAbs(speed_setpoint_control_rf[2],max_vert_speed);

        //Compute the speed error in the control rf:
        speed_error_vect_control_rf[0] = speed_setpoint_control_rf[0] - speed_vect_control_rf[0];
        speed_error_vect_control_rf[1] = speed_setpoint_control_rf[1] - speed_vect_control_rf[1] * (1 - compute_lat_speed_multiplier(min_speed_transition,ref_speed_transition,airspeed));
        speed_error_vect_control_rf[2] = speed_setpoint_control_rf[2] - speed_vect_control_rf[2];

        //Compute the acceleration setpoints in the control rf:
        acc_setpoint[0] = speed_error_vect_control_rf[0] * active_gains.d.x;
        acc_setpoint[1] = speed_error_vect_control_rf[1] * active_gains.d.y;
        acc_setpoint[2] = speed_error_vect_control_rf[2] * active_gains.d.z;

        //Apply saturation points for the accelerations in the control rf:
        Bound(acc_setpoint[0],min_fwd_acc,max_fwd_acc);
        BoundAbs(acc_setpoint[1],max_lat_acc);
        BoundAbs(acc_setpoint[2],max_vert_acc);

        //Compute the pseudocontrol acceleration array:
        data_to_am7_module.pseudocontrol_ax = acc_setpoint[0] - accel_vect_filt_control_rf[0];
        data_to_am7_module.pseudocontrol_ay = acc_setpoint[1] - accel_vect_filt_control_rf[1];
        data_to_am7_module.pseudocontrol_az = acc_setpoint[2] - accel_vect_filt_control_rf[2];

        //Produce commands based on what the am7 returned: 
        act_cmd_to_t4.cmd_timestamp = get_sys_time_float();
        if(autopilot.motors_on){
            act_cmd_to_t4.motor_arm = 1;
            act_cmd_to_t4.servo_arm = 1;
        }
        else{
            act_cmd_to_t4.motor_arm = 0;
            act_cmd_to_t4.servo_arm = 0;
        }
        act_cmd_to_t4.motor_control_mode = 2;
        act_cmd_to_t4.motor_1_cmd = (float) get_am7_data_in()->motor_1_cmd_int * 0.1f; 
        act_cmd_to_t4.motor_2_cmd = (float) get_am7_data_in()->motor_2_cmd_int * 0.1f;
        act_cmd_to_t4.motor_3_cmd = (float) get_am7_data_in()->motor_3_cmd_int * 0.1f;
        act_cmd_to_t4.motor_4_cmd = (float) get_am7_data_in()->motor_4_cmd_int * 0.1f;
        act_cmd_to_t4.servo_el_1_angle_deg = (float) get_am7_data_in()->el_1_cmd_int * 0.01f;
        act_cmd_to_t4.servo_el_2_angle_deg = (float) get_am7_data_in()->el_2_cmd_int * 0.01f;
        act_cmd_to_t4.servo_el_3_angle_deg = (float) get_am7_data_in()->el_3_cmd_int * 0.01f;
        act_cmd_to_t4.servo_el_4_angle_deg = (float) get_am7_data_in()->el_4_cmd_int * 0.01f;
        act_cmd_to_t4.servo_az_1_angle_deg = (float) get_am7_data_in()->az_1_cmd_int * 0.01f;
        act_cmd_to_t4.servo_az_2_angle_deg = (float) get_am7_data_in()->az_2_cmd_int * 0.01f;
        act_cmd_to_t4.servo_az_3_angle_deg = (float) get_am7_data_in()->az_3_cmd_int * 0.01f;
        act_cmd_to_t4.servo_az_4_angle_deg = (float) get_am7_data_in()->az_4_cmd_int * 0.01f;
        act_cmd_to_t4.flaperon_right_angle_deg = (float) get_am7_data_in()->ailerons_cmd_int * 0.01f;
        act_cmd_to_t4.flaperon_left_angle_deg = (float) get_am7_data_in()->ailerons_cmd_int * 0.01f;

        if(autopilot.motors_on){
            act_cmd_to_t4.motor_arm = 1;
            act_cmd_to_t4.servo_arm = 1;
        }
        else{
            act_cmd_to_t4.motor_arm = 0;
            act_cmd_to_t4.servo_arm = 0;
            //Kill motors and initialize a spiral mode with the flaperons:
            act_cmd_to_t4.flaperon_right_angle_deg = 15; 
            act_cmd_to_t4.flaperon_left_angle_deg = 15;
        }
        
        //Submit the data to the AM7 module:
        AbiSendMsgAM7_DATA_OUT(ABI_AM7_DATA_OUT_ID, &data_to_am7_module);
    }
    // NAV INDI nonlinear control
    else if(autopilot.mode == AP_MODE_NAV){ 
        if(control_mode_ovc_vehicle != 3){
            //INITIALIZATION OF THE NAVIGATION MODE:  
        }
        control_mode_ovc_vehicle = 3;

        //Calculate the desired values:
        data_to_am7_module.desired_phi_rad = AM7_SETTINGS_MAX_CMD_ROLL_ANGLE * radio_control.values[RADIO_MANUAL_ROLL_CMD] / MAX_PPRZ;
        data_to_am7_module.desired_theta_rad = AM7_SETTINGS_MAX_CMD_PITCH_ANGLE * radio_control.values[RADIO_MANUAL_PITCH_CMD] / MAX_PPRZ;
        data_to_am7_module.desired_motor_rad_s = 0; data_to_am7_module.desired_el_rad = 0; data_to_am7_module.desired_az_rad = 0;
        data_to_am7_module.desired_ail_rad = 0;

        #if USE_SHIP_BOX_EXT_REF_ATTITUDE
            if(approach_state){
                data_to_am7_module.desired_phi_rad = ship_info_receive.phi * M_PI/180;
                data_to_am7_module.desired_theta_rad = ship_info_receive.theta * M_PI/180;
            }
        #endif

        #if USE_SIXDOF_EXT_REF_ATTITUDE
            if(approach_state && get_am7_data_in()->sixdof_system_status == 3){
                data_to_am7_module.desired_phi_rad = - (get_am7_data_in()->sixdof_relative_phi * 0.01f * M_PI/180 - euler_vect[0]);
                data_to_am7_module.desired_theta_rad = - (get_am7_data_in()->sixdof_relative_theta * 0.01f * M_PI/180  - euler_vect[1]);
            }
        #endif     

        //Generate the pseudocontrol array: 
        euler_setpoint[0] = get_am7_data_in()->phi_cmd_int * 0.01f * M_PI/180;
        euler_setpoint[1] = get_am7_data_in()->theta_cmd_int * 0.01f * M_PI/180;
        BoundAbs(euler_setpoint[0],OVERACTUATED_GAINS_MAX_PHI);
        BoundAbs(euler_setpoint[1],OVERACTUATED_GAINS_MAX_THETA);
        euler_error[0] = euler_setpoint[0] - euler_vect[0];
        euler_error[1] = euler_setpoint[1] - euler_vect[1];
        // For the yaw, we can directly control the rates:
        float yaw_rate_setpoint_manual = 0;
        if(abs(radio_control.values[RADIO_YAW]) >= 100){
            yaw_rate_setpoint_manual = AM7_SETTINGS_MAX_CMD_YAW_RATE * radio_control.values[RADIO_YAW] / MAX_PPRZ;
        }

        #if USE_SIXDOF_EXT_HEADING
            if(approach_state && get_am7_data_in()->sixdof_system_status == 3){
                yaw_rate_setpoint_manual = - get_am7_data_in()->sixdof_relative_psi * 0.01f * M_PI/180;
            }
        #endif

        euler_error[2] = yaw_rate_setpoint_manual;
        #if !FULLY_MANUAL_HEADING
            euler_error[2] += compute_yaw_rate_turn();
        #endif

        //Update the value to the structure to be sent to the AM7 module: 
        data_to_am7_module.psi_dot_cmd_rad_s = euler_error[2];

        //Apply euler angle gains: 
        float gain_to_speed_constant = 1 - airspeed * k_gain_airspeed; 
        Bound(gain_to_speed_constant, 0.1, 1);
        float phi_dot = euler_error[0]  * active_gains.p.phi * gain_to_speed_constant;
        float theta_dot = euler_error[1]  * active_gains.p.theta * gain_to_speed_constant;
        float psi_dot = euler_error[2];
        float phi_value = euler_vect[0];
        float theta_value = euler_vect[1];

        //Calculate the body error using the body to euler conversion: 
        rate_setpoint[0] = phi_dot - sin(theta_value) * psi_dot;
        rate_setpoint[1] = cos(phi_value) * theta_dot + sin(phi_value) * cos(theta_value) * psi_dot;
        rate_setpoint[2] = -sin(phi_value) * theta_dot + cos(phi_value) * cos(theta_value) * psi_dot;

        //Compute the angular acceleration setpoint using the filtered rates:
        acc_setpoint[3] = (rate_setpoint[0] - rate_vect_filt[0]) * active_gains.d.phi * gain_to_speed_constant;
        acc_setpoint[4] = (rate_setpoint[1] - rate_vect_filt[1]) * active_gains.d.theta * gain_to_speed_constant;
        acc_setpoint[5] = (rate_setpoint[2] - rate_vect_filt[2]) * active_gains.d.psi * gain_to_speed_constant;

        //Compute the pseudocontrol angular array:
        data_to_am7_module.pseudocontrol_p_dot = acc_setpoint[3] - rate_vect_dot_filt[0];
        data_to_am7_module.pseudocontrol_q_dot = acc_setpoint[4] - rate_vect_dot_filt[1];
        data_to_am7_module.pseudocontrol_r_dot = acc_setpoint[5] - rate_vect_dot_filt[2];

        //Linear acceleration control:
        pos_setpoint[0] = nav.target.y; 
        pos_setpoint[1] = nav.target.x; 
        pos_setpoint[2] = -nav.fp_altitude;   
        compute_speed_ref_from_waypoint(speed_setpoint_control_rf, pos_setpoint, pos_vect, airspeed, euler_vect[2]);
        #ifdef RECTIFY_LAT_AND_FWD_SPEED 
            //Make sure not to divide by zero: 
            if(speed_setpoint_control_rf[0] < 0.01 && speed_setpoint_control_rf[0] >= 0){
                speed_setpoint_control_rf[0] = 0.01;
            }
            if(speed_setpoint_control_rf[0] > -0.01 && speed_setpoint_control_rf[0] < 0){
                speed_setpoint_control_rf[0] = -0.01;
            }

            //Compute the angle between the desired speed array and the vehicle x-control axis:
            float alpha_speed = atan2f(speed_setpoint_control_rf[1],speed_setpoint_control_rf[0]);

            //Compute weight to move from one lateral speed reference to another: 
            float lat_speed_weight = compute_lat_speed_multiplier(min_speed_transition,ref_speed_transition,airspeed);

            //Estimate aoa (useful in the next two loops):
            float aoa_angle_estimation = euler_vect[1] - flight_path_angle;
            BoundAbs(aoa_angle_estimation, (M_PI/2 - 0.01));

            // Compute first term of lateral speed desired based on the alpha_speed value:
            float first_term_lateral_speed = (airspeed / cosf(aoa_angle_estimation)) * alpha_speed * lat_speed_weight * extra_lat_gain;
            float second_term_lateral_speed = speed_setpoint_control_rf[1] * (1-lat_speed_weight);
            //Conpute the lateral speed desired
            speed_setpoint_control_rf[1] = first_term_lateral_speed + second_term_lateral_speed;

            //Apply full fwd speed if requested:
            if(force_forward){
                speed_setpoint_control_rf[0] = WP_CONTROL_FWD_SPEED_FORCE_FWD_MODE;
            }
        #endif
        //Bound speeds based on the maximum airspeed or maximum ground speed:
        float max_Vx_airspeed = max_V_control_from_max_airspeed(airspeed, speed_vect_control_rf[0], max_airspeed_am7);
        Bound(speed_setpoint_control_rf[0],min_fwd_speed,Min(max_fwd_speed,max_Vx_airspeed));
        BoundAbs(speed_setpoint_control_rf[1],max_lat_speed);
        BoundAbs(speed_setpoint_control_rf[2],max_vert_speed);

        //Compute the speed error in the control rf:
        speed_error_vect_control_rf[0] = speed_setpoint_control_rf[0] - speed_vect_control_rf[0];
        speed_error_vect_control_rf[1] = speed_setpoint_control_rf[1] - speed_vect_control_rf[1] * (1 - compute_lat_speed_multiplier(min_speed_transition,ref_speed_transition,airspeed));
        speed_error_vect_control_rf[2] = speed_setpoint_control_rf[2] - speed_vect_control_rf[2];

        //Compute the acceleration setpoints in the control rf:
        acc_setpoint[0] = speed_error_vect_control_rf[0] * active_gains.d.x;
        acc_setpoint[1] = speed_error_vect_control_rf[1] * active_gains.d.y;
        acc_setpoint[2] = speed_error_vect_control_rf[2] * active_gains.d.z;

        //Apply saturation points for the accelerations in the control rf:
        Bound(acc_setpoint[0],min_fwd_acc,max_fwd_acc);
        BoundAbs(acc_setpoint[1],max_lat_acc);
        BoundAbs(acc_setpoint[2],max_vert_acc);

        //Compute the pseudocontrol acceleration array:
        data_to_am7_module.pseudocontrol_ax = acc_setpoint[0] - accel_vect_filt_control_rf[0];
        data_to_am7_module.pseudocontrol_ay = acc_setpoint[1] - accel_vect_filt_control_rf[1];
        data_to_am7_module.pseudocontrol_az = acc_setpoint[2] - accel_vect_filt_control_rf[2];

        //Produce commands based on what the am7 returned: 
        act_cmd_to_t4.cmd_timestamp = get_sys_time_float();
        act_cmd_to_t4.motor_control_mode = 2;
        act_cmd_to_t4.motor_1_cmd = (float) get_am7_data_in()->motor_1_cmd_int * 0.1f; 
        act_cmd_to_t4.motor_2_cmd = (float) get_am7_data_in()->motor_2_cmd_int * 0.1f;
        act_cmd_to_t4.motor_3_cmd = (float) get_am7_data_in()->motor_3_cmd_int * 0.1f;
        act_cmd_to_t4.motor_4_cmd = (float) get_am7_data_in()->motor_4_cmd_int * 0.1f;
        act_cmd_to_t4.servo_el_1_angle_deg = (float) get_am7_data_in()->el_1_cmd_int * 0.01f;
        act_cmd_to_t4.servo_el_2_angle_deg = (float) get_am7_data_in()->el_2_cmd_int * 0.01f;
        act_cmd_to_t4.servo_el_3_angle_deg = (float) get_am7_data_in()->el_3_cmd_int * 0.01f;
        act_cmd_to_t4.servo_el_4_angle_deg = (float) get_am7_data_in()->el_4_cmd_int * 0.01f;
        act_cmd_to_t4.servo_az_1_angle_deg = (float) get_am7_data_in()->az_1_cmd_int * 0.01f;
        act_cmd_to_t4.servo_az_2_angle_deg = (float) get_am7_data_in()->az_2_cmd_int * 0.01f;
        act_cmd_to_t4.servo_az_3_angle_deg = (float) get_am7_data_in()->az_3_cmd_int * 0.01f;
        act_cmd_to_t4.servo_az_4_angle_deg = (float) get_am7_data_in()->az_4_cmd_int * 0.01f;
        act_cmd_to_t4.flaperon_right_angle_deg = (float) get_am7_data_in()->ailerons_cmd_int * 0.01f;
        act_cmd_to_t4.flaperon_left_angle_deg = (float) get_am7_data_in()->ailerons_cmd_int * 0.01f;
        if(autopilot.motors_on){
            act_cmd_to_t4.motor_arm = 1;
            act_cmd_to_t4.servo_arm = 1;
        }
        else{
            act_cmd_to_t4.motor_arm = 0;
            act_cmd_to_t4.servo_arm = 0;
            //Kill motors and initialize a spiral mode with the flaperons:
            act_cmd_to_t4.flaperon_right_angle_deg = 15; 
            act_cmd_to_t4.flaperon_left_angle_deg = 15;
        }

        //Submit the data to the AM7 module:
        AbiSendMsgAM7_DATA_OUT(ABI_AM7_DATA_OUT_ID, &data_to_am7_module);
    }
    else{
        //Prepare the actuator commands to be kill by default: 
        act_cmd_to_t4.cmd_timestamp = get_sys_time_float();
        act_cmd_to_t4.motor_arm = 0;
        act_cmd_to_t4.servo_arm = 0;
        act_cmd_to_t4.flaperon_right_angle_deg = 15; 
        act_cmd_to_t4.flaperon_left_angle_deg = 15;
    }
    //Submit the actuator commands to the teensy module: 
    AbiSendMsgSERIAL_ACT_T4_CMD(ABI_SERIAL_ACT_T4_CMD_ID, &act_cmd_to_t4);
}
