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
#include "overactuated_mixing.h"
#include <math.h>
#include "modules/radio_control/radio_control.h"
#include "state.h"
#include "paparazzi.h"
#include "modules/datalink/telemetry.h"
#include "modules/nav/waypoints.h"
#include "generated/flight_plan.h"
#include "modules/sensors/ca_am7.h"
#include "modules/core/abi.h"
#include "mcu_periph/sys_time.h"
#include "modules/sensors/serial_act_t4.h"

/**
 * Variables declaration
 */

#define UPDATE_WP_WITH_ARUCO

// #define GOTO_ARUCO_AUTO

//AM7 variables:
struct am7_data_out am7_data_out_local;
float extra_data_out_local[255] __attribute__((aligned));
static abi_event AM7_in;
float extra_data_in_local[255] __attribute__((aligned));
struct am7_data_in myam7_data_in_local;

void assign_variables(void);

/**
 * ABI routine called by the serial_act_t4 ABI event
 */
static void data_AM7_abi_in(uint8_t sender_id __attribute__((unused)), struct am7_data_in * myam7_data_in_ptr, float * extra_data_in_ptr){
    memcpy(&myam7_data_in_local,myam7_data_in_ptr,sizeof(struct am7_data_in));
    memcpy(&extra_data_in_local,extra_data_in_ptr,255 * sizeof(float));


    #ifdef UPDATE_WP_WITH_ARUCO

        // Send to the GCS that the waypoint has been moved
        static uint8_t wp_id = WP_ARUCO_HOVER;
        RunOnceEvery(PERIODIC_FREQUENCY / 2, { //Update ARUCO_HOVER waypoint every 0.5 seconds
            DOWNLINK_SEND_WP_MOVED_ENU(DefaultChannel, DefaultDevice, &wp_id,
                                    &waypoints[WP_ARUCO_HOVER].enu_i.x,
                                    &waypoints[WP_ARUCO_HOVER].enu_i.y,
                                    &waypoints[WP_ARUCO_HOVER].enu_i.z);
        });
    #endif

    #ifdef GOTO_ARUCO_AUTO
        float alt_offset_beacon = 15.0f;
        struct EnuCoor_f target_pos_aruco = {myam7_data_in_local.aruco_NED_pos_y, myam7_data_in_local.aruco_NED_pos_x, -myam7_data_in_local.aruco_NED_pos_z + alt_offset_beacon};

        // Update the waypoint for the goto function:
        waypoint_set_enu(WP_ARUCO_HOVER, &target_pos_aruco);
    #endif
}

/**
 * Initialize the overactuated mixing module
 */
void overactuated_mixing_init(void) {
    //Init abi bind msg to Raspberry Pi:
    AbiBindMsgAM7_DATA_IN(ABI_BROADCAST, &AM7_in, data_AM7_abi_in);
}

/**
 * Ad each iteration upload global variables
 */
void assign_variables(void){
    // float rate_vect[3];
    // rate_vect[0] = stateGetBodyRates_f()->p;
    // rate_vect[1] = stateGetBodyRates_f()->q;
    // rate_vect[2] = stateGetBodyRates_f()->r;

    float euler_vect[3];
    euler_vect[0] = stateGetNedToBodyEulers_f()->phi;
    euler_vect[1] = stateGetNedToBodyEulers_f()->theta;
    euler_vect[2] = stateGetNedToBodyEulers_f()->psi;

    // float acc_vect[3];
    // acc_vect[0] = stateGetAccelNed_f()->x;
    // acc_vect[1] = stateGetAccelNed_f()->y;
    // acc_vect[2] = stateGetAccelNed_f()->z;

    // float speed_vect[3];
    // speed_vect[0] = stateGetSpeedNed_f()->x;
    // speed_vect[1] = stateGetSpeedNed_f()->y;
    // speed_vect[2] = stateGetSpeedNed_f()->z;

    float pos_vect[3];
    pos_vect[0] = stateGetPositionNed_f()->x;
    pos_vect[1] = stateGetPositionNed_f()->y;
    pos_vect[2] = stateGetPositionNed_f()->z;

    //Only variables needed by the aruco:
    am7_data_out_local.UAV_NED_pos_x = pos_vect[0];
    am7_data_out_local.UAV_NED_pos_y = pos_vect[1];
    am7_data_out_local.UAV_NED_pos_z = pos_vect[2];

    float fake_beta = 0;
    am7_data_out_local.beta_state_int = (int16_t) (fake_beta * 1e2);
    am7_data_out_local.theta_state_int = (int16_t) (euler_vect[0] * 1e2 * 180/M_PI);
    am7_data_out_local.phi_state_int = (int16_t) (euler_vect[1] * 1e2 * 180/M_PI);
    am7_data_out_local.psi_state_int = (int16_t) (euler_vect[2] * 1e2 * 180/M_PI);

}

/**
 * Run the overactuated mixing
 */
void overactuated_mixing_run(void)
{
    //Assign variables:
    assign_variables();

    //Send variables:
    AbiSendMsgAM7_DATA_OUT(ABI_AM7_DATA_OUT_ID, &am7_data_out_local, extra_data_out_local);

}

/**
 * Run the overactuated mixing
 */
void assign_and_send_cmds(void)
{   
  //Send values to FBW system: 
  struct serial_act_t4_out myserial_act_t4_out_local;
  float serial_act_t4_extra_data_out_local[255] __attribute__((aligned));
  for(int i = 0; i<254; i++){
    serial_act_t4_extra_data_out_local[i] = 0.0f;
  }
  if(autopilot_get_motors_on()) {
    //Arm motor:
    myserial_act_t4_out_local.motor_arm_int = 1;
  }
  else {
    //Disarm motor:
    myserial_act_t4_out_local.motor_arm_int = 0;
  }
  myserial_act_t4_out_local.servo_arm_int = 1;

  myserial_act_t4_out_local.motor_1_dshot_cmd_int = (int16_t) ((2*actuators_pprz[3])/9.6);
  myserial_act_t4_out_local.motor_2_dshot_cmd_int = (int16_t) ((2*actuators_pprz[2])/9.6);
//   Bound(myserial_act_t4_out_local.motor_1_dshot_cmd_int,0,2000);
//   Bound(myserial_act_t4_out_local.motor_2_dshot_cmd_int,0,2000);
  myserial_act_t4_out_local.motor_3_dshot_cmd_int = (int16_t) (0);
  myserial_act_t4_out_local.motor_4_dshot_cmd_int = (int16_t) (0);

  myserial_act_t4_out_local.servo_1_cmd_int = (int16_t) ((actuators_pprz[0])/9.6)*MAX_RANGE_SERVOS_DEG*100;
  myserial_act_t4_out_local.servo_4_cmd_int = (int16_t) ((actuators_pprz[1])/9.6)*MAX_RANGE_SERVOS_DEG*100;

  //SEND MESSAGE:
  AbiSendMsgSERIAL_ACT_T4_OUT(ABI_SERIAL_ACT_T4_OUT_ID, &myserial_act_t4_out_local, &serial_act_t4_extra_data_out_local[0]);


}