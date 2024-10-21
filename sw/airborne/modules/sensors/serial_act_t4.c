/*
 * Copyright (C) 2022 MAVLab
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
 * @file "modules/serial_act_t4.c"
 * @author Alessandro Mancinelli
 */

#include "modules/sensors/serial_act_t4.h"
#include "pprzlink/pprz_transport.h"
#include "mcu_periph/uart.h"
#include "mcu_periph/sys_time.h"
#include <time.h>
#include <sys/time.h>
#include "modules/core/abi.h"

//Sliders control: 
int test_rpm_control = 0;
float motor_1_rad_s_slider = 0, motor_2_rad_s_slider = 0, motor_3_rad_s_slider = 0, motor_4_rad_s_slider = 0;
int test_dshot_cmd = 0;
float motor_1_dshot_slider = 0, motor_2_dshot_slider = 0, motor_3_dshot_slider = 0, motor_4_dshot_slider = 0;
int test_servo_angles = 0;
float el_1_angle_deg_slider = 0, el_2_angle_deg_slider = 0, el_3_angle_deg_slider = 0, el_4_angle_deg_slider = 0;
float az_1_angle_deg_slider = 0, az_2_angle_deg_slider = 0, az_3_angle_deg_slider = 0, az_4_angle_deg_slider = 0;
float flaperon_right_angle_deg_slider = 0, flaperon_left_angle_deg_slider = 0;
float max_pwm_servo_9 = FBW_T4_SERVO_9_MAX_PWM, neutral_pwm_servo_9 = FBW_T4_SERVO_9_NEUTRAL_PWM ,min_pwm_servo_9 = FBW_T4_SERVO_9_MIN_PWM;
float max_pwm_servo_10 = FBW_T4_SERVO_10_MAX_PWM, neutral_pwm_servo_10 = FBW_T4_SERVO_10_NEUTRAL_PWM, min_pwm_servo_10 = FBW_T4_SERVO_10_MIN_PWM;

//Default command structure, in case of no command received [FAILSAFE]: 
struct ActCmd_t act_cmd_default = {
    .cmd_timestamp = 0,
    .motor_arm = 0,
    .servo_arm = 0,
    .motor_control_mode = 1,
    .motor_1_cmd = 0,
    .motor_2_cmd = 0,
    .motor_3_cmd = 0,
    .motor_4_cmd = 0
    .servo_el_1_angle_deg = 0,
    .servo_el_2_angle_deg = 0,
    .servo_el_3_angle_deg = 0,
    .servo_el_4_angle_deg = 0,
    .servo_az_1_angle_deg = 0,
    .servo_az_2_angle_deg = 0,
    .servo_az_3_angle_deg = 0,
    .servo_az_4_angle_deg = 0,
    .flaperon_right_angle_deg = 0,
    .flaperon_left_angle_deg = 0
};

//Variables for the INDI RPM controller: 
float dshot_cmd_indi[4], dshot_cmd_indi_filtered[4], dshot_cmd_indi_filtered_delayed[4][FBW_T4_MOTOR_DYN_DELAY_TS];
float dshot_cmd_indi_state_filtered[4], motors_rad_s_filtered[4];
float K_indi_rad_s_dshot = FBW_T4_K_INDI_RAD_S_DSHOT;

//Variables for the communication with the other modules, to receive commands and to send telemetry: 
struct ActCmd_t ActCmd, received_ActCmd; 
struct ActStates_t ActStates;

//Variables for outbound packet
static abi_event SERIAL_ACT_T4_CMD;
uint8_t serial_act_t4_out_msg_id;
struct serial_act_t4_out myserial_act_t4_out;


//Variables for inbound packet
struct serial_act_t4_in myserial_act_t4_in;
float serial_act_t4_extra_data_in[255]__attribute__((aligned));
uint16_t serial_act_t4_buf_in_cnt = 0;
uint32_t serial_act_t4_missed_packets_in = 0;
uint32_t serial_act_t4_received_packets = 0;
uint16_t serial_act_t4_message_frequency_in = 0;
float serial_act_t4_last_ts = 0;
static uint8_t serial_act_t4_msg_buf_in[sizeof(struct serial_act_t4_in)*2]__attribute__((aligned));   ///< The message buffer for the device chosen to be 2* message_size total


#if PERIODIC_TELEMETRY

    #include "modules/datalink/telemetry.h"

    static void serial_act_t4_downlink(struct transport_tx *trans, struct link_device *dev)
    {
        int16_t motor_1_rpm_int_telemetry = myserial_act_t4_in.motor_1_rpm_int;
        int16_t motor_2_rpm_int_telemetry = myserial_act_t4_in.motor_2_rpm_int;
        int16_t motor_3_rpm_int_telemetry = myserial_act_t4_in.motor_3_rpm_int;
        int16_t motor_4_rpm_int_telemetry = myserial_act_t4_in.motor_4_rpm_int;

        int16_t motor_1_error_code_int_telemetry = myserial_act_t4_in.motor_1_error_code_int;
        int16_t motor_2_error_code_int_telemetry = myserial_act_t4_in.motor_2_error_code_int;
        int16_t motor_3_error_code_int_telemetry = myserial_act_t4_in.motor_3_error_code_int;
        int16_t motor_4_error_code_int_telemetry = myserial_act_t4_in.motor_4_error_code_int;

        int16_t motor_1_current_int_telemetry = myserial_act_t4_in.motor_1_current_int;
        int16_t motor_2_current_int_telemetry = myserial_act_t4_in.motor_2_current_int;
        int16_t motor_3_current_int_telemetry = myserial_act_t4_in.motor_3_current_int;
        int16_t motor_4_current_int_telemetry = myserial_act_t4_in.motor_4_current_int;

        int16_t motor_1_voltage_int_telemetry = myserial_act_t4_in.motor_1_voltage_int;
        int16_t motor_2_voltage_int_telemetry = myserial_act_t4_in.motor_2_voltage_int;
        int16_t motor_3_voltage_int_telemetry = myserial_act_t4_in.motor_3_voltage_int;
        int16_t motor_4_voltage_int_telemetry = myserial_act_t4_in.motor_4_voltage_int;

        int16_t rotor_1_az_angle_int_telemetry = myserial_act_t4_in.servo_1_angle_int;
        int16_t rotor_1_el_angle_int_telemetry = myserial_act_t4_in.servo_2_angle_int;
        int16_t rotor_2_az_angle_int_telemetry = myserial_act_t4_in.servo_5_angle_int;
        int16_t rotor_2_el_angle_int_telemetry = myserial_act_t4_in.servo_6_angle_int;
        int16_t rotor_3_az_angle_int_telemetry = myserial_act_t4_in.servo_7_angle_int; 
        int16_t rotor_3_el_angle_int_telemetry = myserial_act_t4_in.servo_8_angle_int;
        int16_t rotor_4_az_angle_int_telemetry = myserial_act_t4_in.servo_3_angle_int;
        int16_t rotor_4_el_angle_int_telemetry = myserial_act_t4_in.servo_4_angle_int;
        int16_t servo_9_angle_int_telemetry = myserial_act_t4_in.servo_9_angle_int;
        int16_t servo_10_angle_int_telemetry = myserial_act_t4_in.servo_10_angle_int;

        int16_t rotor_1_az_angle_update_time_us_telemetry = myserial_act_t4_in.servo_1_update_time_us;
        int16_t rotor_1_el_angle_update_time_us_telemetry = myserial_act_t4_in.servo_2_update_time_us;
        int16_t rotor_2_az_angle_update_time_us_telemetry = myserial_act_t4_in.servo_5_update_time_us;
        int16_t rotor_2_el_angle_update_time_us_telemetry = myserial_act_t4_in.servo_6_update_time_us;
        int16_t rotor_3_az_angle_update_time_us_telemetry = myserial_act_t4_in.servo_7_update_time_us;
        int16_t rotor_3_el_angle_update_time_us_telemetry = myserial_act_t4_in.servo_8_update_time_us;
        int16_t rotor_4_az_angle_update_time_us_telemetry = myserial_act_t4_in.servo_3_update_time_us;
        int16_t rotor_4_el_angle_update_time_us_telemetry = myserial_act_t4_in.servo_4_update_time_us;
        int16_t servo_9_update_time_us_telemetry = myserial_act_t4_in.servo_9_update_time_us;
        int16_t servo_10_update_time_us_telemetry = myserial_act_t4_in.servo_10_update_time_us;
        float rolling_msg_in_telemetry = myserial_act_t4_in.rolling_msg_in;
        uint8_t rolling_msg_in_id_telemetry = myserial_act_t4_in.rolling_msg_in_id; 


        pprz_msg_send_SERIAL_ACT_T4_IN(trans, dev, AC_ID, 
                &motor_1_rpm_int_telemetry, &motor_2_rpm_int_telemetry, &motor_3_rpm_int_telemetry, &motor_4_rpm_int_telemetry,
                &rotor_1_az_angle_int_telemetry, &rotor_1_el_angle_int_telemetry, &rotor_2_az_angle_int_telemetry, &rotor_2_el_angle_int_telemetry,
                &rotor_3_az_angle_int_telemetry, &rotor_3_el_angle_int_telemetry, &rotor_4_az_angle_int_telemetry, &rotor_4_el_angle_int_telemetry,
                &servo_9_angle_int_telemetry, &servo_10_angle_int_telemetry, 
                &serial_act_t4_missed_packets_in, &serial_act_t4_message_frequency_in,
                &rolling_msg_in_telemetry, &rolling_msg_in_id_telemetry,
                &motor_1_error_code_int_telemetry, &motor_2_error_code_int_telemetry, &motor_3_error_code_int_telemetry, &motor_4_error_code_int_telemetry,
                &rotor_1_az_angle_update_time_us_telemetry, &rotor_1_el_angle_update_time_us_telemetry, &rotor_2_az_angle_update_time_us_telemetry, &rotor_2_el_angle_update_time_us_telemetry,
                &rotor_3_az_angle_update_time_us_telemetry, &rotor_3_el_angle_update_time_us_telemetry, &rotor_4_az_angle_update_time_us_telemetry, &rotor_4_el_angle_update_time_us_telemetry,
                &servo_9_update_time_us_telemetry, &servo_10_update_time_us_telemetry,
                &motor_1_current_int_telemetry, &motor_2_current_int_telemetry, &motor_3_current_int_telemetry, &motor_4_current_int_telemetry,
                &motor_1_voltage_int_telemetry, &motor_2_voltage_int_telemetry, &motor_3_voltage_int_telemetry, &motor_4_voltage_int_telemetry);
    }

    static void serial_act_t4_uplink(struct transport_tx *trans, struct link_device *dev)
    {
    
    int8_t motor_arm_int_telemetry = myserial_act_t4_out.motor_arm_int;
    int8_t servo_arm_int_telemetry = myserial_act_t4_out.servo_arm_int;
    int16_t motor_1_dshot_cmd_int_telemetry = myserial_act_t4_out.motor_1_dshot_cmd_int; 
    int16_t motor_2_dshot_cmd_int_telemetry = myserial_act_t4_out.motor_2_dshot_cmd_int; 
    int16_t motor_3_dshot_cmd_int_telemetry = myserial_act_t4_out.motor_3_dshot_cmd_int; 
    int16_t motor_4_dshot_cmd_int_telemetry = myserial_act_t4_out.motor_4_dshot_cmd_int; 

    int16_t rotor_1_az_angle_cmd_int_telemetry = myserial_act_t4_out.servo_1_cmd_int; 
    int16_t rotor_1_el_angle_cmd_int_telemetry = myserial_act_t4_out.servo_2_cmd_int;  
    int16_t rotor_2_az_angle_cmd_int_telemetry = myserial_act_t4_out.servo_5_cmd_int; 
    int16_t rotor_2_el_angle_cmd_int_telemetry = myserial_act_t4_out.servo_6_cmd_int; 
    int16_t rotor_3_az_angle_cmd_int_telemetry = myserial_act_t4_out.servo_7_cmd_int; 
    int16_t rotor_3_el_angle_cmd_int_telemetry = myserial_act_t4_out.servo_8_cmd_int; 
    int16_t rotor_4_az_angle_cmd_int_telemetry = myserial_act_t4_out.servo_3_cmd_int; 
    int16_t rotor_4_el_angle_cmd_int_telemetry = myserial_act_t4_out.servo_4_cmd_int;  
    int16_t servo_9_cmd_int_telemetry = myserial_act_t4_out.servo_9_cmd_int; 
    int16_t servo_10_cmd_int_telemetry = myserial_act_t4_out.servo_10_cmd_int;

    float rolling_msg_out_telemetry = myserial_act_t4_out.rolling_msg_out;
    uint8_t rolling_msg_out_id_telemetry = myserial_act_t4_out.rolling_msg_out_id;

	   pprz_msg_send_SERIAL_ACT_T4_OUT(trans, dev, AC_ID, 
                    &motor_arm_int_telemetry, &servo_arm_int_telemetry,
                    &motor_1_dshot_cmd_int_telemetry, &motor_2_dshot_cmd_int_telemetry, &motor_3_dshot_cmd_int_telemetry, &motor_4_dshot_cmd_int_telemetry, 
                    &rotor_1_az_angle_cmd_int_telemetry, &rotor_1_el_angle_cmd_int_telemetry, &rotor_2_az_angle_cmd_int_telemetry, &rotor_2_el_angle_cmd_int_telemetry, 
                    &rotor_3_az_angle_cmd_int_telemetry, &rotor_3_el_angle_cmd_int_telemetry, &rotor_4_az_angle_cmd_int_telemetry, &rotor_4_el_angle_cmd_int_telemetry, 
                    &servo_9_cmd_int_telemetry, &servo_10_cmd_int_telemetry, 
                    &rolling_msg_out_telemetry, &rolling_msg_out_id_telemetry);

    }

#endif

//Generate the motor command if the indi rpm control law is needed:
void indi_motor_controller(float *motors_dshot_cmd_indi, float *motors_des_rot_rad_s, float *motors_current_rot_rad_s){
    //Calculate the dshot command for each motor:
    for (int i = 0; i < 4; i++){
        
        //Filter the motor rotatioal speed and the old dshot command with the same filter: 
        motors_rad_s_filtered[i] = motors_rad_s_filtered[i] + FBW_T4_FILT_FIRST_ORDER_RPM_COEFF * (motors_current_rot_rad_s[i] - motors_rad_s_filtered[i]);

        //Apply the same filter on the dshot command:
        dshot_cmd_indi_filtered[i] = dshot_cmd_indi_filtered[i] + FBW_T4_FILT_FIRST_ORDER_RPM_COEFF * (dshot_cmd_indi[i] - dshot_cmd_indi_filtered[i]);

        //Shift the delayed cmd array:
        for (int j = 0; j < FBW_T4_MOTOR_DYN_DELAY_TS - 1; j++){
            dshot_cmd_indi_filtered_delayed[i][j] = dshot_cmd_indi_filtered_delayed[i][j+1];
        }
        //Assign current value to the delayed array of state in the last position: 
        dshot_cmd_indi_filtered_delayed[i][FBW_T4_MOTOR_DYN_DELAY_TS-1] = dshot_cmd_indi_filtered[i];

        //Estimate the cmd evolution based on the provided motor dynamics:
        dshot_cmd_indi_state_filtered[i] = dshot_cmd_indi_state_filtered[i] + FBW_T4_MOTOR_DYN_COEFF * (dshot_cmd_indi_filtered_delayed[i][0] - dshot_cmd_indi_state_filtered[i]);

        //Compute the incremental dshot cmd for the motors:
        dshot_cmd_indi[i] = ( dshot_cmd_indi_state_filtered[i] + K_indi_rad_s_dshot * (motors_des_rot_rad_s[i] - motors_rad_s_filtered[i]));

        //Bound the dshot cmd to the max and min DSHOT value 
        Bound(dshot_cmd_indi[i], 0, FBW_T4_MAX_DSHOT_CMD);
        
        //Assign computed cmd to the output array: 
        for (int i = 0; i < 4; i++){
            motors_dshot_cmd_indi[i] = dshot_cmd_indi[i];
        }
    }
}

//Callback function to receive the command from the other modules:
static void data_serial_act_t4_cmd(uint8_t sender_id __attribute__((unused)), struct ActCmd_t * ActCmd_ptr){
    //Copying the struct to a local struct:
    memcpy(&received_ActCmd,ActCmd_ptr,sizeof(struct ActCmd_t));
}

//Send the data over serial to the Teensy 4.0:
void serial_act_t4_data_send()
{
    //Increase the counter to track the sending messages:
    myserial_act_t4_out.rolling_msg_out = serial_act_t4_extra_data_out[serial_act_t4_out_msg_id];
    myserial_act_t4_out.rolling_msg_out_id = serial_act_t4_out_msg_id;
    serial_act_t4_out_msg_id++;
    if(serial_act_t4_out_msg_id == 255){
        serial_act_t4_out_msg_id = 0;
    }

    //Send the message over serial to the Teensy 4.0:
    uint8_t *buf_send = (uint8_t *)&myserial_act_t4_out;
    //Calculating the checksum
    uint8_t checksum_out_local = 0;
    for(uint16_t i = 0; i < sizeof(struct serial_act_t4_out) - 1; i++){
        checksum_out_local += buf_send[i];
    }
    myserial_act_t4_out.checksum_out = checksum_out_local;

    //Send bytes
    uart_put_byte(&(SERIAL_ACT_T4_PORT), 0, START_BYTE_SERIAL_ACT_T4);
    for(uint8_t i = 0; i < sizeof(struct serial_act_t4_out) ; i++){
        uart_put_byte(&(SERIAL_ACT_T4_PORT), 0, buf_send[i]);
    }
}

//Init the serial communication, rpm control variables and the ABI bind message:
void serial_act_t4_init() 
{
    serial_act_t4_buf_in_cnt = 0;
    serial_act_t4_out_msg_id = 0;

    //Init abi bind msg:
    AbiBindMsgSERIAL_ACT_T4_CMD(ABI_BROADCAST, &SERIAL_ACT_T4_CMD, data_serial_act_t4_cmd);

    //Init filters and cmd elements: 
    for(int i = 0; i < 4; i++){
        motors_rad_s_filtered[i] = 0;
        dshot_cmd_indi[i] = 0;
        dshot_cmd_indi_filtered[i] = 0;
        dshot_cmd_indi_state_filtered[i] = 0;
        for(int j = 0; j < FBW_T4_MOTOR_DYN_DELAY_TS; j++){
            dshot_cmd_indi_filtered_delayed[i][j] = 0;
        }

    }

    #if PERIODIC_TELEMETRY
    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_SERIAL_ACT_T4_IN, serial_act_t4_downlink);
    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_SERIAL_ACT_T4_OUT, serial_act_t4_uplink);
    #endif
}

//Parse message and fill up the ActStates struct so other modules can make use of it: 
void serial_act_t4_parse_msg_in()
{
    memcpy(&myserial_act_t4_in, &serial_act_t4_msg_buf_in[1], sizeof(struct serial_act_t4_in)); //Starting from 1 to avoid reading the starting byte
    //Assign the rolling message:
    serial_act_t4_extra_data_in[myserial_act_t4_in.rolling_msg_in_id] = myserial_act_t4_in.rolling_msg_in;

    //Fill up the ActStates struct so other modules can make use of it: 
    ActStates.timestamp = get_sys_time_float();
    ActStates.motor_1_rad_s = (float) 2.0f*M_PI*myserial_act_t4_in.motor_1_rpm_int/60.0f;
    ActStates.motor_2_rad_s = (float) 2.0f*M_PI*myserial_act_t4_in.motor_2_rpm_int/60.0f;
    ActStates.motor_3_rad_s = (float) 2.0f*M_PI*myserial_act_t4_in.motor_3_rpm_int/60.0f;
    ActStates.motor_4_rad_s = (float) 2.0f*M_PI*myserial_act_t4_in.motor_4_rpm_int/60.0f;
    ActStates.el_1_angle_deg = (float) (myserial_act_t4_in.servo_2_angle_int/100.0f)/FBW_T4_K_RATIO_GEAR_EL;
    ActStates.el_2_angle_deg = (float) (myserial_act_t4_in.servo_6_angle_int/100.0f)/FBW_T4_K_RATIO_GEAR_EL;
    ActStates.el_3_angle_deg = (float) (myserial_act_t4_in.servo_8_angle_int/100.0f)/FBW_T4_K_RATIO_GEAR_EL;
    ActStates.el_4_angle_deg = (float) (myserial_act_t4_in.servo_4_angle_int/100.0f)/FBW_T4_K_RATIO_GEAR_EL;
    ActStates.az_1_angle_deg = (float) (myserial_act_t4_in.servo_1_angle_int/100.0f)/FBW_T4_K_RATIO_GEAR_AZ;
    ActStates.az_2_angle_deg = (float) (myserial_act_t4_in.servo_5_angle_int/100.0f)/FBW_T4_K_RATIO_GEAR_AZ;
    ActStates.az_3_angle_deg = (float) -(myserial_act_t4_in.servo_7_angle_int/100.0f)/FBW_T4_K_RATIO_GEAR_AZ;
    ActStates.az_4_angle_deg = (float) -(myserial_act_t4_in.servo_3_angle_int/100.0f)/FBW_T4_K_RATIO_GEAR_AZ;
    ActStates.flaperon_right_angle_deg = (float) -myserial_act_t4_in.servo_9_angle_int/100.0f;
    ActStates.flaperon_left_angle_deg = (float) myserial_act_t4_in.servo_10_angle_int/100.0f;
    ActStates.el_1_angle_deg_corrected = ActStates.el_1_angle_deg + FBW_T4_EL_1_ZERO_VALUE * 180/M_PI;
    ActStates.el_2_angle_deg_corrected = ActStates.el_2_angle_deg + FBW_T4_EL_2_ZERO_VALUE * 180/M_PI;
    ActStates.el_3_angle_deg_corrected = ActStates.el_3_angle_deg + FBW_T4_EL_3_ZERO_VALUE * 180/M_PI;
    ActStates.el_4_angle_deg_corrected = ActStates.el_4_angle_deg + FBW_T4_EL_4_ZERO_VALUE * 180/M_PI;
    ActStates.az_1_angle_deg_corrected = ActStates.az_1_angle_deg + FBW_T4_AZ_1_ZERO_VALUE * 180/M_PI;
    ActStates.az_2_angle_deg_corrected = ActStates.az_2_angle_deg + FBW_T4_AZ_2_ZERO_VALUE * 180/M_PI;
    ActStates.az_3_angle_deg_corrected = ActStates.az_3_angle_deg + FBW_T4_AZ_3_ZERO_VALUE * 180/M_PI;
    ActStates.az_4_angle_deg_corrected = ActStates.az_4_angle_deg + FBW_T4_AZ_4_ZERO_VALUE * 180/M_PI;
    ActStates.flaperon_right_angle_deg_corrected = ActStates.flaperon_right_angle_deg;
    ActStates.flaperon_left_angle_deg_corrected = ActStates.flaperon_left_angle_deg;
}

// Event checking if serial packet are available on the bus
void serial_act_t4_event()
{
    if(fabs(get_sys_time_float() - serial_act_t4_last_ts) > 5){ //Reset received packets to zero every 5 second to update the statistics
        serial_act_t4_received_packets = 0;
        serial_act_t4_last_ts = get_sys_time_float();
    }
    while(uart_char_available(&(SERIAL_ACT_T4_PORT)) > 0) {
        uint8_t serial_act_t4_byte_in;
        serial_act_t4_byte_in = uart_getch(&(SERIAL_ACT_T4_PORT));
        if ((serial_act_t4_byte_in == START_BYTE_SERIAL_ACT_T4) || (serial_act_t4_buf_in_cnt > 0)) {
            serial_act_t4_msg_buf_in[serial_act_t4_buf_in_cnt] = serial_act_t4_byte_in;
            serial_act_t4_buf_in_cnt++;
        }
        if (serial_act_t4_buf_in_cnt > sizeof(struct serial_act_t4_in) ) {
            serial_act_t4_buf_in_cnt = 0;
            uint8_t checksum_in_local = 0;
            for(uint16_t i = 1; i < sizeof(struct serial_act_t4_in) ; i++){
                checksum_in_local += serial_act_t4_msg_buf_in[i];
            }
            if(checksum_in_local == serial_act_t4_msg_buf_in[sizeof(struct serial_act_t4_in)]){
                serial_act_t4_parse_msg_in();
                serial_act_t4_received_packets++;
            }
            else {
                serial_act_t4_missed_packets_in++;
            }
        }
    }
    serial_act_t4_message_frequency_in = (uint16_t) serial_act_t4_received_packets/(get_sys_time_float() - serial_act_t4_last_ts);
}

// Function to implement RPM control and to prepare the structure to send
void serial_act_t4_control()
{
    //If the command is older than 0.5 seconds, then use the default command:
    if(get_sys_time_float() - received_ActCmd.cmd_timestamp > 0.5f){
        memcpy(&ActCmd, &act_cmd_default, sizeof(struct ActCmd_t));
    }
    else{
        memcpy(&ActCmd, &received_ActCmd, sizeof(struct ActCmd_t));
    }

    //////////////////////////////////////////////////////////////////////////////////////////////MOTORS COMMAND GENERATION
    //TESTING VARIABLES: 
    if(test_rpm_control){
        ActCmd.motor_control_mode = 2; 
        ActCmd.motor_1_cmd = motor_1_rad_s_slider;
        ActCmd.motor_2_cmd = motor_2_rad_s_slider;
        ActCmd.motor_3_cmd = motor_3_rad_s_slider;
        ActCmd.motor_4_cmd = motor_4_rad_s_slider;
    }
    if(test_dshot_cmd){
        ActCmd.motor_control_mode = 1; 
        ActCmd.motor_1_cmd = motor_1_dshot_slider;
        ActCmd.motor_2_cmd = motor_2_dshot_slider;
        ActCmd.motor_3_cmd = motor_3_dshot_slider;
        ActCmd.motor_4_cmd = motor_4_dshot_slider;
    }

    //If motors are armed and the control mode is 2, 
    //produce the dshot command out of the desired RPM: 
    if(ActCmd.motor_arm && ActCmd.motor_control_mode == 2){
        //Arm motors: 
        myserial_act_t4_out.motor_arm_int = 1;
        //Calculate the dshot command for each motor:
        float motors_dshot_cmd_indi[4]__attribute__((aligned)) = {0,0,0,0};
        float motors_des_rot_rad_s[4]__attribute__((aligned)) = {ActCmd.motor_1_cmd, ActCmd.motor_2_cmd, 
                                                                 ActCmd.motor_3_cmd, ActCmd.motor_4_cmd};
        float motors_current_rot_rad_s[4]__attribute__((aligned)) = {ActStates.motor_1_rad_s, ActStates.motor_2_rad_s,
                                                                     ActStates.motor_3_rad_s, ActStates.motor_4_rad_s};                                         
        //Bound rotational speed of motors, before the indi loop: 
        Bound(motors_des_rot_rad_s[0], 0, FBW_T4_MAX_OMEGA_RAD_S);
        Bound(motors_des_rot_rad_s[1], 0, FBW_T4_MAX_OMEGA_RAD_S);
        Bound(motors_des_rot_rad_s[2], 0, FBW_T4_MAX_OMEGA_RAD_S);
        Bound(motors_des_rot_rad_s[3], 0, FBW_T4_MAX_OMEGA_RAD_S);
        //Apply the indi rotational speed control law:
        indi_motor_controller(&motors_dshot_cmd_indi[0], &motors_des_rot_rad_s[0], &motors_current_rot_rad_s[0]);
        myserial_act_t4_out.motor_1_dshot_cmd_int = (int16_t) (motors_dshot_cmd_indi[0]);
        myserial_act_t4_out.motor_2_dshot_cmd_int = (int16_t) (motors_dshot_cmd_indi[1]);
        myserial_act_t4_out.motor_3_dshot_cmd_int = (int16_t) (motors_dshot_cmd_indi[2]);
        myserial_act_t4_out.motor_4_dshot_cmd_int = (int16_t) (motors_dshot_cmd_indi[3]);
    }
    else if(ActCmd.motor_arm && ActCmd.motor_control_mode == 1){
        //Arm motors: 
        myserial_act_t4_out.motor_arm_int = 1;
        //If the motors are not armed, then set the dshot command to zero:
        myserial_act_t4_out.motor_1_dshot_cmd_int = (int16_t) ActCmd.motor_1_cmd;
        myserial_act_t4_out.motor_2_dshot_cmd_int = (int16_t) ActCmd.motor_2_cmd;
        myserial_act_t4_out.motor_3_dshot_cmd_int = (int16_t) ActCmd.motor_3_cmd;
        myserial_act_t4_out.motor_4_dshot_cmd_int = (int16_t) ActCmd.motor_4_cmd;
    }
    else{
        //Disarm motors:
        myserial_act_t4_out.motor_arm_int = 0;
        //If the motors are not armed, then set the dshot command to zero:
        myserial_act_t4_out.motor_1_dshot_cmd_int = (int16_t) 0;
        myserial_act_t4_out.motor_2_dshot_cmd_int = (int16_t) 0;
        myserial_act_t4_out.motor_3_dshot_cmd_int = (int16_t) 0;
        myserial_act_t4_out.motor_4_dshot_cmd_int = (int16_t) 0;
    }

    //Bound motor command to max dshot: 
    Bound(myserial_act_t4_out.motor_1_dshot_cmd_int, 0, FBW_T4_MAX_DSHOT_CMD);
    Bound(myserial_act_t4_out.motor_2_dshot_cmd_int, 0, FBW_T4_MAX_DSHOT_CMD);
    Bound(myserial_act_t4_out.motor_3_dshot_cmd_int, 0, FBW_T4_MAX_DSHOT_CMD);
    Bound(myserial_act_t4_out.motor_4_dshot_cmd_int, 0, FBW_T4_MAX_DSHOT_CMD);

    /////////////////////////////////////////////////////////////////////////////////////////////SERVOS COMMAND GENERATION 
    //TESTING VARIABLES: 
    if(test_servo_angles){
        ActCmd.servo_arm = 1; 
        ActCmd.servo_el_1_angle_deg = el_1_angle_deg_slider;
        ActCmd.servo_el_2_angle_deg = el_2_angle_deg_slider;
        ActCmd.servo_el_3_angle_deg = el_3_angle_deg_slider;
        ActCmd.servo_el_4_angle_deg = el_4_angle_deg_slider;
        ActCmd.servo_az_1_angle_deg = az_1_angle_deg_slider;
        ActCmd.servo_az_2_angle_deg = az_2_angle_deg_slider;
        ActCmd.servo_az_3_angle_deg = az_3_angle_deg_slider;
        ActCmd.servo_az_4_angle_deg = az_4_angle_deg_slider;
        ActCmd.flaperon_right_angle_deg = flaperon_right_angle_deg_slider;
        ActCmd.flaperon_left_angle_deg = flaperon_left_angle_deg_slider;
    }

    //First, bound the commands to the max and min values, excluding the zeros: 
    Bound(ActCmd.servo_el_1_angle_deg, FBW_T4_SERVO_EL_MIN_ANGLE_DEG, FBW_T4_SERVO_EL_MAX_ANGLE_DEG);
    Bound(ActCmd.servo_el_2_angle_deg, FBW_T4_SERVO_EL_MIN_ANGLE_DEG, FBW_T4_SERVO_EL_MAX_ANGLE_DEG);
    Bound(ActCmd.servo_el_3_angle_deg, FBW_T4_SERVO_EL_MIN_ANGLE_DEG, FBW_T4_SERVO_EL_MAX_ANGLE_DEG);
    Bound(ActCmd.servo_el_4_angle_deg, FBW_T4_SERVO_EL_MIN_ANGLE_DEG, FBW_T4_SERVO_EL_MAX_ANGLE_DEG);
    Bound(ActCmd.servo_az_1_angle_deg, FBW_T4_SERVO_AZ_MIN_ANGLE_DEG, FBW_T4_SERVO_AZ_MAX_ANGLE_DEG);
    Bound(ActCmd.servo_az_2_angle_deg, FBW_T4_SERVO_AZ_MIN_ANGLE_DEG, FBW_T4_SERVO_AZ_MAX_ANGLE_DEG);
    Bound(ActCmd.servo_az_3_angle_deg, FBW_T4_SERVO_AZ_MIN_ANGLE_DEG, FBW_T4_SERVO_AZ_MAX_ANGLE_DEG);
    Bound(ActCmd.servo_az_4_angle_deg, FBW_T4_SERVO_AZ_MIN_ANGLE_DEG, FBW_T4_SERVO_AZ_MAX_ANGLE_DEG);
    Bound(ActCmd.flaperon_right_angle_deg, FBW_T4_SERVO_FLAPERON_RIGHT_MIN_ANGLE_DEG, FBW_T4_SERVO_FLAPERON_RIGHT_MAX_ANGLE_DEG);
    Bound(ActCmd.flaperon_left_angle_deg, FBW_T4_SERVO_FLAPERON_LEFT_MIN_ANGLE_DEG, FBW_T4_SERVO_FLAPERON_LEFT_MAX_ANGLE_DEG);

    //Copy the servo arm command to the output struct:
    myserial_act_t4_out.servo_arm_int = ActCmd.servo_arm;

    //Assign angles to servos: 
    myserial_act_t4_out.servo_2_cmd_int = (int16_t) (ActCmd.servo_el_1_angle_deg * FBW_T4_K_RATIO_GEAR_EL - FBW_T4_EL_1_ZERO_VALUE * 180/M_PI) * 100;
    myserial_act_t4_out.servo_6_cmd_int = (int16_t) (ActCmd.servo_el_2_angle_deg * FBW_T4_K_RATIO_GEAR_EL - FBW_T4_EL_2_ZERO_VALUE * 180/M_PI) * 100;
    myserial_act_t4_out.servo_8_cmd_int = (int16_t) (ActCmd.servo_el_3_angle_deg * FBW_T4_K_RATIO_GEAR_EL - FBW_T4_EL_3_ZERO_VALUE * 180/M_PI) * 100;
    myserial_act_t4_out.servo_4_cmd_int = (int16_t) (ActCmd.servo_el_4_angle_deg * FBW_T4_K_RATIO_GEAR_EL - FBW_T4_EL_4_ZERO_VALUE * 180/M_PI) * 100;

    myserial_act_t4_out.servo_1_cmd_int = (int16_t) (ActCmd.servo_az_1_angle_deg * FBW_T4_K_RATIO_GEAR_AZ - FBW_T4_AZ_1_ZERO_VALUE * 180/M_PI) * 100;
    myserial_act_t4_out.servo_5_cmd_int = (int16_t) (ActCmd.servo_az_2_angle_deg * FBW_T4_K_RATIO_GEAR_AZ - FBW_T4_AZ_2_ZERO_VALUE * 180/M_PI) * 100;
    myserial_act_t4_out.servo_7_cmd_int = (int16_t) (ActCmd.servo_az_3_angle_deg * FBW_T4_K_RATIO_GEAR_AZ - FBW_T4_AZ_3_ZERO_VALUE * 180/M_PI) * 100;
    myserial_act_t4_out.servo_3_cmd_int = (int16_t) (ActCmd.servo_az_4_angle_deg * FBW_T4_K_RATIO_GEAR_AZ - FBW_T4_AZ_4_ZERO_VALUE * 180/M_PI) * 100;

    //For the PWM angles, the zeros are handled in the Teensy 4.0 code, so the angles are directly assigned:
    myserial_act_t4_out.servo_9_cmd_int = (int16_t) (ActCmd.flaperon_right_angle_deg) * 100;
    myserial_act_t4_out.servo_10_cmd_int = (int16_t) (ActCmd.flaperon_left_angle_deg) * 100;

    ///////////////////////////////////////////////////////////////////////////////////////////////ASSIGN EXTRA DATA OUT
    serial_act_t4_extra_data_out[0] = FBW_T4_AILERONS_FIRST_ORD_DEN;
    serial_act_t4_extra_data_out[1] = FBW_T4_AILERONS_FIRST_ORD_NUM;
    
    serial_act_t4_extra_data_out[2] = max_pwm_servo_9;
    serial_act_t4_extra_data_out[3] = min_pwm_servo_9;
    serial_act_t4_extra_data_out[4] = neutral_pwm_servo_9;
    serial_act_t4_extra_data_out[5] = FBW_T4_SERVO_9_MIN_ANGLE_DEG;
    serial_act_t4_extra_data_out[6] = FBW_T4_SERVO_9_MAX_ANGLE_DEG;
    serial_act_t4_extra_data_out[7] = FBW_T4_SERVO_9_DELAY_TS;

    serial_act_t4_extra_data_out[8] = FBW_T4_AILERONS_FIRST_ORD_DEN;
    serial_act_t4_extra_data_out[9] = FBW_T4_AILERONS_FIRST_ORD_NUM;  
    
    serial_act_t4_extra_data_out[10] = max_pwm_servo_10;
    serial_act_t4_extra_data_out[11] = min_pwm_servo_10;
    serial_act_t4_extra_data_out[12] = neutral_pwm_servo_10;
    serial_act_t4_extra_data_out[13] = FBW_T4_SERVO_10_MIN_ANGLE_DEG;
    serial_act_t4_extra_data_out[14] = FBW_T4_SERVO_10_MAX_ANGLE_DEG;
    serial_act_t4_extra_data_out[15] = FBW_T4_SERVO_10_DELAY_TS;

    //Submit the message to the uart:
    serial_act_t4_data_send();
}