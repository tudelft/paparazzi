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

//Variables for outbound packet
static abi_event SERIAL_ACT_T4_OUT;
uint8_t serial_act_t4_out_msg_id;
struct serial_act_t4_out myserial_act_t4_out;
float serial_act_t4_extra_data_out[255]__attribute__((aligned));

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

    static void serial_act_t4_downlink(struct transport_tx *trans, struct link_device *dev){

        int16_t servo_1_angle_int_telemetry = myserial_act_t4_in.servo_1_angle_int;
        int16_t servo_2_angle_int_telemetry = myserial_act_t4_in.servo_2_angle_int;
        int16_t servo_3_angle_int_telemetry = myserial_act_t4_in.servo_3_angle_int;
        int16_t servo_4_angle_int_telemetry = myserial_act_t4_in.servo_4_angle_int;
        int16_t servo_5_angle_int_telemetry = myserial_act_t4_in.servo_5_angle_int; 
        int16_t servo_6_angle_int_telemetry = myserial_act_t4_in.servo_6_angle_int;

        int16_t servo_1_update_time_us_telemetry = myserial_act_t4_in.servo_1_feedback_update_time_us;
        int16_t servo_2_update_time_us_telemetry = myserial_act_t4_in.servo_2_feedback_update_time_us;
        int16_t servo_3_update_time_us_telemetry = myserial_act_t4_in.servo_3_feedback_update_time_us;
        int16_t servo_4_update_time_us_telemetry = myserial_act_t4_in.servo_4_feedback_update_time_us;
        int16_t servo_5_update_time_us_telemetry = myserial_act_t4_in.servo_5_feedback_update_time_us;
        int16_t servo_6_update_time_us_telemetry = myserial_act_t4_in.servo_6_feedback_update_time_us;

        int16_t servo_1_load_telemetry = myserial_act_t4_in.servo_1_load_int;
        int16_t servo_2_load_telemetry = myserial_act_t4_in.servo_2_load_int;
        int16_t servo_3_load_telemetry = myserial_act_t4_in.servo_3_load_int;
        int16_t servo_4_load_telemetry = myserial_act_t4_in.servo_4_load_int;
        int16_t servo_5_load_telemetry = myserial_act_t4_in.servo_5_load_int;
        int16_t servo_6_load_telemetry = myserial_act_t4_in.servo_6_load_int;

        int16_t servo_1_speed_telemetry = myserial_act_t4_in.servo_1_speed_int;
        int16_t servo_2_speed_telemetry = myserial_act_t4_in.servo_2_speed_int;
        int16_t servo_3_speed_telemetry = myserial_act_t4_in.servo_3_speed_int;     
        int16_t servo_4_speed_telemetry = myserial_act_t4_in.servo_4_speed_int;
        int16_t servo_5_speed_telemetry = myserial_act_t4_in.servo_5_speed_int;
        int16_t servo_6_speed_telemetry = myserial_act_t4_in.servo_6_speed_int;

        int16_t servo_1_volt_telemetry = myserial_act_t4_in.servo_1_volt_int;
        int16_t servo_2_volt_telemetry = myserial_act_t4_in.servo_2_volt_int;
        int16_t servo_3_volt_telemetry = myserial_act_t4_in.servo_3_volt_int;        
        int16_t servo_4_volt_telemetry = myserial_act_t4_in.servo_4_volt_int;
        int16_t servo_5_volt_telemetry = myserial_act_t4_in.servo_5_volt_int;
        int16_t servo_6_volt_telemetry = myserial_act_t4_in.servo_6_volt_int;

        int16_t servo_1_temp_telemetry = myserial_act_t4_in.servo_1_temp_int;
        int16_t servo_2_temp_telemetry = myserial_act_t4_in.servo_2_temp_int;
        int16_t servo_3_temp_telemetry = myserial_act_t4_in.servo_3_temp_int;   
        int16_t servo_4_temp_telemetry = myserial_act_t4_in.servo_4_temp_int;
        int16_t servo_5_temp_telemetry = myserial_act_t4_in.servo_5_temp_int;
        int16_t servo_6_temp_telemetry = myserial_act_t4_in.servo_6_temp_int;
        
        float rolling_msg_in_telemetry = myserial_act_t4_in.rolling_msg_in;
        uint8_t rolling_msg_in_id_telemetry = myserial_act_t4_in.rolling_msg_in_id; 


       pprz_msg_send_SERIAL_ACT_T4_IN(trans, dev, AC_ID,
                            &serial_act_t4_missed_packets_in, &serial_act_t4_message_frequency_in, &rolling_msg_in_telemetry, &rolling_msg_in_id_telemetry,
                            &servo_1_angle_int_telemetry, &servo_2_angle_int_telemetry, &servo_3_angle_int_telemetry, &servo_4_angle_int_telemetry,&servo_5_angle_int_telemetry, &servo_6_angle_int_telemetry,
                            &servo_1_update_time_us_telemetry, &servo_2_update_time_us_telemetry, &servo_3_update_time_us_telemetry, &servo_4_update_time_us_telemetry,&servo_5_update_time_us_telemetry, &servo_6_update_time_us_telemetry,
                            &servo_1_load_telemetry, &servo_2_load_telemetry, &servo_3_load_telemetry, &servo_4_load_telemetry,&servo_5_load_telemetry, &servo_6_load_telemetry,
                            &servo_1_speed_telemetry, &servo_2_speed_telemetry, &servo_3_speed_telemetry,&servo_4_speed_telemetry, &servo_5_speed_telemetry, &servo_6_speed_telemetry,
                            &servo_1_volt_telemetry, &servo_2_volt_telemetry, &servo_3_volt_telemetry,&servo_4_volt_telemetry, &servo_5_volt_telemetry, &servo_6_volt_telemetry,
                            &servo_1_temp_telemetry, &servo_2_temp_telemetry, &servo_3_temp_telemetry,&servo_4_temp_telemetry, &servo_5_temp_telemetry, &servo_6_temp_telemetry
                            
                            );

    }

    static void serial_act_t4_uplink(struct transport_tx *trans, struct link_device *dev)
    {
    
    int8_t servo_arm_int_telemetry = myserial_act_t4_out.servo_arm_int;

    int16_t servo_1_angle_cmd_int_telemetry = myserial_act_t4_out.servo_1_cmd_int; 
    int16_t servo_2_angle_cmd_int_telemetry = myserial_act_t4_out.servo_2_cmd_int;  
    int16_t servo_3_angle_cmd_int_telemetry = myserial_act_t4_out.servo_3_cmd_int; 
    int16_t servo_4_angle_cmd_int_telemetry = myserial_act_t4_out.servo_4_cmd_int; 
    int16_t servo_5_angle_cmd_int_telemetry = myserial_act_t4_out.servo_5_cmd_int; 
    int16_t servo_6_angle_cmd_int_telemetry = myserial_act_t4_out.servo_6_cmd_int; 

    float rolling_msg_out_telemetry = myserial_act_t4_out.rolling_msg_out;
    uint8_t rolling_msg_out_id_telemetry = myserial_act_t4_out.rolling_msg_out_id;

	   pprz_msg_send_SERIAL_ACT_T4_OUT(trans, dev, AC_ID, 
                    &servo_arm_int_telemetry,
                    &servo_1_angle_cmd_int_telemetry, &servo_2_angle_cmd_int_telemetry, &servo_3_angle_cmd_int_telemetry, 
                    &servo_4_angle_cmd_int_telemetry, &servo_5_angle_cmd_int_telemetry, &servo_6_angle_cmd_int_telemetry, 
                    &rolling_msg_out_telemetry, &rolling_msg_out_id_telemetry);

    }

#endif

static void data_serial_act_t4_out(uint8_t sender_id __attribute__((unused)), struct serial_act_t4_out * myserial_act_t4_out_ptr, float * serial_act_t4_extra_data_out_ptr){
    //Copying the struct to be transmitted (and the extra datas) in a local variable: 
    memcpy(&myserial_act_t4_out,myserial_act_t4_out_ptr,sizeof(struct serial_act_t4_out));
    memcpy(&serial_act_t4_extra_data_out,serial_act_t4_extra_data_out_ptr, sizeof(serial_act_t4_extra_data_out) );

    //Increase the counter to track the sending messages:
    myserial_act_t4_out.rolling_msg_out = 0.0;
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

void serial_act_t4_init() 
{
    serial_act_t4_buf_in_cnt = 0;
    serial_act_t4_out_msg_id = 0;

    //Init abi bind msg:
    AbiBindMsgSERIAL_ACT_T4_OUT(ABI_BROADCAST, &SERIAL_ACT_T4_OUT, data_serial_act_t4_out);

    #if PERIODIC_TELEMETRY
    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_SERIAL_ACT_T4_IN, serial_act_t4_downlink);
    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_SERIAL_ACT_T4_OUT, serial_act_t4_uplink);
    #endif
}

/* Send the received message over ABI so then someone else can use it */
void serial_act_t4_parse_msg_in(void)
{
    memcpy(&myserial_act_t4_in, &serial_act_t4_msg_buf_in[1], sizeof(struct serial_act_t4_in)); //Starting from 1 to avoid reading the starting byte
    //Assign the rolling message:
    serial_act_t4_extra_data_in[myserial_act_t4_in.rolling_msg_in_id] = myserial_act_t4_in.rolling_msg_in;
    //Send msg through ABI:
    AbiSendMsgSERIAL_ACT_T4_IN(ABI_SERIAL_ACT_T4_IN_ID, &myserial_act_t4_in, &serial_act_t4_extra_data_in[0]);
}

/* Event checking if serial packet are available on the bus */
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



