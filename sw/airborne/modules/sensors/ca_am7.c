/*
 * Copyright (C) 2022 OpenUAS
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
 * @file "modules/ca_am7.c"
 * @author OpenUAS
 */

#include "modules/sensors/ca_am7.h"
#include "pprzlink/pprz_transport.h"
//#include "pprzlink/intermcu_msg.h"
#include "mcu_periph/uart.h"
#include "mcu_periph/sys_time.h"

/* Main AM7 structure */
static struct am7_t am7 = {
  .device = (&((AM7_PORT).device)),
  .msg_available = false
};

static uint8_t am7_msg_buf_in[MESSAGE_LENGTH_IN*2]  __attribute__((aligned));   ///< The message buffer for the device chosen to be 2* message_size total

static struct am7_data_in myam7_data_in;
static struct am7_data_out myam7_data_out;
uint16_t buffer_in_counter;

#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"
static void am7_downlink(struct transport_tx *trans, struct link_device *dev)
{
	   pprz_msg_send_AM7_IN(trans, dev, AC_ID, &myam7_data_in.motor_1_cmd_int, &myam7_data_in.motor_2_cmd_int, &myam7_data_in.motor_3_cmd_int,
		 	  &myam7_data_in.motor_4_cmd_int, &myam7_data_in.el_1_cmd_int, &myam7_data_in.el_2_cmd_int, &myam7_data_in.el_3_cmd_int,
		 	  &myam7_data_in.el_4_cmd_int, &myam7_data_in.az_1_cmd_int,  &myam7_data_in.az_2_cmd_int, &myam7_data_in.az_3_cmd_int,
              &myam7_data_in.az_4_cmd_int);
}
static void am7_uplink(struct transport_tx *trans, struct link_device *dev)
{
	   pprz_msg_send_AM7_OUT(trans, dev, AC_ID, &myam7_data_out.motor_1_state_int, &myam7_data_out.motor_2_state_int, &myam7_data_out.motor_3_state_int,
		 	  &myam7_data_out.motor_4_state_int, &myam7_data_out.el_1_state_int, &myam7_data_out.el_2_state_int, &myam7_data_out.el_3_state_int,
		 	  &myam7_data_out.el_4_state_int, &myam7_data_out.az_1_state_int,  &myam7_data_out.az_2_state_int, &myam7_data_out.az_3_state_int,
              &myam7_data_out.az_4_state_int);
}
#endif

void am7_init() 
{
    myam7_data_in.buffer_counter = 0;
 #if PERIODIC_TELEMETRY
   register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_AM7_IN, am7_downlink);
   register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_AM7_OUT, am7_uplink);
 #endif
}

/* Parse the InterMCU message */
void am7_parse_msg_in(void)
{
    // Read the AM7 State information
    myam7_data_in.motor_1_cmd_int = (int16_t)((am7_msg_buf_in[2] <<8 | am7_msg_buf_in[1]));
    myam7_data_in.motor_2_cmd_int = (int16_t)((am7_msg_buf_in[4] <<8 | am7_msg_buf_in[3]));
    myam7_data_in.motor_3_cmd_int = (int16_t)((am7_msg_buf_in[6] <<8 | am7_msg_buf_in[5]));
    myam7_data_in.motor_4_cmd_int = (int16_t)((am7_msg_buf_in[8] <<8 | am7_msg_buf_in[7]));

    myam7_data_in.el_1_cmd_int = (int16_t)((am7_msg_buf_in[10] <<8 | am7_msg_buf_in[9]));
    myam7_data_in.el_2_cmd_int = (int16_t)((am7_msg_buf_in[12] <<8 | am7_msg_buf_in[11]));
    myam7_data_in.el_3_cmd_int = (int16_t)((am7_msg_buf_in[14] <<8 | am7_msg_buf_in[13]));
    myam7_data_in.el_4_cmd_int = (int16_t)((am7_msg_buf_in[16] <<8 | am7_msg_buf_in[15]));

    myam7_data_in.az_1_cmd_int = (int16_t)((am7_msg_buf_in[18] <<8 | am7_msg_buf_in[17]));
    myam7_data_in.az_2_cmd_int = (int16_t)((am7_msg_buf_in[20] <<8 | am7_msg_buf_in[19]));
    myam7_data_in.az_3_cmd_int = (int16_t)((am7_msg_buf_in[22] <<8 | am7_msg_buf_in[21]));
    myam7_data_in.az_4_cmd_int = (int16_t)((am7_msg_buf_in[24] <<8 | am7_msg_buf_in[23]));

}

void am7_periodic(){
    myam7_data_out.motor_1_state_int =  3000;
    myam7_data_out.motor_2_state_int =  4000;
    myam7_data_out.motor_3_state_int =  5000;
    myam7_data_out.motor_4_state_int = 60;

    myam7_data_out.el_1_state_int = 315;
    myam7_data_out.el_2_state_int = 11345;
    myam7_data_out.el_3_state_int = 519;
    myam7_data_out.el_4_state_int = 611;

    myam7_data_out.az_1_state_int = 322;
    myam7_data_out.az_2_state_int = 4234;
    myam7_data_out.az_3_state_int = 525;
    myam7_data_out.az_4_state_int = 4067;

    uint8_t buf_send[MESSAGE_LENGTH_OUT];

    memcpy( & buf_send[0], &myam7_data_out, MESSAGE_LENGTH_OUT);
    uart_put_byte(&(AM7_PORT), 0, START_BYTE);
    for(uint8_t i = 0; i < MESSAGE_LENGTH_OUT ; i++){
        uart_put_byte(&(AM7_PORT), 0, buf_send[i]);
    }


}
/* We need to wait for incoming messages */
void am7_event()
{

    while(uart_char_available(&(AM7_PORT)) > 0) {

        uint8_t am7_byte_in;
        am7_byte_in = uart_getch(&(AM7_PORT));

        if ((am7_byte_in == START_BYTE) || (buffer_in_counter > 0)) {
            am7_msg_buf_in[buffer_in_counter] = am7_byte_in;
            buffer_in_counter++;
        }

        if (buffer_in_counter > MESSAGE_LENGTH_IN) {
            buffer_in_counter = 0;
            am7_parse_msg_in();
        }
    }
}

