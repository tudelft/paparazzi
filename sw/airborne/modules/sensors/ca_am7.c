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

static uint8_t am7_msg_buf[2048]  __attribute__((aligned));   ///< The message buffer for the device chosen to be 2* message_size total
//int* message = malloc(MESSAGE_LENGTH*sizeof(int));
static uint8_t message[MESSAGE_LENGTH]  __attribute__((aligned));
int loop_var;
uint16_t check_sum = 0;
uint16_t verify_bytes = 0;

static struct am7_data myam7_data;

#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"
static void am7_downlink(struct transport_tx *trans, struct link_device *dev)
{
	   pprz_msg_send_AM7(trans, dev, AC_ID, &myam7_data.bale_no, &myam7_data.rx_throttle, &myam7_data.output_throttle,
		 	  &myam7_data.rpm, &myam7_data.voltage, &myam7_data.busbar_current, &myam7_data.phase_wire_current, &myam7_data.mosfet_temp,
		 	  &myam7_data.capacitor_temp, &myam7_data.status_code,  &myam7_data.verify_code );
}
#endif

void am7_init() 
{

 	pprz_transport_init(&am7.transport);
 	uart_periph_set_baudrate(&uart1, B115200);
 	myam7_data.buffer_counter = 0;
 #if PERIODIC_TELEMETRY
   register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_AM7, am7_downlink);
 #endif
}

/* Parse the InterMCU message */
void am7_parse_msg(void)
{
	// Dummy variables to apply the bitshifting later (for scaling according to protocol)
// 	  uint16_t rx_throttle_dummy = (am7_msg_buf[6] <<8 | am7_msg_buf[7]);
// 	  uint16_t output_throttle_dummy = (am7_msg_buf[8] <<8 | am7_msg_buf[9]);
// 	  uint16_t rpm_dummy = (am7_msg_buf[10] <<8 | am7_msg_buf[11]);

// 	  uint16_t rx_throttle_dummy_2 = rx_throttle_dummy << 6 + rx_throttle_dummy << 5 + rx_throttle_dummy << 2;
// 	  uint16_t output_throttle_dummy_2 = output_throttle_dummy << 6 + output_throttle_dummy << 5 + output_throttle_dummy << 2;
// 	  uint16_t rpm_dummy_2 = rpm_dummy << 3 + rpm_dummy << 1 ;

// 	  uint16_t rpm_dummy_3 = (rpm_dummy_2 >> 6 );

// 	  // Temperature lookup table
// 	  uint8_t mosfet_temp_adc = am7_msg_buf[18];
// 	  uint8_t capacitor_temp_adc = am7_msg_buf[19];
// 	  const uint8_t tempTable[ 220 ] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,16,17,18,19,19,20,21,21,22,23,23,24,25,25,26,26,27,28,28,29,29,30,30,
// 			  31,32,32,33,33,34,34,35,35,36,36,37,37,38,38,39,39,40,40,41,41,42,42,43,43,44,44,44,45,45,46,46,47,47,48,48,49,49,50,50,50,51,51,52,52,53,53,
// 			  53,54,54,55,55,56,56,57,57,58,58,58,59,59,60,60,61,61,61,62,62,63,63,64,64,65,65,66,66,66,67,67,68,68,69,69,70,70,71,71,72,72,73,73,74,74,75,
// 			  75,75,76,77,77,78,78,79,79,80,80,81,81,82,82,83,83,84,85,85,86,86,86,87,88,88,89,90,90,91,92,92,93,94,95,95,96,96,96,97,98,98,99,100,101,101,
// 			  102,103,103,104,105,106,106,107,107,108,109,110,110,111,112,113,113,114,115,115,116,117,117,118,119,120,120,121,122,122,123,124,125,125,126,
// 			  127,127,128,129,129};

// 	  int8_t dummy_idx_mosfet = 241 - mosfet_temp_adc;
// 	  int8_t dummy_idx_capacitor = 241 - capacitor_temp_adc;

// 	  // Read the AM7 State information
// 	  myam7_data.bale_no = (uint16_t)((am7_msg_buf[4] <<8 | am7_msg_buf[5]));
// 	  myam7_data.rx_throttle = (uint16_t)((rx_throttle_dummy_2)>>10); // Original (uint16_t)((am7_msg_buf[6] <<8 | am7_msg_buf[7])*100/1024);
// 	  myam7_data.output_throttle = (uint16_t)((output_throttle_dummy_2)>>10); // Original (uint16_t)((am7_msg_buf[8] <<8 | am7_msg_buf[9])*100/1024);
// 	  myam7_data.rpm = (uint16_t)(rpm_dummy_2 /108); // (uint16_t)((am7_msg_buf[10] <<8 | am7_msg_buf[11])*10/108);
// 	  myam7_data.voltage = ((uint16_t)((am7_msg_buf[12] <<8 | am7_msg_buf[13]))); // Needs to be divided by 10 to get real voltage
// 	  myam7_data.busbar_current = ((int16_t)((am7_msg_buf[14] <<8 | am7_msg_buf[15])));// Needs to be divided by 64
// 	  myam7_data.phase_wire_current = ((int16_t)((am7_msg_buf[16] <<8 | am7_msg_buf[17]))); //Needs to be divided by 64
// 	  myam7_data.mosfet_temp = tempTable[dummy_idx_mosfet];
// 	  myam7_data.capacitor_temp = tempTable[dummy_idx_capacitor];
// 	  myam7_data.status_code = (uint16_t)((am7_msg_buf[20] <<8 | am7_msg_buf[21]));
// 	  myam7_data.verify_code = (uint16_t)((am7_msg_buf[22] <<8 | am7_msg_buf[23]));// shift the right byte instead of the

// #if PERIOD_TELEMETRY
//  // register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_ALPHA_ESC, alpha_esc_downlink);
// #endif

}

/* We need to wait for incoming messages */
void am7_event()
{	
	 uint8_t am7_byte;
	 am7_byte = uart_getch(&uart1);
	 if ((am7_byte == START_BYTE)|| (myam7_data.buffer_counter > 0)) {
	 	am7_msg_buf[myam7_data.buffer_counter] = am7_byte;
	 	myam7_data.buffer_counter = 	myam7_data.buffer_counter + 1;
	 }

	 if (myam7_data.buffer_counter > 24){
	 	check_sum = am7_msg_buf[0] + am7_msg_buf[1] + am7_msg_buf[2] + am7_msg_buf[3] + am7_msg_buf[4] +
	 			am7_msg_buf[5] + am7_msg_buf[6] + am7_msg_buf[7] + am7_msg_buf[8] + am7_msg_buf[9] +
	 			am7_msg_buf[10] + am7_msg_buf[11] + am7_msg_buf[12] + am7_msg_buf[13] + am7_msg_buf[14] +
	 			am7_msg_buf[15] + am7_msg_buf[16] + am7_msg_buf[17] + am7_msg_buf[18] + am7_msg_buf[19]+
	 			am7_msg_buf[20] + am7_msg_buf[21] ;

	 	verify_bytes = (uint16_t)((am7_msg_buf[22]) | (am7_msg_buf[23]<<8));
	 	myam7_data.buffer_counter = 0;
	 	if ((am7_msg_buf[1]==22) && (am7_msg_buf[2]== 1) && (am7_msg_buf[3]== 2) && (verify_bytes == check_sum) )
	 	{
	 		am7_parse_msg();
	 	}
	 }
}

