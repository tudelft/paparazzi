/*
 * Copyright (C) nilay994
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
 * @file "modules/uart_driver/uart_driver.c"
 * @author nilay_994@hotmail.com
 * uart between jetson and bebop
 */
/*
+--------+------------+--------------+--------------------------------+--------------+--------------+-------------------+----------+
| field  | start byte | frame type   | packet length                  | pot          | button       | checksum          | end byte |
|        |            | (info frame) | (info frame)                   | (data frame) | (data frame) |                   |          |
+--------+------------+--------------+--------------------------------+--------------+--------------+-------------------+----------+
| value  | $          | DATA/OTHER   | start byte to data length = 8B | float        | fragged bool | info + data frame | *        |
+--------+------------+--------------+--------------------------------+--------------+--------------+-------------------+----------+
| length | 1B         | 1B           | 1B                             | 4B           | 1B           | 1B                | 1B       |
+--------+------------+--------------+--------------------------------+--------------+--------------+-------------------+----------+
*/
/* printf, fprintf */
#include <stdio.h>

/* memset, memcpy */
#include <string.h>

/* float sys get time */
#include "mcu_periph/sys_time.h"

/* gps functions */
#include "state.h"

#include "modules/uart_driver/uart_driver.h"

/* add pprz messages for jetson heartbeat later */
// #include "subsystems/datalink/telemetry.h"

#define DBG

/* deep copied variable */
data_frame_t uart_rx_buffer;

// tx: finally send a hex array to jetson
static uint8_t send_to_jetson(uint8_t *s, uint8_t len) {

	// augment start bytes
	uart_put_byte(&(UART_DRIVER_PORT), 0, 0x24); // '$'

	uint8_t checksum = 0;

	// maximum of 255 bytes
 	uint8_t i = 0;
	for (i = 0; i < len; i ++) {
		uart_put_byte(&(UART_DRIVER_PORT), 0, s[i]);
		checksum += s[i];
	}	
	uart_put_byte(&(UART_DRIVER_PORT), 0, checksum);
	uart_put_byte(&(UART_DRIVER_PORT), 0, 0x2a); // '*'

	#ifdef DBG
		printf("appended checksum when bbp tx: 0x%02x\n", checksum);
	#endif

	return (i + 3);
}

// tx: send struct to esp32
static void tx_data_struct(uart_packet_t *uart_packet_tx) {

	uint8_t tx_string[UART_MAX_LEN] = {0};
	// copy packed struct into a string
	memcpy(tx_string, uart_packet_tx, sizeof(uart_packet_t));

	#ifdef DBG
		printf("[tx] type: %d, pot: %f\n", 
		uart_packet_tx->info.packet_type,
		uart_packet_tx->data.pot);

		// check via wireshark
		printf("Jetson should receive bytes:\n");
		for (int i = 0; i < sizeof(uart_packet_t); i++) {
			printf("0x%02x,", tx_string[i]);
		}
		printf(" + checksum ****\n");
	#endif
	
	// send "stringed" struct
	send_to_jetson(tx_string, sizeof(uart_packet_t));
}

// rx: print struct received after checksum match
static void print_rx_struct(uart_packet_t *uart_packet_rx) {
	printf("[rx] type: %d, len: %d, pot: %f, but0: %d\n", 
					uart_packet_rx->info.packet_type,
					uart_packet_rx->info.packet_length, 
					uart_packet_rx->data.pot,
					uart_packet_rx->data.but0);
}

// rx: parse uart bytes that are sent from the jetson board
static void parse(uint8_t c) {

	// start state-machine for syncing to jetson
	static uint8_t jetson_state = JTSN_SYNC;

	// temp data-buffer to store jetson data
	static uint8_t databuf[UART_MAX_LEN] = {0};

	static uint8_t byte_ctr = 0;
	static uint8_t packet_type = 0;
	static uint8_t packet_length = 0;
	static uint8_t checksum = 0;
	// static uint8_t prev_char = 0;
	
	#ifdef DBG
  	printf("byte ctr: %d, jetson state: %d, char rxed: 0x%02x\n", byte_ctr, jetson_state, c);
	#endif
  
	// uart state machine
	switch (jetson_state) {
		
    case JTSN_SYNC: {
			/* first char, sync string */
			if (c == '$') {
				byte_ctr = byte_ctr + 1;
				jetson_state = JTSN_INFO;
    	}
		} break;

    case JTSN_INFO: {
			if (byte_ctr == 1) {
				/* take note of packet type */
				packet_type = c;
				byte_ctr = byte_ctr + 1;
				break;
			} 
			if (byte_ctr == 2) {
				/* take note of packet length */
				packet_length = c;
				byte_ctr = byte_ctr + 1;

				// info frame populated!! 
				printf("[uart] packet_length: %d, packet_type: %d\n", packet_length, packet_type);

				// this if loop will barely be used!
				/* if (packet_type == ACK_FRAME && packet_length == 3) {
					// jetson sent you an ACK_FRAME indicating it received the earlier sent data from bebop, 
					// indicate that on bool pprz msg ack? 
					jetson_state = JTSN_RX_OK;
					byte_ctr = 0;
				} */

        /* packet length will always be shorter than padded struct, create some leeway */
        if (packet_type == DATA_FRAME && (packet_length >= (sizeof(info_frame_t) + sizeof(data_frame_t)))) {
					// overwrite old checksum, start afresh
					checksum = packet_type + packet_length;
					jetson_state = JTSN_DATA;
				}  
				if (packet_length > UART_MAX_LEN) {
					printf("[uart-err] Packet unexpectedly long\n");
					jetson_state = JTSN_RX_ERR;
				}
			} else {
				// do nothing?!
			}
		} break;

		case JTSN_DATA: {
			// start byte = 2 bytes from info frame + 1 packet start byte
			const uint8_t st_byte_pos = sizeof(info_frame_t) + 1;

			if (byte_ctr < packet_length) {
				/* fill a databuf from zero and calculate data+info checksum */
				databuf[byte_ctr - st_byte_pos] = c;
				checksum += databuf[byte_ctr - st_byte_pos];
				byte_ctr = byte_ctr + 1;
			}

			/* after rx, go for error checking */
			if (byte_ctr == packet_length) {
				byte_ctr = 0;
				jetson_state = JTSN_ERR_CHK;
			}
		} break;

		case JTSN_ERR_CHK: {
			/* take in the last byte and check if it matches data+info checksum */
			if (c == checksum) {
				jetson_state = JTSN_RX_OK;
				#ifdef DBG
					printf("[uart] checksum matched!\n");
				#endif
			}	else {
				jetson_state = JTSN_RX_ERR;
			}
			/* todo: think of footer logic
			if (c == "*") {
				jetson_state = JTSN_RX_OK;
			} */
		} // no break statement required;

    case JTSN_RX_OK: {
			#ifdef DBG
			printf("[uart] received string: ");
			// print string
			for (int i = 0; i < packet_length; i++) {
				printf("0x%02x,", databuf[i]);
			}
			#endif
			if (packet_type == DATA_FRAME) {
				/* checksum matches, proceed to populate the data struct */
				/* hope this is atomic, vo reads from externed dr_data */
				memcpy(&uart_rx_buffer, &databuf, sizeof(data_frame_t));

				/* checksum matches, proceed to populate the info struct */
				uart_packet_t tmp_uart_packet_rx = {
					.info = {
						.packet_type = packet_type,
						.packet_length = packet_length,
					},			
				};
				/* now rest of the packet.. for dropout logs later */
				tmp_uart_packet_rx.data = uart_rx_buffer;
				#ifdef DBG
					print_rx_struct(&tmp_uart_packet_rx);
				#endif
			}
		
			/* reset state machine */
			memset(databuf, 0, UART_MAX_LEN);
			checksum = 0;
			jetson_state = JTSN_SYNC;
		} break;

    case JTSN_RX_ERR: {
			#ifdef DBG
				printf("[uart] JTSN_RX_ERR\n");
			#endif
			memset(databuf, 0, UART_MAX_LEN);
			byte_ctr = 0;
			checksum = 0;
			/* reset state machine, string terminated earlier than expected */
			jetson_state = JTSN_SYNC;
    } break;

    default: {
			#ifdef DBG
				printf("[uart] JTSN_RX_ERR\n");
			#endif
			memset(databuf, 0, UART_MAX_LEN);
			byte_ctr = 0;
			checksum = 0;
			jetson_state = JTSN_SYNC;
		} break;
  }

	// /* reset state machine asap, when start pattern is observed */
	// if (prev_char == '$' && c == 178) {
	// 	byte_ctr = 2;
	// 	jetson_state = ESP_DRONE_INFO;
  // }
	/* for start byte pattern check */
	// prev_char = c;
}

// uart rx: event based polling function for data rxed from esp32
void uart_driver_rx_event(void) {
	// Look for data on serial link and send to parser
	while (uart_char_available(&(UART_DRIVER_PORT))) {
		uint8_t ch = uart_getch(&(UART_DRIVER_PORT));
		parse(ch);
	}
}


void uart_driver_init() {

}

// send data to jetson every few milliseconds
void uart_driver_tx_loop() {
	// bool gps_valid = stateIsLocalCoordinateValid();
	// if (gps_valid) {

		uart_packet_t uart_packet_tx = {
			.info = {
				.packet_type = DATA_FRAME,
				.packet_length = 1 + sizeof(uart_packet_t),  // 7 byte frame + 1 start bytes
			},
			.data = {
				.pot = 0.25f,
				.but0 = 0,
				.but1 = 1,
				.but2 = 0,
				.but3 = 1,
				.but4 = 0,
			},
		};

		/* send to jetson */
		tx_data_struct(&uart_packet_tx);

		// LOG: MAXDRONES for me = 2 (can't use ID 0x00 packet terminates..):(
		// fmt: x,y,vel,head,time
		// for (int id = 1; id < 3; id++) {
		// 	fprintf(drone_data_f, "%f,%f,", dr_data[id].pos.x, dr_data[id].pos.y);
		// 	fprintf(drone_data_f, "%f,%f,", dr_data[id].vel.x, dr_data[id].vel.y);
		// }
		// fprintf(drone_data_f, "%f\n", get_sys_time_float());

	// mutex, don't tx to esp when ack is being sent
  // if (esp.state!= ESP_RX_OK) {
		
	//}

}
