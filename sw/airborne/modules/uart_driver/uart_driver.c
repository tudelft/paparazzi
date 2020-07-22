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

#include <stdio.h>
#include <string.h>
#include "mcu_periph/sys_time.h"
#include "state.h"

#include "modules/uart_driver/uart_driver.h"

/* add pprz messages for jetson heartbeat later */
// #include "subsystems/datalink/telemetry.h"

// #define DBG

/* deep copied variable */
thurst_frame_t uart_rx_buffer;

/* mutex for rx struct */
pthread_mutex_t* rx_mutex;

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

	return (i + 3);
}

static void tx_data_struct(divergence_packet_t *uart_packet_tx) {
	uint8_t tx_string[UART_MAX_LEN] = {0};

	// copy packed struct into a string
	memcpy(tx_string, uart_packet_tx, sizeof(divergence_packet_t));

	#ifdef DBG
	printf("[TX] cnt: %i, div: %f, divdot: %f\n", uart_packet_tx->data.cnt, uart_packet_tx->data.divergence, uart_packet_tx->data.divergence_dot);
	#endif
	
	// send "stringed" struct
	send_to_jetson(tx_string, sizeof(divergence_packet_t));
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

				// this if loop will barely be used!
				/* if (packet_type == ACK_FRAME && packet_length == 3) {
					// jetson sent you an ACK_FRAME indicating it received the earlier sent data from bebop, 
					// indicate that on bool pprz msg ack? 
					jetson_state = JTSN_RX_OK;
					byte_ctr = 0;
				} */

        /* packet length will always be shorter than padded struct, create some leeway */
        if (packet_type == DATA_FRAME && (packet_length >= (sizeof(info_frame_t) + sizeof(thurst_frame_t)))) {
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

			if (byte_ctr < packet_length+1) {
				/* fill a databuf from zero and calculate data+info checksum */
				databuf[byte_ctr - st_byte_pos] = c;
				checksum += databuf[byte_ctr - st_byte_pos];
				byte_ctr = byte_ctr + 1;
			}

			/* after rx, go for error checking */
			if (byte_ctr == packet_length+1) {
				byte_ctr = 0;
				jetson_state = JTSN_ERR_CHK;
			}
		} break;

		case JTSN_ERR_CHK: {
			/* take in the last byte and check if it matches data+info checksum */
			if (c == checksum) {
				jetson_state = JTSN_RX_OK;
			}	else {
				jetson_state = JTSN_RX_ERR;
			}
			/* todo: think of footer logic
			if (c == "*") {
				jetson_state = JTSN_RX_OK;
			} */
		} // no break statement required;

    case JTSN_RX_OK: {
			if (packet_type == DATA_FRAME) {
				/* checksum matches, proceed to populate the data struct */
				/* hope this is atomic, vo reads from externed dr_data */
				pthread_mutex_lock(rx_mutex);
				memcpy(&uart_rx_buffer, &databuf, sizeof(thurst_frame_t));
				pthread_mutex_unlock(rx_mutex);

				#ifdef DBG
				printf("[RX] cnt: %i, thrust: %f\n", uart_rx_buffer.cnt, uart_rx_buffer.thurst);
				#endif
			}
		
			/* reset state machine */
			memset(databuf, 0, UART_MAX_LEN);
			checksum = 0;
			jetson_state = JTSN_SYNC;
		} break;

    case JTSN_RX_ERR: {
			memset(databuf, 0, UART_MAX_LEN);
			byte_ctr = 0;
			checksum = 0;
			/* reset state machine, string terminated earlier than expected */
			jetson_state = JTSN_SYNC;
    } break;

    default: {
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

	/* mutex for rx struct */
	rx_mutex = malloc(sizeof(pthread_mutex_t));
	pthread_mutex_init(rx_mutex, NULL);

}

// send data to jetson every few milliseconds
void uart_driver_tx_loop() {
	static int cnt = 0;
	cnt++;

	// TODO: send actual values
	divergence_packet_t uart_packet_tx = {
		.info = {
			.packet_type = DATA_FRAME,
			.packet_length = 3 + sizeof(divergence_packet_t),
		},
		.data = {
			.cnt = cnt,
			.divergence = 0.75f,
			.divergence_dot = 0.14f,
		},
	};

	/* send to jetson */
	tx_data_struct(&uart_packet_tx);

}

// event-triggered function to send data to jetson
void uart_driver_tx_event(float divergence, float divergence_dot) {
	static int cnt = 0;
	cnt++;

	divergence_packet_t uart_packet_tx = {
		.info = {
			.packet_type = DATA_FRAME,
			.packet_length = 3 + sizeof(divergence_packet_t),
		},
		.data = {
			.cnt = cnt,
			.divergence = divergence,
			.divergence_dot = divergence_dot,
		},
	};

	/* send to jetson */
	tx_data_struct(&uart_packet_tx);

}