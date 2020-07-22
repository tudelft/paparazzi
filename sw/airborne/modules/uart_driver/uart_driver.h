/*
 * Copyright (C) nilay
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
 * @file "modules/uart_driver/uart_driver.h"
 * @author nilay
 */

#ifndef UART_DRIVER_H
#define UART_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <pthread.h>
#include "mcu_periph/uart.h"

#define UART_MAX_LEN 30

typedef enum {
  ACK_FRAME = 0,
  DATA_FRAME,
  OTHER_FRAME
} frame_type_t;

// decoder state
typedef enum {
  JTSN_SYNC = 0,
  JTSN_INFO,
  JTSN_DATA,
  JTSN_ERR_CHK,  // TODO: shift to crc
  JTSN_RX_OK,
  JTSN_RX_ERR
} jetson_state_t;

typedef struct __attribute__((packed)) {
  uint8_t packet_type;
  uint8_t packet_length; 
} info_frame_t;

/*
LOIHI LANDER - TX DIVERGENCE
*/

typedef struct __attribute__((packed)) {
  int cnt;
  float divergence;
  float divergence_dot;
} divergence_frame_t;

typedef struct __attribute__((packed)) {
  info_frame_t info;
  divergence_frame_t data;
} divergence_packet_t;

/*
LOIHI LANDER - RX THRUST
*/

typedef struct __attribute__((packed)) {
  int cnt;
  float thurst;
} thurst_frame_t;

typedef struct __attribute__((packed)) {
  info_frame_t info;
  thurst_frame_t data;
} thurst_packet_t;

extern thurst_frame_t uart_rx_buffer;
extern pthread_mutex_t* rx_mutex;

extern void uart_driver_rx_event(void);
extern void uart_driver_init(void);
extern void uart_driver_tx_loop(void);
extern void uart_driver_tx_event(float divergence, float divergence_dot);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif
