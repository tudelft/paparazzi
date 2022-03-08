/*
 * Copyright (C) Paparazzi Team
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
 * @file "modules/sensors/ca_am7.h"
 * @author OpenUAS
 * Converts telemtry data from a CA device AM7 type to the autopilot
 */

#ifndef AM7_H
#define AM7_H

#define START_BYTE 0x9B  //1st start block identifier byte
#define SECOND_BYTE 0x16 //2nd Start block identifier byte
#define THIRD_BYTE 0x01  //3rd Start block identifier byte
#define FOURTH_BYTE 0x02 //4rd Start block identifier byte

#define MESSAGE_LENGTH_IN 24 // 24 bytes
#define MESSAGE_LENGTH_OUT 24 // 24 bytes

#include "std.h"
#include <stdbool.h>
#include <stdlib.h>
#include "generated/airframe.h"
#include "pprzlink/pprz_transport.h"

/* Main AM7 strcuture */
struct am7_t {
  struct link_device *device;           ///< The device which is used for communication
  struct pprz_transport transport;      ///< The transport layer (PPRZ)
  bool msg_available;                 ///< If we received a message
};

struct   __attribute__((__packed__)) am7_data_in {
    //Motor command
	int16_t motor_1_cmd_int;
	int16_t motor_2_cmd_int;
	int16_t motor_3_cmd_int;
	int16_t motor_4_cmd_int;
	int16_t el_1_cmd_int;
	int16_t el_2_cmd_int;
	int16_t el_3_cmd_int;
    int16_t el_4_cmd_int;
    int16_t az_1_cmd_int;
    int16_t az_2_cmd_int;
    int16_t az_3_cmd_int;
    int16_t az_4_cmd_int;
    int16_t theta_cmd_int;
    int16_t phi_cmd_int;
    //Optimization info
    uint8_t n_iteration;
    uint8_t n_evaluation;
    //Residuals
    int16_t residual_ax_int;
    int16_t residual_ay_int;
    int16_t residual_az_int;
    int16_t residual_p_dot_int;
    int16_t residual_q_dot_int;
    int16_t residual_r_dot_int;
};

struct   __attribute__((__packed__)) am7_data_out {
    //Motor command
    int16_t motor_1_state_int;
    int16_t motor_2_state_int;
    int16_t motor_3_state_int;
    int16_t motor_4_state_int;
    int16_t el_1_state_int;
    int16_t el_2_state_int;
    int16_t el_3_state_int;
    int16_t el_4_state_int;
    int16_t az_1_state_int;
    int16_t az_2_state_int;
    int16_t az_3_state_int;
    int16_t az_4_state_int;
};

extern void am7_init(void);
extern void am7_event(void);
extern void am7_periodic(void);

#endif

