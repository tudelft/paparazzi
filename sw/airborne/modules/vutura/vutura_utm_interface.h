/*
 * Copyright (C) Dennis van Wijngaarden
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
 * @file "modules/vutura/vutura_utm_interface.h"
 * @author Dennis van Wijngaarden
 * Sends over position and speed data to vutura utm module
 */

#ifndef VUTURA_UTM_INTERFACE_H
#define VUTURA_UTM_INTERFACE_H

#include <stdint.h>
#include <stdbool.h>

#include "state.h"

union paparazzi_to_vutura_msg_t {
	struct {
		int32_t lon;
		int32_t lat;
		int32_t alt;
		int32_t Vn;
		int32_t Ve;
		int32_t Vd;
	};
	unsigned char bytes;
} __attribute((__packed__));
typedef union paparazzi_to_vutura_msg_t PaparazziToVuturaMsg;

union vutura_to_paparazzi_msg_t {
	struct  {
		bool avoid;
		int32_t vn;
		int32_t ve;
		int32_t vd;
		int32_t lat; //[dege7]
		int32_t lon; //[dege7]
	};
	unsigned char bytes;
} __attribute((__packed__));
typedef union vutura_to_paparazzi_msg_t VuturaToPaparazziMsg;

struct avoidance_parameters_t
{
	bool avoid;
	int32_t vn;
	int32_t ve;
	int32_t vd;
	int32_t lat; //[dege7]
	int32_t lon; //[dege7]
};
typedef struct avoidance_parameters_t AvoidanceParameters;

struct flightplan_parameters_t
{
	uint8_t start_index_ROUTE;
	uint8_t start_index_AVOID;
	struct LlaCoor_i lla_ref_i;
	struct LtpDef_i ltp_ref_i;
};
typedef struct flightplan_parameters_t FlightplanParameters;

// Global variables
extern AvoidanceParameters avoidance;
extern FlightplanParameters flightplan;

extern void init_vutura_utm_interface(void);
extern void parse_gps(void);
extern void avoid_check(void);

// functions to be called by flightplan exceptions
extern bool GetAvoid(void);

// functions that reads flightplan and moves avoidance waypoints
extern void InitFlightplan(void);
extern void RunAvoidance(void);
extern void set_wp_at_latlon(uint8_t wp_id, int32_t lat, int32_t lon); // [dege7]

#endif

