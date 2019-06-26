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

#ifndef SSD_VSET
#define SSD_VSET 12.
#endif

#include <stdint.h>
#include <stdbool.h>

#include "state.h"

enum utm_state_t {
	UTM_STATE_INIT = 0,
	UTM_STATE_LOGGED_IN,
	UTM_STATE_REQUEST_PENDING,
	UTM_STATE_REQUEST_REJECTED,
	UTM_STATE_REQUEST_APPROVED,
	UTM_STATE_FLIGHT_STARTED
};

enum utm_request_t {
	UTM_REQUEST_FLIGHT,
	UTM_REQUEST_START,
	UTM_REQUEST_END
};

union paparazzi_to_vutura_msg_t {
	struct {
		int32_t lon;
		int32_t lat;
		int32_t alt;
		int32_t Vn;
		int32_t Ve;
		int32_t Vd;
		uint32_t target_wp;
		int32_t wind_north;
		int32_t wind_east;
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
		bool skip_wp;
		uint32_t skip_to_wp;
	};
	unsigned char bytes;
} __attribute((__packed__));
typedef union vutura_to_paparazzi_msg_t VuturaToPaparazziMsg;

union paparazzi_to_utm_interface_t {
	struct {
		int32_t utm_request;
	};
	unsigned char bytes;
} __attribute((__packed__));
typedef union paparazzi_to_utm_interface_t PaparazziToUtmInterfaceMsg;

union utm_interface_to_paparazzi_t {
	struct {
		int32_t utm_state;
	};
	unsigned char bytes;
} __attribute((__packed__));
typedef union utm_interface_to_paparazzi_t UtmInterfaceToPaparazziMsg;

struct avoidance_parameters_t
{
	bool avoid;
	int32_t vn;
	int32_t ve;
	int32_t vd;
	int32_t lat; //[dege7]
	int32_t lon; //[dege7]
	bool skip_wp;
	uint32_t skip_to_wp;
};
typedef struct avoidance_parameters_t AvoidanceParameters;

struct flightplan_parameters_t
{
	uint8_t start_index_ROUTE;
	uint8_t start_index_AVOID;
	uint8_t start_index_LEG_BLOCK;
	uint8_t N_wpts;
	struct LlaCoor_i lla_ref_i;
	struct LtpDef_i ltp_ref_i;
	uint8_t target_leg;
	uint8_t target_wp;
};
typedef struct flightplan_parameters_t FlightplanParameters;

// Global variables
extern AvoidanceParameters avoidance;
extern FlightplanParameters flightplan;
extern enum utm_state_t utm_state;
extern bool avoidance_message_received;

extern void init_vutura_utm_interface(void);
extern void parse_gps(void);
extern void avoid_check(void);
extern void utm_interface_event(void);

// functions to be called by flightplan exceptions
extern bool GetAvoid(void);
extern void ResetAvoid(void);

// functions that reads flightplan and moves avoidance waypoints
extern void InitFlightplan(void);
extern void RunAvoidance(void);
extern void set_wp_at_latlon(uint8_t wp_id, int32_t lat, int32_t lon); // [dege7]
extern bool In_Soft_geofence(float _x, float _y);
extern void set_avoidance_wp_fixed_for_carrot_time(uint8_t wp_id, int32_t lat, int32_t lon); //[dege7]
extern void reset_avoidance_waypoints(void);


// functions to manage utm requests using the flightplan
void utm_request(enum utm_request_t request);

#endif

