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
 * @file "modules/vutura/vutura_utm_interface.c"
 * @author Dennis van Wijngaarden
 * Sends over position and speed data to vutura utm module
 */

#include "modules/vutura/vutura_utm_interface.h"
#include "state.h"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <arpa/inet.h>


#define VUTURA_UTM_INTERFACE_VERBOSE 1

#define PRINT(string,...) fprintf(stderr, "[orange_avoider->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if VUTURA_UTM_INTERFACE_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

struct vutura_utm_interface_t {
	int fd;
	struct sockaddr_in ext_addr;
	struct sockaddr_in loc_addr;
} vutura_utm_data;

AvoidanceParameters avoidance;


void init_vutura_utm_interface(void)
{
	// initialise constants
	avoidance.avoid = false;
	avoidance.vn = 0;
	avoidance.ve = 0;
	avoidance.vd = 0;

	// Make a UDP connection
	vutura_utm_data.fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

	// Listen to Vutura UTM
	memset(&vutura_utm_data.loc_addr, 0, sizeof(vutura_utm_data.loc_addr));
	vutura_utm_data.loc_addr.sin_family = AF_INET;
	vutura_utm_data.loc_addr.sin_addr.s_addr = INADDR_ANY;
	vutura_utm_data.loc_addr.sin_port = htons(8200);

	if (bind(vutura_utm_data.fd, (struct sockaddr *)&vutura_utm_data.loc_addr, sizeof(struct sockaddr)) == -1)
	{
		VERBOSE_PRINT("UDP bind failed");
		close(vutura_utm_data.fd);
		exit(EXIT_FAILURE);
	}

	// Configure socket for sending from UAV to Vutura_utm
	memset(&vutura_utm_data.ext_addr, 0, sizeof(vutura_utm_data.ext_addr));
	vutura_utm_data.ext_addr.sin_family = AF_INET;
	vutura_utm_data.ext_addr.sin_addr.s_addr = inet_addr("127.0.0.1");
	vutura_utm_data.ext_addr.sin_port = htons(14551);

}

void parse_gps(void)
{
	struct LlaCoor_i *lonlatalt = stateGetPositionLla_i();
	struct NedCoor_f *speed_ned = stateGetSpeedNed_f();

	// Construct message
	PaparazziToVuturaMsg msg;

	msg.lon = lonlatalt->lon; // [deg e7]
	msg.lat = lonlatalt->lat; // [deg e7]]
	msg.alt = lonlatalt->alt; // [mm]
	msg.Vn  = speed_ned->x * 1000.; // [mm/s]
	msg.Ve  = speed_ned->y * 1000.; // [mm/s]
	msg.Vd  = speed_ned->z * 1000.; // [mm/s]

	//VERBOSE_PRINT("lon %i [degE7], lat %i [degE7], alt %i [mm] \n", msg.lon, msg.lat, msg.alt);
	//VERBOSE_PRINT("Vn %i [mm/s], Ve %i [mm/s], Vd %i [mm/s] \n", msg.Vn, msg.Ve, msg.Vd);

	// Put it in the buffer

	// Send over UDP

	ssize_t bytes_sent = sendto(vutura_utm_data.fd, &msg, sizeof(msg), 0, (struct sockaddr*)&vutura_utm_data.ext_addr, sizeof(struct sockaddr_in));
	(void) bytes_sent;
	return;
}

void avoid_check(void)
{

	VuturaToPaparazziMsg msg;

	socklen_t addr_len = sizeof(vutura_utm_data.loc_addr);
	ssize_t count = recvfrom(vutura_utm_data.fd, &msg, sizeof(msg), MSG_DONTWAIT, (struct sockaddr*)&vutura_utm_data.loc_addr, &addr_len);
	if (count == -1)
		{
			//VERBOSE_PRINT("No UDP data\n");
		}
		else if (count == sizeof(msg))
		{
			avoidance.avoid = msg.avoid;
			avoidance.vn = msg.vn;
			avoidance.ve = msg.ve;
			avoidance.vd = msg.vd;
			VERBOSE_PRINT("received avoidance msg: avoid->%i, vn->%i, ve->%i, vd->%i\n", avoidance.avoid, avoidance.vn, avoidance.ve, avoidance.vd);
		}
		else
		{
			//VeRBOSE_PRINT("msg not of correct size);
		}

	return;
}

bool GetAvoid(void)
{
	return avoidance.avoid;
}

