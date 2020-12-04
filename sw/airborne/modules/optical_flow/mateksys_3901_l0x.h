/*
 * Copyright (C) 2020 Paparazzi Team
 * 
 * This file is part of paparazzi.
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
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

/** @file modules/optical_flow/mateksys_3901_l0x.h
 *  @brief Driver for the mateksys_3901_l0x sensor via MSPx protocol output
 *
 */

/*
Since the is no official MSP2 nor MSP1 message definition we choose for time being that
the IDs to be compatible with INAV as see here
https://github.com/iNavFlight/inav/blob/master/src/main/msp/msp_protocol_v2_sensor.h
https://github.com/iNavFlight/inav/wiki/MSP-V2
*/

#ifndef MATEKSYS_3901_L0X_H
#define MATEKSYS_3901_L0X_H

#include "std.h"
#include "stdbool.h"
//#include "filters/median_filter.h" //Who knows we need it ;)

#define MSP2_IS_SENSOR_MESSAGE(x)   ((x) >= 0x1F00 && (x) <= 0x1FFF)

#define MSP2_SENSOR_RANGEFINDER     0x1F01
#define MSP2_SENSOR_OPTIC_FLOW      0x1F02

enum Mateksys3901l0XParseStatus {
  MATEKSYS_3901_L0X_INITIALIZE,               // initialization
  MATEKSYS_3901_L0X_PARSE_HEAD,               // head of MSP is $ for all: 24
  MATEKSYS_3901_L0X_PARSE_HEAD2,              // if MSP2 then X: 4d 
  MATEKSYS_3901_L0X_PARSE_DIRECTION,          // MSP direction/status flag
  MATEKSYS_3901_L0X_PARSE_LENGTHV1,           // payload size for MSP1 = MSP2 size + 6
  MATEKSYS_3901_L0X_PARSE_MESSAGETYPEV1,      // fixed message type for V1: 0xFF
  MATEKSYS_3901_L0X_PARSE_FLAGISV2,           // flag signalling embedded v2 message
  MATEKSYS_3901_L0X_PARSE_MESSAGETYPEV2_B1,   // first bit of sensor message
  MATEKSYS_3901_L0X_PARSE_MESSAGETYPEV2_B2,   // second bit of sensor message
  MATEKSYS_3901_L0X_PARSE_LENGTHV2_B1,        // first bit of payload size v2
  MATEKSYS_3901_L0X_PARSE_LENGTHV2_B2,        // second bit of payload size v2
  MATEKSYS_3901_L0X_PARSE_MOTIONQUALITY,
  MATEKSYS_3901_L0X_PARSE_MOTIONX_B1,
  MATEKSYS_3901_L0X_PARSE_MOTIONX_B2,
  MATEKSYS_3901_L0X_PARSE_MOTIONX_B3,
  MATEKSYS_3901_L0X_PARSE_MOTIONX_B4,
  MATEKSYS_3901_L0X_PARSE_MOTIONY_B1,
  MATEKSYS_3901_L0X_PARSE_MOTIONY_B2,
  MATEKSYS_3901_L0X_PARSE_MOTIONY_B3,
  MATEKSYS_3901_L0X_PARSE_MOTIONY_B4,
  MATEKSYS_3901_L0X_PARSE_DISTANCEQUALITY,
  MATEKSYS_3901_L0X_PARSE_DISTANCE_B1,
  MATEKSYS_3901_L0X_PARSE_DISTANCE_B2,
  MATEKSYS_3901_L0X_PARSE_DISTANCE_B3,
  MATEKSYS_3901_L0X_PARSE_DISTANCE_B4,
  MATEKSYS_3901_L0X_PARSE_CHECKSUM_V2,        // checksum closing for embedded v2 message
  MATEKSYS_3901_L0X_PARSE_CHECKSUM_V1         // checksum closing for v1 message
};

struct Mateksys3901l0X {
  struct link_device *device;
  enum Mateksys3901l0XParseStatus parse_status;
	uint8_t  motion_quality;
  int32_t  motionX;
  int32_t  motionY;
	uint8_t  distancemm_quality;
	int32_t  distancemm;// in inav implementation set negative value for out of range, what a waste of bytes but we cannot have it all for now  ;)
  float    distance; // [m]
  bool     update_agl;
  bool     compensate_rotation;
	uint8_t  parse_crc;
  uint8_t  debug_list[3]; 
};

extern struct Mateksys3901l0X mateksys3901l0x;

extern void mateksys3901l0x_init(void);
extern void mateksys3901l0x_event(void);
extern void mateksys3901l0x_downlink(void);

#endif /* MATEKSYS_3901_L0X_H */

