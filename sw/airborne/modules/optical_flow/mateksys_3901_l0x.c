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
 *  @brief Driver for the mateksys_3901_l0x sensor via MSP protocol output
 *
 */

#include "mateksys_3901_l0x.h"
#include "mcu_periph/uart.h"
#include "subsystems/abi.h"

// State interface for rotation compensation
#include "state.h"

// Messages
#include "pprzlink/messages.h"
#include "subsystems/datalink/downlink.h"


// Define configuration parameters
#ifndef MATEKSYS_3901_L0X_MOTION_THRES
#define MATEKSYS_3901_L0X_MOTION_THRES
#endif

#ifndef MATEKSYS_3901_L0X_DISTANCE_THRES
#define MATEKSYS_3901_L0X_DISTANCE_THRES
#endif

#ifndef USE_MATEKSYS_3901_L0X_AGL
#define USE_MATEKSYS_3901_L0X_AGL
#endif

#ifndef MATEKSYS_3901_L0X_COMPENSATE_ROTATION
#define MATEKSYS_3901_L0X_COMPENSATE_ROTATION
#endif

struct Mateksys3901l0X mateksys3901l0x = {
  .parse_status = MATEKSYS_3901_L0X_INITIALIZE
};

static void mateksys3901l0x_parse(uint8_t byte);

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"

/**
 * Downlink message flow and lidar (included velocity estimation, not yet tested)
 */
static void mateksys3901l0x_send_matek(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_MATEKSYS_FLOW_LIDAR(trans, dev, AC_ID,
                                    &mateksys3901l0x.sensor_id,
                                    &mateksys3901l0x.motion_quality,
                                    &mateksys3901l0x.motionX,
                                    &mateksys3901l0x.motionY,
                                    &mateksys3901l0x.distancemm_quality,
                                    &mateksys3901l0x.distancemm,
                                    &mateksys3901l0x.velocityX,
                                    &mateksys3901l0x.velocityY);
}

#endif

/**
 * Initialization function
 */
void mateksys3901l0x_init(void)
{
  mateksys3901l0x.device = &((MATEKSYS_3901_L0X_PORT).device);
  mateksys3901l0x.parse_crc = 0;
	mateksys3901l0x.motion_quality = 0;                                             
  mateksys3901l0x.motionX = 0;                                                    
  mateksys3901l0x.motionY = 0;                                                    
  mateksys3901l0x.distancemm_quality = 0;
	mateksys3901l0x.distancemm = -1;                                                   
  mateksys3901l0x.velocityX = 0;
  mateksys3901l0x.velocityY = 0;
  mateksys3901l0x.parse_status = MATEKSYS_3901_L0X_PARSE_HEAD;

#if PERIODIC_TELEMETRY
	register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_MATEKSYS_FLOW_LIDAR, mateksys3901l0x_send_matek);
#endif
}

/**
 * Receive bytes from the UART port and parse them
 */
void mateksys3901l0x_event(void)
{
  while (mateksys3901l0x.parse_status != MATEKSYS_3901_L0X_INITIALIZE && mateksys3901l0x.device->char_available(mateksys3901l0x.device->periph)) {
    mateksys3901l0x_parse(mateksys3901l0x.device->get_byte(mateksys3901l0x.device->periph));
  }
}

/**
 * Parse the sensor MSP output bytes 1 by 1
 */
static void mateksys3901l0x_parse(uint8_t byte)
{

  switch (mateksys3901l0x.parse_status) {
    case MATEKSYS_3901_L0X_INITIALIZE:
      break;

    case MATEKSYS_3901_L0X_PARSE_HEAD: // MSP general header $
      if (byte == 0x24) {
        mateksys3901l0x.parse_status++;
      }
      break;

    case MATEKSYS_3901_L0X_PARSE_HEAD2: // MSPv2 identifier X
      if (byte == 0x58) {
        mateksys3901l0x.parse_status++;
      } else {
        mateksys3901l0x.parse_status = MATEKSYS_3901_L0X_PARSE_HEAD;
      }
      break;
    
    case MATEKSYS_3901_L0X_PARSE_DIRECTION: // direction <
      if (byte == 0x3C) {
        mateksys3901l0x.parse_status++;
      } else {
        mateksys3901l0x.parse_status = MATEKSYS_3901_L0X_PARSE_HEAD;
      }
      break;

    case MATEKSYS_3901_L0X_PARSE_LENGTH: // set to 0
      if (byte == 0x00) {
        mateksys3901l0x.parse_crc += byte;
        mateksys3901l0x.parse_status++;
      } else {
        mateksys3901l0x.parse_status = MATEKSYS_3901_L0X_PARSE_HEAD;
      }
      break;

    case MATEKSYS_3901_L0X_PARSE_FUNCTION_ID_B1: // 0x01 = rangefinder; 0x02 = opticalflow
      if (byte == 0x01 || byte == 0x02) { 
        mateksys3901l0x.sensor_id = byte;
        mateksys3901l0x.parse_crc += byte;
        mateksys3901l0x.parse_status++;
      } else {
        mateksys3901l0x.parse_status = MATEKSYS_3901_L0X_PARSE_HEAD;
      }
      break;
    
    case MATEKSYS_3901_L0X_PARSE_FUNCTION_ID_B2: // sensor id pointer
      if (byte == 0x1F) { 
        mateksys3901l0x.parse_status++;
        mateksys3901l0x.parse_crc += byte;
      } else {
        mateksys3901l0x.parse_status = MATEKSYS_3901_L0X_PARSE_HEAD;
      }
      break;

    case MATEKSYS_3901_L0X_PARSE_SIZE: 
      if (byte == 0x05 || byte == 0x09) { 
        mateksys3901l0x.parse_status++;
        mateksys3901l0x.parse_crc += byte;
      } else {
        mateksys3901l0x.parse_status = MATEKSYS_3901_L0X_PARSE_HEAD;
      }
      break;

    case MATEKSYS_3901_L0X_PARSE_POINTER:  // should be zero, used to redirect to motion parsing or lidar parsing
      if (mateksys3901l0x.sensor_id == 0x01) { 
        mateksys3901l0x.parse_status = MATEKSYS_3901_L0X_PARSE_DISTANCEQUALITY;
      } else if (mateksys3901l0x.sensor_id == 0x02) {
        mateksys3901l0x.parse_status = MATEKSYS_3901_L0X_PARSE_MOTIONQUALITY;
      } else {
        mateksys3901l0x.parse_status = MATEKSYS_3901_L0X_PARSE_HEAD;
      }
      break;

    // rangefinder data parsing
    case MATEKSYS_3901_L0X_PARSE_DISTANCEQUALITY:
      mateksys3901l0x.distancemm_quality = byte;
      mateksys3901l0x.parse_crc += byte;
      mateksys3901l0x.parse_status++;
      break;

    case MATEKSYS_3901_L0X_PARSE_DISTANCE_B1:
      if (mateksys3901l0x.distancemm_quality <= MATEKSYS_3901_L0X_DISTANCE_THRES) {
        mateksys3901l0x.parse_status = MATEKSYS_3901_L0X_PARSE_HEAD;
      } else {
        mateksys3901l0x.distancemm = byte;
        mateksys3901l0x.parse_crc += byte;
        mateksys3901l0x.parse_status++;
      }
      break;

    case MATEKSYS_3901_L0X_PARSE_DISTANCE_B2:
      mateksys3901l0x.distancemm |= (byte << 8);
      mateksys3901l0x.parse_crc += byte;
      mateksys3901l0x.parse_status++;
      break;

		case MATEKSYS_3901_L0X_PARSE_DISTANCE_B3:
      mateksys3901l0x.distancemm |= (byte << 16);
      mateksys3901l0x.parse_crc += byte;
      mateksys3901l0x.parse_status++;
      break;

    case MATEKSYS_3901_L0X_PARSE_DISTANCE_B4:
      mateksys3901l0x.distancemm |= (byte << 24);
      mateksys3901l0x.parse_crc += byte;
      mateksys3901l0x.parse_status = MATEKSYS_3901_L0X_PARSE_CHECKSUM;
      break;

    // optical flow data parsing
    case MATEKSYS_3901_L0X_PARSE_MOTIONQUALITY:
      mateksys3901l0x.motion_quality = byte;
      mateksys3901l0x.parse_crc += byte;
      mateksys3901l0x.parse_status++;
      break;

	  case MATEKSYS_3901_L0X_PARSE_MOTIONY_B1:
      if (mateksys3901l0x.motion_quality <= MATEKSYS_3901_L0X_MOTION_THRES) {
        mateksys3901l0x.parse_status = MATEKSYS_3901_L0X_PARSE_HEAD;
      } else {
        mateksys3901l0x.motionY = byte;
        mateksys3901l0x.parse_crc += byte;
        mateksys3901l0x.parse_status++;
      }
      break;

    case MATEKSYS_3901_L0X_PARSE_MOTIONY_B2:
      mateksys3901l0x.motionY |= (byte << 8);
      mateksys3901l0x.parse_crc += byte;
      mateksys3901l0x.parse_status++;
      break;

		case MATEKSYS_3901_L0X_PARSE_MOTIONY_B3:
      mateksys3901l0x.motionY |= (byte << 16);
      mateksys3901l0x.parse_crc += byte;
      mateksys3901l0x.parse_status++;
      break;

    case MATEKSYS_3901_L0X_PARSE_MOTIONY_B4:
      mateksys3901l0x.motionY |= (byte << 24);
      mateksys3901l0x.velocityY = mateksys3901l0x.distancemm * sin(mateksys3901l0x.motionY*(M_PI/180));
      mateksys3901l0x.parse_crc += byte;
      mateksys3901l0x.parse_status++;
      break;

    case MATEKSYS_3901_L0X_PARSE_MOTIONX_B1:
      mateksys3901l0x.motionX = byte;
      mateksys3901l0x.parse_crc += byte;
      mateksys3901l0x.parse_status++;
      break;

    case MATEKSYS_3901_L0X_PARSE_MOTIONX_B2:
      mateksys3901l0x.motionX |= (byte << 8);
      mateksys3901l0x.parse_crc += byte;
      mateksys3901l0x.parse_status++;
      break;

		case MATEKSYS_3901_L0X_PARSE_MOTIONX_B3:
      mateksys3901l0x.motionX |= (byte << 16);
      mateksys3901l0x.parse_crc += byte;
      mateksys3901l0x.parse_status++;
      break;
		
		case MATEKSYS_3901_L0X_PARSE_MOTIONX_B4:
      mateksys3901l0x.motionX |= (byte << 24);
      mateksys3901l0x.velocityX = mateksys3901l0x.distancemm * sin(mateksys3901l0x.motionX*(M_PI/180));
      mateksys3901l0x.parse_crc += byte;
      mateksys3901l0x.parse_status++;
      break;
    
    case MATEKSYS_3901_L0X_PARSE_CHECKSUM:
      mateksys3901l0x.parse_status = MATEKSYS_3901_L0X_PARSE_HEAD;
      break;

    default:
      // Error, return to start
      mateksys3901l0x.parse_status = MATEKSYS_3901_L0X_PARSE_HEAD;
      break;

//     case MATEKSYS_3901_L0X_PARSE_CHECKSUM:
//       // When the CRC matches we can do stuff, jippy
//       if (mateksys3901l0x.parse_crc == byte) {
//         uint32_t now_ts = get_sys_time_usec();
//         mateksys3901l0x.distance = mateksys3901l0x.distance / 100.f;
//         mateksys3901l0x.quality = 222;  //TODO:remove after ebugging

//         // When the distance is valid
//         if (mateksys3901l0x.distance >=0 ) {
//           // compensate AGL measurement for body rotation
//           if (mateksys3901l0x.compensate_rotation) {
//             float phi = stateGetNedToBodyEulers_f()->phi;
//             float theta = stateGetNedToBodyEulers_f()->theta;
//             float gain = (float)fabs((double)(cosf(phi) * cosf(theta)));
//             mateksys3901l0x.distance = mateksys3901l0x.distance * gain;
//           }

//           // send message (if requested)
//           if (mateksys3901l0x.update_agl) {
//             //TODO: correct message of mis-use anothe AGL sensor ID
// 						//AbiSendMsgAGL(AGL_LIDAR_MATEKSYS_3901_L0X_ID, now_ts, mateksys3901l0x.distance);
//           }
//         }
//       // Start reading again
//       mateksys3901l0x.parse_status = MATEKSYS_3901_L0X_PARSE_HEAD;
//       break;

 }   
}
