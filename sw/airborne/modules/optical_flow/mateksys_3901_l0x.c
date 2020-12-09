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

struct Mateksys3901l0X mateksys3901l0x = {
  .parse_status = MATEKSYS_3901_L0X_INITIALIZE
};

static void mateksys3901l0x_parse(uint8_t byte);

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"

/**
 * Downlink message for debug
 */
// void mateksys3901l0x_downlink(void)
// {
//   DOWNLINK_SEND_MATEKFLOW(DefaultChannel, DefaultDevice, 
//                         &mateksys3901l0x.motion_quality,
//                         &mateksys3901l0x.motionX,
//                         &mateksys3901l0x.motionY,
//                         &mateksys3901l0x.distancemm_quality,
//                         &mateksys3901l0x.distancemm,
//                         &mateksys3901l0x.distance,
//                         &mateksys3901l0x.update_agl, 
//                         &mateksys3901l0x.compensate_rotation,
//                         &mateksys3901l0x.parse_crc
//                         );
// }

/**
 * Downlink message lidar
 */
static void mateksys3901l0x_send_lidar(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_MATEK(trans, dev, AC_ID,
                      &mateksys3901l0x.id1,
                      &mateksys3901l0x.id2,
                      &mateksys3901l0x.data1,
                      &mateksys3901l0x.data2,
                      &mateksys3901l0x.data3,
                      &mateksys3901l0x.data4,
                      &mateksys3901l0x.data5,
                      &mateksys3901l0x.data6,
                      &mateksys3901l0x.data7,
                      &mateksys3901l0x.data8,
                      &mateksys3901l0x.data9,
                      &mateksys3901l0x.data10,
                      &mateksys3901l0x.data11,
                      &mateksys3901l0x.data12,
                      &mateksys3901l0x.data13,
                      &mateksys3901l0x.data14,
                      &mateksys3901l0x.data15,
                      &mateksys3901l0x.data16);
}

#endif

/**
 * Initialization function
 */
void mateksys3901l0x_init(void)
{
  mateksys3901l0x.device = &((MATEKSYS_3901_L0X_PORT).device);
	mateksys3901l0x.motion_quality = 0;                                             //TODO :is this a good choice?
  mateksys3901l0x.motionX = 0;                                                    //TODO :is this a good choice?
  mateksys3901l0x.motionY = 0;                                                    //TODO :is this a good choice?
  mateksys3901l0x.distancemm_quality = 0;
	mateksys3901l0x.distancemm= -1;                                                 //TODO :is this a good choice?
  mateksys3901l0x.distance = 0;   
  mateksys3901l0x.id1 = 0;
  mateksys3901l0x.id2 = 0;
  mateksys3901l0x.data1 = 0;
  mateksys3901l0x.data2 = 0;
  mateksys3901l0x.data3 = 0;
  mateksys3901l0x.data4 = 0;
  mateksys3901l0x.data5 = 0;
  mateksys3901l0x.data6 = 0;
  mateksys3901l0x.data7 = 0;
  mateksys3901l0x.data8 = 0;
  mateksys3901l0x.data9 = 0;
  mateksys3901l0x.data10 = 0;
  mateksys3901l0x.data11 = 0;
  mateksys3901l0x.data12 = 0;
  mateksys3901l0x.data13 = 0;
  mateksys3901l0x.data14 = 0;
  mateksys3901l0x.data15 = 0;
  mateksys3901l0x.data16 = 0;




	mateksys3901l0x.update_agl = USE_MATEKSYS_3901_L0X_AGL;
  mateksys3901l0x.compensate_rotation = MATEKSYS_3901_L0X_COMPENSATE_ROTATION;
  mateksys3901l0x.parse_status = MATEKSYS_3901_L0X_PARSE_HEAD;

#if PERIODIC_TELEMETRY
	register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_LIDAR, mateksys3901l0x_send_lidar);
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

    case MATEKSYS_3901_L0X_PARSE_HEAD: // OK
      if (byte == 0x24) {
        mateksys3901l0x.parse_status++;
      }
      break;

    case MATEKSYS_3901_L0X_PARSE_HEAD2: // OK
      if (byte == 0x58) {
        mateksys3901l0x.parse_status++;
      } else {
        mateksys3901l0x.parse_status = MATEKSYS_3901_L0X_PARSE_HEAD;
      }
      break;
    
    case MATEKSYS_3901_L0X_PARSE_DIRECTION: // OK (3C IS REQUEST) / (3E IS RESPONSE)
      if (byte == 0x3C) {
        mateksys3901l0x.parse_status++;
      } else {
        mateksys3901l0x.parse_status = MATEKSYS_3901_L0X_PARSE_HEAD;
      }
      break;

    case MATEKSYS_3901_L0X_PARSE_LENGTHV1: // OK
      if (byte == 0x00) {
        mateksys3901l0x.parse_status++;
      } else {
        mateksys3901l0x.parse_status = MATEKSYS_3901_L0X_PARSE_HEAD;
      }
      break;

    case MATEKSYS_3901_L0X_PARSE_FUNCTION_ID_B1: // OK this is 01 or 02 depending wether is lidar or optic flow (function ID)
      if (byte == 0x02) { 
        mateksys3901l0x.id1 = byte;
        mateksys3901l0x.parse_status++;
      } else {
        mateksys3901l0x.parse_status = MATEKSYS_3901_L0X_PARSE_HEAD;
      }
      break;
    
    case MATEKSYS_3901_L0X_PARSE_FUNCTION_ID_B2: // OK this is the common placeholder for sensortype function ID
      if (byte == 0x1F) { 
        mateksys3901l0x.id2 = byte;
        mateksys3901l0x.parse_status++;
      } else {
        mateksys3901l0x.parse_status = MATEKSYS_3901_L0X_PARSE_HEAD;
      }
      break;

    case MATEKSYS_3901_L0X_PARSE_DATA_1: 
      if (byte >= 0x00) { 
        mateksys3901l0x.data1 = byte;
        mateksys3901l0x.parse_status++;
      } else {
        mateksys3901l0x.parse_status = MATEKSYS_3901_L0X_PARSE_HEAD;
      }
      break;

    case MATEKSYS_3901_L0X_PARSE_DATA_2: 
      if (byte >= 0x00) { 
        mateksys3901l0x.data2 = byte;
        mateksys3901l0x.parse_status++;
      } else {
        mateksys3901l0x.parse_status = MATEKSYS_3901_L0X_PARSE_HEAD;
      }
      break;
    
    case MATEKSYS_3901_L0X_PARSE_DATA_3: 
      if (byte >= 0x00) { 
        mateksys3901l0x.data3 = byte;
        mateksys3901l0x.parse_status++;
      } else {
        mateksys3901l0x.parse_status = MATEKSYS_3901_L0X_PARSE_HEAD;
      }
      break;
    
    case MATEKSYS_3901_L0X_PARSE_DATA_4: 
      if (byte >= 0x00) { 
        mateksys3901l0x.data4 = byte;
        mateksys3901l0x.parse_status++;
      } else {
        mateksys3901l0x.parse_status = MATEKSYS_3901_L0X_PARSE_HEAD;
      }
      break;

    case MATEKSYS_3901_L0X_PARSE_DATA_5: 
      if (byte >= 0x00) { 
        mateksys3901l0x.data5 = byte;
        mateksys3901l0x.parse_status++;
      } else {
        mateksys3901l0x.parse_status = MATEKSYS_3901_L0X_PARSE_HEAD;
      }
      break;

    case MATEKSYS_3901_L0X_PARSE_DATA_6: 
      if (byte >= 0x00) { 
        mateksys3901l0x.data6 = byte;
        mateksys3901l0x.parse_status++;
      } else {
        mateksys3901l0x.parse_status = MATEKSYS_3901_L0X_PARSE_HEAD;
      }
      break;
    
    case MATEKSYS_3901_L0X_PARSE_DATA_7: 
      if (byte >= 0x00) { 
        mateksys3901l0x.data7 = byte;
        mateksys3901l0x.parse_status++;
      } else {
        mateksys3901l0x.parse_status = MATEKSYS_3901_L0X_PARSE_HEAD;
      }
      break;

    case MATEKSYS_3901_L0X_PARSE_DATA_8: 
      if (byte >= 0x00) { 
        mateksys3901l0x.data8 = byte;
        mateksys3901l0x.parse_status++;
      } else {
        mateksys3901l0x.parse_status = MATEKSYS_3901_L0X_PARSE_HEAD;
      }
      break;

    case MATEKSYS_3901_L0X_PARSE_DATA_9: 
      if (byte >= 0x00) { 
        mateksys3901l0x.data9 = byte;
        mateksys3901l0x.parse_status++;
      } else {
        mateksys3901l0x.parse_status = MATEKSYS_3901_L0X_PARSE_HEAD;
      }
      break;
    
    case MATEKSYS_3901_L0X_PARSE_DATA_10: 
      if (byte >= 0x00) { 
        mateksys3901l0x.data10 = byte;
        mateksys3901l0x.parse_status++;
      } else {
        mateksys3901l0x.parse_status = MATEKSYS_3901_L0X_PARSE_HEAD;
      }
      break;

    case MATEKSYS_3901_L0X_PARSE_DATA_11: 
      if (byte >= 0x00) { 
        mateksys3901l0x.data11 = byte;
        mateksys3901l0x.parse_status++;
      } else {
        mateksys3901l0x.parse_status = MATEKSYS_3901_L0X_PARSE_HEAD;
      }
      break;

    case MATEKSYS_3901_L0X_PARSE_DATA_12: 
      if (byte >= 0x00) { 
        mateksys3901l0x.data12 = byte;
        mateksys3901l0x.parse_status++;
      } else {
        mateksys3901l0x.parse_status = MATEKSYS_3901_L0X_PARSE_HEAD;
      }
      break;
    
    case MATEKSYS_3901_L0X_PARSE_DATA_13: 
      if (byte >= 0x00) { 
        mateksys3901l0x.data13 = byte;
        mateksys3901l0x.parse_status++;
      } else {
        mateksys3901l0x.parse_status = MATEKSYS_3901_L0X_PARSE_HEAD;
      }
      break;

    case MATEKSYS_3901_L0X_PARSE_DATA_14: 
      if (byte >= 0x00) { 
        mateksys3901l0x.data14 = byte;
        mateksys3901l0x.parse_status++;
      } else {
        mateksys3901l0x.parse_status = MATEKSYS_3901_L0X_PARSE_HEAD;
      }
      break;
















    default:
      // Error, return to start
      mateksys3901l0x.parse_status = MATEKSYS_3901_L0X_PARSE_HEAD;
      break;


// //We are lazy ATM ;) for the 1st exaple...refactor plz to a better handler see e.g alhpa_ESC code
//     case MATEKSYS_3901_L0X_PARSE_MOTIONQUALITY:
//       mateksys3901l0x.motionX = byte;
//       mateksys3901l0x.parse_crc += byte;
//       mateksys3901l0x.parse_status++;
//       break;

//     case MATEKSYS_3901_L0X_PARSE_MOTIONX_B1:
//       mateksys3901l0x.motionX = byte;
//       mateksys3901l0x.parse_crc += byte;
//       mateksys3901l0x.parse_status++;
//       break;

//     case MATEKSYS_3901_L0X_PARSE_MOTIONX_B2:
//       mateksys3901l0x.motionX |= (byte << 8);
//       mateksys3901l0x.parse_crc += byte;
//       mateksys3901l0x.parse_status++;
//       break;

// 		case MATEKSYS_3901_L0X_PARSE_MOTIONX_B3:
//       mateksys3901l0x.motionX |= (byte << 8);//fixme
//       mateksys3901l0x.parse_crc += byte;
//       mateksys3901l0x.parse_status++;
//       break;
		
// 		case MATEKSYS_3901_L0X_PARSE_MOTIONX_B4:
//       mateksys3901l0x.motionX |= (byte << 8);//fxme
//       mateksys3901l0x.parse_crc += byte;
//       mateksys3901l0x.parse_status++;
//     break;

// 	  case MATEKSYS_3901_L0X_PARSE_MOTIONY_B1:
//       mateksys3901l0x.raw_flowy = byte;
//       mateksys3901l0x.parse_crc += byte;
//       mateksys3901l0x.parse_status++;
//       break;
//     case MATEKSYS_3901_L0X_PARSE_MOTIONY_B2:
//       mateksys3901l0x.motionY |= (byte << 8);
//       mateksys3901l0x.parse_crc += byte;
//       mateksys3901l0x.parse_status++;
//       break;
// 		case MATEKSYS_3901_L0X_PARSE_MOTIONY_B3:
//       mateksys3901l0x.motionY |= (byte << 8);//Fixme
//       mateksys3901l0x.parse_crc += byte;
//       mateksys3901l0x.parse_status++;
//       break;
//     case MATEKSYS_3901_L0X_PARSE_MOTIONY_B4:
//       mateksys3901l0x.motionY |= (byte << 8);//Fixme
//       mateksys3901l0x.parse_crc += byte;
//       mateksys3901l0x.parse_status++;
//       break;

//     case MATEKSYS_3901_L0X_PARSE_DISTANCEQUALITY:
//       mateksys3901l0x.raw_distancemm = byte;
//       mateksys3901l0x.parse_crc += byte;
//       mateksys3901l0x.parse_status++;
//       break;
//     case MATEKSYS_3901_L0X_PARSE_DISTANCE_B1:
//       mateksys3901l0x.raw_distancemm = byte;
//       mateksys3901l0x.parse_crc += byte;
//       mateksys3901l0x.parse_status++;
//       break;
//     case MATEKSYS_3901_L0X_PARSE_DISTANCE_B2:
//       mateksys3901l0x.distancemm |= (byte << 8);
//       mateksys3901l0x.parse_crc += byte;
//       mateksys3901l0x.parse_status++;
//       break;
// 		case MATEKSYS_3901_L0X_PARSE_DISTANCE_B3:
//       mateksys3901l0x.distancemm |= (byte << 8);//Fixme
//       mateksys3901l0x.parse_crc += byte;
//       mateksys3901l0x.parse_status++;
//       break;
//     case MATEKSYS_3901_L0X_PARSE_DISTANCE_B4:
//       mateksys3901l0x.distancemm |= (byte << 8);//Fixme
//       mateksys3901l0x.parse_crc += byte;
//       mateksys3901l0x.parse_status++;
//       break;

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
