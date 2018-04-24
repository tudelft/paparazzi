/*
 * Copyright (C) Kevin van Hecke
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
 * @file "modules/odroid_outback/odroid_outback.c"
 * @author Kevin van Hecke
 * Odroid uart (RS232) communication
 */

#include "modules/odroid_outback/odroid_outback.h"

//#include "modules/telemetry/telemetry_intermcu_ap.h"
#include "pprzlink/pprz_transport.h"
#include "pprzlink/intermcu_msg.h"
#include "mcu_periph/uart.h"
#include "subsystems/abi.h"
#include "subsystems/imu.h"
#include "state.h"
#include "subsystems/navigation/waypoints.h"
#include "firmwares/rotorcraft/navigation.h"
#include "subsystems/gps.h"

#include "generated/flight_plan.h"

/* Main magneto structure */
static struct odroid_outback_t odroid_outback = {
  .device = (&((ODROID_OUTBACK_PORT).device)),
  .msg_available = false
};
static uint8_t mp_msg_buf[128]  __attribute__((aligned));   ///< The message buffer for the Odroid

struct  Vision2PPRZPackage v2p_package;
bool odroid_outback_enable_landing = false;
bool odroid_outback_enable_spotsearch = false;
bool odroid_outback_enable_findjoe = false;
bool odroid_outback_enable_opticflow = false;
bool odroid_outback_enable_attcalib = false;
bool odroid_outback_enable_videorecord = false;
bool het_moment = false;
bool vision_timeout = false;

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"

uint8_t timeoutcount = 0;

static void send_odroid_outback( struct transport_tx *trans, struct link_device *dev)
{

  //  //fix rotated orientation of camera in DelftaCopter
  pprz_msg_send_VISION_OUTBACK(trans, dev, AC_ID,
                          &v2p_package.status,
                          (uint8_t *)&het_moment,
                          &timeoutcount,
                          (uint8_t *)&vision_timeout,
                          &v2p_package.height,
                          &v2p_package.out_of_range_since,
                          &v2p_package.marker_enu_x,
                          &v2p_package.marker_enu_y,
                          &v2p_package.flow_x,
                          &v2p_package.flow_y);
}
#endif



/* Initialize the Odroid */
void odroid_outback_init() {
  // Initialize transport protocol
  pprz_transport_init(&odroid_outback.transport);


#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_VISION_OUTBACK, send_odroid_outback);
#endif

  NavSetWaypointHere(WP_dummy); //WP_ODROID_OUTBACK_LANDING
  v2p_package.height = -0.01;
  v2p_package.status = 1;
  vision_timeout = false;
  timeoutcount = ODROID_OUTBACK_PERIODIC_FREQ / 2;

}

/* Parse the InterMCU message */
static inline void odroid_outback_parse_msg(void)
{

  /* Parse the odroid_outback message */
  uint8_t msg_id = mp_msg_buf[1];

  switch (msg_id) {

    /* Received a part of a thumbnail: forward to 900MHz and irridium... */
    case DL_IMCU_PAYLOAD:

      ////////////////////////////////////
      // Forward to 900MHz (or XBee)
//      uint8_t size = DL_IMCU_PAYLOAD_data_length(mp_msg_buf);
//      uint8_t *data = DL_IMCU_PAYLOAD_data(mp_msg_buf);

//      DOWNLINK_SEND_PAYLOAD(DefaultChannel, DefaultDevice, size, data);

      ////////////////////////////////////
      // Forward to FBW with IRRIDIUM
      //pprz_msg_send_PAYLOAD(&(telemetry_intermcu.trans.trans_tx), &telemetry_intermcu.dev, AC_ID, size, data);

      break;

      /* Got a odroid_outback message */
    case DL_IMCU_DEBUG: {
        uint8_t size = DL_IMCU_DEBUG_msg_length(mp_msg_buf);
        uint8_t *msg = DL_IMCU_DEBUG_msg(mp_msg_buf);

        unsigned char * tmp = (unsigned char*)&v2p_package;
        for(uint8_t i = 0; i < size; i++) {
            tmp[i] = msg[i];
          }
        timeoutcount = ODROID_OUTBACK_PERIODIC_FREQ / 2;
        vision_timeout = false;
        static int32_t frame_id_prev = 0;
        if (frame_id_prev >= v2p_package.frame_id) {
            timeoutcount = 0;
          }
        frame_id_prev = v2p_package.frame_id;

        //struct EnuCoor_f *pos = stateGetPositionEnu_f();

        //float diff_search = (odroid_outback_search_height - k2p_package.height)*odroid_outback_height_gain;

        if (odroid_outback_enable_spotsearch) {
            // WP_ODROID_OUTBACK_LANDSPOT
            waypoint_set_xy_i(WP_dummy, POS_BFP_OF_REAL(v2p_package.land_enu_x), POS_BFP_OF_REAL(v2p_package.land_enu_y));
          }

        if (odroid_outback_enable_landing) {
            if ((v2p_package.out_of_range_since > 0 && v2p_package.out_of_range_since < 1.f) || (v2p_package.out_of_range_since < 0 && v2p_package.height < 0.3 )) {
                het_moment = true;
              } else {
                het_moment = false;
              }
          } else {
            het_moment = false;
          }

        if (odroid_outback_enable_findjoe) {
            waypoint_set_xy_i(WP_dummy, POS_BFP_OF_REAL(v2p_package.marker_enu_x), POS_BFP_OF_REAL(v2p_package.marker_enu_y)); // WP_ODROID_OUTBACK_JOE

            uint8_t wp_id = WP_dummy; //WP_ODROID_OUTBACK_JOE;
            RunOnceEvery(60, DOWNLINK_SEND_WP_MOVED_ENU(DefaultChannel, DefaultDevice, &wp_id,&(waypoints[wp_id].enu_i.x),
                                                        &(waypoints[wp_id].enu_i.y), &(waypoints[wp_id].enu_i.z)));
          }

        // Send ABI message
        if (timeoutcount > 0) {
          AbiSendMsgAGL(AGL_SONAR_ADC_ID, v2p_package.height);
        }

        break;
      }
    default:
      break;
    }
}

/* We need to wait for incomming messages */
void odroid_outback_event() {
  // Check if we got some message from the Odroid
  pprz_check_and_parse(odroid_outback.device, &odroid_outback.transport, mp_msg_buf, &odroid_outback.msg_available);

  // If we have a message we should parse it
  if (odroid_outback.msg_available) {
      odroid_outback_parse_msg();
      odroid_outback.msg_available = false;
    }
}

void odroid_outback_periodic() {

  struct FloatEulers *attE = stateGetNedToBodyEulers_f();
  struct FloatQuat *att = stateGetNedToBodyQuat_f();
  struct EnuCoor_f *pos = stateGetPositionEnu_f();


  struct PPRZ2VisionPackage p2k_package;
  p2k_package.phi = attE->theta;
  p2k_package.theta = -attE->phi;
  p2k_package.psi = attE->psi;
  p2k_package.qi = att->qi;
  p2k_package.qx = att->qx;
  p2k_package.qy = att->qy;
  p2k_package.qz = att->qz;
  p2k_package.gpsx = pos->x;
  p2k_package.gpsy = pos->y;
  p2k_package.gpsz = pos->z;

  if (state.ned_initialized_f) {
      p2k_package.geo_init_gpsx = state.ned_origin_f.lla.lat;
      p2k_package.geo_init_gpsy = state.ned_origin_f.lla.lon;
      p2k_package.geo_init_gpsz = state.ned_origin_f.lla.alt;
    } else {
      p2k_package.geo_init_gpsx = 0;
      p2k_package.geo_init_gpsy = 0;
      p2k_package.geo_init_gpsz = 0;
    }



  p2k_package.enables = 0;
  if (odroid_outback_enable_landing)
    p2k_package.enables |= 0b1;
  if (odroid_outback_enable_spotsearch)
    p2k_package.enables |= 0b10;
  if (odroid_outback_enable_findjoe)
    p2k_package.enables |= 0b100;
  if (odroid_outback_enable_opticflow)
    p2k_package.enables |= 0b1000;
  if (odroid_outback_enable_attcalib)
    p2k_package.enables |= 0b10000;
  if (odroid_outback_enable_videorecord)
    p2k_package.enables |= 0b100000;

  if (timeoutcount > 0) {
      timeoutcount--;
    } else {
      v2p_package.status = 1;      
    }
  if (v2p_package.status != 0) {
      vision_timeout = true;
  }

  pprz_msg_send_IMCU_DEBUG(&(odroid_outback.transport.trans_tx), odroid_outback.device,
                           1, sizeof(struct PPRZ2VisionPackage), (unsigned char *)(&p2k_package));
}

void enableOdroidLandingspotSearch(bool b) {
  odroid_outback_enable_spotsearch = b;
}

void enableOdroidDescent(bool b) {
  odroid_outback_enable_landing = b;
}

void enableOdroidFindJoe(bool b) {
  odroid_outback_enable_findjoe = b;
}

void enableOdroidOpticFlow(bool b) {
  odroid_outback_enable_opticflow = b;
}

bool enableOdroidAttCalib(bool b) {
  //http://mariotapilouw.blogspot.com/2011/05/plane-fitting-using-opencv.html
  odroid_outback_enable_attcalib = b;
  return true; // klote pprz flight plan
}

bool enableOdroidVideoRecord(bool b) {
  odroid_outback_enable_videorecord = b;
  return true; // klote pprz flight plan
}

bool getOdroidReady(void) {
  return v2p_package.status == 0;
}
