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
 * @file "modules/vision_outback/vision_outback.c"
 * @author Kevin van Hecke
 * Vision uart (RS232) communication
 */

#include "modules/vision_outback/vision_outback.h"
#include "modules/boards/opa_controller_ap.h"

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

#include <inttypes.h>
#include <stdbool.h>
#include "pprzlink/pprzlink_transport.h"
#include "pprzlink/pprzlink_device.h"

#include "air_data/air_data.h"

#ifdef VISION_PWR_ON
#define VISION_POWER_ON_AT_BOOT false
#else
#define VISION_POWER_ON_AT_BOOT true
#endif

#define WANTED_VISION_VERSION 2.1f

/* Main magneto structure */
static struct vision_outback_t vision_outback = {
  .device = (&((VISION_OUTBACK_PORT).device)),
  .msg_available = false
};
static uint8_t mp_msg_buf[128]  __attribute__((aligned));   ///< The message buffer for the Vision
int shutdown_count;

struct  Vision2PPRZPackage v2p_package;

bool vision_outback_enable_landing = false;
bool vision_outback_enable_take_foto = false;
bool vision_outback_enable_findjoe = false;
bool vision_outback_enable_opticflow = false;
bool vision_outback_enable_attcalib = false;
bool vision_outback_enable_videorecord = false;
bool vision_outback_close_process = false;
bool vision_outback_update_system = false;
#ifdef VISION_PWR_OFF
uint8_t vision_outback_power = VISION_SETTING_STATUS_REQUEST_POWER_OFF;
#else
uint8_t vision_outback_power = VISION_SETTING_STATUS_REQUEST_POWER_ON;
#endif
bool het_moment = false;
bool vision_timeout = false; // TODO: remove
float vision_height = false; // TODO: remove
bool vision_found_joe = false;
float unfiltered_vision_height = 0; // Raw vision height not fused with gps

float msg_marker_x = 0;
float msg_marker_y = 0;
uint8_t msg_best_landing_status = 0;

//TMP for debugging weird uart crossover talk shizzle
int turbocnt = 0;
int turbosize = 0;

//TODO: make airframe file settings:
#ifdef VISION_PWR_ON // DelftaCopter...
#define DRONE_WIDTH 1.f;
#define DRONE_LENGTH 0.01f; //disabled because cam not in the middel of the pitch direction
float vision_outback_moment_height = 0.25;
#else  // IRIS
#define DRONE_WIDTH 0.2f;
#define DRONE_LENGTH 0.2f;
float vision_outback_moment_height = 0.1;
#endif

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"

uint16_t timeoutcount = 0;
void enable_wp_telemetry_updates(void);
void do_power_state_machine(void);

static void send_vision_outback( struct transport_tx *trans, struct link_device *dev)
{

  pprz_msg_send_VISION_OUTBACK(trans, dev, AC_ID,
                               &v2p_package.status,
                               &v2p_package.landing_status,
                               (uint8_t *)&het_moment,
                               (uint8_t *)&timeoutcount,
                               (uint8_t *)&vision_timeout,
                               &v2p_package.height,
                               &v2p_package.out_of_range_since,
                               &msg_marker_x,
                               &msg_marker_y,
                               &v2p_package.stupid_pprz_height, // In the message this is flow_x, but we use it for height
                               &v2p_package.version);
}
#endif



/* Initialize the Vision */
void vision_outback_init() {
  // Initialize transport protocol
  pprz_transport_init(&vision_outback.transport);

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_VISION_OUTBACK, send_vision_outback);
#endif

  v2p_package.height = -0.01;
  v2p_package.status = 1;
  timeoutcount = 0;
  enable_wp_telemetry_updates();

  msg_marker_x = WaypointX(WP_JOE);
  msg_marker_y = WaypointY(WP_JOE);
  msg_best_landing_status = ls_init;

}

/* Parse the InterMCU message */
static inline void vision_outback_parse_msg(void)
{

  /* Parse the vision_outback message */
  uint8_t msg_id = pprzlink_get_msg_id(mp_msg_buf);
  uint8_t sender_id = pprzlink_get_msg_sender_id(mp_msg_buf);

  if (sender_id != 3) {
      turbosize = sender_id;
      return;
    }
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

      /* Got a vision_outback message */
    case DL_IMCU_DEBUG: {


        uint8_t size = DL_IMCU_DEBUG_msg_length(mp_msg_buf);
        uint8_t *msg = DL_IMCU_DEBUG_msg(mp_msg_buf);

        //TMP for debugging:
        turbosize = size;
        turbocnt++;

        unsigned char * tmp = (unsigned char*)&v2p_package;

        //impossible case, but seems to happen:
        if (size != sizeof(struct Vision2PPRZPackage))
          break;

        for(uint8_t i = 0; i < size; i++) {
            tmp[i] = msg[i];
          }
        if (timeoutcount < 1*VISION_OUTBACK_PERIODIC_FREQ)
          timeoutcount = 1*VISION_OUTBACK_PERIODIC_FREQ;
        static int32_t frame_id_prev = 0;
        if (frame_id_prev == v2p_package.frame_id) { // Realsense resets id when changing from stereo to color. So, only check if frame is not freezing.
            timeoutcount = 0;
          }
        frame_id_prev = v2p_package.frame_id;

        if (vision_outback_enable_landing) {

            //compensate het_moment for the attitude of the drone, if the drone has an angle the sides will be lower then the camera:
            struct FloatEulers * attE = stateGetNedToBodyEulers_f();
            float height_roll = tanf(fabs(attE->phi)) * DRONE_WIDTH;
            float height_pitch = tanf(fabs(attE->theta)) * DRONE_LENGTH;
            float height_att = height_roll;
            if (height_pitch > height_roll)
              height_att = height_pitch;          

            if ((v2p_package.out_of_range_since > 0 && v2p_package.out_of_range_since < 1.f) || (v2p_package.out_of_range_since < 0 && v2p_package.height -height_att < vision_outback_moment_height )) {
              het_moment = true;
            } else {
              het_moment = false;
            }
            // If height==-1, the measurement is faulty so don't send it
            if (v2p_package.stupid_pprz_height > 0) {
              AbiSendMsgAGL(AGL_SONAR_ADC_ID, v2p_package.stupid_pprz_height);
            }
            // This should go outside the if above, since any sanity checks should also fail if lower than 0
            unfiltered_vision_height = v2p_package.stupid_pprz_height;

            vision_height = v2p_package.height;
          } else {
            het_moment = false;
          }

        if (vision_outback_enable_findjoe) {
            /*if (
                (msg_best_landing_status < ls_found_a_joe) ||
                (v2p_package.landing_status == ls_found_a_good_joe) ||
                (v2p_package.landing_status == ls_found_a_good_joe) ||
                (v2p_package.landing_status == ls_fixed_joe_location) ||
                (v2p_package.landing_status == ls_aruco_lock) ||
                (v2p_package.landing_status == ls_refound_fixed_joe)
                )
            {*/
              msg_best_landing_status = v2p_package.landing_status;
              waypoint_set_xy_i(WP_TD_mrk, POS_BFP_OF_REAL(v2p_package.marker_enu_x), POS_BFP_OF_REAL(v2p_package.marker_enu_y));
              waypoint_set_xy_i(WP_JOE_found, POS_BFP_OF_REAL(v2p_package.marker_enu_x), POS_BFP_OF_REAL(v2p_package.marker_enu_y));
              msg_marker_x = v2p_package.marker_enu_x;
              msg_marker_y = v2p_package.marker_enu_y;
            //}
          }

        if (!(v2p_package.version > WANTED_VISION_VERSION-0.09 && v2p_package.version < WANTED_VISION_VERSION+0.09 ))
          v2p_package.status = 2;
        break;
      }
    default:
      break;
    }
}

void enable_wp_telemetry_updates(void) {
  uint8_t wp_joe_id = WP_JOE_found;
  RunOnceEvery(10, DOWNLINK_SEND_WP_MOVED_ENU(DefaultChannel, DefaultDevice, &wp_joe_id,&(waypoints[wp_joe_id].enu_i.x),
                                              &(waypoints[wp_joe_id].enu_i.y), &(waypoints[wp_joe_id].enu_i.z)));
  uint8_t wp_TD_mkr_id = WP_TD_mrk;
  RunOnceEvery(10, DOWNLINK_SEND_WP_MOVED_ENU(DefaultChannel, DefaultDevice, &wp_TD_mkr_id,&(waypoints[wp_TD_mkr_id].enu_i.x),
                                              &(waypoints[wp_TD_mkr_id].enu_i.y), &(waypoints[wp_TD_mkr_id].enu_i.z)));
}

/* We need to wait for incomming messages */
void vision_outback_event() {
  // Check if we got some message from the Vision
  pprz_check_and_parse(vision_outback.device, &vision_outback.transport, mp_msg_buf, &vision_outback.msg_available);

  // If we have a message we should parse it
  if (vision_outback.msg_available) {
      vision_outback_parse_msg();
      vision_outback.msg_available = false;
    }
}

void vision_outback_periodic() {

  struct FloatQuat *att = stateGetNedToBodyQuat_f();
  struct FloatRates *rate = stateGetBodyRates_f();

  struct EnuCoor_f *pos = stateGetPositionEnu_f();


  struct PPRZ2VisionPackage p2k_package;
  p2k_package.att_qi = att->qi;
  p2k_package.att_qx = att->qx;
  p2k_package.att_qy = att->qy;
  p2k_package.att_qz = att->qz;
  p2k_package.rate_p = rate->p;
  p2k_package.rate_q = rate->q;
  p2k_package.rate_r = rate->r;
  p2k_package.gpsx = pos->x;
  p2k_package.gpsy = pos->y;
  p2k_package.gpsz = pos->z;
  p2k_package.baro =air_data.amsl_baro;
  p2k_package.real_gpsz = gps.hmsl;
  p2k_package.accz = imu.accel.z;
  p2k_package.thrust = stabilization_cmd[COMMAND_THRUST];

  if (state.ned_initialized_f) {
      p2k_package.geo_init_gpsx = state.ned_origin_f.lla.lat;
      p2k_package.geo_init_gpsy = state.ned_origin_f.lla.lon;
      p2k_package.geo_init_gpsz = state.ned_origin_f.lla.alt;
    } else {
      p2k_package.geo_init_gpsx = 0;
      p2k_package.geo_init_gpsy = 0;
      p2k_package.geo_init_gpsz = 0;
    }

  p2k_package.ac_id = AC_ID;

  p2k_package.enables = 0;
  if (vision_outback_enable_landing)
    p2k_package.enables |= 0b1;
  if (vision_outback_enable_take_foto) {
      p2k_package.enables |= 0b10;
      vision_outback_enable_take_foto = false;
    }
  if (vision_outback_enable_findjoe)
    p2k_package.enables |= 0b100;
  if (vision_outback_enable_opticflow)
    p2k_package.enables |= 0b1000;
  if (vision_outback_enable_attcalib)
    p2k_package.enables |= 0b10000;
  if (vision_outback_enable_videorecord)
    p2k_package.enables |= 0b100000;
  if (vision_outback_power != VISION_SETTING_STATUS_REQUEST_POWER_ON)
    p2k_package.enables |= 0b1000000;
  if (vision_outback_close_process)
    p2k_package.enables |= 0b10000000;
  if (vision_outback_update_system)
    p2k_package.enables |= 0b11000000;


  vision_found_joe =  (vision_outback_enable_findjoe && (v2p_package.landing_status == ls_found_a_good_joe ||  v2p_package.landing_status == ls_fixed_joe_location || v2p_package.landing_status == ls_refound_fixed_joe || v2p_package.landing_status == ls_aruco_lock));


  if (timeoutcount > 0) {
      timeoutcount--;
      vision_timeout = false;
    } else {
      v2p_package.status = 1;
      vision_timeout = true;
      vision_outback_close_process = false;
      vision_outback_update_system = false;
    }
  if (v2p_package.status != 0 )
    vision_timeout = true;
  do_power_state_machine();

  pprz_msg_send_IMCU_DEBUG(&(vision_outback.transport.trans_tx), vision_outback.device,
                           1, sizeof(struct PPRZ2VisionPackage), (unsigned char *)(&p2k_package));
}

enum vision_power_status power_state = VISION_POWER_STATUS_INIT;
void do_power_state_machine(void) {
  switch(power_state){
    case VISION_POWER_STATUS_INIT:
      if (VISION_POWER_ON_AT_BOOT) {
          power_state = VISION_POWER_STATUS_POWER_ON;
        } else {
#ifdef VISION_PWR_OFF
          VISION_PWR_OFF(VISION_PWR, VISION_PWR_PIN);
#endif
          power_state = VISION_POWER_STATUS_POWERED_OFF;
        }
      break;
    case VISION_POWER_STATUS_POWERED_OFF:
      if (v2p_package.status == 0) {
          power_state = VISION_POWER_STATUS_READY;
        }
      if (vision_outback_power == VISION_SETTING_STATUS_REQUEST_POWER_ON) {
          power_state = VISION_POWER_STATUS_POWER_ON;
          break;
        }
#ifndef VISION_PWR_ON
      power_state = VISION_POWER_STATUS_POWER_ON; // reset the flag to on, so that when vision is replugged it not immidiately halts
#endif
      vision_outback_power = VISION_SETTING_STATUS_REQUEST_POWER_OFF;
      break;
    case VISION_POWER_STATUS_POWER_ON:
      vision_outback_power = VISION_SETTING_STATUS_REQUEST_POWER_ON;
#ifdef VISION_PWR_ON
      VISION_PWR_ON(VISION_PWR, VISION_PWR_PIN);
#endif
      power_state = VISION_POWER_STATUS_BOOTING;
      break;
    case VISION_POWER_STATUS_BOOTING:
      if (v2p_package.status == 0) {
          power_state = VISION_POWER_STATUS_READY;
        }
      if (vision_outback_power  == VISION_SETTING_STATUS_REQUEST_POWER_OFF) {
              power_state = VISION_POWER_STATUS_POWER_OFF;
        }
      break;
    case VISION_POWER_STATUS_READY:
      if (vision_outback_power  == VISION_SETTING_STATUS_REQUEST_HALT)
        power_state = VISION_POWER_STATUS_HALTING;
      else if (vision_outback_power  == VISION_SETTING_STATUS_REQUEST_POWER_OFF)
        power_state = VISION_POWER_STATUS_POWER_OFF;
      break;
    case VISION_POWER_STATUS_HALTING:
      if (v2p_package.status > 0) {
          shutdown_count = 120*VISION_OUTBACK_PERIODIC_FREQ;
          power_state = VISION_POWER_STATUS_HALT_WAIT;
        }
      if (vision_outback_power  == VISION_SETTING_STATUS_REQUEST_POWER_OFF) {
              power_state = VISION_POWER_STATUS_POWER_OFF;
        }
      break;
    case VISION_POWER_STATUS_HALT_WAIT:
      shutdown_count--;
      if (shutdown_count == 0 )
        power_state = VISION_POWER_STATUS_POWER_OFF;
      if (vision_outback_power  == VISION_SETTING_STATUS_REQUEST_POWER_OFF) {
              power_state = VISION_POWER_STATUS_POWER_OFF;
        }
      break;      
    case VISION_POWER_STATUS_POWER_OFF:
#ifdef VISION_PWR_OFF
      VISION_PWR_OFF(VISION_PWR, VISION_PWR_PIN);
#endif
      power_state = VISION_POWER_STATUS_POWER_OFF_WAIT;
      break;
    case VISION_POWER_STATUS_POWER_OFF_WAIT:
      if (v2p_package.status > 0) {
          power_state = VISION_POWER_STATUS_POWERED_OFF;
        }
      if (vision_outback_power == VISION_SETTING_STATUS_REQUEST_POWER_ON) {
          power_state = VISION_POWER_STATUS_POWER_ON;
        }
      break;
    default:
      break;
    }
}

void enableVisionDescent(bool b) {
  timeoutcount = 10*VISION_OUTBACK_PERIODIC_FREQ;
  vision_outback_enable_landing = b;
}

void enableVisionFindJoe(bool b) {
  timeoutcount = 10*VISION_OUTBACK_PERIODIC_FREQ;
  vision_outback_enable_findjoe = b;
}

void enableVisionOpticFlow(bool b) {
  vision_outback_enable_opticflow = b;
}

bool enableVisionAttCalib(bool b) {
  //http://mariotapilouw.blogspot.com/2011/05/plane-fitting-using-opencv.html
  vision_outback_enable_attcalib = b;
  return true; // klote pprz flight plan
}

bool enableVisionVideoRecord(bool b) {
  vision_outback_enable_videorecord = b;
  return true; // klote pprz flight plan
}

bool enableVisionShutdown(bool b) {
  if (b)
    vision_outback_power= VISION_SETTING_STATUS_REQUEST_HALT;
  return true; // klote pprz flight plan
}

bool enableVisionCloseProcess(bool b) {
  vision_outback_close_process = b;
  return true; // klote pprz flight plan
}

bool getVisionReady(void) {
  return v2p_package.status == 0;
}

bool hasFoundGoodJoe(void) {
  return v2p_package.landing_status == ls_found_a_good_joe;
}

bool hasArucoLock(void) {
  return v2p_package.landing_status == ls_aruco_lock;
}

bool isVisionHeightUsedInINS(void) {
  return unfiltered_vision_height > INS_SONAR_MIN_RANGE && unfiltered_vision_height < INS_SONAR_MAX_RANGE;
}

void enableVisionPower(void)
{
  vision_outback_power = VISION_POWER_STATUS_POWER_ON;
}

void killVision(void)
{
  vision_outback_power = VISION_POWER_STATUS_POWERED_OFF;
}


