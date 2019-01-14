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
 * @file "modules/vision_outback/vision_outback.h"
 * @author Kevin van Hecke
 * Vision uart (RS232) communication
 */

#ifndef VISION_OUTBACK_H
#define VISION_OUTBACK_H

#include "std.h"
#include "generated/airframe.h"
#include "pprzlink/pprz_transport.h"
#include "math/pprz_orientation_conversion.h"
#include "math/pprz_algebra_float.h"


/* Main vision_outback structure */
struct vision_outback_t {
  struct link_device *device;           ///< The device which is uses for communication
  struct pprz_transport transport;      ///< The transport layer (PPRZ)
  struct OrientationReps imu_to_mag;    ///< IMU to magneto translation
  bool msg_available;                 ///< If we received a message
};

//should be exactly the same as pprz.h
struct Vision2PPRZPackage
{
    int32_t frame_id;
    float height ;
    float out_of_range_since;
    float raw_height;
    float marker_enu_x;
    float marker_enu_y;
    float marker_score;
    float flow_x;
    float flow_y;
    float version;
    uint8_t status;
    uint8_t landing_status;
} __attribute__((__packed__));


//should be exactly the same as pprz.h
struct PPRZ2VisionPackage
{
    float att_qi;
    float att_qx;
    float att_qy;
    float att_qz;
    float rate_p;
    float rate_q;
    float rate_r;
    float gpsx;
    float gpsy;
    float gpsz;
    float baro;
    float real_gpsz;
    float accz;
    float thrust;
    float geo_init_gpsx; // lat in radians
    float geo_init_gpsy; // lon in radians
    float geo_init_gpsz; //TODO: remove, unused
    uint8_t ac_id;
    unsigned char enables;
} __attribute__((__packed__));

//should be exactly the same as pprz.h
enum Landing_Status {
    ls_init = 0,
    ls_searching_joe = 1,
    ls_found_a_joe = 2,
    ls_found_a_good_joe = 3,
    ls_fixed_joe_location = 4,
    ls_lost_fixed_joe = 5,
    ls_refound_fixed_joe = 6,
    ls_aruco_lock = 7,
    ls_lost_aruco_lock = 8,
    ls_error = 9
};


enum vision_power_status {VISION_POWER_STATUS_INIT,VISION_POWER_STATUS_POWERED_OFF,VISION_POWER_STATUS_POWER_ON,VISION_POWER_STATUS_BOOTING,VISION_POWER_STATUS_READY,VISION_POWER_STATUS_HALTING,VISION_POWER_STATUS_HALT_WAIT,VISION_POWER_STATUS_POWER_OFF,VISION_POWER_STATUS_POWER_OFF_WAIT };
enum vision_power_status_setting {
  VISION_SETTING_STATUS_REQUEST_HALT = 1,
  VISION_SETTING_STATUS_REQUEST_POWER_OFF = 0,
  VISION_SETTING_STATUS_REQUEST_POWER_ON = 2
};

extern float vision_outback_search_height;
extern float vision_outback_moment_height;
extern bool vision_outback_enable_landing ;
extern bool vision_outback_enable_take_foto;
extern bool vision_outback_enable_findjoe;
extern bool vision_outback_enable_opticflow;
extern bool vision_outback_enable_attcalib;
extern bool vision_outback_enable_videorecord;
extern bool vision_outback_close_process;
extern bool vision_outback_update_system;
extern uint8_t vision_outback_power;
extern struct FloatVect3 land_cmd;
extern bool het_moment;
extern bool vision_timeout;
extern bool vision_found_joe;
extern float vision_height;
extern float unfiltered_vision_height;

extern void vision_outback_init(void);
extern void vision_outback_event(void);
extern void vision_outback_periodic(void);

extern void enableVisionDescent(bool b);
extern void enableVisionOpticFlow(bool b);
extern void enableVisionFindJoe(bool b);
extern bool enableVisionAttCalib(bool b);
extern bool enableVisionVideoRecord(bool b);
extern bool enableVisionShutdown(bool b);
extern bool enableVisionCloseProcess(bool b);

extern void enableVisionPower(void);
extern void killVision(void);

extern bool getVisionReady(void);
extern bool hasFoundGoodJoe(void);
extern bool hasArucoLock(void);
extern bool isVisionHeightUsedInINS(void);

#endif

