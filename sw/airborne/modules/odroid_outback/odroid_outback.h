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
 * @file "modules/odroid_outback/odroid_outback.h"
 * @author Kevin van Hecke
 * Odroid uart (RS232) communication
 */

#ifndef ODROID_OUTBACK_H
#define ODROID_OUTBACK_H

#include "std.h"
#include "generated/airframe.h"
#include "pprzlink/pprz_transport.h"
#include "math/pprz_orientation_conversion.h"
#include "math/pprz_algebra_float.h"


/* Main odroid_outback structure */
struct odroid_outback_t {
  struct link_device *device;           ///< The device which is uses for communication
  struct pprz_transport transport;      ///< The transport layer (PPRZ)
  struct OrientationReps imu_to_mag;    ///< IMU to magneto translation
  bool msg_available;                 ///< If we received a message
};


//should be exactly the same as pprz.h
struct Vision2PPRZPackage {
    int32_t frame_id;
    float height;
    float out_of_range_since;
    float marker_enu_x;
    float marker_enu_y;
    float land_enu_x;
    float land_enu_y;
    float flow_x;
    float flow_y;
    uint8_t status;
} __attribute__((__packed__));
extern struct Vision2PPRZPackage v2p_package;

//should be exactly the same as pprz.h
struct PPRZ2VisionPackage {
    float phi;
    float theta;
    float psi;
    float qi;
    float qx;
    float qy;
    float qz;
    float gpsx;
    float gpsy;
    float gpsz;
    float geo_init_gpsx;
    float geo_init_gpsy;
    float geo_init_gpsz;
    unsigned char enables;
}__attribute__((__packed__));

extern float odroid_outback_search_height;
extern bool odroid_outback_enable_landing ;
extern bool odroid_outback_enable_spotsearch;
extern bool odroid_outback_enable_findjoe;
extern bool odroid_outback_enable_opticflow;
extern bool odroid_outback_enable_attcalib;
extern bool odroid_outback_enable_videorecord;
extern struct FloatVect3 land_cmd;
extern bool het_moment;
extern bool vision_timeout;

extern void odroid_outback_init(void);
extern void odroid_outback_event(void);
extern void odroid_outback_periodic(void);

extern void enableOdroidLandingspotSearch(bool b);
extern void enableOdroidDescent(bool b);
extern void enableOdroidOpticFlow(bool b);
extern void enableOdroidFindJoe(bool b);
extern bool enableOdroidAttCalib(bool b);
extern bool enableOdroidVideoRecord(bool b);

extern bool getOdroidReady(void);

#endif

