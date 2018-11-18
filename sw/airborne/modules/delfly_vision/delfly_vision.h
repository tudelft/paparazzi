/*
 * Copyright (C) Matej Karasek, Kirk Scheper
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
 * @file "modules/delfly_vision/delfly_vision.h"
 * @author Matej Karasek, Kirk Scheper
 * Vision module for (tail less) DelFlies
 */

#ifndef DELFLY_VISION_H
#define DELFLY_VISION_H

#include "pprzlink/pprz_transport.h"
#include "math/pprz_algebra_float.h"

/* Main stereocam structure */
struct stereocam_t {
  struct link_device *device;           ///< The device which is used for communication
  struct pprz_transport transport;      ///< The transport layer (PPRZ)
  struct FloatRMat body_to_cam;         ///< IMU to stereocam rotation
  bool msg_available;                   ///< If we received a message
};

/* gate structure */
struct gate_t {
  uint8_t quality;
  float width;
  float height;
  float phi;
  float theta;
  float depth;
  float dt;
  uint16_t color_cnt[64];
  uint16_t num_color_bins;
};

struct gate_filt_t {
  float width;
  float height;
  float psi;
  float theta;
};

extern struct gate_t gate_raw;

///* follow structure */
//struct follow_t {
//  int32_t line_quality;
//  float line_phi;
//  float line_theta;
//  float line_slope;
//  float obst_phi;
//  float obst_theta;
//  float line_phiF;
//  float line_thetaF;
//  float line_slopeF;
//  float obst_phiF;
//  float obst_thetaF;
//  float dt;
//};

/* follow structure */
struct follow_t {
  int32_t line_quality;
  float dt;
  float A;
  float B;
  float C;
  float r2;
  float line_lon;
  float line_lat;
  float line_angle;
  float line_lonF;
  float line_latF;
  float line_angleF;
  float line_dt;
  float obst_phi;
  float obst_theta;
  float obst_phiF;
  float obst_thetaF;
  float obst_lat;
  float obst_lon;
};

extern struct follow_t follow;


struct pid_t {
  float p;
  float i;
  float d;
};

extern void delfly_vision_init(void);
extern void delfly_vision_periodic(void);
extern void delfly_vision_event(void);

/* Implement own Horizontal loops */
extern void guidance_h_module_init(void);
extern void guidance_h_module_enter(void);
extern void guidance_h_module_read_rc(void);
extern void guidance_h_module_run(bool in_flight);
/* Implement own Vertical loops */
extern void guidance_v_module_init(void);
extern void guidance_v_module_enter(void);
extern void guidance_v_module_run(bool in_flight);

// settings
extern float filt_gate_tc;  // gate filter time constant
extern float filt_line_tc;  // line filter time constant
extern float filt_obst_tc;  // obstacle filter time constant
extern float altitude_setp;
extern float y_slope; // vertical camera angle where slope of the fitted curve is computed
extern float y_offset; // vertical camera angle where offset of the fitted curve is computed
extern float sp_theta_gate;
extern float sp_theta_follow;

extern struct pid_t phi_gains;
extern struct pid_t theta_gains;
extern struct pid_t thrust_gains;

#endif

