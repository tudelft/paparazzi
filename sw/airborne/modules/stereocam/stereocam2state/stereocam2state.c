/*
 * Copyright (C) Kimberly McGuire
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/stereocam/stereocam2state/stereocam2state.c"
 * @author Kimberly McGuire
 * This module sends the data retrieved from an external stereocamera modules, to the state filter of the drone. This is done so that the guidance modules can use that information for couadcopter
 */

#include "stereocam2state.h"
#include "modules/stereocam/stereocam.h"

#include "subsystems/abi.h"
#include "subsystems/datalink/telemetry.h"
#include "filters/median_filter.h"
#include "state.h"

#include "math/pprz_algebra_float.h"
#include "math/pprz_orientation_conversion.h"

#ifndef STEREOCAM2STATE_SENDER_ID
#define STEREOCAM2STATE_SENDER_ID ABI_BROADCAST
#endif

#include "subsystems/datalink/telemetry.h"

struct MedianFilter3Int global_filter;
struct MedianFilter3Int pixelwise_filter;

struct Egomotion stereo_motion;

uint8_t tracked_x, tracked_y;
uint8_t win_x, win_y, win_cert, win_size;
uint16_t win_dist;

struct Int16Vect3 vel_pixel_i, vel_global_i;
uint8_t agl; // in dm
uint8_t fps;

uint16_t range_finder[4]; // distance from range finder in mm clockwise starting with front

void stereocam_to_state(void);

#if PERIODIC_TELEMETRY
static void stereocam_telem_send(struct transport_tx *trans, struct link_device *dev)
{
  int8_t foe_x = stereo_motion.foe.x;
  int8_t foe_y =  stereo_motion.foe.y;
  pprz_msg_send_STEREOCAM_OPTIC_FLOW(trans, dev, AC_ID,
                                     &fps, &agl, &vel_pixel_i.x, &vel_pixel_i.z,
                                     &vel_global_i.x, &vel_global_i.y, &vel_global_i.z,
                                     &tracked_x, &tracked_y, &win_x, &win_y, &win_cert, &win_size, &win_dist,
                                     &foe_x, &foe_y);

  //  DOWNLINK_SEND_OPTIC_FLOW_EST(DefaultChannel, DefaultDevice, &fps, &dummy_uint16, &dummy_uint16, &flow_x, &flow_y, &foe_x, &foe_y,
  //    &vel_x, &vel_y,&dummy_float, &dummy_float, &dummy_float);
}
#endif

void stereo_to_state_init(void)
{
  InitMedianFilterVect3Int(global_filter);
  InitMedianFilterVect3Int(pixelwise_filter);

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_STEREOCAM_OPTIC_FLOW, stereocam_telem_send);
#endif

  memset(range_finder, 0, sizeof(range_finder));
}

void stereo_to_state_periodic(void)
{
  if (stereocam_data.fresh && stereocam_data.len == 28) { // length of SEND_EDGEFLOW message
    stereocam_to_state();
    stereocam_data.fresh = 0;
  } else if (stereocam_data.fresh && stereocam_data.len == 2) {  // length of Lucas Kanade point tracker message
    tracked_x = stereocam_data.data[0];
    tracked_y = stereocam_data.data[1];
    stereocam_data.fresh = 0;
  } else if (stereocam_data.fresh && stereocam_data.len == 8) {  // array from range finders
    uint16_t *int16Arrray = (uint16_t *)stereocam_data.data;
    range_finder[0] = int16Arrray[0];
    range_finder[1] = int16Arrray[1];
    range_finder[2] = int16Arrray[2];
    range_finder[3] = int16Arrray[3];
    AbiSendMsgRANGE_SENSORS(STEREOCAM2STATE_SENDER_ID, range_finder[0], range_finder[1], range_finder[2], range_finder[3]);
    stereocam_data.fresh = 0;
  } else if (stereocam_data.fresh && stereocam_data.len == 9) {  // length of WINDOW message
    win_x = stereocam_data.data[0];
    win_y = stereocam_data.data[1];
    win_cert = stereocam_data.data[2];
    win_size = stereocam_data.data[6];
    win_dist = (uint16_t)stereocam_data.data[8] | ((uint16_t)stereocam_data.data[7] << 8);
    stereocam_data.fresh = 0;
  }
}

void stereocam_to_state(void)
{
  // Get info from stereocam data
  const int16_t *buffer = (int16_t *)stereocam_data.data;
  int16_t div_x = buffer[0];
  int16_t flow_x = buffer[1];
  int16_t div_y = buffer[2];
  int16_t flow_y = buffer[3];

  agl = stereocam_data.data[8]; // in dm
  fps = stereocam_data.data[9];

  int16_t vel_x_global_int = buffer[5];
  int16_t vel_y_global_int = buffer[6];
  int16_t vel_z_global_int = buffer[7];

  int16_t vel_x_pixelwise_int = buffer[8];
  int16_t vel_z_pixelwise_int = buffer[9];

  //int16_t vel_x_stereo_avoid_pixelwise_int = buffer[10];
  //int16_t vel_z_stereo_avoid_pixelwise_int = buffer[11];

  stereo_motion.foe.x = buffer[12];
  stereo_motion.foe.y = buffer[13];

  int16_t RES = buffer[14];

  vel_pixel_i.x = vel_x_pixelwise_int;
  vel_pixel_i.y = 0;
  vel_pixel_i.z = vel_z_pixelwise_int;

  vel_global_i.x = vel_x_global_int;
  vel_global_i.y = vel_y_global_int;
  vel_global_i.z = vel_z_global_int;

  // todo implement float median filter
  UpdateMedianFilterVect3Int(global_filter, vel_pixel_i);
  UpdateMedianFilterVect3Int(pixelwise_filter, vel_global_i);

  struct FloatVect3 vel_f;
  vel_f.x = (float)vel_global_i.x / RES;  // pixelwise doesn't seem great just yet.
  vel_f.y = (float)vel_global_i.y / RES;
  vel_f.z = (float)vel_global_i.z / RES;

  // Transform from camera frame to body frame
  struct FloatVect3 vel_body_f;

  float_rmat_transp_vmult(&vel_body_f, &body_to_stereocam, &vel_f);

  stereo_motion.ventral_flow.x = (float)flow_x / RES;
  stereo_motion.ventral_flow.y = (float)flow_y / RES;
  stereo_motion.div.x = (float)div_x / RES;
  stereo_motion.div.y = (float)div_y / RES;

  //Send velocity estimate to state
  //TODO:: Make variance dependable on line fit error, after new horizontal filter is made
  uint32_t now_ts = get_sys_time_usec();

  if (abs(vel_body_f.x) < 1.5 && abs(vel_body_f.y) < 1.5) {
    AbiSendMsgVELOCITY_ESTIMATE(STEREOCAM2STATE_SENDER_ID, now_ts,
                                vel_body_f.x,
                                vel_body_f.y,
                                vel_body_f.z,
                                0.3f
                               );
  }
}
