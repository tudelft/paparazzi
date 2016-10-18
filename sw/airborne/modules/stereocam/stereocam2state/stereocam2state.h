/*
 * Copyright (C) Kimberly McGuire
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/stereocam/stereocam2state/stereocam2state.h"
 * @author Kimberly McGuire
 * This module sends the data retrieved from an external stereocamera modules, to the state filter of the drone. This is done so that the guidance modules can use that information for couadcopter
 */

#ifndef STEREOCAM2STATE_H
#define STEREOCAM2STATE_H

#include <std.h>
#include "math/pprz_algebra_float.h"

struct Egomotion {
  struct FloatVect2 div; // estimated divergence
  struct FloatVect2 ventral_flow; // estimated ventral flow
  struct FloatVect2 foe;  // estimated focus of expansion
};

extern uint8_t tracked_x, tracked_y;
extern uint8_t win_x, win_y, win_cert, disp_sum, win_processed;
extern int8_t disp_diff;
extern uint16_t win_dist, win_size;

extern uint8_t stereo_agl; // in dm

extern struct Egomotion stereo_motion;

extern void stereo_to_state_init(void);
extern void stereo_to_state_periodic(void);

#endif

