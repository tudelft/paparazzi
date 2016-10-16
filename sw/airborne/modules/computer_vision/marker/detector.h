/*
 * Copyright (C) IMAV 2016
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
 * @file "modules/computer_vision/marker/detector.h"
 */

#ifndef MARKER_DETECTOR_H
#define MARKER_DETECTOR_H

#include "math/pprz_geodetic_float.h"
#include "lib/vision/image.h"

struct Detector {
    struct video_listener *item_front;
    struct video_listener *bucket_front;
    struct video_listener *item_bottom;
    struct video_listener *bucket_bottom;
    struct video_listener *helipad_bottom;
};

extern struct Detector detector;

struct Marker {
    volatile bool detected;
    volatile bool processed;
    struct point_t pixel;
    struct NedCoor_f geo_location;
    struct FloatVect3 geo_relative;
    float found_time;
};

extern struct Marker marker1;
extern struct Marker marker2;

extern int SQRS;
extern int BIN_THRESH;
extern float ZSCORE;
extern int AREA_THRESH;

void detector_init(void);

void detector_disable_all(void);


#endif
