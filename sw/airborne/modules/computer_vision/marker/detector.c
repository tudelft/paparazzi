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
 * @file "modules/computer_vision/marker/detector.c"
 */

#include <stdio.h>

#include "state.h"
#include "math/pprz_orientation_conversion.h"

#include "modules/computer_vision/cv.h"
#include "modules/computer_vision/marker/detector.h"

#include "modules/pose_history/pose_history.h"
#include "modules/sonar/sonar_bebop.h"

#include "modules/computer_vision/blob/blob_finder.h"

#include "modules/computer_vision/opencv_imav_landingpad.h"     // OpenCV contour based marker detection
#include "subsystems/datalink/telemetry.h"

static bool SHOW_MARKER = true;
static float MARKER_FOUND_TIME_MAX = 5.0;

// General outputs
struct Detector detector;
struct Marker marker1;
struct Marker marker2;

static struct Marker single_blob_finder(struct image_t *img, struct image_filter_t *filter, int threshold) {
  // Output image
  struct image_t dst;
  image_create(&dst, img->w, img->h, IMAGE_GRADIENT);

  // Labels
  uint16_t labels_count = 512;
  struct image_label_t labels[512];

  // Blob finder
  image_labeling(img, &dst, filter, 1, labels, &labels_count);

  int largest_id = -1;
  int largest_size = 0;

  // Find largest
  for (int i = 0; i < labels_count; i++) {
    // Only consider large blobs
    if (labels[i].pixel_cnt > largest_size) {
      largest_size = labels[i].pixel_cnt;
      largest_id = i;
    }
  }

  fprintf(stderr, "[detector %i] largest blob size %i.\n", img->w, largest_size);

  struct Marker marker;

  if (largest_id >= 0 && largest_size > threshold) {
    marker.pixel.x = labels[largest_id].x_sum / labels[largest_id].pixel_cnt * 2;
    marker.pixel.y = labels[largest_id].y_sum / labels[largest_id].pixel_cnt;
    marker.pixel_cnt = labels[largest_id].pixel_cnt;
    marker.detected = true;
  } else {
    marker.detected = false;
  }

  image_free(&dst);

  return marker;
}


static void geo_locate_marker(struct Marker *marker, struct image_t *img) {
  // Obtain the relative pixel location (measured from center in body frame) rotated to vehicle reference frame
  struct FloatVect3 pixel_relative;
  pixel_relative.x = (float) (img->h / 2) - (float) marker->pixel.y;
  pixel_relative.y = (float) marker->pixel.x - (float) (img->w / 2);
  pixel_relative.z = 400.; // estimated focal length in px

  // Get the rotation measured at image capture
  struct pose_t pose = get_rotation_at_timestamp(img->pprz_ts);

  // Create a orientation representation
  struct FloatRMat ned_to_body;
  float_rmat_of_eulers(&ned_to_body, &pose.eulers);

  // Rotate the pixel vector from body frame to the north-east-down frame
  float_rmat_transp_vmult(&marker->geo_relative, &ned_to_body, &pixel_relative);

  // Divide by z-component to normalize the projection vector
  float zi = marker->geo_relative.z;

  // Pointing up or horizontal -> no ground projection
  if (zi <= 0.) { return; }

  // Scale the parameters based on distance to ground and focal point
  struct NedCoor_f *pos = stateGetPositionNed_f();
  float agl = -pos->z; // sonar_bebop.distance

  marker->geo_relative.x *= agl / zi;
  marker->geo_relative.y *= agl / zi;
  marker->geo_relative.z = agl;

  // TODO filter this location over time to reduce the jitter in output
  // TODO use difference in position as a velocity estimate along side opticflow in hff...

  // NED
  marker->geo_location.x = pos->x + marker->geo_relative.x;
  marker->geo_location.y = pos->y + marker->geo_relative.y;
  marker->geo_location.z = 0;
}


static void marker_detected(struct Marker *marker, struct image_t *img, int pixelx, int pixely) {
  marker->detected = true;

  // store marker pixel location
  marker->pixel.x = pixelx;
  marker->pixel.y = pixely;

  // Increase marker detected time
  marker->found_time += img->dt / 1000000.f;

  if (marker->found_time > MARKER_FOUND_TIME_MAX) {
    marker->found_time = MARKER_FOUND_TIME_MAX;
  }

  marker->processed = false;
}


static void marker_not_detected(struct Marker *marker, struct image_t *img) {
  marker->detected = false;

  marker->found_time -= 1 * img->dt / 1000000.f;

  if (marker->found_time < 0) {
    marker->found_time = 0;
  }
}


static struct image_t *detect_front_blue_item(struct image_t *img) {

  // Color Filter
  struct image_filter_t filter;
  filter.y_min = 15;
  filter.y_max = 231;
  filter.u_min = 150;
  filter.u_max = 255;
  filter.v_min = 0;
  filter.v_max = 154;

  int threshold = 50;

  struct Marker marker = single_blob_finder(img, &filter, threshold);

  if (marker.detected) {
    marker_detected(&marker2, img, marker.pixel.x, marker.pixel.y);
  } else {
    marker_not_detected(&marker2, img);
  }

  return NULL;
}


static struct image_t *detect_bottom_blue_item(struct image_t *img) {

  // Color Filter
  struct image_filter_t filter;
  filter.y_min = 26;
  filter.y_max = 224;
  filter.u_min = 157;
  filter.u_max = 255;
  filter.v_min = 0;
  filter.v_max = 168;

  int threshold = 50;

  struct Marker marker = single_blob_finder(img, &filter, threshold);

  if (marker.detected) {
    marker_detected(&marker1, img, marker.pixel.x, marker.pixel.y);
    geo_locate_marker(&marker1, img);
  } else {
    marker_not_detected(&marker1, img);
  }

  return NULL;
}


static struct image_t *detect_front_blue_bucket(struct image_t *img) {

  // Color Filter
  struct image_filter_t filter;
  filter.y_min = 38;
  filter.y_max = 236;
  filter.u_min = 145;
  filter.u_max = 172;
  filter.v_min = 0;
  filter.v_max = 150;

  int threshold = 50;

  struct Marker marker = single_blob_finder(img, &filter, threshold);

  if (marker.detected) {
    marker_detected(&marker2, img, marker.pixel.x, marker.pixel.y);
  } else {
    marker_not_detected(&marker2, img);
  }

  return NULL;
}


static struct image_t *detect_bottom_blue_bucket(struct image_t *img) {

  // Color Filter
  struct image_filter_t filter;
  filter.y_min = 53;
  filter.y_max = 134;
  filter.u_min = 111;
  filter.u_max = 255;
  filter.v_min = 0;
  filter.v_max = 150;

  int threshold = 50;

  struct Marker marker = single_blob_finder(img, &filter, threshold);

  if (marker.detected) {
    marker_detected(&marker1, img, marker.pixel.x, marker.pixel.y);
    geo_locate_marker(&marker1, img);
  } else {
    marker_not_detected(&marker1, img);
  }

  return NULL;
}


static struct image_t *detect_front_red_item(struct image_t *img) {

  // Color Filter
  struct image_filter_t filter;
  filter.y_min = 23;
  filter.y_max = 230;
  filter.u_min = 79;
  filter.u_max = 134;
  filter.v_min = 202;
  filter.v_max = 255;

  int threshold = 50;

  struct Marker marker = single_blob_finder(img, &filter, threshold);

  if (marker.detected) {
    marker_detected(&marker2, img, marker.pixel.x, marker.pixel.y);
  } else {
    marker_not_detected(&marker2, img);
  }

  return NULL;
}


static struct image_t *detect_bottom_red_item(struct image_t *img) {

  // Color Filter
  struct image_filter_t filter;
  filter.y_min = 56;
  filter.y_max = 208;
  filter.u_min = 12;
  filter.u_max = 175;
  filter.v_min = 158;
  filter.v_max = 255;

  int threshold = 50;

  struct Marker marker = single_blob_finder(img, &filter, threshold);

  if (marker.detected) {
    marker_detected(&marker1, img, marker.pixel.x, marker.pixel.y);
    geo_locate_marker(&marker1, img);
  } else {
    marker_not_detected(&marker1, img);
  }

  return NULL;
}


static struct image_t *detect_front_red_bucket(struct image_t *img) {

  // Color Filter
  struct image_filter_t filter;
  filter.y_min = 23;
  filter.y_max = 186;
  filter.u_min = 69;
  filter.u_max = 174;
  filter.v_min = 159;
  filter.v_max = 255;

  int threshold = 50;

  struct Marker marker = single_blob_finder(img, &filter, threshold);

  if (marker.detected) {
    marker_detected(&marker2, img, marker.pixel.x, marker.pixel.y);
  } else {
    marker_not_detected(&marker2, img);
  }

  return NULL;
}


static struct image_t *detect_bottom_red_bucket(struct image_t *img) {

  // Color Filter
  struct image_filter_t filter;
  filter.y_min = 36;
  filter.y_max = 149;
  filter.u_min = 98;
  filter.u_max = 214;
  filter.v_min = 154;
  filter.v_max = 255;

  int threshold = 50;

//  img->h = 120; // set image to half so blob finder only scans top part
  struct Marker marker = single_blob_finder(img, &filter, threshold);
  img->h = 240;

  if (marker.detected) {
    marker_detected(&marker1, img, marker.pixel.x, marker.pixel.y);
    geo_locate_marker(&marker1, img);
  } else {
    marker_not_detected(&marker1, img);
  }

  return NULL;
}


static struct image_t *detect_front_white_building(struct image_t *img) {

  // Color Filter
  struct image_filter_t filter;
  filter.y_min = 175;
  filter.y_max = 194;
  filter.u_min = 100;
  filter.u_max = 118;
  filter.v_min = 103;
  filter.v_max = 128;

  int threshold = 5000;

  struct Marker marker = single_blob_finder(img, &filter, threshold);

  if (marker.detected) {
    marker_detected(&marker2, img, marker.pixel.x, marker.pixel.y);
  } else {
    marker_not_detected(&marker2, img);
  }

  return NULL;
}


static struct image_t *detect_helipad_marker(struct image_t *img) {
  struct results helipad_marker = opencv_imav_landing(
          (char *) img->buf,
          img->w,
          img->h,
          2, //squares
          210, //binary threshold
          0, img->dt); //modify image, time taken

  if (helipad_marker.marker) {
    marker_detected(&marker1, img, helipad_marker.maxx, helipad_marker.maxy);
    geo_locate_marker(&marker1, img);
  } else {
    marker_not_detected(&marker1, img);
  }
  return NULL;
}


static struct image_t *draw_target_marker1(struct image_t *img) {
  if (marker1.detected && SHOW_MARKER) {
    struct point_t t = {marker1.pixel.x, marker1.pixel.y - 50},
            b = {marker1.pixel.x, marker1.pixel.y + 50},
            l = {marker1.pixel.x - 50, marker1.pixel.y},
            r = {marker1.pixel.x + 50, marker1.pixel.y};

    image_draw_line(img, &t, &b);
    image_draw_line(img, &l, &r);
  }

  uint8_t detected = marker1.detected;
  DOWNLINK_SEND_DETECTOR(DefaultChannel, DefaultDevice,
                         &detected,
                         &marker1.pixel.x,
                         &marker1.pixel.y,
                         &marker1.geo_relative.x,
                         &marker1.geo_relative.y,
                         &marker1.found_time);

  return img;
}


static struct image_t *draw_target_marker2(struct image_t *img) {
  if (marker2.detected && SHOW_MARKER) {
    struct point_t t = {marker2.pixel.x, marker2.pixel.y - 50},
            b = {marker2.pixel.x, marker2.pixel.y + 50},
            l = {marker2.pixel.x - 50, marker2.pixel.y},
            r = {marker2.pixel.x + 50, marker2.pixel.y};

    image_draw_line(img, &t, &b);
    image_draw_line(img, &l, &r);
  }

  return img;
}


void detector_disable_all() {
  detector.front_blue_item->active = false;
  detector.bottom_blue_item->active = false;
  detector.front_blue_bucket->active = false;
  detector.bottom_blue_bucket->active = false;
  detector.front_red_item->active = false;
  detector.bottom_red_item->active = false;
  detector.front_red_bucket->active = false;
  detector.bottom_red_bucket->active = false;
  detector.front_white_building->active = false;
  detector.helipad_bottom->active = false;
}


void detector_init(void) {
  // BOTTOM MARKER
  marker1.detected = false;
  marker1.processed = true;
  marker1.pixel.x = 0;
  marker1.pixel.y = 0;
  marker1.found_time = 0;

  detector.helipad_bottom = cv_add_to_device_async(&DETECTOR_CAMERA1, detect_helipad_marker, 5);
  detector.helipad_bottom->maximum_fps = 20;

  detector.bottom_red_item = cv_add_to_device(&DETECTOR_CAMERA1, detect_bottom_red_item);
  detector.bottom_red_bucket = cv_add_to_device(&DETECTOR_CAMERA1, detect_bottom_red_bucket);
  detector.bottom_blue_item = cv_add_to_device(&DETECTOR_CAMERA1, detect_bottom_blue_item);
  detector.bottom_blue_bucket = cv_add_to_device(&DETECTOR_CAMERA1, detect_bottom_blue_bucket);

  cv_add_to_device(&DETECTOR_CAMERA1, draw_target_marker1);

  // FRONT MARKER
  marker2.detected = false;
  marker2.processed = true;
  marker2.pixel.x = 0;
  marker2.pixel.y = 0;
  marker2.found_time = 0;

  detector.front_blue_item = cv_add_to_device(&DETECTOR_CAMERA2, detect_front_blue_item);
  detector.front_blue_bucket = cv_add_to_device(&DETECTOR_CAMERA2, detect_front_blue_bucket);
  detector.front_red_item = cv_add_to_device(&DETECTOR_CAMERA2, detect_front_red_item);
  detector.front_red_bucket = cv_add_to_device(&DETECTOR_CAMERA2, detect_front_red_bucket);
  detector.front_white_building = cv_add_to_device(&DETECTOR_CAMERA2, detect_front_white_building);

  cv_add_to_device(&DETECTOR_CAMERA2, draw_target_marker2);

  // INITIAL STATE
  detector_disable_all();
}
