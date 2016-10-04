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
static float MARKER_FOUND_TIME_MAX = 10.0;
static float MARKER_MID_TIME_MAX = 1.0;

//This is for bucket
//static int MARKER_WINDOW = 25;

//This is for landing pad
static int MARKER_WINDOW = 40;

// General outputs
struct Marker marker1;
struct Marker marker2;

// Helipad detection
static struct video_listener* helipad_listener;
static struct video_listener* bucket_listener_front;
static struct video_listener* bucket_listener_bottom;


static void geo_locate_marker(struct Marker *marker, struct image_t* img) {
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
    float agl = sonar_bebop.distance; // -pos->z

    marker->geo_relative.x *= agl/zi;
    marker->geo_relative.y *= agl/zi;
    marker->geo_relative.z = agl;

    // TODO filter this location over time to reduce the jitter in output
    // TODO use difference in position as a velocity estimate along side opticflow in hff...

    // NED
    marker->geo_location.x = pos->x + marker->geo_relative.x;
    marker->geo_location.y = pos->y + marker->geo_relative.y;
    marker->geo_location.z = 0;
}


static void marker_detected(struct Marker *marker, struct image_t* img, int pixelx, int pixely)
{
    marker->detected = true;

    // store marker pixel location
    marker->pixel.x = pixelx;
    marker->pixel.y = pixely;

    // calculate geo location
    geo_locate_marker(marker, img);

    // Increase marker detected time and mid counter

    marker->found_time += img->dt / 1000000.f;

    if (marker->found_time > MARKER_FOUND_TIME_MAX) { marker->found_time = MARKER_FOUND_TIME_MAX; }
    if (marker->mid_time > MARKER_MID_TIME_MAX) { marker->mid_time = MARKER_MID_TIME_MAX; }

    // This is for bucket
//    if (marker->pixel.x < 120 + MARKER_WINDOW &&
//        marker->pixel.x > 120 - MARKER_WINDOW &&
//        marker->pixel.y < 100 + MARKER_WINDOW && //To account for gripper in the back
//        marker->pixel.y > 100 - MARKER_WINDOW)

    // This is for landing pad
    if (marker->pixel.x < 120 + MARKER_WINDOW &&
        marker->pixel.x > 120 - MARKER_WINDOW &&
        marker->pixel.y < 120 + MARKER_WINDOW && //To account for gripper in the back
        marker->pixel.y > 120 - MARKER_WINDOW)
    {
        marker->mid = true;
        marker->mid_time += img-> dt / 1000000.f;
    } else {
        marker->mid = false;
        marker->mid_time -= 0.1 * img-> dt / 1000000.f;
        if (marker->mid_time < 0) { marker->mid_time = 0; }
    }

    marker->processed = false;
}

static void marker_not_detected(struct Marker *marker, struct image_t* img)
{
    marker->detected = false;
    marker->mid = false;

    //This is for bucket
//    marker->found_time -= 1.5 * img->dt / 1000000.f;
//    marker->mid_time -= 1* img-> dt / 1000000.f;
    //This is for landing pad
    marker->found_time -= 1 * img->dt / 1000000.f;
    marker->mid_time -= 0.5* img-> dt / 1000000.f;

    if (marker->found_time < 0) { marker->found_time = 0; }
    if (marker->mid_time < 0) { marker->mid_time = 0; }
}

static struct image_t *detect_bottom_bucket(struct image_t* img) {

    // Color Filter
    struct image_filter_t filter;
    filter.y_min = 87;    // red
    filter.y_max = 255;
    filter.u_min = 105;
    filter.u_max = 255;
    filter.v_min = 170;
    filter.v_max = 255;

    int threshold = 50;

    // Output image
    struct image_t dst;
    image_create(&dst, img->w, img->h, IMAGE_GRADIENT);

    // Labels
    uint16_t labels_count = 512;
    struct image_label_t labels[512];

    // Blob finder
    image_labeling(img, &dst, &filter, 1, labels, &labels_count);

    int largest_id = -1;
    int largest_size = 0;

    // Find largest
    for (int i=0; i<labels_count; i++) {
        // Only consider large blobs
        if (labels[i].pixel_cnt > threshold) {
            if (labels[i].pixel_cnt > largest_size) {
                largest_size = labels[i].pixel_cnt;
                largest_id = i;
            }
        }
    }

    if (largest_id >= 0)
    {
        int xloc   = labels[largest_id].x_sum / labels[largest_id].pixel_cnt * 2;
        int yloc   = labels[largest_id].y_sum / labels[largest_id].pixel_cnt - 20; //-20 is for the gripper position bias
        marker_detected(&marker1, img, xloc, yloc);
    }
    else
    {
        marker_not_detected(&marker1, img);
    }

    image_free(&dst);

//    fprintf(stderr, "[blob] fps %.2f \n", (1000000.f / img->dt));

    return NULL;
}

static struct image_t *detect_front_bucket(struct image_t* img) {

    // Color Filter
    struct image_filter_t filter;
    filter.y_min = 87;    // red
    filter.y_max = 162;
    filter.u_min = 94;
    filter.u_max = 255;
    filter.v_min = 184;
    filter.v_max = 255;

    int threshold = 50;

    // Output image
    struct image_t dst;
    image_create(&dst, img->w, img->h, IMAGE_GRADIENT);

    // Labels
    uint16_t labels_count = 512;
    struct image_label_t labels[512];

    // Blob finder
    image_labeling(img, &dst, &filter, 1, labels, &labels_count);

    int largest_id = -1;
    int largest_size = 0;

    // Find largest
    for (int i=0; i<labels_count; i++) {
        // Only consider large blobs
        if (labels[i].pixel_cnt > threshold) {
            if (labels[i].pixel_cnt > largest_size) {
                largest_size = labels[i].pixel_cnt;
                largest_id = i;
            }
        }
    }

    if (largest_id >= 0)
    {
        int xloc   = labels[largest_id].x_sum / labels[largest_id].pixel_cnt * 2;
        int yloc   = labels[largest_id].y_sum / labels[largest_id].pixel_cnt;
        marker_detected(&marker2, img, xloc, yloc);
    }
    else
    {
        marker_not_detected(&marker2, img);
    }

    image_free(&dst);

//    fprintf(stderr, "[blob] fps %.2f \n", (1000000.f / img->dt));

    return NULL;
}

static struct image_t *detect_helipad_marker(struct image_t* img)
{
    struct results helipad_marker = opencv_imav_landing(
            (char*) img->buf,
            img->w,
            img->h,
            2, //squares
            210, //binary threshold
            0, img->dt); //modify image, time taken

    if (helipad_marker.marker)
    {
        marker_detected(&marker1, img, helipad_marker.maxx, helipad_marker.maxy);
    }
    else
    {
        marker_not_detected(&marker1, img);
    }
    return NULL;
}

static struct image_t *draw_target_marker1(struct image_t* img)
{
    if (marker1.detected && SHOW_MARKER) {
        struct point_t t = {marker1.pixel.x, marker1.pixel.y - 50},
                b = {marker1.pixel.x, marker1.pixel.y + 50},
                l = {marker1.pixel.x - 50, marker1.pixel.y},
                r = {marker1.pixel.x + 50, marker1.pixel.y};

        image_draw_line(img, &t, &b);
        image_draw_line(img, &l, &r);
    }

//    DOWNLINK_SEND_DETECTOR(DefaultChannel, DefaultDevice,
//                           &marker1.detected,
//                           &marker1.pixel.x,
//                           &marker1.pixel.y,
//                           &marker1.found_time,
//                           &marker1.mid_time);

    return img;
}

static struct image_t *draw_target_marker2(struct image_t* img)
{
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

void detector_locate_bucket(void)
{
    bucket_listener_bottom->active = true;
    helipad_listener->active = false;
}

void detector_locate_helipad(void)
{
    bucket_listener_bottom->active = false;
    helipad_listener->active = true;
}

void detector_init(void)
{
    // BOTTOM MARKER

    marker1.detected = false;
    marker1.mid = false;
    marker1.processed = true;
    marker1.pixel.x = 0;
    marker1.pixel.y = 0;
    marker1.found_time = 0;
    marker1.mid_time = 0;

    helipad_listener = cv_add_to_device_async(&DETECTOR_CAMERA1, detect_helipad_marker, 5);
    helipad_listener->maximum_fps = 20;

    bucket_listener_bottom = cv_add_to_device(&DETECTOR_CAMERA1, detect_bottom_bucket);

    cv_add_to_device(&DETECTOR_CAMERA1, draw_target_marker1);

    // FRONT MARKER
    marker2.detected = false;
    marker2.processed = true;
    marker2.pixel.x = 0;
    marker2.pixel.y = 0;
    marker2.found_time = 0;

    bucket_listener_front = cv_add_to_device_async(&DETECTOR_CAMERA2, detect_front_bucket, 5);
    bucket_listener_front->maximum_fps = 20;

    cv_add_to_device(&DETECTOR_CAMERA2, draw_target_marker2);

    // INITIAL STATE
    detector_locate_bucket();
}
