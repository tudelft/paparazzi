/*
 * Copyright (C) 2019 Kirk Scheper <kirkscheper@gmail.com>
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/**
 * @file modules/computer_vision/MAV_cv_detect_group12_cmjong.c
 *
 * Cascading obstacle detector: color detection → (edge detection) → optical flow.
 * Cheapest checks run first; expensive OF only when nothing else detected.
 */

// Own header
#include "modules/computer_vision/MAV_cv_detect_group12_cmjong.h"
#include "modules/computer_vision/MAV_cv_color_group12_cmjong.h"

#include "modules/computer_vision/cv.h"
#include "modules/core/abi.h"
#include "std.h"

#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>
#include "pthread.h"

// Optical flow includes
#include "generated/airframe.h"
#include "modules/computer_vision/lib/vision/image.h"
#include "modules/computer_vision/opticflow/opticflow_calculator.h"
#include "modules/pose_history/pose_history.h"

#define PRINT(string,...) fprintf(stderr, "[object_detector->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if OBJECT_DETECTOR_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

static pthread_mutex_t mutex;

#ifndef COLOR_OBJECT_DETECTOR_FPS1
#define COLOR_OBJECT_DETECTOR_FPS1 0 ///< Default FPS (zero means run at camera fps)
#endif
#ifndef COLOR_OBJECT_DETECTOR_FPS2
#define COLOR_OBJECT_DETECTOR_FPS2 0 ///< Default FPS (zero means run at camera fps)
#endif

// ── Color filter settings ────────────────────────────────────────────────────
uint8_t orange_lum_min = 0;
uint8_t orange_lum_max = 0;
uint8_t orange_cb_min = 0;
uint8_t orange_cb_max = 0;
uint8_t orange_cr_min = 0;
uint8_t orange_cr_max = 0;

uint8_t blue_lum_min = 0;
uint8_t blue_lum_max = 0;
uint8_t blue_cb_min = 0;
uint8_t blue_cb_max = 0;
uint8_t blue_cr_min = 0;
uint8_t blue_cr_max = 0;

uint8_t green_lum_min = 0;
uint8_t green_lum_max = 0;
uint8_t green_cb_min = 0;
uint8_t green_cb_max = 0;
uint8_t green_cr_min = 0;
uint8_t green_cr_max = 0;

bool cod_draw = false;

float threshold_orange_detector = 0;
float threshold_blue_detector = 0;
float threshold_green_detector = 0;

// ── Crop settings (GCS-tunable) ──────────────────────────────────────────────
float crop_h_frac = 0.80f;  // height fraction for initial crop (color + edge)
float crop_w_frac = 0.40f;  // width fraction for secondary OF crop

// ── Optical flow settings ────────────────────────────────────────────────────
#ifndef LUKE_OF_VISUAL_DETECTION_ID
#define LUKE_OF_VISUAL_DETECTION_ID ABI_BROADCAST
#endif

#define LUKE_OF_CAMERA_SLOTS 1

float luke_of_divergence_threshold = 0.01f;
bool  luke_of_show_stream_overlay = false;
bool  luke_of_derotation = true;
float luke_of_ema_alpha = 0.5f;
float luke_of_smoothed_divergence = 0.0f;

bool  luke_of_request_reset = false;

struct opticflow_t luke_of_opticflow[LUKE_OF_CAMERA_SLOTS];

// ── Shared message struct ────────────────────────────────────────────────────
struct cv_detect_message {
  int16_t  detected;
  int16_t  left_loss;
  int16_t  center_loss;
  int16_t  right_loss;
  int16_t  lowest_loss_dir;   // -1=left, 0=straight, +1=right
  bool     updated;
  struct opticflow_result_t of_result;
  bool     of_updated;
};
struct cv_detect_message global_message[1];

static const char *loss_dir_to_str(int16_t dir) __attribute__((unused));
static const char *loss_dir_to_str(int16_t dir)
{
  if (dir < 0) {
    return "LEFT";
  }
  if (dir > 0) {
    return "RIGHT";
  }
  return "CENTER";
}


// ── Crop helper ──────────────────────────────────────────────────────────────
static void crop_image_center(struct image_t *img, float keep_w_frac, float keep_h_frac)
{
  uint16_t new_w = (uint16_t)(img->w * keep_w_frac);
  uint16_t new_h = (uint16_t)(img->h * keep_h_frac);
  new_w &= ~1u;  // round down to even for YUV422 macro-pixel alignment

  uint16_t x_off = (img->w - new_w) / 2;
  uint16_t y_off = (img->h - new_h) / 2;
  x_off &= ~1u;  // keep even

  uint16_t old_row_bytes = img->w * 2;
  uint16_t new_row_bytes = new_w * 2;
  uint8_t *src = (uint8_t *)img->buf + y_off * old_row_bytes + x_off * 2;
  uint8_t *dst = (uint8_t *)img->buf;

  for (uint16_t r = 0; r < new_h; r++) {
    memmove(dst, src, new_row_bytes);
    dst += new_row_bytes;
    src += old_row_bytes;
  }

  img->w = new_w;
  img->h = new_h;
  img->buf_size = new_w * new_h * 2;
}


/*
 * Cascading obstacle detector callback — called for every new camera frame.
 *
 * Stage 0: crop to horizontal obstacle band (removes floor/ceiling)
 * Stage 1: color detection                  [cheap]
 * Stage 2: edge detection                   [future slot]
 * Stage 3: optical flow                     [expensive, last resort]
 */
static struct image_t *object_detector(struct image_t *img, uint8_t camera_id)
{
  if (camera_id >= 1) return img;
  uint16_t detected_local = 0;
  bool     of_ran = false;

  // --- Stage 0: crop to horizontal obstacle band (removes floor/ceiling) ---
  crop_image_center(img, crop_w_frac, crop_h_frac);

  // --- Stage 1: cheap color detection on cropped band (per-column) ---
  struct column_counts orange_cols = color_detection_columns(img,
      orange_lum_min, orange_lum_max, orange_cb_min, orange_cb_max,
      orange_cr_min, orange_cr_max, false);
  struct column_counts blue_cols = color_detection_columns(img,
      blue_lum_min, blue_lum_max, blue_cb_min, blue_cb_max,
      blue_cr_min, blue_cr_max, false);
  struct column_counts green_cols = color_detection_columns(img,
      green_lum_min, green_lum_max, green_cb_min, green_cb_max,
      green_cr_min, green_cr_max, cod_draw);

  // Combined loss per column (sum all colors)
  uint16_t left_loss   = orange_cols.left   + blue_cols.left   + green_cols.left;
  uint16_t center_loss = orange_cols.center + blue_cols.center + green_cols.center;
  uint16_t right_loss  = orange_cols.right  + blue_cols.right  + green_cols.right;

  // Trigger on CENTER column only (per-color thresholds)
  bool orange_detected = threshold_orange_detector > 0.0f && orange_cols.center >= threshold_orange_detector;
  bool blue_detected   = threshold_blue_detector   > 0.0f && blue_cols.center   >= threshold_blue_detector;
  bool green_detected  = threshold_green_detector  > 0.0f && green_cols.center  >= threshold_green_detector;
  if (orange_detected || blue_detected || green_detected) {
    detected_local = 1;
  }

  // Determine lowest-loss direction across all three columns.
  // Use 0 for "straight/center" and also for ties in the minimum-loss column.
  int16_t lowest_loss_dir = 0;
  uint16_t min_loss = center_loss;
  bool min_is_unique = true;

  if (left_loss < min_loss) {
    min_loss = left_loss;
    lowest_loss_dir = -1;
    min_is_unique = true;
  } else if (left_loss == min_loss) {
    lowest_loss_dir = 0;
    min_is_unique = false;
  }

  if (right_loss < min_loss) {
    min_loss = right_loss;
    lowest_loss_dir = 1;
    min_is_unique = true;
  } else if (right_loss == min_loss) {
    lowest_loss_dir = 0;
    min_is_unique = false;
  }

  if (!min_is_unique) {
    lowest_loss_dir = 0;
  }

  VERBOSE_PRINT("LowestLoss: %s | det:%u | L:%u C:%u R:%u\n",
                loss_dir_to_str(lowest_loss_dir),
                detected_local,
                left_loss, center_loss, right_loss);

  // --- Stage 2: edge detection (future — insert here) ---
  // if (detected_local == 0) {
  //   run edge detection on already-cropped img
  //   if (edges_above_threshold) detected_local = 2;
  // }

  // --- Stage 3: expensive OF, only if stages 1-2 found nothing ---
  static struct opticflow_result_t temp_of_result[LUKE_OF_CAMERA_SLOTS];
  static uint16_t of_skip_count = 0;
  if (detected_local == 0) {
    bool of_reset_this_frame = false;
    // Reset OF tracker if it was idle or the fast controller requested a reset
    if (of_skip_count > 0 || luke_of_request_reset) {
      opticflow_calc_init(luke_of_opticflow);
      luke_of_opticflow[0].show_flow = false;
      memset(&temp_of_result[0], 0, sizeof(temp_of_result[0]));
      of_skip_count = 0;
      of_reset_this_frame = true;
      if (luke_of_request_reset) {
        luke_of_smoothed_divergence = 0.0f;
        luke_of_request_reset = false;
      }
    }

    // crop_image_center(img, crop_w_frac, 1.0f);  // narrow width further for OF
    struct pose_t pose = get_rotation_at_timestamp(img->pprz_ts);
    img->eulers = pose.eulers;
    luke_of_opticflow[0].derotation = luke_of_derotation;

    of_ran = opticflow_calc_frame(&luke_of_opticflow[0], img, &temp_of_result[0]);
    // fprintf(stderr,
    //         "[cv_detect_cb] of_attempt reset=%d success=%d crop=%ux%u fast9=%u got_first=%d corners=%u tracked=%u orange=%u blue=%u green=%u\n",
    //         of_reset_this_frame, of_ran, img->w, img->h, luke_of_opticflow[0].fast9_threshold,
    //         luke_of_opticflow[0].got_first_img, temp_of_result[0].corner_cnt,
    //         temp_of_result[0].tracked_cnt, orange_count, blue_count, green_count);
  } else {
    of_skip_count++;
  }

  // --- Store results ---
  pthread_mutex_lock(&mutex);
  global_message[camera_id].detected        = detected_local;
  global_message[camera_id].left_loss       = left_loss;
  global_message[camera_id].center_loss     = center_loss;
  global_message[camera_id].right_loss      = right_loss;
  global_message[camera_id].lowest_loss_dir = lowest_loss_dir;
  global_message[camera_id].updated         = true;
  if (of_ran) {
    global_message[camera_id].of_result  = temp_of_result[0];
    global_message[camera_id].of_updated = true;
  }
  pthread_mutex_unlock(&mutex);

  return img;
}


/*
 * Wrapper so cv_add_to_device always calls with camera_id=0.
 */
struct image_t *object_detector1(struct image_t *img, uint8_t camera_id);
struct image_t *object_detector1(struct image_t *img, uint8_t camera_id __attribute__((unused)))
{
  return object_detector(img, 0);
}


/*
 * Called once at startup.
 */
void MAV_cv_detect_group12_cmjong_init(void)
{
  memset(global_message, 0, 1*sizeof(struct cv_detect_message));
  pthread_mutex_init(&mutex, NULL);
#ifdef COLOR_OBJECT_DETECTOR_CAMERA
#ifdef ORANGE_OBJECT_DETECTOR_LUM_MIN
  orange_lum_min = ORANGE_OBJECT_DETECTOR_LUM_MIN;
  orange_lum_max = ORANGE_OBJECT_DETECTOR_LUM_MAX;
  orange_cb_min  = ORANGE_OBJECT_DETECTOR_CB_MIN;
  orange_cb_max  = ORANGE_OBJECT_DETECTOR_CB_MAX;
  orange_cr_min  = ORANGE_OBJECT_DETECTOR_CR_MIN;
  orange_cr_max  = ORANGE_OBJECT_DETECTOR_CR_MAX;
#endif

#ifdef BLUE_OBJECT_DETECTOR_LUM_MIN
  blue_lum_min = BLUE_OBJECT_DETECTOR_LUM_MIN;
  blue_lum_max = BLUE_OBJECT_DETECTOR_LUM_MAX;
  blue_cb_min  = BLUE_OBJECT_DETECTOR_CB_MIN;
  blue_cb_max  = BLUE_OBJECT_DETECTOR_CB_MAX;
  blue_cr_min  = BLUE_OBJECT_DETECTOR_CR_MIN;
  blue_cr_max  = BLUE_OBJECT_DETECTOR_CR_MAX;
#endif

#ifdef GREEN_OBJECT_DETECTOR_LUM_MIN
  green_lum_min = GREEN_OBJECT_DETECTOR_LUM_MIN;
  green_lum_max = GREEN_OBJECT_DETECTOR_LUM_MAX;
  green_cb_min  = GREEN_OBJECT_DETECTOR_CB_MIN;
  green_cb_max  = GREEN_OBJECT_DETECTOR_CB_MAX;
  green_cr_min  = GREEN_OBJECT_DETECTOR_CR_MIN;
  green_cr_max  = GREEN_OBJECT_DETECTOR_CR_MAX;
#endif

#ifdef THRESHOLD_ORANGE_DETECTOR
  threshold_orange_detector = THRESHOLD_ORANGE_DETECTOR;
#endif
#ifdef THRESHOLD_BLUE_DETECTOR
  threshold_blue_detector = THRESHOLD_BLUE_DETECTOR;
#endif
#ifdef THRESHOLD_GREEN_DETECTOR
  threshold_green_detector = THRESHOLD_GREEN_DETECTOR;
#endif

#ifdef COLOR_OBJECT_DETECTOR_DRAW
  cod_draw = COLOR_OBJECT_DETECTOR_DRAW;
#endif

  // Initialize optical flow calculator
  opticflow_calc_init(luke_of_opticflow);
  luke_of_opticflow[0].show_flow = false;
  luke_of_opticflow[0].derotation = luke_of_derotation;

  cv_add_to_device(&COLOR_OBJECT_DETECTOR_CAMERA, object_detector1, COLOR_OBJECT_DETECTOR_FPS1, 0);
#endif
}


/*
 * Periodic function — sends ABI messages to the fast controller.
 */
void MAV_cv_detect_group12_cmjong_periodic(void)
{
  static struct cv_detect_message local_message[1];
  pthread_mutex_lock(&mutex);
  memcpy(local_message, global_message, 1*sizeof(struct cv_detect_message));
  global_message[0].updated    = false;
  global_message[0].of_updated = false;
  pthread_mutex_unlock(&mutex);

  // Color detection ABI message: detected, left_loss, center_loss, right_loss, lowest_loss_dir
  if(local_message[0].updated){
    AbiSendMsgVISUAL_DETECTION(COLOR_OBJECT_DETECTION1_ID,
    local_message[0].detected,
    local_message[0].left_loss,
    local_message[0].center_loss,
    local_message[0].right_loss,
    (int32_t)local_message[0].lowest_loss_dir, 0);
  }

  // Optical flow ABI message (with EMA smoothing)
  const char *ema_branch;
  if (local_message[0].of_updated) {
    // OF ran this frame — update EMA with new sample
    float raw_div = local_message[0].of_result.div_size;
    float clamped = raw_div > 0.0f ? raw_div : 0.0f;
    luke_of_smoothed_divergence = luke_of_ema_alpha * clamped
                                + (1.0f - luke_of_ema_alpha) * luke_of_smoothed_divergence;
    ema_branch = "OF_UPDATE";
  } else if (local_message[0].updated) {
    // Camera frame processed but no fresh OF result (OF failed or was skipped).
    // Apply slow per-frame decay so stale divergence clears in ~2-3 seconds
    // without the aggressive 50 Hz decay that killed the signal before.
    luke_of_smoothed_divergence *= 0.95f;
    ema_branch = "SLOW_DECAY";
  } else {
    // no camera frame this tick — hold current value (no spurious decay)
    ema_branch = "HOLD";
  }

  int32_t quality = (luke_of_smoothed_divergence > luke_of_divergence_threshold) ? 1 : 0;
  AbiSendMsgVISUAL_DETECTION(LUKE_OF_VISUAL_DETECTION_ID, 0, 0, 0, 0, quality, 0);

  // fprintf(stderr, "[cv_detect] %s L=%d C=%d R=%d dir=%d div_raw=%.4f smoothed=%.4f thresh=%.4f q=%d tracked=%d\n",
  //   ema_branch,
  //   local_message[0].left_loss, local_message[0].center_loss, local_message[0].right_loss,
  //   local_message[0].lowest_loss_dir,
  //   local_message[0].of_updated ? local_message[0].of_result.div_size : -1.f,
  //   luke_of_smoothed_divergence, luke_of_divergence_threshold, quality,
  //   local_message[0].of_updated ? (int)local_message[0].of_result.tracked_cnt : -1);
}
