/*
 * MAV Course 2026 - Luke Optical Flow Module
 *
 * Thin obstacle-detection wrapper around Paparazzi's opticflow_calculator.
 * The standard calculator handles corner detection, Lucas-Kanade tracking,
 * feature management, derotation, and size-divergence estimation. This module
 * maps the resulting size divergence to a simple VISUAL_DETECTION quality flag
 * and provides a lightweight RTP debug overlay.
 */

#include "luke_optical_flow.h"

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <pthread.h>

#include "generated/airframe.h"
#include "modules/core/abi.h"
#include "modules/computer_vision/cv.h"
#include "modules/computer_vision/lib/vision/image.h"
#include "modules/computer_vision/opticflow/opticflow_calculator.h"
#include "modules/pose_history/pose_history.h"

// ABI sender ID for outgoing VISUAL_DETECTION messages.
#ifndef LUKE_OF_VISUAL_DETECTION_ID
#define LUKE_OF_VISUAL_DETECTION_ID ABI_BROADCAST
#endif

// Luke wraps a single opticflow calculator instance for one forward camera.
#define LUKE_OF_CAMERA_SLOTS 1
#define LUKE_OF_OVERLAY_POINTS 64

// Divergence above this value triggers quality=1 (obstacle detected).
float luke_of_divergence_threshold = 0.01f;
bool luke_of_show_stream_overlay = false;
bool luke_of_derotation = true;

// EMA smoothing: smoothed = alpha * new_sample + (1-alpha) * smoothed.
float luke_of_ema_alpha = 0.5f;
float luke_of_smoothed_divergence = 0.0f;

// Exposed so the GCS can tune selected standard opticflow parameters.
struct opticflow_t luke_of_opticflow[LUKE_OF_CAMERA_SLOTS];

static struct opticflow_result_t luke_of_result;
static bool luke_of_got_result = false;
static pthread_mutex_t luke_of_mutex;

struct luke_of_stream_debug_snapshot {
  bool valid;
  uint8_t camera_id;
  float divergence;
  bool threshold_crossed;
  uint16_t point_cnt;
  struct point_t points[LUKE_OF_OVERLAY_POINTS];
};
static struct luke_of_stream_debug_snapshot luke_of_stream_debug;

static struct image_t *luke_optical_flow_process(struct image_t *img, uint8_t camera_id)
{
  if (img == NULL || img->buf == NULL || camera_id >= LUKE_OF_CAMERA_SLOTS) {
    return img;
  }

  // Match the standard opticflow module: use the pose closest to the image timestamp.
  struct pose_t pose = get_rotation_at_timestamp(img->pprz_ts);
  img->eulers = pose.eulers;

  luke_of_opticflow[camera_id].derotation = luke_of_derotation;

  // Static result keeps the calculator's feature-management state across frames.
  static struct opticflow_result_t temp_result[LUKE_OF_CAMERA_SLOTS];
  if (opticflow_calc_frame(&luke_of_opticflow[camera_id], img, &temp_result[camera_id])) {
    pthread_mutex_lock(&luke_of_mutex);
    luke_of_result = temp_result[camera_id];
    luke_of_got_result = true;

    luke_of_stream_debug.valid = true;
    luke_of_stream_debug.camera_id = camera_id;
    luke_of_stream_debug.divergence = temp_result[camera_id].div_size;
    luke_of_stream_debug.threshold_crossed =
      (temp_result[camera_id].div_size > luke_of_divergence_threshold);

    uint16_t snapshot_points = temp_result[camera_id].tracked_cnt;
    if (snapshot_points > LUKE_OF_OVERLAY_POINTS) {
      snapshot_points = LUKE_OF_OVERLAY_POINTS;
    }
    luke_of_stream_debug.point_cnt = snapshot_points;
    if (snapshot_points > 0 && luke_of_opticflow[camera_id].fast9_ret_corners != NULL) {
      memcpy(luke_of_stream_debug.points, luke_of_opticflow[camera_id].fast9_ret_corners,
             snapshot_points * sizeof(struct point_t));
    }
    pthread_mutex_unlock(&luke_of_mutex);
  } else {
    pthread_mutex_lock(&luke_of_mutex);
    luke_of_stream_debug.valid = false;
    pthread_mutex_unlock(&luke_of_mutex);
  }

  return img;
}

void luke_optical_flow_annotate_stream(struct image_t *img, uint8_t camera_id)
{
#if !(defined(USE_NPS) && USE_NPS)
  (void)img;
  (void)camera_id;
  return;
#else
  if (img == NULL || img->buf == NULL || img->w == 0 || img->h == 0 || !luke_of_show_stream_overlay) {
    return;
  }

  struct luke_of_stream_debug_snapshot snapshot;
  pthread_mutex_lock(&luke_of_mutex);
  snapshot = luke_of_stream_debug;
  pthread_mutex_unlock(&luke_of_mutex);

  if (!snapshot.valid || snapshot.camera_id != camera_id) {
    return;
  }

  uint8_t green[4] = {90, 150, 90, 150};
  uint8_t red[4] = {90, 76, 240, 76};
  uint8_t yellow[4] = {16, 220, 146, 220};
  uint8_t *overlay_color = snapshot.point_cnt > 0 ?
                           (snapshot.threshold_crossed ? red : green) :
                           yellow;

  image_draw_rectangle(img, 0, img->w - 1, 0, img->h - 1, overlay_color);
  if (snapshot.point_cnt > 0) {
    image_show_points_color(img, snapshot.points, snapshot.point_cnt, overlay_color);
  }
#endif
}

void luke_optical_flow_init(void)
{
  pthread_mutex_init(&luke_of_mutex, NULL);
  memset(&luke_of_result, 0, sizeof(luke_of_result));
  memset(&luke_of_stream_debug, 0, sizeof(luke_of_stream_debug));

  opticflow_calc_init(luke_of_opticflow);
  luke_of_opticflow[0].show_flow = false;
  luke_of_opticflow[0].derotation = luke_of_derotation;

  cv_add_to_device(&OPTICFLOW_CAMERA, luke_optical_flow_process, 0, 0);
}

void luke_optical_flow_periodic(void)
{
  struct opticflow_result_t local_result;
  bool got;

  pthread_mutex_lock(&luke_of_mutex);
  local_result = luke_of_result;
  got = luke_of_got_result;
  luke_of_got_result = false;
  pthread_mutex_unlock(&luke_of_mutex);

  if (!got) {
    return;
  }

  // Clamp negative divergence to zero: we only care about expansion (approaching).
  // Negative values from turns/noise would drag the EMA baseline down and delay detection.
  float clamped_div = local_result.div_size > 0.0f ? local_result.div_size : 0.0f;

  // Exponential moving average to smooth noisy per-frame divergence.
  luke_of_smoothed_divergence = luke_of_ema_alpha * clamped_div
                              + (1.0f - luke_of_ema_alpha) * luke_of_smoothed_divergence;

  printf("[luke_of] div=%.4f smooth=%.4f thr=%.3f tracked=%d max_corners=%d\n",
         local_result.div_size,
         luke_of_smoothed_divergence,
         luke_of_divergence_threshold,
         local_result.tracked_cnt,
         luke_of_opticflow[0].max_track_corners);

  int32_t quality = (luke_of_smoothed_divergence > luke_of_divergence_threshold) ? 1 : 0;
  AbiSendMsgVISUAL_DETECTION(LUKE_OF_VISUAL_DETECTION_ID,
                             0, 0, 0, 0, quality, 0);
}
