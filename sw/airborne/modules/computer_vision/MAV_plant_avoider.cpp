/**
 * @file MAV_plant_avoider.cpp
 * @brief Simple YUV-based plant avoider.
 *
 * Logic:
 *  - Detect green pixels directly in YUV422.
 *  - Only process pixels in the top half of the image.
 *  - Pick steering direction toward least green.
 *  - Priority/tie-break: straight first, then right.
 *  - If near ground, ignore detections.
 */

#include "MAV_plant_avoider.h"

extern "C" {
#include "modules/computer_vision/cv.h"
#include "modules/computer_vision/lib/vision/image.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "firmwares/rotorcraft/navigation.h"
#include "modules/datalink/downlink.h"
#include "state.h"
}

#include <math.h>
#include <pthread.h>
#include <stdio.h>
#include <string.h>

#ifndef PLANT_AVOIDER_CAMERA
#define PLANT_AVOIDER_CAMERA front_camera
#endif

#ifndef PLANT_AVOIDER_FPS
#define PLANT_AVOIDER_FPS 4
#endif

/* Temporary debug: print sampled YUV pixel values from camera stream. */
#ifndef PLANT_AVOIDER_DEBUG_YUV
#define PLANT_AVOIDER_DEBUG_YUV 1
#endif

#ifndef PLANT_AVOIDER_DEBUG_YUV_PERIOD_FRAMES
#define PLANT_AVOIDER_DEBUG_YUV_PERIOD_FRAMES 10U
#endif

#ifndef PLANT_AVOIDER_DEBUG_YUV_TO_GCS
#define PLANT_AVOIDER_DEBUG_YUV_TO_GCS 1
#endif

#ifndef PLANT_AVOIDER_DEBUG_YUV_STDOUT
#define PLANT_AVOIDER_DEBUG_YUV_STDOUT 0
#endif

/* In NAV mode, route guidance can overwrite speed setpoints.
 * Enable this to let avoider commands drive horizontal guidance in simulation. */
#ifndef PLANT_AVOIDER_NAV_MODE_CONTROL
#define PLANT_AVOIDER_NAV_MODE_CONTROL 0
#endif

/* When 0, this module does not send guidance commands. */
#ifndef PLANT_AVOIDER_ENABLE_STANDALONE_CONTROL
#define PLANT_AVOIDER_ENABLE_STANDALONE_CONTROL 0
#endif

/* Keep camera processing/highlighting active even if control is disabled. */
#ifndef PLANT_AVOIDER_ENABLE_VISION_CALLBACK
#define PLANT_AVOIDER_ENABLE_VISION_CALLBACK 1
#endif

#ifndef PLANT_AVOIDER_SHOW_MASK
#define PLANT_AVOIDER_SHOW_MASK 0
#endif

/* YUV color used to render the debug mask in the video stream. */
#ifndef PLANT_AVOIDER_MASK_Y
#define PLANT_AVOIDER_MASK_Y 145U
#endif
#ifndef PLANT_AVOIDER_MASK_U
#define PLANT_AVOIDER_MASK_U 54U
#endif
#ifndef PLANT_AVOIDER_MASK_V
#define PLANT_AVOIDER_MASK_V 34U
#endif

#ifndef PLANT_AVOIDER_GROUND_ALT_M
#define PLANT_AVOIDER_GROUND_ALT_M 0.50f
#endif

#ifndef PLANT_AVOIDER_FORWARD_SPEED
#define PLANT_AVOIDER_FORWARD_SPEED 0.3f
#endif

#ifndef PLANT_AVOIDER_TURN_SPEED
#define PLANT_AVOIDER_TURN_SPEED 0.3f
#endif

#ifndef PLANT_AVOIDER_STRAIGHT_BIAS
#define PLANT_AVOIDER_STRAIGHT_BIAS 0.10f
#endif

/* Gazebo front camera is typically not rotated; set to 1 only for rotated feeds (e.g. some Bebop setups). */
#ifndef PLANT_AVOIDER_ROTATED_CAMERA_TOP_HALF
#define PLANT_AVOIDER_ROTATED_CAMERA_TOP_HALF 1
#endif

/* Set to 1 to keep legacy half-frame ROI, 0 to detect/draw across the full frame. */
#ifndef PLANT_AVOIDER_USE_HALF_FOV
#define PLANT_AVOIDER_USE_HALF_FOV 1
#endif

#ifndef PLANT_AVOIDER_STRAIGHT_SIDE_BONUS
#define PLANT_AVOIDER_STRAIGHT_SIDE_BONUS 0.05f
#endif

#ifndef PLANT_AVOIDER_RIGHT_TIE_BONUS
#define PLANT_AVOIDER_RIGHT_TIE_BONUS 0.02f
#endif

/*
 * Control rule based on straight-sector load percentage (resolution independent):
 * - If straight load is below safe percent, fly straight.
 * - Otherwise, turn to the side with the lower load.
 */
#ifndef PLANT_AVOIDER_STRAIGHT_SAFE_PCT
#define PLANT_AVOIDER_STRAIGHT_SAFE_PCT 40.0f
#endif

/* Emergency behavior when center is heavily blocked: stop forward motion while turning. */
#ifndef PLANT_AVOIDER_STRAIGHT_BLOCKED_PCT
#define PLANT_AVOIDER_STRAIGHT_BLOCKED_PCT 60.0f
#endif

/* Temporary calibration target: Y=90 U=100 V=124 */
#ifndef PLANT_AVOIDER_Y_CENTER
#define PLANT_AVOIDER_Y_CENTER 90
#endif
#ifndef PLANT_AVOIDER_U_CENTER
#define PLANT_AVOIDER_U_CENTER 100
#endif
#ifndef PLANT_AVOIDER_V_CENTER
#define PLANT_AVOIDER_V_CENTER 124
#endif

/* Tolerance window around sampled YUV values for robust matching. */
#ifndef PLANT_AVOIDER_Y_TOL
#define PLANT_AVOIDER_Y_TOL 20
#endif
#ifndef PLANT_AVOIDER_U_TOL
#define PLANT_AVOIDER_U_TOL 20
#endif
#ifndef PLANT_AVOIDER_V_TOL
#define PLANT_AVOIDER_V_TOL 20
#endif

#ifndef PLANT_AVOIDER_Y_MIN
#define PLANT_AVOIDER_Y_MIN (PLANT_AVOIDER_Y_CENTER - PLANT_AVOIDER_Y_TOL)
#endif
#ifndef PLANT_AVOIDER_Y_MAX
#define PLANT_AVOIDER_Y_MAX (PLANT_AVOIDER_Y_CENTER + PLANT_AVOIDER_Y_TOL)
#endif
#ifndef PLANT_AVOIDER_U_MIN
#define PLANT_AVOIDER_U_MIN (PLANT_AVOIDER_U_CENTER - PLANT_AVOIDER_U_TOL)
#endif
#ifndef PLANT_AVOIDER_U_MAX
#define PLANT_AVOIDER_U_MAX (PLANT_AVOIDER_U_CENTER + PLANT_AVOIDER_U_TOL)
#endif
#ifndef PLANT_AVOIDER_V_MIN
#define PLANT_AVOIDER_V_MIN (PLANT_AVOIDER_V_CENTER - PLANT_AVOIDER_V_TOL)
#endif
#ifndef PLANT_AVOIDER_V_MAX
#define PLANT_AVOIDER_V_MAX (PLANT_AVOIDER_V_CENTER + PLANT_AVOIDER_V_TOL)
#endif

float pa_straight_bias = PLANT_AVOIDER_STRAIGHT_BIAS;
float pa_forward_speed = PLANT_AVOIDER_FORWARD_SPEED;
float pa_turn_speed = PLANT_AVOIDER_TURN_SPEED;

float pa_load_left = 0.0f;
float pa_load_straight = 0.0f;
float pa_load_right = 0.0f;
int8_t pa_last_direction = 0;
float pa_weight_left = 0.0f;
float pa_weight_straight = 1.0f;
float pa_weight_right = 0.0f;

struct pa_zone_scores_t {
  uint32_t left;
  uint32_t straight;
  uint32_t right;
  uint32_t total;
};

static struct pa_zone_scores_t g_scores;
static pthread_mutex_t g_mutex;
static uint32_t g_debug_yuv_frame_count = 0U;

static bool yuv422_get_pixel(const struct image_t *img, uint16_t x, uint16_t y,
                             uint8_t *y_out, uint8_t *u_out, uint8_t *v_out)
{
  if (!img || !img->buf || img->type != IMAGE_YUV422) {
    return false;
  }
  if (x >= img->w || y >= img->h) {
    return false;
  }

  uint8_t *buf = (uint8_t *)img->buf;
  const uint32_t row_base = (uint32_t)y * 2U * (uint32_t)img->w;
  const uint32_t base = row_base + (uint32_t)(2U * x);

  if ((x & 1U) == 0U) {
    *u_out = buf[base];
    *y_out = buf[base + 1U];
    *v_out = buf[base + 2U];
  } else {
    *u_out = buf[base - 2U];
    *y_out = buf[base + 1U];
    *v_out = buf[base];
  }

  return true;
}

static inline bool is_green_yuv(uint8_t y, uint8_t u, uint8_t v)
{
  return (y >= PLANT_AVOIDER_Y_MIN && y <= PLANT_AVOIDER_Y_MAX &&
          u >= PLANT_AVOIDER_U_MIN && u <= PLANT_AVOIDER_U_MAX &&
          v >= PLANT_AVOIDER_V_MIN && v <= PLANT_AVOIDER_V_MAX);
}

static bool is_drone_near_ground(void)
{
  return stateGetPositionEnu_f()->z <= PLANT_AVOIDER_GROUND_ALT_M;
}

static void detect_green_top_half(struct image_t *img, bool draw_mask, struct pa_zone_scores_t *out)
{
  memset(out, 0, sizeof(*out));

  const uint16_t w = img->w;
  const uint16_t h = img->h;
  const uint16_t one_third_y = h / 3;
#if !PLANT_AVOIDER_ROTATED_CAMERA_TOP_HALF
  const uint16_t one_third_x = w / 3;
#endif
  uint8_t *buf = (uint8_t*)img->buf;

  for (uint16_t y = 0; y < h; y++) {
    const uint32_t row_base = (uint32_t)y * 2U * (uint32_t)w;
    for (uint16_t x = 0; x < w; x++) {
#if PLANT_AVOIDER_ROTATED_CAMERA_TOP_HALF
  if (PLANT_AVOIDER_USE_HALF_FOV && x < (w / 2U)) {
        continue;
      }
#else
  if (PLANT_AVOIDER_USE_HALF_FOV && y >= (h / 2U)) {
        continue;
      }
#endif

      uint8_t yp;
      uint8_t up;
      uint8_t vp;

      if ((x & 1U) == 0U) {
        const uint32_t base = row_base + (uint32_t)(2U * x);
        up = buf[base];
        yp = buf[base + 1U];
        vp = buf[base + 2U];
      } else {
        const uint32_t base = row_base + (uint32_t)(2U * x);
        up = buf[base - 2U];
        yp = buf[base + 1U];
        vp = buf[base];
      }

      if (!is_green_yuv(yp, up, vp)) {
        continue;
      }

#if PLANT_AVOIDER_SHOW_MASK
      if (draw_mask) {
        /* YUV422 packs chroma per 2-pixel pair: [U Y0 V Y1]. */
        const uint16_t x_pair = (uint16_t)(x & (uint16_t)~1U);
        const uint32_t pair_base = row_base + (uint32_t)(2U * x_pair);
        const uint32_t row_end = row_base + (uint32_t)(2U * w);
        if ((pair_base + 3U) < row_end) {
          buf[pair_base] = PLANT_AVOIDER_MASK_U;
          buf[pair_base + 2U] = PLANT_AVOIDER_MASK_V;
          if ((x & 1U) == 0U) {
            buf[pair_base + 1U] = PLANT_AVOIDER_MASK_Y;
          } else {
            buf[pair_base + 3U] = PLANT_AVOIDER_MASK_Y;
          }
        }
      }
#endif

#if PLANT_AVOIDER_ROTATED_CAMERA_TOP_HALF
      if (y < one_third_y) {
        out->left++;
      } else if (y < (2U * one_third_y)) {
        out->straight++;
      } else {
        out->right++;
      }
#else
      if (x < one_third_x) {
        out->left++;
      } else if (x < (2U * one_third_x)) {
        out->straight++;
      } else {
        out->right++;
      }
#endif
      out->total++;
    }
  }
}

extern "C" void plant_avoider_detect_losses(struct image_t *img, bool draw_mask,
                                              uint32_t *left, uint32_t *straight, uint32_t *right)
{
  struct pa_zone_scores_t s;
  detect_green_top_half(img, draw_mask, &s);

  if (left) {
    *left = s.left;
  }
  if (straight) {
    *straight = s.straight;
  }
  if (right) {
    *right = s.right;
  }
}

static struct image_t *plant_avoider_func(struct image_t *img, uint8_t camera_id)
{
  (void)camera_id;
  if (!img || !img->buf || img->type != IMAGE_YUV422) {
    return img;
  }

  struct pa_zone_scores_t s;

  /* Debug behavior: always run detection/highlighting, even on ground. */
  detect_green_top_half(img, true, &s);

#if PLANT_AVOIDER_DEBUG_YUV
  uint8_t py, pu, pv;
  const uint16_t sx = (uint16_t)(img->w / 2U);
  const uint16_t sy = (uint16_t)(img->h / 2U);
  if (yuv422_get_pixel(img, sx, sy, &py, &pu, &pv)) {
    if ((g_debug_yuv_frame_count % PLANT_AVOIDER_DEBUG_YUV_PERIOD_FRAMES) == 0U) {
#if PLANT_AVOIDER_DEBUG_YUV_TO_GCS
      float yuv_msg[5];
      yuv_msg[0] = (float)sx;
      yuv_msg[1] = (float)sy;
      yuv_msg[2] = (float)py;
      yuv_msg[3] = (float)pu;
      yuv_msg[4] = (float)pv;
      DOWNLINK_SEND_PAYLOAD_FLOAT(DefaultChannel, DefaultDevice, 5, yuv_msg);
#endif
#if PLANT_AVOIDER_DEBUG_YUV_STDOUT
      printf("[plant_avoider] sample pixel x=%u y=%u -> Y=%u U=%u V=%u\n",
             (unsigned)sx, (unsigned)sy, (unsigned)py, (unsigned)pu, (unsigned)pv);
#endif
    }
    g_debug_yuv_frame_count++;
  }
#endif

  pthread_mutex_lock(&g_mutex);
  g_scores = s;
  pthread_mutex_unlock(&g_mutex);

  return img;
}

extern "C" void plant_avoider_init(void)
{
  memset(&g_scores, 0, sizeof(g_scores));
  pthread_mutex_init(&g_mutex, NULL);

  pa_straight_bias = PLANT_AVOIDER_STRAIGHT_BIAS;
  pa_forward_speed = PLANT_AVOIDER_FORWARD_SPEED;
  pa_turn_speed = PLANT_AVOIDER_TURN_SPEED;

#if PLANT_AVOIDER_ENABLE_VISION_CALLBACK
  cv_add_to_device(&PLANT_AVOIDER_CAMERA, plant_avoider_func, PLANT_AVOIDER_FPS, 0);
#endif
}

extern "C" void plant_avoider_periodic(void)
{
#if !PLANT_AVOIDER_ENABLE_STANDALONE_CONTROL
  return;
#else
  const bool grounded = is_drone_near_ground();
  const bool control_mode_active = (guidance_h.mode == GUIDANCE_H_MODE_GUIDED) ||
                                   (guidance_h.mode == GUIDANCE_H_MODE_NAV);

  struct pa_zone_scores_t s;
  pthread_mutex_lock(&g_mutex);
  s = g_scores;
  pthread_mutex_unlock(&g_mutex);

  if (s.total > 0U && !grounded) {
    pa_load_left = (100.0f * (float)s.left) / (float)s.total;
    pa_load_straight = (100.0f * (float)s.straight) / (float)s.total;
    pa_load_right = (100.0f * (float)s.right) / (float)s.total;
  } else {
    pa_load_left = 0.0f;
    pa_load_straight = 0.0f;
    pa_load_right = 0.0f;
  }

  int8_t direction = 0;
  if (!grounded && s.total > 0U) {
    const bool middle_is_safe = (pa_load_straight < PLANT_AVOIDER_STRAIGHT_SAFE_PCT);
    const bool left_is_best = (pa_load_left < pa_load_straight) &&
                              (pa_load_left < pa_load_right);

    if (middle_is_safe) {
      direction = 0;
    } else if (left_is_best) {
      direction = -1;
    } else {
      /* Match CMJong tie behavior: if not middle and not left, choose right. */
      direction = +1;
    }
  }

  pa_weight_left = (direction < 0) ? 1.0f : 0.0f;
  pa_weight_straight = (direction == 0) ? 1.0f : 0.0f;
  pa_weight_right = (direction > 0) ? 1.0f : 0.0f;

  pa_last_direction = direction;

  float vx = pa_forward_speed;
  float vy = 0.0f;

  if (direction != 0) {
    vx = 0.0f;
  }

  if (direction < 0) {
    vy = -pa_turn_speed;
  } else if (direction > 0) {
    vy = pa_turn_speed;
  }

  if (control_mode_active && !grounded) {
#if PLANT_AVOIDER_NAV_MODE_CONTROL
    if (guidance_h.mode == GUIDANCE_H_MODE_NAV) {
      nav.horizontal_mode = NAV_HORIZONTAL_MODE_GUIDED;
      nav.setpoint_mode = NAV_SETPOINT_MODE_SPEED;
    }
#endif
    guidance_h_set_body_vel(vx, vy);
  }
#endif
}
