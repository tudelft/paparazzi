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
#include "state.h"
}

#include <math.h>
#include <pthread.h>
#include <string.h>

#ifndef PLANT_AVOIDER_CAMERA
#define PLANT_AVOIDER_CAMERA front_camera
#endif

#ifndef PLANT_AVOIDER_FPS
#define PLANT_AVOIDER_FPS 4
#endif

#ifndef PLANT_AVOIDER_SHOW_MASK
#define PLANT_AVOIDER_SHOW_MASK 1
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

/* Bebop camera feed is rotated in this setup: selecting x < w/2 maps to top half in viewer. */
#ifndef PLANT_AVOIDER_ROTATED_CAMERA_TOP_HALF
#define PLANT_AVOIDER_ROTATED_CAMERA_TOP_HALF 1
#endif

#ifndef PLANT_AVOIDER_STRAIGHT_SIDE_BONUS
#define PLANT_AVOIDER_STRAIGHT_SIDE_BONUS 0.05f
#endif

#ifndef PLANT_AVOIDER_RIGHT_TIE_BONUS
#define PLANT_AVOIDER_RIGHT_TIE_BONUS 0.02f
#endif

/*
 * CMJong-style control rule:
 * - If the middle/top-forward sector is below this raw loss threshold, fly straight.
 * - Otherwise, turn to the side with the lowest green load.
 */
#ifndef PLANT_AVOIDER_LOSS_SAFE_THRESHOLD
#define PLANT_AVOIDER_LOSS_SAFE_THRESHOLD 10000U
#endif

/* Green range in YUV: [Y, U(Cb), V(Cr)] */
#ifndef PLANT_AVOIDER_Y_MIN
#ifdef GREEN_OBJECT_DETECTOR_LUM_MIN
#define PLANT_AVOIDER_Y_MIN GREEN_OBJECT_DETECTOR_LUM_MIN
#else
#define PLANT_AVOIDER_Y_MIN 50
#endif
#endif
#ifndef PLANT_AVOIDER_Y_MAX
#ifdef GREEN_OBJECT_DETECTOR_LUM_MAX
#define PLANT_AVOIDER_Y_MAX GREEN_OBJECT_DETECTOR_LUM_MAX
#else
#define PLANT_AVOIDER_Y_MAX 255
#endif
#endif
#ifndef PLANT_AVOIDER_U_MIN
#ifdef GREEN_OBJECT_DETECTOR_CB_MIN
#define PLANT_AVOIDER_U_MIN GREEN_OBJECT_DETECTOR_CB_MIN
#else
#define PLANT_AVOIDER_U_MIN 100
#endif
#endif
#ifndef PLANT_AVOIDER_U_MAX
#ifdef GREEN_OBJECT_DETECTOR_CB_MAX
#define PLANT_AVOIDER_U_MAX GREEN_OBJECT_DETECTOR_CB_MAX
#else
#define PLANT_AVOIDER_U_MAX 150
#endif
#endif
#ifndef PLANT_AVOIDER_V_MIN
#ifdef GREEN_OBJECT_DETECTOR_CR_MIN
#define PLANT_AVOIDER_V_MIN GREEN_OBJECT_DETECTOR_CR_MIN
#else
#define PLANT_AVOIDER_V_MIN 0
#endif
#endif
#ifndef PLANT_AVOIDER_V_MAX
#ifdef GREEN_OBJECT_DETECTOR_CR_MAX
#define PLANT_AVOIDER_V_MAX GREEN_OBJECT_DETECTOR_CR_MAX
#else
#define PLANT_AVOIDER_V_MAX 128
#endif
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

static void detect_green_top_half(struct image_t *img, struct pa_zone_scores_t *out)
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
  if (x < (w / 2U)) {
        continue;
      }
#else
      if (y >= (h / 2U)) {
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
      /* Safe overlay: adjust luma only, keep shared U/V bytes untouched. */
      if ((x & 1U) == 0U) {
        const uint32_t base = row_base + (uint32_t)(2U * x);
        buf[base + 1U] = 235U;  /* Y pixel x */
      } else {
        const uint32_t base = row_base + (uint32_t)(2U * x);
        buf[base + 1U] = 235U;  /* Y pixel x */
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

static struct image_t *plant_avoider_func(struct image_t *img, uint8_t camera_id)
{
  (void)camera_id;
  if (!img || !img->buf || img->type != IMAGE_YUV422) {
    return img;
  }

  struct pa_zone_scores_t s;

  /* Debug behavior: always run detection/highlighting, even on ground. */
  detect_green_top_half(img, &s);

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

  cv_add_to_device(&PLANT_AVOIDER_CAMERA, plant_avoider_func, PLANT_AVOIDER_FPS, 0);
}

extern "C" void plant_avoider_periodic(void)
{
  const bool grounded = is_drone_near_ground();
  const bool guided_mode = (guidance_h.mode == GUIDANCE_H_MODE_GUIDED);

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
    const bool middle_is_safe = (s.straight < PLANT_AVOIDER_LOSS_SAFE_THRESHOLD);
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

  if (direction < 0) {
    vy = -pa_turn_speed;
  } else if (direction > 0) {
    vy = pa_turn_speed;
  }

  if (guided_mode && !grounded) {
    guidance_h_set_body_vel(vx, vy);
  }
}
