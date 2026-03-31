/**
 * @file cv_detect_color_object.c
 * @brief Color-based object detection with ROI filtering.
 *
 * This module detects colored objects using YCbCr thresholds and
 * region-of-interest (ROI) filtering.
 *
 * Filters:
 *  - Filter 1 & 2: lower trapezoid (ground-level detection)
 *  - Filter 3 & 4: upper rectangle (forward/elevated detection)
 *
 * Outputs are sent via ABI messages to other modules (e.g., avoidance).
 *
 * Design goals:
 *  - Real-time performance
 *  - Reduced noise via ROI restriction
 *  - Modular multi-filter architecture
 */
#include "modules/computer_vision/cv_detect_color_object.h"
#include "modules/computer_vision/cv.h"
#include "modules/core/abi.h"
#include "std.h"

#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>
#include "pthread.h"

#define PRINT(string,...) fprintf(stderr, "[object_detector->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if OBJECT_DETECTOR_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

static pthread_mutex_t mutex;

#ifndef COLOR_OBJECT_DETECTOR_FPS1
#define COLOR_OBJECT_DETECTOR_FPS1 0
#endif
#ifndef COLOR_OBJECT_DETECTOR_FPS2
#define COLOR_OBJECT_DETECTOR_FPS2 0
#endif
#ifndef COLOR_OBJECT_DETECTOR_FPS3
#define COLOR_OBJECT_DETECTOR_FPS3 0
#endif
#ifndef COLOR_OBJECT_DETECTION3_ID
#define COLOR_OBJECT_DETECTION3_ID 30
#endif
#ifndef COLOR_OBJECT_DETECTION4_ID
#define COLOR_OBJECT_DETECTION4_ID 4
#endif

/* Filter settings */
uint8_t cod_lum_min1 = 0, cod_lum_max1 = 0, cod_cb_min1 = 0, cod_cb_max1 = 0, cod_cr_min1 = 0, cod_cr_max1 = 0;
uint8_t cod_lum_min2 = 0, cod_lum_max2 = 0, cod_cb_min2 = 0, cod_cb_max2 = 0, cod_cr_min2 = 0, cod_cr_max2 = 0;
uint8_t cod_lum_min3 = 0, cod_lum_max3 = 0, cod_cb_min3 = 0, cod_cb_max3 = 0, cod_cr_min3 = 0, cod_cr_max3 = 0;
uint8_t cod_lum_min4 = 0, cod_lum_max4 = 0, cod_cb_min4 = 0, cod_cb_max4 = 0, cod_cr_min4 = 0, cod_cr_max4 = 0;

bool cod_draw1 = false;
bool cod_draw2 = false;
bool cod_draw3 = false;
bool cod_draw4 = false;

struct color_object_t {
  int32_t x_c;
  int32_t y_c;
  uint32_t color_count;
  bool updated;
};

struct color_object_t global_filters[4];

/* ROI helpers */
static bool pixel_in_lower_trapezoid(uint16_t x, uint16_t y, uint16_t img_w, uint16_t img_h);
static bool pixel_on_lower_trapezoid_border(uint16_t x, uint16_t y, uint16_t img_w, uint16_t img_h);
static bool pixel_in_upper_rectangle(uint16_t x, uint16_t y, uint16_t img_w, uint16_t img_h);
static bool pixel_on_upper_rectangle_border(uint16_t x, uint16_t y, uint16_t img_w, uint16_t img_h);

uint32_t find_object_centroid(struct image_t *img, int32_t* p_xc, int32_t* p_yc, bool draw,
                              uint8_t lum_min, uint8_t lum_max,
                              uint8_t cb_min, uint8_t cb_max,
                              uint8_t cr_min, uint8_t cr_max,
                              uint8_t filter);

static bool pixel_in_lower_trapezoid(uint16_t x, uint16_t y, uint16_t img_w, uint16_t img_h)
{
  int col_end = (int)(0.35f * img_w);
  int margin_start = 80;
  int margin_end = (int)(0.25f * img_h);

  if (col_end <= 0) {
    return false;
  }
  if ((int)x > col_end) {
    return false;
  }

  float alpha = (float)x / (float)col_end;
  float margin = (1.0f - alpha) * margin_start + alpha * margin_end;

  int y_min = (int)roundf(margin);
  int y_max = (int)roundf((float)img_h - margin);

  return ((int)y >= y_min && (int)y <= y_max);
}

static bool pixel_on_lower_trapezoid_border(uint16_t x, uint16_t y, uint16_t img_w, uint16_t img_h)
{
  int col_end = (int)(0.35f * img_w);
  int margin_start = 80;
  int margin_end = (int)(0.25f * img_h);

  if (col_end <= 0) {
    return false;
  }
  if ((int)x > col_end) {
    return false;
  }

  float alpha = (float)x / (float)col_end;
  float margin = (1.0f - alpha) * margin_start + alpha * margin_end;

  int y_min = (int)roundf(margin);
  int y_max = (int)roundf((float)img_h - margin);

  if (((int)x == 0 || (int)x == col_end) && ((int)y >= y_min && (int)y <= y_max)) {
    return true;
  }
  if (abs((int)y - y_min) <= 1 || abs((int)y - y_max) <= 1) {
    return true;
  }
  return false;
}

static bool pixel_in_upper_rectangle(uint16_t x, uint16_t y, uint16_t img_w, uint16_t img_h)
{
  int x_min = (int)(0.40f * img_w);
  int x_max = img_w - 1;
  int y_min = (int)(0.25f * img_h);
  int y_max = (int)(0.75f * img_h);

  return ((int)x >= x_min && (int)x <= x_max &&
          (int)y >= y_min && (int)y <= y_max);
}

static bool pixel_on_upper_rectangle_border(uint16_t x, uint16_t y, uint16_t img_w, uint16_t img_h)
{
  int x_min = (int)(0.40f * img_w);
  int x_max = img_w - 1;
  int y_min = (int)(0.25f * img_h);
  int y_max = (int)(0.75f * img_h);

  bool on_vertical = (((int)x == x_min || (int)x == x_max) && ((int)y >= y_min && (int)y <= y_max));
  bool on_horizontal = (((int)y == y_min || (int)y == y_max) && ((int)x >= x_min && (int)x <= x_max));

  return on_vertical || on_horizontal;
}

static struct image_t *object_detector(struct image_t *img, uint8_t filter)
{
  uint8_t lum_min, lum_max;
  uint8_t cb_min, cb_max;
  uint8_t cr_min, cr_max;
  bool draw;

  switch (filter) {
    case 1:
      lum_min = cod_lum_min1; lum_max = cod_lum_max1;
      cb_min  = cod_cb_min1;  cb_max  = cod_cb_max1;
      cr_min  = cod_cr_min1;  cr_max  = cod_cr_max1;
      draw = cod_draw1;
      break;
    case 2:
      lum_min = cod_lum_min2; lum_max = cod_lum_max2;
      cb_min  = cod_cb_min2;  cb_max  = cod_cb_max2;
      cr_min  = cod_cr_min2;  cr_max  = cod_cr_max2;
      draw = cod_draw2;
      break;
    case 3:
      lum_min = cod_lum_min3; lum_max = cod_lum_max3;
      cb_min  = cod_cb_min3;  cb_max  = cod_cb_max3;
      cr_min  = cod_cr_min3;  cr_max  = cod_cr_max3;
      draw = cod_draw3;
      break;
    case 4:
      lum_min = cod_lum_min4; lum_max = cod_lum_max4;
      cb_min  = cod_cb_min4;  cb_max  = cod_cb_max4;
      cr_min  = cod_cr_min4;  cr_max  = cod_cr_max4;
      draw = cod_draw4;
      break;
    default:
      return img;
  }

  int32_t x_c, y_c;
  uint32_t count = find_object_centroid(img, &x_c, &y_c, draw,
                                        lum_min, lum_max, cb_min, cb_max, cr_min, cr_max,
                                        filter);

  pthread_mutex_lock(&mutex);
  global_filters[filter - 1].color_count = count;
  global_filters[filter - 1].x_c = x_c;
  global_filters[filter - 1].y_c = y_c;
  global_filters[filter - 1].updated = true;
  pthread_mutex_unlock(&mutex);

  return img;
}

struct image_t *object_detector1(struct image_t *img, uint8_t camera_id);
struct image_t *object_detector1(struct image_t *img, uint8_t camera_id __attribute__((unused)))
{
  return object_detector(img, 1);
}

struct image_t *object_detector2(struct image_t *img, uint8_t camera_id);
struct image_t *object_detector2(struct image_t *img, uint8_t camera_id __attribute__((unused)))
{
  return object_detector(img, 2);
}

struct image_t *object_detector3(struct image_t *img, uint8_t camera_id);
struct image_t *object_detector3(struct image_t *img, uint8_t camera_id __attribute__((unused)))
{
  return object_detector(img, 3);
}

struct image_t *object_detector4(struct image_t *img, uint8_t camera_id);
struct image_t *object_detector4(struct image_t *img, uint8_t camera_id __attribute__((unused)))
{
  return object_detector(img, 4);
}

void color_object_detector_init(void)
{
  memset(global_filters, 0, 4* sizeof(struct color_object_t));
  pthread_mutex_init(&mutex, NULL);

#ifdef COLOR_OBJECT_DETECTOR_CAMERA1
#ifdef COLOR_OBJECT_DETECTOR_LUM_MIN1
  cod_lum_min1 = COLOR_OBJECT_DETECTOR_LUM_MIN1;
  cod_lum_max1 = COLOR_OBJECT_DETECTOR_LUM_MAX1;
  cod_cb_min1  = COLOR_OBJECT_DETECTOR_CB_MIN1;
  cod_cb_max1  = COLOR_OBJECT_DETECTOR_CB_MAX1;
  cod_cr_min1  = COLOR_OBJECT_DETECTOR_CR_MIN1;
  cod_cr_max1  = COLOR_OBJECT_DETECTOR_CR_MAX1;
#endif
#ifdef COLOR_OBJECT_DETECTOR_DRAW1
  cod_draw1 = COLOR_OBJECT_DETECTOR_DRAW1;
#endif
  cv_add_to_device(&COLOR_OBJECT_DETECTOR_CAMERA1, object_detector1, COLOR_OBJECT_DETECTOR_FPS1, 0);
#endif

#ifdef COLOR_OBJECT_DETECTOR_CAMERA2
#ifdef COLOR_OBJECT_DETECTOR_LUM_MIN2
  cod_lum_min2 = COLOR_OBJECT_DETECTOR_LUM_MIN2;
  cod_lum_max2 = COLOR_OBJECT_DETECTOR_LUM_MAX2;
  cod_cb_min2  = COLOR_OBJECT_DETECTOR_CB_MIN2;
  cod_cb_max2  = COLOR_OBJECT_DETECTOR_CB_MAX2;
  cod_cr_min2  = COLOR_OBJECT_DETECTOR_CR_MIN2;
  cod_cr_max2  = COLOR_OBJECT_DETECTOR_CR_MAX2;
#endif
#ifdef COLOR_OBJECT_DETECTOR_DRAW2
  cod_draw2 = COLOR_OBJECT_DETECTOR_DRAW2;
#endif
  cv_add_to_device(&COLOR_OBJECT_DETECTOR_CAMERA2, object_detector2, COLOR_OBJECT_DETECTOR_FPS2, 1);
#endif

#ifdef COLOR_OBJECT_DETECTOR_CAMERA3
#ifdef COLOR_OBJECT_DETECTOR_LUM_MIN3
  cod_lum_min3 = COLOR_OBJECT_DETECTOR_LUM_MIN3;
  cod_lum_max3 = COLOR_OBJECT_DETECTOR_LUM_MAX3;
  cod_cb_min3  = COLOR_OBJECT_DETECTOR_CB_MIN3;
  cod_cb_max3  = COLOR_OBJECT_DETECTOR_CB_MAX3;
  cod_cr_min3  = COLOR_OBJECT_DETECTOR_CR_MIN3;
  cod_cr_max3  = COLOR_OBJECT_DETECTOR_CR_MAX3;
#endif
#ifdef COLOR_OBJECT_DETECTOR_DRAW3
  cod_draw3 = COLOR_OBJECT_DETECTOR_DRAW3;
#endif
  cv_add_to_device(&COLOR_OBJECT_DETECTOR_CAMERA3, object_detector3, COLOR_OBJECT_DETECTOR_FPS3, 2);
#endif

#ifdef COLOR_OBJECT_DETECTOR_CAMERA4
#ifdef COLOR_OBJECT_DETECTOR_LUM_MIN4
  cod_lum_min4 = COLOR_OBJECT_DETECTOR_LUM_MIN4;
  cod_lum_max4 = COLOR_OBJECT_DETECTOR_LUM_MAX4;
  cod_cb_min4  = COLOR_OBJECT_DETECTOR_CB_MIN4;
  cod_cb_max4  = COLOR_OBJECT_DETECTOR_CB_MAX4;
  cod_cr_min4  = COLOR_OBJECT_DETECTOR_CR_MIN4;
  cod_cr_max4  = COLOR_OBJECT_DETECTOR_CR_MAX4;
#endif
#ifdef COLOR_OBJECT_DETECTOR_DRAW4
  cod_draw4 = COLOR_OBJECT_DETECTOR_DRAW4;
#endif
  // This registers the 4th filter function
  cv_add_to_device(&COLOR_OBJECT_DETECTOR_CAMERA4, object_detector4, COLOR_OBJECT_DETECTOR_FPS4, 3);
#endif
}

uint32_t find_object_centroid(struct image_t *img, int32_t* p_xc, int32_t* p_yc, bool draw,
                              uint8_t lum_min, uint8_t lum_max,
                              uint8_t cb_min, uint8_t cb_max,
                              uint8_t cr_min, uint8_t cr_max,
                              uint8_t filter)
{
  uint32_t cnt = 0;
  uint32_t tot_x = 0;
  uint32_t tot_y = 0;
  uint8_t *buffer = img->buf;

  for (uint16_t y = 0; y < img->h; y++) {
    for (uint16_t x = 0; x < img->w; x++) {

      bool inside_roi = true;

      if (filter == 1 || filter == 2) {
        inside_roi = pixel_in_lower_trapezoid(x, y, img->w, img->h);
      } else if (filter == 3 || filter == 4) {
        inside_roi = pixel_in_upper_rectangle(x, y, img->w, img->h);
      }

      uint8_t *yp, *up, *vp;
      if (x % 2 == 0) {
        up = &buffer[y * 2 * img->w + 2 * x];
        yp = &buffer[y * 2 * img->w + 2 * x + 1];
        vp = &buffer[y * 2 * img->w + 2 * x + 2];
      } else {
        up = &buffer[y * 2 * img->w + 2 * x - 2];
        vp = &buffer[y * 2 * img->w + 2 * x];
        yp = &buffer[y * 2 * img->w + 2 * x + 1];
      }

      if (draw && filter == 1 && pixel_on_lower_trapezoid_border(x, y, img->w, img->h)) {
        *yp = 255;
      }
      if (draw && filter == 2 && pixel_on_lower_trapezoid_border(x, y, img->w, img->h)) {
        *yp = 255;
      }
      if (draw && filter == 3 && pixel_on_upper_rectangle_border(x, y, img->w, img->h)) {
        *yp = 255;
      }

      if (!inside_roi) {
        continue;
      }

      if ((*yp >= lum_min) && (*yp <= lum_max) &&
          (*up >= cb_min)  && (*up <= cb_max) &&
          (*vp >= cr_min)  && (*vp <= cr_max)) {
        cnt++;
        tot_x += x;
        tot_y += y;
        if (draw) {
          *yp = 255;
        }
      }
    }
  }

  if (cnt > 0) {
    *p_xc = (int32_t)roundf(tot_x / ((float) cnt) - img->w * 0.5f);
    *p_yc = (int32_t)roundf(img->h * 0.5f - tot_y / ((float) cnt));
  } else {
    *p_xc = 0;
    *p_yc = 0;
  }

  return cnt;
}

void color_object_detector_periodic(void)
{
  static struct color_object_t local_filters[4];
  pthread_mutex_lock(&mutex);
  memcpy(local_filters, global_filters, 4 * sizeof(struct color_object_t));
  pthread_mutex_unlock(&mutex);

  if (local_filters[0].updated) {
    AbiSendMsgVISUAL_DETECTION(COLOR_OBJECT_DETECTION1_ID,
                               local_filters[0].x_c, local_filters[0].y_c,
                               0, 0, local_filters[0].color_count, 0);
    local_filters[0].updated = false;
  }

  if (local_filters[1].updated) {
    AbiSendMsgVISUAL_DETECTION(COLOR_OBJECT_DETECTION2_ID,
                               local_filters[1].x_c, local_filters[1].y_c,
                               0, 0, local_filters[1].color_count, 1);
    local_filters[1].updated = false;
  }

  if (local_filters[2].updated) {
    AbiSendMsgVISUAL_DETECTION(COLOR_OBJECT_DETECTION3_ID,
                               local_filters[2].x_c, local_filters[2].y_c,
                               0, 0, local_filters[2].color_count, 2);
    local_filters[2].updated = false;
  }

  if (local_filters[3].updated) {
  AbiSendMsgVISUAL_DETECTION(COLOR_OBJECT_DETECTION4_ID,
                             local_filters[3].x_c, local_filters[3].y_c,
                             0, 0, local_filters[3].color_count, 3);
  local_filters[3].updated = false;
  }
}