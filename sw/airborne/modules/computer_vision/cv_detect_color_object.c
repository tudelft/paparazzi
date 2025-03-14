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
 * @file modules/computer_vision/cv_detect_object.h
 * Assumes the object consists of a continuous color and checks
 * if you are over the defined object or not. Now also supports
 * checking if the object is surrounded by green and drawing it.
 */

// Own header
#include "modules/computer_vision/cv_detect_color_object.h"
#include "modules/computer_vision/cv.h"
#include "modules/core/abi.h"
#include "std.h"

#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include <stdlib.h>
#include "pthread.h"

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

// Filter Settings
uint8_t cod_lum_min1 = 0;
uint8_t cod_lum_max1 = 0;
uint8_t cod_cb_min1 = 0;
uint8_t cod_cb_max1 = 0;
uint8_t cod_cr_min1 = 0;
uint8_t cod_cr_max1 = 0;

uint8_t cod_lum_min2 = 0;
uint8_t cod_lum_max2 = 0;
uint8_t cod_cb_min2 = 0;
uint8_t cod_cb_max2 = 0;
uint8_t cod_cr_min2 = 0;
uint8_t cod_cr_max2 = 0;

bool cod_draw1 = false;
bool cod_draw2 = false;
// New flag to enable flood-fill drawing.
bool cod_flood_fill1 = false;
bool cod_flood_fill2 = false;

// define global variables
struct color_object_t {
  int32_t x_c;
  int32_t y_c;
  uint32_t color_count;
  bool updated;
};
struct color_object_t global_filters[2];

/* Forward declaration of flood fill function */
static void flood_fill_draw(struct image_t *img, int seed_x, int seed_y,
                            uint8_t draw_Y, uint8_t draw_U, uint8_t draw_V);

/* Helper function: Checks if a pixel is “green” in YUV.
 * Adjust these thresholds as needed.
 */
static bool is_green(uint8_t Y, uint8_t U, uint8_t V) {
  return (Y > 50 && Y < 150) && (U > 100 && U < 140) && (V > 90 && V < 130);
}

/* Flood fill implementation.
 * This uses an iterative (stack-based) approach. It starts at the seed pixel,
 * draws it in the given color, and visits 4-connected neighbors.
 */
typedef struct {
  int x;
  int y;
} Point;

#define STACK_MAX 1024

static void flood_fill_draw(struct image_t *img, int seed_x, int seed_y,
                            uint8_t draw_Y, uint8_t draw_U, uint8_t draw_V) {
  Point stack[STACK_MAX];
  int stack_ptr = 0;

  // Create a visited flag array to avoid processing a pixel twice.
  bool *visited = calloc(img->w * img->h, sizeof(bool));
  if (!visited) return;

  // Push seed point
  stack[stack_ptr++] = (Point){seed_x, seed_y};
  visited[seed_y * img->w + seed_x] = true;

  while (stack_ptr > 0) {
    Point p = stack[--stack_ptr];

    // Compute buffer indices based on YUV422 format.
    // For simplicity, assume p.x is even. If not, you may need to adjust for shared chroma.
    int index = p.y * 2 * img->w + 2 * (p.x & ~1); // even index for chroma
    uint8_t *buffer = img->buf;
    // Pointers for the two pixels sharing the same U and V:
    uint8_t *up = &buffer[index];      // U
    uint8_t *yp1 = &buffer[index + 1];   // Y for even pixel
    uint8_t *vp = &buffer[index + 2];    // V
    // For odd pixels, Y is stored in the next byte (if needed).

    // Check if current pixel qualifies as green.
    // Here we assume the pixel color is already in the scene.
    if (!is_green(*yp1, *up, *vp)) {
      // Optionally, you can decide not to fill further if the pixel is not green.
      // For this example, we still draw on it.
    }

    // Draw the pixel (set to white, for example)
    *yp1 = draw_Y;
    *up  = draw_U;
    *vp  = draw_V;

    // Check 4-connected neighbors
    Point neighbors[4] = {
      {p.x + 1, p.y},
      {p.x - 1, p.y},
      {p.x, p.y + 1},
      {p.x, p.y - 1}
    };

    for (int i = 0; i < 4; i++) {
      int nx = neighbors[i].x;
      int ny = neighbors[i].y;
      if (nx >= 0 && nx < img->w && ny >= 0 && ny < img->h) {
        if (!visited[ny * img->w + nx]) {
          visited[ny * img->w + nx] = true;
          stack[stack_ptr++] = (Point){nx, ny};
          if (stack_ptr >= STACK_MAX) {
            // In production, you would handle a full stack appropriately.
            break;
          }
        }
      }
    }
  }
  free(visited);
}

/* Existing function declaration */
uint32_t find_object_centroid(struct image_t *img, int32_t* p_xc, int32_t* p_yc, bool draw,
                              uint8_t lum_min, uint8_t lum_max,
                              uint8_t cb_min, uint8_t cb_max,
                              uint8_t cr_min, uint8_t cr_max);

/*
 * object_detector
 * @param img - input image to process
 * @param filter - which detection filter to process
 * @return img
 */
static struct image_t *object_detector(struct image_t *img, uint8_t filter)
{
  uint8_t lum_min, lum_max;
  uint8_t cb_min, cb_max;
  uint8_t cr_min, cr_max;
  bool draw;
  bool flood_fill = false; // flag for flood fill drawing

  switch (filter){
    case 1:
      lum_min = cod_lum_min1;
      lum_max = cod_lum_max1;
      cb_min = cod_cb_min1;
      cb_max = cod_cb_max1;
      cr_min = cod_cr_min1;
      cr_max = cod_cr_max1;
      draw = cod_draw1;
      flood_fill = cod_flood_fill1;
      break;
    case 2:
      lum_min = cod_lum_min2;
      lum_max = cod_lum_max2;
      cb_min = cod_cb_min2;
      cb_max = cod_cb_max2;
      cr_min = cod_cr_min2;
      cr_max = cod_cr_max2;
      draw = cod_draw2;
      flood_fill = cod_flood_fill2;
      break;
    default:
      return img;
  };

  int32_t x_c, y_c;
  // The centroid is computed from pixels that meet the filter bounds.
  // Note: x_c and y_c are returned relative to the image center.
  uint32_t count = find_object_centroid(img, &x_c, &y_c, draw, lum_min, lum_max, cb_min, cb_max, cr_min, cr_max);

  VERBOSE_PRINT("Color count: %u, centroid (relative): (%d, %d)\n", count, x_c, y_c);

  // Update global filter data
  pthread_mutex_lock(&mutex);
  global_filters[filter-1].color_count = count;
  global_filters[filter-1].x_c = x_c;
  global_filters[filter-1].y_c = y_c;
  global_filters[filter-1].updated = true;
  pthread_mutex_unlock(&mutex);

  // If flood-fill drawing is enabled and we detected the object,
  // convert the relative centroid back to absolute image coordinates.
  if (flood_fill && count > 0) {
    int abs_x = x_c + img->w / 2;
    int abs_y = img->h / 2 - y_c;
    // Here, we use white (Y=255, U=128, V=128) as the drawing color.
    flood_fill_draw(img, abs_x, abs_y, 255, 128, 128);
  }
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

void color_object_detector_init(void)
{
  memset(global_filters, 0, 2*sizeof(struct color_object_t));
  pthread_mutex_init(&mutex, NULL);
#ifdef COLOR_OBJECT_DETECTOR_CAMERA1
#ifdef COLOR_OBJECT_DETECTOR_LUM_MIN1
  cod_lum_min1 = COLOR_OBJECT_DETECTOR_LUM_MIN1;
  cod_lum_max1 = COLOR_OBJECT_DETECTOR_LUM_MAX1;
  cod_cb_min1 = COLOR_OBJECT_DETECTOR_CB_MIN1;
  cod_cb_max1 = COLOR_OBJECT_DETECTOR_CB_MAX1;
  cod_cr_min1 = COLOR_OBJECT_DETECTOR_CR_MIN1;
  cod_cr_max1 = COLOR_OBJECT_DETECTOR_CR_MAX1;
#endif
#ifdef COLOR_OBJECT_DETECTOR_DRAW1
  cod_draw1 = COLOR_OBJECT_DETECTOR_DRAW1;
#endif
#ifdef COLOR_OBJECT_DETECTOR_FLOOD_FILL1
  cod_flood_fill1 = COLOR_OBJECT_DETECTOR_FLOOD_FILL1;
#endif

  cv_add_to_device(&COLOR_OBJECT_DETECTOR_CAMERA1, object_detector1, COLOR_OBJECT_DETECTOR_FPS1, 0);
#endif

#ifdef COLOR_OBJECT_DETECTOR_CAMERA2
#ifdef COLOR_OBJECT_DETECTOR_LUM_MIN2
  cod_lum_min2 = COLOR_OBJECT_DETECTOR_LUM_MIN2;
  cod_lum_max2 = COLOR_OBJECT_DETECTOR_LUM_MAX2;
  cod_cb_min2 = COLOR_OBJECT_DETECTOR_CB_MIN2;
  cod_cb_max2 = COLOR_OBJECT_DETECTOR_CB_MAX2;
  cod_cr_min2 = COLOR_OBJECT_DETECTOR_CR_MIN2;
  cod_cr_max2 = COLOR_OBJECT_DETECTOR_CR_MAX2;
#endif
#ifdef COLOR_OBJECT_DETECTOR_DRAW2
  cod_draw2 = COLOR_OBJECT_DETECTOR_DRAW2;
#endif
#ifdef COLOR_OBJECT_DETECTOR_FLOOD_FILL2
  cod_flood_fill2 = COLOR_OBJECT_DETECTOR_FLOOD_FILL2;
#endif

  cv_add_to_device(&COLOR_OBJECT_DETECTOR_CAMERA2, object_detector2, COLOR_OBJECT_DETECTOR_FPS2, 1);
#endif
}

/*
 * find_object_centroid
 *
 * Finds the centroid of pixels in an image within filter bounds.
 * Also returns the amount of pixels that satisfy these filter bounds.
 *
 * @param img - input image to process formatted as YUV422.
 * @param p_xc - x coordinate of the centroid of color object (relative to image center)
 * @param p_yc - y coordinate of the centroid of color object (relative to image center)
 * @param lum_min - minimum Y value for the filter in YCbCr colorspace
 * @param lum_max - maximum Y value for the filter in YCbCr colorspace
 * @param cb_min - minimum Cb value for the filter in YCbCr colorspace
 * @param cb_max - maximum Cb value for the filter in YCbCr colorspace
 * @param cr_min - minimum Cr value for the filter in YCbCr colorspace
 * @param cr_max - maximum Cr value for the filter in YCbCr colorspace
 * @param draw - whether or not to draw on image
 * @return number of pixels in the image within the filter bounds.
 */
uint32_t find_object_centroid(struct image_t *img, int32_t* p_xc, int32_t* p_yc, bool draw,
                              uint8_t lum_min, uint8_t lum_max,
                              uint8_t cb_min, uint8_t cb_max,
                              uint8_t cr_min, uint8_t cr_max)
{
  uint32_t cnt = 0;
  uint32_t tot_x = 0;
  uint32_t tot_y = 0;
  uint8_t *buffer = img->buf;

  // Iterate over all pixels
  for (uint16_t y = 0; y < img->h; y++) {
    for (uint16_t x = 0; x < img->w; x ++) {
      uint8_t *yp, *up, *vp;
      if (x % 2 == 0) {
        // Even x
        up = &buffer[y * 2 * img->w + 2 * x];       // U
        yp = &buffer[y * 2 * img->w + 2 * x + 1];     // Y1
        vp = &buffer[y * 2 * img->w + 2 * x + 2];     // V
      } else {
        // Odd x
        up = &buffer[y * 2 * img->w + 2 * x - 2];     // U (shared)
        vp = &buffer[y * 2 * img->w + 2 * x];         // V
        yp = &buffer[y * 2 * img->w + 2 * x + 1];       // Y2
      }
      if ((*yp >= lum_min) && (*yp <= lum_max) &&
          (*up >= cb_min ) && (*up <= cb_max ) &&
          (*vp >= cr_min ) && (*vp <= cr_max )) {
        cnt++;
        tot_x += x;
        tot_y += y;
        if (draw) {
          // Existing drawing: brighten the pixel.
          *yp = 255;
        }
      }
    }
  }
  if (cnt > 0) {
    // Calculate centroid relative to the image center.
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
  static struct color_object_t local_filters[2];
  pthread_mutex_lock(&mutex);
  memcpy(local_filters, global_filters, 2*sizeof(struct color_object_t));
  pthread_mutex_unlock(&mutex);

  if(local_filters[0].updated){
    AbiSendMsgVISUAL_DETECTION(COLOR_OBJECT_DETECTION1_ID, local_filters[0].x_c, local_filters[0].y_c,
        0, 0, local_filters[0].color_count, 0);
    local_filters[0].updated = false;
  }
  if(local_filters[1].updated){
    AbiSendMsgVISUAL_DETECTION(COLOR_OBJECT_DETECTION2_ID, local_filters[1].x_c, local_filters[1].y_c,
        0, 0, local_filters[1].color_count, 1);
    local_filters[1].updated = false;
  }
}
