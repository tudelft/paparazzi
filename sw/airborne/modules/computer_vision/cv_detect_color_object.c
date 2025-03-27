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

uint16_t num_segments = 5;
uint8_t fill_y_limit = 128;


// define global variables
struct color_object_t {
  int32_t x_c;
  int32_t y_c;
  int32_t color_count;
  bool updated;
  int32_t segment_counts[5]; // Pre-allocated array for segment counts
};

struct color_object_t global_filters[2];

#define STACK_MAX 1024

/* Existing function declaration */
struct image_t *object_detector1(struct image_t *img, uint8_t camera_id);
struct image_t *object_detector2(struct image_t *img, uint8_t camera_id);

void draw_vertical_line(struct image_t *img, int x, int y);
void draw_horizontal_line(struct image_t *img, int x, int y);

uint32_t count_green_pixels(struct image_t *img, bool draw, 
                              int *segment_counts, int num_segments,
                              uint8_t lum_min, uint8_t lum_max,
                              uint8_t cb_min, uint8_t cb_max,
                              uint8_t cr_min, uint8_t cr_max, uint8_t fill_y_limit);

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

  switch (filter){
    case 1:
      lum_min = cod_lum_min1;
      lum_max = cod_lum_max1;
      cb_min = cod_cb_min1;
      cb_max = cod_cb_max1;
      cr_min = cod_cr_min1;
      cr_max = cod_cr_max1;
      draw = cod_draw1;
      break;
    case 2:
      lum_min = cod_lum_min2;
      lum_max = cod_lum_max2;
      cb_min = cod_cb_min2;
      cb_max = cod_cb_max2;
      cr_min = cod_cr_min2;
      cr_max = cod_cr_max2;
      draw = cod_draw2;
      break;
    default:
      return img;
  };

  int segment_counts[num_segments];

  for (uint16_t i = 0; i < num_segments; i++) {
    segment_counts[i] = 0;
  }
  
  int32_t count = count_green_pixels(img, draw, segment_counts, num_segments, lum_min, lum_max, cb_min, cb_max, cr_min, cr_max,fill_y_limit);

  // #########################
  // Chnage the 128 value (the value before the draw bool to hard code the fill limit)
  // It is now set to half of the image width (255). For ideal results, this should be right at the horizon during forward flight so keep inmind the forward pitch.
  //img = process_image(img, 235, 255, 86, 106, 120, 140, 128, draw, &count);
  // #########################
  
  // calculate the percentage of green in every segment
  for (uint16_t i = 0; i < num_segments; i++) {
    segment_counts[i] = (segment_counts[i] * 100) / (img->w * (img->h / num_segments));
  }

  // Update global filter data
  pthread_mutex_lock(&mutex);
  global_filters[filter-1].color_count = count;
  global_filters[filter-1].updated = true;
  for (uint16_t i = 0; i < 5; i++) {
    global_filters[filter-1].segment_counts[i] = segment_counts[i];
  }
  pthread_mutex_unlock(&mutex);

  return img;
}

struct image_t *object_detector1(struct image_t *img, uint8_t camera_id __attribute__((unused)))
{
  return object_detector(img, 1);
}

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

    cv_add_to_device(&COLOR_OBJECT_DETECTOR_CAMERA2, object_detector2, COLOR_OBJECT_DETECTOR_FPS2, 1);
  #endif
}

/*
 * count_green_pixels
 *
 * Also returns the amount of pixels that satisfy these filter bounds.
 *
 * @param img - input image to process formatted as YUV422.
 * @param lum_min - minimum Y value for the filter in YCbCr colorspace
 * @param lum_max - maximum Y value for the filter in YCbCr colorspace
 * @param cb_min - minimum Cb value for the filter in YCbCr colorspace
 * @param cb_max - maximum Cb value for the filter in YCbCr colorspace
 * @param cr_min - minimum Cr value for the filter in YCbCr colorspace
 * @param cr_max - maximum Cr value for the filter in YCbCr colorspace
 * @param draw - whether or not to draw on image
 * @return number of pixels in the image within the filter bounds.
 */
uint32_t count_green_pixels(struct image_t *img, bool draw, 
                            int *segment_counts, int num_segments,
                            uint8_t lum_min, uint8_t lum_max,
                            uint8_t cb_min, uint8_t cb_max,
                            uint8_t cr_min, uint8_t cr_max,
                            uint8_t fill_y_limit)
{
  uint32_t cnt = 0;
  uint8_t *buffer = img->buf;

  int IMAGE_WIDTH = img->w;
  int IMAGE_HEIGHT = img->h;
  int LIMIT = (int) fill_y_limit;

  int segment_height = (IMAGE_HEIGHT + num_segments - 1) / num_segments;

  draw_vertical_line(img, 0, IMAGE_HEIGHT/5);
  draw_vertical_line(img, 0, IMAGE_HEIGHT*2/5);
  draw_vertical_line(img, 0, IMAGE_HEIGHT*3/5);
  draw_vertical_line(img, 0, IMAGE_HEIGHT*4/5);
  draw_horizontal_line(img, 128, 0);
  draw_horizontal_line(img, IMAGE_WIDTH-40, 0);

  for (uint16_t y = IMAGE_HEIGHT/5; y < IMAGE_HEIGHT*4/5; y++) {
    bool detected_right = false;
    int segment_index = y / segment_height;

    for (int x = LIMIT; x >= 40; x--) {
      uint8_t *yp, *up, *vp;

      if (x % 2 == 0) {
        // Even x
        up = &buffer[y * 2 * IMAGE_WIDTH + 2 * x];       // U
        yp = &buffer[y * 2 * IMAGE_WIDTH + 2 * x + 1];   // Y1
        vp = &buffer[y * 2 * IMAGE_WIDTH + 2 * x + 2];   // V
      } else {
        // Odd x
        up = &buffer[y * 2 * IMAGE_WIDTH + 2 * x - 2];   // U (shared)
        vp = &buffer[y * 2 * IMAGE_WIDTH + 2 * x];       // V
        yp = &buffer[y * 2 * IMAGE_WIDTH + 2 * x + 1];   // Y2
      }

      if ((*yp >= lum_min) && (*yp <= lum_max) &&
          (*up >= cb_min ) && (*up <= cb_max ) &&
          (*vp >= cr_min ) && (*vp <= cr_max )) {
        detected_right = true;
      }

      // Once green is detected in this row, count all pixels to the left
      if (detected_right) {
        cnt++;
        segment_counts[segment_index]++;
        if (draw) {
          *yp = 255;
        }
      }
    }
  }

  return cnt;
}


void color_object_detector_periodic(void)
{
  static struct color_object_t local_filters[2];
  pthread_mutex_lock(&mutex);
  memcpy(local_filters, global_filters, 2*sizeof(struct color_object_t));
  pthread_mutex_unlock(&mutex);

  if (local_filters[0].updated) {
    // Send the VISUAL_DETECTION message (existing functionality)
    AbiSendMsgVISUAL_DETECTION(COLOR_OBJECT_DETECTION1_ID, local_filters[0].x_c, local_filters[0].y_c,
                               0, 0, local_filters[0].color_count, 0);

    // Send the SEGMENT_COUNTS message for filter 1
    AbiSendMsgSEGMENT_COUNTS(GREEN_DETECTOR_SEGMENT_COUNTS_ID,
                              local_filters[0].segment_counts[0],
                              local_filters[0].segment_counts[1],
                              local_filters[0].segment_counts[2],
                              local_filters[0].segment_counts[3],
                              local_filters[0].segment_counts[4]);
    
    local_filters[0].updated = false;
  }

  if (local_filters[1].updated) {
    // Send the VISUAL_DETECTION message (existing functionality)
    printf("%d", local_filters[1].color_count);
    AbiSendMsgVISUAL_DETECTION(COLOR_OBJECT_DETECTION2_ID, local_filters[1].x_c, local_filters[1].y_c,
                                0, 0, local_filters[1].color_count, 1);

    // Send the SEGMENT_COUNTS message for filter 1
    AbiSendMsgSEGMENT_COUNTS(GREEN_DETECTOR_SEGMENT_COUNTS_ID,
                              local_filters[1].segment_counts[0],
                              local_filters[1].segment_counts[1],
                              local_filters[1].segment_counts[2],
                              local_filters[1].segment_counts[3],
                              local_filters[1].segment_counts[4]);

    local_filters[1].updated = false;
  }
}

void draw_vertical_line(struct image_t *img, int x, int y) {
  if (x < 0 || x >= img->w) return; // Bounds check

  uint8_t *buffer = img->buf;

  for (x;x < img->w; x++) {
      uint8_t *yp;
      if (x % 2 == 0) {
          // Even x, gets new U and V values
          yp = &buffer[y * 2 * img->w + 2 * x + 1];  // Y1
      } else {
          // Odd x, shares U and V with the previous pixel
          yp = &buffer[y * 2 * img->w + 2 * x + 1];  // Y2
      }
      *yp = 0;  // Darken pixel to lowest intensity (black)
  }
}

void draw_horizontal_line(struct image_t *img, int x, int y) {
  if (x < 0 || x >= img->h) return; // Bounds check

  uint8_t *buffer = img->buf;

  // Green Line
  for (y; y < img->h; y++) {
    int index = y * 2 * img->w + 2 * x;
    if (x % 2 == 0) {
      buffer[index + 1] = 0;
      buffer[index]     = 0;
      buffer[index + 2] = 0;
    } else {
      buffer[index + 1] = 0;
      buffer[index - 2] = 0;
      buffer[index]     = 0;
    }
  }
}