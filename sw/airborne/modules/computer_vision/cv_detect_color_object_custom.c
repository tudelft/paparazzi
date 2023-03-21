#include "modules/computer_vision/cv_detect_color_object_custom.h"
#include "modules/computer_vision/cv.h"
#include "modules/core/abi.h"
#include "std.h"

#include <stdio.h>
#include <stdbool.h>
#include <math.h>
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


//NEW GLOBAL VARIABLES

#ifndef kernel_size
#define kernel_size 25
#endif

#ifndef half_kernel_size
#define half_kernel_size 12
#endif

//NEW

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

// define global variables
struct color_object_t {
  int32_t x_c;
  int32_t y_c;
  uint32_t color_count;
  bool updated; 

  int16_t vector_x;
  int16_t vector_y;
};

struct return_value {
  uint32_t color_count;
  int16_t vector_x;
  int16_t vector_y;
};

struct pixel_values {
  uint8_t *yp;
  uint8_t *up;
  uint8_t *vp;
};

struct color_object_t global_filters[1];

struct return_value find_object_centroid(struct image_t *img, int32_t* p_xc, int32_t* p_yc, bool draw,
                              uint8_t lum_min, uint8_t lum_max,
                              uint8_t cb_min, uint8_t cb_max,
                              uint8_t cr_min, uint8_t cr_max);


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
    default:
      return img;
  };

  int32_t x_c, y_c;

  struct return_value result;
  result = find_object_centroid(img, &x_c, &y_c, draw, lum_min, lum_max, cb_min, cb_max, cr_min, cr_max);
  VERBOSE_PRINT("Color count %d: %u, threshold %u, x_c %d, y_c %d\n", camera, object_count, count_threshold, x_c, y_c);
  VERBOSE_PRINT("centroid %d: (%d, %d) r: %4.2f a: %4.2f\n", camera, x_c, y_c,
        hypotf(x_c, y_c) / hypotf(img->w * 0.5, img->h * 0.5), RadOfDeg(atan2f(y_c, x_c)));

  pthread_mutex_lock(&mutex);
  global_filters[0].color_count = result.color_count;
  global_filters[0].x_c = x_c;
  global_filters[0].y_c = y_c;
  global_filters[0].updated = true;
  global_filters[0].vector_x = result.vector_x;
  global_filters[0].vector_y = result.vector_y;  
  pthread_mutex_unlock(&mutex);

  return img;
}

struct image_t *object_detector1(struct image_t *img, uint8_t camera_id);
struct image_t *object_detector1(struct image_t *img, uint8_t camera_id __attribute__((unused)))
{
  return object_detector(img, 1);
}

void color_object_detector_init(void)
{
  memset(global_filters, 0, sizeof(struct color_object_t));
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
}

struct pixel_values compute_pixel_yuv(struct image_t *img, int16_t x, int16_t y)
{
  struct pixel_values result;
  uint8_t *buffer = img->buf;
  uint8_t *yp, *up, *vp;
  if (x % 2 == 0) {
    // Even x
    up = &buffer[y * 2 * img->w + 2 * x];      // U
    yp = &buffer[y * 2 * img->w + 2 * x + 1];  // Y1
    vp = &buffer[y * 2 * img->w + 2 * x + 2];  // V
    //yp = &buffer[y * 2 * img->w + 2 * x + 3]; // Y2
  } else {
    // Uneven x
    up = &buffer[y * 2 * img->w + 2 * x - 2];  // U
    //yp = &buffer[y * 2 * img->w + 2 * x - 1]; // Y1
    vp = &buffer[y * 2 * img->w + 2 * x];      // V
    yp = &buffer[y * 2 * img->w + 2 * x + 1];  // Y2
  }
  result.yp = yp;
  result.up = up;
  result.vp = vp;
  return result;
}

struct return_value find_object_centroid(struct image_t *img, int32_t* p_xc, int32_t* p_yc, bool draw,
                              uint8_t lum_min, uint8_t lum_max,
                              uint8_t cb_min, uint8_t cb_max,
                              uint8_t cr_min, uint8_t cr_max)
{
  uint32_t cnt = 0;
  uint32_t tot_x = 0;
  uint32_t tot_y = 0;
  uint8_t *buffer = img->buf;

  struct return_value test;
  struct pixel_values pix_values;  

  int16_t heigth = img->h;
  int16_t width = img->w;
  int16_t kernel_cnt = 0;

  int16_t threshold = 255;

  int16_t x = 0;
  int16_t y = 0;
  int16_t kur_x = 0;
  int16_t kur_y = 0;

  int16_t kernel_centroid = 0;

  int16_t kernel_w_cnt = floor(width/kernel_size);
  int16_t kernel_h_cnt = floor(heigth/kernel_size);

  int16_t vector_array[20] = {0};

  for (int8_t y_k = 0; y_k < kernel_h_cnt; y_k++){
    // for (int8_t x_k = 0; x_k < kernel_w_cnt; x_k++){

      int8_t state = 0;

      for (int8_t x_k = kernel_w_cnt-1; x_k >= 0; x_k--){

      kernel_cnt = 0;
      kur_x = kernel_size*x_k;
      kur_y = kernel_size*y_k;
      for (int8_t i = 0; i < kernel_size; i++){
        for (int8_t j = 0; j < kernel_size; j++){
          x = kur_x + j;
          y = kur_y + i;

          uint8_t *yp, *up, *vp;
          pix_values = compute_pixel_yuv(img, x, y);
          yp = pix_values.yp;
          up = pix_values.up;
          vp = pix_values.vp;

          if ( (*yp >= lum_min) && (*yp <= lum_max) &&
           (*up >= cb_min ) && (*up <= cb_max ) &&
           (*vp >= cr_min ) && (*vp <= cr_max )) {
            if (draw){
              *yp = 255;  // make pixel brighter in image
            }
            kernel_cnt++;
          }          
        }
      }
      //add break when ready
      //TODO
      if (kernel_cnt > threshold){
        if (state == 0) {
          state = 1;
          kernel_centroid = kernel_size * x_k + half_kernel_size;
          // PRINT("Vector length %d\n", kernel_centroid);
          // PRINT("Yk value %d\n", y_k);
          vector_array[y_k] = kernel_centroid;
        }
      }
    }
    if (state == 0) {
      state = 1;
      kernel_centroid = 0;
      // PRINT("Vector length %d\n", kernel_centroid);
      // PRINT("Yk value %d\n", y_k);
      vector_array[y_k] = kernel_centroid;
    }
  }
  if (draw){
    int16_t max = 0;
    int8_t vector_count = 0;
    for (int16_t y = half_kernel_size; y < (img->h - kernel_size); y+= kernel_size){
      max = vector_array[vector_count];
      if (max<0) {
        max = 0;
      }
      if (max > img->w) {
        max = img->w;
      }
      for(int16_t x = 0; x < max; x++){
        uint8_t *yp, *up, *vp;
        pix_values = compute_pixel_yuv(img, x, y);
        yp = pix_values.yp;
        up = pix_values.up;
        vp = pix_values.vp;
        *up = 0;
        *vp = 255;
        *yp = 125;
      }
      vector_count++;
    }
  }

  int16_t T_x = 30;
  int16_t T_y = 160;
  float T_mid = 10.0*kernel_size - half_kernel_size;
  float alpha = T_x/(0.5 * T_y);
  float beta1 = T_x - alpha*(T_mid);
  float beta2 = T_x + alpha*(T_mid);

  int16_t vector_x = 0;
  int16_t vector_y = 0;
  bool in_triangle = true;

  for (int8_t i = 0; i < 20; i++){
    int16_t vector_length = vector_array[i];

    int16_t y = i*kernel_size + half_kernel_size;

    if (y > T_mid) {
        x = y*-1.0*alpha + beta2;
      }
    else {
      x = y*alpha + beta1;
    }

    if (x <= 0){
      if (vector_length > vector_x){
        vector_x = vector_length;
        vector_y = y;
      }
    }
    else {
      if (vector_length < x) {
        in_triangle = false;
      }
    }

    if(!in_triangle) {
      cnt = 0;
    }
    else{
      cnt = vector_array[10];
    }
  }

  if (draw){
    int16_t x = 0;

    for (int16_t y = T_mid - T_y/2; y < T_mid + T_y/2; y++){
      if (y > T_mid) {
        x = y*-1.0*alpha + beta2;
      }
      else {
        x = y*alpha + beta1;
      }

        uint8_t *yp, *up, *vp;
        pix_values = compute_pixel_yuv(img, x, y);
        yp = pix_values.yp;
        up = pix_values.up;
        vp = pix_values.vp;

        if (in_triangle){
          *up = 128;
          *vp = 0;
          *yp = 128;
        }
        else {
          *up = 128;
          *vp = 255;
          *yp = 128;
        }
      }

      int16_t max = vector_x;
      y = vector_y;
      if (max<0) {
        max = 0;
      }
      if (max > img->w) {
        max = img->w;
      }
      for(int16_t x = 0; x < max; x++){
        uint8_t *yp, *up, *vp;
        pix_values = compute_pixel_yuv(img, x, y);
        yp = pix_values.yp;
        up = pix_values.up;
        vp = pix_values.vp;
        *up = 192;
        *vp = 128;
        *yp = 20;
      }
    }
  test.color_count = cnt;
  test.vector_x = vector_x;
  test.vector_y = vector_y;
  return test;
}

void color_object_detector_periodic(void)
{
  static struct color_object_t local_filters[1];
  pthread_mutex_lock(&mutex);
  memcpy(local_filters, global_filters, sizeof(struct color_object_t));
  pthread_mutex_unlock(&mutex);

  if(local_filters[0].updated){
    AbiSendMsgVISUAL_DETECTION(COLOR_OBJECT_DETECTION1_ID, local_filters[0].x_c, local_filters[0].y_c,
        local_filters[0].vector_x, local_filters[0].vector_y, local_filters[0].color_count, 0);
    local_filters[0].updated = false;
  }
}
