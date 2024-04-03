/*
 * This file is part of the Paparazzi project and has been modified by the contributors
 * to include advanced color detection functionalities, including ground and plant detection.
 * It is tailored for the AE4317 Autonomous Flight of Micro Air Vehicles course at TU Delft.
 *
 * This module employs computer vision to detect continuous color objects, the ground,
 * and plants by analyzing the camera feed in real-time. It allows UAVs to make informed decisions
 * based on the visual characteristics of their surroundings, supporting tasks such as
 * autonomous navigation, obstacle avoidance, and environmental monitoring.
 *
 * Enhancements include ground calibration for improved ground detection accuracy,
 * central floor area analysis for boundary awareness, and targeted plant detection
 * for environmental interaction. These features equip UAVs with the ability to
 * better understand and react to complex environments.
 *
 * The implementation leverages the YUV color space for efficient color filtering,
 * and introduces calibration procedures to dynamically adjust detection parameters
 * based on environmental conditions.
 */

#include "modules/computer_vision/cv_detect_color_object.h"
#include "modules/computer_vision/cv.h"
#include "modules/core/abi.h"
#include "std.h"
#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include "pthread.h"

// Verbose printing settings
#define PRINT(string,...) fprintf(stderr, "[object_detector->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if OBJECT_DETECTOR_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

static pthread_mutex_t mutex;

// Default frames per second settings for different filters
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

uint8_t cod_lum_min3 = 0;
uint8_t cod_lum_max3 = 0;
uint8_t cod_cb_min3 = 0;
uint8_t cod_cb_max3 = 0;
uint8_t cod_cr_min3 = 0;
uint8_t cod_cr_max3 = 0;

// Margins for color initialization
uint8_t Y_MARGIN = 0;
uint8_t CB_MARGIN = 0;
uint8_t CR_MARGIN = 0;

// Average color values for dynamic calibration
uint8_t avg_y1 = 0;
uint8_t avg_cb1 = 0;
uint8_t avg_cr1 = 0;
uint8_t avg_y = 0;
uint8_t avg_cb = 0;
uint8_t avg_cr = 0;

// Flags to enable drawing detections on output images
bool cod_draw1 = false;
bool cod_draw2 = false;

// Flag to check if initial ground calibration is done
static bool ground_calibration_done = true;

// define global variables
struct color_object_t {
  int32_t x_c; // X centroid
  int32_t y_c; // Y centroid
  uint32_t color_count; // Count of pixels matching color criteria
  uint32_t color_ground_count; // Count for central_ground detection
  uint32_t color_plant_count; // Count for plant detection
  bool updated; // Flag to indicate if the data is updated
};
struct color_object_t global_filters[2];  // Array to hold filter results

// Functions
// Floor calibration function
void calibrate_floor_color(struct image_t *img);

// Object centroid function
uint32_t find_object_centroid(struct image_t *img, int32_t* p_xc, int32_t* p_yc, bool draw,
                              uint8_t lum_min, uint8_t lum_max,
                              uint8_t cb_min, uint8_t cb_max,
                              uint8_t cr_min, uint8_t cr_max);
// Central ground detection function
uint32_t find_floor_count_central(struct image_t *img,
                              uint8_t lum_min, uint8_t lum_max,
                              uint8_t cb_min, uint8_t cb_max,
                              uint8_t cr_min, uint8_t cr_max);

// Plant detection function
uint32_t find_plant_count(struct image_t *img,
                              uint8_t lum_min, uint8_t lum_max,
                              uint8_t cb_min, uint8_t cb_max,
                              uint8_t cr_min, uint8_t cr_max);

/**
 * Calibrates the floor color based on a specific area of the input image.
 * It calculates average YCbCr values to adjust detection thresholds dynamically,
 * focusing on the specified floor area to enhance ground detection accuracy.
 *
 * @param img The input image to calibrate floor color from.
 */
void calibrate_floor_color(struct image_t *img) {
    // Defining the area of the image considered as 'floor' for calibration purposes
    uint32_t y_sum = 0, cb_sum = 0, cr_sum = 0, count = 0;
    uint8_t *buffer = img->buf;

    // Targeting the central bottom part of the image for floor color calibration taking into account the image is rotated 90 degrees
    int floor_area_start_y = img->h * 0.4; // Starting from 40% of the height
    int floor_area_end_y = img->h * 0.6; // Ending at 60% to focus on the central area

    int floor_area_start_x = img->w * 0; // Starting from the left edge of the image
    int floor_area_end_x = img->w * 0.2; // Ending at 20% width from the left to use the bottom part

    // Iterate through the defined 'floor' area of the imagse
    for (uint16_t y = floor_area_start_y; y < floor_area_end_y; y++) {
        for (uint16_t x = floor_area_start_x; x < floor_area_end_x; x++) {
            uint8_t *yp, *up, *vp;
            // Correctly access the Y, U (Cb), and V (Cr) values based on pixel position
            // Pixels are accessed in YUV422 format where each 4 bytes contain 2 pixels
            if (x % 2 == 0) { // Even x
                up = &buffer[y * 2 * img->w + 2 * x];      // U
                yp = &buffer[y * 2 * img->w + 2 * x + 1];  // Y1
                vp = &buffer[y * 2 * img->w + 2 * x + 2];  // V
            } else { // Odd x
                up = &buffer[y * 2 * img->w + 2 * x - 2];  // U
                vp = &buffer[y * 2 * img->w + 2 * x];      // V
                yp = &buffer[y * 2 * img->w + 2 * x + 1];  // Y2
            }
            // Sum up the Y, Cb, and Cr values for averaging
            y_sum += *yp;
            cb_sum += *up;
            cr_sum += *vp;
            count++;
        }
    }

    // Calculate and apply the average YCbCr values for the floor area
    if (count > 0) {
        avg_y = y_sum / count;
        avg_cb = cb_sum / count;
        avg_cr = cr_sum / count;

        // Margins for color detection
        Y_MARGIN = 45; 
        CB_MARGIN = 25;
        CR_MARGIN = 15;

        // Updating global variables for ground detection thresholds
        cod_lum_min3 = avg_y > Y_MARGIN ? avg_y - Y_MARGIN : 0;
        cod_lum_max3 = avg_y + 10 + Y_MARGIN > 255 ? 255 : avg_y + Y_MARGIN;
        cod_cb_min3 = avg_cb > CB_MARGIN ? avg_cb - CB_MARGIN : 0;
        cod_cb_max3 = avg_cb + CB_MARGIN > 255 ? 255 : avg_cb + CB_MARGIN;
        cod_cr_min3 = avg_cr > CR_MARGIN ? avg_cr - CR_MARGIN : 0;
        cod_cr_max3 = avg_cr + CR_MARGIN > 255 ? 255 : avg_cr + CR_MARGIN;

        // For simulation use predifined values
        // cod_lum_min3 = 0;
        // cod_lum_max3 = 255;
        // cod_cb_min3 = 0;
        // cod_cb_max3 = 110;
        // cod_cr_min3 = 0;
        // cod_cr_max3 = 130;
    }
  }


/**
 * Initiates ground color calibration.
 * This function is called to start the calibration process for ground detection,
 * setting the flag to indicate that calibration needs to be performed.
 */
void start_ground_calibration() {
    // Enables the ground calibration process to be performed again
    ground_calibration_done = false;
}

/**
 * Processes an image for object, ground, or plant detection based on the specified filter.
 * It performs color filtering and centroid detection for the object, computes floor count
 * in a central area for boundary awareness, and identifies plants in a specified region,
 * adapting to the environment dynamically through calibration.
 *
 * @param img The input image to process.
 * @param filter Specifies which detection filter to apply (1 for object, 2 for ground/plant).
 * @return The processed image, potentially with visual markers if drawing is enabled.
 */
static struct image_t *object_detector(struct image_t *img, uint8_t filter)
{
  uint8_t lum_min, lum_max;
  uint8_t cb_min, cb_max;
  uint8_t cr_min, cr_max;
  bool draw;


// Check if we need to perform initial ground calibration
  if (!ground_calibration_done && filter == 2) {
    // Perform the ground calibration
    calibrate_floor_color(img);
    ground_calibration_done = true; // Mark calibration as done
      
    // Apply the new calibration thresholds for ground detection
    cod_lum_min2 = cod_lum_min3;
    cod_lum_max2 = cod_lum_max3;
    cod_cb_min2 = cod_cb_min3;
    cod_cb_max2 = cod_cb_max3;
    cod_cr_min2 = cod_cr_min3;
    cod_cr_max2 = cod_cr_max3;
  }

  // update_color_detection_thresholds();

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

  int32_t x_c, y_c;
  // get amount of green in central part
  uint32_t floor_count_central = find_floor_count_central(img, lum_min, lum_max, cb_min, cb_max, cr_min, cr_max);
  // detect_plant in upper central area
  uint32_t plant_count = find_plant_count(img, lum_min, lum_max, cb_min, cb_max, cr_min, cr_max);
  // Filter and find centroid
  uint32_t count = find_object_centroid(img, &x_c, &y_c, draw, lum_min, lum_max, cb_min, cb_max, cr_min, cr_max);
  VERBOSE_PRINT("Color count %d: %u, threshold %u, x_c %d, y_c %d\n", camera, object_count, count_threshold, x_c, y_c);
  VERBOSE_PRINT("centroid %d: (%d, %d) r: %4.2f a: %4.2f\n", camera, x_c, y_c,
        hypotf(x_c, y_c) / hypotf(img->w * 0.5, img->h * 0.5), RadOfDeg(atan2f(y_c, x_c)));

  // Update global filter results with the detection counts and centroid location
  pthread_mutex_lock(&mutex);
  global_filters[filter-1].color_count = count;
  global_filters[filter-1].color_ground_count = floor_count_central;
  global_filters[filter-1].color_plant_count = plant_count;
  global_filters[filter-1].x_c = x_c;
  global_filters[filter-1].y_c = y_c;
  global_filters[filter-1].updated = true;
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

/**
 * Initializes the color object detector module.
 * It sets up initial filter thresholds, drawing flags, and adds the detector to the camera devices
 * based on configuration. It also initializes mutexes and resets calibration flags.
 */
void color_object_detector_init(void)
{
  memset(global_filters, 0, 2*sizeof(struct color_object_t));
  pthread_mutex_init(&mutex, NULL); // Initialize the mutex for thread safety
    ground_calibration_done = true; // Initially set to true, indicating no immediate calibration is needed

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
  cod_lum_min2 = COLOR_OBJECT_DETECTOR_LUM_MIN2; // Initial threshold values for ground detection
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
 * find_object_centroid
 *
 * Finds the centroid of pixels in an image within filter bounds.
 * Also returns the amount of pixels that satisfy these filter bounds.
 *
 * @param img - input image to process formatted as YUV422.
 * @param p_xc - x coordinate of the centroid of color object
 * @param p_yc - y coordinate of the centroid of color object
 * @param lum_min - minimum y value for the filter in YCbCr colorspace
 * @param lum_max - maximum y value for the filter in YCbCr colorspace
 * @param cb_min - minimum cb value for the filter in YCbCr colorspace
 * @param cb_max - maximum cb value for the filter in YCbCr colorspace
 * @param cr_min - minimum cr value for the filter in YCbCr colorspace
 * @param cr_max - maximum cr value for the filter in YCbCr colorspace
 * @param draw - whether or not to draw on image
 * @return number of pixels of image within the filter bounds.
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

  // Go through all the pixels
  for (uint16_t y = 0; y < img->h; y++) {
    for (uint16_t x = 0; x < img->w; x ++) {
      // Check if the color is inside the specified values
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
      if ( (*yp >= lum_min) && (*yp <= lum_max) &&
           (*up >= cb_min ) && (*up <= cb_max ) &&
           (*vp >= cr_min ) && (*vp <= cr_max )) {
        cnt ++;
        tot_x += x;
        tot_y += y;
        if (draw){
          *yp = 255;  // make pixel brighter in image
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

/**
 * Calculates the number of pixels in a central area of the floor that match the specified color range.
 * This function is primarily used for detecting the presence of the floor in the central part of the image.
 *
 * @param img Input image in YUV422 format.
 * @param lum_min Minimum Y value for color filtering.
 * @param lum_max Maximum Y value for color filtering.
 * @param cb_min Minimum Cb value for color filtering.
 * @param cb_max Maximum Cb value for color filtering.
 * @param cr_min Minimum Cr value for color filtering.
 * @param cr_max Maximum Cr value for color filtering.
 * @return The count of floor pixels in the central area.
 */
uint32_t find_floor_count_central(struct image_t *img,
                              uint8_t lum_min, uint8_t lum_max,
                              uint8_t cb_min, uint8_t cb_max,
                              uint8_t cr_min, uint8_t cr_max)
{
  uint32_t cnt = 0;
  uint8_t *buffer = img->buf;

  // Go through all the pixels
  for (uint16_t y = img->h * 0.4; y < img->h * 0.6; y++) {
    for (uint16_t x = 0; x < img->w * 0.3; x ++) {
      // Check if the color is inside the specified values
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
      if ( (*yp >= lum_min) && (*yp <= lum_max) &&
           (*up >= cb_min ) && (*up <= cb_max ) &&
           (*vp >= cr_min ) && (*vp <= cr_max )) {
        cnt ++;
      }
    }
  }
  return cnt;
}

/**
 * Calculates the number of pixels matching the specified color range that are likely to be part of plants.
 * This function aims at identifying plant areas in the specified part of the image.
 *
 * @param img Input image in YUV422 format.
 * @param lum_min Minimum Y value for color filtering.
 * @param lum_max Maximum Y value for color filtering.
 * @param cb_min Minimum Cb value for color filtering.
 * @param cb_max Maximum Cb value for color filtering.
 * @param cr_min Minimum Cr value for color filtering.
 * @param cr_max Maximum Cr value for color filtering.
 * @return The count of pixels likely to be part of plants.
 */
uint32_t find_plant_count(struct image_t *img,
                              uint8_t lum_min, uint8_t lum_max,
                              uint8_t cb_min, uint8_t cb_max,
                              uint8_t cr_min, uint8_t cr_max)
{
  uint32_t cnt = 0;
  uint8_t *buffer = img->buf;

  // Go through all the pixels
  for (uint16_t y = img->h * 0.3; y < img->h * 0.7; y++) {
    for (uint16_t x = img->w * 0.65; x < img->w; x ++) {
      // Check if the color is inside the specified values
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
      if ( (*yp >= lum_min) && (*yp <= lum_max) &&
           (*up >= cb_min ) && (*up <= cb_max ) &&
           (*vp >= cr_min ) && (*vp <= cr_max )) {
        cnt ++;
      }
    }
  }
  return cnt;
}

/**
 * Periodically checks the updated status of the detected color objects and sends
 * ABI messages for each detection. This function ensures that other modules in the
 * Paparazzi system can receive and process the detection results for further actions.
 */
void color_object_detector_periodic(void)
{
  static struct color_object_t local_filters[2];
  pthread_mutex_lock(&mutex);
  memcpy(local_filters, global_filters, 2*sizeof(struct color_object_t));
  pthread_mutex_unlock(&mutex);

  // Check if the first filter's results are updated and send ABI message
  if(local_filters[0].updated){
    AbiSendMsgVISUAL_DETECTION(COLOR_OBJECT_DETECTION1_ID, local_filters[0].x_c, local_filters[0].y_c,
        0, 0, local_filters[0].color_count, 0);
    local_filters[0].updated = false;
  }

  // Check if the second filter's results are updated and send multiple ABI messages
  // for different types of detections: ground, central ground, and plant
  if(local_filters[1].updated){
    AbiSendMsgVISUAL_DETECTION(COLOR_OBJECT_DETECTION2_ID, local_filters[1].x_c, local_filters[1].y_c,
        0, 0, local_filters[1].color_count, 1);
    AbiSendMsgVISUAL_DETECTION(COLOR_OBJECT_DETECTION3_ID, local_filters[1].x_c, local_filters[1].y_c,
        0, 0, local_filters[1].color_ground_count, 2);
    AbiSendMsgVISUAL_DETECTION(COLOR_OBJECT_DETECTION4_ID, local_filters[1].x_c, local_filters[1].y_c,
        0, 0, local_filters[1].color_plant_count, 2);
    local_filters[1].updated = false;
  }
}

