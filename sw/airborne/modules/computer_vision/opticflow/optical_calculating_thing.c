#include "std.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>

// Own Header
#include "opticflow_calculator_dense.h"

// Computer Vision
#include "lib/vision/image.h"
#include "modules/sonar/agl_dist.h"
// Header for our c++ file that has the openCV function
#include "get_flow.h"
#include "linear_flow_fit.h"
#include "image.h"
// to get the definition of front_camera / bottom_camera
#include BOARD_CONFIG
// Parameters for the cvOpticalFlowFarneback function
#ifndef OPTICFLOW_WINDOW_SIZE
#define OPTICFLOW_WINDOW_SIZE 10
#endif
PRINT_CONFIG_VAR(OPTICFLOW_WINDOW_SIZE)

#ifndef OPTICFLOW_PYR_SCALE
#define OPTICFLOW_PYR_SCALE 0.1
#endif
PRINT_CONFIG_VAR(OPTICFLOW_PYR_SCALE)

#ifndef OPTICFLOW_LEVELS
#define OPTICFLOW_LEVELS 3
#endif
PRINT_CONFIG_VAR(OPTICFLOW_LEVELS)

#ifndef OPTICFLOW_POLY_N
#define OPTICFLOW_POLY_N 2
#endif
PRINT_CONFIG_VAR(OPTICFLOW_POLY_N)

#ifndef OPTICFLOW_POLY_SIGMA
#define OPTICFLOW_POLY_SIGMA 3.0
#endif
PRINT_CONFIG_VAR(OPTICFLOW_POLY_SIGMA)

#ifndef OPTICFLOW_FLAGS
#define OPTICFLOW_FLAGS 0
#endif
PRINT_CONFIG_VAR(OPTICFLOW_FLAGS)

#ifndef OPTICFLOW_MAX_ITERATIONS
#define OPTICFLOW_MAX_ITERATIONS 2
#endif
PRINT_CONFIG_VAR(OPTICFLOW_MAX_ITERATIONS)

#ifndef OPTICFLOW_ERROR_THRESHOLD
#define OPTICFLOW_ERROR_THRESHOLD 10
#endif

#ifndef OPTICFLOW_N_ITERATIONS
#define OPTICFLOW_N_ITERATIONS 100
#endif

#ifndef OPTICFLOW_N_SAMPLES
#define OPTICFLOW_N_SAMPLES 3
#endif

#define PRINT(string, ...) fprintf(stderr, "[mav_exercise->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)

// Function from the original file this was based on, might use
/* Functions only used here */
static uint32_t timeval_diff(struct timeval *starttime, struct timeval *finishtime);

struct linear_flow_fit_info *info;
struct linear_flow_fit_info INFO;
info = &INFO;

struct flow_t *opt_vect;
struct flow_t OPT_VECT;
opt_vect = &OPT_VECT;

int counterr = 0;
uint16_t height_crop = 0;
uint16_t width_crop = 0;
bool result_analyzer = false;
//float actual_div = 0.f;
/**
 * Initialize the opticflow calculator
 * @param[out] *opticflow The new optical flow calculator
 */

// Initializing the values for the settings for the
// OpenCV function that will be used, stored in the
// opticflow struct from the original file
void opticflow_calc_init(struct opticflow_t *opticflow)
{
  /* Set the default values */
  opticflow->window_size = OPTICFLOW_WINDOW_SIZE;
  opticflow->pyr_scale = OPTICFLOW_PYR_SCALE;
  opticflow->levels = OPTICFLOW_LEVELS;
  opticflow->poly_n = OPTICFLOW_POLY_N;
  opticflow->poly_sigma = OPTICFLOW_POLY_SIGMA;
  opticflow->flags = OPTICFLOW_FLAGS;
  opticflow->max_iterations = OPTICFLOW_MAX_ITERATIONS;

}

// Function that does the 'organizing' of the incoming images, and calls the c++ function
// which will then call the openCV function.
static bool calc_opticfarneback(struct opticflow_t *opticflow, struct image_t *img, float *div)
{
	// Checking if the very first image has been received
	if (!opticflow->got_first_img) {
		// Creating images to store the incoming images
		image_create(&opticflow->img_gray, img->w, img->h, IMAGE_YUV422);
		image_create(&opticflow->prev_img_gray, img->w, img->h, IMAGE_YUV422);

		// Store very first image in the opticflow struct
		image_copy(img, &opticflow->prev_img_gray);

		// Set this to true so it won't go into this condition again
		opticflow->got_first_img = true;

		// Return false since no calculation yet
		return false;
	}

	// We should have previous image from the previous loop, so store 'current' in opticflow struct
	image_copy(img, &opticflow->img_gray);

	// Clock so that we can time the optical flow calculation function
	clock_t start, end;
	double cpu_time_used;

	start = clock();

	// This is the main function where the optic flow is calculated
	//get_flow(opticflow->prev_img_gray.buf, opticflow->img_gray.buf, opticflow->pyr_scale, opticflow->levels, opticflow->window_size,
	  //opticflow->max_iterations, opticflow->poly_n, opticflow->poly_sigma, opticflow->flags,
	  //of_diff, div, img->w, img->h);
  opt_vect, counterr, width_crop, height_crop = determine_flow(opticflow->prev_img_gray.buf, opticflow->img_gray.buf, opticflow->pyr_scale, opticflow->levels, opticflow->window_size,
	  opticflow->max_iterations, opticflow->poly_n, opticflow->poly_sigma, opticflow->flags, img->w, img->h);
    
  result_analyzer = analyze_linear_flow_field(opt_vect, counterr, OPTICFLOW_ERROR_THRESHOLD, OPTICFLOW_N_ITERATIONS, OPTICFLOW_N_SAMPLES, width_crop, height_crop, info);
  if(!result_analyzer){
    return false;
  }
  *div = info->divergence;
	end = clock();
	cpu_time_used = ((double) (end - start)) / CLOCKS_PER_SEC; // Calculating elapsed time
	PRINT("Time taken %lf \n", cpu_time_used);
//	image_copy(&opticflow->prev_img_gray, img);                // This is only needed if you want to show
	                                                           // the 'colored' optic flow image
	// Put current image in previous to be ready for next loop
	image_switch(&opticflow->img_gray, &opticflow->prev_img_gray);
  
  return true;
}

// Also essentiallly from original file we based this on
/**
 * Run the optical flow on a new image frame
 * @param[in] *opticflow The opticalflow structure that keeps track of previous images
 * @param[in] *state The state of the drone
 * @param[in] *img The image frame to calculate the optical flow from
 * @param[out] *result The optical flow result
 */
bool opticflow_calc_module_call(struct opticflow_t *opticflow, struct image_t *img, float *div)
{
  bool flow_successful = false;
  flow_successful = calc_opticfarneback(opticflow, img, div);   // calc_farneback defined just above

  return flow_successful;
}

// From original file, might be useful
/**
 * Calculate the difference from start till finish
 * @param[in] *starttime The start time to calculate the difference from
 * @param[in] *finishtime The finish time to calculate the difference from
 * @return The difference in milliseconds
 */
static uint32_t timeval_diff(struct timeval *starttime, struct timeval *finishtime)
{
  uint32_t msec;
  msec = (finishtime->tv_sec - starttime->tv_sec) * 1000;
  msec += (finishtime->tv_usec - starttime->tv_usec) / 1000;
  return msec;
}