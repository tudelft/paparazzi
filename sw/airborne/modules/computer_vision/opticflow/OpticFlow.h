/*
 * Copyright (C) C. De Wagter
 *
 * This file is part of paparazzi
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/computer_vision/cv_opencvdemo.h"
 * @author C. De Wagter
 * opencv
 */

#include <stdint.h>



#ifndef CV_OPENCVDEMO_H
#define CV_OPENCVDEMO_H
#endif

#include "lib/vision/image.h"
#include "size_divergence.h"

// #ifndef LINEAR_FLOW_FIT
// #define LINEAR_FLOW_FIT

// // structure that contains all outputs of a linear flow fied fit:
// struct linear_flow_fit_info {
//   float slope_x;      ///< Slope of the surface in x-direction - given sufficient lateral motion
//   float slope_y;      ///< Slope of the surface in y-direction - given sufficient lateral motion
//   float surface_roughness;  ///< The error of the linear fit is a measure of surface roughness
//   float focus_of_expansion_x; ///< Image x-coordinate of the focus of expansion (contraction)
//   float focus_of_expansion_y; ///< Image y-coordinate of the focus of expansion (contraction)
//   float relative_velocity_x;  ///< Relative velocity in x-direction, i.e., vx / z, where z is the depth in direction of the camera's principal axis
//   float relative_velocity_y;  ///< Relative velocity in y-direction, i.e., vy / z, where z is the depth in direction of the camera's principal axis
//   float relative_velocity_z;  ///< Relative velocity in z-direction, i.e., vz / z, where z is the depth in direction of the camera's principal axis
//   float time_to_contact;    ///< Basically, 1 / relative_velocity_z
//   float divergence;   ///< Basically, relative_velocity_z. Actual divergence of a 2D flow field is 2 * relative_velocity_z
//   float fit_error;    ///< Error of the fit (same as surface roughness)
//   int n_inliers_u;    ///< Number of inliers in the horizontal flow fit
//   int n_inliers_v;    ///< Number of inliers in the vertical flow fit
// };

// // This is the function called externally, passing the vector of optical flow vectors and information on the number of vectors and image size:
// bool analyze_linear_flow_field(struct flow_t *vectors, int count, float error_threshold, int n_iterations, int n_samples, int im_width, int im_height, struct linear_flow_fit_info* info);

// // Fits the linear flow field with RANSAC:
// void fit_linear_flow_field(struct flow_t *vectors, int count, float error_threshold, int n_iterations, int n_samples, float *parameters_u, float *parameters_v, float *fit_error, float *min_error_u, float *min_error_v, int *n_inliers_u, int *n_inliers_v);

// // Extracts relevant information from the fit parameters:
// void extract_information_from_parameters(float *parameters_u, float *parameters_v, int im_width, int im_height, struct linear_flow_fit_info* info);


//#endif


#ifdef __cplusplus
extern "C" {
#endif

  struct flow_t *determine_flow(char* prev, char* curr, int height, int width, uint16_t winSize_i, uint16_t maxLevel, float OPTICFLOW_ERROR_THRESHOLD, int OPTICFLOW_N_ITERATIONS, int OPTICFLOW_N_SAMPLES, int *array_size);


#ifdef __cplusplus
}
#endif





