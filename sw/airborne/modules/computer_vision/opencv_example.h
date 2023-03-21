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
 * A simple module showing what you can do with opencv on the bebop.
 */

#ifndef OPENCV_EXAMPLE_H
#define OPENCV_EXAMPLE_H

#include "modules/computer_vision/cv.h"

#ifdef __cplusplus
extern "C" {
#endif

int opencv_example(char *img, int width, int height);
// void scale_mat(const cv::Mat matrix, cv::Mat& matrix_left, cv::Mat& matrix_right, const int width, const int height, const int width_img, const int height_img);
// void calculate_magnitudes_flow(cv::Mat& mag, cv::Mat prvs, cv::Mat next);
// extern void farneback_playback(const std::string& filename, int width, int height);
void farneback(char *img, float* output_flow, int width, int height, int width_img, int height_img);

#ifdef __cplusplus
}
#endif

#endif

// #ifndef OPTICAL_FLOW_GROUP3_H
// #define OPTICAL_FLOW_GROUP3_H
// #include "stdio.h"
// #include <opencv2/core/core.hpp>
// #include <opencv2/imgproc/imgproc.hpp>
// // #include "opencv2/core/hal/intrin.hpp"
// // #include <opencv2/highgui/highgui.hpp>
// // #include "opencv2\highgui.hpp"

// #include "opencv2/video/tracking.hpp"

// #ifdef __cplusplus
// extern "C" {
// #endif
// // extern void hi();
// extern void scale_mat(const cv::Mat matrix, cv::Mat& matrix_left, cv::Mat& matrix_right, const int width, const int height, const int width_img, const int height_img);
// extern void calculate_magnitudes_flow(cv::Mat& mag, cv::Mat prvs, cv::Mat next);
// // extern void farneback_playback(const std::string& filename, int width, int height);
// extern void farneback(char *img, float* output_flow, int width, int height, int width_img, int height_img);
// #ifdef __cplusplus
// }
// #endif
// #endif

