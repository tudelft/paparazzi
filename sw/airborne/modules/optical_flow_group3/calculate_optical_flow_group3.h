#ifndef OPTICAL_FLOW_GROUP3_H
#define OPTICAL_FLOW_GROUP3_H
#include "stdio.h"
#include </home/matthijs/paparazzi/sw/ext/opencv_bebop/opencv/modules/core/include/opencv2/core.hpp>

#ifdef __cplusplus
extern "C" {
#endif
extern void scale_mat(const cv::Mat matrix, cv::Mat& matrix_left, cv::Mat& matrix_right, const int width, const int height, const int width_img, const int height_img);
extern void calculate_magnitudes_flow(cv::Mat& mag, cv::Mat prvs, cv::Mat next);
extern void farneback_playback(const std::string& filename, int width, int height);
extern void farneback(char *img, float* output_flow, int width, int height, int width_img, int height_img);
#ifdef __cplusplus
}
#endif
#endif