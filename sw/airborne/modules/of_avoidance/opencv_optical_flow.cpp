/**
 * This file is created by TEAM 2 from 2023 for the AFMAV course
 */

#include "opencv_optical_flow.h"
#include <stdio.h>
#include <cstdint>

using namespace std;
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
using namespace cv;
#include "opencv_image_functions.h"
#include <algorithm>
#include <stdlib.h>
#include <chrono>
#include <thread>


// All these parameters can be set in conf/modules/opencv_optical_flow.xml
#ifndef DET_THRESHOLD // Divergence cutoff threshold
#define DET_THRESHOLD 0.14
#endif
#ifndef RESIZE // Resize factor use for the camera image in x and y direction
#define RESIZE 0.25
#endif
#ifndef RESIZE_FX
#define RESIZE_FX RESIZE
#endif
#ifndef RESIZE_FY
#define RESIZE_FY RESIZE
#endif
#ifndef OFF_PYR_SCALE // Farneback optical flow setting
#define OFF_PYR_SCALE 0.5
#endif
#ifndef OFF_LEVELS // Farneback optical flow setting
#define OFF_LEVELS 3
#endif
#ifndef OFF_WINSIZE // Farneback optical flow setting
#define OFF_WINSIZE 10
#endif
#ifndef OFF_ITERATIONS // Farneback optical flow setting
#define OFF_ITERATIONS 1
#endif
#ifndef OFF_POLY_N // Farneback optical flow setting
#define OFF_POLY_N 1
#endif
#ifndef OFF_POLY_SIGMA // Farneback optical flow setting
#define OFF_POLY_SIGMA 1.2
#endif
#ifndef OFF_FLAGS // Farneback optical flow setting
#define OFF_FLAGS 0
#endif

double param_DET_THRESHOLD = DET_THRESHOLD;
double param_RESIZE_FX = RESIZE_FX;
double param_RESIZE_FY = RESIZE_FY;
double param_OFF_PYR_SCALE = OFF_PYR_SCALE;
int param_OFF_LEVELS = OFF_LEVELS;
int param_OFF_WINSIZE = OFF_WINSIZE;
int param_OFF_ITERATIONS = OFF_ITERATIONS;
int param_OFF_POLY_N = OFF_POLY_N;
double param_OFF_POLY_SIGMA = OFF_POLY_SIGMA;
double param_OFF_FLAGS = OFF_FLAGS;

// These values are applied to the original image, which has size (w x h) 520 x 240
#ifndef CROP_X // Crop offset in x-direction before resizing for the left and from the right [0, 520 / 2]
#define CROP_X 60
#endif
#ifndef CROP_Y // Crop offset in y-direction before resizing (so from the top) [0, 240]
#define CROP_Y 50
#endif
#ifndef CROP_HEIGHT // Crop size in y-direction before resizing [0, 240]
#define CROP_HEIGHT 50
#endif

int param_CROP_X = CROP_X * RESIZE_FX;
int param_CROP_Y = CROP_Y  * RESIZE_FY;  // 50 * 0.25
int param_CROP_WIDTH = 520 * RESIZE_FX -  2 * param_CROP_X; // (520 * 0.25) - 2 * 15 = 100
int param_CROP_HEIGHT = CROP_HEIGHT * RESIZE_FY;  // Total 240 -> [50, 100]

//
#ifndef N_DIRBLOCKS // The number of blocks over which to divide the field of view (should be odd)
#define N_DIRBLOCKS 5
#endif
#ifndef M_DIRFILTER // The number of adjacent blocks to take into account to determine the no. of obstacles for a certain direction
#define M_DIRFILTER 1
#endif
#ifndef PERCENTAGE_HISTORY_IMPORTANCE // The extent to which the past detections are used when determining the new detections [0.0, 1.0]
#define PERCENTAGE_HISTORY_IMPORTANCE 0.7
#endif

double param_PERCENTAGE_HISTORY_IMPORTANCE = PERCENTAGE_HISTORY_IMPORTANCE;

#ifndef PREFERRED_INDEX
#define PREFERRED_INDEX N_DIRBLOCKS / 2
#endif
#ifndef SIDE_PENALTY
#define SIDE_PENALTY 0.05
#endif

int param_PREFERRED_INDEX = PREFERRED_INDEX;
double param_SIDE_PENALTY = SIDE_PENALTY;

Mat old_frame_grayscale; // Store the previous grayscale frame for the optical flow
double detection_history[N_DIRBLOCKS] = {}; // The history of detections per block (the FOV is divided in N_DIRBLOCKS)

// Function to find the best direction to go when given a certain detection array.
// The detection array stores for every pixel in x-direction of the resized image if there is an obstacle or not.
int find_best_direction_index(const cv::Mat &detection_horizon) {

  // Initialise array to hold values of detection horizon
  std::vector<float> array;

  // Just to be safe, check if Mat is continuous
  if (detection_horizon.isContinuous())

    // Assign Mat to array, mapping all values to it
    array.assign((float*)detection_horizon.data, (float*)detection_horizon.data + detection_horizon.total() * detection_horizon.channels());

  // Determine pixel width of detection horizon
  int horizon_width = detection_horizon.rows;

  // Determine the amount of pixels/array elements in one block (the FOV is divided in N_DIRBLOCKS)
  int pixels_per_block = horizon_width / N_DIRBLOCKS;

  double detections_per_block[N_DIRBLOCKS] = {};
  double updated_detection_history[N_DIRBLOCKS] = {};

  // Loop through discrete direction blocks
  // Sum up all the detections inside one block
  for (int n = 0; n < N_DIRBLOCKS; n++) {

      // Determine low and high index of block
      int lowest_index = n * pixels_per_block;
      int highest_index = std::min((n + 1) * pixels_per_block, horizon_width) - 1;

      // Variable to accumulate sum of detections
      double detection_count = 0;

      // Loop through indices belonging to this block
      for (int i = lowest_index; i <= highest_index; i++) {

          // Accumulate sum of detections in that block
          detection_count += 1 - array[i];

      }

      // Save the total number of detections inside a block
      detections_per_block[n] = detection_count;

  }

  // Loop through direction blocks again to add the detection within the neighbour blocks with a weight of 0.5
  // This means that blocks close to a block with many detections are also less likely to be chosen.
  for (int i = 0; i < N_DIRBLOCKS; i++) {

      // Variable to store filtered (convoluted) detections
      double filtered_detection_sum = 0;

      // Loop through convolution filter
      for (int j = -M_DIRFILTER; j <= M_DIRFILTER; j++) {

          // Check if the index is not out of bounds
          if (i + j >= 0 && i + j < N_DIRBLOCKS) {

              // Add the 'detection score' to the total; weigh the neighbours to be worth less
              filtered_detection_sum += (1.0 / (std::abs(j) + 1.0)) * detections_per_block[i + j];

          }

      }

      filtered_detection_sum += std::abs(i - param_PREFERRED_INDEX) * param_SIDE_PENALTY;

      // Weigh the detections and take historical detections into account as well
      updated_detection_history[i] = (1 - param_PERCENTAGE_HISTORY_IMPORTANCE) * filtered_detection_sum + param_PERCENTAGE_HISTORY_IMPORTANCE * detection_history[i];

  }

  // Copy updated detection history to the history array
  std::copy(std::begin(updated_detection_history), std::end(updated_detection_history), std::begin(detection_history));

  // Variables to hold 'best' index and score
  double lowest_filtered_score = 10000;
  int lowest_filtered_index = -1;

  // Loop through direction blocks again and find the block with the lowest value. This is the best index to go.
  for (int i = 0; i < N_DIRBLOCKS; i++) {

      // If it finds a better score...
      if (detection_history[i] < lowest_filtered_score) {

          // Set to this newest best index and score
          lowest_filtered_score = detection_history[i];
          lowest_filtered_index = i;

      }
  }

  return lowest_filtered_index;
}

// Calculate the divergence of the optical flow
// The input is an image with 2 channels representing respectively the flow in horizontal and vertical direction
cv::Mat calculate_divergence(const cv::Mat& flow)
{
  // Split the two channels of the flow Mat into separate Mats
  std::vector<cv::Mat> flow_channels;
  cv::split(flow, flow_channels);
  cv::Mat flow_x = flow_channels[0];
  cv::Mat flow_y = flow_channels[1];

  // Initialize output Mat with zeros
  cv::Mat divergence = cv::Mat::zeros(flow_x.size(), CV_32FC1);

  // Create Mat to hold x and y derivatives of flow
  cv::Mat dx, dy, dx_abs, dy_abs;

  // Create kernels for the center-difference in x and y direction
  Mat kernelx = (Mat_<float>(1, 3) << -0.5, 0, 0.5);
  Mat kernely = (Mat_<float>(3, 1) << -0.5, 0, 0.5);

  // Apply kernels and get result
  cv::filter2D(flow_x, dx, -1, kernelx);
  cv::filter2D(flow_y, dy, -1, kernely);

  // Take absolute value to get both strongly positive and strongly negative values
  dx_abs = cv::abs(dx);
  dy_abs = cv::abs(dy);

  // Calculate divergence over x and y axis
  cv::Mat div_x = dx_abs + dy_abs;

  // Store the computed divergence in the output Mat
  div_x.copyTo(divergence);

  return divergence;
}

int opencv_main(char *img, int width, int height) {

    // Cast the image struct into an opencv Mat
    Mat frame(height, width, CV_8UC2, img);

    // Convert the YUV input to grayscale
    Mat frame_grayscale, gray_resized;
    cv::cvtColor(frame, frame_grayscale, CV_YUV2GRAY_Y422);

    // Resize the gray frame
    cv::resize(frame_grayscale, gray_resized, Size(), param_RESIZE_FX, param_RESIZE_FY);

    // Define and apply the crop to get a small window to apply optical flow on, since optical flow requires a lot of
    // computational power
    cv::Rect cropped_region(gray_resized.cols - param_CROP_HEIGHT - param_CROP_Y, param_CROP_X, param_CROP_HEIGHT, param_CROP_WIDTH);
    Mat cropped_gray_frame = gray_resized(cropped_region);

    // If there is no previous frame (only in the first step) use the current frame as the previous frame
    // This will indeed result in zero optical flow
    if (old_frame_grayscale.empty()) {
        cropped_gray_frame.copyTo(old_frame_grayscale);
    }

    // Apply the Farneback optical flow to a grayscale part the image by using an OpenCV function
    Mat flow_field;
    cv::calcOpticalFlowFarneback(old_frame_grayscale, cropped_gray_frame, flow_field, param_OFF_PYR_SCALE,
                                 param_OFF_LEVELS, param_OFF_WINSIZE, param_OFF_ITERATIONS, param_OFF_POLY_N,
                                 param_OFF_POLY_SIGMA, param_OFF_FLAGS);

    // Save the current frame to be used as the previous frame in the next iteration
    cropped_gray_frame.copyTo(old_frame_grayscale);

    // Calculate the divergence with our own function also in this file
    Mat output;
    output = calculate_divergence(flow_field);

    // Calculate the mean divergence in the height direction of the image,
    // so we are left with an 1D array of values along the width of the image
    Mat column_mean;
    cv::reduce(output, column_mean, 1, cv::REDUCE_AVG);

    // Apply a threshold to the mean divergence to determine whether there is an obstacle or not.
    Mat thresholded_divergence;
    cv::threshold(column_mean, thresholded_divergence, param_DET_THRESHOLD, 1, THRESH_BINARY_INV);

    // ==== Visual the thresholded detections as a 'barcode' on the camera stream ====
    Mat divergence_img;
    // Give the 1D array with the thresholded divergence the right size to be laid on the image
    cv::resize(thresholded_divergence, divergence_img, Size(50, 520));
    divergence_img = divergence_img * 255;
    divergence_img.convertTo(divergence_img, CV_8U);
    // Lay the 'barcode' on the grayscale image
    divergence_img.copyTo(frame_grayscale(cv::Rect(0, 0, divergence_img.cols, divergence_img.rows)));
    // ==== END ====

    // Call our own function `find_best_direction_index` to determine the index
    // of the block (the FOV is divided in N_DIRBLOCKS) which is the best way to go
    int lowest_detection_index = find_best_direction_index(thresholded_divergence);

    // ==== Visual the best index by highlighting the specific block with a white box ====
    int box_width = frame_grayscale.rows / N_DIRBLOCKS;
    cv::Rect rect(50, lowest_detection_index * box_width, 190, box_width);
    cv::rectangle(frame_grayscale, rect, 255);
    // ==== END ====

    // Convert the grayscale image back to YUV so the video thread can handle it
    grayscale_opencv_to_yuv422(frame_grayscale, img, width, height);

    // Return the best index
    return lowest_detection_index;
}
