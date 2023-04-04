#include "state.h"
#include "modules/core/abi.h"
#include <time.h>

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "farneback_calculator.h"
#include "opencv2/video/tracking.hpp"
#include <opencv2/core/types.hpp>

// Logging function for profiling purposes
#define LOG(x) fprintf(stderr, "LOG: %s:%d %s %lu \n", __FILE__, __LINE__, x, clock()); 



#define MOVING_MEAN_COUNT 3 // We use 3 consecutive flow values to be less sensitive to noise in the data
#define LEFT_IDX 0
#define RIGHT_IDX 1
#define MIDDLE_IDX 2
using namespace cv;

int frame_id = 0;
float flow_arr[MOVING_MEAN_COUNT][3];
Mat previous_frame_left, previous_frame_right, previous_frame_middle;
Mat previous_frame_all;
void scale_mats(const Mat matrix, Mat& matrix_left, Mat& matrix_right, Mat& matrix_middle, const int width, const int height);
void scale_mats(const Mat matrix, Mat& matrix_left, Mat& matrix_right, Mat& matrix_middle, const int width, const int height)
{
  // Divide the computed magnitudes up into the 3 respective areas: left, middle and right
  // All areas are equally big
  auto range_width = Range(0, width);
  matrix_left = matrix(range_width, Range(0, (int) height/2));
  matrix_middle = matrix(range_width, Range((int) height/4, (int) 3*height/4));
  matrix_right = matrix(range_width, Range((int) height/2, height));
}
void scale_mat_whole(const Mat matrix, Mat& matrix_all, const int width, const int height, const int width_img, const int height_img);
void scale_mat_whole(const Mat matrix, Mat& matrix_all, const int width, const int height, const int width_img, const int height_img)
{
  // Scale down the image to only use a section of the entire image to cut down computation time
    auto range_width = Range((int) (width_img/2 - width/2),(int) (width_img/2 + width/2));
    matrix_all = matrix(range_width, Range((int) (height_img/2 - height/2),(int) (height_img/2 + height/2)));
}

void calculate_magnitudes_flow(Mat& mag, const Mat prvs, const Mat next);
void calculate_magnitudes_flow(Mat& mag, const Mat prvs, const Mat next)
{
    Mat flow(prvs.size(), CV_32FC1);
    calcOpticalFlowFarneback(prvs, next, flow, 0.5, 3, 15, 3, 5, 1.2, 0); // Calculate the flow vectors  
    Mat flow_parts[2];
    split(flow, flow_parts); // Split the flow up into 2 dimensional flow vectors
    magnitude(flow_parts[0], flow_parts[1], mag); // Compute the magnitude of these vectors
}

void calculate_output_flow(const Mat mag, float* output_flow, const int idx);
void calculate_output_flow(const Mat mag, float* output_flow, const int idx)
{
  // Compute the sum of all the magnitudes of the flow vectors
  float flow_sum = 0;
  for (int i=0; i<mag.rows; i++)
    {
      for (int j=0; j<mag.cols; j++)
      {
        flow_sum+=mag.at<float>(i,j);
      }
    }

  flow_arr[frame_id%MOVING_MEAN_COUNT][idx] = flow_sum; // Assign flow value to element in the array where we store the flow values

  float flow_sum_tot = 0.0;
  for (int i=0; i<MOVING_MEAN_COUNT; i++) // Use the buffer to get data less sensitive to noise
    {
      flow_sum_tot += flow_arr[i][idx];
    }
  output_flow[idx] = flow_sum_tot;
}


void farneback(char *img, float* output_flow, int width, int height, int width_img, int height_img)
{
    LOG("start farneback")

    Mat next_frame(width_img, height_img, CV_8UC2, img); 
    
    Mat next_frame_gray;

    cvtColor(next_frame, next_frame_gray, CV_YUV2GRAY_Y422); // Convert frame to gray to use it for farneback

    Mat next_frame_all;
    scale_mat_whole(next_frame_gray, next_frame_all, width, height, width_img, height_img); // Scale down the whole window

    if (frame_id==0)
    {
    for (int i=0; i<MOVING_MEAN_COUNT; i++)
    {
      for (int j=0; j<3; j++)
      {
        flow_arr[i][j] = 0.0;
      }
    }
      previous_frame_all = next_frame_all;
      frame_id++;
      return;
    }  
    Mat mag_all(next_frame_all.size(), CV_32FC1);
    calculate_magnitudes_flow(mag_all, previous_frame_all, next_frame_all); // Calculate the flow

    Mat mag_left, mag_right, mag_middle; 
    scale_mats(mag_all, mag_left, mag_right, mag_middle, width, height); // Divide the flow vectors over their corresponding areas
    // Output the flow
    calculate_output_flow(mag_left, output_flow, LEFT_IDX);
    calculate_output_flow(mag_right, output_flow, RIGHT_IDX);
    calculate_output_flow(mag_middle, output_flow, MIDDLE_IDX);
    previous_frame_all = next_frame_all;
    frame_id++;
    LOG("end farneback")
}
