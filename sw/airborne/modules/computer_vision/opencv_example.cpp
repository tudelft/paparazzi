#include "state.h"
#include "modules/core/abi.h"
#include <time.h>

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv_example.h"
#include "opencv2/video/tracking.hpp"
#include <opencv2/core/types.hpp>

#define MOVING_MEAN_COUNT 3
#define LEFT_IDX 0
#define RIGHT_IDX 1
#define MIDDLE_IDX 2
using namespace cv;

int frame_id = 0;
float flow_arr[MOVING_MEAN_COUNT][3];
Mat previous_frame_left, previous_frame_right, previous_frame_middle;
void scale_mat(const Mat matrix, Mat& matrix_left, Mat& matrix_right, Mat& matrix_middle, const int width, const int height, const int width_img, const int height_img);
void scale_mat(const Mat matrix, Mat& matrix_left, Mat& matrix_right, Mat& matrix_middle, const int width, const int height, const int width_img, const int height_img)
{
  auto range_width = Range((int) (width_img/2 - width/2),(int) (width_img/2 + width/2));
  matrix_left = matrix(range_width, Range((int) (height_img/2 - height/2),(int) (height_img/2)));
  matrix_middle = matrix(range_width, Range((int) (height_img/2 - height/4),(int) (height_img/2 + height/4)));
  matrix_right = matrix(range_width, Range((int) (height_img/2),(int) (height_img/2 + height/2)));
  
}
void calculate_magnitudes_flow(Mat& mag, const Mat prvs, const Mat next);
void calculate_magnitudes_flow(Mat& mag, const Mat prvs, const Mat next)
{
    Mat flow(prvs.size(), CV_32FC1);
    calcOpticalFlowFarneback(prvs, next, flow, 0.5, 3, 15, 3, 5, 1.2, 0);      
    Mat flow_parts[2];
    split(flow, flow_parts);
    magnitude(flow_parts[0], flow_parts[1], mag);
}

void calculate_output_flow(const Mat mag, float* output_flow, const int idx);
void calculate_output_flow(const Mat mag, float* output_flow, const int idx)
{
  float flow_sum = 0;
  for (int i=0; i<mag.rows; i++)
    {
      for (int j=0; j<mag.cols; j++)
      {
        flow_sum+=mag.at<float>(i,j);
      }
    }

  flow_arr[frame_id%MOVING_MEAN_COUNT][idx] = flow_sum;

  float flow_sum_tot = 0.0;
  for (int i=0; i<MOVING_MEAN_COUNT; i++)
    {
      flow_sum_tot += flow_arr[i][idx];
    }
  output_flow[idx] = flow_sum_tot;
}


void farneback(char *img, float* output_flow, int width, int height, int width_img, int height_img)
{
    std::cout<<"farneback"<<"\n";
    Mat next_frame(width_img, height_img, CV_8UC2, img); 
    
    Mat next_frame_gray;

    cvtColor(next_frame, next_frame_gray, CV_YUV2GRAY_Y422);

    Mat next_frame_left, next_frame_right, next_frame_middle;
    scale_mat(next_frame_gray, next_frame_left, next_frame_right, next_frame_middle, width, height, width_img, height_img);

    if (frame_id==0)
    {
    for (int i=0; i<MOVING_MEAN_COUNT; i++)
    {
      for (int j=0; j<3; j++)
      {
        flow_arr[i][j] = 0.0;
      }
    }
      previous_frame_left = next_frame_left;
      previous_frame_middle = next_frame_middle;
      previous_frame_right = next_frame_right;
      frame_id++;
      return;
    }  
    Mat mag_left(next_frame_left.size(), CV_32FC1);
    calculate_magnitudes_flow(mag_left, previous_frame_left, next_frame_left);
    
    Mat mag_right(next_frame_right.size(), CV_32FC1);
    calculate_magnitudes_flow(mag_right, previous_frame_right, next_frame_right);

    Mat mag_middle(next_frame_middle.size(), CV_32FC1);
    calculate_magnitudes_flow(mag_middle, previous_frame_middle, next_frame_middle);

    calculate_output_flow(mag_left, output_flow, LEFT_IDX);
    calculate_output_flow(mag_right, output_flow, RIGHT_IDX);
    calculate_output_flow(mag_middle, output_flow, MIDDLE_IDX);

      previous_frame_left = next_frame_left;
      previous_frame_right = next_frame_right;
      previous_frame_middle = next_frame_middle;
      std::cout<<"left: "<<output_flow[0]<<", right: "<<output_flow[1]<<"middle: "<<output_flow[2]<<"\n"; 
      frame_id++;
}


// int main()
// {
//   std::string filename = "/home/matthijs/paparazzi/videos/vlc-record-2023-03-10-09h59m06s-rtp_5000.sdp-.avi";
//   farneback_playback(filename, 200, 50);
//   return 0;
// }

