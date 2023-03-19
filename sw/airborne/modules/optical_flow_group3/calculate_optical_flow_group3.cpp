#include <iostream>
#include "calculate_optical_flow_group3.h"
#include <opencv2/opencv.hpp>

#define MOVING_MEAN_COUNT 3

using namespace cv;

int frame_id = 0;
float flow_left_arr[MOVING_MEAN_COUNT];
float flow_right_arr[MOVING_MEAN_COUNT];
Mat previous_frame_left, previous_frame_right;

void scale_mat(const Mat matrix, Mat& matrix_left, Mat& matrix_right, const int width, const int height, const int width_img, const int height_img)
{
  // std::cout << "w"<< width_img<<" "<<width<<"h"<<height_img<<" "<<height<<"\n";
  matrix_left = matrix(Range((int) (height_img/2-height/2), (int) (height_img/2)), Range((int) (width_img/2-width/2), (int) (width_img/2+width/2)));
  matrix_right = matrix(Range((int) (height_img/2), (int) (height_img/2+height/2)), Range((int) (width_img/2-width/2), (int) (width_img/2+width/2)));
  
}
void calculate_magnitudes_flow(Mat& mag, Mat prvs, Mat next)
{
    Mat flow(prvs.size(), CV_32FC1);
    calcOpticalFlowFarneback(prvs, next, flow, 0.5, 3, 15, 3, 5, 1.2, 0);      
    Mat flow_parts[2];
    split(flow, flow_parts);
    magnitude(flow_parts[0], flow_parts[1], mag);
}

void farneback_playback(const std::string& filename, int width, int height)
{
      VideoCapture capture(filename);
    if (!capture.isOpened()){
        //error in opening the video input
        std::cerr << "Unable to open file!" << std::endl;
        return;
    }
    Mat frame1, prvs, prvs_left, prvs_right;
    
    capture >> frame1;
    cvtColor(frame1, prvs, COLOR_BGR2GRAY);
    int width_img = prvs.cols;
    int height_img = prvs.rows;
    scale_mat(prvs, prvs_left, prvs_right, width, height, width_img, height_img);
 
    float flow_left_arr[MOVING_MEAN_COUNT];
    float flow_right_arr[MOVING_MEAN_COUNT];
    for (int i=0; i<MOVING_MEAN_COUNT; i++)
    {
      flow_left_arr[i] = 0.0;
      flow_right_arr[i] = 0.0;
    }
    int frame_nr = 0;
    while(true)
    {
        Mat frame2, next, next_left, next_right;
        capture >> frame2;
        if (frame2.empty())
            break;
        cvtColor(frame2, next, COLOR_BGR2GRAY);
        scale_mat(next, next_left, next_right, width, height, width_img, height_img);
        Mat mag_left(prvs_left.size(), CV_32FC1);
        calculate_magnitudes_flow(mag_left, prvs_left, next_left);
        Mat mag_right(prvs_right.size(), CV_32FC1);
        calculate_magnitudes_flow(mag_right, prvs_right, next_right);
        float flow_right_sum = 0.0;
        float flow_left_sum = 0.0;
        
        for (int i=0; i<prvs_left.rows; i++)
        {
          for (int j=0; j<prvs_left.cols; j++)
          {
            flow_left_sum+=mag_left.at<float>(i,j);
            flow_right_sum+=mag_right.at<float>(i,j);
          }
        }
        flow_left_arr[frame_nr%MOVING_MEAN_COUNT] = flow_left_sum;
        flow_right_arr[frame_nr%MOVING_MEAN_COUNT] = flow_right_sum;
        
        float flow_left_sum_tot = 0.0;
        float flow_right_sum_tot = 0.0;
        for (int i=0; i<MOVING_MEAN_COUNT; i++)
        {
          flow_left_sum_tot += flow_left_arr[i];
          flow_right_sum_tot += flow_right_arr[i];
        }
        

        
        std::cout<<flow_left_sum_tot<<" "<<flow_right_sum_tot<<"\n";
        frame_nr++;
      
        float max_flow = std::max<float>(flow_left_sum_tot,flow_right_sum_tot);
        Mat viz (Size(width, height), CV_32FC1);
        for (int i=0; i<((int) viz.rows/2); i++)
        {
          for (int j=0; j<viz.cols; j++)
            viz.at<float>(i, j) = flow_left_sum_tot/max_flow;
        }
        for (int i=((int) viz.rows/2); i<viz.rows; i++)
        {
          for (int j=0; j<viz.cols; j++)
          {
            viz.at<float>(i, j) = flow_right_sum_tot/max_flow;
            }
        }
        std::cout<<viz.size()<<"\n";
        imshow("flows", viz);
        imshow("videofeed", next);

        int keyboard = waitKey(200);
        if (keyboard == 'q' || keyboard == 27)
            break;
        prvs = next;
        prvs_left = next_left;
        prvs_right = next_right;
    }
}


void farneback(char *img, float* output_flow, int width, int height, int width_img, int height_img)
{
    Mat next_frame(width_img, height_img, CV_8UC2, img); 
    // cvtColor(next_frame, next_frame, CV_YUV2RGB_Y422);
    // cvtColor(next_frame, next_frame, CV_RGB2GRAY);
    
    Mat next_frame_gray;
    cvtColor(next_frame, next_frame_gray, COLOR_BGR2GRAY);
    Mat next_frame_left, next_frame_right;
    scale_mat(next_frame_gray, next_frame_left, next_frame_right, width, height, width_img, height_img);
    
    if (frame_id==0)
    {
    for (int i=0; i<MOVING_MEAN_COUNT; i++)
    {
      flow_left_arr[i] = 0.0;
      flow_right_arr[i] = 0.0;
    }
      previous_frame_left = next_frame_left;
      previous_frame_right = next_frame_right;
      return;
    }  
    
    Mat mag_left(next_frame_left.size(), CV_32FC1);
    calculate_magnitudes_flow(mag_left, previous_frame_left, next_frame_left);
    
    Mat mag_right(next_frame_right.size(), CV_32FC1);
    calculate_magnitudes_flow(mag_right, previous_frame_right, next_frame_right);
    
    float flow_right_sum = 0.0;
    float flow_left_sum = 0.0;
      
      for (int i=0; i<mag_left.rows; i++)
      {
        for (int j=0; j<mag_left.cols; j++)
        {
          flow_left_sum+=mag_left.at<float>(i,j);
          flow_right_sum+=mag_right.at<float>(i,j);
        }
      }
      flow_left_arr[frame_id%MOVING_MEAN_COUNT] = flow_left_sum;
      flow_right_arr[frame_id%MOVING_MEAN_COUNT] = flow_right_sum;
      
      float flow_left_sum_tot = 0.0;
      float flow_right_sum_tot = 0.0;
      for (int i=0; i<MOVING_MEAN_COUNT; i++)
      {
        flow_left_sum_tot += flow_left_arr[i];
        flow_right_sum_tot += flow_right_arr[i];
      }
      
      frame_id++;
      output_flow[0] = flow_left_sum_tot;
      output_flow[1] = flow_right_sum_tot;

      previous_frame_left = next_frame_left;
      previous_frame_right = next_frame_right;
}

// int main()
// {
//   std::string filename = "/home/matthijs/paparazzi/videos/vlc-record-2023-03-10-09h59m06s-rtp_5000.sdp-.avi";
//   farneback_playback(filename, 200, 50);
//   return 0;
// }
