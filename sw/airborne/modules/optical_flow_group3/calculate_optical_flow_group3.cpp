#include <iostream>
#include <opencv2/opencv.hpp>

#define WIDTH 50
#define HEIGHT 200
#define MOVING_MEAN_COUNT 3

using namespace cv;

void scale_mat(const Mat matrix, Mat& matrix_left, Mat& matrix_right, const int width, const int height, const int width_img, const int height_img)
{
  // std::cout << "w"<< width_img<<" "<<width<<"h"<<height_img<<" "<<height<<"\n";
  matrix_left = matrix(Range((int) (height_img/2-height/2), (int) (height_img/2)), Range((int) (width_img/2-width/2), (int) (width_img/2+width/2)));
  matrix_right = matrix(Range((int) (height_img/2), (int) (height_img/2+height/2)), Range((int) (width_img/2-width/2), (int) (width_img/2+width/2)));
  
}
void calculate_magnitudes_flow(Mat& mag, Mat prvs, Mat next)
{
    Mat flow(prvs.size(), CV_32FC2);
    calcOpticalFlowFarneback(prvs, next, flow, 0.5, 3, 15, 3, 5, 1.2, 0);      
    Mat flow_parts[2];
    split(flow, flow_parts);
    magnitude(flow_parts[0], flow_parts[1], mag);
}

void farneback(const std::string& filename, bool to_gray, int width, int height)
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
    scale_mat(prvs, prvs_left, prvs_right, WIDTH, HEIGHT, width_img, height_img);
    std::cout<<prvs_left.size()<<"\n";
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
        scale_mat(next, next_left, next_right, WIDTH, HEIGHT, width_img, height_img);
        Mat mag_left(prvs_left.size(), CV_32FC2);
        calculate_magnitudes_flow(mag_left, prvs_left, next_left);
        Mat mag_right(prvs_right.size(), CV_32FC2);
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

        int keyboard = waitKey(1000);
        if (keyboard == 'q' || keyboard == 27)
            break;
        prvs = next;
    }
}
int main()
{
  std::string filename = "/home/matthijs/paparazzi/videos/vlc-record-2023-03-10-09h59m06s-rtp_5000.sdp-.avi";
  farneback(filename, true, 200, 50);
  return 0;
}