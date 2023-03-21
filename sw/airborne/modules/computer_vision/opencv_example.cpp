#include "modules/computer_vision/cv.h"
#include "modules/computer_vision/cv_opencvdemo.h"
#include "modules/computer_vision/opencv_example.h"


#include "modules/orange_avoider/orange_avoider.h"
#include "firmwares/rotorcraft/navigation.h"
#include "generated/airframe.h"
#include "state.h"
#include "modules/core/abi.h"
#include <time.h>
#include <stdio.h>

#define NAV_C // needed to get the nav functions like Inside...

#include "generated/flight_plan.h"

#define ORANGE_AVOIDER_VERBOSE TRUE

#define PRINT(string,...) fprintf(stderr, "[orange_avoider->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if ORANGE_AVOIDER_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

#ifndef OPENCVDEMO_FPS
#define OPENCVDEMO_FPS 0       ///< Default FPS (zero means run at camera fps)
#endif


// static uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters);
// static uint8_t calculateForwards(struct EnuCoor_i *new_coor, float distanceMeters);
// static uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor);
// static uint8_t increase_nav_heading(float incrementDegrees);






///////////////////////////////////////////////////////////////////////////////////////

// #include "opencv_example.h"



// using namespace std;
// #include <opencv2/core/core.hpp>
// #include <opencv2/imgproc/imgproc.hpp>

// using namespace cv;
// #include "opencv_image_functions.h"
// #include "modules/computer_vision/cv.h"


// int opencv_example(char *img, int width, int height)
// {
//   // Create a new image, using the original bebop image.
//   struct image_t *a;
  
//   Mat M(height, width, CV_8UC2, img);
//   Mat image;

// #if OPENCVDEMO_GRAYSCALE
//   //  Grayscale image example
//   cvtColor(M, image, CV_YUV2GRAY_Y422);
//   // Canny edges, only works with grayscale image
//   int edgeThresh = 35;
//   Canny(image, image, edgeThresh, edgeThresh * 3);
//   // Convert back to YUV422, and put it in place of the original image
//   grayscale_opencv_to_yuv422(image, img, width, height);
// #else // OPENCVDEMO_GRAYSCALE
//   // Color image example
//   // Convert the image to an OpenCV Mat
//   cvtColor(M, image, CV_YUV2BGR_Y422);
//   // Blur it, because we can
//   blur(image, image, Size(5, 5));
//   // Convert back to YUV422 and put it in place of the original image
//   colorbgr_opencv_to_yuv422(image, img, width, height);
// #endif // OPENCVDEMO_GRAYSCALE

//   return 0;
// }
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
// #include "opencv2/core/hal/intrin.hpp"
// #include <opencv2/highgui/highgui.hpp>
// #include "opencv2\highgui.hpp"
#include "opencv_example.h"
#include "opencv2/video/tracking.hpp"
#include <opencv2/core/types.hpp>

#define MOVING_MEAN_COUNT 3

using namespace cv;

int frame_id = 0;
float flow_left_arr[MOVING_MEAN_COUNT];
float flow_right_arr[MOVING_MEAN_COUNT];
Mat previous_frame_left, previous_frame_right;

void scale_mat(const Mat matrix, Mat& matrix_left, Mat& matrix_right, const int width, const int height, const int width_img, const int height_img)
{
  std::cout << "w"<< width_img<<" "<<width<<"h"<<height_img<<" "<<height<<"\n";

  std::cout << "1: "<<(int) (height_img/2-height/2)<<", 2: "<< (int) (height_img/2)<<", 3: "<<(int) (width_img/2-width/2)<<", 4: "<<(int) (width_img/2+width/2)<<"\n";
  matrix_left = matrix(Range(95,145), Range(160,260));//(Range((int) (height_img/2-height/2), (int) (height_img/2)), Range((int) (width_img/2-width/2), (int) (width_img/2+width/2)));

  matrix_right =matrix(Range(95,145),Range(260,360)); //matrix(Range((int) (height_img/2), (int) (height_img/2+height/2)), Range((int) (width_img/2-width/2), (int) (width_img/2+width/2)));
  
}
void calculate_magnitudes_flow(Mat& mag, Mat prvs, Mat next)
{
    Mat flow(prvs.size(), CV_32FC1);
    calcOpticalFlowFarneback(prvs, next, flow, 0.5, 3, 15, 3, 5, 1.2, 0);      
    Mat flow_parts[2];
    split(flow, flow_parts);
    magnitude(flow_parts[0], flow_parts[1], mag);
}



void farneback(char *img, float* output_flow, int width, int height, int width_img, int height_img)
{
    std::cout<<"farneback"<<"\n";
    Mat next_frame(width_img, height_img, CV_8UC2, img); 
    // cvtColor(next_frame, next_frame, CV_YUV2RGB_Y422);
    // cvtColor(next_frame, next_frame, CV_RGB2GRAY);
    
    Mat next_frame_gray;

    cvtColor(next_frame, next_frame_gray, CV_YUV2GRAY_Y422);

    Mat next_frame_left, next_frame_right;

    next_frame_left = next_frame_gray;
    next_frame_right = next_frame_gray;
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
      frame_id++;
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
      std::cout<<"left: "<<output_flow[0]<<", right: "<<output_flow[1]<<"\n";

      // increase_nav_heading(6.f);
     // moveWaypointForward(WP_TRAJECTORY, 0.8f);
     
}





// int main()
// {
//   std::string filename = "/home/matthijs/paparazzi/videos/vlc-record-2023-03-10-09h59m06s-rtp_5000.sdp-.avi";
//   farneback_playback(filename, 200, 50);
//   return 0;
// }




//////////////////////////////////////////////


// /*
//  * Calculates coordinates of distance forward and sets waypoint 'waypoint' to those coordinates
//  */
// uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters)
// {
//   struct EnuCoor_i new_coor;
//   calculateForwards(&new_coor, distanceMeters);
//   moveWaypoint(waypoint, &new_coor);
//   return false;
// }

// /*
//  * Calculates coordinates of a distance of 'distanceMeters' forward w.r.t. current position and heading
//  */
// uint8_t calculateForwards(struct EnuCoor_i *new_coor, float distanceMeters)
// {
//   float heading  = stateGetNedToBodyEulers_f()->psi;

//   // Now determine where to place the waypoint you want to go to
//   new_coor->x = stateGetPositionEnu_i()->x + POS_BFP_OF_REAL(sinf(heading) * (distanceMeters));
//   new_coor->y = stateGetPositionEnu_i()->y + POS_BFP_OF_REAL(cosf(heading) * (distanceMeters));
//   VERBOSE_PRINT("Calculated %f m forward position. x: %f  y: %f based on pos(%f, %f) and heading(%f)\n", distanceMeters,	
//                 POS_FLOAT_OF_BFP(new_coor->x), POS_FLOAT_OF_BFP(new_coor->y),
//                 stateGetPositionEnu_f()->x, stateGetPositionEnu_f()->y, DegOfRad(heading));
//   return false;
// }


// /*
//  * Sets waypoint 'waypoint' to the coordinates of 'new_coor'
//  */
// uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor)
// {
//   VERBOSE_PRINT("Moving waypoint %d to x:%f y:%f\n", waypoint, POS_FLOAT_OF_BFP(new_coor->x),
//                 POS_FLOAT_OF_BFP(new_coor->y));
//   waypoint_move_xy_i(waypoint, new_coor->x, new_coor->y);
//   return false;
// }

// // }
// //  if(!autopilot_in_flight()){
// // //    return;
// // //  }
