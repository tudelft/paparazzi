//
// Created by anish on 12-03-21.
//

#include "opencv_ground_detection.h"
#include <stdio.h>

#include "modules/computer_vision/lib/vision/image.h"

#ifndef VIDEO_CAPTURE_FPS
#define VIDEO_CAPTURE_FPS 0       ///< Default FPS (zero means run at camera fps)
#endif
PRINT_CONFIG_VAR(COLORFILTER_FPS)



using namespace std;
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
using namespace cv;
#include "opencv_image_functions.h"

const int max_value = 255;
const int low_H = 35, low_S = 50, low_V = 50;
const int high_H = 40, high_S = max_value, high_V = max_value;
const int cropped_rect_x = 25,cropped_rect_y = 0,cropped_rect_w = 15,cropped_rect_h = 50; //The first 2 variables state where the southwest edge of the rectangle goes, and the other specify the width and height


int cv_go_no_go(char *img, int height, int width){

    Mat M(height, width, CV_8UC2, img); //Get the image from the front camera
    Mat image_BGR,image_HSV, filter_image; //Container for an image

    resize(M,M,Size(50,50)); //Here we are downscaling the image
    cvtColor(M, image_BGR, CV_YUV2BGR_Y422);
    cvtColor(image_BGR, image_HSV, COLOR_BGR2HSV); //Convert the image to HSV colorspace to have the filter work
    inRange(image_HSV, Scalar(low_H, low_S, low_V), Scalar(high_H, high_S, high_V), filter_image); //This gives a black and white image to detect contours

    Rect cropped_region(cropped_rect_x,cropped_rect_y,cropped_rect_w,cropped_rect_h);
    Mat cropped = filter_image(cropped_region); //With this Mat object we want to do our calculations


    for(int row = 49; row >= 40; row--) {

        for (int p_x = 0; p_x <= cropped.cols-1; p_x++) {
            Scalar intensity = cropped.at<uchar>(row, p_x);

            if (intensity.val[0] != 255){
                return 0;
            }
        }
        }
    return 1;
    }








