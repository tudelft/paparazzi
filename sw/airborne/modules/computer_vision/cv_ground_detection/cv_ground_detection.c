//
// Created by anish on 12-03-21.
//

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "opencv_image_functions.h"
#include "cv_ground_detection.h"
#include "modules/computer_vision/cv.h"
#include "modules/computer_vision/opencv_ground_detection.h"

#ifndef OPENCV_FPS
#define OPENCV_FPS 0       ///< Default FPS (zero means run at camera fps)
#endif
PRINT_CONFIG_VAR(OPENCV_FPS)

int go_no_go = 0;

// Function
struct image_t *cv_ground_detection_func(struct image_t *img);

struct image_t *cv_ground_detection_func(struct image_t *img)
{

    if (img->type == IMAGE_YUV422) {
        // Call OpenCV (C++ from paparazzi C function)
        cv_go_no_go((char *) img->buf, img->w, img->h); //Here we obtain a cropped black and white picture

    }

// opencv_example(NULL, 10,10);

    return NULL;
}

void cv_ground_detection_init(void)
{
    cv_add_to_device(&OPENCVDEMO_CAMERA, cv_ground_detection_func, OPENCVDEMO_FPS);
    printf("Hello this works");
}
