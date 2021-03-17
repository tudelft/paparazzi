//
// Created by anish on 12-03-21.
//

#include "cv_ground_detection.h"
#include "modules/computer_vision/cv.h"
#include "modules/computer_vision/opencv_ground_detection.h"

#ifndef VIDEO_CAPTURE_FPS
#define VIDEO_CAPTURE_FPS 0       ///< Default FPS (zero means run at camera fps)
#endif

PRINT_CONFIG_VAR(VIDEO_CAPTURE_FPS)


// Function
struct image_t *cv_ground_detection_func(struct image_t *img);


struct image_t *cv_ground_detection_func(struct image_t *img)
{

    if (img->type == IMAGE_YUV422) {
        // Call OpenCV (C++ from paparazzi C function)
        int go_no_go = cv_go_no_go((char *) img->buf, img->w, img->h); //Here we obtain a cropped black and white picture

    }


// opencv_example(NULL, 10,10);

    return img;
}

void cv_ground_detection_init(void)
{
    cv_add_to_device(&VIDEO_CAPTURE_CAMERA, cv_ground_detection_func, VIDEO_CAPTURE_FPS);

}
