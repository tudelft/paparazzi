//
// Created by anish on 17-03-21.
//

#include "ground_detection_testing.h"
#include "modules/computer_vision/cv.h"
#include "subsystems/abi.h"
#include "std.h"

#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include "pthread.h"

#ifndef FPS_WIDTH
#define FPS_WIDTH 0
#endif

struct image_t *width_function(struct image_t *img){

    printf("the width of the image is %d\n",img->w);

    return img;
}

void image_width_printer_init(void) {

#ifdef WIDTH_CAMERA
    cv_add_to_device(&WIDTH_CAMERA, width_function, FPS_WIDTH);
#endif
}