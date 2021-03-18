//
// Created by anish on 17-03-21.
//

#include "ground_detection_testing.h"
#include "modules/computer_vision/cv.h"
#include "modules/computer_vision/lib/vision/image.h"
#include "subsystems/abi.h"
#include "std.h"

#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include "pthread.h"

#ifndef FPS_WIDTH
#define FPS_WIDTH 0
#endif

static uint8_t lum_min = 0;
static uint8_t lum_max = 255;
static uint8_t cb_min  = 0;
static uint8_t cb_max  = 110;
static uint8_t cr_min  = 0;
static uint8_t cr_max  = 130;

static double percent_w = 0.50; //rectangle width in percentage of image
static double percent_h = 0.25;

volatile int cnt = 0;




struct image_t *check_rect(struct image_t *img){ //In this function we want to look at the amount of green pixels in a given rectangle

    printf("Working");
    uint8_t *buffer = img->buf;

    //Go trough the pixels in the rectangle
    for (uint16_t y = (1-percent_h)*0.5 * img->h; y < (1-((1-percent_h)*0.5))* img->h; y++)//This now goes trough a certain percentage pf the image {
        //for (uint16_t x = (1-percent_w)*0.5* img->w; x < (1-((1-percent_w)*0.5))* img->w; x ++)
          for (uint16_t x = 0 * img->w; x < percent_w * img->w; x ++) {
            // Check if the color is inside the specified values
            uint8_t *yp, *up, *vp;
            if (x % 2 == 0) {
                // Even x
                up = &buffer[y * 2 * img->w + 2 * x];      // U
                yp = &buffer[y * 2 * img->w + 2 * x + 1];  // Y1
                vp = &buffer[y * 2 * img->w + 2 * x + 2];  // V
                //yp = &buffer[y * 2 * img->w + 2 * x + 3]; // Y2
            }
            else {
                // Uneven x
                up = &buffer[y * 2 * img->w + 2 * x - 2];  // U
                //yp = &buffer[y * 2 * img->w + 2 * x - 1]; // Y1
                vp = &buffer[y * 2 * img->w + 2 * x];      // V
                yp = &buffer[y * 2 * img->w + 2 * x + 1];  // Y2
            }
            if ( (*yp >= lum_min) && (*yp <= lum_max) &&
                 (*up >= cb_min ) && (*up <= cb_max ) &&
                 (*vp >= cr_min ) && (*vp <= cr_max )) {

                //printf("SAFE @ row %d and column %d\n",y,x);

                //tot_x += x;
                //tot_y += y;
                //if (draw){
                    //*yp = 255;  // make pixel brighter in image
                //}
            }
            else{
                printf("NO GO");
                //printf("NOT SAFE @ row %d and column %d\n",y,x);
        }
    }

    //cnt = image_yuv422_colorfilt(img, img,color_lum_min, color_lum_max,color_cb_min, color_cb_max,color_cr_min, color_cr_max);

    return img;
}

void image_width_printer_init(void) {

#ifdef WIDTH_CAMERA
    cv_add_to_device(&WIDTH_CAMERA, check_rect, FPS_WIDTH);
#endif
}