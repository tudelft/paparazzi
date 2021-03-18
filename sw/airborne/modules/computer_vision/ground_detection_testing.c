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

volatile int go_no_go;

//Remember image is rotated. Width is longer of the two dimensions
//TODO: Later add to XML so they can be adjusted as settings during testing
static double PERCENT_WIDTH_RECT_1 = 0.5; //rectangle width in percentage of image width
static double PERCENT_HEIGHT_RECT_1 = 0.25; //rectangle height in percentage of image height

volatile int cnt = 0;


int check_for_green(struct image_t *img, double width_percentage, double height_percentage) {
    //printf("Working\n");

    //pointer to buffer where image is stored
    uint8_t *buffer = img->buf;

    //Go trough the pixels in the rectangle
    //TODO: Instead of checking whole rectangle just check horizontal lines through rectangle
    /*TODO: Instead of rectangles move towards checking triangle consisting of horizontal lines that get smaller the
     * higher in the image*/
    for (uint16_t y = (1 - height_percentage) * 0.5 * img->h;y < (1 - ((1 - height_percentage) * 0.5)) *img->h; y++){
        //This now goes trough a certain percentage pf the image
        //for (uint16_t x = (1-percent_w)*0.5* img->w; x < (1-((1-percent_w)*0.5))* img->w; x ++)
        for (uint16_t x = 0 * img->w; x < width_percentage * img->w; x++) {
            // Check if the color is inside the specified values
            uint8_t *yp, *up, *vp;
            if (x % 2 == 0) {
                // Even x
                up = &buffer[y * 2 * img->w + 2 * x];      // U
                yp = &buffer[y * 2 * img->w + 2 * x + 1];  // Y1
                vp = &buffer[y * 2 * img->w + 2 * x + 2];  // V
                //yp = &buffer[y * 2 * img->w + 2 * x + 3]; // Y2
            } else {
                // Uneven x
                up = &buffer[y * 2 * img->w + 2 * x - 2];  // U
                //yp = &buffer[y * 2 * img->w + 2 * x - 1]; // Y1
                vp = &buffer[y * 2 * img->w + 2 * x];      // V
                yp = &buffer[y * 2 * img->w + 2 * x + 1];  // Y2
            }
            //TODO: make this an if not statement instead of a dummy statement
            if (!((*yp >= lum_min) && (*yp <= lum_max) &&
                (*up >= cb_min) && (*up <= cb_max) &&
                (*vp >= cr_min) && (*vp <= cr_max)) ){

                //printf("the pixel at row %d and column %d is NOT green!!!!!!!\n", y, x);
                return 0;
            }

            //printf("SAFE @ row %d and column %d\n",y,x);

            //tot_x += x;
            //tot_y += y;
            //if (draw){
            //*yp = 255;  // make pixel brighter in image
            //}
        }
        }
    /*else{

        printf("NOT SAFE @ row %d and column %d\n",y,x);
    }*/
    return 1;

}

struct image_t *check_rect(struct image_t *img){ //In this function we want to look at the amount of green pixels in a given rectangle

    go_no_go = check_for_green(img,PERCENT_WIDTH_RECT_1,PERCENT_HEIGHT_RECT_1);
    //printf("To go or not to go: %d \n",go_no_go);
    //TODO: send this go_no_go to navigation (ground_detector.c)
    //cnt = image_yuv422_colorfilt(img, img,color_lum_min, color_lum_max,color_cb_min, color_cb_max,color_cr_min, color_cr_max);

    return img;
}

void image_width_printer_init(void) {

#ifdef WIDTH_CAMERA
    //printf("There is a camera");
    cv_add_to_device(&WIDTH_CAMERA, check_rect, FPS_WIDTH);
    //check_for_green(cv_add_to_device(&WIDTH_CAMERA, check_rect, FPS_WIDTH),PERCENT_WIDTH_RECT_1,PERCENT_HEIGHT_RECT_1);
    //printf("To go or not to go: %d \n",go_no_go);
#endif
}