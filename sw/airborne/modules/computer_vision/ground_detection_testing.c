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

//TODO: make either boolean or integer with less bits
volatile int go_no_go;

//Remember image is rotated. Width is longer of the two dimensions
//TODO: Later add to XML so they can be adjusted as settings during testing
//Length of the rectangle most to the left as percentage of image length (height)
static double BOTTOM_LENGTH_PERCENTAGE = 0.5;
//Length of the rectangle most to the right as percentage of image length (height)
static double TOP_LENGTH_PERCENTAGE = 0.25;
//How far to the left the rectangles extend as a percentage of image width
static double TOP_WIDTH_PERCENTAGE = 0.75;
//#width in pixels of each rectangle
static int WIDTH_RECT = 5;


int check_for_green(struct image_t *img, int right_corner_row, int right_corner_column, int rect_length) {
    //printf("Working\n");

    //pointer to buffer where image is stored
    uint8_t *buffer = img->buf;

    //Go trough the pixels in the rectangle
    //TODO: Instead of checking whole rectangle just check horizontal lines through rectangle
    /*TODO: Instead of rectangles move towards checking triangle consisting of horizontal lines that get smaller the
     * higher in the image*/
    for (uint16_t y = right_corner_row;y <= right_corner_row + rect_length ; y++){
        //This now goes trough a certain percentage pf the image
        //for (uint16_t x = (1-percent_w)*0.5* img->w; x < (1-((1-percent_w)*0.5))* img->w; x ++)
        for (uint16_t x = right_corner_column ; x <= right_corner_column + WIDTH_RECT; x++) {
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

struct image_t *get_rect(struct image_t *img){ //In this function we want to look at the amount of green pixels in a given rectangle
    printf("Working \n");

    int rows = img->h;
    int columns = img->w;

    //The number of rectangles which are considered. It is set so that the space
    //up to TOP_WIDTH_PERCENTAGE is filled with non-overlapping rectangles
    int num_rect = ceil(TOP_WIDTH_PERCENTAGE*columns/WIDTH_RECT);

    //The difference in length between 2 adjacent rectangles in percentage of image length (height)
    double rect_length_increment = (BOTTOM_LENGTH_PERCENTAGE-TOP_LENGTH_PERCENTAGE)/num_rect;

    //Go through rectangles starting from smallest (most to the right)
    for (int rect_num = 0; rect_num < num_rect; rect_num++) {
        //The length of the rectangle for this iteration pixels
        int rect_length = floor((BOTTOM_LENGTH_PERCENTAGE - rect_num * rect_length_increment)*rows);
        //Coordinates of right corner
        int right_corner_row = TOP_WIDTH_PERCENTAGE*columns - rect_num*WIDTH_RECT;
        int right_corner_column = 0.5*(rows-rect_length);

        //Check if this rectangle is completely green and if so we are good to go straight ahead
        if (check_for_green(img, right_corner_row, right_corner_column, rect_length) == 1) {
            printf("Rectangle at (%d,%d) of length (%d) is a go \n", right_corner_column, right_corner_row, rect_length);
            go_no_go = 1;
            return img;
        }
    }

    go_no_go = 0;
    return img;

    //go_no_go = check_for_green(img,PERCENT_WIDTH_RECT_1,PERCENT_HEIGHT_RECT_1);
    //printf("To go or not to go: %d \n",go_no_go);


    return img;
}

void image_width_printer_init(void) {

#ifdef WIDTH_CAMERA
    cv_add_to_device(&WIDTH_CAMERA, get_rect, FPS_WIDTH);
#endif
}