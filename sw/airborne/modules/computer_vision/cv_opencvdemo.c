/*
 * Copyright (C) C. De Wagter
 *
 * This file is part of paparazzi
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/computer_vision/cv_opencvdemo.c"
 * @author C. De Wagter
 * A simple module showing what you can do with opencv on the bebop.
 */

#include "modules/computer_vision/cv.h"
#include "modules/computer_vision/cv_opencvdemo.h"
#include "modules/computer_vision/opencv_example.h"
#include "modules/computer_vision/finite_state_machine.h"


#include "modules/orange_avoider/orange_avoider.h"
#include "firmwares/rotorcraft/navigation.h"
#include "generated/airframe.h"
#include "state.h"
#include "modules/core/abi.h"
#include <time.h>
#include <stdio.h>
#include "math.h"

#define NAV_C // needed to get the nav functions like Inside...
#include "generated/flight_plan.h"

#define ORANGE_AVOIDER_VERBOSE TRUE

#define PRINT(string,...) fprintf(stderr, "[orange_avoider->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if ORANGE_AVOIDER_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif





////////////////////////////////////////////////////
////////////PARAMETERS_TO_TEST_ON///////////////////
////////////////////////////////////////////////////
#ifndef OPENCVDEMO_FPS
#define OPENCVDEMO_FPS 0       ///< Default FPS (zero means run at camera fps)
#endif
#define WIDTH_2_PROCESS 30   // TWEAKABLE
#define HEIGHT_2_PROCESS 140 // TWEAKABLE
// #define FLOWMIDDLE_DIV_LOWER 0.80f
// #define FLOWMIDDLE_ABS_MIN 0.0f // Not too sure whether using this is a smart move, otherwise set to 0
// #define LEFTFLOW_TURNING_TH 1.22f
// #define RIGHTFLOW_TURNING_TH 0.75f
// #define COUNTER2_MAX 9
// #define HEADING_INC_LEFT_RIGHT 45.f
// #define HEADING_INC_MIDDLE 60.f
// #define HEADING_INC_OUT_OF_BOUND 90.f

float movedistance = 1.2f;     // TWEAKABLE (changes spead)

float right_obstacle_threshold = 0.78f;
float flowmiddle_obstacle_threshold = 1.3f;                          // TWEAKABLE
float left_obstacle_threshold= 1.28f;                                 // TWEAKABLE



float x_prev;
float y_prev;
float tot_dist = 0.f;


// // ::google::protobuf::internal::GetCurrentTime(&seconds, &nanoseconds);
// // int64_t seconds;
// // int32_t nanoseconds;
#define LOG(x) fprintf(stderr, "LOG: %s:%d %s %lu \n", __FILE__, __LINE__, x, clock());
// // #define LOG(x)


///////////////////////////////////////////////////////////////////


// static uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters);
// static uint8_t calculateForwards(struct EnuCoor_i *new_coor, float distanceMeters);
// static uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor);
// static uint8_t increase_nav_heading(float incrementDegrees);

float oa_color_count_frac = 0.18f; // ***HAVE TO DEFINE THIS HERE TO GET IT TO COMPILE

// Define navigation states



float right_left_normalizer = 1.0f;                                  // TWEAKABLE
// float right_obstacle_threshold = 1.0f // TWEAKABLE



float flowleft_temp = 0.0f;
float flowright_temp = 0.0f;

int counter = 0;
int counter2 = 0;
int step = 0;
float output_flow[3];



void image_editing(struct image_t *img, float right_left_normaliser, float flowmiddle_divergence) {
//    Mat image(width_img, height_img, CV_8UC2, img);
//    Mat matrix_left = image(Range(95,145), Range(160,260));
//    Mat matrix_right = image(Range(95,145),Range(260,360));

//    farneback((char *) img->buf, output_flow, WIDTH_2_PROCESS, HEIGHT_2_PROCESS, img->w, img->h);
    uint8_t *buffer = img->buf;
    auto width_img = img->w;
    auto height_img = img->h;
    auto width = WIDTH_2_PROCESS;
    auto height = HEIGHT_2_PROCESS;
    int row_start = width_img/2 - width/2; int row_end = width_img/2 + width/2;
    int left_start = height_img/2 - height/2; int left_end = height_img/2;
    int middle_start = height_img/2 - height/4; int middle_end = height_img/2 + height/4 ;
    int right_start = height_img/2; int right_end = height_img/2 + height/2;
    int chosen_start; int chosen_end;
    if(right_left_normalizer < right_obstacle_threshold){
        chosen_start = right_start;
        chosen_end = right_end;
    }
    else if(right_left_normalizer > left_obstacle_threshold){
        chosen_start = left_start;
        chosen_end = left_end;

    }
    else if(flowmiddle_divergence > flowmiddle_obstacle_threshold){
        chosen_start = middle_start;
        chosen_end = middle_end;

    }

    // L_R = 0 means obstacle left
    for (uint16_t y = chosen_start; y < chosen_end; y++) {
        for (uint16_t x = row_start; x < row_end; x++) {
            uint8_t *yp, *up, *vp;
            if (x % 4 == 0) {
                // Even x
                up = &buffer[y * 2 * img->w + 2 * x];      // U
                yp = &buffer[y * 2 * img->w + 2 * x + 1];  // Y1
                vp = &buffer[y * 2 * img->w + 2 * x + 2];  // V
                //yp = &buffer[y * 2 * img->w + 2 * x + 3]; // Y2
                *yp = 0;
                *up = 0;
                *vp = 0;
            } else {
                // Uneven x
                up = &buffer[y * 2 * img->w + 2 * x - 2];  // U
                //yp = &buffer[y * 2 * img->w + 2 * x - 1]; // Y1
                vp = &buffer[y * 2 * img->w + 2 * x];      // V
                yp = &buffer[y * 2 * img->w + 2 * x + 1];  // Y2
//                *yp = 0;
//                *up = 0;
//                *vp = 0;
            }
        }
    }


}

void image_cover(struct image_t *img){
    uint8_t *buffer = img->buf;
    float square_1[4] = {0,img->h/2 - HEIGHT_2_PROCESS/2, 0, img->w}; //[0,160,0,img->w];
    float square_2[4] = {img->h/2 + HEIGHT_2_PROCESS/2,img->h,0,img->w};// [360,img->h,0,img->w];
    float square_3[4] = {img->h/2 - HEIGHT_2_PROCESS/2,img->h/2 + HEIGHT_2_PROCESS/2,0,img->w/2 - WIDTH_2_PROCESS/2}; //[160,360,0,95];
    float square_4[4] = {img->h/2 - HEIGHT_2_PROCESS/2,img->h/2 + HEIGHT_2_PROCESS/2,img->w/2 + WIDTH_2_PROCESS/2,img->w}; // [160,360,145,img->w];
    float* squares[4] = {square_1,square_2,square_3,square_4};
    for (int chosen = 0;chosen<4;chosen++){
        float* ptr = squares[chosen];
        for (uint16_t y = ptr[0]; y < ptr[1]; y++) {
            for (uint16_t x = ptr[2]; x < ptr[3]; x++) {
                uint8_t *yp, *up, *vp;
                if (x % 2 == 0) {
                    // Even x
                    up = &buffer[y * 2 * img->w + 2 * x];      // U
                    yp = &buffer[y * 2 * img->w + 2 * x + 1];  // Y1
                    vp = &buffer[y * 2 * img->w + 2 * x + 2];  // V
                    //yp = &buffer[y * 2 * img->w + 2 * x + 3]; // Y2
                    *yp = 255;
                    *up = 255;
                    *vp = 255;
                } else {
                    // Uneven x
                    up = &buffer[y * 2 * img->w + 2 * x - 2];  // U
                    //yp = &buffer[y * 2 * img->w + 2 * x - 1]; // Y1
                    vp = &buffer[y * 2 * img->w + 2 * x];      // V
                    yp = &buffer[y * 2 * img->w + 2 * x + 1];  // Y2
                    *yp = 255;
                    *up = 255;
                    *vp = 255;
                }
            }
        }
    }
}

struct image_t *optical_flow_func(struct image_t *img, int camera_id);
struct image_t *optical_flow_func(struct image_t *img, int camera_id)
{
    if (img->type == IMAGE_YUV422) {
        farneback((char *) img->buf, output_flow, WIDTH_2_PROCESS, HEIGHT_2_PROCESS, img->w, img->h);
    }

    run_fsm(output_flow[0], output_flow[1], output_flow[2]);

    return img;
}

void calc_action_optical_flow_init(void)
{
    cv_add_to_device(&OPENCVDEMO_CAMERA, optical_flow_func, OPENCVDEMO_FPS, 0);
    output_flow[0] = 0.0;
    output_flow[1] = 0.0;
}


void compute_dist(int i)
{
  if (i==0)
  {
    x_prev = POS_FLOAT_OF_BFP(stateGetPositionEnu_i()->x);
    y_prev = POS_FLOAT_OF_BFP(stateGetPositionEnu_i()->y);
    return;
  }
  printf("x_prev %f\n", x_prev);
  tot_dist += sqrt((POS_FLOAT_OF_BFP(stateGetPositionEnu_i()->x) - x_prev) * (POS_FLOAT_OF_BFP(stateGetPositionEnu_i()->x) - x_prev) + (POS_FLOAT_OF_BFP(stateGetPositionEnu_i()->y) - y_prev) * (POS_FLOAT_OF_BFP(stateGetPositionEnu_i()->y) - y_prev));
  printf("tot_dist: %lf\n", tot_dist);
  x_prev = stateGetPositionEnu_i()->x;
  y_prev = stateGetPositionEnu_i()->y;
}

