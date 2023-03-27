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

#define WIDTH_2_PROCESS 40
#define HEIGHT_2_PROCESS 190

// ::google::protobuf::internal::GetCurrentTime(&seconds, &nanoseconds);
// int64_t seconds;
// int32_t nanoseconds;
//#define LOG(x) fprintf(stderr, "LOG: %s:%d %s %lu \n", __FILE__, __LINE__, x, clock());
#define LOG(x)


///////////////////////////////////////////////////////////////////


static uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters);
static uint8_t calculateForwards(struct EnuCoor_i *new_coor, float distanceMeters);
static uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor);
static uint8_t increase_nav_heading(float incrementDegrees);

float oa_color_count_frac = 0.18f; // ***HAVE TO DEFINE THIS HERE TO GET IT TO COMPILE

// Define navigation states
enum navigation_state_t {
  SAFE,
  IJUSTTURNED1,
  IJUSTTURNED2,
  OBSTACLE_LEFT,
  OBSTACLE_RIGHT,
  OBSTACLE_MIDDLE,
  OUT_OF_BOUNDS,
};

enum navigation_state_t navigation_state = SAFE;
float flowleft = 0.0f;
float flowright = 0.0f;
float flowmiddle = 1.0f;
float flowmiddle_prev = 1.0f;
float flowmiddle_divergence = 0.0f;

float flowleft_threshold = 5.0f;
float flowright_threshold = 5.0f;
float flowmiddle_threshold = 5.0f;

float right_left_normalizer = 1.0f;

float flowleft_temp = 0.0f;
float flowright_temp = 0.0f;

// float flowcombined_treshold = 10.0f;
// if flowcombined > flowcombined_treshold, then turn 180 degrees (run away)

float heading_increment = 7.f; 
float movedistance = 1.0f;
float output_flow[3];

void image_editing(struct image_t *img, float flow_left, float flow_right, float flow_middle) {
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
//    auto range_width = Range((int) (width_img/2 - width/2),(int) (width_img/2 + width/2));
//    auto left_end = Range((int) (height_img/2 - height/2),(int) (height_img/2));
//    auto middle_end = Range((int) (height_img/2 - height/4),(int) (height_img/2 + height/4));
//    auto right_end = Range((int) (height_img/2),(int) (height_img/2 + height/2));

    if (flow_left>flow_right && flow_left> flow_middle && flow_left>= flowleft_threshold){
        chosen_start = left_start;
        chosen_end = left_end;
        printf("right left normaliser %f. flowmiddle divergence %f ", right_left_normalizer, flowmiddle_divergence);
        printf("\n");

    }
    else if (flow_right> flow_left && flow_left > flow_middle && flow_right >= flowright_threshold){
        chosen_start=right_start;
        chosen_end = right_end;
        printf("right left normaliser %f. flowmiddle divergence %f ", right_left_normalizer, flowmiddle_divergence);
        printf("\n");
    }
    else if (flow_middle > flow_right && flow_middle > flow_left && flow_middle>=flowmiddle_threshold){
        chosen_start = middle_start;
        chosen_end = middle_end;
        printf("right left normaliser %f. flowmiddle divergence %f ", right_left_normalizer, flowmiddle_divergence);
        printf("\n");
    }
//    else{
//        printf("No flow above threshold \n");
//    }


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
    float square_1[4] = {0,165, 0, img->w}; //[0,160,0,img->w];
    float square_2[4] = {355,img->h,0,img->w};// [360,img->h,0,img->w];
    float square_3[4] = {165,355,0,100}; //[160,360,0,95];
    float square_4[4] = {165,355,140,img->w}; // [160,360,145,img->w];
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
    // action act = STANDBY;
    LOG("before farneback")
    if (img->type == IMAGE_YUV422) {
        farneback((char *) img->buf, output_flow, WIDTH_2_PROCESS, HEIGHT_2_PROCESS, img->w, img->h);
       
    }
    LOG("after farneback") 
    //increase_nav_heading(6.f);
    //moveWaypointForward(WP_TRAJECTORY, movedistance);
    //moveWaypointForward(WP_GOAL, 0.5);
    //
    flowmiddle_prev = flowmiddle;
    flowleft = output_flow[0];
    flowright = output_flow[1];
    flowmiddle = output_flow[2];

    
    //** EXTREMELY QUICK NO NEED TO OPTIMIZE
    right_left_normalizer = flowleft / flowright; // ADDED THIS, ABSULUTE VALUES DONT SEEM TO WORK SO WELL
    flowmiddle_divergence = (flowmiddle / flowmiddle_prev);
    //**
    if (right_left_normalizer < 0.78|| right_left_normalizer > 1.3|| flowmiddle_divergence > 1.3){
        image_editing(img,flowleft,flowright,flowmiddle);
    }
    else{
        printf("No flow above threshold \n");
    }

    image_cover(img);



    switch (navigation_state){
      case SAFE:
        // Move waypoint forward
        moveWaypointForward(WP_TRAJECTORY, 0.5f);

        LOG("start of safe")
        

     
        if (!InsideObstacleZone(WaypointX(WP_TRAJECTORY),WaypointY(WP_TRAJECTORY))){
          navigation_state = OUT_OF_BOUNDS;
          LOG("Before obstacle right evaluation")
        } else if (right_left_normalizer < 0.78){ // added this
          LOG("after obstacle right evaluation")
          navigation_state = OBSTACLE_RIGHT;
        } else if (right_left_normalizer > 1.3){ // added this
          navigation_state = OBSTACLE_LEFT;
        } 
          else if (flowmiddle_divergence > 1.3){ 
          navigation_state = OBSTACLE_MIDDLE;
        } else {
          moveWaypointForward(WP_GOAL, 0.5f);
        }
        LOG("end of safe")

        break;

      case IJUSTTURNED1:
      LOG("before IJUSTTURNED1")
      moveWaypointForward(WP_TRAJECTORY, movedistance);
      moveWaypointForward(WP_GOAL, movedistance);


      LOG("after IJUSTTURNED1")
      navigation_state = IJUSTTURNED2;
      break;

      case IJUSTTURNED2:
      LOG("before IJUSTTURNED2")
      moveWaypointForward(WP_TRAJECTORY, movedistance);
      moveWaypointForward(WP_GOAL, movedistance);
      LOG("after IJUSTTURNED2")

      navigation_state = SAFE; 
      break;


      case OBSTACLE_LEFT:
        LOG("Before left obstacle")
      // stop
        waypoint_move_here_2d(WP_GOAL);
        waypoint_move_here_2d(WP_TRAJECTORY);

        // CUSTOM CODE
        LOG("BEFORE HEADING INCREASE")
        increase_nav_heading(30.f); // SHOULD BE TWEAKED

        LOG("AFTER HEADING INCREASE")
        printf("Turned Right");
        moveWaypointForward(WP_TRAJECTORY, movedistance);
        
        // right_left_normalizer = 1.0f; // THIS SHOULDNT BE NECESSARY BUT I DUNNO
        // navigation_state = SAFE;
        LOG("After left obstacle")
        navigation_state = IJUSTTURNED1;
        
        break;

      case OBSTACLE_RIGHT:
        LOG("Before right obstacle")
        // stop
        waypoint_move_here_2d(WP_GOAL);
        waypoint_move_here_2d(WP_TRAJECTORY);

        
        increase_nav_heading(-30.f); // SHOULD BE TWEAKED
        


        moveWaypointForward(WP_TRAJECTORY, movedistance);
        // right_left_normalizer = 1.0f;
        // navigation_state = SAFE;
        LOG("After right obstacle")
        navigation_state = IJUSTTURNED1;
        
        break;

    case OBSTACLE_MIDDLE: 
      LOG("Before middle obstacle")
      // stop
      waypoint_move_here_2d(WP_GOAL);
      waypoint_move_here_2d(WP_TRAJECTORY);

      increase_nav_heading(30.f);

      
      
      
      moveWaypointForward(WP_TRAJECTORY, movedistance);
      // navigation_state = SAFE;

      LOG("After middle obstacle")
      navigation_state = IJUSTTURNED1; 

      
      break;
    case OUT_OF_BOUNDS:
      LOG("Before out of bounds")
      
      increase_nav_heading(heading_increment);
      moveWaypointForward(WP_TRAJECTORY, 1.5f);

      if (InsideObstacleZone(WaypointX(WP_TRAJECTORY),WaypointY(WP_TRAJECTORY))){
        // add offset to head back into arena
        increase_nav_heading(heading_increment);
        LOG("After out of bounds")
        navigation_state = SAFE;
        
      }
      break;
    default:
      break;
  }

     return img;
}

void calc_action_optical_flow_init(void)
{
    cv_add_to_device(&OPENCVDEMO_CAMERA, optical_flow_func, OPENCVDEMO_FPS, 0);
    output_flow[0] = 0.0;
    output_flow[1] = 0.0;
}






// moveWaypointForward(WP_TRAJECTORY, movedistance);

void calc_action_optical_flow_periodic(void)
{
//    printf("out: %f",output_flow[0]);

}
// define variables
////////////////////////////////////////////////////////////////////////////////////////////

/*
 * Increases the NAV heading. Assumes heading is an INT32_ANGLE. It is bound in this function.
 */
uint8_t increase_nav_heading(float incrementDegrees)
{
  float new_heading = stateGetNedToBodyEulers_f()->psi + RadOfDeg(incrementDegrees);

  // normalize heading to [-pi, pi]
  FLOAT_ANGLE_NORMALIZE(new_heading);

  // set heading, declared in firmwares/rotorcraft/navigation.h
  // for performance reasons the navigation variables are stored and processed in Binary Fixed-Point format
  nav_heading = ANGLE_BFP_OF_REAL(new_heading);

//  VERBOSE_PRINT("Increasing heading to %f\n", DegOfRad(new_heading));
  return false;
}

/*
 * Calculates coordinates of distance forward and sets waypoint 'waypoint' to those coordinates
 */
uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters)
{
  struct EnuCoor_i new_coor;
  calculateForwards(&new_coor, distanceMeters);
  moveWaypoint(waypoint, &new_coor);
  return false;
}

/*
 * Calculates coordinates of a distance of 'distanceMeters' forward w.r.t. current position and heading
 */
uint8_t calculateForwards(struct EnuCoor_i *new_coor, float distanceMeters)
{
  float heading  = stateGetNedToBodyEulers_f()->psi;

  // Now determine where to place the waypoint you want to go to
  new_coor->x = stateGetPositionEnu_i()->x + POS_BFP_OF_REAL(sinf(heading) * (distanceMeters));
  new_coor->y = stateGetPositionEnu_i()->y + POS_BFP_OF_REAL(cosf(heading) * (distanceMeters));
//  VERBOSE_PRINT("Calculated %f m forward position. x: %f  y: %f based on pos(%f, %f) and heading(%f)\n", distanceMeters,
//                POS_FLOAT_OF_BFP(new_coor->x), POS_FLOAT_OF_BFP(new_coor->y),
//                stateGetPositionEnu_f()->x, stateGetPositionEnu_f()->y, DegOfRad(heading));
  return false;
}



uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor)
{
//  VERBOSE_PRINT("Moving waypoint %d to x:%f y:%f\n", waypoint, POS_FLOAT_OF_BFP(new_coor->x),
//                POS_FLOAT_OF_BFP(new_coor->y));
  waypoint_move_xy_i(waypoint, new_coor->x, new_coor->y);
  return false;
}



