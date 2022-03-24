/*
 * Copyright (C) Kirk Scheper <kirkscheper@gmail.com>
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/orange_avoider/orange_avoider_guided.c"
 * @author Kirk Scheper
 * This module is an example module for the course AE4317 Autonomous Flight of Micro Air Vehicles at the TU Delft.
 * This module is used in combination with a color filter (cv_detect_color_object) and the guided mode of the autopilot.
 * The avoidance strategy is to simply count the total number of orange pixels. When above a certain percentage threshold,
 * (given by color_count_frac) we assume that there is an obstacle and we turn.
 *
 * The color filter settings are set using the cv_detect_color_object. This module can run multiple filters simultaneously
 * so you have to define which filter to use with the ORANGE_AVOIDER_VISUAL_DETECTION_ID setting.
 * This module differs from the simpler orange_avoider.xml in that this is flown in guided mode. This flight mode is
 * less dependent on a global positioning estimate as witht the navigation mode. This module can be used with a simple
 * speed estimate rather than a global position.
 *
 * Here we also need to use our onboard sensors to stay inside of the cyberzoo and not collide with the nets. For this
 * we employ a simple color detector, similar to the orange poles but for green to detect the floor. When the total amount
 * of green drops below a given threshold (given by floor_count_frac) we assume we are near the edge of the zoo and turn
 * around. The color detection is done by the cv_detect_color_object module, use the FLOOR_VISUAL_DETECTION_ID setting to
 * define which filter to use.
 */

#include "modules/orange_avoider/orange_avoider_guided.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "firmwares/rotorcraft/navigation.h"
#include "generated/airframe.h"
#include "state.h"
#include "modules/core/abi.h"
#include <stdio.h>
#include <time.h>
#include <math.h>
#include <unistd.h> 

#define NAV_C
#include "generated/flight_plan.h"

#define ORANGE_AVOIDER_VERBOSE TRUE

#define PRINT(string,...) fprintf(stderr, "[orange_avoider_guided->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if ORANGE_AVOIDER_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif


// Initialise functions
static uint8_t calculateForwards(struct EnuCoor_i *new_coor, float distanceMeters, float heading_inc);
uint8_t chooseIncrementAvoidance(void);
uint8_t CheckWall(struct EnuCoor_i new_coor);
uint8_t RotationOperation(float *x, float *y, float *psi);
float findGap(int Nobs, float obstacles[Nobs][4]);
bool gapWideEnough(float Lx, float Lz, float Rx, float Rz, float heading);
float calcHeading(float Lx, float Lz, float Rx, float Rz);
float calcDistance(float heading);


// List of navigation states
enum navigation_state_t {
  SAFE,
  SEARCH_FOR_SAFE_HEADING,
  LAST_CHECK,
  OUT_OF_BOUNDS,
  REENTER_ARENA,
};


// Struct type from the ABI message, ideally moved to header file
struct color_object_t {
  int32_t Nobs;
  int32_t obstacle_data[100][4];
  uint32_t y_max;
  uint32_t x_max;
  bool updated;
};

// define settings
float oag_max_speed = 1.f;                // max flight speed [m/s]

float oag_oob_rate = 90.0f;               // turning rate when turning back to arena [deg/s]
float oag_oob_dist = 5.0f;                // minimum distance to fly when turning back to arena [m]

float min_gap = 1.5;                      // minimum gap between obstacles
float min_clearance = 1.2;                // minimum clearance past an obstacle


// global variables
int wall;
int count;
bool succes;
int Nobs=0;
float obstacles[100][4];


// define and initialise global variables
enum navigation_state_t navigation_state = SEARCH_FOR_SAFE_HEADING;   // current state in state machine
float avoidance_heading_direction = 0;  // heading change direction for avoidance [rad/s]


// This callback processes the obstacle data from the cv_detect_color_object module
#ifndef OPTIC_FLOW_OBSTACLE_DATA1_ID
#define OPTIC_FLOW_OBSTACLE_DATA1_ID ABI_BROADCAST
#endif
static abi_event optic_flow_obstacle_data_ev;
static void optic_flow_obstacle_data_cb(uint8_t __attribute__((unused)) sender_id,
          struct color_object_t *cv_data)
{
  Nobs = cv_data->Nobs;
  float obstacles_pixels[100][4];// = data_matrix
  int imax = cv_data->x_max;
  int jmax = cv_data->y_max;

  // RunOnceEvery(100, {
  //   for (int i=0; i<Nobs; ++i){ 
  //     VERBOSE_PRINT("od: %d %d %d %d [%d] \n", cv_data->obstacle_data[i][0], cv_data->obstacle_data[i][1], cv_data->obstacle_data[i][2], cv_data->obstacle_data[i][3], i);
  //   }
  //   VERBOSE_PRINT("\n");
  // });
      
  for (int i=0; i<Nobs; ++i){
    for (int j=0; j<4; ++j){
      obstacles_pixels[i][j] = cv_data->obstacle_data[i][j];
    }
  }

  for (int i=0; i<Nobs; i++){
    float z1 = 9.527*(obstacles_pixels[i][0]/imax)+1.9;
    float w1 = 27.8*(obstacles_pixels[i][0]/imax)+4.8;
    float x1 = (obstacles_pixels[i][1]/jmax)*w1 - w1/2;
    float z2 = 9.527*(obstacles_pixels[i][2]/imax)+1.9;
    float w2 = 27.8*(obstacles_pixels[i][2]/imax)+4.8;
    float x2 = (obstacles_pixels[i][3]/jmax)*w2 - w2/2;  

    obstacles[i][0] = x1;
    obstacles[i][1] = z1;
    obstacles[i][2] = x2;
    obstacles[i][3] = z2;  
  }


  // Now sort the obstacle matrix
  float temp[4];

  for (int i=0; i<Nobs; ++i){
      for (int k=i+1; k<Nobs; ++k){
          if(obstacles[i][0] > obstacles[k][0]) {   
              for (int j=0; j<4; ++j){
                  temp[j] = obstacles[i][j];    
                  obstacles[i][j] = obstacles[k][j];    
                  obstacles[k][j] = temp[j];  
              }
          }
      }
  }
}

/*
 * Initialisation function
 */
void orange_avoider_guided_init(void)
{
  // Initialise random values
  srand(time(NULL));

  // bind our colorfilter callbacks to receive the color filter outputs
  AbiBindMsgOF_OBSTACLE_DATA(OPTIC_FLOW_OBSTACLE_DATA1_ID, &optic_flow_obstacle_data_ev, optic_flow_obstacle_data_cb);
}

/*
 * Periodic function that flies to the edge of the zoo and then chooses the longest path between obstacles
 */
void orange_avoider_guided_periodic(void)
{
  // Only run the mudule if we are in the correct flight mode
  if (guidance_h.mode != GUIDANCE_H_MODE_GUIDED) {
    navigation_state = SEARCH_FOR_SAFE_HEADING;
    return;
  }

  // VERBOSE_PRINT("x/y/psi: %f/%f/%f\n", POS_FLOAT_OF_BFP(stateGetPositionEnu_i()->x),POS_FLOAT_OF_BFP(stateGetPositionEnu_i()->y), DegOfRad(stateGetNedToBodyEulers_f()->psi));


  // State machine
  switch (navigation_state){
    case SAFE: ;
      // VERBOSE_PRINT("Safe\n");
      struct EnuCoor_i new_coor;
      calculateForwards(&new_coor, 1.0f, 0.f);

      if (!InsideObstacleZone(POS_FLOAT_OF_BFP(new_coor.x),POS_FLOAT_OF_BFP(new_coor.y))){ // Checks if the drone will fly out of the obstacle zone
        // VERBOSE_PRINT("x/y: %s %s\n", new_coor.x, new_coor.y);
        CheckWall(new_coor);
        navigation_state = OUT_OF_BOUNDS;
      } else {
        guidance_h_set_guided_body_vel(oag_max_speed, 0);
        RunOnceEvery(10, {
          // VERBOSE_PRINT("Rechecking\n");
          count = 0;
          navigation_state = LAST_CHECK;
        });
      }

      break;
    case SEARCH_FOR_SAFE_HEADING:
      // VERBOSE_PRINT("Search for safe heading\n");
      // VERBOSE_PRINT("Nobs: %d \n", Nobs);
      // for (int i=0; i<100; ++i){
      //   for (int j=0; j<4; ++j){
      //     VERBOSE_PRINT("obstacle [%i][%i]: %f", i, j, obstacles[i][j]);
      //   }
      // }
      count ++;

      if (count > 3){
        if (Nobs==0){
          navigation_state = SAFE;
        }else{
          float heading = findGap(Nobs, obstacles);
          if (succes){
            guidance_h_set_guided_heading(stateGetNedToBodyEulers_f()->psi + heading);
            count = 0;
            navigation_state = LAST_CHECK;
          }else{
            guidance_h_set_guided_heading(stateGetNedToBodyEulers_f()->psi + avoidance_heading_direction * RadOfDeg(30.f));
            count = 0;
            navigation_state = SEARCH_FOR_SAFE_HEADING;
          }           
        }      
      }

      break;
    case LAST_CHECK: ;
      count ++;

      if (count > 3){
        for (int i=0; i<Nobs; ++i){ 
          // VERBOSE_PRINT("obs: %f %f %f %f\n", obstacles[i][0], obstacles[i][1], obstacles[i][2], obstacles[i][3]);
        }
        // VERBOSE_PRINT("\n");
        bool clear = true;
        for (int i=0; i<Nobs; ++i){
          if (obstacles[i][0]*obstacles[i][2] < 0){
            // VERBOSE_PRINT("case 1\n");
            clear = false;
          } else if (fabs(obstacles[i][0])<min_clearance){
            // VERBOSE_PRINT("case 2\n");
            clear = false;
          } else if (fabs(obstacles[i][2])<min_clearance){
            // VERBOSE_PRINT("case 3\n");
            clear = false;
          }
        }
        // VERBOSE_PRINT("clear: %d\n", clear);

        if (clear) {
          navigation_state = SAFE;
        }else{
          guidance_h_set_guided_body_vel(0.0f, 0.0f);
          guidance_h_set_guided_heading(stateGetNedToBodyEulers_f()->psi + avoidance_heading_direction * RadOfDeg(10.f));
          count = 0;
        }
      }

      break;
    case OUT_OF_BOUNDS: ;
      // VERBOSE_PRINT("Out of bounds\n");
      // stop

      chooseIncrementAvoidance();
      guidance_h_set_guided_body_vel(0.0f, 0.0f);
      // start turn back into arena
      guidance_h_set_guided_heading_rate(avoidance_heading_direction * RadOfDeg(oag_oob_rate));
      navigation_state = REENTER_ARENA;

      break;
    case REENTER_ARENA: ;
      // VERBOSE_PRINT("Reenter\n");
      // Check if coords a couple meters in front are still in the arena
      struct EnuCoor_i new_coor2;
      calculateForwards(&new_coor2, oag_oob_dist, 0.f);
      if (InsideObstacleZone(POS_FLOAT_OF_BFP(new_coor2.x), POS_FLOAT_OF_BFP(new_coor2.y))){
        // return to heading mode
        guidance_h_set_guided_heading(stateGetNedToBodyEulers_f()->psi);
        guidance_h_set_guided_body_vel(0.0f,0.0f);
        count = 0;
        navigation_state = SEARCH_FOR_SAFE_HEADING;
      }

      break;
    default:
      break;
  }
  return;
}



/*
 * Chooses a way for the drone to turn. Usually the shortest turn away from the wall, but in the corners the decision making is more complex.
 */
uint8_t chooseIncrementAvoidance(void)
{
  float x = POS_FLOAT_OF_BFP(stateGetPositionEnu_i()->x);
  float y = POS_FLOAT_OF_BFP(stateGetPositionEnu_i()->y);
  float psi = DegOfRad(stateGetNedToBodyEulers_f()->psi);

  RotationOperation(&x, &y, &psi);

  if (psi<0){
    psi = psi+360;
  }else if (psi>360){
    psi = psi-360;
  }
  
  int quad; // What quadrant does the heading point to
  if (0 <= psi && psi <= 90){
    quad = 1;
  }else if (90 <= psi && psi <= 180){
    quad = 2;
  }else if (180 <= psi && psi <= 270){
    quad = 3;
  } else if (270 <= psi && psi <= 360){
    quad = 4;
  }

  int corner; // In what quadrant is the drone
  if (x>0 && y>0){
    corner = 1;
  }else if (x>0 && y<0){
    corner = 2;
  }else if (x<0 && y<0){
    corner = 3;
  }else if (x<0 && y>0){
    corner = 4;
  }

  float rest = fmod(psi, 90.0);
  
  if (wall == quad){
    avoidance_heading_direction = 1.f;
  }else if (((wall-quad) == 1) || (wall==1 && quad==4)){
    avoidance_heading_direction = -1.f;
  }else{
    avoidance_heading_direction = 0.f;
  }

  // VERBOSE_PRINT("wall/quad/corner: %i/%i/%i\n", wall, quad, corner);

  if (fabs(x)>2.8 && fabs(y)>2.8){
    // VERBOSE_PRINT("In corner %i\n", corner);
    if(corner == quad){
      if (!(40 <= rest && rest <= 50)){
        if (rest <= 40){
          avoidance_heading_direction = -1.f;
        }else{
          avoidance_heading_direction = 1.f;
        }
      }
    }
  }
  return false;
}



/*
 * Checks what wall of the cyberzoo the drone would collide with 
 */
uint8_t CheckWall(struct EnuCoor_i new_coor)
{
  float x = POS_FLOAT_OF_BFP(new_coor.x);
  float y = POS_FLOAT_OF_BFP(new_coor.y);
  float psi = 0.0;

  RotationOperation(&x, &y, &psi);

  if (fabs(y)>fabs(x)){
    if (y>0){
      wall = 1;
    }else{
      wall = 3;
    }
  }else{
    if (x>0){
      wall = 2; 
    }else{
      wall = 4;
    }
  }
  return false;
}



/*
 * Transforms the coordinates to an inertial reference frame for ease of calculations
 */
uint8_t RotationOperation(float *x, float *y, float *psi){
  float rot_angle = RadOfDeg(35.18285188);

  float x_old = *x;
  float y_old = *y;

  *x = cosf(rot_angle)* x_old+sinf(rot_angle)* y_old;
  *y = (-sinf(rot_angle)* x_old+cosf(rot_angle)* y_old);
  *psi = *psi+DegOfRad(rot_angle);

  return false;
}




/*
 * Calculates coordinates of a distance of 'distanceMeters' forward w.r.t. current position and heading
 */
uint8_t calculateForwards(struct EnuCoor_i *new_coor, float distanceMeters, float heading_inc)
{
  float heading  = stateGetNedToBodyEulers_f()->psi+heading_inc;

  // Now determine where to place the waypoint you want to go to
  new_coor->x = stateGetPositionEnu_i()->x + POS_BFP_OF_REAL(sinf(heading) * (distanceMeters));
  new_coor->y = stateGetPositionEnu_i()->y + POS_BFP_OF_REAL(cosf(heading) * (distanceMeters));
  // VERBOSE_PRINT("Calculated %f m forward position. x: %f  y: %f based on pos(%f, %f) and heading(%f)\n", distanceMeters,	
                // POS_FLOAT_OF_BFP(new_coor->x), POS_FLOAT_OF_BFP(new_coor->y),
                // stateGetPositionEnu_f()->x, stateGetPositionEnu_f()->y, DegOfRad(heading));
  return false;
}




/*
 * Finds the gap between obstacles that will allow the drone to travel the greatest distance to the other side of the zoo
*/
float findGap(int Nobs, float obstacles[100][4]){
  float heading = 0;
  float distance = 0;
  float pot_heading;
  float pot_distance;

  VERBOSE_PRINT("Nobs: %d\n", Nobs);
 
  // for (int i=0; i<Nobs; ++i){ 
  //   VERBOSE_PRINT("obs: %f %f %f %f\n", obstacles[i][0], obstacles[i][1], obstacles[i][2], obstacles[i][3]);
  // }
  // VERBOSE_PRINT("\n");

  for (int i = 0; i<=Nobs; ++i){
    if (i==0){
      pot_heading = calcHeading(-0.8f, 1.f, obstacles[i][0], obstacles[i][1]);
      if (gapWideEnough(-0.8f, 1.f, obstacles[i][0], obstacles[i][1], pot_heading)){
        pot_distance = calcDistance(pot_heading);
      }else{
        pot_heading = 0;
        pot_distance = 0;
      }
    }else if (i==Nobs){
      pot_heading = calcHeading(obstacles[i-1][2], obstacles[i-1][3], 0.8f, 1.f);
      if (gapWideEnough(obstacles[i-1][2], obstacles[i-1][3], 0.8f, 1.f, pot_heading)){
        pot_distance = calcDistance(pot_heading);
      }else{
        pot_heading = 0;
        pot_distance = 0;
      }
    }else{
      pot_heading = calcHeading(obstacles[i-1][2], obstacles[i-1][3], obstacles[i][0], obstacles[i][1]);
      if (gapWideEnough(obstacles[i-1][2], obstacles[i-1][3], obstacles[i][0], obstacles[i][1], pot_heading)){
        pot_distance = calcDistance(pot_heading);
      }else{
        pot_heading = 0;
        pot_distance = 0;
      }
    }

    if (pot_distance>distance){
      heading = pot_heading;
      distance = pot_distance;
    }
  }

  if (heading==0 && distance==0){
    // what if no valid heading is found?
    succes = false;
  }else{
    succes = true;
  }
  // VERBOSE_PRINT("Chosen heading increment: %f\n", heading);
  return heading;
}



/*
 * Calculates if the gap between obstacles is wide enough for the drone
*/
bool gapWideEnough(float Lx, float Lz, float Rx, float Rz, float heading){
  float absolute_gap = sqrtf((Rx-Lx)*(Rx-Lx)+(Rz-Lz)*(Rz-Lz));
  float correction_angle = heading+atanf((Rz-Lz)/(Rx-Lx));
  float gap = cosf(correction_angle)*absolute_gap;
  if (fabs(gap) > min_gap){
    return true;
  }else{
    return false;
  }  
}



/*
 * Calculates heading in the middle of two obstacles
*/
float calcHeading(float Lx, float Lz, float Rx, float Rz){
    float heading1 = atanf(Lx/Lz);
    float heading2 = atanf(Rx/Rz);
    float heading = 0.5*(heading1+heading2);
  return heading;
}



/*
 * Calculates the distance to the other side of the cyberzoo
*/
float calcDistance(float heading){
  struct EnuCoor_i new_coor;
  float distance = 0.5;
  calculateForwards(&new_coor, distance, heading);
  while (InsideObstacleZone(POS_FLOAT_OF_BFP(new_coor.x),POS_FLOAT_OF_BFP(new_coor.y))){
    distance = distance+0.5;
    calculateForwards(&new_coor, distance, heading);
  }
  distance = distance-0.5;
  return distance;
}




