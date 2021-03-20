//
// Created by anish on 14-03-21.
//

//TODO: fix the names of the files

#include "ground_detector.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "generated/airframe.h"
#include "state.h"
#include "subsystems/abi.h"
#include <stdio.h>
#include <time.h>

#include "modules/computer_vision/ground_detection_testing.h"
#include "firmwares/rotorcraft/navigation.h"
#include "generated/flight_plan.h"

#define NAV_C // needed to get the nav functions like Inside... ???


#define GROUND_DETECTOR_VERBOSE TRUE

#define PRINT(string,...) fprintf(stderr, "[ground_detector->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if GROUND_DETECTOR_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

volatile int go_no_go;

uint8_t chooseRandomIncrementAvoidance(void);
//uint8_t chooseRandomIncrementAvoidance2(void); //in case we have L/R


enum navigation_state_t {
  SAFE,
  OBSTACLE_FOUND,
  SEARCH_FOR_SAFE_HEADING,
  OBSTACLE_FOUND_L,
  OBSTACLE_FOUND_R
};

// Define settings - also ComputerVision ones?
float GD_MAX_SPEED = 0.5f;               // max flight speed [m/s]
float GD_HEADING_RATE = RadOfDeg(20.f);  // heading change setpoint for avoidance [rad/s]
float GD_SLOW_FACTOR_FOUND = 0.2f;
float GD_SLOW_FACTOR_SEARCH = 0.2f;


// Define and initialise global variables
enum navigation_state_t navigation_state = SAFE;   // current state in state machine
float avoidance_heading_direction = 0;  // heading change direction for avoidance [rad/s]


/*
 * Initialisation function
 */
void get_signal_init(void){
    srand(time(NULL));
    chooseRandomIncrementAvoidance(); //change to 2 if we have L/R
}

void get_signal_periodic(void){
    //TODO: Add (periodic) navigation logic
    VERBOSE_PRINT("%d \n", go_no_go); /*This prints the go_no_go variable in paparazzi. This variable is 1 if the drone
                                        can fly straight ahead and 0 otherwise*/
    //if (go_no_go ==1){
        //navigation_state = SAFE;
    //}else navigation_state = OBSTACLE_FOUND;


    //float speed_sp = fminf(oag_max_speed, 0.2f * obstacle_free_confidence); //Makes sure that we don't fly too fast into an object

    float speed_sp = GD_MAX_SPEED; //Makes sure that we don't fly too fast into an object


    switch (navigation_state){
        case SAFE:
            if (go_no_go==0){
                navigation_state = OBSTACLE_FOUND;
            } else {
            guidance_h_set_guided_body_vel(speed_sp, 0);
            }
            break;
        case OBSTACLE_FOUND:
            guidance_h_set_guided_body_vel(GD_SLOW_FACTOR_FOUND*speed_sp, 0); //slow down
        
            chooseRandomIncrementAvoidance(); // randomly select new search direction
            navigation_state = SEARCH_FOR_SAFE_HEADING;
            break;
        case SEARCH_FOR_SAFE_HEADING:
            //guidance_h_set_guided_heading_rate(avoidance_heading_direction * oag_heading_rate); //ROTATE
            guidance_h_nav_new((avoidance_heading_direction * GD_HEADING_RATE), GD_SLOW_FACTOR_SEARCH*speed_sp, 0); 

            //NEED A LOOP HERE THAT ENDS WITH SAFE ONCE WE HAVE A 1
            if (go_no_go ==1){
                guidance_h_set_guided_heading(stateGetNedToBodyEulers_f()->psi);
                navigation_state = SAFE;
            }
            break;
        default:
            break;

    }
    return;
}



/*
 * Sets the variable 'incrementForAvoidance' randomly positive/negative
 */
uint8_t chooseRandomIncrementAvoidance(void)
{
  // Randomly choose CW or CCW avoiding direction
  if (rand() % 2 == 0) {
    avoidance_heading_direction = 1.f;
    VERBOSE_PRINT("Set avoidance increment to: %f\n", avoidance_heading_direction * GD_HEADING_RATE);
  } else {
    avoidance_heading_direction = -1.f;
    VERBOSE_PRINT("Set avoidance increment to: %f\n", avoidance_heading_direction * GD_HEADING_RATE);
  }
  return false;
}




//uint8_t chooseRandomIncrementAvoidance2(void) //don't want this one to be random
//{
  // Randomly choose CW or CCW avoiding direction
  //if (rand() % 2 == 0) { //randomly chooses whether obstacle is L/R, to be changed by pixel count
    //navigation_state = OBSTACLE_FOUND_L;   //many bad pixels on left side
  //} else {
    //navigation_state = OBSTACLE_FOUND_R;   //else, bad pixels on the right side
  //}
  //if (navigation_state == OBSTACLE_FOUND_L) { 
    //avoidance_heading_direction = 1.f; 	   // if obstacle on left, turn right
    //VERBOSE_PRINT("Set avoidance increment to: %f\n", avoidance_heading_direction * OAG_HEADING_RATE); //this is only a print, the real thing is in SEARCH_FOR_NEW_HEADING
  //} else{
    //avoidance_heading_direction = -1.f;    // if obstacle on the right, turn left
    //VERBOSE_PRINT("Set avoidance increment to: %f\n", avoidance_heading_direction * OAG_HEADING_RATE);
  //}
  
  //return false;
//}