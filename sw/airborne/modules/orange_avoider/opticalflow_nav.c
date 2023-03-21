#include "modules/orange_avoider/orange_avoider.h"
#include "firmwares/rotorcraft/navigation.h"
#include "generated/airframe.h"
#include "state.h"
#include "modules/core/abi.h"
#include <time.h>
#include <stdio.h>

#define NAV_C // needed to get the nav functions like Inside...
// #include "generated/flight_plan.h"

#define ORANGE_AVOIDER_VERBOSE TRUE

#define PRINT(string,...) fprintf(stderr, "[orange_avoider->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if ORANGE_AVOIDER_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

float oa_color_count_frac = 0.18f; // ***HAVE TO DEFINE THIS HERE TO GET IT TO COMPILE

// Define navigation states
enum navigation_state_t {
  SAFE,
  OBSTACLE_LEFT,
  OBSTACLE_RIGHT,
  OBSTACLE_MIDDLE,
  OUT_OF_BOUNDS,
};



// define variables
enum navigation_state_t navigation_state = SAFE;
float flowleft = 0.0f;
float flowright = 0.0f;
float flowmiddle = 0.0f;

// float flowcombined = flowleft + flowright + flowmiddle;

float flowleft_threshold = 5.0f;
float flowright_threshold = 5.0f;
float flowmiddle_threshold = 5.0f;

float flowleft_temp = 0.0f;
float flowright_temp = 0.0f;

// float flowcombined_treshold = 10.0f;
// if flowcombined > flowcombined_treshold, then turn 180 degrees (run away)

float heading_increment = 5.f; 
float maxDistance = 2.25;   

// define functions
static uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters);
static uint8_t calculateForwards(struct EnuCoor_i *new_coor, float distanceMeters);
static uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor);
static uint8_t increase_nav_heading(float incrementDegrees);


void orange_avoider_init(void)
{
  // Initialise random values
  srand(time(NULL));

  // bind our colorfilter callbacks to receive the color filter outputs
  // not sure if needed
  // AbiBindMsgVISUAL_DETECTION(ORANGE_AVOIDER_VISUAL_DETECTION_ID, &color_detection_ev, color_detection_cb);
}

void orange_avoider_periodic(void)
{
  // only evaluate our state machine if we are flying
  if(!autopilot_in_flight()){
    return;
  }
  moveWaypointForward(WP_TRAJECTORY, 0.5f);

//   // compute current color thresholds
//   // int32_t color_count_threshold = oa_color_count_frac * front_camera.output_size.w * front_camera.output_size.h;
  
//   //retrieve flow values
//   //VERBOSE_PRINT("Optic Flow Left: %d  threshold: %d state: %d \n", flowleft, flowleft_threshold, navigation_state); *****GIVES COMPILE ERROR
//   //VERBOSE_PRINT("Optic Flow Right: %d  threshold: %d state: %d \n", flowright, flowright_threshold, navigation_state);
//   //VERBOSE_PRINT("Optic Flow Middle: %d  threshold: %d state: %d \n", flowmiddle, flowmiddle_threshold, navigation_state);

//   // float moveDistance = fminf(maxDistance, 0.2f * obstacle_free_confidence);

//   float moveDistance = fminf(maxDistance, 0.8f);

//   switch (navigation_state){
//     case SAFE:
//       // Move waypoint forward
//       moveWaypointForward(WP_TRAJECTORY, 1.5f * moveDistance);

//       // set flowmiddle equal to 10 with a 10% probability
//       if (rand() % 10 == 0){
//         flowmiddle = 10;
//       } else {
//         flowmiddle = 0;
//       }
//       if (!InsideObstacleZone(WaypointX(WP_TRAJECTORY),WaypointY(WP_TRAJECTORY))){
//         navigation_state = OUT_OF_BOUNDS;
//       } else if (flowmiddle > flowmiddle_threshold){ // added this
//         navigation_state = OBSTACLE_MIDDLE;
//       } else if (flowleft > flowleft_threshold){ // added this
//         navigation_state = OBSTACLE_LEFT;
//       } else if (flowright > flowright_threshold){ // added this
//         navigation_state = OBSTACLE_RIGHT;
//       } else {
//         moveWaypointForward(WP_GOAL, moveDistance);
//       }

//       break;
//     case OBSTACLE_LEFT:
//       // stop
//       waypoint_move_here_2d(WP_GOAL);
//       waypoint_move_here_2d(WP_TRAJECTORY);

//       // set new waypoint 20 degrees to the right and 0.8 meter forward
//        for (int i = 0; i < 5; i++     ){
//           increase_nav_heading(6.f);
//         }
//       moveWaypointForward(WP_TRAJECTORY, 0.8f);
//       navigation_state = SAFE;
//       break;

//     case OBSTACLE_RIGHT:
//       // stop
//       waypoint_move_here_2d(WP_GOAL);
//       waypoint_move_here_2d(WP_TRAJECTORY);

//       // set new waypoint 20 degrees to the left and 0.8 meter forward
  
//       for (int i = 0; i < 5; i++     ){
//           increase_nav_heading(-6.f);
//         }
  


//       moveWaypointForward(WP_TRAJECTORY, 0.8f);
//       navigation_state = SAFE;
//       break;

//     case OBSTACLE_MIDDLE:
//       // stop
//       waypoint_move_here_2d(WP_GOAL);
//       waypoint_move_here_2d(WP_TRAJECTORY);

//       // set new waypoint 45 degrees to the left and 0.8 meter forward if obstacle is on the left, set waypoint to the right, inside a for loop with range 3
     

//       // make flowleft equal to 1 with 50% probability
//       if (rand() % 2 == 0){
//         flowleft_temp = 1;
//       } else {
//         flowleft_temp = -1;
        
//       }
//       increase_nav_heading(45.f);

// /*
//       for (int i = 0; i < 5; i++     ){
//         if (flowleft_temp > flowright_temp){
//           increase_nav_heading(9.f);
//           printf("left");
//         } else {
//           increase_nav_heading(-9.f);
//           printf("right");
//         }
//       }
// */

   

      

//       moveWaypointForward(WP_TRAJECTORY, 0.1f);
//       navigation_state = SAFE;
//       break;
//     case OUT_OF_BOUNDS:
//       increase_nav_heading(heading_increment);
//       moveWaypointForward(WP_TRAJECTORY, 1.5f);

//       if (InsideObstacleZone(WaypointX(WP_TRAJECTORY),WaypointY(WP_TRAJECTORY))){
//         // add offset to head back into arena
//         increase_nav_heading(heading_increment);
//         navigation_state = SAFE;
//       }
//       break;
//     default:
//       break;
//   }
//   return;
}

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

  VERBOSE_PRINT("Increasing heading to %f\n", DegOfRad(new_heading));
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
  VERBOSE_PRINT("Calculated %f m forward position. x: %f  y: %f based on pos(%f, %f) and heading(%f)\n", distanceMeters,	
                POS_FLOAT_OF_BFP(new_coor->x), POS_FLOAT_OF_BFP(new_coor->y),
                stateGetPositionEnu_f()->x, stateGetPositionEnu_f()->y, DegOfRad(heading));
  return false;
}




/*
 * Sets waypoint 'waypoint' to the coordinates of 'new_coor'
 */
uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor)
{
  VERBOSE_PRINT("Moving waypoint %d to x:%f y:%f\n", waypoint, POS_FLOAT_OF_BFP(new_coor->x),
                POS_FLOAT_OF_BFP(new_coor->y));
  waypoint_move_xy_i(waypoint, new_coor->x, new_coor->y);
  return false;
}