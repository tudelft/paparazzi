// implementation of the green detector inside the orange_avoider module
#include "modules/orange_avoider/orange_avoider.h"
#include "firmwares/rotorcraft/navigation.h"
#include "generated/flight_plan.h"
#include "generated/airframe.h"
#include "modules/core/abi.h"
#include "state.h"
#include <time.h>
#include <stdio.h>

#define NAV_C
#define ORANGE_AVOIDER_VERBOSE TRUE
#define PRINT(string,...) fprintf(stderr, "[orange_avoider->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if ORANGE_AVOIDER_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

// declaring functions based on the green detector implementation
static uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters);
static uint8_t calculateForwards(struct EnuCoor_i *new_coor, float distanceMeters);
static uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor);
static uint8_t increase_nav_heading(float incrementDegrees);
static uint8_t change_direction(void);

// define the navigation states, this is all the states the drone can be in
enum navigation_state_t {
  SAFE,
  OBSTACLE_FOUND,
  SEARCH_FOR_SAFE_HEADING,
  OUT_OF_BOUNDS
  };

// define and initialise global variables
enum navigation_state_t navigation_state = SEARCH_FOR_SAFE_HEADING; // starting state
int32_t color_count = 0;                      // green color count from color filter for obstacle detection
int16_t obstacle_free_confidence = 0;         // a measure of how certain we are that the way ahead is safe.
float heading_increment = 5.f;                // heading angle increment [deg]
float maxDistance = 5;                        // max waypoint displacement [m]

int32_t segment_perc[5] = {0};                // array of segment percentages from color filter
int32_t green_perc_threshold = 20;            // percentage threshold for obstacle detection right in front of the drone
int32_t green_sides_threshold = 15;           // percentage threshold for obstacle detection on the sides
const int16_t max_trajectory_confidence = 5;  // number of consecutive negative object detections to be sure we are obstacle free
static abi_event color_detection_ev;          // creating an ABI event for the color detection callback

// function to bind the color detection callback
static void color_detection_cb(int32_t quality, int32_t segment1_count, int32_t segment2_count, int32_t segment3_count, int32_t segment4_count, int32_t segment5_count)
{
  color_count = quality;
  // segment_perc[0] = segment1_count;  // the green percentage of the outer most segment, not used
  segment_perc[1] = segment2_count;
  segment_perc[2] = segment3_count;
  segment_perc[3] = segment4_count;
  // segment_perc[4] = segment5_count;  // the green percentage of the outer most segment, not used
}


// initialisation function, called once at the start of the program
void green_detector_init(void)
{
  // Initialise random values
  srand(time(NULL));
  change_direction();

  // bind our colorfilter callbacks to receive the color filter outputs
  AbiBindMsgVISUAL_DETECTION(ORANGE_AVOIDER_VISUAL_DETECTION_ID, &color_detection_ev, color_detection_cb);
  AbiBindMsgSEGMENT_COUNTS(GREEN_DETECTOR_SEGMENT_COUNTS_ID, &color_detection_ev, color_detection_cb);
}


// function that checks it is safe to move forwards, and then moves a waypoint forward or changes the heading
void green_detector_periodic(void)
{
  // only evaluate our state machine if we are flying
  if(!autopilot_in_flight()){
    return;
  }
  
  // update our safe confidence using color threshold
  if(segment_perc[2] > green_perc_threshold){
    obstacle_free_confidence++;
  } else {
    obstacle_free_confidence -= 3;  // be more cautious with positive obstacle detections
  }

  // printing the green percentage at every segment for debugging
  printf("Segment percentages: %d %d %d %d %d\n", segment_perc[0], segment_perc[1], segment_perc[2], segment_perc[3], segment_perc[4]);

  // bound obstacle_free_confidence, so it doesnt get to big
  Bound(obstacle_free_confidence, 0, max_trajectory_confidence);

  // calculate the move distance based on the obstacle free confidence
  float moveDistance = fminf(maxDistance, 0.2f * obstacle_free_confidence);

  // meain loop for the state machine
  switch (navigation_state){

    // if safe, move forward, and check if we are still safe
    case SAFE:
      moveWaypointForward(WP_TRAJECTORY, 1.f * moveDistance);
      if (obstacle_free_confidence <= 0 || (segment_perc[1] < green_sides_threshold || segment_perc[3] < green_sides_threshold)){
        if (segment_perc[1] < green_sides_threshold || segment_perc[3] < green_sides_threshold){
          obstacle_free_confidence = 0;
        }
        navigation_state = OBSTACLE_FOUND;
        printf("- found obstacle");
      } 
      else {
        moveWaypointForward(WP_GOAL, moveDistance);
        moveWaypointForward(WP_RETREAT, -1.0f * moveDistance);
        printf("- moving forward");
      }
      break;

    // if we have found an obstacle, we need to stop and change direction
    case OBSTACLE_FOUND:
      // stop
      waypoint_move_here_2d(WP_GOAL);
      waypoint_move_here_2d(WP_RETREAT);
      waypoint_move_here_2d(WP_TRAJECTORY);

      // select direction to turn
      change_direction();

      navigation_state = SEARCH_FOR_SAFE_HEADING;

      break;

    // if we are searching for a safe heading, we need to stop and find a safe heading
    case SEARCH_FOR_SAFE_HEADING:
      increase_nav_heading(heading_increment);
      // make sure we have a couple of good readings before declaring the way safe
      if (obstacle_free_confidence >= 2 && (segment_perc[1] > green_sides_threshold && segment_perc[3] > green_sides_threshold) ){
        navigation_state = SAFE;
      }
      break;

    default:
      break;
  }
  return;
}


// increases the NAV heading. Assumes heading is an INT32_ANGLE. It is bound in this function.
uint8_t increase_nav_heading(float incrementDegrees)
{
  float new_heading = stateGetNedToBodyEulers_f()->psi + RadOfDeg(incrementDegrees);

  // normalize heading to [-pi, pi]
  FLOAT_ANGLE_NORMALIZE(new_heading);

  // set heading, declared in firmwares/rotorcraft/navigation.h
  nav.heading = new_heading;

  return false;
}


// calculates coordinates of distance forward and sets waypoint 'waypoint' to those coordinates
uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters)
{
  struct EnuCoor_i new_coor;
  calculateForwards(&new_coor, distanceMeters);
  moveWaypoint(waypoint, &new_coor);
  return false;
}


// calculates coordinates of a distance of 'distanceMeters' forward w.r.t. current position and heading
uint8_t calculateForwards(struct EnuCoor_i *new_coor, float distanceMeters)
{
  float heading  = stateGetNedToBodyEulers_f()->psi;

  // Now determine where to place the waypoint you want to go to
  new_coor->x = stateGetPositionEnu_i()->x + POS_BFP_OF_REAL(sinf(heading) * (distanceMeters));
  new_coor->y = stateGetPositionEnu_i()->y + POS_BFP_OF_REAL(cosf(heading) * (distanceMeters));
  return false;
}

// sets waypoint 'waypoint' to the coordinates of 'new_coor'
uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor)
{
  waypoint_move_xy_i(waypoint, new_coor->x, new_coor->y);
  return false;
}

// sets the variable 'heading_increment' randomly positive/negative
uint8_t change_direction(void)
{
  // Compare segment_perc[1] (left) and segment_perc[3] (right)
  if (segment_perc[1] > segment_perc[3]) {
    heading_increment = -4.f; // Turn left
    printf("Turning left: segment_perc[1] = %d, segment_perc[3] = %d\n", segment_perc[1], segment_perc[3]);
  } else if (segment_perc[3] > segment_perc[1]) {
    heading_increment = 4.f; // Turn right
    printf("Turning right: segment_perc[1] = %d, segment_perc[3] = %d\n", segment_perc[1], segment_perc[3]);
  }
  return false;
}

