/**
 * This file is created by TEAM 2 from 2023 for the AFMAV course
 */

#include "firmwares/rotorcraft/navigation.h"
#include "generated/airframe.h"
#include "state.h"
#include "modules/core/abi.h"
#include <time.h>
#include <stdio.h>
#include <sys/time.h>

#define NAV_C // needed to get the nav functions like Inside...
#include "generated/flight_plan.h"

#define OBJECT_AVOIDER_VERBOSE TRUE

#define PRINT(string,...) fprintf(stderr, "[object_avoider->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if OBJECT_AVOIDER_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

static uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters);
static uint8_t calculateForwards(struct EnuCoor_i *new_coor, float distanceMeters);
static uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor);
static uint8_t increase_nav_heading(float incrementDegrees);
static void lowest_index_cb(uint8_t sender_id, uint8_t i_safe);

enum navigation_state_t {
  SAFE,
  OBSTACLE_FOUND,
  OUT_OF_BOUNDS
};

// define and initialise global variables
enum navigation_state_t navigation_state = SAFE;
float angle = 0;
float heading_increment = 100.f; // heading angle increment when out of bounds (is applied 2 times, so approx. a U turn [deg]
int new_heading_index = 0;

// These parameters are in the xml-file: conf/modules/object_avoider.xml
#ifndef N_DIRBLOCKS // Possible heading directions
#define N_DIRBLOCKS 5
#endif

#ifndef FOV_ANGLE // Determines the size of the steering correction when detecting an obstacle
#define FOV_ANGLE 160
#endif

#ifndef DIR_CHANGE_THRESHOLD // Number of consecutive measurements necessary to decide to steer away
#define DIR_CHANGE_THRESHOLD 10
#endif

#ifndef PAUSE_TIME // The confidence values reset and aren't updated for (PAUSE_TIME * 100ms) after a steer correction
#define PAUSE_TIME 15
#endif

#ifndef PAUSE_V_FACTOR // The percentage of the default speed it has when pausing [0.0 - 1.0]
#define PAUSE_V_FACTOR 0.4
#endif

unsigned long pause_time = PAUSE_TIME;
int param_DIR_CHANGE_THRESHOLD = DIR_CHANGE_THRESHOLD;
int param_FOV_ANGLE = FOV_ANGLE;
int param_N_DIRBLOCKS = N_DIRBLOCKS;
int direction_accumulator[N_DIRBLOCKS] = {0};
int center_index = N_DIRBLOCKS / 2; // The center index -> for flying straight ahead

// Variables used to pause the confidence updates during turning
bool do_pause = false;
struct timeval start_time, now;

static abi_event lowestFilteredIndex;
uint8_t lowest_index = 120; // Just an arbitrary value -> stores the value send by the ABI DIVERGENCE_SAFE_HEADING

// Callback for the ABI DIVERGENCE_SAFE_HEADING
// These messages are send by the image processing thread. It contains the index of the direction with the lowest
// amount of obstacles detected.
static void lowest_index_cb(uint8_t sender_id, uint8_t i_safe) {
    // If the drone is not turning, so it shouldn't pause
    // +1 to the preferred index and -1 for all other indeces
    if (!do_pause) {
        for (int i = 0; i < N_DIRBLOCKS; i++) {
            if (i == i_safe) {
                direction_accumulator[i]++;
                if (direction_accumulator[i] == param_DIR_CHANGE_THRESHOLD && i != center_index) {
                    navigation_state = OBSTACLE_FOUND;
                    new_heading_index = i_safe;
                }
            } else {
                if (direction_accumulator[i] != 0) {
                    direction_accumulator[i]--;
                }

            }
        }
    // If the drone is turning and should pause:
    // - Don't update the confidence
    // - If the pausing is over; reset the confidence values
    } else {
        gettimeofday(&now, NULL);
        unsigned long difference = (now.tv_sec - start_time.tv_sec) * 1000000 + now.tv_usec - start_time.tv_usec;

        // Check if the pausing should stop
        if ((difference) >= (pause_time * 100 * 1000)) { // * 100 ms * 1000 (us -> ms)
            do_pause = false;
            // reset the confidence values
            for (int i = 0; i < N_DIRBLOCKS; i++) {
                direction_accumulator[i] = 0;
            }
        }
    }
}

// Init function where the ABI is bound
void object_avoider_init(void)
{
  // Initialise random values
  srand(time(NULL));

    // Simply subscribe to all the send ABI messages of type DIVERGENCE_SAFE_HEADING
    AbiBindMsgDIVERGENCE_SAFE_HEADING(ABI_BROADCAST, &lowestFilteredIndex, lowest_index_cb);
}

// The autopilot function that decides whether to turn or not
void object_avoider_periodic(void)
{
    // only evaluate our state machine if we are flying
    if(!autopilot_in_flight()){
        return;
    }

    float moveDistance = 0.5f;
    int angle;

    switch (navigation_state){
        case SAFE:
            // if the drone is turning and the image processing output is not useful, move forward a bit slower
            if (do_pause) {
                moveDistance = PAUSE_V_FACTOR * moveDistance;
            }

            // Move forward
            moveWaypointForward(WP_TRAJECTORY, 1.5f * moveDistance);
            if (!InsideObstacleZone(WaypointX(WP_TRAJECTORY),WaypointY(WP_TRAJECTORY))){
                navigation_state = OUT_OF_BOUNDS;
            } else {
                moveWaypointForward(WP_GOAL, moveDistance);
            }

            break;
        case OBSTACLE_FOUND:
            // Calculate the steering angle based on the best index, the total number of indeces and the field of view
            angle = (1.0 * param_FOV_ANGLE / param_N_DIRBLOCKS) * (new_heading_index + 0.5f) - (param_FOV_ANGLE) / 2.0f;

            // The output of the image processing is useless during turning, should the output should be ignored
            // for a specific period of time.
            do_pause = true;
            // Save the current time to know when to start using the output of the image processing again
            gettimeofday(&start_time, NULL);

            // Apply the calculated angle.
            increase_nav_heading(angle);

            navigation_state = SAFE;

            break;
        case OUT_OF_BOUNDS:
            increase_nav_heading(heading_increment);
            moveWaypointForward(WP_TRAJECTORY, 1.5f);

            if (InsideObstacleZone(WaypointX(WP_TRAJECTORY),WaypointY(WP_TRAJECTORY))){
                // add offset to head back into arena
                increase_nav_heading(heading_increment);

                // It is safe to continue again
                navigation_state = SAFE;
            }

            // The output of the image processing is useless during turning, should the output should be ignored
            // for a specific period of time.
            do_pause = true;
            // Save the current time to know when to start using the output of the image processing again
            gettimeofday(&start_time, NULL);
            break;
        default:
            break;
    }

    return;
}

// Increases the heading, copied from the orange avoider module
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

// Move the waypoint forward, copied from the orange avoider module
uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters)
{
  struct EnuCoor_i new_coor;
  calculateForwards(&new_coor, distanceMeters);
  moveWaypoint(waypoint, &new_coor);
  return false;
}


// Calculates coordinates of a distance of 'distanceMeters' forward w.r.t. current position and heading
// copied from the orange avoider module
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


// Sets waypoint 'waypoint' to the coordinates of 'new_coor'
// copied from the orange avoider module
uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor)
{
  VERBOSE_PRINT("Moving waypoint %d to x:%f y:%f\n", waypoint, POS_FLOAT_OF_BFP(new_coor->x),
                POS_FLOAT_OF_BFP(new_coor->y));
  waypoint_move_xy_i(waypoint, new_coor->x, new_coor->y);
  return false;
}
