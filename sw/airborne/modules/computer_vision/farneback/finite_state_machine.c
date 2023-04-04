#include "finite_state_machine.h"
#define NAV_C // needed to get the nav functions like Inside...
#include "generated/flight_plan.h"
#include "modules/orange_avoider/orange_avoider.h"
#include "firmwares/rotorcraft/navigation.h"
#include "generated/airframe.h"
#include "state.h"
#include "modules/core/abi.h"
#include <time.h>
#include <stdio.h>

// Navigation parameters
#define MOVEDISTANCE 1.2f
#define HEADING_INC_LEFT_RIGHT 45
#define HEADING_INC_MIDDLE 60
#define HEADING_INC_OUT_OF_BOUND 90

// Decision parameters
#define SIDES_OBS_TH 1.22f
#define MIDDLE_OBS_TH_UPPER 1.3f
#define MIDDLE_OBS_TH_LOWER 0.6f
#define LEFTFLOW_TURNING_TH 1.18f
#define RIGHTFLOW_TURNING_TH 0.75f
#define MIDDLE_OBS_TH_UPPER_2_STEP 2.0f
#define MIDDLE_OBS_TH_LOWER_2_STEP 0.4f

// Counters to repeat a certain step X times
#define STEPS_COUNT_PHASE_1_TURNING 3
#define STEPS_COUNT_PHASE_2_TURNING 9


#define LOG(x) fprintf(stderr, "LOG: %s:%d %s %lu \n", __FILE__, __LINE__, x, clock());

static uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters);
static uint8_t calculateForwards(struct EnuCoor_i *new_coor, float distanceMeters);
static uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor);
static uint8_t increase_nav_heading(float incrementDegrees);

enum navigation_state_t {
  SAFE,
  JUST_TURNED,
  OBSTACLE_LEFT,
  OBSTACLE_RIGHT,
  OBSTACLE_MIDDLE,
  OUT_OF_BOUNDS,
};

enum navigation_state_t navigation_state = SAFE;


int counter_turn_1 = 0;
int counter_turn_2 = 0;
struct OpticalFlow flow_vals; // holds all the relevant flow variables

struct OpticalFlow run_fsm(float flow_left, float flow_right, float flow_middle)
{
    // Set new flow_vals
    flow_vals.middle_prev_prev = flow_vals.middle_prev;
    flow_vals.middle_prev = flow_vals.middle;
    flow_vals.left = flow_left;
    flow_vals.right = flow_right;
    flow_vals.middle = flow_middle;

    // Calculate the relevant ratios
    flow_vals.middle_divergence = (flow_middle / flow_vals.middle_prev);
    flow_vals.middle_divergence_prev = (flow_middle / flow_vals.middle_prev_prev);
    flow_vals.right_left_ratio = flow_left / flow_right; 
    log_flow();
    // State machine
    switch (navigation_state){
      case SAFE:
        state_safe();

      case JUST_TURNED:
        state_just_turned();
        break;

      case OBSTACLE_LEFT:
        state_obs_left();
        break;

      case OBSTACLE_RIGHT:
        state_obs_right();
        break;

    case OBSTACLE_MIDDLE: 
      state_obs_middle();
      break;
    case OUT_OF_BOUNDS:
      state_out_of_bounds();
      break;
    default:
      break;
  }
  return flow_vals;
}


void log_flow(void)
{
  printf("New cycle \n");
  printf("flowleft: %f \n", flow_vals.left);
  printf("flowright: %f \n", flow_vals.right);
  printf("flowmiddle: %f \n", flow_vals.middle);
  printf("flowmiddle_prev: %f \n", flow_vals.middle_prev);
  printf("right_left_normalizer: %f \n", flow_vals.right_left_ratio);
  printf("flowmiddle divergence: %f \n", flow_vals.middle_divergence);
  printf("flowmiddle_divergence_prev: %f \n", flow_vals.middle_divergence_prev);
  printf("Navigation state: %d \n", navigation_state);
}


void state_safe(void)
{
  LOG("start of safe state")
    moveWaypointForward(WP_TRAJECTORY, MOVEDISTANCE);
    if (!InsideObstacleZone(WaypointX(WP_TRAJECTORY),WaypointY(WP_TRAJECTORY))){
      navigation_state = OUT_OF_BOUNDS;
      printf("OUT OF BOUNDS \n");
      return; 
      
    // Conditions for an obstacle on the right
    } else if (flow_vals.right_left_ratio < (1/SIDES_OBS_TH)){
      printf("Obstacle RIGHT \n");
      navigation_state = OBSTACLE_RIGHT;
      return; 
    // Conditions for an obstacle on the left
    } else if (flow_vals.right_left_ratio > SIDES_OBS_TH){
      printf("Obstacle LEFT \n");
      navigation_state = OBSTACLE_LEFT;
      return;
      
    } 
    // Conditions for an obstacle in the middle
      else if ((flow_vals.middle_divergence > MIDDLE_OBS_TH_UPPER) ||  (flow_vals.middle_divergence < MIDDLE_OBS_TH_LOWER) || (flow_vals.middle_divergence_prev < MIDDLE_OBS_TH_LOWER_2_STEP) || (flow_vals.middle_divergence_prev > MIDDLE_OBS_TH_UPPER_2_STEP)){
      printf("Obstacle MIDDLE \n");
      navigation_state = OBSTACLE_MIDDLE;
      return; 
    } 
    else {
      moveWaypointForward(WP_GOAL, 0.5f * MOVEDISTANCE);
      printf("NO OBSTACLE \n");
    }
    LOG("end of safe state")
    return;
}

void state_obs_right(void)
{
  LOG("OBSTACLE RIGHT")
  waypoint_move_here_2d(WP_GOAL);
  waypoint_move_here_2d(WP_TRAJECTORY);
  increase_nav_heading(-HEADING_INC_LEFT_RIGHT);
  navigation_state = JUST_TURNED;
}

void state_obs_left(void)
{
  LOG("OBSTACLE LEFT")
  waypoint_move_here_2d(WP_GOAL);
  waypoint_move_here_2d(WP_TRAJECTORY);
  increase_nav_heading(HEADING_INC_LEFT_RIGHT);
  navigation_state = JUST_TURNED;
}

void state_obs_middle(void)
{
  LOG("OBSTACLE MIDDLE")
  waypoint_move_here_2d(WP_GOAL);
  waypoint_move_here_2d(WP_TRAJECTORY);
  increase_nav_heading(HEADING_INC_MIDDLE);
  navigation_state = JUST_TURNED; 
}

void reset_counters_turn(void)
{
  counter_turn_1 = 0;
  counter_turn_2 = 0;
}

void state_just_turned(void)
{
  LOG("JUST_TURNED")
  // Right after turning we don't look at any obstacles but very briefly move forward
  moveWaypointForward(WP_TRAJECTORY, 0.5 * MOVEDISTANCE);
  moveWaypointForward(WP_GOAL, 0.5 * MOVEDISTANCE);
  if (counter_turn_1 < STEPS_COUNT_PHASE_1_TURNING){
      counter_turn_1++;
      return;
    }
    counter_turn_2++;
  // If we are out of bounds, we need to move away
  if (!InsideObstacleZone(WaypointX(WP_TRAJECTORY),WaypointY(WP_TRAJECTORY))){
    navigation_state = OUT_OF_BOUNDS;
    reset_counters_turn();
    return;
  } 
  // briefly after turning, the middle flow values sometimes become slightly of the charts, 
  // therefore we only look at left and right flow with slightly different thresholds
  else if (flow_vals.right_left_ratio < RIGHTFLOW_TURNING_TH){ // added this
    LOG("after obstacle right evaluation")
    navigation_state = OBSTACLE_RIGHT;
    reset_counters_turn();
    return;
         
  } 
  else if (flow_vals.right_left_ratio > LEFTFLOW_TURNING_TH){ 
    printf("Obstacle LEFT \n");
    navigation_state = OBSTACLE_LEFT;
    reset_counters_turn();
    return;
  } 
  else if (counter_turn_2 == STEPS_COUNT_PHASE_1_TURNING){
    navigation_state = SAFE;
    reset_counters_turn();
    return;
  }
}

void state_out_of_bounds(void)
{
  LOG("Out of bounds")
  waypoint_move_here_2d(WP_GOAL);
  waypoint_move_here_2d(WP_TRAJECTORY);
  increase_nav_heading(HEADING_INC_OUT_OF_BOUND);
  moveWaypointForward(WP_TRAJECTORY, MOVEDISTANCE);
  if (!InsideObstacleZone(WaypointX(WP_TRAJECTORY), WaypointY(WP_TRAJECTORY))){
    navigation_state = OUT_OF_BOUNDS; 
    return;
    }
  else {
    LOG("NOT OUT OF BOUNDS ANYMORE")
    moveWaypointForward(WP_GOAL, MOVEDISTANCE);
    counter_turn_1 = STEPS_COUNT_PHASE_1_TURNING; // Skip the first phase of the procedure that is performed after turning when going out of bounds
    navigation_state = JUST_TURNED;
    return;
  }
}

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
