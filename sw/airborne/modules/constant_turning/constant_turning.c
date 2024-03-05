#include "modules/constant_turning/constant_turning.h"
//#include "firmwares/rotorcraft/navigation.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"

#include "generated/airframe.h"
#include "state.h"
#include "modules/core/abi.h"
#include <time.h>
#include <stdio.h>

#define NAV_C // needed to get the nav functions like Inside...
#include "generated/flight_plan.h"
// init: empty for now, we could get the velocity from the settings
void constant_turning_init(void)
{ 
  srand(time(NULL));
  printf("init...");

  // waypoint_move_here_2d(WP_GOAL);
  // waypoint_move_here_2d(WP_TRAJECTORY);
  return;
}
void constant_turning_periodic(void)
{
  // constant turning with angular velocity
  printf("Turning (guided)...");

  // moveWaypointForward(WP_TRAJECTORY, 1.5f);

  // guidance_h_set_body_vel(1.0f, 0);
  guidance_h_set_heading_rate(RadOfDeg(15.f));
  return;
}

