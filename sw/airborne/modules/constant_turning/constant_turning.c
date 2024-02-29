#include "modules/constant_turning/constant_turning.h"
#include "firmwares/rotorcraft/navigation.h"
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
  printf("turning init...");
  return;
}
void constant_turning_periodic(void)
{
  // constant turning with angular velocity
  printf("Turning...");
  guidance_h_set_body_vel(0.5f, 0);
  guidance_h_set_heading_rate(RadOfDeg(20.f));
  return;
}

