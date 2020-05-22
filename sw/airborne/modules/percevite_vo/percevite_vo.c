

#include "modules/percevite_vo/percevite_vo.h"
#include "generated/flight_plan.h"
#include "firmware/rotorcraft/navigation.h"

bool percevite_requires_avoidance = false;



// Module init


// Module driver (event) -> if data from "Avoid device"


// Module periodic
// Velocity obtacle:

void avoid_periodic(void) // 512Hz
// if too close: abort normal navigation

percevite_requires_avoidance = true;
// Compute where to fly to:
// move_forward(WP_AVOID, distance, heading, speed)


// when far again, resume normal navigation
percevite_requires_avoidance = false;
