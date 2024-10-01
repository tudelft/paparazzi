/*
 * Copyright (C) 2024 Dennis van Wijngaarden <D.C.vanWijngaarden@tudelft.nl>
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

/** @file "modules/nav/nav_rotwing.c"
 * @author Dennis van Wijngaarden <D.C.vanWijngaarden@tudelft.nl>
 * Custom mission nav patterns for the rotating wing drone
 */

#include "modules/nav/nav_rotwing.h"
#include "navigation.h"
#include "firmwares/rotorcraft/autopilot_firmware.h"

#include "modules/rotwing_drone/rotwing_state.h"

#include "generated/flight_plan.h" // TODO. Make fp independent
#include "sonar/agl_dist.h"

// navigation time step
static const float dt_navigation = 1.0f / ((float)NAVIGATION_FREQUENCY);

enum TakeoffStatus {
  StartEngine,
  RunEngine,
  Takeoff_init,
  Takeoff,
  Climb_init,
  Climb
};

enum LandingStatus {
  Descend,
  Flare,
  Flarelow
};

static enum TakeoffStatus rotwing_takeoff_status = StartEngine;
static enum LandingStatus rotwing_landing_status = Descend;

static float takeoff_timer = 0.0f;
static float landing_timer = 0.0f;

#if USE_MISSION
#include "modules/mission/mission_common.h"

static bool nav_rotwing_takeoff(uint8_t nb __attribute__((unused)), float *params __attribute__((unused)), enum MissionRunFlag flag)
{
  if (flag == MissionInit) {
    if (!autopilot.in_flight) {
      rotwing_takeoff_status = StartEngine;
      printf("Start_engine\n");
    } else {
      rotwing_takeoff_status = Climb;
    }
    takeoff_timer = 0.0f;
    return nav_rotwing_takeoff_run();
  }
  else if (flag == MissionRun) {
    return nav_rotwing_takeoff_run();
  }
  return false;
}

static bool nav_rotwing_land(uint8_t nb __attribute__((unused)), float *params __attribute__((unused)), enum MissionRunFlag flag)
{
  if (flag == MissionInit) {
    rotwing_state_set(ROTWING_STATE_REQUEST_HOVER);
    rotwing_landing_status = Descend;
    landing_timer = 0.0f;
    return nav_rotwing_land_run();
  }
  else if (flag == MissionRun) {
    return nav_rotwing_land_run();
  }
  return false;
}

#endif // USE_MISSION

void nav_rotwing_init(void)
{
  #if USE_MISSION
    mission_register(nav_rotwing_takeoff, "TO");
    mission_register(nav_rotwing_land, "LAND");
  #endif
}

bool nav_rotwing_takeoff_run(void)
{
  switch (rotwing_takeoff_status) {
    case StartEngine:
      rotwing_state_set(ROTWING_STATE_FORCE_HOVER);
      NavResurrect();
      NavAttitude(RadOfDeg(0));
      NavVerticalAutoThrottleMode(RadOfDeg(0));
      NavVerticalThrottleMode(9600*(0));
      // Switch to next state
      rotwing_takeoff_status = RunEngine;
      break;
    case RunEngine:
      if (takeoff_timer > 10.0) {
        rotwing_takeoff_status = Takeoff_init;
      }
      break;
    case Takeoff_init:
      autopilot_set_in_flight(true);
      NavSetWaypointHere(WP_CLIMB);
      NavAttitude(RadOfDeg(0));
      NavVerticalAutoThrottleMode(RadOfDeg(0));
      NavVerticalThrottleMode(9600*(0.750000));
      rotwing_takeoff_status = Takeoff;
      break;
    case Takeoff:
      if (takeoff_timer > 13. || (agl_dist_valid && (agl_dist_value>1.0))) {
        rotwing_takeoff_status = Climb_init;
      }
      break;
    case Climb_init:
      nav_set_heading_current();
      NavGotoWaypoint(WP_CLIMB);
      NavVerticalAutoThrottleMode(RadOfDeg(0.000000));
      NavVerticalClimbMode(nav.climb_vspeed);
      rotwing_takeoff_status = Climb;
      break;
    case Climb:
      if (GetPosAlt()>20.000000) {
        return false;
      }
      break;
  }

  takeoff_timer += dt_navigation;

  return true;
}

bool nav_rotwing_land_run(void)
{
  return false;
}