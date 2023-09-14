/*
 * Copyright (C) 2023 Dennis van Wijngaarden <D.C.vanWijngaarden@tudelft.nl>
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

/** @file "modules/rotwing/rotwing_state.c"
 * @author Dennis van Wijngaarden <D.C.vanWijngaarden@tudelft.nl>
 * This module keeps track of the current state of a rotating wing drone and desired state set by the RC or flightplan. Paramters are being scheduled in each change of a current state and desired state. Functions are defined in this module to call the actual state and desired state and set a desired state.
 */

#include "modules/rot_wing_drone/rotwing_state.h"
#include "firmwares/rotorcraft/guidance/guidance_indi_rot_wing.h"

struct RotwingState rotwing_state;

// Quad state identification
#ifndef ROTWING_MIN_SKEW_ANGLE_QUAD
#define ROTWING_MIN_SKEW_ANGLE_QUAD 10.0
#endif
#ifndef ROTWING_MIN_SKEW_ANGLE_COUNTER
#define ROTWING_MIN_SKEW_ANGLE_COUNTER 10         // Minimum number of loops the skew angle is below ROTWING_MIN_SKEW_ANGLE_COUNTER
#endif

// Half skew state identification
#ifndef ROTWING_HALF_SKEW_ANGLE_DEG
#define ROTWING_HALF_SKEW_ANGLE_DEG 55.0
#endif
#ifndef ROTWING_HALF_SKEW_ANGLE_RANG
#define ROTWING_HALF_SKEW_ANGLE_HALF_RANGE 10.0
#endif
#ifndef ROTWING_HALF_SKEW_COUNTER
#define ROTWING_HALF_SKEW_COUNTER 10              // Minimum number of loops the skew angle is at HALF_SKEW_ANGLE_DEG +/- ROTWING_HALF_SKEW_ANGLE_HALF_RANGE to trigger ROTWING_HALF_SKEW_ANGLE state
#endif

// FW state identification
#ifndef ROTWING_MIN_FW_SKEW_ANGLE_DEG
#define ROTWING_MIN_FW_SKEW_ANGLE_DEG 80.0        // Minimum wing angle to fly in fixed wing state 
#endif
#ifndef ROTWING_MIN_FW_COUNTER
#define ROTWING_MIN_FW_COUNTER 10                 // Minimum number of loops the skew angle is above the MIN_FW_SKEW_ANGLE
#endif

void init_rotwing_state(void)
{
  // Start the drone in a desired hover state
  rotwing_state.current_state = ROTWING_STATE_HOVER;
  rotwing_state.desired_state = ROTWING_STATE_HOVER;
}

void periodic_rotwing_state(void)
{
  // Check the current state
  // States can be checked according to wing angle sensor, setpoints .....
  // check state

  // float skew_angle_deg; //TODO: put actual skew angle in degrees to this float

  // Check difference with desired state



  //TODO: incorparate motor active / disbaling depending on called flight state
  // Switch on motors if flight mode is attitude
  if (guidance_h.mode == GUIDANCE_H_MODE_ATTITUDE) {
    bool_disable_hover_motors = false;
  } else if (guidance_h.mode == GUIDANCE_H_MODE_FORWARD) {
    bool_disable_hover_motors = false;
  }

}

// void rotwing_determine_current_state(float skew_angle_deg)
// {
//   if (skew_angle_deg < ROTWING_MIN_SKEW_ANGLE_QUAD)
//   {
//     // increment counter

//     // set to quad if counter is high enough
//   } else if (skew_angle_deg > ROTWING_MIN_SKEW_ANGLE_QUAD && skew_angle_deg < (ROTWING_HALF_SKEW_ANGLE_DEG - ROTWING_HALF_SKEW_ANGLE_HALF_RANGE) {

//   } else if (skew_angle_deg > (ROTWING_HALF_SKEW_ANGLE_DEG - ROTWING_HALF_SKEW_ANGLE_HALF_RANGE) && skew_angle_deg < (ROTWING_HALF_SKEW_ANGLE_DEG + ROTWING_HALF_SKEW_ANGLE_HALF_RANGE)) {

//   } else {
    
//   }
// }


