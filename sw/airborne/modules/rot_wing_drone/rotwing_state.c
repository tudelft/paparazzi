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
#include "firmwares/rotorcraft/stabilization/stabilization_indi.h"
#include "firmwares/rotorcraft/guidance/guidance_indi_rot_wing.h"
#include "firmwares/rotorcraft/autopilot_firmware.h"
#include "modules/rot_wing_drone/wing_rotation_controller_v3b.h"
#include "modules/actuators/actuators.h"
#include "modules/core/abi.h"

struct RotwingState rotwing_state;

// Quad state identification
#ifndef ROTWING_MIN_SKEW_ANGLE_DEG_QUAD
#define ROTWING_MIN_SKEW_ANGLE_DEG_QUAD 10.0
#endif
#ifndef ROTWING_MIN_SKEW_ANGLE_COUNTER
#define ROTWING_MIN_SKEW_ANGLE_COUNTER 10         // Minimum number of loops the skew angle is below ROTWING_MIN_SKEW_ANGLE_COUNTER
#endif

// Skewing state identification
#ifndef ROTWING_SKEWING_COUNTER
#define ROTWING_SKEWING_COUNTER 10                // Minimum number of loops the skew angle is in between QUAD and FW
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

// FW idle state identification
#ifndef ROTWING_MIN_THRUST_IDLE
#define ROTWING_MIN_THRUST_IDLE 100
#endif
#ifndef ROTWING_MIN_THRUST_IDLE_COUNTER
#define ROTWING_MIN_THRUST_IDLE_COUNTER 10
#endif

// FW hov mot off state identification
#ifndef ROTWING_HOV_MOT_OFF_RPM_TH
#define ROTWING_HOV_MOT_OFF_RPM_TH 50
#endif
#ifndef ROTWING_HOV_MOT_OFF_COUNTER
#define ROTWING_HOV_MOT_OFF_COUNTER 10
#endif

#ifndef ROTWING_STATE_RPM_ID
#define ROTWING_STATE_RPM_ID ABI_BROADCAST
#endif
abi_event rotwing_state_rpm_ev;
static void rotwing_state_rpm_cb(uint8_t sender_id, struct rpm_act_t * rpm_message, uint8_t num_act);
int32_t rotwing_state_hover_rpm[4] = {0, 0, 0, 0};

bool rotwing_state_force_quad = false;

uint8_t rotwing_state_hover_counter = 0;
uint8_t rotwing_state_skewing_counter = 0;
uint8_t rotwing_state_fw_counter = 0;
uint8_t rotwing_state_fw_idle_counter = 0;
uint8_t rotwing_state_fw_m_off_counter = 0;

inline void rotwing_check_set_current_state(void);
inline void rotwing_update_desired_state(void);
inline void rotwing_switch_state(void);

#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"
static void send_rotating_wing_state(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_ROTATING_WING_STATE(trans, dev, AC_ID, 
                                        &rotwing_state.current_state,
                                        &rotwing_state.desired_state);
}
#endif // PERIODIC_TELEMETRY

void init_rotwing_state(void)
{ 
  // Bind ABI messages
  AbiBindMsgRPM(ROTWING_STATE_RPM_ID, &rotwing_state_rpm_ev, rotwing_state_rpm_cb);

  // Start the drone in a desired hover state
  rotwing_state.current_state = ROTWING_STATE_HOVER;
  rotwing_state.desired_state = ROTWING_STATE_HOVER;

  #if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_ROTATING_WING_STATE, send_rotating_wing_state);
  #endif
}

void periodic_rotwing_state(void)
{
  // Check and set the current state
  rotwing_check_set_current_state();

  // Check and update desired state
  rotwing_update_desired_state();

  // Check difference with desired state
  // rotwing_switch_state();

  //TODO: incorparate motor active / disbaling depending on called flight state
  // Switch on motors if flight mode is attitude
  if (guidance_h.mode == GUIDANCE_H_MODE_ATTITUDE) {
    bool_disable_hover_motors = false;
  } else if (guidance_h.mode == GUIDANCE_H_MODE_FORWARD) {
    bool_disable_hover_motors = false;
  }
}

// Function to request a state
void request_rotwing_state(uint8_t state)
{
  if (state == ROTWING_STATE_HOVER) {
    rotwing_state.current_state = ROTWING_STATE_HOVER;
  } else if (state == ROTWING_STATE_FW) {
    rotwing_state.current_state = ROTWING_STATE_FW;
  }
}

void rotwing_check_set_current_state(void)
{
  // if !in_flight, set state to hover
  if (!autopilot.in_flight) {
    rotwing_state_hover_counter = 0;
    rotwing_state_skewing_counter = 0;
    rotwing_state_fw_counter = 0;
    rotwing_state_fw_idle_counter = 0;
    rotwing_state_fw_m_off_counter = 0;
    rotwing_state.current_state = ROTWING_STATE_HOVER;
    return;
  }

  // States can be checked according to wing angle sensor, setpoints .....
  uint8_t prev_state = rotwing_state.current_state;
  switch (prev_state) {
      case ROTWING_STATE_HOVER:
        // Check if state needs to be set to skewing
        if (wing_rotation.wing_angle_deg > ROTWING_MIN_SKEW_ANGLE_DEG_QUAD)
        {
          rotwing_state_skewing_counter++;
        } else {
          rotwing_state_skewing_counter = 0;
        }

        // Switch state if necessary 
        if (rotwing_state_skewing_counter > ROTWING_MIN_SKEW_ANGLE_COUNTER) {
          rotwing_state.current_state = ROTWING_STATE_SKEWING;
          rotwing_state_skewing_counter = 0;
        }
        break;

      case ROTWING_STATE_SKEWING:
        // Check if state needs to be set to hover
        if (wing_rotation.wing_angle_deg < ROTWING_MIN_SKEW_ANGLE_DEG_QUAD)
        {
          rotwing_state_hover_counter++;
        } else {
          rotwing_state_hover_counter = 0;
        }

        // Check if state needs to be set to fixed wing
        if (wing_rotation.wing_angle_deg > ROTWING_MIN_FW_SKEW_ANGLE_DEG)
        {
          rotwing_state_fw_counter++;
        } else {
          rotwing_state_fw_counter = 0;
        }

        // Switch state if necessary
        if (rotwing_state_hover_counter > ROTWING_MIN_SKEW_ANGLE_COUNTER) {
          rotwing_state.current_state = ROTWING_STATE_HOVER;
          rotwing_state_hover_counter = 0;
        }

        if (rotwing_state_fw_counter > ROTWING_MIN_FW_COUNTER) {
          rotwing_state.current_state = ROTWING_STATE_FW;
          rotwing_state_fw_counter = 0;
        }
        break;

      case ROTWING_STATE_FW:
        // Check if state needs to be set to skewing
        if (wing_rotation.wing_angle_deg < ROTWING_MIN_FW_SKEW_ANGLE_DEG) {
          rotwing_state_skewing_counter++;
        } else {
          rotwing_state_skewing_counter = 0;
        }

        // Check if state needs to be set to fixed wing with hover motors idle (If hover thrust below threshold)
        if (stabilization_cmd[COMMAND_THRUST] < ROTWING_MIN_THRUST_IDLE) {
            rotwing_state_fw_idle_counter++;
        } else {
          rotwing_state_fw_idle_counter = 0;
        }

        // Switch state if necessary
        if (rotwing_state_skewing_counter > ROTWING_MIN_FW_COUNTER) {
          rotwing_state.current_state = ROTWING_STATE_SKEWING;
          rotwing_state_skewing_counter = 0;
          rotwing_state_fw_idle_counter = 0;
        } else if (rotwing_state_fw_idle_counter > ROTWING_MIN_THRUST_IDLE_COUNTER 
                    && rotwing_state_skewing_counter == 0) {
          rotwing_state.current_state = ROTWING_STATE_FW_HOV_MOT_IDLE;
          rotwing_state_skewing_counter = 0;
          rotwing_state_fw_idle_counter = 0;
        }
        break;

      case ROTWING_STATE_FW_HOV_MOT_IDLE:
        // Check if state needs to be set to fixed wing with hover motors activated
        if (stabilization_cmd[COMMAND_THRUST] > ROTWING_MIN_THRUST_IDLE) {
          rotwing_state_fw_counter++;
        } else {
          rotwing_state_fw_counter = 0;
        }

        // Check if state needs to be set to fixed wing with hover motors off (if zero rpm on hover motors)
        if (rotwing_state_hover_rpm[0] == 0 
            && rotwing_state_hover_rpm[1] == 0 
            && rotwing_state_hover_rpm[2] == 0
            && rotwing_state_hover_rpm[3] == 0) {
          rotwing_state_fw_m_off_counter++;
        } else {
          rotwing_state_fw_m_off_counter = 0;
        }

        // Switch state if necessary
        if (rotwing_state_fw_counter > ROTWING_MIN_THRUST_IDLE_COUNTER) {
          rotwing_state.current_state = ROTWING_STATE_FW;
          rotwing_state_fw_counter = 0;
          rotwing_state_fw_m_off_counter = 0;
        } else if (rotwing_state_fw_m_off_counter > ROTWING_HOV_MOT_OFF_COUNTER 
                    && rotwing_state_fw_counter == 0) {
          rotwing_state.current_state = ROTWING_STATE_FW_HOV_MOT_OFF;
          rotwing_state_fw_counter = 0;
          rotwing_state_fw_m_off_counter = 0;
        }
        break;

      case ROTWING_STATE_FW_HOV_MOT_OFF:
        // Check if state needs to be set to fixed wing with hover motors idle (if rpm on hover motors)
        if (rotwing_state_hover_rpm[0] > ROTWING_HOV_MOT_OFF_RPM_TH
            && rotwing_state_hover_rpm[1] > ROTWING_HOV_MOT_OFF_RPM_TH 
            && rotwing_state_hover_rpm[2] > ROTWING_HOV_MOT_OFF_RPM_TH
            && rotwing_state_hover_rpm[3] > ROTWING_HOV_MOT_OFF_RPM_TH) {
          rotwing_state_fw_idle_counter++;
        } else {
          rotwing_state_fw_idle_counter = 0;
        }

        // Switch state if necessary
        if (rotwing_state_fw_counter > ROTWING_MIN_THRUST_IDLE_COUNTER) {
          rotwing_state.current_state = ROTWING_STATE_FW_HOV_MOT_IDLE;
          rotwing_state_fw_idle_counter = 0;
        }
        break;

      default:
        break;
  }
}

void rotwing_update_desired_state(void) 
{
  // If force_forward and nav selected, then the desired state if FW with hover motors off;
  switch (guidance_h.mode) {
    case GUIDANCE_H_MODE_NAV:
      if (rotwing_state_force_quad) {
        rotwing_state.desired_state = ROTWING_STATE_HOVER;
      } else if (force_forward) {
        rotwing_state.desired_state = ROTWING_STATE_FW_HOV_MOT_OFF;
      } else {
        rotwing_state.desired_state = ROTWING_STATE_FREE;
      }
      break;

    case GUIDANCE_H_MODE_FORWARD:
      rotwing_state.desired_state = ROTWING_STATE_FW_HOV_MOT_OFF;
      break;

    case GUIDANCE_H_MODE_ATTITUDE:
      rotwing_state.desired_state = ROTWING_STATE_HOVER;
      break;

    default:
      rotwing_state.desired_state = ROTWING_STATE_FREE;
      break;
  }
}

// Function that handles settings for switching state(s)
void rotwing_switch_state(void)
{
  switch (rotwing_state.current_state) {
    case ROTWING_STATE_HOVER:
      if (rotwing_state.desired_state > ROTWING_STATE_HOVER) {
        // if in hover state, but higher state requested, switch on wing_rotation scheduler
        wing_rotation.airspeed_scheduling = true;
      } else {
        // if hover state desired and at the state, fix wing in quad
        wing_rotation.airspeed_scheduling = false;
        wing_rotation.wing_angle_deg_sp = 0;
      }
      break;

    case ROTWING_STATE_SKEWING:
      if (rotwing_state.desired_state > ROTWING_STATE_SKEWING) {
        // Keep wing rotation scheduler on if skewing to a higher state
        wing_rotation.airspeed_scheduling = true;
      } else {
        // go back to quad state if fulfilling conditions for skewing back (motors on, skewing back not too fast).
      }
      break;

    case ROTWING_STATE_FW:
      break;

    case ROTWING_STATE_FW_HOV_MOT_IDLE:
      if (rotwing_state.desired_state > ROTWING_STATE_FW_HOV_MOT_IDLE) {
        // Switch off hover motors if in idle and desired state is higher
        bool_disable_hover_motors = true;
      } else if (rotwing_state.desired_state < ROTWING_STATE_FW_HOV_MOT_IDLE) {
        // Allow hover motors to generate thrust when desired state is lower than current state
        hover_motors_active = true;
      } else {
        // if idle desired and already at idle, let motors spin idle
        bool_disable_hover_motors = false;
        hover_motors_active = false;
      }
      break;

    case ROTWING_STATE_FW_HOV_MOT_OFF:
      if (rotwing_state.desired_state < ROTWING_STATE_FW_HOV_MOT_OFF) {
        // Switch on the hover motors when going to a lower state
        bool_disable_hover_motors = false;
      }
      wing_rotation.airspeed_scheduling = false;
      wing_rotation.wing_angle_deg_sp = 90;
      break; 
  }
}

static void rotwing_state_rpm_cb(uint8_t __attribute__((unused)) sender_id, struct rpm_act_t UNUSED * rpm_message, uint8_t UNUSED num_act)
{
  // Sanity check that index is valid
  if (rpm_message->actuator_idx<num_act){
      if (rpm_message->actuator_idx < 4) {
        rotwing_state_hover_rpm[rpm_message->actuator_idx] = rpm_message->rpm;
      }
    }
}