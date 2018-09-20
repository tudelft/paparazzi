/*
 * Copyright (C) 2018 Freek van Tienen <freek.v.tienen@gmail.com>
 *
 * This file is part of paparazzi.
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
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

#include "mission_path.h"

/** @file modules/mission/mission_path.c
 *  @brief efficient mission path planner
 */

#ifndef MISSION_PATH_MAX
#define MISSION_PATH_MAX 64
#endif

static struct mission_path_elem_t mission_path[MISSION_PATH_MAX];   ///< The mission path elements
static uint8_t mission_path_idx;                                    ///< The current path index
static uint8_t mission_path_last_idx;                               ///< The last mission index

static bool mission_path_add(struct mission_path_elem_t *elem);
static bool mission_path_delete(uint8_t id);
//static void mission_path_reset(void);

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"

static void send_mission_path_status(struct transport_tx *trans, struct link_device *dev)
{
  uint16_t mission_ids[MISSION_PATH_MAX];
  for(uint8_t i = 0; i < mission_path_last_idx; i++)
    mission_ids[i] = mission_path[i].id;
  
  pprz_msg_send_MISSION_PATH_STATUS(trans, dev, AC_ID,
                                &mission_path_idx,
                                mission_path_last_idx,
                                mission_ids);
}

#endif


/**
 * Initialize the mission path module
 */
void mission_path_init(void) {
  mission_path_idx = 0;
  mission_path_last_idx = 0;

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_MISSION_PATH_STATUS, send_mission_path_status);
#endif
}

/**
 * Run from flight plan
 */
bool mission_path_run(void) {
  // Check if we have at least one element
  if(mission_path_last_idx == 0 && mission_path_idx >= mission_path_last_idx)
    return false;
  
  // Goto first waypoint
  if(mission_path_idx == 0) {
    struct EnuCoor_i *target_wp = &mission_path[mission_path_idx].wp;

    // Check if we have reached the target waypoint
    if (nav_approaching_from(target_wp, NULL, CARROT))
      mission_path_idx++;
    
    // Navigate to the target waypoint
    horizontal_mode = HORIZONTAL_MODE_WAYPOINT;
    VECT3_COPY(navigation_target, *target_wp);
    NavVerticalAutoThrottleMode(RadOfDeg(0.000000));
    NavVerticalAltitudeMode(POS_FLOAT_OF_BFP(target_wp->z), 0.);
    return true;
  }
  // Route between previous and current waypoint
  else {
    struct EnuCoor_i *from_wp = &mission_path[mission_path_idx-1].wp;
    struct EnuCoor_i *to_wp = &mission_path[mission_path_idx].wp;

    // Check if we have reached the target waypoint
    if (nav_approaching_from(to_wp, from_wp, CARROT))
      mission_path_idx++;

    // Route Between from-to
    horizontal_mode = HORIZONTAL_MODE_ROUTE;
    nav_route(from_wp, to_wp);
    NavVerticalAutoThrottleMode(RadOfDeg(0.0));
    NavVerticalAltitudeMode(POS_FLOAT_OF_BFP(to_wp->z), 0.);
    return true;
  }
}

/**
 * Add a mission path at a specific index
 */
static bool mission_path_add(struct mission_path_elem_t *elem) {
  uint8_t add_idx = 0;

  // Go through the mission path
  for(; add_idx < mission_path_last_idx; add_idx++) {

    // Check if it needs to be added before this waypoint
    if(elem->id <= mission_path[add_idx].id)
      break;
  }

  // Add the element
  if(mission_path[add_idx].id != elem->id && add_idx < MISSION_PATH_MAX-1) {
    // Update the current index if the point is added before
    if(add_idx < mission_path_idx)
      mission_path_idx++;

    // Add the element itself
    struct mission_path_elem_t saved_elem = mission_path[add_idx];
    mission_path[add_idx++] = *elem;
    mission_path_last_idx++;

    // Move the last elements
    for(; add_idx < mission_path_last_idx; add_idx++){
      struct mission_path_elem_t tmp = mission_path[add_idx];
      mission_path[add_idx] = saved_elem;
      saved_elem = tmp;
    }
  }
  // Change the element
  else if(add_idx < MISSION_PATH_MAX) {
    mission_path[add_idx] = *elem;
  }
  else {
    return false;
  }
  return true;
}

/**
 * Delete a path element
 */
static bool mission_path_delete(uint8_t id) {
  bool deleted = false;
  for(uint8_t i = 0; i < mission_path_last_idx; i++) {
    if(mission_path[i].id == id) {
      deleted = true;
      mission_path_last_idx--;
    }
    if(deleted) { 
      mission_path[i] = mission_path[i+1];
    }
  }

  return deleted;
}

/**
 * Delete all path elements
 */
/*static void mission_path_reset(void) {
  mission_path_idx = 0;
  mission_path_last_idx = 0;
}*/

/**
 * Wen a datalink MISSION_PATH_ADD is received
 */
bool mission_path_parse_ADD(struct link_device *dev, struct transport_tx *trans, uint8_t *buf) {
  if (DL_MISSION_PATH_ADD_ac_id(buf) != AC_ID) { return false; }

  // Add the path
  struct mission_path_elem_t elem;
  elem.id = DL_MISSION_PATH_ADD_id(buf);
  elem.wp.x = DL_MISSION_PATH_ADD_wp_east(buf);
  elem.wp.y = DL_MISSION_PATH_ADD_wp_north(buf);
  elem.wp.z = DL_MISSION_PATH_ADD_wp_up(buf);
  mission_path_add(&elem);

  // Send ACK back
  pprz_msg_send_MISSION_PATH_ADD_ACK(trans, dev, AC_ID, &mission_path_idx, &elem.id);
  return true;
}

/**
 * Wen a datalink MISSION_PATH_DELETE is received
 */
bool mission_path_parse_DELETE(struct link_device *dev, struct transport_tx *trans, uint8_t *buf) {
  if (DL_MISSION_PATH_DELETE_ac_id(buf) != AC_ID) { return false; }

  // Delete the path waypoint
  uint16_t id = DL_MISSION_PATH_DELETE_id(buf);
  mission_path_delete(id);

  // Send ACK back
  pprz_msg_send_MISSION_PATH_DELETE_ACK(trans, dev, AC_ID, &mission_path_idx, &id);
  return true;
}