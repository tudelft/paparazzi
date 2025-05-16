/*
 * Copyright (C) 2015 Lodewijk Sikkel <l.n.c.sikkel@tudelft.nl>
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

/** @file modules/datalink/missionlib/mission_manager.h
*  @brief Common functions used within the mission library, blocks and
*         waypoints cannot be send simultaneously (which should not
*         matter)
*/

#ifndef MISSIONLIB_COMMON_H
#define MISSIONLIB_COMMON_H

#include <mavlink/mavlink_types.h>
#include "modules/mission/mission_common.h"

// include mavlink headers, but ignore some warnings
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Waddress-of-packed-member"
#pragma GCC diagnostic ignored "-Wswitch-default"
#include "mavlink/ardupilotmega/mavlink.h"
#pragma GCC diagnostic pop

#ifndef MAVLINK_TIMEOUT
#define MAVLINK_TIMEOUT 1.5 // 1500ms as recommended in https://mavlink.io/en/services/mission.html
#endif

#ifndef MAVLINK_MISSION_ITEM_TIMEOUT
#define MAVLINK_MISSION_ITEM_TIMEOUT 0.25 // 250ms as recommended in https://mavlink.io/en/services/mission.html
#endif

#ifndef MAVLINK_MISSION_MAX_RETRIES
#define MAVLINK_MISSION_MAX_RETRIES 5 // Maximum number of retries
#endif

struct mavlink_mission_mgr {
  uint8_t count; // Count of mission elements
  uint8_t mission_state; // The current MISSION_STATE defined by common mavlink
  uint16_t end_index; // Index of final element in a (partial) upload transaction
  uint8_t rem_sysid; // Remote system id
  uint8_t rem_compid; // Remote component id
  int timer_id; // Timer id
  uint8_t nb_retries; // Number of retries to get the current item
  mavlink_mission_item_int_t mission_items[MISSION_ELEMENT_NB]; // The activated mission items
};

void mavlink_mission_init(void);
void mavlink_mission_periodic(void);

void mavlink_mission_message_handler(const mavlink_message_t *msg);

void mavlink_mission_set_timer(float duration);
void mavlink_mission_cancel_timer(void);

bool mavlink_mission_set_active(void);

void mavlink_wp_message_handler(const mavlink_message_t *msg);

bool mavlink_mission_item_from_pprz_mission_element(mavlink_mission_item_int_t *mission_item_int, struct _mission_element *me);
bool pprz_mission_element_from_mavlink_mission_item(struct _mission_element *me, mavlink_mission_item_int_t *mi);

uint16_t first_missing_mission_item(void);
void mavlink_lla_of_global(mavlink_mission_item_int_t *mi, struct LlaCoor_i *lla);
void mavlink_lla_of_global_relative_alt(mavlink_mission_item_int_t *mi, struct LlaCoor_i *lla);
void mavlink_mission_item_set_lla(mavlink_mission_item_int_t *mi, struct LlaCoor_i *lla);

#endif
