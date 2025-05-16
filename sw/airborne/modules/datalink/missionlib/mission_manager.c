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

/** @file modules/datalink/missionlib/mission_manager.c
*  @brief Common functions used within the mission library
*/

#include "math/pprz_geodetic_int.h"
#include "state.h"
#include "math.h"
#include "mcu_periph/sys_time.h"
#include "modules/datalink/telemetry.h"
#include "modules/datalink/mavlink.h"
#include "modules/datalink/missionlib/mission_manager.h"

static struct mavlink_mission_mgr mission_mgr = {0};

static void mavlink_send_mission_current(struct transport_tx *trans, struct link_device *dev);

uint16_t first_missing_mission_item(void) {
  for (uint16_t i = 0; i < MISSION_ELEMENT_NB; i++) {
    if (mission_mgr.mission_items[i].seq == UINT16_MAX) {
      return i;
    }
  }

  return MISSION_ELEMENT_NB;
}

void mavlink_lla_of_global(mavlink_mission_item_int_t *mi, struct LlaCoor_i *lla) {
  lla->lat = mi->x; // lattitude in degrees*1e7
  lla->lon = mi->y; // longitude in degrees*1e7
  lla->alt = MM_OF_M(mi->z); // altitude in millimeters
}

void mavlink_lla_of_global_relative_alt(mavlink_mission_item_int_t *mi, struct LlaCoor_i *lla) {
  lla->lat = mi->x; // lattitude in degrees*1e7
  lla->lon = mi->y; // longitude in degrees*1e7
  lla->alt = state.ned_origin_i.hmsl + MM_OF_M(mi->z); // altitude in millimeters
}

void mavlink_mission_item_set_lla(mavlink_mission_item_int_t *mi, struct LlaCoor_i *lla) {
  mi->x = lla->lat;
  mi->y = lla->lon;
  mi->z = (float)M_OF_MM(lla->alt);
}

void mavlink_mission_init(void) {
  mission_mgr.mission_state = MISSION_STATE_NO_MISSION;
  mission_mgr.timer_id = -1;

  for (uint8_t i = 0; i < MISSION_ELEMENT_NB; i++) {
    mission_mgr.mission_items[i].seq = UINT16_MAX;
  }

#if PERIODIC_TELEMETRY && defined TELEMETRY_MAVLINK_NB_MSG
  register_periodic_telemetry(&mavlink_telemetry, MAVLINK_MSG_ID_MISSION_CURRENT, mavlink_send_mission_current);
#endif
}

void mavlink_mission_set_timer(float duration) {
  if (mission_mgr.timer_id < 0) {
    mission_mgr.timer_id = sys_time_register_timer(duration, NULL);
  }
  else {
    sys_time_update_timer(mission_mgr.timer_id, duration);
  }
}

void mavlink_mission_cancel_timer(void) {
  if (mission_mgr.timer_id >= 0) {
    sys_time_cancel_timer(mission_mgr.timer_id);
  }
  mission_mgr.timer_id = -1;
}

void mavlink_mission_message_handler(const mavlink_message_t *msg) {
  // Go through handlers until message is processed
  // mavlink_block_message_handler(msg);
  mavlink_wp_message_handler(msg);

  if (msg->msgid == MAVLINK_MSG_ID_MISSION_ACK) {
    MAVLINK_DEBUG("Received MISSION_ACK message\n");
    mavlink_mission_cancel_timer();
  }
}

// update current block and send if changed
void mavlink_mission_periodic(void) {

  #if !defined (TELEMETRY_MAVLINK_NB_MSG)
  RunOnceEvery(10, mavlink_send_mission_current(NULL, NULL));
  #endif

  // check if we had a timeout on a transaction
  if (sys_time_check_and_ack_timer(mission_mgr.timer_id)) {
    mavlink_mission_cancel_timer();

    // If we have timed out too many times, send an error message
    if (mission_mgr.nb_retries >= MAVLINK_MISSION_MAX_RETRIES) {
      MAVLINK_DEBUG("Error: Mavlink mission request timed out too many times!\n");
      mavlink_msg_mission_ack_send(MAVLINK_COMM_0, mission_mgr.rem_sysid, mission_mgr.rem_compid,
                                  MAV_MISSION_OPERATION_CANCELLED, MAV_MISSION_TYPE_MISSION);
      MAVLinkSendMessage();
      return;
    }

    uint16_t first_missing_item = first_missing_mission_item();
    MAVLINK_DEBUG("Warning: Mavlink mission request timed out at index %u!\n", first_missing_item);

    // Request the first missing item
    mavlink_msg_mission_request_int_send(MAVLINK_COMM_0, mission_mgr.rem_sysid, mission_mgr.rem_compid, first_missing_item, MAV_MISSION_TYPE_MISSION);
    MAVLinkSendMessage();
    mavlink_mission_set_timer(MAVLINK_MISSION_ITEM_TIMEOUT);
    mission_mgr.nb_retries++;
  }
}

bool mavlink_mission_set_active(void) {
  enum MissionInsertMode mode = ReplaceAll;
  struct _mission_element me;
  bool me_valid = false;
  bool me_inserted = false;
  uint8_t current_item = 0;
  for (uint8_t i = 0; i < mission_mgr.count; i++) {
    me_valid = pprz_mission_element_from_mavlink_mission_item(&me, &mission_mgr.mission_items[i]);

    if (!me_valid) {
      MAVLINK_DEBUG("Invalid mission element %i\n", i);
      return false;
    }

    current_item = (mission_mgr.mission_items[i].current == 1)? i:current_item; // Why?
    me_inserted = mission_insert(mode, &me);

    /* Start with replace all then move to append */
    if (i == 0) {
      mode = Append;
    }

    if (!me_inserted) {
      MAVLINK_DEBUG("Failed to insert mission element %i\n", i);
      return false;
    }

#if PERIODIC_TELEMETRY
  DOWNLINK_SEND_MISSION_ITEM(DefaultChannel, DefaultDevice,
                              &mission_mgr.count,
                              &mission_mgr.mission_items[i].seq,
                              &mission_mgr.mission_items[i].command,
                              &mission_mgr.mission_items[i].x,
                              &mission_mgr.mission_items[i].y,
                              &mission_mgr.mission_items[i].z);
#endif
  }
    
  MAVLINK_DEBUG("Mission with count %i activated", mission_mgr.count);
  return true;
}

void mavlink_wp_message_handler(const mavlink_message_t *msg) {  
  
  switch (msg->msgid) {
    
    /* initiate mission/waypoint write transaction */
    case MAVLINK_MSG_ID_MISSION_COUNT: {
      mavlink_mission_count_t mission_count;
      mavlink_msg_mission_count_decode(msg, &mission_count);
      if (mission_count.target_system != mavlink_system.sysid) {
        MAVLINK_DEBUG("MISSION_COUNT FAIL target id not equal to sysid\n");
        return;
      }
      MAVLINK_DEBUG("Received MISSION_COUNT message with count %i\n", mission_count.count);
      
      if (mission_count.count > MISSION_ELEMENT_NB) {
        MAVLINK_DEBUG("MISSION_COUNT error: request writing %i waypoints while %i waypoints is maximum\n",
                      mission_count.count, MISSION_ELEMENT_NB);
        return;
      }
      
      /* If fence */
      if (mission_count.mission_type == MAV_MISSION_TYPE_FENCE) {
        mavlink_msg_mission_ack_send(MAVLINK_COMM_0, msg->sysid, msg->compid,
          MAV_MISSION_UNSUPPORTED, MAV_MISSION_TYPE_MISSION);
        MAVLinkSendMessage();
      }
      
      /* If rally */
      if (mission_count.mission_type == MAV_MISSION_TYPE_RALLY) {
        mavlink_msg_mission_ack_send(MAVLINK_COMM_0, msg->sysid, msg->compid,
          MAV_MISSION_UNSUPPORTED, MAV_MISSION_TYPE_MISSION);
        MAVLinkSendMessage();
        return;
      }

      /* Store the remote sender ID in case of time outs */
      mission_mgr.rem_sysid = mission_count.target_system; 
      mission_mgr.rem_compid = mission_count.target_component; 

      /* Reset the mission items */
      for (uint8_t i = 0; i < mission_mgr.count; i++) {
        mission_mgr.mission_items[i].seq = UINT16_MAX;
      }
      
      /* valid initiation of waypoint write transaction, ask for first waypoint */
      mission_mgr.count = mission_count.count;
      MAVLINK_DEBUG("MISSION_COUNT: Requesting first waypoint\n");
      mavlink_msg_mission_request_int_send(MAVLINK_COMM_0, msg->sysid, msg->compid, 0, MAV_MISSION_TYPE_MISSION);
      MAVLinkSendMessage();

      // Register the timeout timer
      mavlink_mission_set_timer(MAVLINK_MISSION_ITEM_TIMEOUT);
      break;
    }

    /* got MISSION_ITEM, update one waypoint if in valid transaction */
    case MAVLINK_MSG_ID_MISSION_ITEM_INT: {
      mavlink_mission_item_int_t mission_item_int;
      mavlink_msg_mission_item_int_decode(msg, &mission_item_int);

      if (mission_item_int.target_system != mavlink_system.sysid) {
        return;
      }

      MAVLINK_DEBUG("Received MISSION_ITEM_INT message with seq %i and frame %i\n",
                    mission_item_int.seq, mission_item_int.frame);

      /* reject if seq number too high */
      if (mission_item_int.seq >= MISSION_ELEMENT_NB ||
          mission_item_int.seq >= mission_mgr.count) {
        MAVLINK_DEBUG("rejected MISSION_ITEM_INT command %i, seq %i because seq number is too high\n",
                      mission_item_int.command, mission_item_int.seq);
        mavlink_msg_mission_ack_send(MAVLINK_COMM_0, msg->sysid, msg->compid,
                                    MAV_MISSION_NO_SPACE, MAV_MISSION_TYPE_MISSION);
        MAVLinkSendMessage();
        mavlink_mission_cancel_timer();
        return;
      }

      /* reject if non supported command */
      if (mission_item_int.command != MAV_CMD_NAV_WAYPOINT &&
        mission_item_int.command != MAV_CMD_NAV_TAKEOFF &&
        mission_item_int.command != MAV_CMD_NAV_VTOL_TAKEOFF &&
        mission_item_int.command != MAV_CMD_NAV_LAND &&
        mission_item_int.command != MAV_CMD_DO_CHANGE_SPEED) {
        MAVLINK_DEBUG("rejected MISSION_ITEM_INT command %i, seq %i due to unsupported command\n",
                      mission_item_int.command, mission_item_int.seq);
        mavlink_msg_mission_ack_send(MAVLINK_COMM_0, msg->sysid, msg->compid,
                                    MAV_MISSION_UNSUPPORTED, MAV_MISSION_TYPE_MISSION);
        MAVLinkSendMessage();
        mavlink_mission_cancel_timer();
        return;
      }

      /* reject if non supported frame */
      if (mission_item_int.frame != MAV_FRAME_GLOBAL &&
        mission_item_int.frame != MAV_FRAME_GLOBAL_RELATIVE_ALT && 
        mission_item_int.frame != MAV_FRAME_MISSION) {
        MAVLINK_DEBUG("rejected MISSION_ITEM_INT frame %i, seq %i due to unsupported frame\n",
                      mission_item_int.frame, mission_item_int.seq);
        mavlink_msg_mission_ack_send(MAVLINK_COMM_0, msg->sysid, msg->compid,
                                    MAV_MISSION_INVALID_SEQUENCE, MAV_MISSION_TYPE_MISSION);
        MAVLinkSendMessage();
        mavlink_mission_cancel_timer();
        return;
      }

      // get next item in sequence
      uint16_t next_item = first_missing_mission_item();

      /* If sequence wrong in write transaction, ignore wp */
      if (mission_item_int.seq != next_item) {
        MAVLINK_DEBUG("MISSION_ITEM_INT, got waypoint seq %i, but requested %i\n",
          mission_item_int.seq, next_item);
        // Let the timer run out
        return;
      }
      
      /* Write item to mission_mgr */
      mission_mgr.mission_items[next_item] = mission_item_int;
      mission_mgr.nb_retries = 0;

      // End of transaction
      if (mission_item_int.seq == mission_mgr.count-1) {
        // Check validity of mission
        MAVLINK_DEBUG("Acknowledging end of waypoint write transaction\n");
        mavlink_msg_mission_ack_send(MAVLINK_COMM_0, msg->sysid, msg->compid,
                                    MAV_MISSION_ACCEPTED, MAV_MISSION_TYPE_MISSION);
        MAVLinkSendMessage();
        mavlink_mission_cancel_timer();
        
        bool success = mavlink_mission_set_active();
        
        if (!success) {
          MAVLINK_DEBUG("Failed to set mission active\n");
          mavlink_msg_mission_ack_send(MAVLINK_COMM_0, msg->sysid, msg->compid,
                                      MAV_MISSION_ERROR, MAV_MISSION_TYPE_MISSION);
          MAVLinkSendMessage();
        }
        return;
      }
      
      // Request next waypoint if still in middle of transaction
      MAVLINK_DEBUG("Requesting waypoint %i\n", mission_item_int.seq + 1);
      mavlink_msg_mission_request_int_send(MAVLINK_COMM_0, msg->sysid, msg->compid,
                                        mission_item_int.seq + 1, MAV_MISSION_TYPE_MISSION);
      MAVLinkSendMessage();
      mavlink_mission_set_timer(MAVLINK_MISSION_ITEM_TIMEOUT);
      break;
    }

    /* request for mission list, answer with number of waypoints */
    case MAVLINK_MSG_ID_MISSION_REQUEST_LIST: {
      break;
    }

    /* request for mission item, answer with waypoint */
    case MAVLINK_MSG_ID_MISSION_REQUEST_INT: {
      break;
    }

    /* initiate partial mission/waypoint write transaction */
    case MAVLINK_MSG_ID_MISSION_WRITE_PARTIAL_LIST: {
      break;
    }

    default:
      break;

  }
}

bool mavlink_mission_item_from_pprz_mission_element(mavlink_mission_item_int_t *mi, struct _mission_element *me) {
  /* Should always be the case but necessary for conversions */
  if (!state.ned_initialized_i) {
    return false;
  }

  mi->target_system = mission_mgr.rem_sysid;
  mi->target_component = mission_mgr.rem_compid;

  mi->autocontinue = 1;
  mi->seq = me->index;
  mi->frame = MAV_FRAME_GLOBAL;
  mi->mission_type = MAV_MISSION_TYPE_MISSION;
  mi->current = (me->index == mission.current_idx)? 1:0;

  switch (me->type) {
    case MissionWP:
      mi->command = MAV_CMD_NAV_WAYPOINT;
      mi->param1 = 0; // hold time
      mi->param2 = ARRIVED_AT_WAYPOINT; // acceptance radius
      mi->param3 = 0; // pass radius
      mi->param4 = NAN; // yaw, NaN to use the current system yaw heading mode
      
      struct EnuCoor_i wp_enu_i;
      struct LlaCoor_i lla_i;
      ENU_BFP_OF_REAL(wp_enu_i, me->element.mission_wp.wp);
      lla_of_enu_pos_i(&lla_i, &state.ned_origin_i, &wp_enu_i);
      mavlink_mission_item_set_lla(mi, &lla_i);
      break;

    case MissionCircle:
      mi->command = (me->duration < 0.)? MAV_CMD_NAV_LOITER_UNLIM : MAV_CMD_NAV_LOITER_TIME;
      mi->param1 = (me->duration < 0.)? 0 : me->duration;
      mi->param2 = 0; // Leave circle only once heading is towards the next waypoint (1), or once duration is over (0)
      mi->param3 = me->element.mission_circle.radius; // Leave circle only once heading is towards the next waypoint
      mi->param4 = NAN; // Loiter circle exit location and path to the next waypoint
      
      struct EnuCoor_i circ_center_enu_i;
      ENU_BFP_OF_REAL(circ_center_enu_i, me->element.mission_circle.center);
      lla_of_enu_pos_i(&lla_i, &state.ned_origin_i, &circ_center_enu_i);
      mavlink_mission_item_set_lla(mi, &lla_i);
      break;

    case MissionCustom:

      /* Take-off custom mission */
      if (me->element.mission_custom.reg == mission_get_registered("TO")) {
        #define VTOL_TRANSITION_HEADING_ANY 4 // move to appropriate place, make airframe define?
        mi->command = MAV_CMD_NAV_VTOL_TAKEOFF; // for now only support VTOL take-off
        mi->param2 = VTOL_TRANSITION_HEADING_ANY;
        mi->param4 = NAN; // Use current system yaw heading mode
        // param 1 and 3 unused 
        
        struct LlaCoor_i* cur_pos_lla = stateGetPositionLla_i();
        mavlink_mission_item_set_lla(mi, cur_pos_lla);
        break;
      }
      /* Land custom mission */
      else if (me->element.mission_custom.reg == mission_get_registered("LAND")) {
        #define PRECISION_LAND_MODE_OPPORTUNISTIC 1 // move to appropriate place, make airframe define?
        mi->command = MAV_CMD_NAV_LAND;
        mi->param1 = 0; // System default abort altitude
        mi->param2 = PRECISION_LAND_MODE_OPPORTUNISTIC; // System default landing type
        mi->param4 = NAN; // Use current system yaw heading mode
        // param 3 unused
        break;
      } 
      /* Speed change custom mission */
      else if (me->element.mission_custom.reg == mission_get_registered("SPD")) {
        return false; // TODO: implement
      }
      return false;
    case MissionSegment:
      return false;
    case MissionPath:
      return false;
    default:
      return false;
  }
  return true;
}

bool pprz_mission_element_from_mavlink_mission_item(struct _mission_element *me, mavlink_mission_item_int_t *mi) {
  switch (mi->command) {
    case MAV_CMD_NAV_WAYPOINT: {
      me->type = MissionWP;
      struct LlaCoor_i lla;
      if (mi->frame == MAV_FRAME_GLOBAL) {
        MAVLINK_DEBUG("MISSION_ITEM_INT, global wp: lat=%i, lon=%i, alt=%f\n",
                        mi->x, mi->y, mi->z);
        MAVLINK_DEBUG("Current set: %i\n", mi->current);
        mavlink_lla_of_global(mi, &lla);
        if (!mission_point_of_lla(&me->element.mission_wp.wp, &lla)) {return false;}
      } 
      else if (mi->frame == MAV_FRAME_GLOBAL_RELATIVE_ALT) {
        MAVLINK_DEBUG("MISSION_ITEM_INT, global_rel_alt wp: lat=%i, lon=%i, relative alt=%f\n",
          mi->x, mi->y, mi->z);
        mavlink_lla_of_global_relative_alt(mi, &lla);

        // if there is no valid local coordinate, do not insert mission element
        if (!mission_point_of_lla(&me->element.mission_wp.wp, &lla)) {return false;}

      } 
      else {
        MAVLINK_DEBUG("No handler for MISSION_ITEM_INT with frame %i\n", mi->frame);
        return false;
      }

      me->duration = -1;
      me->index = mi->seq;
      break;
    }
    case MAV_CMD_NAV_TAKEOFF: {
      me->type = MissionCustom;
      me->element.mission_custom.reg = mission_get_registered("TO");

      if (me->element.mission_custom.reg == NULL) {
        MAVLINK_DEBUG("NO TO Custom mission item defined\n");
        return false;
      }

      me->duration = -1;
      me->index = mi->seq;
      break;
    }
    case MAV_CMD_NAV_VTOL_TAKEOFF: {
      MAVLINK_DEBUG("MISSION_ITEM_INT MAV_CMD_NAV_VTOL_TAKEOFF received\n");

      me->type = MissionCustom;
      me->element.mission_custom.reg = mission_get_registered("TO");
      if (me->element.mission_custom.reg == NULL) {
        MAVLINK_DEBUG("Warning: No TO Custom mission item defined\n");
        return false; 
      }
      me->duration = -1;
      me->index = mi->seq;
      break;
    }
    case MAV_CMD_NAV_LAND: {
      me->type = MissionCustom;
      me->element.mission_custom.reg = mission_get_registered("LAND");
      if (me->element.mission_custom.reg == NULL) {
        MAVLINK_DEBUG("Warning: No LAND Custom mission item defined\n");
        return false; 
      }
      me->duration = -1;
      me->index = mi->seq;
      break;
    }
    case MAV_CMD_DO_CHANGE_SPEED:{
      MAVLINK_DEBUG("MAV_CMD_DO_CHANGE_SPEED received, registering custom mission SPD\n");
      me->type = MissionCustom;
      me->element.mission_custom.reg = mission_get_registered("SPD");

      if (me->element.mission_custom.reg == NULL) {
        MAVLINK_DEBUG("Warning: No SPD Custom mission item defined\n");
        return false; 
      }

      me->duration = -1;
      me->index = mi->seq;
      break;
    }
    case MAV_CMD_NAV_LOITER_UNLIM: {
      MAVLINK_DEBUG("MAV_CMD_NAV_LOITER_UNLIM is currently unsupported\n");
      return false;
    }
    case MAV_CMD_NAV_LOITER_TIME: {
      MAVLINK_DEBUG("MAV_CMD_NAV_LOITER_TIME is currently unsupported\n");
      return false;
    }
    case MAV_CMD_NAV_RETURN_TO_LAUNCH: {
      MAVLINK_DEBUG("MAV_CMD_NAV_RETURN_TO_LAUNCH is currently unsupported\n");
      return false;
    }
    default:
      MAVLINK_DEBUG("MAVLINK_CMD not supported\n");
      return false;
  }

  return true;
}

/* ignore the unused-parameter warnings */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"

static void mavlink_send_mission_current(struct transport_tx *trans, struct link_device *dev) {
  struct _mission_element *mission_item = mission_get();
  static uint8_t mission_mode = 0; // TODO: Support mission modes
  uint8_t count = mission_get_nb_elements();
  
  if(mission_item != NULL) {
    mavlink_msg_mission_current_send(MAVLINK_COMM_0, mission_item->index, count, mission_mgr.mission_state, mission_mode);
    MAVLinkSendMessage();
  }
}

#pragma GCC diagnostic pop