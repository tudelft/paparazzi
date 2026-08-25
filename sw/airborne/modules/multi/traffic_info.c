/*
 * Copyright (C) Pascal Brisset, Antoine Drouin (2008), 
 *               Kirk Scheper (2016), OpenUAS (2026)
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
/**
 * @file modules/multi/traffic_info.c
 * @brief Maintains traffic-aircraft state from legacy traffic messages.
 * @author Pascal Brisset
 * @author Antoine Drouin
 * @author Kirk Scheper
 * @author OpenUAS
 * 
 * This translation unit owns the shared traffic table and legacy message
 * paths. Optional MESH_STATE transport and policy live in traffic_info_mesh.c.
 */

#include "modules/multi/traffic_info.h"
#include "modules/multi/traffic_info_internal.h"
#include "modules/multi/traffic_info_policy.h"

#include "generated/airframe.h"     // AC_ID
#include "generated/flight_plan.h"  // NAV_MSL0

#include "modules/datalink/datalink.h"
#include "modules/datalink/telemetry.h"
#include "pprzlink/dl_protocol.h"   // datalink messages
#include "pprzlink/messages.h"     // telemetry messages

#include "state.h"
#include "math/pprz_geodetic_utm.h"
#include "math/pprz_geodetic_wgs84.h"
#include <math.h>

#if AC_ID < 1 || AC_ID > TRAFFIC_INFO_MAX_AC_ID
#error "Airborne AC_ID must be in 1 through 254; 0 is GCS and 255 is broadcast"
#endif

#if TRAFFIC_INFO_USE_LOG
#include "modules/loggers/logger_utils.h"
#if !USE_CHIBIOS_RTOS
static FILE* pprzLogFile = NULL;

/* Set the default log path to bebop storage */
#ifndef TRAFFIC_INFO_FILE_PATH
#define TRAFFIC_INFO_FILE_PATH /data/ftp/internal_000/acinfo
#endif

#endif // !USE_CHIBIOS_RTOS
#endif // TRAFFIC_INFO_USE_LOG

/* Number of occupied entries in ti_acs, including GCS and this aircraft. */
uint8_t ti_acs_idx;
/* AC_ID-to-ti_acs index map; zero means unknown except for GCS AC_ID 0. */
uint8_t ti_acs_id[NB_ACS_ID];
/* Fixed-capacity state table shared by legacy and optional mesh messages. */
struct acInfo ti_acs[NB_ACS];
bool traffic_info_capacity_exceeded;
bool traffic_info_surveillance_established;
static uint64_t traffic_position_received_ms[NB_ACS];
static uint64_t traffic_velocity_received_ms[NB_ACS];
static uint64_t traffic_slot_activity_ms[NB_ACS];
static bool traffic_has_position_observation[NB_ACS];
static bool traffic_has_velocity_observation[NB_ACS];
static uint32_t traffic_source_itow[NB_ACS];
static bool traffic_has_source_itow[NB_ACS];
static uint64_t traffic_source_itow_started_ms[NB_ACS];
static bool traffic_equal_itow_episode_active[NB_ACS];
enum traffic_source_position_kind {
  TRAFFIC_SOURCE_POSITION_NONE,
  TRAFFIC_SOURCE_POSITION_UTM,
  TRAFFIC_SOURCE_POSITION_LLA
};
struct traffic_source_observation {
  enum traffic_source_position_kind kind;
  int32_t position_1;
  int32_t position_2;
  int32_t altitude;
  int16_t course;
  uint16_t gspeed;
  int16_t climb;
  uint8_t utm_zone;
};
static struct traffic_source_observation traffic_source_observation[NB_ACS];
static uint64_t traffic_monotonic_epoch_ms;
static uint32_t traffic_monotonic_last_ms;

/** Extend the wrapping 32-bit system clock into a monotonic 64-bit timestamp. */
uint64_t traffic_monotonic_time_ms(void)
{
  const uint32_t now_ms = get_sys_time_msec();
  if (now_ms < traffic_monotonic_last_ms) {
    traffic_monotonic_epoch_ms += (UINT64_C(1) << 32);
  }
  traffic_monotonic_last_ms = now_ms;
  return traffic_monotonic_epoch_ms + now_ms;
}

static bool traffic_source_observation_changed(
  uint8_t slot, const struct traffic_source_observation *observation)
{
  const struct traffic_source_observation *stored =
    &traffic_source_observation[slot];
  return observation->kind != stored->kind
         || observation->position_1 != stored->position_1
         || observation->position_2 != stored->position_2
         || observation->altitude != stored->altitude
         || observation->course != stored->course
         || observation->gspeed != stored->gspeed
         || observation->climb != stored->climb
         || observation->utm_zone != stored->utm_zone;
}

void WEAK traffic_info_slot_reassigned(uint8_t slot __attribute__((unused)))
{
}

static void traffic_info_reset_slot(uint8_t slot, uint8_t id)
{
  const uint8_t old_id = ti_acs[slot].ac_id;
  /* Notify slot-indexed consumers before clearing the old identity. TCAS uses
   * it to remove any advisory that would otherwise attach to the new owner. */
  traffic_info_slot_reassigned(slot);
  if (traffic_info_id_valid(old_id) && ti_acs_id[old_id] == slot) {
    ti_acs_id[old_id] = 0;
  }
  memset(&ti_acs[slot], 0, sizeof(ti_acs[slot]));
  traffic_position_received_ms[slot] = 0;
  traffic_velocity_received_ms[slot] = 0;
  traffic_slot_activity_ms[slot] = 0;
  traffic_has_position_observation[slot] = false;
  traffic_has_velocity_observation[slot] = false;
  traffic_source_itow[slot] = 0;
  traffic_has_source_itow[slot] = false;
  traffic_source_itow_started_ms[slot] = 0;
  traffic_equal_itow_episode_active[slot] = false;
  memset(&traffic_source_observation[slot], 0,
         sizeof(traffic_source_observation[slot]));
#if TRAFFIC_INFO_USE_MESH
  traffic_info_mesh_reset_slot(slot);
#endif
  ti_acs[slot].ac_id = id;
  ti_acs_id[id] = slot;
}

uint8_t ti_acs_slot(uint8_t id)
{
  if (!traffic_info_id_valid(id)) {
    return TI_ACS_NONE;
  }
  uint8_t slot = ti_acs_id[id];
  if (slot != 0 || id == TRAFFIC_INFO_GCS_ID) {
    return slot;
  }
  if (ti_acs_idx < NB_ACS) {
    slot = ti_acs_idx++;
    traffic_info_reset_slot(slot, id);
    return slot;
  }

  const uint64_t now_ms = traffic_monotonic_time_ms();
  uint64_t oldest_activity_ms = UINT64_MAX;
  uint8_t oldest_slot = TI_ACS_NONE;
  for (uint8_t candidate = 2; candidate < ti_acs_idx; candidate++) {
    const uint64_t activity_ms = traffic_slot_activity_ms[candidate];
    if (traffic_info_reclaim_candidate_preferred(
          now_ms, activity_ms, TRAFFIC_INFO_RECLAIM_MS,
          oldest_slot != TI_ACS_NONE, oldest_activity_ms)) {
      oldest_activity_ms = activity_ms;
      oldest_slot = candidate;
    }
  }
  if (oldest_slot == TI_ACS_NONE) {
    traffic_info_capacity_exceeded = true;
    return TI_ACS_NONE;
  }
  traffic_info_reset_slot(oldest_slot, id);
  traffic_info_capacity_exceeded = false;
  return oldest_slot;
}

/* Geoid height (msl) over ellipsoid [mm] */
int32_t geoid_height;

/* Send ACINFO_LLA message from the datalink class */
static void send_acinfo_lla(struct transport_tx *trans, struct link_device *dev)
{
  int16_t course = (int16_t)DeciDegOfRad(stateGetHorizontalSpeedDir_f());
  struct LlaCoor_i* lla = stateGetPositionLla_i();
  uint32_t itow = gps.tow;
  uint16_t speed = (uint16_t)(stateGetHorizontalSpeedNorm_f() * 100.f);
  int16_t climb = (int16_t)(stateGetSpeedEnu_f()->z * 100.f);
  int32_t alt = (int32_t)(lla->alt/10);
  uint8_t ac_id = AC_ID;

  // broadcast GPS message
  struct pprzlink_msg msg;
  msg.trans = trans;
  msg.dev = dev;
  msg.sender_id = AC_ID;
  msg.receiver_id = PPRZLINK_MSG_BROADCAST;
  msg.component_id = 0;
  pprzlink_msg_send_ACINFO_LLA(&msg,
      &course,
      &lla->lat, &lla->lon, &alt,
      &itow, &speed, &climb, &ac_id);

#if TRAFFIC_INFO_USE_LOG
  struct LlaCoor_f* lla_f = stateGetPositionLla_f();
  struct EnuCoor_f* enu_f = stateGetPositionEnu_f();
  if (LogFileIsOpen(pprzLogFile)) {
    LogWrite(pprzLogFile, "S,%d,%d,%.7f,%.7f,%.3f,%.3f,%.3f,%.3f,%d,0\n", // 0 at the end to have same length than receive log lines
        msg.sender_id, msg.receiver_id,
        DegOfRad(lla_f->lat), DegOfRad(lla_f->lon), lla_f->alt,
        enu_f->x, enu_f->y, enu_f->z,
        itow);
  }
#endif
}

void traffic_info_init(void)
{
  memset(ti_acs_id, 0, NB_ACS_ID);
  memset(traffic_position_received_ms, 0, sizeof(traffic_position_received_ms));
  memset(traffic_velocity_received_ms, 0, sizeof(traffic_velocity_received_ms));
  memset(traffic_slot_activity_ms, 0, sizeof(traffic_slot_activity_ms));
  memset(traffic_has_position_observation, 0, sizeof(traffic_has_position_observation));
  memset(traffic_has_velocity_observation, 0, sizeof(traffic_has_velocity_observation));
  memset(traffic_source_itow, 0, sizeof(traffic_source_itow));
  memset(traffic_has_source_itow, 0, sizeof(traffic_has_source_itow));
  memset(traffic_source_itow_started_ms, 0, sizeof(traffic_source_itow_started_ms));
    memset(traffic_equal_itow_episode_active, 0,
      sizeof(traffic_equal_itow_episode_active));
  memset(traffic_source_observation, 0, sizeof(traffic_source_observation));
  traffic_info_capacity_exceeded = false;
  traffic_info_surveillance_established = false;

  ti_acs_id[TRAFFIC_INFO_GCS_ID] = 0;
  ti_acs_id[AC_ID] = 1;
  ti_acs[ti_acs_id[AC_ID]].ac_id = AC_ID;
  ti_acs_idx = 2;

  geoid_height = NAV_MSL0;

#if TRAFFIC_INFO_USE_MESH
  traffic_info_mesh_init();
#endif

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_ACINFO_LLA, send_acinfo_lla);
#if TRAFFIC_INFO_USE_MESH
  traffic_info_mesh_register_telemetry();
#endif
#endif
}

/**
 * Update estimate of the geoid height
 * Requires an available hsml and/or lla measurement, if not available value isn't updated
 */
static void update_geoid_height(void) {
  if(bit_is_set(gps.valid_fields, GPS_VALID_HMSL_BIT) && bit_is_set(gps.valid_fields, GPS_VALID_POS_LLA_BIT))
  {
    geoid_height = gps.lla_pos.alt - gps.hmsl;
  } else if(bit_is_set(gps.valid_fields, GPS_VALID_POS_LLA_BIT))
  {
    geoid_height = wgs84_ellipsoid_to_geoid_i(gps.lla_pos.lat, gps.lla_pos.lon);
  } /* default is just keep last available height */
}

bool parse_acinfo_dl(uint8_t *buf)
{
  uint8_t sender_id = SenderIdOfPprzMsg(buf);
  uint8_t msg_id = IdOfPprzMsg(buf);
  uint8_t class_id = pprzlink_get_msg_class_id(buf);
  uint32_t itow = 0;

  /* handle telemetry message */
#if PPRZLINK_DEFAULT_VER == 2
  if (class_id == DL_telemetry_CLASS_ID) {
#else
  if (sender_id > 0) {
#endif
    if (!traffic_info_id_valid(sender_id)) {
      return FALSE;
    }
    switch (msg_id) {
#if TRAFFIC_INFO_USE_MESH
      case DL_ALIVE:
        return traffic_info_mesh_parse_telemetry(sender_id, msg_id);
#endif
      case DL_GPS_SMALL: {
        uint32_t multiplex_speed = DL_GPS_SMALL_multiplex_speed(buf);
        int32_t altitude_mm;
        if (!traffic_info_cm_to_mm(DL_GPS_SMALL_alt(buf), &altitude_mm)) {
          return FALSE;
        }

        // decode compressed values
        int16_t course = (int16_t)((multiplex_speed >> 21) & 0x7FF); // bits 31-21 course in decideg
        if (course & 0x400) {
          course |= 0xF800;  // fix for twos complements
        }
        course *= 2; // scale course by resolution
        uint16_t gspeed = (uint16_t)((multiplex_speed >> 10) & 0x7FF); // bits 20-10 ground speed cm/s
        int16_t climb = (int16_t)(multiplex_speed & 0x3FF); // bits 9-0 z climb speed in cm/s
        if (climb & 0x200) {
          climb |= 0xFC00;  // fix for twos complements
        }
        set_ac_info_lla_arrival(sender_id,
              DL_GPS_SMALL_lat(buf),
              DL_GPS_SMALL_lon(buf),
            altitude_mm,
              course,
              gspeed,
              climb);
      }
      break;
      case DL_GPS: {
        itow = DL_GPS_itow(buf);
        set_ac_info_utm(sender_id,
                    DL_GPS_utm_east(buf),
                    DL_GPS_utm_north(buf),
                    DL_GPS_alt(buf),
                    DL_GPS_utm_zone(buf),
                    DL_GPS_course(buf),
                    DL_GPS_speed(buf),
                    DL_GPS_climb(buf),
                    itow);
      }
      break;
      case DL_GPS_LLA: {
        itow = DL_GPS_LLA_itow(buf);
        set_ac_info_lla(sender_id,
                        DL_GPS_LLA_lat(buf),
                        DL_GPS_LLA_lon(buf),
                        DL_GPS_LLA_alt(buf),
                        DL_GPS_LLA_course(buf),
                        DL_GPS_LLA_speed(buf),
                        DL_GPS_LLA_climb(buf),
                        itow);
      }
      break;
      case DL_GPS_INT: {
        struct LtpDef_i def = {0};
        struct EcefCoor_i ecef_pos = {
          .x = DL_GPS_INT_ecef_x(buf),
          .y = DL_GPS_INT_ecef_y(buf),
          .z = DL_GPS_INT_ecef_z(buf),
        };
        struct EcefCoor_i ecef_vel = {
          .x = DL_GPS_INT_ecef_xd(buf),
          .y = DL_GPS_INT_ecef_yd(buf),
          .z = DL_GPS_INT_ecef_zd(buf),
        };
        struct NedCoor_i ned_vel;
        ltp_def_from_ecef_i(&def, &ecef_pos);
        ned_of_ecef_vect_i(&ned_vel, &def, &ecef_vel);
        struct NedCoor_f ned_vel_f;
        VECT3_FLOAT_OF_CM(ned_vel_f, ned_vel);
        int16_t course = (int16_t)(10.f * DegOfRad(atan2f(ned_vel_f.y, ned_vel_f.x))); // decideg
        uint16_t gspeed = (uint16_t)(CM_OF_M(FLOAT_VECT2_NORM(ned_vel_f)));
        int16_t climb = (int16_t)(CM_OF_M(-ned_vel_f.z));
        itow = DL_GPS_INT_tow(buf);
        set_ac_info_lla(sender_id,
                        DL_GPS_INT_lat(buf),
                        DL_GPS_INT_lon(buf),
                        DL_GPS_INT_alt(buf),
                        course,
                        gspeed,
                        climb,
                        itow);
      }
      break;
      default:
        return FALSE;
    }
  /* handle datalink message */
  } else if (class_id == DL_datalink_CLASS_ID) {
    switch (msg_id) {
#if TRAFFIC_INFO_USE_MESH
      case DL_MESH_STATE: {
        const enum traffic_info_mesh_parse_result result =
          traffic_info_mesh_parse_state(buf, sender_id, &itow);
        if (result == TRAFFIC_INFO_MESH_PARSE_REJECTED) {
          return FALSE;
        }
        if (result == TRAFFIC_INFO_MESH_PARSE_HANDLED) {
          return TRUE;
        }
      }
      break;
    #endif
      case DL_ACINFO: {
        sender_id = DL_ACINFO_ac_id(buf); // may overwrite GCS id
        if (!traffic_info_id_valid(sender_id)) {
          return FALSE;
        }
        int32_t altitude_mm;
        if (!traffic_info_cm_to_mm(DL_ACINFO_alt(buf), &altitude_mm)) {
          return FALSE;
        }
        itow = DL_ACINFO_itow(buf);
        set_ac_info_utm(sender_id,
                        DL_ACINFO_utm_east(buf),
                        DL_ACINFO_utm_north(buf),
                        altitude_mm,
                        DL_ACINFO_utm_zone(buf),
                        DL_ACINFO_course(buf),
                        DL_ACINFO_speed(buf),
                        DL_ACINFO_climb(buf),
                        itow);
      }
      break;
      case DL_ACINFO_LLA: {
        sender_id = DL_ACINFO_LLA_ac_id(buf); // may overwrite GCS id
        if (!traffic_info_id_valid(sender_id)) {
          return FALSE;
        }
        int32_t altitude_mm;
        if (!traffic_info_cm_to_mm(DL_ACINFO_LLA_alt(buf), &altitude_mm)) {
          return FALSE;
        }
        itow = DL_ACINFO_LLA_itow(buf);
        set_ac_info_lla(sender_id,
                  DL_ACINFO_LLA_lat(buf),
                  DL_ACINFO_LLA_lon(buf),
                  altitude_mm,
                  DL_ACINFO_LLA_course(buf),
                  DL_ACINFO_LLA_speed(buf),
                  DL_ACINFO_LLA_climb(buf),
                  itow);
      }
      break;
      default:
        return FALSE;
    }
  } else {
    return FALSE; // unsupported class
  }
#if TRAFFIC_INFO_USE_LOG
  struct LlaCoor_f* lla_f = acInfoGetPositionLla_f(sender_id);
  struct EnuCoor_f* enu_f = acInfoGetPositionEnu_f(sender_id);
  if (lla_f != NULL && enu_f != NULL && LogFileIsOpen(pprzLogFile)) {
    LogWrite(pprzLogFile, "R,%d,%d,%.7f,%.7f,%.3f,%.3f,%.3f,%.3f,%d,%d\n",
        sender_id, AC_ID,
        DegOfRad(lla_f->lat), DegOfRad(lla_f->lon), lla_f->alt,
        enu_f->x, enu_f->y, enu_f->z,
        itow, gps.tow);
  }
#endif
  return TRUE;
}

bool traffic_info_get_age(uint8_t ac_id, uint32_t *age_ms)
{
  if (age_ms == NULL || !traffic_info_id_valid(ac_id)) {
    return false;
  }
  const uint8_t slot = ti_acs_id[ac_id];
  if (slot >= ti_acs_idx || ti_acs[slot].ac_id != ac_id
      || !traffic_has_position_observation[slot]
      || !traffic_has_velocity_observation[slot]) {
    return false;
  }
  const uint64_t received_ms = Min(traffic_position_received_ms[slot],
                                   traffic_velocity_received_ms[slot]);
  const uint64_t age = traffic_monotonic_time_ms() - received_ms;
  *age_ms = age > UINT32_MAX ? UINT32_MAX : (uint32_t)age;
  return true;
}

bool traffic_info_get_snapshot(uint8_t ac_id,
                               struct EnuCoor_f *position,
                               struct EnuCoor_f *velocity,
                               uint32_t *age_ms)
{
  if (position == NULL || velocity == NULL || age_ms == NULL
      || !traffic_info_get_age(ac_id, age_ms)) {
    return false;
  }

  const uint8_t slot = ti_acs_registered_slot(ac_id);
  const uint16_t position_mask = (1u << AC_INFO_POS_UTM_I)
                                 | (1u << AC_INFO_POS_LLA_I)
                                 | (1u << AC_INFO_POS_ENU_I)
                                 | (1u << AC_INFO_POS_UTM_F)
                                 | (1u << AC_INFO_POS_LLA_F)
                                 | (1u << AC_INFO_POS_ENU_F);
  const uint16_t velocity_mask = (1u << AC_INFO_VEL_ENU_I)
                                 | (1u << AC_INFO_VEL_ENU_F)
                                 | (1u << AC_INFO_VEL_LOCAL_F);
  if (slot == TI_ACS_NONE
      || (ti_acs[slot].status & position_mask) == 0
      || (ti_acs[slot].status & velocity_mask) == 0) {
    return false;
  }

  const bool enu_position_available = bit_is_set(ti_acs[slot].status, AC_INFO_POS_ENU_I)
                                      || bit_is_set(ti_acs[slot].status, AC_INFO_POS_ENU_F);
  if (!enu_position_available
      && !(state.ned_initialized_i || state.ned_initialized_f || state.utm_initialized_f)) {
    return false;
  }

  *position = *acInfoGetPositionEnu_f(ac_id);
  if (!bit_is_set(ti_acs[slot].status, AC_INFO_POS_ENU_F)) {
    return false;
  }
  *velocity = *acInfoGetVelocityEnu_f(ac_id);
  if (!bit_is_set(ti_acs[slot].status, AC_INFO_VEL_ENU_F)) {
    return false;
  }

  return isfinite(position->x) && isfinite(position->y) && isfinite(position->z)
         && isfinite(velocity->x) && isfinite(velocity->y) && isfinite(velocity->z);
}

void traffic_info_touch(uint8_t slot)
{
  if (slot < NB_ACS) {
    const uint64_t now_ms = traffic_monotonic_time_ms();
    traffic_position_received_ms[slot] = now_ms;
    traffic_velocity_received_ms[slot] = now_ms;
    traffic_slot_activity_ms[slot] = now_ms;
    traffic_has_position_observation[slot] = true;
    traffic_has_velocity_observation[slot] = true;
    if (slot >= 2) {
      traffic_info_surveillance_established = true;
    }
  }
}

#if TRAFFIC_INFO_USE_MESH
void traffic_info_internal_note_slot_activity(uint8_t slot,
                                              uint64_t received_ms)
{
  if (slot < NB_ACS) {
    traffic_slot_activity_ms[slot] = received_ms;
  }
}

void traffic_info_internal_store_mesh(uint8_t slot, int32_t lat, int32_t lon,
                                      int32_t altitude_mm, int16_t course,
                                      uint16_t gspeed, int16_t climb,
                                      uint32_t itow)
{
  if (slot >= NB_ACS) {
    return;
  }
  ti_acs[slot].status = 0;
  ti_acs[slot].lla_pos_i.lat = lat;
  ti_acs[slot].lla_pos_i.lon = lon;
  ti_acs[slot].lla_pos_i.alt = altitude_mm;
  SetBit(ti_acs[slot].status, AC_INFO_POS_LLA_I);
  SetBit(ti_acs[slot].status, AC_INFO_VEL_LOCAL_F);
  SetBit(ti_acs[slot].status, AC_INFO_SOURCE_MESH);
  ti_acs[slot].course = RadOfDeciDeg(course);
  ti_acs[slot].gspeed = MOfCm(gspeed);
  ti_acs[slot].climb = MOfCm(climb);
  ti_acs[slot].itow = itow;
}

void traffic_info_internal_store_mesh_heartbeat(uint8_t slot, uint32_t itow)
{
  if (slot >= NB_ACS) {
    return;
  }
  const bool complete_legacy =
    !bit_is_set(ti_acs[slot].status, AC_INFO_SOURCE_MESH)
    && traffic_has_position_observation[slot]
    && traffic_has_velocity_observation[slot];
  if (!complete_legacy) {
    ti_acs[slot].status = (1u << AC_INFO_SOURCE_MESH);
    ti_acs[slot].itow = itow;
  }
}
#endif

void traffic_info_touch_position(uint8_t slot)
{
  if (slot < NB_ACS) {
#if TRAFFIC_INFO_USE_MESH
    if (traffic_info_mesh_clear_source(slot)) {
      ti_acs[slot].status &= ~AC_INFO_VELOCITY_MASK;
      traffic_has_velocity_observation[slot] = false;
    }
#endif
    const uint64_t now_ms = traffic_monotonic_time_ms();
    traffic_position_received_ms[slot] = now_ms;
    traffic_slot_activity_ms[slot] = now_ms;
    traffic_has_position_observation[slot] = true;
    if (slot >= 2 && traffic_has_velocity_observation[slot]) {
      traffic_info_surveillance_established = true;
    }
  }
}

void traffic_info_touch_velocity(uint8_t slot)
{
  if (slot < NB_ACS) {
#if TRAFFIC_INFO_USE_MESH
    if (traffic_info_mesh_clear_source(slot)) {
      ti_acs[slot].status &= ~AC_INFO_POSITION_MASK;
      traffic_has_position_observation[slot] = false;
    }
#endif
    const uint64_t now_ms = traffic_monotonic_time_ms();
    traffic_velocity_received_ms[slot] = now_ms;
    traffic_slot_activity_ms[slot] = now_ms;
    traffic_has_velocity_observation[slot] = true;
    if (slot >= 2 && traffic_has_position_observation[slot]) {
      traffic_info_surveillance_established = true;
    }
  }
}

void set_ac_info_utm(uint8_t id, int32_t utm_east, int32_t utm_north, int32_t alt, uint8_t utm_zone, int16_t course,
                     uint16_t gspeed, int16_t climb, uint32_t itow)
{
  const uint8_t slot = ti_acs_slot(id);
  if (slot == TI_ACS_NONE) {
    return;
  }

  #if TRAFFIC_INFO_USE_MESH
    if (traffic_info_mesh_source_active(slot)) {
      return;
    }
  #endif
  const struct traffic_source_observation observation = {
    .kind = TRAFFIC_SOURCE_POSITION_UTM,
    .position_1 = utm_east,
    .position_2 = utm_north,
    .altitude = alt,
    .course = course,
    .gspeed = gspeed,
    .climb = climb,
    .utm_zone = utm_zone
  };
  const bool payload_changed =
    traffic_source_observation_changed(slot, &observation);
  const uint64_t now_ms = traffic_monotonic_time_ms();
  const bool newer_itow = traffic_has_source_itow[slot]
                          && traffic_info_itow_is_newer(itow, traffic_source_itow[slot]);
  if (traffic_has_source_itow[slot] && !newer_itow) {
    const bool equal_itow = itow % TRAFFIC_INFO_GPS_WEEK_MS
                            == traffic_source_itow[slot] % TRAFFIC_INFO_GPS_WEEK_MS;
    if (!equal_itow || !traffic_info_equal_tow_episode_accepts(
          payload_changed, traffic_equal_itow_episode_active[slot], now_ms,
          traffic_source_itow_started_ms[slot], TRAFFIC_INFO_EQUAL_TOW_COMPAT_MS)) {
      return; // don't update on old data
    }
    if (!traffic_equal_itow_episode_active[slot]) {
      traffic_equal_itow_episode_active[slot] = true;
      traffic_source_itow_started_ms[slot] = now_ms;
    }
  }

  ti_acs[slot].status = 0;

  const struct UtmCoor_f *utm_origin = stateGetUtmOrigin_f();
  const uint8_t my_zone = utm_origin != NULL ? utm_origin->zone : utm_zone;
  if (utm_origin == NULL || utm_zone == my_zone) {
    ti_acs[slot].utm_pos_i.east = utm_east;
    ti_acs[slot].utm_pos_i.north = utm_north;
    ti_acs[slot].utm_pos_i.alt = alt;
    ti_acs[slot].utm_pos_i.zone = utm_zone;
    SetBit(ti_acs[slot].status, AC_INFO_POS_UTM_I);

  } else { // store other uav in utm extended zone
    struct UtmCoor_i utm = {.east = utm_east, .north = utm_north, .alt = alt, .zone = utm_zone};
    struct LlaCoor_i lla;
    lla_of_utm_i(&lla, &utm);
    update_geoid_height();
    lla.alt += geoid_height; /* incoming UTM altitude is MSL; LLA is ellipsoid */
    LLA_COPY(ti_acs[slot].lla_pos_i, lla);
    SetBit(ti_acs[slot].status, AC_INFO_POS_LLA_I);

    utm.zone = my_zone;
    utm_of_lla_i(&utm, &lla);
    utm.alt = alt; /* preserve the source MSL altitude across zone reprojection */

    UTM_COPY(ti_acs[slot].utm_pos_i, utm);
    SetBit(ti_acs[slot].status, AC_INFO_POS_UTM_I);
  }

  ti_acs[slot].course = RadOfDeciDeg(course);
  ti_acs[slot].gspeed = MOfCm(gspeed);
  ti_acs[slot].climb = MOfCm(climb);
  SetBit(ti_acs[slot].status, AC_INFO_VEL_LOCAL_F);

  ti_acs[slot].itow = itow;
  traffic_source_itow[slot] = itow;
  if (!traffic_has_source_itow[slot] || newer_itow) {
    traffic_equal_itow_episode_active[slot] = false;
  }
  traffic_has_source_itow[slot] = true;
  traffic_source_observation[slot] = observation;
  traffic_info_touch(slot);
}

void set_ac_info_lla(uint8_t id, int32_t lat, int32_t lon, int32_t alt,
                     int16_t course, uint16_t gspeed, int16_t climb, uint32_t itow)
{
  const uint8_t slot = ti_acs_slot(id);
  if (slot == TI_ACS_NONE) {
    return;
  }

  #if TRAFFIC_INFO_USE_MESH
    if (traffic_info_mesh_source_active(slot)) {
      return;
    }
  #endif
  const struct traffic_source_observation observation = {
    .kind = TRAFFIC_SOURCE_POSITION_LLA,
    .position_1 = lat,
    .position_2 = lon,
    .altitude = alt,
    .course = course,
    .gspeed = gspeed,
    .climb = climb,
    .utm_zone = 0
  };
  const bool payload_changed =
    traffic_source_observation_changed(slot, &observation);
  const uint64_t now_ms = traffic_monotonic_time_ms();
  const bool newer_itow = traffic_has_source_itow[slot]
                          && traffic_info_itow_is_newer(itow, traffic_source_itow[slot]);
  if (traffic_has_source_itow[slot] && !newer_itow) {
    const bool equal_itow = itow % TRAFFIC_INFO_GPS_WEEK_MS
                            == traffic_source_itow[slot] % TRAFFIC_INFO_GPS_WEEK_MS;
    if (!equal_itow || !traffic_info_equal_tow_episode_accepts(
          payload_changed, traffic_equal_itow_episode_active[slot], now_ms,
          traffic_source_itow_started_ms[slot], TRAFFIC_INFO_EQUAL_TOW_COMPAT_MS)) {
      return; // don't update on old data
    }
    if (!traffic_equal_itow_episode_active[slot]) {
      traffic_equal_itow_episode_active[slot] = true;
      traffic_source_itow_started_ms[slot] = now_ms;
    }
  }

  ti_acs[slot].status = 0;

  struct LlaCoor_i lla = {.lat = lat, .lon = lon, .alt = alt};
  LLA_COPY(ti_acs[slot].lla_pos_i, lla);
  SetBit(ti_acs[slot].status, AC_INFO_POS_LLA_I);

  ti_acs[slot].course = RadOfDeciDeg(course);
  ti_acs[slot].gspeed = MOfCm(gspeed);
  ti_acs[slot].climb = MOfCm(climb);
  SetBit(ti_acs[slot].status, AC_INFO_VEL_LOCAL_F);

  ti_acs[slot].itow = itow;
  traffic_source_itow[slot] = itow;
  if (!traffic_has_source_itow[slot] || newer_itow) {
    traffic_equal_itow_episode_active[slot] = false;
  }
  traffic_has_source_itow[slot] = true;
  traffic_source_observation[slot] = observation;
  traffic_info_touch(slot);
}

void set_ac_info_lla_arrival(uint8_t id, int32_t lat, int32_t lon, int32_t alt,
                             int16_t course, uint16_t gspeed, int16_t climb)
{
  const uint8_t slot = ti_acs_slot(id);
  if (slot == TI_ACS_NONE || traffic_has_source_itow[slot]) {
    return;
  }
#if TRAFFIC_INFO_USE_MESH
  if (traffic_info_mesh_source_active(slot)) {
    return;
  }
#endif

  ti_acs[slot].status = 0;
  ti_acs[slot].lla_pos_i.lat = lat;
  ti_acs[slot].lla_pos_i.lon = lon;
  ti_acs[slot].lla_pos_i.alt = alt;
  SetBit(ti_acs[slot].status, AC_INFO_POS_LLA_I);
  SetBit(ti_acs[slot].status, AC_INFO_VEL_LOCAL_F);
  ti_acs[slot].course = RadOfDeciDeg(course);
  ti_acs[slot].gspeed = MOfCm(gspeed);
  ti_acs[slot].climb = MOfCm(climb);
  ti_acs[slot].itow = gps_tow_from_sys_ticks(sys_time.nb_tick);
  traffic_info_touch(slot);
}

/* Lazy reference-frame conversions. Each function computes only a missing
 * representation and marks it valid in the aircraft's status bit field. */

/** Copy UTM traffic into the current ownship UTM zone. */
static bool traffic_info_utm_in_origin_zone(uint8_t ac_id, struct UtmCoor_f *utm)
{
  const struct UtmCoor_f *source = acInfoGetPositionUtm_f(ac_id);
  const struct UtmCoor_f *origin = stateGetUtmOrigin_f();
  if (source == NULL || origin == NULL) { return false; }

  *utm = *source;
  if (utm->zone != origin->zone) {
    struct LlaCoor_f lla;
    lla_of_utm_f(&lla, utm);
    utm->zone = origin->zone;
    utm_of_lla_f(utm, &lla);
  }
  return true;
}

/* compute UTM position of aircraft with ac_id (int) */
void acInfoCalcPositionUtm_i(uint8_t ac_id)
{
  const uint8_t ac_nr = ti_acs_registered_slot(ac_id);
  if (ac_nr == TI_ACS_NONE) { return; }
  bool converted = false;
  if (bit_is_set(ti_acs[ac_nr].status, AC_INFO_POS_UTM_I))
  {
    return;
  }

  /* LLA_i -> UTM_i is more accurate than from UTM_f */
  if (bit_is_set(ti_acs[ac_nr].status, AC_INFO_POS_LLA_I)
      && state.utm_initialized_f)
  {
    // use my zone as reference, i.e zone extend
    ti_acs[ac_nr].utm_pos_i.zone = stateGetUtmOrigin_f()->zone;
    utm_of_lla_i(&ti_acs[ac_nr].utm_pos_i, &ti_acs[ac_nr].lla_pos_i);
    update_geoid_height();
    ti_acs[ac_nr].utm_pos_i.alt -= geoid_height;
    converted = true;
  } else if (bit_is_set(ti_acs[ac_nr].status, AC_INFO_POS_UTM_F))
  {
    UTM_BFP_OF_REAL(ti_acs[ac_nr].utm_pos_i, ti_acs[ac_nr].utm_pos_f);
    converted = true;
  } else if (bit_is_set(ti_acs[ac_nr].status, AC_INFO_POS_LLA_F)
             && state.utm_initialized_f)
  {
    // use my zone as reference, i.e zone extend
    ti_acs[ac_nr].utm_pos_f.zone = stateGetUtmOrigin_f()->zone;
    utm_of_lla_f(&ti_acs[ac_nr].utm_pos_f, &ti_acs[ac_nr].lla_pos_f);
    update_geoid_height();
    ti_acs[ac_nr].utm_pos_f.alt -= geoid_height/1000.;
    SetBit(ti_acs[ac_nr].status, AC_INFO_POS_UTM_F);
    UTM_BFP_OF_REAL(ti_acs[ac_nr].utm_pos_i, ti_acs[ac_nr].utm_pos_f);
    converted = true;
  }
  if (converted) {
    SetBit(ti_acs[ac_nr].status, AC_INFO_POS_UTM_I);
  }
}

/* compute LLA position of aircraft with ac_id (int) */
void acInfoCalcPositionLla_i(uint8_t ac_id)
{
  const uint8_t ac_nr = ti_acs_registered_slot(ac_id);
  if (ac_nr == TI_ACS_NONE) { return; }
  bool converted = false;
  if (bit_is_set(ti_acs[ac_nr].status, AC_INFO_POS_LLA_I))
  {
    return;
  }

  if (bit_is_set(ti_acs[ac_nr].status, AC_INFO_POS_LLA_F))
  {
    LLA_BFP_OF_REAL(ti_acs[ac_nr].lla_pos_i, ti_acs[ac_nr].lla_pos_f);
    converted = true;
  } else if (bit_is_set(ti_acs[ac_nr].status, AC_INFO_POS_UTM_I))
  {
    lla_of_utm_i(&ti_acs[ac_nr].lla_pos_i, &ti_acs[ac_nr].utm_pos_i);
    update_geoid_height();
    ti_acs[ac_nr].lla_pos_i.alt += geoid_height;
    converted = true;
  } else if (bit_is_set(ti_acs[ac_nr].status, AC_INFO_POS_UTM_F))
  {
    lla_of_utm_f(&ti_acs[ac_nr].lla_pos_f, &ti_acs[ac_nr].utm_pos_f);
    update_geoid_height();
    ti_acs[ac_nr].lla_pos_f.alt += geoid_height/1000.;
    SetBit(ti_acs[ac_nr].status, AC_INFO_POS_LLA_F);
    LLA_BFP_OF_REAL(ti_acs[ac_nr].lla_pos_i, ti_acs[ac_nr].lla_pos_f);
    converted = true;
  }
  if (converted) {
    SetBit(ti_acs[ac_nr].status, AC_INFO_POS_LLA_I);
  }
}

/* compute ENU position of aircraft with ac_id (int) */
void acInfoCalcPositionEnu_i(uint8_t ac_id)
{
  const uint8_t ac_nr = ti_acs_registered_slot(ac_id);
  if (ac_nr == TI_ACS_NONE) { return; }
  bool converted = false;
  if (bit_is_set(ti_acs[ac_nr].status, AC_INFO_POS_ENU_I))
  {
    return;
  }

  if (bit_is_set(ti_acs[ac_nr].status, AC_INFO_POS_ENU_F))
  {
    ENU_BFP_OF_REAL(ti_acs[ac_nr].enu_pos_i, ti_acs[ac_nr].enu_pos_f);
    converted = true;
  }
  else if (state.ned_initialized_i)
  {
    if ((ti_acs[ac_nr].status & AC_INFO_POSITION_MASK) != 0)
    {
      const struct LlaCoor_i *lla = acInfoGetPositionLla_i(ac_id);
      if (lla != NULL) {
        struct EnuCoor_i enu;
        struct LlaCoor_i lla_copy = *lla;
        enu_of_lla_point_i(&enu, stateGetNedOrigin_i(), &lla_copy);
        // convert ENU pos from cm to BFP with INT32_POS_FRAC
        enu.x = POS_BFP_OF_REAL(enu.x) / 100;
        enu.y = POS_BFP_OF_REAL(enu.y) / 100;
        enu.z = POS_BFP_OF_REAL(enu.z) / 100;
        ti_acs[ac_nr].enu_pos_i = enu;
        converted = true;
      }
    }
  } else if (state.ned_initialized_f) {
    if ((ti_acs[ac_nr].status & AC_INFO_POSITION_MASK) != 0) {
      const struct LlaCoor_f *lla = acInfoGetPositionLla_f(ac_id);
      if (lla != NULL) {
        struct LlaCoor_f lla_copy = *lla;
        enu_of_lla_point_f(&ti_acs[ac_nr].enu_pos_f,
                           stateGetNedOrigin_f(), &lla_copy);
        SetBit(ti_acs[ac_nr].status, AC_INFO_POS_ENU_F);
        ENU_BFP_OF_REAL(ti_acs[ac_nr].enu_pos_i, ti_acs[ac_nr].enu_pos_f);
        converted = true;
      }
    }
  } else if (state.utm_initialized_f
             && (ti_acs[ac_nr].status
                 & ((1u << AC_INFO_POS_UTM_I) | (1u << AC_INFO_POS_UTM_F)
                    | (1u << AC_INFO_POS_LLA_I) | (1u << AC_INFO_POS_LLA_F))) != 0)
  {
    /* if utm origin is initialized we use the ENU = UTM - UTM_ORIGIN as in state to facilitate comparison */
    struct UtmCoor_f utm;
    if (traffic_info_utm_in_origin_zone(ac_id, &utm)) {
      ENU_OF_UTM_DIFF(ti_acs[ac_nr].enu_pos_f, utm, *stateGetUtmOrigin_f());
      SetBit(ti_acs[ac_nr].status, AC_INFO_POS_ENU_F);
      ENU_BFP_OF_REAL(ti_acs[ac_nr].enu_pos_i, ti_acs[ac_nr].enu_pos_f);
      converted = true;
    }
  }
  if (converted) {
    SetBit(ti_acs[ac_nr].status, AC_INFO_POS_ENU_I);
  }
}

/* compute UTM position of aircraft with ac_id (float) */
void acInfoCalcPositionUtm_f(uint8_t ac_id)
{
  const uint8_t ac_nr = ti_acs_registered_slot(ac_id);
  if (ac_nr == TI_ACS_NONE) { return; }
  bool converted = false;
  if (bit_is_set(ti_acs[ac_nr].status, AC_INFO_POS_UTM_F))
  {
    return;
  }

  if (bit_is_set(ti_acs[ac_nr].status, AC_INFO_POS_UTM_I))
  {
    UTM_FLOAT_OF_BFP(ti_acs[ac_nr].utm_pos_f, ti_acs[ac_nr].utm_pos_i);
    converted = true;
  } else if (bit_is_set(ti_acs[ac_nr].status, AC_INFO_POS_LLA_I)
             && state.utm_initialized_f)
  {
    // use my zone as reference, i.e zone extend
    ti_acs[ac_nr].utm_pos_i.zone = stateGetUtmOrigin_f()->zone;
    utm_of_lla_i(&ti_acs[ac_nr].utm_pos_i, &ti_acs[ac_nr].lla_pos_i);
    update_geoid_height();
    ti_acs[ac_nr].utm_pos_i.alt -= geoid_height;
    SetBit(ti_acs[ac_nr].status, AC_INFO_POS_UTM_I);
    UTM_FLOAT_OF_BFP(ti_acs[ac_nr].utm_pos_f, ti_acs[ac_nr].utm_pos_i);
    converted = true;
  } else if (bit_is_set(ti_acs[ac_nr].status, AC_INFO_POS_LLA_F)
             && state.utm_initialized_f)
  {
    /* not very accurate with float ~5cm */
    ti_acs[ac_nr].utm_pos_f.zone = stateGetUtmOrigin_f()->zone;
    utm_of_lla_f(&ti_acs[ac_nr].utm_pos_f, &ti_acs[ac_nr].lla_pos_f);
    update_geoid_height();
    ti_acs[ac_nr].utm_pos_f.alt -= geoid_height/1000.;
    converted = true;
  }
  if (converted) {
    SetBit(ti_acs[ac_nr].status, AC_INFO_POS_UTM_F);
  }
}

/* compute LLA position of aircraft with ac_id (float) */
void acInfoCalcPositionLla_f(uint8_t ac_id)
{
  const uint8_t ac_nr = ti_acs_registered_slot(ac_id);
  if (ac_nr == TI_ACS_NONE) { return; }
  bool converted = false;
  if (bit_is_set(ti_acs[ac_nr].status, AC_INFO_POS_LLA_F))
  {
    return;
  }

  if (bit_is_set(ti_acs[ac_nr].status, AC_INFO_POS_LLA_I))
  {
    LLA_FLOAT_OF_BFP(ti_acs[ac_nr].lla_pos_f, ti_acs[ac_nr].lla_pos_i);
    converted = true;
  } else if (bit_is_set(ti_acs[ac_nr].status, AC_INFO_POS_UTM_I))
  {
    lla_of_utm_i(&ti_acs[ac_nr].lla_pos_i, &ti_acs[ac_nr].utm_pos_i);
    update_geoid_height();
    ti_acs[ac_nr].lla_pos_i.alt += geoid_height;
    SetBit(ti_acs[ac_nr].status, AC_INFO_POS_LLA_I);
    LLA_FLOAT_OF_BFP(ti_acs[ac_nr].lla_pos_f, ti_acs[ac_nr].lla_pos_i);
    converted = true;
  } else if (bit_is_set(ti_acs[ac_nr].status, AC_INFO_POS_UTM_F))
  {
    lla_of_utm_f(&ti_acs[ac_nr].lla_pos_f, &ti_acs[ac_nr].utm_pos_f);
    update_geoid_height();
    ti_acs[ac_nr].lla_pos_f.alt += geoid_height/1000.;
    converted = true;
  }
  if (converted) {
    SetBit(ti_acs[ac_nr].status, AC_INFO_POS_LLA_F);
  }
}

/* Compute ENU position of aircraft with ac_id (float). */
void acInfoCalcPositionEnu_f(uint8_t ac_id)
{
  const uint8_t ac_nr = ti_acs_registered_slot(ac_id);
  if (ac_nr == TI_ACS_NONE) { return; }
  bool converted = false;
  if (bit_is_set(ti_acs[ac_nr].status, AC_INFO_POS_ENU_F))
  {
    return;
  }

  if (bit_is_set(ti_acs[ac_nr].status, AC_INFO_POS_ENU_I))
  {
    ENU_FLOAT_OF_BFP(ti_acs[ac_nr].enu_pos_f, ti_acs[ac_nr].enu_pos_i);
    converted = true;
  }
  else if (state.ned_initialized_f)
  {
    if (bit_is_set(ti_acs[ac_nr].status, AC_INFO_POS_LLA_F) || bit_is_set(ti_acs[ac_nr].status, AC_INFO_POS_UTM_F))
    {
      const struct LlaCoor_f *lla = acInfoGetPositionLla_f(ac_id);
      if (lla != NULL) {
        struct LlaCoor_f lla_copy = *lla;
        enu_of_lla_point_f(&ti_acs[ac_nr].enu_pos_f,
                           stateGetNedOrigin_f(), &lla_copy);
        converted = true;
      }
    } else if (bit_is_set(ti_acs[ac_nr].status, AC_INFO_POS_LLA_I) || bit_is_set(ti_acs[ac_nr].status, AC_INFO_POS_UTM_I))
    {
      const struct LlaCoor_i *lla_i = acInfoGetPositionLla_i(ac_id);
      if (lla_i != NULL) {
        struct LlaCoor_f lla;
        LLA_FLOAT_OF_BFP(lla, *lla_i);
        enu_of_lla_point_f(&ti_acs[ac_nr].enu_pos_f, stateGetNedOrigin_f(), &lla);
        converted = true;
      }
    }
  } else if (state.ned_initialized_i) {
    if ((ti_acs[ac_nr].status & AC_INFO_POSITION_MASK) != 0) {
      const struct LlaCoor_i *lla = acInfoGetPositionLla_i(ac_id);
      if (lla != NULL) {
        struct EnuCoor_i enu;
        struct LlaCoor_i lla_copy = *lla;
        enu_of_lla_point_i(&enu, stateGetNedOrigin_i(), &lla_copy);
        // convert ENU pos from cm to BFP with INT32_POS_FRAC
        enu.x = POS_BFP_OF_REAL(enu.x) / 100;
        enu.y = POS_BFP_OF_REAL(enu.y) / 100;
        enu.z = POS_BFP_OF_REAL(enu.z) / 100;
        ti_acs[ac_nr].enu_pos_i = enu;
        SetBit(ti_acs[ac_nr].status, AC_INFO_POS_ENU_I);
        ENU_FLOAT_OF_BFP(ti_acs[ac_nr].enu_pos_f, ti_acs[ac_nr].enu_pos_i);
        converted = true;
      }
    }
  } else if (state.utm_initialized_f
             && (ti_acs[ac_nr].status
                 & ((1u << AC_INFO_POS_UTM_I) | (1u << AC_INFO_POS_UTM_F)
                    | (1u << AC_INFO_POS_LLA_I) | (1u << AC_INFO_POS_LLA_F))) != 0)
  {
    /* if utm origin is initialized we use the ENU = UTM - UTM_ORIGIN as in state to facilitate comparison */
    struct UtmCoor_f utm;
    if (traffic_info_utm_in_origin_zone(ac_id, &utm)) {
      ENU_OF_UTM_DIFF(ti_acs[ac_nr].enu_pos_f, utm, *stateGetUtmOrigin_f());
      converted = true;
    }
  }
  if (converted) {
    SetBit(ti_acs[ac_nr].status, AC_INFO_POS_ENU_F);
  }
}

/* compute ENU velocity of aircraft with ac_id (int) */
void acInfoCalcVelocityEnu_i(uint8_t ac_id)
{
  const uint8_t ac_nr = ti_acs_registered_slot(ac_id);
  if (ac_nr == TI_ACS_NONE) { return; }
  bool converted = false;
  if (bit_is_set(ti_acs[ac_nr].status, AC_INFO_VEL_ENU_I))
  {
    return;
  }

  if (bit_is_set(ti_acs[ac_nr].status, AC_INFO_VEL_ENU_F))
  {
    SPEEDS_BFP_OF_REAL(ti_acs[ac_nr].enu_vel_i, ti_acs[ac_nr].enu_vel_f);
    converted = true;
  } else if (bit_is_set(ti_acs[ac_nr].status, AC_INFO_VEL_LOCAL_F)) {
    ti_acs[ac_nr].enu_vel_f.x = ti_acs[ac_nr].gspeed * sinf(ti_acs[ac_nr].course);
    ti_acs[ac_nr].enu_vel_f.y = ti_acs[ac_nr].gspeed * cosf(ti_acs[ac_nr].course);
    ti_acs[ac_nr].enu_vel_f.z = ti_acs[ac_nr].climb;
    SetBit(ti_acs[ac_nr].status, AC_INFO_VEL_ENU_F);
    SPEEDS_BFP_OF_REAL(ti_acs[ac_nr].enu_vel_i, ti_acs[ac_nr].enu_vel_f);
    converted = true;
  }
  if (converted) {
    SetBit(ti_acs[ac_nr].status, AC_INFO_VEL_ENU_I);
  }
}

/* compute ENU position of aircraft with ac_id (float) */
void acInfoCalcVelocityEnu_f(uint8_t ac_id)
{
  const uint8_t ac_nr = ti_acs_registered_slot(ac_id);
  if (ac_nr == TI_ACS_NONE) { return; }
  bool converted = false;
  if (bit_is_set(ti_acs[ac_nr].status, AC_INFO_VEL_ENU_F))
  {
    return;
  }

  if (bit_is_set(ti_acs[ac_nr].status, AC_INFO_VEL_ENU_I))
  {
    SPEEDS_FLOAT_OF_BFP(ti_acs[ac_nr].enu_vel_f, ti_acs[ac_nr].enu_vel_i);
    converted = true;
  } else if (bit_is_set(ti_acs[ac_nr].status, AC_INFO_VEL_LOCAL_F)) {
    ti_acs[ac_nr].enu_vel_f.x = ti_acs[ac_nr].gspeed * sinf(ti_acs[ac_nr].course);
    ti_acs[ac_nr].enu_vel_f.y = ti_acs[ac_nr].gspeed * cosf(ti_acs[ac_nr].course);
    ti_acs[ac_nr].enu_vel_f.z = ti_acs[ac_nr].climb;
    converted = true;
  }
  if (converted) {
    SetBit(ti_acs[ac_nr].status, AC_INFO_VEL_ENU_F);
  }
}

void traffic_info_log_start(void)
{
#if TRAFFIC_INFO_USE_LOG
  LogOpen(pprzLogFile, STRINGIFY(TRAFFIC_INFO_FILE_PATH), NULL);
#endif
}

void traffic_info_log_stop(void)
{
#if TRAFFIC_INFO_USE_LOG
  if (LogFileIsOpen(pprzLogFile)) {
    LogClose(pprzLogFile);
  }
#endif
}

