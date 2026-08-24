/*
 * Copyright (C) Pascal Brisset, Antoine Drouin (2008), 
 *               Kirk Scheper (2016), OpenUAS (2026)
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
 */

/**
 * @file modules/multi/traffic_info.h
 * @brief Traffic-aircraft state storage, conversion, and optional mesh exchange.
 * @author Pascal Brisset
 * @author Antoine Drouin
 * @author Kirk Scheper
 * @author OpenUAS
 * 
 * The legacy API stores positions and velocities received through ACINFO and
 * GPS-family messages. Define TRAFFIC_INFO_USE_MESH to add compact MESH_STATE
 * reception and self-organising TDMA transmission without changing that API.
 */

#ifndef TRAFFIC_INFO_H
#define TRAFFIC_INFO_H

#include "mcu_periph/sys_time.h"
#include "math/pprz_geodetic_int.h"
#include "math/pprz_geodetic_float.h"
#include "modules/gps/gps.h"
#include "modules/multi/traffic_info_mesh.h"

#ifndef TRAFFIC_INFO_USE_MESH
#define TRAFFIC_INFO_USE_MESH 0
#endif

/** PPRZLink aircraft-ID domain used by traffic information.
 *
 * ID 0 is reserved for the ground control station. Airborne aircraft use
 * IDs 1 through 254. ID 255 is reserved for protocol broadcast and internal
 * sentinels, so it must never identify an aircraft or a message sender.
 */
#define TRAFFIC_INFO_GCS_ID 0u
#define TRAFFIC_INFO_MAX_AC_ID 254u
#define TRAFFIC_INFO_RESERVED_ID 255u

#ifndef NB_ACS_ID
#define NB_ACS_ID (TRAFFIC_INFO_MAX_AC_ID + 1u)
#endif
#ifndef NB_ACS
#define NB_ACS 24
#endif

/** Maximum local interval in which changing data may reuse one source TOW. */
#ifndef TRAFFIC_INFO_EQUAL_TOW_COMPAT_MS
#define TRAFFIC_INFO_EQUAL_TOW_COMPAT_MS 5000u
#endif

/** Minimum inactivity before a full table may reuse a remote-aircraft slot. */
#ifndef TRAFFIC_INFO_RECLAIM_MS
#define TRAFFIC_INFO_RECLAIM_MS 300000u
#endif

#if NB_ACS_ID <= TRAFFIC_INFO_MAX_AC_ID
#error "NB_ACS_ID must contain every traffic ID from 0 through 254"
#endif
#if NB_ACS_ID > 255
#error "NB_ACS_ID must fit the supported 0 through 254 traffic-ID domain"
#endif
#if NB_ACS > 255
#error "NB_ACS must fit the byte-sized traffic-table index"
#endif
#if NB_ACS < 2
#error "NB_ACS must provide slots for the GCS and local aircraft"
#endif

/** Return whether @p id is a supported GCS or airborne traffic ID. */
static inline bool traffic_info_id_valid(uint8_t id)
{
  return id <= TRAFFIC_INFO_MAX_AC_ID;
}

/** Return whether @p id identifies an airborne aircraft rather than the GCS. */
static inline bool traffic_info_aircraft_id_valid(uint8_t id)
{
  return id != TRAFFIC_INFO_GCS_ID && traffic_info_id_valid(id);
}

/** Invalid traffic-table index returned when a new aircraft cannot be stored.
 *
 * This value is never a valid index into @ref ti_acs, which has at most
 * @ref NB_ACS entries.
 */
#define TI_ACS_NONE 0xFF

/**
 * @defgroup ac_info Traffic-aircraft state representations
 * @brief Storage and lazy conversion of traffic positions and velocities.
 * @{
 */
#define AC_INFO_POS_UTM_I 0
#define AC_INFO_POS_LLA_I 1
#define AC_INFO_POS_ENU_I 2
#define AC_INFO_POS_UTM_F 3
#define AC_INFO_POS_LLA_F 4
#define AC_INFO_POS_ENU_F 5
#define AC_INFO_VEL_ENU_I 6
#define AC_INFO_VEL_ENU_F 7
#define AC_INFO_VEL_LOCAL_F 8

/* Velocity validity is represented by status bits, not vector magnitude.
 * A measured (0, 0, 0) velocity is a valid stationary traffic track. */

#define AC_INFO_POSITION_MASK ((1u << AC_INFO_POS_UTM_I) | (1u << AC_INFO_POS_LLA_I) \
                               | (1u << AC_INFO_POS_ENU_I) | (1u << AC_INFO_POS_UTM_F) \
                               | (1u << AC_INFO_POS_LLA_F) | (1u << AC_INFO_POS_ENU_F))
#define AC_INFO_VELOCITY_MASK ((1u << AC_INFO_VEL_ENU_I) | (1u << AC_INFO_VEL_ENU_F) \
                               | (1u << AC_INFO_VEL_LOCAL_F))

struct acInfo {
  uint8_t ac_id;
  /**
   * Holds the status bits for all acinfo position and velocity representations.
   * When the corresponding bit is set the representation
   * is already computed.
   */
  uint16_t status;

  /**
   * Position in UTM coordinates.
   * Units x,y: centimetres.
   * Units z: millimetres above MSL
   */
  struct UtmCoor_i utm_pos_i;

  /**
   * Position in Latitude, Longitude and Altitude.
   * Units lat,lon: degrees*1e7
  * Units alt: millimeters above reference ellipsoid
   */
  struct LlaCoor_i lla_pos_i;

  /**
  * Position in East-North-Up coordinates.
   * Units: m in BFP with #INT32_POS_FRAC
   */
  struct EnuCoor_i enu_pos_i;

  /**
  * Velocity in East-North-Up coordinates.
   * Units: m/s in BFP with #INT32_SPEED_FRAC
   */
  struct EnuCoor_i enu_vel_i;

  /**
   * Position in UTM coordinates.
   * Units x,y: meters.
   * Units z: meters above MSL
   */
  struct UtmCoor_f utm_pos_f;

  /**
   * Position in Latitude, Longitude and Altitude.
   * Units lat,lon: radians
   * Units alt: meters above reference ellipsoid
   */
  struct LlaCoor_f lla_pos_f;

  /**
  * Position in East-North-Up coordinates
   * Units: m */
  struct EnuCoor_f enu_pos_f;

  /**
  * @brief Velocity in East-North-Up coordinates.
   * @details Units: m/s */
  struct EnuCoor_f enu_vel_f;

  float course;        ///< rad
  float gspeed;        ///< m/s
  float climb;         ///< m/s
  uint32_t itow;       ///< ms
};

extern uint8_t ti_acs_idx;
extern uint8_t ti_acs_id[];
extern struct acInfo ti_acs[];
/** An unknown aircraft could not obtain a slot; traffic may be unrepresented. */
extern bool traffic_info_capacity_exceeded;
/** Sticky evidence that a remote track has supplied position and velocity. */
extern bool traffic_info_surveillance_established;

extern void traffic_info_init(void);

/**
 * Resolve an aircraft id to its slot in ::ti_acs, inserting it if needed.
 *
 * Known IDs always retain their slot. When the table is full, the least
 * recently active remote slot is reused only after #TRAFFIC_INFO_RECLAIM_MS;
 * GCS and ownship slots are never reclaimed. This keeps current tracks moving
 * without silently replacing active traffic.
 *
 * @param[in] id aircraft id, 0 is the GCS and always maps to slot 0
 * @return slot index in ::ti_acs, or #TI_ACS_NONE when the aircraft is unknown
 *         and the table is full
 */
extern uint8_t ti_acs_slot(uint8_t id);

/** Notify slot-indexed consumers immediately before a slot is cleared.
 *
 * Overrides must discard all state associated with @p slot. At callback time,
 * `ti_acs[slot].ac_id` still identifies the previous owner, allowing TCAS and
 * similar consumers to invalidate global state that refers to that aircraft.
 *
 * @param[in] slot Traffic-table slot about to be assigned or reused.
 */
extern void traffic_info_slot_reassigned(uint8_t slot);

/** Resolve an already registered traffic ID without inserting a new record. */
static inline uint8_t ti_acs_registered_slot(uint8_t id)
{
  if (!traffic_info_id_valid(id)) {
    return TI_ACS_NONE;
  }
  const uint8_t slot = ti_acs_id[id];
  if (slot >= ti_acs_idx || ti_acs[slot].ac_id != id) {
    return TI_ACS_NONE;
  }
  return slot;
}

/** Resolve an ID for legacy getters that cannot report lookup failure.
 *
 * Historical pointer getters always returned storage and unknown IDs read slot
 * zero. Preserve that API while preventing reserved ID 255 from indexing past
 * ::ti_acs_id. New safety-sensitive code should use the checked snapshot APIs.
 */
static inline uint8_t ti_acs_legacy_read_slot(uint8_t id)
{
  return traffic_info_id_valid(id) ? ti_acs_id[id] : 0u;
}

/**
 * Parse a supported traffic-position message.
 *
 * Legacy mode handles GPS_SMALL, GPS, GPS_LLA, GPS_INT, ACINFO, and
 * ACINFO_LLA. Mesh mode additionally handles MESH_STATE.
 *
 * @param[in] buf Encoded PPRZLink message buffer.
 * @return @c true when the message was handled, otherwise @c false.
 */
extern bool parse_acinfo_dl(uint8_t *buf);

/** Return local monotonic age of the complete position/velocity pair.
 *
 * The older component determines age, so refreshing only position or velocity
 * cannot make an incomplete stale pair appear current. The clock is monotonic,
 * independent of source TOW rollover, and advances only for accepted data.
 *
 * @param[in] ac_id Traffic ID to query.
 * @param[out] age_ms Age in milliseconds, saturated at UINT32_MAX.
 * @return @c true when both position and velocity have accepted observations.
 */
extern bool traffic_info_get_age(uint8_t ac_id, uint32_t *age_ms);

/** Copy a complete traffic observation in local ENU coordinates.
 *
 * Unlike the legacy pointer getters, this checked API reports failed or
 * unavailable coordinate conversions explicitly. It is intended for
 * safety-critical consumers that must fail closed on incomplete data.
 *
 * @param[in] ac_id Traffic ID to query.
 * @param[out] position Local ENU position in meters.
 * @param[out] velocity Local ENU velocity in meters per second.
 * @param[out] age_ms Local monotonic age of the observation in milliseconds.
 * @return @c true for a complete finite observation, otherwise @c false.
 */
extern bool traffic_info_get_snapshot(uint8_t ac_id,
                                      struct EnuCoor_f *position,
                                      struct EnuCoor_f *velocity,
                                      uint32_t *age_ms);

/** Mark both components current after one coherent observation was stored.
 *
 * This is the only touch operation that can establish surveillance by itself.
 */
extern void traffic_info_touch(uint8_t slot);

/** Mark only position current.
 *
 * When a legacy update supersedes mesh ownership, the opposite mesh component
 * is invalidated so checked snapshots cannot combine different sources.
 */
extern void traffic_info_touch_position(uint8_t slot);

/** Mark only velocity current; see traffic_info_touch_position() for why a
 * source change invalidates the opposite component.
 */
extern void traffic_info_touch_velocity(uint8_t slot);

/************************ Set functions ****************************/

/**
 * Set Aircraft info.
 * @param[in] id traffic ID; 0 is the GCS and the local aircraft uses AC_ID
 * @param[in] utm_east UTM east in cm
 * @param[in] utm_north UTM north in cm
 * @param[in] alt Altitude in mm above MSL
 * @param[in] utm_zone UTM zone
 * @param[in] course Course in decideg (CW)
 * @param[in] gspeed Ground speed in cm/s
 * @param[in] climb Climb rate in cm/s
 * @param[in] itow GPS time of week in ms
 *
 * Older source-TOW samples are rejected. Changed equal-TOW samples remain
 * compatible only for #TRAFFIC_INFO_EQUAL_TOW_COMPAT_MS, preventing a frozen
 * sender clock from refreshing one track indefinitely. Fresh mesh data, when
 * enabled, remains authoritative until its configured drop horizon.
 */
extern void set_ac_info_utm(uint8_t id, int32_t utm_east, int32_t utm_north, int32_t alt, uint8_t utm_zone,
                            int16_t course, uint16_t gspeed, int16_t climb, uint32_t itow);

/**
 * Set Aircraft info.
 * @param[in] id traffic ID; 0 is the GCS and the local aircraft uses AC_ID
 * @param[in] lat Latitude in 1e7deg
 * @param[in] lon Longitude in 1e7deg
 * @param[in] alt Altitude in mm above ellipsoid
 * @param[in] course Course in decideg (CW)
 * @param[in] gspeed Ground speed in cm/s
 * @param[in] climb Climb rate in cm/s
 * @param[in] itow GPS time of week in ms
 *
 * Ordering and mesh-authority rules are identical to set_ac_info_utm().
 */
extern void set_ac_info_lla(uint8_t id, int32_t lat, int32_t lon, int32_t alt,
                            int16_t course, uint16_t gspeed, int16_t climb, uint32_t itow);

/** Set LLA traffic received without a source timestamp.
 *
 * Arrival-stamped observations never replace a source-TOW ordered stream;
 * otherwise a delayed low-information packet could overwrite ordered data.
 */
extern void set_ac_info_lla_arrival(uint8_t id, int32_t lat, int32_t lon, int32_t alt,
                                    int16_t course, uint16_t gspeed, int16_t climb);

/** Set position from UTM coordinates (int).
* @param[in] ac_id aircraft id of aircraft info to set
* @param[in] utm_pos UTM position (int)
*/
static inline void acInfoSetPositionUtm_i(uint8_t ac_id, struct UtmCoor_i *utm_pos)
{
  const uint8_t slot = ti_acs_slot(ac_id);
  if (slot == TI_ACS_NONE) {
    return;
  }
  UTM_COPY(ti_acs[slot].utm_pos_i, *utm_pos);
  /* clear bits for all position representations and only set the new one */
  ti_acs[slot].status = (ti_acs[slot].status & ~AC_INFO_POSITION_MASK)
                        | (1u << AC_INFO_POS_UTM_I);
  ti_acs[slot].itow = gps_tow_from_sys_ticks(sys_time.nb_tick);
  traffic_info_touch_position(slot);
}

/** Set position from LLA coordinates (int).
* @param[in] ac_id aircraft id of aircraft info to set
* @param[in] lla_pos LLA position (int)
*/
static inline void acInfoSetPositionLla_i(uint8_t ac_id, struct LlaCoor_i *lla_pos)
{
  const uint8_t slot = ti_acs_slot(ac_id);
  if (slot == TI_ACS_NONE) {
    return;
  }
  LLA_COPY(ti_acs[slot].lla_pos_i, *lla_pos);
  /* clear bits for all position representations and only set the new one */
  ti_acs[slot].status = (ti_acs[slot].status & ~AC_INFO_POSITION_MASK)
                        | (1u << AC_INFO_POS_LLA_I);
  ti_acs[slot].itow = gps_tow_from_sys_ticks(sys_time.nb_tick);
  traffic_info_touch_position(slot);
}

/** Set position from ENU coordinates (int).
* @param[in] ac_id aircraft id of aircraft info to set
* @param[in] enu_pos position in ENU (int)
*/
static inline void acInfoSetPositionEnu_i(uint8_t ac_id, struct EnuCoor_i *enu_pos)
{
  const uint8_t slot = ti_acs_slot(ac_id);
  if (slot == TI_ACS_NONE) {
    return;
  }
  VECT3_COPY(ti_acs[slot].enu_pos_i, *enu_pos);
  /* clear bits for all position representations and only set the new one */
  ti_acs[slot].status = (ti_acs[slot].status & ~AC_INFO_POSITION_MASK)
                        | (1u << AC_INFO_POS_ENU_I);
  ti_acs[slot].itow = gps_tow_from_sys_ticks(sys_time.nb_tick);
  traffic_info_touch_position(slot);
}

/** Set position from UTM coordinates (float).
* @param[in] ac_id aircraft id of aircraft info to set
* @param[in] utm_pos UTM position (float)
*/
static inline void acInfoSetPositionUtm_f(uint8_t ac_id, struct UtmCoor_f *utm_pos)
{
  const uint8_t slot = ti_acs_slot(ac_id);
  if (slot == TI_ACS_NONE) {
    return;
  }
  UTM_COPY(ti_acs[slot].utm_pos_f, *utm_pos);
  /* clear bits for all position representations and only set the new one */
  ti_acs[slot].status = (ti_acs[slot].status & ~AC_INFO_POSITION_MASK)
                        | (1u << AC_INFO_POS_UTM_F);
  ti_acs[slot].itow = gps_tow_from_sys_ticks(sys_time.nb_tick);
  traffic_info_touch_position(slot);
}

/** Set position from LLA coordinates (float).
* @param[in] ac_id aircraft id of aircraft info to set
* @param[in] lla_pos LLA position (float)
*/
static inline void acInfoSetPositionLla_f(uint8_t ac_id, struct LlaCoor_f *lla_pos)
{
  const uint8_t slot = ti_acs_slot(ac_id);
  if (slot == TI_ACS_NONE) {
    return;
  }
  LLA_COPY(ti_acs[slot].lla_pos_f, *lla_pos);
  /* clear bits for all position representations and only set the new one */
  ti_acs[slot].status = (ti_acs[slot].status & ~AC_INFO_POSITION_MASK)
                        | (1u << AC_INFO_POS_LLA_F);
  ti_acs[slot].itow = gps_tow_from_sys_ticks(sys_time.nb_tick);
  traffic_info_touch_position(slot);
}

/** Set position from ENU coordinates (float).
* @param[in] ac_id aircraft id of aircraft info to set
* @param[in] enu_pos position in ENU (float)
*/
static inline void acInfoSetPositionEnu_f(uint8_t ac_id, struct EnuCoor_f *enu_pos)
{
  const uint8_t slot = ti_acs_slot(ac_id);
  if (slot == TI_ACS_NONE) {
    return;
  }
  VECT3_COPY(ti_acs[slot].enu_pos_f, *enu_pos);
  /* clear bits for all position representations and only set the new one */
  ti_acs[slot].status = (ti_acs[slot].status & ~AC_INFO_POSITION_MASK)
                        | (1u << AC_INFO_POS_ENU_F);
  ti_acs[slot].itow = gps_tow_from_sys_ticks(sys_time.nb_tick);
  traffic_info_touch_position(slot);
}

/** Set velocity from ENU coordinates (int).
* @param[in] ac_id aircraft id of aircraft info to set
* @param[in] enu_vel velocity in ENU (int)
*/
static inline void acInfoSetVelocityEnu_i(uint8_t ac_id, struct EnuCoor_i *enu_vel)
{
  const uint8_t slot = ti_acs_slot(ac_id);
  if (slot == TI_ACS_NONE) {
    return;
  }
  VECT3_COPY(ti_acs[slot].enu_vel_i, *enu_vel);
  /* Keep exactly one velocity representation authoritative. */
  ti_acs[slot].status = (ti_acs[slot].status & ~AC_INFO_VELOCITY_MASK)
                        | (1u << AC_INFO_VEL_ENU_I);
  ti_acs[slot].itow = gps_tow_from_sys_ticks(sys_time.nb_tick);
  traffic_info_touch_velocity(slot);
}

/** Set velocity from ENU coordinates (float).
 * @param[in] ac_id aircraft id of aircraft info to set
 * @param[in] enu_vel velocity in ENU (float)
 */
static inline void acInfoSetVelocityEnu_f(uint8_t ac_id, struct EnuCoor_f *enu_vel)
{
  const uint8_t slot = ti_acs_slot(ac_id);
  if (slot == TI_ACS_NONE) {
    return;
  }
  VECT3_COPY(ti_acs[slot].enu_vel_f, *enu_vel);
  /* Keep exactly one velocity representation authoritative. */
  ti_acs[slot].status = (ti_acs[slot].status & ~AC_INFO_VELOCITY_MASK)
                        | (1u << AC_INFO_VEL_ENU_F);
  ti_acs[slot].itow = gps_tow_from_sys_ticks(sys_time.nb_tick);
  traffic_info_touch_velocity(slot);
}


/*************** Reference frame conversion functions ***************/

/** Lazily derive missing traffic representations.
 *
 * Conversion requires a valid source representation and, for local frames,
 * an initialized ownship origin. These legacy functions return no status;
 * callers must inspect the requested validity bit afterward. Safety-critical
 * consumers should prefer traffic_info_get_snapshot().
 */
extern void acInfoCalcPositionUtm_i(uint8_t ac_id);
extern void acInfoCalcPositionUtm_f(uint8_t ac_id);
extern void acInfoCalcPositionLla_i(uint8_t ac_id);
extern void acInfoCalcPositionLla_f(uint8_t ac_id);
extern void acInfoCalcPositionEnu_i(uint8_t ac_id);
extern void acInfoCalcPositionEnu_f(uint8_t ac_id);
extern void acInfoCalcVelocityEnu_i(uint8_t ac_id);
extern void acInfoCalcVelocityEnu_f(uint8_t ac_id);

/************************ Get functions ****************************/

/** Get position from UTM coordinates (int).
 * @param[in] ac_id aircraft id of aircraft info to get
 */
static inline struct UtmCoor_i *acInfoGetPositionUtm_i(uint8_t ac_id)
{
  const uint8_t slot = ti_acs_legacy_read_slot(ac_id);
  if (!bit_is_set(ti_acs[slot].status, AC_INFO_POS_UTM_I)) {
    acInfoCalcPositionUtm_i(ac_id);
  }
  return &ti_acs[slot].utm_pos_i;
}

/** Get position from LLA coordinates (int).
 * @param[in] ac_id aircraft id of aircraft info to get
 */
static inline struct LlaCoor_i *acInfoGetPositionLla_i(uint8_t ac_id)
{
  const uint8_t slot = ti_acs_legacy_read_slot(ac_id);
  if (!bit_is_set(ti_acs[slot].status, AC_INFO_POS_LLA_I)) {
    acInfoCalcPositionLla_i(ac_id);
  }
  return &ti_acs[slot].lla_pos_i;
}

/** Get position in local ENU coordinates (int).
 * @param[in] ac_id aircraft id of aircraft info to get
 */
static inline struct EnuCoor_i *acInfoGetPositionEnu_i(uint8_t ac_id)
{
  const uint8_t slot = ti_acs_legacy_read_slot(ac_id);
  if (!bit_is_set(ti_acs[slot].status, AC_INFO_POS_ENU_I)) {
    acInfoCalcPositionEnu_i(ac_id);
  }
  return &ti_acs[slot].enu_pos_i;
}

/** Get position from UTM coordinates (float).
 * @param[in] ac_id aircraft id of aircraft info to get
 */
static inline struct UtmCoor_f *acInfoGetPositionUtm_f(uint8_t ac_id)
{
  const uint8_t slot = ti_acs_legacy_read_slot(ac_id);
  if (!bit_is_set(ti_acs[slot].status, AC_INFO_POS_UTM_F)) {
    acInfoCalcPositionUtm_f(ac_id);
  }
  return &ti_acs[slot].utm_pos_f;
}

/** Get position from LLA coordinates (float).
 * @param[in] ac_id aircraft id of aircraft info to get
 */
static inline struct LlaCoor_f *acInfoGetPositionLla_f(uint8_t ac_id)
{
  const uint8_t slot = ti_acs_legacy_read_slot(ac_id);
  if (!bit_is_set(ti_acs[slot].status, AC_INFO_POS_LLA_F)) {
    acInfoCalcPositionLla_f(ac_id);
  }
  return &ti_acs[slot].lla_pos_f;
}

/** Get position in local ENU coordinates (float).
 * @param[in] ac_id aircraft id of aircraft info to get
 */
static inline struct EnuCoor_f *acInfoGetPositionEnu_f(uint8_t ac_id)
{
  const uint8_t slot = ti_acs_legacy_read_slot(ac_id);
  if (!bit_is_set(ti_acs[slot].status, AC_INFO_POS_ENU_F)) {
    acInfoCalcPositionEnu_f(ac_id);
  }
  return &ti_acs[slot].enu_pos_f;
}

/** Get velocity in local ENU coordinates (integer BFP).
 * @param[in] ac_id aircraft id of aircraft info to get
 */
static inline struct EnuCoor_i *acInfoGetVelocityEnu_i(uint8_t ac_id)
{
  const uint8_t slot = ti_acs_legacy_read_slot(ac_id);
  if (!bit_is_set(ti_acs[slot].status, AC_INFO_VEL_ENU_I)) {
    acInfoCalcVelocityEnu_i(ac_id);
  }
  return &ti_acs[slot].enu_vel_i;
}

/** Get velocity in local ENU coordinates (float).
 * @param[in] ac_id aircraft id of aircraft info to get
 */
static inline struct EnuCoor_f *acInfoGetVelocityEnu_f(uint8_t ac_id)
{
  const uint8_t slot = ti_acs_legacy_read_slot(ac_id);
  if (!bit_is_set(ti_acs[slot].status, AC_INFO_VEL_ENU_F)) {
    acInfoCalcVelocityEnu_f(ac_id);
  }
  return &ti_acs[slot].enu_vel_f;
}

/** Get vehicle course (float).
 * @param[in] ac_id aircraft id of aircraft info to get
 */
static inline float acInfoGetCourse(uint8_t ac_id)
{
  return ti_acs[ti_acs_legacy_read_slot(ac_id)].course;
}

/** Get vehicle ground speed (float).
 * @param[in] ac_id aircraft id of aircraft info to get
 */
static inline float acInfoGetGspeed(uint8_t ac_id)
{
  return ti_acs[ti_acs_legacy_read_slot(ac_id)].gspeed;
}

/** Get vehicle climb speed (float).
 * @param[in] ac_id aircraft id of aircraft info to get
 */
static inline float acInfoGetClimb(uint8_t ac_id)
{
  return ti_acs[ti_acs_legacy_read_slot(ac_id)].climb;
}

/** Get time of week from latest message (ms).
 * @param[in] ac_id aircraft id of aircraft info to get
 */
static inline uint32_t acInfoGetItow(uint8_t ac_id)
{
  return ti_acs[ti_acs_legacy_read_slot(ac_id)].itow;
}

// Logging functions
#define traffic_info_log() {}
extern void traffic_info_log_start(void);
extern void traffic_info_log_stop(void);

/** @} */

#endif
