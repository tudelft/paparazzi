/*
 * Copyright (C) Pascal Brisset, Antoine Drouin (2008), Kirk Scheper (2016), (OpenUAS 2026)
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

#if TRAFFIC_INFO_USE_MESH
/**
 * @defgroup mesh_state Optional MESH_STATE transport
 * @brief Compact state exchange for narrowband broadcast mesh radios.
 *
 * MESH_STATE is a 17 byte (25 bytes on the wire) replacement for ACINFO_LLA,
 * designed for the EByte E52-xxxNWxxS class of LoRa MESH modems running in
 * broadcast mode.  See the message definition in messages.xml for the exact
 * bit layout.
 *
 * Two mechanisms protect the modem's 5-frame transmit cache:
 *
 *  1. a GPS time-of-week synchronised TDMA slot, so that at most one node in
 *     the whole mesh originates a broadcast at any instant.  Without it, k
 *     simultaneous originations put k relay frames into *every* node's cache
 *     at once, which is exactly how "OUT OF CACHE" (and the resulting global
 *     buffer flush) happens;
 *  2. a leaky-bucket governor that tracks how many frames are estimated to be
 *     still inside the modem and refuses to hand over a new one above the high
 *     water mark.
 *
 * Enable this group with @c TRAFFIC_INFO_USE_MESH. Both mechanisms are O(1),
 * branch-light, and use only statically allocated storage.
 * @{
 */

/** Length of one TDMA superframe in ms.
 *
 *  Fixed and identical on every node. It is NOT tied to the node count: with
 *  nodes joining and leaving freely there is no fixed count to tie it to. */
#ifndef MESH_TDMA_SUPERFRAME_MS
#define MESH_TDMA_SUPERFRAME_MS 12000
#endif

/** Slots per superframe, i.e. the maximum number of nodes the mesh can carry
 *  at full membership. Sized well above the expected population so that
 *  arrivals always find a free slot. */
#ifndef MESH_TDMA_NB_SLOTS
#define MESH_TDMA_NB_SLOTS 32
#endif

/** Maximum slots one node may occupy when the mesh is sparsely populated.
 *  This is what converts spare membership into update rate: with N slots and
 *  k nodes present each node claims up to N/k of them, so the position rate
 *  rises automatically as nodes leave and falls back as they return. Channel
 *  occupancy stays constant either way - the frame is always full. */
#ifndef MESH_TDMA_MAX_REUSE
#define MESH_TDMA_MAX_REUSE 4
#endif

/** Superframes a node listens before claiming more than its primary slot.
 *
 *  Network entry. Without it a fleet powering up together would each see an
 *  empty map, each conclude it was alone, each grab the maximum share, and
 *  collide on every slot - and since a collision delivers nothing, none of
 *  them would ever discover the others. AIS solves it the same way. */
#ifndef MESH_ENTRY_FRAMES
#define MESH_ENTRY_FRAMES 3
#endif

/** Lease on the PRIMARY slot, in superframes. Long: it exists only to break a
 *  primary-versus-primary deadlock, where the two nodes are mutually deaf and
 *  no evidence of the clash is available to anybody. In a healthy mesh a node
 *  keeps its slot across the whole flight. */
#ifndef MESH_PRIMARY_HOLD_MIN
#define MESH_PRIMARY_HOLD_MIN 10
#endif
#ifndef MESH_PRIMARY_HOLD_SPAN
#define MESH_PRIMARY_HOLD_SPAN 10
#endif

/** Lease on an opportunistic (secondary) slot, in superframes: held for
 *  MIN..MIN+SPAN-1 and then surrendered. Randomised per node so that two nodes
 *  which ended up sharing a slot - the one collision nobody can observe,
 *  because a collision is silent to everybody - lapse at different times and
 *  diverge. The same mechanism AIS uses to time out slot reservations. */
#ifndef MESH_SLOT_HOLD_MIN
#define MESH_SLOT_HOLD_MIN 6
#endif
#ifndef MESH_SLOT_HOLD_SPAN
#define MESH_SLOT_HOLD_SPAN 8
#endif

/** Superframes a slot stays reserved after its owner was last heard.
 *  Long enough to ride out a few lost frames, short enough that a landed or
 *  departed node releases its slot promptly. */
#ifndef MESH_SLOT_AGE_FRAMES
#define MESH_SLOT_AGE_FRAMES 4
#endif

/** Superframes between one-rank rotations of remainder-slot entitlement.
 *
 * The interval exceeds quiet-slot ageing plus a serialized expansion turn for
 * every possible member. Only one rank enters and one leaves the winner set at
 * each boundary, avoiding fleet-wide claim churn. */
#ifndef MESH_REMAINDER_EPOCH_FRAMES
#define MESH_REMAINDER_EPOCH_FRAMES \
  (4 * MESH_SLOT_AGE_FRAMES + 2 * MESH_TDMA_NB_SLOTS + 1)
#endif

/** Nominal slot length in ms. Informational only: it can be fractional, so the
 *  slot index is computed by exact integer scaling in traffic_info.c rather
 *  than by dividing through this. */
#define MESH_TDMA_SLOT_MS (MESH_TDMA_SUPERFRAME_MS / MESH_TDMA_NB_SLOTS)

/** Maximum age of valid mesh kinematics before another source may take over. */
#ifndef TRAFFIC_INFO_MESH_DROP_MS
#define TRAFFIC_INFO_MESH_DROP_MS \
  (2u * MESH_TDMA_SUPERFRAME_MS + MESH_TDMA_SLOT_MS + 1000u)
#endif

/** Marker for "no node owns this slot".
 *
 * NOT zero. Zero is the ground station's AC_ID, a real and needed identity, and
 * a node that used 0 for "empty" would be unable to represent the ground
 * station in the slot map at all - it would read every slot the GCS occupies as
 * free and transmit straight over it.
 *
 * This module's enforced ID policy reserves 0xFF for PPRZLink broadcast and
 * internal sentinels. The aircraft generator and airborne compile guard both
 * reject AC_ID 255, so it can never be a valid mesh sender or slot owner.
 */
#define MESH_SLOT_FREE TRAFFIC_INFO_RESERVED_ID

/** First slot this node prefers when selecting a primary slot.
 *
 * This is a starting hint, not a static assignment. Runtime observation and
 * conflict resolution decide which slot is ultimately owned.
 */
#ifndef MESH_TDMA_SLOT_HINT
#define MESH_TDMA_SLOT_HINT ((AC_ID) % MESH_TDMA_NB_SLOTS)
#endif

#if MESH_TDMA_SUPERFRAME_MS < MESH_TDMA_NB_SLOTS
#error "MESH_TDMA_SUPERFRAME_MS is too short for MESH_TDMA_NB_SLOTS"
#endif
#if MESH_TDMA_NB_SLOTS > 255
#error "MESH_TDMA_NB_SLOTS must fit in uint8_t"
#endif
#if MESH_TDMA_MAX_REUSE < 1
#error "MESH_TDMA_MAX_REUSE must be at least 1"
#endif
#if MESH_REMAINDER_EPOCH_FRAMES <= (2 * MESH_SLOT_AGE_FRAMES + MESH_TDMA_NB_SLOTS)
#error "MESH_REMAINDER_EPOCH_FRAMES is too short for safe remainder rotation"
#endif

/** Maximum GPS-to-monotonic clock correction tolerated without relearning slots. */
#ifndef MESH_CLOCK_STEP_MAX_MS
#define MESH_CLOCK_STEP_MAX_MS 10
#endif

/** Estimated time for one frame to leave the modem transmit cache, in ms.
 *  Air time of the longest frame plus the worst case CSMA back-off plus one
 *  relay of a neighbour's frame. */
#ifndef MESH_MODEM_DRAIN_MS
#define MESH_MODEM_DRAIN_MS 60
#endif

/** Never hand a frame to the modem when this many are estimated to be still
 *  queued inside it. The hardware limit is 5. */
#ifndef MESH_CACHE_HIGH_WATER
#define MESH_CACHE_HIGH_WATER 3
#endif

#if MESH_CACHE_HIGH_WATER >= 5
#error "MESH_CACHE_HIGH_WATER must stay below the 5 frame hardware cache"
#endif

/** Unified, firmware-independent mode stored in the MESH_STATE flags field. */
#define MESH_MODE_MANUAL    0u
#define MESH_MODE_ASSISTED  1u
#define MESH_MODE_AUTO      2u
#define MESH_MODE_HOME      3u
#define MESH_MODE_NOGPS     4u
#define MESH_MODE_FAILSAFE  5u
#define MESH_MODE_KILL      6u
#define MESH_MODE_UNKNOWN   7u

#define MESH_FLAG_MODE_MASK  0x07u
#define MESH_FLAG_ROTORCRAFT 0x08u
#define MESH_FLAG_POS_VALID  0x10u
#define MESH_FLAG_AIRBORNE   0x20u
#define MESH_FLAG_ALERT      0x40u
#define MESH_FLAG_EMERGENCY  0x80u

/** One TDMA slot's observed owner.
 *
 * Ownership is *learned*, never configured: a frame's slot is implied by its
 * arrival time, which every node agrees on because the frame is GPS aligned.
 * So the occupancy map costs zero bytes on air.
 */
struct MeshSlot {
  uint8_t  ac_id;        ///< Observed owner, or MESH_SLOT_FREE when unowned.
  uint32_t last_frame;   ///< superframe index when last heard
};

/** Health and back-pressure state of the mesh link.
 *  Statically allocated, updated only from the module task and the telemetry
 *  callback, both of which run in the main loop context.
 */
struct MeshLinkState {
  uint32_t modem_free_ms;    ///< sys time at which the modem cache is expected to be empty
  uint32_t last_emit_key;    ///< superframe*NB_SLOTS + slot of the last origination
  uint16_t tx_count;         ///< MESH_STATE frames handed to the modem
  uint16_t defer_ticks;      ///< module task iterations spent waiting for a slot
  uint16_t throttled_count;  ///< frames held back by the cache governor
  uint16_t reselect_count;   ///< times this node had to move to a different slot
  uint8_t  slot;             ///< own primary slot
  uint8_t  reuse;            ///< slots this node currently claims (1..MESH_TDMA_MAX_REUSE)
  uint8_t  neighbours;       ///< distinct nodes heard in the last age window
  bool     ready;            ///< the telemetry transport is known
  bool     synced;           ///< GPS time of week is usable for slotting
};

extern struct MeshLinkState mesh_link;

/** Record a received MESH_STATE in the inferred TDMA slot map.
 * @param[in] sender Originating aircraft ID; AC_ID 0 is the GCS.
 * @param[in] net_ms GPS-aligned absolute network timestamp in milliseconds.
 */
extern void mesh_slot_observe(uint8_t sender, uint64_t net_ms);

/** Periodic task driving the mesh transmit slot. Call at 20 Hz or faster. */
extern void traffic_info_mesh_periodic(void);

/** @} */
#endif /* TRAFFIC_INFO_USE_MESH */

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
#define AC_INFO_SOURCE_MESH 9

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
extern bool traffic_info_capacity_exceeded;
extern bool traffic_info_surveillance_established;

extern void traffic_info_init(void);

/**
 * Resolve an aircraft id to its slot in ::ti_acs, inserting it if needed.
 *
 * Replaces the open coded `if (ti_acs_idx < NB_ACS) { ... }` guard that used
 * to wrap every setter. That guard also blocked *updates to already known*
 * aircraft once the table was full, which silently froze the whole traffic
 * picture instead of only refusing the new arrival.
 *
 * @param[in] id aircraft id, 0 is the GCS and always maps to slot 0
 * @return slot index in ::ti_acs, or #TI_ACS_NONE when the aircraft is unknown
 *         and the table is full
 */
static inline uint8_t ti_acs_slot(uint8_t id)
{
  if (!traffic_info_id_valid(id)) {
    return TI_ACS_NONE;
  }
  uint8_t slot = ti_acs_id[id];
  if (slot == 0 && id != 0) {         /* not registered yet */
    if (ti_acs_idx >= NB_ACS) {
      traffic_info_capacity_exceeded = true;
      return TI_ACS_NONE;             /* table full, refuse the new arrival */
    }
    slot = ti_acs_idx++;
    ti_acs_id[id] = slot;
    ti_acs[slot].ac_id = id;
  }
  return slot;
}

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

/** Return local monotonic age of the latest accepted traffic observation.
 *
 * This age is independent of GPS time-of-week rollover and is suitable for
 * safety freshness decisions. It is updated only when a setter accepts the
 * observation, not when an out-of-order packet is rejected.
 *
 * @param[in] ac_id Traffic ID to query.
 * @param[out] age_ms Age in milliseconds, saturated at UINT32_MAX.
 * @return @c true when the traffic record has an accepted observation.
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

/** Mark a traffic-table slot as having received complete position/velocity. */
extern void traffic_info_touch(uint8_t slot);

/** Mark only the position component of a traffic observation as current. */
extern void traffic_info_touch_position(uint8_t slot);

/** Mark only the velocity component of a traffic observation as current. */
extern void traffic_info_touch_velocity(uint8_t slot);

#if TRAFFIC_INFO_USE_MESH
/** Copy a mesh observation and project it forward at constant velocity.
 *
 * The stored observation is never modified. Prediction starts from the latest
 * valid MESH_STATE position on every call and is clamped to @p max_prediction_ms.
 *
 * @param[in] ac_id Aircraft ID to query.
 * @param[in] max_prediction_ms Maximum constant-velocity projection interval.
 * @param[out] position Predicted local ENU position in meters.
 * @param[out] velocity Observed local ENU velocity in meters per second.
 * @param[out] age_ms Local monotonic age of the observation in milliseconds.
 * @return @c true for a valid mesh observation, otherwise @c false.
 */
extern bool traffic_info_get_mesh_snapshot(uint8_t ac_id, uint32_t max_prediction_ms,
                                           struct EnuCoor_f *position,
                                           struct EnuCoor_f *velocity,
                                           uint32_t *age_ms);

/** Return the age of an aircraft's last valid MESH_STATE observation.
 *
 * Unlike heartbeat reception age, this timestamp is not refreshed by an
 * invalid-position frame. Safety consumers can therefore distinguish a short
 * sensor outage from a track whose last usable kinematics have expired.
 *
 * @param[in] ac_id Aircraft ID to query.
 * @param[out] age_ms Local monotonic age of the last valid observation.
 * @return @c true when this mesh track has had a valid observation.
 */
extern bool traffic_info_get_mesh_valid_age(uint8_t ac_id, uint32_t *age_ms);

/** Return the latest raw MESH_STATE flags for a mesh peer.
 *
 * The rotorcraft bit is retained for mixed-fleet observability. TCAS does not
 * branch on aircraft type; all avoidance geometry and advisories remain shared.
 */
extern bool traffic_info_get_mesh_flags(uint8_t ac_id, uint8_t *flags);

/** Return whether the latest observation for an aircraft came from MESH_STATE. */
extern bool traffic_info_is_mesh_track(uint8_t ac_id);
#endif

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
 */
extern void set_ac_info_lla(uint8_t id, int32_t lat, int32_t lon, int32_t alt,
                            int16_t course, uint16_t gspeed, int16_t climb, uint32_t itow);

/** Set LLA traffic received without a source timestamp.
 *
 * Arrival-stamped observations never replace a source-TOW ordered stream.
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
  /* clear bits for all position representations and only set the new one */
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
  /* clear bits for all position representations and only set the new one */
  ti_acs[slot].status = (ti_acs[slot].status & ~AC_INFO_VELOCITY_MASK)
                        | (1u << AC_INFO_VEL_ENU_F);
  ti_acs[slot].itow = gps_tow_from_sys_ticks(sys_time.nb_tick);
  traffic_info_touch_velocity(slot);
}


/*************** Reference frame conversion functions ***************/

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
  if (!bit_is_set(ti_acs[ti_acs_id[ac_id]].status, AC_INFO_POS_UTM_I)) {
    acInfoCalcPositionUtm_i(ac_id);
  }
  return &ti_acs[ti_acs_id[ac_id]].utm_pos_i;
}

/** Get position from LLA coordinates (int).
 * @param[in] ac_id aircraft id of aircraft info to get
 */
static inline struct LlaCoor_i *acInfoGetPositionLla_i(uint8_t ac_id)
{
  if (!bit_is_set(ti_acs[ti_acs_id[ac_id]].status, AC_INFO_POS_LLA_I)) {
    acInfoCalcPositionLla_i(ac_id);
  }
  return &ti_acs[ti_acs_id[ac_id]].lla_pos_i;
}

/** Get position in local ENU coordinates (int).
 * @param[in] ac_id aircraft id of aircraft info to get
 */
static inline struct EnuCoor_i *acInfoGetPositionEnu_i(uint8_t ac_id)
{
  if (!bit_is_set(ti_acs[ti_acs_id[ac_id]].status, AC_INFO_POS_ENU_I)) {
    acInfoCalcPositionEnu_i(ac_id);
  }
  return &ti_acs[ti_acs_id[ac_id]].enu_pos_i;
}

/** Get position from UTM coordinates (float).
 * @param[in] ac_id aircraft id of aircraft info to get
 */
static inline struct UtmCoor_f *acInfoGetPositionUtm_f(uint8_t ac_id)
{
  if (!bit_is_set(ti_acs[ti_acs_id[ac_id]].status, AC_INFO_POS_UTM_F)) {
    acInfoCalcPositionUtm_f(ac_id);
  }
  return &ti_acs[ti_acs_id[ac_id]].utm_pos_f;
}

/** Get position from LLA coordinates (float).
 * @param[in] ac_id aircraft id of aircraft info to get
 */
static inline struct LlaCoor_f *acInfoGetPositionLla_f(uint8_t ac_id)
{
  if (!bit_is_set(ti_acs[ti_acs_id[ac_id]].status, AC_INFO_POS_LLA_F)) {
    acInfoCalcPositionLla_f(ac_id);
  }
  return &ti_acs[ti_acs_id[ac_id]].lla_pos_f;
}

/** Get position in local ENU coordinates (float).
 * @param[in] ac_id aircraft id of aircraft info to get
 */
static inline struct EnuCoor_f *acInfoGetPositionEnu_f(uint8_t ac_id)
{
  if (!bit_is_set(ti_acs[ti_acs_id[ac_id]].status, AC_INFO_POS_ENU_F)) {
    acInfoCalcPositionEnu_f(ac_id);
  }
  return &ti_acs[ti_acs_id[ac_id]].enu_pos_f;
}

/** Get velocity in local ENU coordinates (integer BFP).
 * @param[in] ac_id aircraft id of aircraft info to get
 */
static inline struct EnuCoor_i *acInfoGetVelocityEnu_i(uint8_t ac_id)
{
  if (!bit_is_set(ti_acs[ti_acs_id[ac_id]].status, AC_INFO_VEL_ENU_I)) {
    acInfoCalcVelocityEnu_i(ac_id);
  }
  return &ti_acs[ti_acs_id[ac_id]].enu_vel_i;
}

/** Get velocity in local ENU coordinates (float).
 * @param[in] ac_id aircraft id of aircraft info to get
 */
static inline struct EnuCoor_f *acInfoGetVelocityEnu_f(uint8_t ac_id)
{
  if (!bit_is_set(ti_acs[ti_acs_id[ac_id]].status, AC_INFO_VEL_ENU_F)) {
    acInfoCalcVelocityEnu_f(ac_id);
  }
  return &ti_acs[ti_acs_id[ac_id]].enu_vel_f;
}

/** Get vehicle course (float).
 * @param[in] ac_id aircraft id of aircraft info to get
 */
static inline float acInfoGetCourse(uint8_t ac_id)
{
  return ti_acs[ti_acs_id[ac_id]].course;
}

/** Get vehicle ground speed (float).
 * @param[in] ac_id aircraft id of aircraft info to get
 */
static inline float acInfoGetGspeed(uint8_t ac_id)
{
  return ti_acs[ti_acs_id[ac_id]].gspeed;
}

/** Get vehicle climb speed (float).
 * @param[in] ac_id aircraft id of aircraft info to get
 */
static inline float acInfoGetClimb(uint8_t ac_id)
{
  return ti_acs[ti_acs_id[ac_id]].climb;
}

/** Get time of week from latest message (ms).
 * @param[in] ac_id aircraft id of aircraft info to get
 */
static inline uint32_t acInfoGetItow(uint8_t ac_id)
{
  return ti_acs[ti_acs_id[ac_id]].itow;
}

// Logging functions
#define traffic_info_log() {}
extern void traffic_info_log_start(void);
extern void traffic_info_log_stop(void);

/** @} */

#endif
