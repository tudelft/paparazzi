/*
 * Copyright (C) 2010 ENAC
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

/** \file tcas.c
 *  \brief Collision avoidance library
 *
 */

#include "multi/tcas.h"
#include "multi/tcas_policy.h"
#include "state.h"
#include "firmwares/fixedwing/nav.h"
#include "generated/flight_plan.h"  // SECURITY_HEIGHT

#include "modules/datalink/downlink.h"
#include <math.h>

#if !FIXEDWING_FIRMWARE
#error "The TCAS command backend is currently implemented only for fixed-wing firmware"
#endif

float tcas_alt_setpoint;
float tcas_tau_ta, tcas_tau_ra, tcas_dmod, tcas_alim;

uint8_t tcas_status;
enum tcas_resolve tcas_resolve;
uint8_t tcas_ac_RA;
struct tcas_ac_status tcas_acs_status[NB_ACS];
static bool tcas_command_valid;
static bool tcas_resolve_received[NB_ACS];
static uint32_t tcas_resolve_received_ms[NB_ACS];

#ifndef TCAS_TAU_TA     // Traffic Advisory
#define TCAS_TAU_TA 2*CARROT
#endif

#ifndef TCAS_TAU_RA     // Resolution Advisory
#define TCAS_TAU_RA CARROT
#endif

#ifndef TCAS_DMOD       // Distance Modification
#define TCAS_DMOD 10.
#endif

#ifndef TCAS_ALIM       // Altitude Limit
#define TCAS_ALIM 15.
#endif

#ifndef TCAS_DT_MAX     // ms (lost com and timeout)
#define TCAS_DT_MAX 1500
#endif

#if TRAFFIC_INFO_USE_MESH
/** Constant-velocity prediction is bounded by the traffic-advisory horizon.
 * Beyond it, maneuver uncertainty outweighs the value of extrapolation. */
#ifndef TCAS_MESH_PREDICTION_MAX_MS
#define TCAS_MESH_PREDICTION_MAX_MS ((uint32_t)(TCAS_TAU_TA * 1000.f))
#endif

/** Stop opening or changing advisories after the prediction horizon. */
#ifndef TCAS_MESH_FRESH_MS
#define TCAS_MESH_FRESH_MS TCAS_MESH_PREDICTION_MAX_MS
#endif

/** Clear a held advisory after two complete update opportunities were missed. */
#ifndef TCAS_MESH_DROP_MS
#define TCAS_MESH_DROP_MS TRAFFIC_INFO_MESH_DROP_MS
#endif

_Static_assert(TCAS_MESH_FRESH_MS <= TCAS_MESH_PREDICTION_MAX_MS,
               "TCAS mesh freshness cannot exceed the prediction horizon");
_Static_assert(TCAS_MESH_DROP_MS > TCAS_MESH_FRESH_MS,
               "TCAS mesh drop time must exceed its freshness interval");
#endif

#define TCAS_HUGE_TAU 100*TCAS_TAU_TA

enum tcas_track_quality {
  TCAS_TRACK_VALID,
  TCAS_TRACK_INVALID
};

static bool tcas_ownship_geometry_valid(void)
{
  return stateIsGlobalCoordinateValid()
         && stateIsLocalCoordinateValid()
         && (state.speed_status & SPEED_LOCAL_COORD) != 0
         && isfinite(stateGetPositionUtm_f()->alt)
         && isfinite(stateGetPositionEnu_f()->x)
         && isfinite(stateGetPositionEnu_f()->y)
         && isfinite(stateGetPositionEnu_f()->z)
         && tcas_velocity_is_usable(stateGetSpeedEnu_f()->x,
                                    stateGetSpeedEnu_f()->y,
                                    stateGetSpeedEnu_f()->z)
         && isfinite(nav_altitude);
}

void callTCAS(void)
{
  if (tcas_status == TCAS_RA && tcas_command_valid
      && tcas_ownship_geometry_valid()) {
    v_ctl_altitude_setpoint = tcas_alt_setpoint;
  }
}

/* AC is inside the horizontol dmod area and twice the vertical alim separation */
#define TCAS_IsInside() ( (ddh < Square(tcas_dmod) && ddv < Square(2*tcas_alim)) ? 1 : 0 )

void tcas_init(void)
{
  tcas_alt_setpoint = ground_alt + SECURITY_HEIGHT;
  tcas_tau_ta = TCAS_TAU_TA;
  tcas_tau_ra = TCAS_TAU_RA;
  tcas_dmod = TCAS_DMOD;
  tcas_alim = TCAS_ALIM;
  tcas_status = TCAS_UNAVAILABLE;
  tcas_resolve = RA_NONE;
  tcas_ac_RA = AC_ID;
  tcas_command_valid = false;
  uint8_t i;
  for (i = 0; i < NB_ACS; i++) {
    tcas_acs_status[i].status = TCAS_NO_ALARM;
    tcas_acs_status[i].resolve = RA_NONE;
    tcas_resolve_received[i] = false;
    tcas_resolve_received_ms[i] = 0;
  }
}

void parseTcasResolve(uint8_t *buf)
{
  if (DL_TCAS_RESOLVE_ac_id(buf) == AC_ID) {
    const uint8_t ac_id_conflict = DL_TCAS_RESOLVE_ac_id_conflict(buf);
    const uint8_t resolve = DL_TCAS_RESOLVE_resolve(buf);
    if (resolve > RA_DESCEND) {
      return;
    }
    if (!traffic_info_aircraft_id_valid(ac_id_conflict)) {
      return;
    }
    const uint8_t slot = ti_acs_id[ac_id_conflict];
    if (slot < ti_acs_idx && ti_acs[slot].ac_id == ac_id_conflict
      && (tcas_acs_status[slot].status == TCAS_TA
        || tcas_acs_status[slot].status == TCAS_RA)) {
      tcas_acs_status[slot].resolve = resolve;
      tcas_resolve_received[slot] = resolve != RA_NONE;
      tcas_resolve_received_ms[slot] = get_sys_time_msec();
    }
  }
}

void parseTcasRA(uint8_t *buf)
{
  if (DL_TCAS_RA_ac_id(buf) == AC_ID && SenderIdOfPprzMsg(buf) != AC_ID) {
    const uint8_t ac_id_conflict = SenderIdOfPprzMsg(buf);
    const uint8_t resolve = DL_TCAS_RA_resolve(buf);
    if (resolve > RA_DESCEND) {
      return;
    }
    if (!traffic_info_aircraft_id_valid(ac_id_conflict)) {
      return;
    }
    const uint8_t slot = ti_acs_id[ac_id_conflict];
    if (slot < ti_acs_idx && ti_acs[slot].ac_id == ac_id_conflict
      && (tcas_acs_status[slot].status == TCAS_TA
        || tcas_acs_status[slot].status == TCAS_RA)) {
      tcas_acs_status[slot].resolve = resolve;
      tcas_resolve_received[slot] = resolve != RA_NONE;
      tcas_resolve_received_ms[slot] = get_sys_time_msec();
    }
  }
}

static enum tcas_track_quality tcas_get_track(uint8_t id,
                                               struct EnuCoor_f *position,
                                               struct EnuCoor_f *velocity,
                                               uint32_t *age_ms,
                                               bool *is_mesh)
{
#if TRAFFIC_INFO_USE_MESH
  if (traffic_info_get_mesh_snapshot(id, TCAS_MESH_PREDICTION_MAX_MS,
                                     position, velocity, age_ms)) {
    *is_mesh = true;
    return TCAS_TRACK_VALID;
  }
  if (traffic_info_is_mesh_track(id)) {
    *is_mesh = true;
    if (!traffic_info_get_mesh_valid_age(id, age_ms)) {
      *age_ms = UINT32_MAX;
    }
    return TCAS_TRACK_INVALID;
  }
#endif
  *is_mesh = false;
  if (!traffic_info_get_age(id, age_ms)) {
    *age_ms = UINT32_MAX;
    return TCAS_TRACK_INVALID;
  }
  const uint8_t slot = ti_acs_id[id];
  const uint16_t position_mask = (1u << AC_INFO_POS_UTM_I)
                                 | (1u << AC_INFO_POS_LLA_I)
                                 | (1u << AC_INFO_POS_ENU_I)
                                 | (1u << AC_INFO_POS_UTM_F)
                                 | (1u << AC_INFO_POS_LLA_F)
                                 | (1u << AC_INFO_POS_ENU_F);
  const uint16_t velocity_mask = (1u << AC_INFO_VEL_ENU_I)
                                 | (1u << AC_INFO_VEL_ENU_F)
                                 | (1u << AC_INFO_VEL_LOCAL_F);
  if ((ti_acs[slot].status & position_mask) == 0
      || (ti_acs[slot].status & velocity_mask) == 0) {
    return TCAS_TRACK_INVALID;
  }
  const bool enu_position_available = bit_is_set(ti_acs[slot].status, AC_INFO_POS_ENU_I)
                                      || bit_is_set(ti_acs[slot].status, AC_INFO_POS_ENU_F);
  if (!enu_position_available
      && !(state.ned_initialized_i || state.ned_initialized_f || state.utm_initialized_f)) {
    return TCAS_TRACK_INVALID;
  }
  *position = *acInfoGetPositionEnu_f(id);
  *velocity = *acInfoGetVelocityEnu_f(id);
    if (!isfinite(position->x) || !isfinite(position->y) || !isfinite(position->z)
      || !tcas_velocity_is_usable(velocity->x, velocity->y, velocity->z)) {
    return TCAS_TRACK_INVALID;
  }
  return TCAS_TRACK_VALID;
}

static inline enum tcas_resolve tcas_test_direction(uint8_t id)
{
  struct EnuCoor_f position;
  struct EnuCoor_f velocity;
  uint32_t age_ms;
  bool is_mesh;
  if (tcas_get_track(id, &position, &velocity, &age_ms, &is_mesh) != TCAS_TRACK_VALID) {
    return AC_ID < id ? RA_DESCEND : RA_CLIMB;
  }
  (void)velocity;
  (void)age_ms;
  (void)is_mesh;

  float dz = position.z - stateGetPositionEnu_f()->z;
  if (dz > tcas_alim / 2) { return RA_DESCEND; }
  else if (dz < -tcas_alim / 2) { return RA_CLIMB; }
  else { // AC with the smallest ID descend
    if (AC_ID < id) { return RA_DESCEND; }
    else { return RA_CLIMB; }
  }
}

/** Add an existing advisory to the stale-track candidate set. */
static bool tcas_hold_advisory(uint8_t slot, uint8_t *held_ra, uint8_t *held_ta)
{
  const uint8_t id = ti_acs[slot].ac_id;
  if (tcas_acs_status[slot].status == TCAS_RA) {
    if (*held_ra == AC_ID || id == tcas_ac_RA
        || (*held_ra != tcas_ac_RA && id < *held_ra)) {
      *held_ra = id;
    }
    return true;
  }
  if (tcas_acs_status[slot].status == TCAS_TA) {
    if (*held_ta == AC_ID || id < *held_ta) {
      *held_ta = id;
    }
    return true;
  }
  return false;
}


/* conflicts detection and monitoring */
void tcas_periodic_task_1Hz(void)
{
  if (!tcas_ownship_geometry_valid()) {
    tcas_status = TCAS_UNAVAILABLE;
    tcas_ac_RA = AC_ID;
    tcas_resolve = RA_NONE;
    tcas_command_valid = false;
    return;
  }
  // no TCAS under security_height
  if (stateGetPositionUtm_f()->alt <= ground_alt + SECURITY_HEIGHT) {
    uint8_t i;
    for (i = 0; i < NB_ACS; i++) {
      tcas_acs_status[i].status = TCAS_NO_ALARM;
      tcas_acs_status[i].resolve = RA_NONE;
      tcas_resolve_received[i] = false;
    }
    tcas_status = TCAS_UNAVAILABLE;
    tcas_resolve = RA_NONE;
    tcas_ac_RA = AC_ID;
    tcas_alt_setpoint = nav_altitude;
    tcas_command_valid = false;
    return;
  }
  // test possible conflicts
#ifdef TCAS_DEBUG
  float tau_min = TCAS_HUGE_TAU;
#endif
  float fresh_ra_score = TCAS_HUGE_TAU;
  float fresh_ta_score = TCAS_HUGE_TAU;
  uint8_t fresh_ra = AC_ID;
  uint8_t held_ra = AC_ID;
  uint8_t fresh_ta = AC_ID;
  uint8_t held_ta = AC_ID;
  bool surveillance_unavailable = traffic_info_capacity_exceeded
                                  || !traffic_info_surveillance_established;
  uint8_t i;
  float vx = stateGetHorizontalSpeedNorm_f() * sinf(stateGetHorizontalSpeedDir_f());
  float vy = stateGetHorizontalSpeedNorm_f() * cosf(stateGetHorizontalSpeedDir_f());
  /* i is a compact ti_acs[] table slot, not an AC_ID. Arbitrary aircraft IDs
   * are mapped into slots as they are first received, so IDs need not be
   * sequential or smaller than NB_ACS. Slot 0 is the GCS and slot 1 is this
   * aircraft; NB_ACS therefore provides NB_ACS - 2 remote-aircraft slots.
   * The GCS contributes mesh routing and traffic visibility, but is excluded
   * from collision avoidance; supporting GCS obstacles would require an
   * explicit stationary-track policy rather than treating it as an aircraft. */
  for (i = 2; i < NB_ACS; i++) {
    if (ti_acs[i].ac_id == TRAFFIC_INFO_GCS_ID) { continue; } // unused slot
    struct EnuCoor_f position;
    struct EnuCoor_f velocity;
    uint32_t age_ms;
    bool is_mesh;
    const enum tcas_track_quality quality =
      tcas_get_track(ti_acs[i].ac_id, &position, &velocity, &age_ms, &is_mesh);
    const bool advisory_active = tcas_acs_status[i].status == TCAS_TA
                                 || tcas_acs_status[i].status == TCAS_RA;
#if TRAFFIC_INFO_USE_MESH
    const uint32_t fresh_ms = TCAS_MESH_FRESH_MS;
    const uint32_t drop_ms = TCAS_MESH_DROP_MS;
#else
    const uint32_t fresh_ms = 0;
    const uint32_t drop_ms = 0;
    (void)is_mesh;
#endif
    const enum tcas_surveillance_action action = tcas_surveillance_action(
      quality == TCAS_TRACK_VALID, is_mesh, advisory_active, age_ms,
      fresh_ms, drop_ms, TCAS_DT_MAX);
    if (action == TCAS_SURVEILLANCE_HOLD) {
      if (tcas_hold_advisory(i, &held_ra, &held_ta)) {
        continue;
      }
    }
    if (action == TCAS_SURVEILLANCE_UNAVAILABLE) {
      tcas_acs_status[i].status = TCAS_UNAVAILABLE;
      tcas_acs_status[i].resolve = RA_NONE;
      tcas_resolve_received[i] = false;
      surveillance_unavailable = true;
      continue;
    }
    float dx = position.x - stateGetPositionEnu_f()->x;
    float dy = position.y - stateGetPositionEnu_f()->y;
    float dz = position.z - stateGetPositionEnu_f()->z;
    float dvx = vx - velocity.x;
    float dvy = vy - velocity.y;
    float dvz = stateGetSpeedEnu_f()->z - velocity.z;
    float scal = dvx * dx + dvy * dy + dvz * dz;
    float ddh = dx * dx + dy * dy;
    float ddv = dz * dz;
    float tau = TCAS_HUGE_TAU;
    if (scal > 0.) { tau = (ddh + ddv) / scal; }
    // monitor conflicts
    uint8_t inside = TCAS_IsInside();
    //enum tcas_resolve test_dir = RA_NONE;
    if (tcas_acs_status[i].status == TCAS_UNAVAILABLE) {
      tcas_acs_status[i].status = TCAS_NO_ALARM;
      tcas_resolve_received[i] = false;
    }
    switch (tcas_acs_status[i].status) {
      case TCAS_RA:
        if (tau >= TCAS_HUGE_TAU && !inside) {
          tcas_acs_status[i].status = TCAS_NO_ALARM; // conflict is now resolved
          tcas_acs_status[i].resolve = RA_NONE;
          tcas_resolve_received[i] = false;
          DOWNLINK_SEND_TCAS_RESOLVED(DefaultChannel, DefaultDevice, &(ti_acs[i].ac_id));
        }
        break;
      case TCAS_TA:
        if (tau < tcas_tau_ra || inside) {
          tcas_acs_status[i].status = TCAS_RA; // TA -> RA
          // Downlink alert
          //test_dir = tcas_test_direction(ti_acs[i].ac_id);
          //DOWNLINK_SEND_TCAS_RA(DefaultChannel, DefaultDevice,&(ti_acs[i].ac_id),&test_dir);// FIXME only one closest AC ???
          break;
        }
        if (tau > tcas_tau_ta && !inside) {
          tcas_acs_status[i].status = TCAS_NO_ALARM;  // conflict is now resolved
          tcas_acs_status[i].resolve = RA_NONE;
          tcas_resolve_received[i] = false;
          DOWNLINK_SEND_TCAS_RESOLVED(DefaultChannel, DefaultDevice, &(ti_acs[i].ac_id));
        }
        break;
      case TCAS_NO_ALARM:
        if (tau < tcas_tau_ta || inside) {
          tcas_acs_status[i].status = TCAS_TA; // NO_ALARM -> TA
          tcas_resolve_received[i] = false;
          // Downlink warning
          DOWNLINK_SEND_TCAS_TA(DefaultChannel, DefaultDevice, &(ti_acs[i].ac_id));
        }
        if (tau < tcas_tau_ra || inside) {
          tcas_acs_status[i].status = TCAS_RA; // NO_ALARM -> RA = big problem ?
          // Downlink alert
          //test_dir = tcas_test_direction(ti_acs[i].ac_id);
          //DOWNLINK_SEND_TCAS_RA(DefaultChannel, DefaultDevice,&(ti_acs[i].ac_id),&test_dir);
        }
        break;
      default:
        break;
    }

    /* Select candidates only after updating their advisory state. Penetrating
     * the protected volume outranks time-to-conflict; current RA wins a tie. */
    const float score = inside ? 0.f : tau;
    if (tcas_acs_status[i].status == TCAS_RA
        && (fresh_ra == AC_ID || score < fresh_ra_score
            || (score == fresh_ra_score && ti_acs[i].ac_id == tcas_ac_RA))) {
      fresh_ra = ti_acs[i].ac_id;
      fresh_ra_score = score;
    } else if (tcas_acs_status[i].status == TCAS_TA
               && (fresh_ta == AC_ID || score < fresh_ta_score)) {
      fresh_ta = ti_acs[i].ac_id;
      fresh_ta_score = score;
    }
  }

  uint8_t ac_id_close = AC_ID;
  bool ac_id_close_fresh = true;
  if (fresh_ra != AC_ID) {
    ac_id_close = fresh_ra;
#ifdef TCAS_DEBUG
    tau_min = fresh_ra_score;
#endif
  } else if (held_ra != AC_ID) {
    ac_id_close = held_ra;
    ac_id_close_fresh = false;
  } else if (fresh_ta != AC_ID) {
    ac_id_close = fresh_ta;
#ifdef TCAS_DEBUG
    tau_min = fresh_ta_score;
#endif
  } else if (held_ta != AC_ID) {
    ac_id_close = held_ta;
    ac_id_close_fresh = false;
  }

  if (ac_id_close == AC_ID) {
    tcas_status = surveillance_unavailable ? TCAS_UNAVAILABLE : TCAS_NO_ALARM;
    tcas_ac_RA = AC_ID;
    tcas_resolve = RA_NONE;
    tcas_command_valid = false;
    return;
  }

  tcas_status = tcas_acs_status[ti_acs_id[ac_id_close]].status;
  if (!ac_id_close_fresh) {
    if (tcas_status == TCAS_RA) {
      if (tcas_ac_RA != ac_id_close) {
        tcas_command_valid = false;
      }
      tcas_ac_RA = ac_id_close;
    } else {
      tcas_ac_RA = AC_ID;
      tcas_resolve = RA_NONE;
      tcas_command_valid = false;
    }
    return;
  }
  // at least one in conflict, deal with closest one
  if (tcas_status == TCAS_RA) {
    const uint8_t previous_ra = tcas_ac_RA;
    const enum tcas_resolve previous_resolve = tcas_resolve;
    tcas_ac_RA = ac_id_close;
    tcas_resolve = tcas_test_direction(tcas_ac_RA);
    const uint8_t ra_slot = ti_acs_id[tcas_ac_RA];
    const bool peer_resolution_fresh = tcas_resolve_received[ra_slot]
                                       && get_sys_time_msec() - tcas_resolve_received_ms[ra_slot] <= TCAS_DT_MAX;
    if (peer_resolution_fresh) {
      const enum tcas_resolve peer_resolve = tcas_acs_status[ra_slot].resolve;
      if (peer_resolve == tcas_resolve) { // same direction, lowest id descends
        tcas_resolve = AC_ID < tcas_ac_RA ? RA_DESCEND : RA_CLIMB;
      }
    } else {
      tcas_resolve_received[ra_slot] = false;
      if (tcas_resolve == RA_CLIMB && ti_acs[ra_slot].climb > 1.0) {
        tcas_resolve = RA_DESCEND;
      } else if (tcas_resolve == RA_DESCEND && ti_acs[ra_slot].climb < -1.0) {
        tcas_resolve = RA_CLIMB;
      }
    }
    if (tcas_ac_RA != previous_ra || tcas_resolve != previous_resolve) {
      tcas_command_valid = false;
    }
    // Downlink alert
    uint8_t resolve = tcas_resolve;
    DOWNLINK_SEND_TCAS_RA(DefaultChannel, DefaultDevice, &tcas_ac_RA, &resolve);
  } else {
    tcas_ac_RA = AC_ID;
    tcas_resolve = RA_NONE;
    tcas_command_valid = false;
  }
#ifdef TCAS_DEBUG
  if (tcas_status == TCAS_RA) { DOWNLINK_SEND_TCAS_DEBUG(DefaultChannel, DefaultDevice, &ac_id_close, &tau_min); }
#endif
}


/* altitude control loop */
void tcas_periodic_task_4Hz(void)
{
  if (!tcas_ownship_geometry_valid()) {
    tcas_command_valid = false;
    return;
  }
  // set alt setpoint
  if (stateGetPositionUtm_f()->alt > ground_alt + SECURITY_HEIGHT && tcas_status == TCAS_RA) {
    struct EnuCoor_f position;
    struct EnuCoor_f velocity;
    uint32_t age_ms;
    bool is_mesh;
    if (tcas_get_track(tcas_ac_RA, &position, &velocity, &age_ms, &is_mesh) != TCAS_TRACK_VALID) {
      /* Loss of track validity does not prove separation. The 1 Hz task owns
       * the bounded hold/drop decision; until then preserve the last command. */
      return;
    }
    (void)velocity;
#if TRAFFIC_INFO_USE_MESH
    const uint32_t fresh_ms = TCAS_MESH_FRESH_MS;
    const uint32_t drop_ms = TCAS_MESH_DROP_MS;
#else
    const uint32_t fresh_ms = 0;
    const uint32_t drop_ms = 0;
#endif
    const enum tcas_surveillance_action action = tcas_surveillance_action(
      true, is_mesh, true, age_ms, fresh_ms, drop_ms, TCAS_DT_MAX);
    if (action != TCAS_SURVEILLANCE_EVALUATE) {
      return; // preserve the last command until the 1 Hz task holds or drops it
    }
    const float intruder_altitude = tcas_intruder_altitude_msl(
                      stateGetPositionUtm_f()->alt,
                      position.z,
                      stateGetPositionEnu_f()->z);
    switch (tcas_resolve) {
      case RA_CLIMB :
        tcas_alt_setpoint = Max(nav_altitude, intruder_altitude + tcas_alim);
        break;
      case RA_DESCEND :
        tcas_alt_setpoint = Min(nav_altitude, intruder_altitude - tcas_alim);
        break;
      case RA_LEVEL :
      case RA_NONE :
        tcas_alt_setpoint = nav_altitude;
        break;
      default:
        break;
    }
    // Bound alt
    tcas_alt_setpoint = Max(ground_alt + SECURITY_HEIGHT, tcas_alt_setpoint);
    tcas_command_valid = isfinite(tcas_alt_setpoint);
  } else {
    tcas_alt_setpoint = nav_altitude;
    tcas_resolve = RA_NONE;
    tcas_command_valid = false;
  }
}
