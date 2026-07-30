/*
 * Copyright (C) 2026 OpenUAS
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
 *
 */

/**
 * @file modules/multi/tcas_policy.h
 * @brief Pure policy helpers shared by TCAS firmware integrations and tests.
 *
 * These helpers contain no mutable module state and do not write navigation
 * setpoints. They centralize numerical validity, surveillance freshness, and
 * altitude-frame policy so fixed-wing and rotorcraft apply identical rules.
 */

#ifndef TCAS_POLICY_H
#define TCAS_POLICY_H

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

/** Action permitted for a traffic track at its current freshness. */
enum tcas_surveillance_action {
  /** Geometry is current enough to open, update, or clear an advisory. */
  TCAS_SURVEILLANCE_EVALUATE,
  /** Preserve an existing advisory without updating its geometry. */
  TCAS_SURVEILLANCE_HOLD,
  /** Geometry cannot support either evaluation or bounded advisory hold. */
  TCAS_SURVEILLANCE_UNAVAILABLE
};

/** Firmware-neutral vertical resolution values. */
enum tcas_resolution {
  /** No vertical command. */
  TCAS_RESOLUTION_NONE,
  /** Reserved level-flight command. */
  TCAS_RESOLUTION_LEVEL,
  /** Resolve above the intruder. */
  TCAS_RESOLUTION_CLIMB,
  /** Resolve below the intruder. */
  TCAS_RESOLUTION_DESCEND
};

/** Return whether an observed velocity vector is numerically usable.
 *
 * A finite zero vector is valid: an aircraft may be stationary relative to
 * the ground, for example while holding position in strong wind. Observation
 * presence is tracked separately; magnitude must never be used as validity.
 *
 * @param[in] east East velocity in meters per second.
 * @param[in] north North velocity in meters per second.
 * @param[in] up Up velocity in meters per second.
 * @return @c true when every component is finite, including an all-zero vector.
 */
static inline bool tcas_velocity_is_usable(float east, float north, float up)
{
  return isfinite(east) && isfinite(north) && isfinite(up);
}

/** Return three legacy update intervals without overflowing milliseconds. */
static inline uint32_t tcas_legacy_drop_ms(uint32_t legacy_hold_ms)
{
  return legacy_hold_ms > UINT32_MAX / 3u
         ? UINT32_MAX : 3u * legacy_hold_ms;
}

/**
 * Decide whether geometry may be evaluated, held, or declared unavailable.
 *
 * Fresh tracks may be evaluated. An active advisory may be held across a
 * bounded outage, but stale data may not open a new advisory. Legacy traffic
 * uses @p legacy_hold_ms as its freshness interval and three times that value
 * as its final drop boundary.
 *
 * @param[in] valid_geometry Whether a complete finite snapshot is available.
 * @param[in] mesh_track Whether mesh freshness limits apply.
 * @param[in] advisory_active Whether this track already owns a TA or RA.
 * @param[in] age_ms Monotonic age of the last accepted observation.
 * @param[in] fresh_ms Maximum mesh age eligible for evaluation.
 * @param[in] drop_ms Maximum mesh age eligible for advisory hold.
 * @param[in] legacy_hold_ms Legacy freshness and base hold interval.
 * @return The action permitted for this track and observation age.
 */
static inline enum tcas_surveillance_action tcas_surveillance_action(
  bool valid_geometry, bool mesh_track, bool advisory_active, uint32_t age_ms,
  uint32_t fresh_ms, uint32_t drop_ms, uint32_t legacy_hold_ms)
{
  if (!valid_geometry) {
    const uint32_t invalid_drop_ms = mesh_track ? drop_ms
                                     : tcas_legacy_drop_ms(legacy_hold_ms);
    return advisory_active && age_ms <= invalid_drop_ms
           ? TCAS_SURVEILLANCE_HOLD : TCAS_SURVEILLANCE_UNAVAILABLE;
  }
  if (mesh_track) {
    if (age_ms > drop_ms) {
      return TCAS_SURVEILLANCE_UNAVAILABLE;
    }
    if (age_ms > fresh_ms) {
      return advisory_active
             ? TCAS_SURVEILLANCE_HOLD : TCAS_SURVEILLANCE_UNAVAILABLE;
    }
    return TCAS_SURVEILLANCE_EVALUATE;
  }
  if (age_ms > tcas_legacy_drop_ms(legacy_hold_ms)) {
    return TCAS_SURVEILLANCE_UNAVAILABLE;
  }
  if (age_ms > legacy_hold_ms) {
    return advisory_active
           ? TCAS_SURVEILLANCE_HOLD : TCAS_SURVEILLANCE_UNAVAILABLE;
  }
  return TCAS_SURVEILLANCE_EVALUATE;
}

/**
 * Convert an intruder's local ENU altitude to the ownship MSL frame.
 *
 * @param[in] own_msl Ownship altitude in meters MSL.
 * @param[in] intruder_enu Intruder up coordinate in the shared ENU frame.
 * @param[in] own_enu Ownship up coordinate in the shared ENU frame.
 * @return Intruder altitude in meters MSL.
 */
static inline float tcas_intruder_altitude_msl(float own_msl,
                                               float intruder_enu,
                                               float own_enu)
{
  return own_msl + intruder_enu - own_enu;
}

/**
 * Convert an MSL command into a firmware's local altitude-above-origin frame.
 *
 * @param[in] altitude_msl Commanded altitude in meters MSL.
 * @param[in] hmsl_origin Local frame origin altitude in meters MSL.
 * @return Commanded altitude in meters above the local origin.
 */
static inline float tcas_altitude_local_from_msl(float altitude_msl,
                                                 float hmsl_origin)
{
  return altitude_msl - hmsl_origin;
}

/** Compose an RA altitude in MSL without mutating navigation state.
 *
 * Climb never lowers the nominal command; descend never raises it. Both paths
 * enforce @p minimum_msl after applying the required separation.
 *
 * @param[in] resolve Firmware-neutral vertical resolution.
 * @param[in] command_valid Whether the cached intruder boundary is usable.
 * @param[in] nominal_msl Current firmware navigation altitude in meters MSL.
 * @param[in] intruder_msl Cached intruder altitude in meters MSL.
 * @param[in] alim Required vertical separation in meters.
 * @param[in] minimum_msl Lowest permitted command in meters MSL.
 * @param[out] resolved_msl Resulting command when the function returns true.
 * @return @c true only when an active climb or descend command was produced.
 */
static inline bool tcas_resolve_altitude_msl(enum tcas_resolution resolve,
                                             bool command_valid,
                                             float nominal_msl,
                                             float intruder_msl,
                                             float alim,
                                             float minimum_msl,
                                             float *resolved_msl)
{
  if (!command_valid || resolved_msl == NULL
      || !isfinite(nominal_msl) || !isfinite(intruder_msl)
      || !isfinite(alim) || alim < 0.f || !isfinite(minimum_msl)) {
    return false;
  }

  float command;
  if (resolve == TCAS_RESOLUTION_CLIMB) {
    const float separation = intruder_msl + alim;
    command = nominal_msl > separation ? nominal_msl : separation;
  } else if (resolve == TCAS_RESOLUTION_DESCEND) {
    const float separation = intruder_msl - alim;
    command = nominal_msl < separation ? nominal_msl : separation;
  } else {
    return false;
  }

  *resolved_msl = command > minimum_msl ? command : minimum_msl;
  return isfinite(*resolved_msl);
}

#endif /* TCAS_POLICY_H */