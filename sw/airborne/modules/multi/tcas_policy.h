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

#ifndef TCAS_POLICY_H
#define TCAS_POLICY_H

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

enum tcas_surveillance_action {
  TCAS_SURVEILLANCE_EVALUATE,
  TCAS_SURVEILLANCE_HOLD,
  TCAS_SURVEILLANCE_UNAVAILABLE
};

/** Return whether an observed velocity vector is numerically usable.
 *
 * A finite zero vector is valid: an aircraft may be stationary relative to
 * the ground, for example while holding position in strong wind. Observation
 * presence is tracked separately; magnitude must never be used as validity.
 */
static inline bool tcas_velocity_is_usable(float east, float north, float up)
{
  return isfinite(east) && isfinite(north) && isfinite(up);
}

/** Decide whether geometry may be evaluated, held, or declared unavailable. */
static inline enum tcas_surveillance_action tcas_surveillance_action(
  bool valid_geometry, bool mesh_track, bool advisory_active, uint32_t age_ms,
  uint32_t fresh_ms, uint32_t drop_ms, uint32_t legacy_hold_ms)
{
  if (!valid_geometry) {
    const uint32_t invalid_drop_ms = mesh_track ? drop_ms : 3u * legacy_hold_ms;
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
  if (age_ms > 3u * legacy_hold_ms) {
    return TCAS_SURVEILLANCE_UNAVAILABLE;
  }
  if (age_ms > legacy_hold_ms) {
    return advisory_active
           ? TCAS_SURVEILLANCE_HOLD : TCAS_SURVEILLANCE_UNAVAILABLE;
  }
  return TCAS_SURVEILLANCE_EVALUATE;
}

/** Convert an intruder's local ENU altitude to the ownship MSL frame. */
static inline float tcas_intruder_altitude_msl(float own_msl,
                                               float intruder_enu,
                                               float own_enu)
{
  return own_msl + intruder_enu - own_enu;
}

#endif /* TCAS_POLICY_H */