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

/**
 * @file modules/multi/tcas.h
 * @brief Public interface for shared fixed-wing and rotorcraft TCAS logic.
 *
 * The module evaluates traffic in a common ENU geometry and exposes resolution
 * altitude commands in meters MSL. Firmware integrations retain authority over
 * when that command may be applied: fixed-wing uses automatic altitude mode,
 * while rotorcraft uses navigation altitude mode.
 */

#ifndef TCAS_H
#define TCAS_H

#include "std.h"
#include "pprzlink/messages.h"      // TCAS_RA
#include "generated/airframe.h"     // AC_INFO
#include "modules/multi/traffic_info.h"
#include "modules/multi/tcas_policy.h"

/** Latest TCAS altitude state in meters MSL, including inactive fallback values. */
extern float tcas_alt_setpoint;
/** Time-to-conflict threshold for a traffic advisory, in seconds. */
extern float tcas_tau_ta;
/** Time-to-conflict threshold for a resolution advisory, in seconds. */
extern float tcas_tau_ra;
/** Horizontal protected-volume radius, in meters. */
extern float tcas_dmod;
/** Required vertical separation, in meters. */
extern float tcas_alim;

/** Represented surveillance was evaluated and requires no advisory. */
#define TCAS_NO_ALARM 0
/** A represented track requires a traffic advisory. */
#define TCAS_TA 1
/** A represented track requires an active resolution advisory. */
#define TCAS_RA 2
/** Surveillance cannot support a no-conflict conclusion.
 *
 * This includes missing/invalid ownship state, absent surveillance history,
 * traffic-table overflow, and expired or unusable tracks. It must never be
 * interpreted as equivalent to TCAS_NO_ALARM.
 */
#define TCAS_UNAVAILABLE 3

/** Vertical resolution command.
 *
 * RA_NONE means no vertical TCAS command is authorized. It does not prove
 * that separation exists; the accompanying TCAS status carries that meaning.
 */
enum tcas_resolve {
  /** No vertical command is authorized. */
  RA_NONE = TCAS_RESOLUTION_NONE,
  /** Reserved level-flight resolution; not currently emitted by the core. */
  RA_LEVEL = TCAS_RESOLUTION_LEVEL,
  /** Command vertical separation above the selected intruder. */
  RA_CLIMB = TCAS_RESOLUTION_CLIMB,
  /** Command vertical separation below the selected intruder. */
  RA_DESCEND = TCAS_RESOLUTION_DESCEND
};

/** Aggregate advisory state: #TCAS_NO_ALARM through #TCAS_UNAVAILABLE. */
extern uint8_t tcas_status;
/** Current vertical resolution, valid only with an active, valid RA command. */
extern enum tcas_resolve tcas_resolve;
/** Aircraft ID selected for the current RA, or @c AC_ID when none is selected. */
extern uint8_t tcas_ac_RA;

/** Per-track advisory and peer-resolution state. */
struct tcas_ac_status {
  /** Track advisory state: #TCAS_NO_ALARM through #TCAS_UNAVAILABLE. */
  uint8_t status;
  /** Latest valid peer resolution received for this track. */
  enum tcas_resolve resolve;
};

/** Per-track TCAS state indexed by compact traffic-table slot. */
extern struct tcas_ac_status tcas_acs_status[NB_ACS];

/** Initialize advisory state, timing thresholds, and altitude caches. */
extern void tcas_init(void);
/** Evaluate traffic tracks and update advisory state at 1 Hz. */
extern void tcas_periodic_task_1Hz(void);
/** Refresh the active RA altitude boundary at 4 Hz. */
extern void tcas_periodic_task_4Hz(void);

/** Resolve the current RA against a firmware's nominal MSL altitude.
 *
 * The function does not mutate firmware navigation state. It returns a command
 * only for an active RA with valid ownship geometry and a valid cached intruder
 * boundary. That cache may be retained during a bounded advisory hold. The
 * configured security-height floor is always applied.
 *
 * @param[in] nominal_altitude_msl Current navigation altitude in meters MSL.
 * @param[out] altitude_msl Resolved avoidance altitude in meters MSL.
 * @return @c true only when an active, valid RA command is available.
 */
extern bool tcas_get_altitude_command(float nominal_altitude_msl, float *altitude_msl);

/**
 * Apply a valid RA to the fixed-wing altitude controller when authorized.
 *
 * This compatibility hook acts only in automatic altitude mode with throttle
 * enabled. Rotorcraft does not use this hook; it consumes
 * tcas_get_altitude_command() from its navigation-altitude guidance path.
 */
extern void callTCAS(void);

/**
 * Parse a directed peer-resolution coordination message.
 *
 * @param[in] buf Encoded PPRZLink TCAS_RESOLVE message buffer.
 */
extern void parseTcasResolve(uint8_t *buf);

/**
 * Parse a directed peer resolution advisory.
 *
 * @param[in] buf Encoded PPRZLink TCAS_RA message buffer.
 */
extern void parseTcasRA(uint8_t *buf);

#endif /* TCAS_H */
