/*
 * Copyright (C) 2026 TU Delft
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

/** @file modules/audio/quiet_listen.h
 *  Low-throttle listening for EARcam on a fixed-wing with the energy controller.
 *
 *  The energy controller's throttle is nominal + climb increment x climb
 *  setpoint + airspeed P + energy P + two bank terms, and its integrator
 *  rewrites nominal every control step. On a listening arc every term but
 *  nominal is zeroed and nominal is set to listen_thr (3-10 %), so the
 *  propeller turns slowly at a fixed, quiet setting instead of stopping.
 *
 *  The terms are saved when an arc starts and restored when it ends,
 *  integrator state included, so power comes back where it was.
 *
 *  Safety net: the request must be renewed at least every
 *  QUIET_LISTEN_HOLD_S (quiet_listen_on(), every nav cycle). A GPS Abi
 *  callback restores the gains once the request is stale. GPS arrives in
 *  every autopilot mode, so this also covers HOME mode (geofence breach, RC
 *  loss), which runs nav_home() instead of the flight plan and would
 *  otherwise leave the gains at zero. The always-false flight plan exception
 *  quiet_listen_guard() does the same from every flight-plan block, in case
 *  GPS stops. A gain changed from the GCS during an arc is overwritten when
 *  the arc ends.
 *
 *  The arc also sets earcam_listen, so EARcam samples only on the arcs,
 *  earcam_quiet_delay_s after the propeller slowed down. The gains and
 *  earcam_listen are also restored as soon as nav_block is no longer the
 *  block that started the arc; quiet_listen_guard() is a global exception,
 *  so on a deroute that happens before the new block's first stage.
 *
 *  EARcam listening session: sampling runs from earcam_start() until the
 *  plan leaves the block that calls quiet_listen_session_hold() by ANY route
 *  (normal hand-over, escape exception, GeoFence, a GCS button). Then it is
 *  stopped, so nothing is recorded outside the search. earcam_listen goes
 *  back to its default (TRUE) for other plans.
 */

#ifndef QUIET_LISTEN_H
#define QUIET_LISTEN_H

#include "std.h"

/** Longest time a listening request stays valid without being renewed, s. */
#ifndef QUIET_LISTEN_HOLD_S
#define QUIET_LISTEN_HOLD_S 0.3f
#endif

extern void quiet_listen_init(void);

/** Start (or renew) a low-throttle listening arc at throttle listen_thr. */
extern void quiet_listen_on(float listen_thr);
/** End the arc: restore the saved gains, stop EARcam sampling. */
extern void quiet_listen_off(void);
/** End the arc if the request is stale or the block changed. */
extern void quiet_listen_check(void);

/** Keep the EARcam session open for the current block; call every cycle. */
extern void quiet_listen_session_hold(void);
/** Close the EARcam session: earcam_stop(), earcam_listen back to TRUE. */
extern void quiet_listen_session_stop(void);
/** Close the session if the plan left its block. */
extern void quiet_listen_session_check(void);

/** Side-effecting condition for a global exception: runs both checks, always false. */
extern bool quiet_listen_guard(void);

#endif /* QUIET_LISTEN_H */
