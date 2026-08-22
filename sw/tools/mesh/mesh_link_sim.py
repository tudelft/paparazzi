#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright (C) 2026 The Paparazzi Team
#
# This file is part of paparazzi.
#
# paparazzi is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2, or (at your option)
# any later version.
#
# paparazzi is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with paparazzi; see the file COPYING.  If not, see
# <http://www.gnu.org/licenses/>.
"""Packet-level link simulator for the 433 MHz airborne LoRa MESH.

The other tools in this directory each answer one question in isolation:

* ``mesh_link_budget.py``  - does a single link close, at what margin?
* ``mesh_phase_optimizer.py`` - does the traffic fit the channel?
* ``mesh_slot_sim.py``     - does the slot self-organisation survive churn?
* ``e52_provision.py``     - is the modem configured to match all of the above?

This tool closes the loop between them: it flies the whole fleet through the
bounded operating area and counts, frame by frame, what the ground station
actually receives.  Geometry, attitude, the two-ray ground reflection, slow
shadowing, the TDMA schedule and the single-copy flood relay all interact
here, which is exactly the interaction none of the single-purpose tools can
see.

Model
-----
* **Geometry**: the GCS is AC_ID 0 on a mast at a corner (or the centre) of a
    rectangular operating area; it is a full routing peer by default, exactly
    like any airframe. Drones wander between random waypoints inside the box at
    a fixed altitude band and bank when they turn.
* **RF**: the mean path loss per pair and per timestep is
  ``max(two-ray local mean, free space)`` from ``mesh_link_budget.py`` - the
  same conservative choice as ::LinkBudget.margin_at_m, at the true heights
  of both ends.  Attitude enters through the polarisation mismatch of the
  banked dipole.
* **Fading**: the fixed 10 dB fade margin of the budget is *removed* and
  replaced by an explicit slow shadowing draw (log-normal, ``--shadow-sigma``
  dB, correlated over ``--shadow-tau`` seconds per pair).  Charging the fixed
  margin *and* drawing fades would double-count; the whole point of the
  Monte-Carlo is to replace the blanket margin with the distribution it
  stands for.
* **MAC**: the delivered profile uses a 12-second superframe with 32 slots.
    This simulator assumes those originations are collision-free; slot
    acquisition and churn are validated separately by ``mesh_slot_sim.py``.
    Routing nodes rebroadcast every frame they hear exactly once (the E52
    single-copy flood); a receiver keeps the first copy that gets through.
* **AC_IDs are arbitrary**: any distinct values in 0..254. Nothing in this
  file derives anything from the numeric value of an ID.

What is reported and asserted
-----------------------------
* per-drone packet delivery ratio (PDR) to the GCS, direct and with relaying;
* the worst instantaneous margin seen on any used link;
* the transmitter duty cycle of the busiest radio against the 40 % ceiling;
* exit status 1 if any drone's relayed PDR falls below ``--min-pdr`` or the
  duty ceiling is broken, so the tool can gate a regression suite.

Run it::

    ./mesh_link_sim.py
    ./mesh_link_sim.py --ac-ids 0,3,19,42,77,101,168,203,251 --relay-ids 0
    ./mesh_link_sim.py --drones 12 --duration 1800 --seed 7
"""

from __future__ import annotations

import argparse
import math
import random
import sys
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import mesh_link_budget as mlb

GCS_AC_ID = 0
NB_SLOTS = 32
DUTY_CEILING = 0.40           # matches mesh_phase_optimizer.py
# Representative mean air time of one frame in the optimizer's traffic mix.
# Duty is charged from actual transmissions, not the 500 ms reservation slot:
# the slot is sized for a complete worst-case flood, not one RF transmission.
AIR_TIME_MS_DEFAULT = 20.0


# --------------------------------------------------------------------------- #
# Mobility
                                                                              #
# --------------------------------------------------------------------------- #

@dataclass
class Drone:
    """Waypoint-wandering aircraft. Position in metres, box-local ENU."""

    ac_id: int
    x: float
    y: float
    alt_m: float
    speed_ms: float
    wp: Tuple[float, float] = (0.0, 0.0)
    bank_deg: float = 0.0
    heading: float = 0.0

    def step(self, dt: float, width: float, height: float,
             rng: random.Random) -> None:
        dx, dy = self.wp[0] - self.x, self.wp[1] - self.y
        dist = math.hypot(dx, dy)
        if dist < self.speed_ms * dt * 2.0:
            self.wp = (rng.uniform(0.0, width), rng.uniform(0.0, height))
            dx, dy = self.wp[0] - self.x, self.wp[1] - self.y
            dist = math.hypot(dx, dy)
        want = math.atan2(dy, dx)
        turn = (want - self.heading + math.pi) % (2.0 * math.pi) - math.pi
        # Bank proportional to the turn being flown, capped at the 45 deg the
        # link budget charges for. Straight legs fly wings level.
        self.bank_deg = max(-45.0, min(45.0, math.degrees(turn)))
        max_turn = math.radians(30.0) * dt          # gentle fixed-wing rate
        self.heading += max(-max_turn, min(max_turn, turn))
        self.x += self.speed_ms * dt * math.cos(self.heading)
        self.y += self.speed_ms * dt * math.sin(self.heading)
        self.x = min(max(self.x, 0.0), width)
        self.y = min(max(self.y, 0.0), height)


# --------------------------------------------------------------------------- #
# Shadowing
                                                                              #
# --------------------------------------------------------------------------- #

class Shadowing:
    """Per-pair log-normal shadowing, first-order Gauss-Markov in time.

    ``tau`` is the decorrelation time in seconds. The process is symmetric in
    the pair, keyed on the unordered ID pair - IDs are used purely as
    dictionary keys, never arithmetically.
    """

    def __init__(self, sigma_db: float, tau_s: float, rng: random.Random):
        self.sigma = sigma_db
        self.rho = math.exp(-1.0 / max(tau_s, 1e-6))
        self.rng = rng
        self.state: Dict[Tuple[int, int], float] = {}

    def draw(self, a: int, b: int) -> float:
        key = (min(a, b), max(a, b))
        prev = self.state.get(key)
        innov = self.rng.gauss(0.0, self.sigma)
        cur = innov if prev is None else (
            self.rho * prev + math.sqrt(1.0 - self.rho * self.rho) * innov)
        self.state[key] = cur
        return cur


# --------------------------------------------------------------------------- #
# Simulation
                                                                              #
# --------------------------------------------------------------------------- #

@dataclass
class Stats:
    sent: int = 0
    direct: int = 0
    delivered: int = 0                       # direct or via any relay copy
    worst_margin_db: float = float("inf")


def receive(lb: mlb.LinkBudget, shadow: Shadowing,
            d_m: float, h_tx: float, h_rx: float,
            tx_id: int, rx_id: int, bank_deg: float) -> Tuple[bool, float]:
    """One reception attempt. Returns (success, instantaneous margin dB).

    The fixed fade margin charged inside ::LinkBudget.threshold_loss_db is
    added back and replaced by the explicit shadowing draw - see the module
    docstring for why charging both would be dishonest.
    """
    loss = max(lb.prop.local_mean_loss_db(d_m, h_tx, h_rx),
               lb.prop.fspl_db(d_m))
    # Charge the actual bank of the transmitting airframe, not the blanket
    # worst case: the budget's bank_loss is already inside threshold_loss_db,
    # so swap it for the instantaneous value.
    pol_delta = (mlb.Antenna.polarisation_loss_db(math.radians(abs(bank_deg)))
                 - lb.bank_loss_db)
    margin = (lb.threshold_loss_db - loss
              + lb.fade_margin_db          # remove the blanket margin ...
              - pol_delta
              - shadow.draw(tx_id, rx_id))  # ... and draw the fade instead
    return margin > 0.0, margin


def simulate(args: argparse.Namespace) -> int:
    rng = random.Random(args.seed)

    ac_ids = ([int(t) for t in args.ac_ids.split(",")] if args.ac_ids
              else [GCS_AC_ID] + rng.sample(range(1, 255), args.drones))
    if any(ac_id < 0 or ac_id > 254 for ac_id in ac_ids):
        print("AC_IDs must be in 0..254; 255 is reserved for broadcast",
              file=sys.stderr)
        return 2
    if len(set(ac_ids)) != len(ac_ids):
        print("AC_IDs must be distinct", file=sys.stderr)
        return 2
    if GCS_AC_ID not in ac_ids:
        print("the GCS (AC_ID 0) must be part of the fleet", file=sys.stderr)
        return 2
    drone_ids = [i for i in ac_ids if i != GCS_AC_ID]
    relay_ids = (set(ac_ids) if args.relay_ids.lower() == "all" else
                 {int(t) for t in args.relay_ids.split(",") if t.strip()})
    unknown = relay_ids - set(ac_ids)
    if unknown:
        print(f"relay ids {sorted(unknown)} not in the fleet", file=sys.stderr)
        return 2
    if len(drone_ids) > NB_SLOTS:
        print(f"more than {NB_SLOTS} slotted transmitters", file=sys.stderr)
        return 2

    prop = mlb.Propagation(ground=args.ground)
    lb = mlb.LinkBudget(prop=prop, gcs_mast_m=args.gcs_mast)
    shadow = Shadowing(args.shadow_sigma, args.shadow_tau, rng)

    width, height = args.width, args.height
    gcs_xy = ((0.0, 0.0) if args.gcs_corner else
              (width / 2.0, height / 2.0))
    drones = {i: Drone(ac_id=i,
                       x=rng.uniform(0.0, width), y=rng.uniform(0.0, height),
                       alt_m=rng.uniform(args.alt_min, args.alt_max),
                       speed_ms=rng.uniform(12.0, 18.0),
                       wp=(rng.uniform(0.0, width), rng.uniform(0.0, height)))
              for i in drone_ids}

    def pos(i: int) -> Tuple[float, float, float]:
        if i == GCS_AC_ID:
            return gcs_xy[0], gcs_xy[1], lb.gcs_mast_m
        d = drones[i]
        return d.x, d.y, d.alt_m

    def bank(i: int) -> float:
        return 0.0 if i == GCS_AC_ID else drones[i].bank_deg

    stats = {i: Stats() for i in drone_ids}
    tx_count = {i: 0 for i in ac_ids}
    frame_credit = {i: 0.0 for i in drone_ids}

    for _t in range(args.duration):
        for d in drones.values():
            d.step(1.0, width, height, rng)

        for src in drone_ids:
            frame_credit[src] += args.frame_rate
            while frame_credit[src] >= 1.0:
                frame_credit[src] -= 1.0
                st = stats[src]
                st.sent += 1
                tx_count[src] += 1
                sx, sy, sh = pos(src)

                heard: Dict[int, bool] = {}
                for rx in ac_ids:
                    if rx == src:
                        continue
                    rxx, rxy, rxh = pos(rx)
                    d_m = max(math.hypot(sx - rxx, sy - rxy), 1.0)
                    ok, margin = receive(lb, shadow, d_m, sh, rxh,
                                         src, rx, bank(src))
                    st.worst_margin_db = min(st.worst_margin_db, margin)
                    heard[rx] = ok

                got = heard[GCS_AC_ID]
                if got:
                    st.direct += 1
                # E52 broadcast flood: every routing node forwards once after
                # first reception. Iterate wave by wave so paths may contain
                # any number of hops; the reached set is the duplicate filter.
                reached = {src} | {rx for rx, ok in heard.items() if ok}
                frontier = [rx for rx in reached if rx in relay_ids and rx != src]
                forwarded = set()
                while frontier:
                    router = frontier.pop(0)
                    if router in forwarded:
                        continue
                    forwarded.add(router)
                    tx_count[router] += 1
                    rtx, rty, rth = pos(router)
                    for rx in ac_ids:
                        if rx in reached:
                            continue
                        rxx, rxy, rxh = pos(rx)
                        d_m = max(math.hypot(rtx - rxx, rty - rxy), 1.0)
                        ok, margin = receive(lb, shadow, d_m, rth, rxh,
                                             router, rx, bank(router))
                        st.worst_margin_db = min(st.worst_margin_db, margin)
                        if ok:
                            reached.add(rx)
                            if rx in relay_ids:
                                frontier.append(rx)
                got = GCS_AC_ID in reached
                if got:
                    st.delivered += 1

    # ---- report ----------------------------------------------------------- #
    dur = args.duration
    print(f"link simulation: {len(drone_ids)} drones + GCS, "
          f"{width / 1000.0:.1f} x {height / 1000.0:.1f} km area, "
          f"{dur} s, seed {args.seed}")
    print(f"  routers: {sorted(relay_ids)}"
          f"{'  (GCS routing)' if GCS_AC_ID in relay_ids else ''}   "
          f"shadowing sigma {args.shadow_sigma:.1f} dB tau {args.shadow_tau:.0f} s")
    print(f"  {'AC_ID':>6} {'sent':>7} {'direct':>8} {'relayed':>8} "
          f"{'worst margin':>13}")
    failed = False
    for i in drone_ids:
        st = stats[i]
        if st.sent == 0:
            print(f"  {i:>6} {0:>7} {'n/a':>8} {'n/a':>8} {'n/a':>13}")
            failed = True
            continue
        pdr_d = st.direct / st.sent
        pdr_r = st.delivered / st.sent
        flag = ""
        if pdr_r < args.min_pdr:
            flag = f"  << below {args.min_pdr:.2f}"
            failed = True
        print(f"  {i:>6} {st.sent:>7} {pdr_d:>8.3f} {pdr_r:>8.3f} "
              f"{st.worst_margin_db:>10.1f} dB{flag}")

    busiest = max(tx_count, key=lambda i: tx_count[i])
    duty = tx_count[busiest] * args.air_time_ms / 1000.0 / dur
    over = duty > DUTY_CEILING
    print(f"  busiest radio: AC_ID {busiest}, duty {duty * 100.0:.1f} % "
          f"(ceiling {DUTY_CEILING * 100.0:.0f} %)"
          f"{'  << OVER' if over else '  OK'}")
    failed = failed or over

    total = sum(s.sent for s in stats.values())
    if total == 0:
        print("  no frames originated; increase --duration or --frame-rate")
        return 2
    print(f"  fleet PDR to GCS: direct "
          f"{sum(s.direct for s in stats.values()) / total:.3f}, relayed "
          f"{sum(s.delivered for s in stats.values()) / total:.3f}")
    return 1 if failed else 0


# --------------------------------------------------------------------------- #
# CLI
                                                                              #
# --------------------------------------------------------------------------- #

def main(argv: Optional[Sequence[str]] = None) -> int:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--drones", type=int, default=8,
                   help="number of airborne nodes (default 8); IDs drawn "
                        "at random to prove nothing depends on their values")
    p.add_argument("--ac-ids", type=str, default=None,
                   help="explicit comma-separated fleet, must include 0 (GCS)")
    p.add_argument("--relay-ids", type=str, default="all",
                   help="routing nodes: all (default) or comma-separated IDs")
    p.add_argument("--width", type=float, default=3000.0,
                   help="operating area width in m (default 3000)")
    p.add_argument("--height", type=float, default=3000.0,
                   help="operating area height in m (default 3000)")
    p.add_argument("--gcs-corner", action="store_true", default=True,
                   help="GCS at a corner (default; worst case geometry)")
    p.add_argument("--gcs-center", dest="gcs_corner", action="store_false",
                   help="GCS at the centre of the box")
    p.add_argument("--gcs-mast", type=float, default=4.0,
                   help="GCS antenna height in m (default 4)")
    p.add_argument("--alt-min", type=float, default=50.0)
    p.add_argument("--alt-max", type=float, default=120.0)
    p.add_argument("--ground", choices=sorted(mlb.GROUND_TYPES),
                   default="average")
    p.add_argument("--frame-rate", type=float, default=1.0 / 6.0,
                   help="originated state frames per node per second (default 1/6)")
    p.add_argument("--air-time-ms", type=float, default=AIR_TIME_MS_DEFAULT,
                   help="mean air time of one frame in ms (default "
                        f"{AIR_TIME_MS_DEFAULT:.0f}, from the optimizer's "
                        "traffic mix)")
    p.add_argument("--duration", type=int, default=600,
                   help="simulated seconds (default 600)")
    p.add_argument("--shadow-sigma", type=float, default=3.0,
                   help="slow shadowing std dev in dB (default 3)")
    p.add_argument("--shadow-tau", type=float, default=30.0,
                   help="shadowing decorrelation time in s (default 30)")
    p.add_argument("--min-pdr", type=float, default=0.95,
                   help="per-drone relayed PDR gate (default 0.95)")
    p.add_argument("--seed", type=int, default=1)
    args = p.parse_args(argv)
    return simulate(args)


if __name__ == "__main__":
    sys.exit(main())
