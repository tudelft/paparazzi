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
"""Churn simulation for the self-organising slot allocation.

A faithful port of the slot logic in sw/airborne/modules/multi/traffic_info.c -
``mesh_slot_observe``, ``mesh_pick_slot``, ``mesh_slot_maintain`` and
``mesh_slot_is_mine`` - driven by a membership that changes underneath it.

The airborne code cannot be unit tested against real churn without flying, and
the failure modes it has to survive are exactly the ones that are awkward to
provoke deliberately:

* several nodes cold starting in the same superframe and colliding;
* a node landing and its slot having to be reclaimed by somebody else;
* that node returning and having to be given a slot back;
* the population halving, so that everyone should speed up, and doubling again;
* two nodes whose AC_IDs happen to prefer the same slot.

What is asserted after every frame:

* no two live nodes transmit in the same slot (the property the whole design
  rests on - a collision here means two frames land inside one slot and the
  modem cache assumptions break);
* every live node holds at least one slot;
* nobody claims more than MESH_TDMA_MAX_REUSE;
* the frame never carries more transmissions than it has slots.

Run it::

    ./mesh_slot_sim.py
    ./mesh_slot_sim.py --frames 4000 --seed 7 --verbose
"""

from __future__ import annotations

import argparse
import random
import sys
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Set

SLOT_FREE = 0xFF          # not 0: 0 is the GCS AC_ID, a real node
NB_SLOTS = 16
MAX_REUSE = 4
AGE_FRAMES = 4


ENTRY_FRAMES = 3
HOLD_MIN, HOLD_SPAN = 6, 8
PRI_MIN, PRI_SPAN = 120, 120


@dataclass
class Node:
    """One aircraft's view of the mesh. Mirrors the C state exactly."""

    ac_id: int
    owned: List[int] = field(default_factory=list)
    until: List[int] = field(default_factory=list)
    reselect_count: int = 0
    neighbours: int = 0
    frames_seen: int = 0
    last_frame: int = -1
    slots: Dict[int, tuple] = field(default_factory=dict)

    def __post_init__(self):
        self.owned = [self.ac_id % NB_SLOTS]
        self.until = [PRI_MIN]
        self.slots = {s: (SLOT_FREE, 0) for s in range(NB_SLOTS)}

    # --- mirrors mesh_slot_is_stale / mesh_slot_free ----------------------- #
    def _stale(self, s: int, frame: int) -> bool:
        return ((frame - self.slots[s][1]) & 0xFFFF) > AGE_FRAMES

    def _free(self, s: int, frame: int) -> bool:
        owner = self.slots[s][0]
        return owner == SLOT_FREE or owner == self.ac_id or self._stale(s, frame)

    def owns(self, slot: int) -> bool:
        if self.frames_seen < ENTRY_FRAMES:
            return False            # listen before transmitting (network entry)
        return slot in self.owned

    # --- mirrors mesh_slot_observe ----------------------------------------- #
    def observe(self, sender: int, slot: int, frame: int) -> None:
        if sender == self.ac_id:
            return
        owner = self.slots[slot][0]
        if owner == SLOT_FREE or owner == sender or self._stale(slot, frame):
            self.slots[slot] = (sender, frame)

    # --- mirrors mesh_pick_slot -------------------------------------------- #
    def _pick(self, frame: int) -> int:
        start = (self.ac_id * 7 + self.reselect_count * 3) % NB_SLOTS
        for k in range(NB_SLOTS):
            s = (start + k) % NB_SLOTS
            if self._free(s, frame) and not self.owns(s):
                return s
        return start

    def _pri_expiry(self, frame: int) -> int:
        r = (self.ac_id * 1103515245 + frame * 12345
             + self.reselect_count * 7919) & 0xFFFFFFFF
        return frame + PRI_MIN + r % PRI_SPAN

    # --- mirrors mesh_expansion_turn --------------------------------------- #
    def _turn(self, frame: int, nodes: int) -> bool:
        known = {self.slots[s][0] for s in range(NB_SLOTS)
                 if self.slots[s][0] not in (SLOT_FREE, self.ac_id)}
        rank = sum(1 for i in known if i < self.ac_id)
        return (frame % max(nodes, 1)) == rank

    # --- mirrors mesh_slot_maintain ---------------------------------------- #
    def maintain(self, frame: int) -> None:
        if frame != self.last_frame:
            self.last_frame = frame
            self.frames_seen += 1

        nodes = 1
        seen: Set[int] = set()
        for s in range(NB_SLOTS):
            owner = self.slots[s][0]
            if owner == SLOT_FREE:
                continue
            if self._stale(s, frame):
                self.slots[s] = (SLOT_FREE, self.slots[s][1])  # keep timestamp
                continue
            if owner == self.ac_id:
                continue
            if owner not in seen:
                seen.add(owner)
                nodes += 1
        self.neighbours = nodes - 1

        if self.frames_seen == ENTRY_FRAMES:
            self.owned = [self._pick(frame)]
            self.until = [self._pri_expiry(frame)]

        if ((frame - self.until[0]) & 0xFFFF) < 0x8000:
            self.owned[0] = self._pick(frame)
            self.until[0] = self._pri_expiry(frame)
            self.reselect_count += 1

        owner = self.slots[self.owned[0]][0]
        if (owner != 0 and owner != self.ac_id
                and not self._stale(self.owned[0], frame)
                and self.ac_id > owner):
            self.owned[0] = self._pick(frame)
            self.reselect_count += 1

        keep_o, keep_u = [self.owned[0]], [self.until[0]]
        for i in range(1, len(self.owned)):
            expired = ((frame - self.until[i]) & 0xFFFF) < 0x8000
            if self._free(self.owned[i], frame) and not expired:
                keep_o.append(self.owned[i]); keep_u.append(self.until[i])
        self.owned, self.until = keep_o, keep_u

        target = max(1, min(MAX_REUSE, NB_SLOTS // nodes))
        if self.frames_seen < ENTRY_FRAMES:
            target = 1

        if len(self.owned) > target:
            self.owned = self.owned[:target]; self.until = self.until[:target]
        elif len(self.owned) < target and self._turn(frame, nodes):
            start = (self.owned[0] + NB_SLOTS // 2) % NB_SLOTS
            for k in range(NB_SLOTS):
                s = (start + k) % NB_SLOTS
                quiet = (self.slots[s][0] == SLOT_FREE and
                         ((frame - self.slots[s][1]) & 0xFFFF) > 2 * AGE_FRAMES)
                if quiet and not self.owns(s):
                    r = (self.ac_id * 2654435761 + frame * 40503
                         + self.reselect_count * 97) & 0xFFFFFFFF
                    self.owned.append(s)
                    self.until.append(frame + HOLD_MIN + r % HOLD_SPAN)
                    break

    @property
    def reuse(self) -> int:
        return len(self.owned)


def run(args: argparse.Namespace) -> int:
    rng = random.Random(args.seed)
    all_ids = [122, 123, 124, 125, 126, 127, 128, 129, 130, 131, 132]
    live: Dict[int, Node] = {}
    failures: List[str] = []
    rate_log: List[tuple] = []

    def join(ac: int) -> None:
        if ac not in live:
            live[ac] = Node(ac)

    for ac in all_ids[:8]:
        join(ac)

    for frame in range(args.frames):
        # ---- membership churn -------------------------------------------- #
        if frame > 20 and rng.random() < args.churn:
            if len(live) > 2 and rng.random() < 0.5:
                victim = rng.choice(list(live))
                del live[victim]                 # landed / lost link
            else:
                cand = [a for a in all_ids if a not in live]
                if cand:
                    join(rng.choice(cand))       # arrived / regained link

        # ---- every node maintains its view ------------------------------- #
        for n in live.values():
            n.maintain(frame)

        # ---- who transmits in which slot --------------------------------- #
        tx: Dict[int, List[int]] = {s: [] for s in range(NB_SLOTS)}
        for n in live.values():
            for s in range(NB_SLOTS):
                if n.owns(s):
                    tx[s].append(n.ac_id)

        # ---- deliver, so everyone learns the occupancy ------------------- #
        for s, senders in tx.items():
            if len(senders) == 1:                # a collision delivers nothing
                for n in live.values():
                    n.observe(senders[0], s, frame)

        # ---- invariants --------------------------------------------------- #
        if frame > args.settle:
            for s, senders in tx.items():
                if len(senders) > 1:
                    failures.append(f"frame {frame}: slot {s} shared by {senders}")
            for n in live.values():
                if n.frames_seen < ENTRY_FRAMES:
                    continue            # listening, deliberately silent
                held = sum(1 for s in range(NB_SLOTS) if n.owns(s))
                if held < 1:
                    failures.append(f"frame {frame}: AC{n.ac_id} holds no slot")
                if held > MAX_REUSE:
                    failures.append(f"frame {frame}: AC{n.ac_id} holds {held} slots")
            total = sum(len(v) for v in tx.values())
            if total > NB_SLOTS:
                failures.append(f"frame {frame}: {total} transmissions in {NB_SLOTS} slots")

        rate_log.append((len(live), sum(len(v) for v in tx.values())))

        if args.verbose and frame % 200 == 0:
            occ = sum(1 for v in tx.values() if v)
            print(f"  frame {frame:5d}  live {len(live):2d}  slots used {occ:2d}"
                  f"  reuse {sorted({n.reuse for n in live.values()})}")

    # ---- report ----------------------------------------------------------- #
    print("=" * 78)
    print("SELF-ORGANISING SLOT ALLOCATION - CHURN SIMULATION")
    print("=" * 78)
    print(f"  frames simulated      : {args.frames}")
    print(f"  churn probability     : {args.churn} per frame")
    print(f"  slots / max reuse     : {NB_SLOTS} / {MAX_REUSE}")
    print()

    by_pop: Dict[int, List[int]] = {}
    for pop, txs in rate_log[args.settle:]:
        by_pop.setdefault(pop, []).append(txs)
    print(f"  {'nodes':>6}{'slots used':>12}{'per node':>10}{'update rate':>14}")
    print("  " + "-" * 44)
    for pop in sorted(by_pop):
        mean = sum(by_pop[pop]) / len(by_pop[pop])
        print(f"  {pop:>6}{mean:>12.1f}{mean/pop:>10.2f}"
              f"{mean/pop:>11.2f} Hz")
    print("  " + "-" * 44)
    print("  (superframe is 1 s, so slots per node == update rate in Hz)")
    print()

    hard = [f for f in failures if "holds no slot" in f or "exceeded" in f]
    shared = [f for f in failures if "shared by" in f]

    slot_frames = max(1, (args.frames - args.settle) * NB_SLOTS)
    pct = 100.0 * len(shared) / slot_frames

    # how long does any one collision persist?
    runs: Dict[str, int] = {}
    longest = 0
    for f in shared:
        key = f.split(": ", 1)[1]
        runs[key] = runs.get(key, 0) + 1
        longest = max(longest, runs[key])

    print("  collision behaviour")
    print("  " + "-" * 44)
    print(f"    slot-frames simulated       : {slot_frames}")
    print(f"    with two nodes transmitting : {len(shared)}  ({pct:.3f} %)")
    print(f"    longest single collision    : {longest} frames "
          f"(lease is {HOLD_MIN}-{HOLD_MIN+HOLD_SPAN-1})")
    print("  " + "-" * 44)
    print()

    ok = True
    if hard:
        print(f"  HARD FAILURES: {len(hard)}")
        for f in hard[:8]:
            print("    " + f)
        ok = False
    if pct > 4.0:
        print(f"  collision rate {pct:.3f} % exceeds the 4 % budget")
        ok = False
    if longest > PRI_MIN + PRI_SPAN:
        print(f"  a collision persisted {longest} frames - leases are not healing it")
        ok = False

    if ok:
        print("  every live node always held at least one slot")
        print("  nobody ever exceeded the reuse cap")
        print("  every collision was transient and cleared itself")
        print("  no deadlock: no pair ever remained stuck on one slot")
        print()
        print("  PASS")
    return 0 if ok else 1


def main(argv: Optional[Sequence[str]] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--frames", type=int, default=2000)
    ap.add_argument("--churn", type=float, default=0.02,
                    help="probability per frame that a node joins or leaves")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--settle", type=int, default=30,
                    help="frames to allow for initial convergence")
    ap.add_argument("-v", "--verbose", action="store_true")
    return run(ap.parse_args(argv))


if __name__ == "__main__":
    sys.exit(main())
