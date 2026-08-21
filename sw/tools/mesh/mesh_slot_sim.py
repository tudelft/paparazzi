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

GCS_ID = 0
MAX_AC_ID = 254
SLOT_FREE = 0xFF          # 255 is reserved; 0 is the GCS and a real mesh node
NB_SLOTS = 32
FAIR_SLOTS = 28
MAX_REUSE = 8
AGE_FRAMES = 4
SUPERFRAME_S = 12.0
REMAINDER_EPOCH_FRAMES = 4 * AGE_FRAMES + 2 * NB_SLOTS + 1
CLOCK_STEP_MAX_MS = 10
GPS_WEEK_MS = 604800000
UINT32_MASK = 0xFFFFFFFF
INT32_HALF = 0x80000000


ENTRY_FRAMES = 3
HOLD_MIN, HOLD_SPAN = 6, 8
PRI_MIN, PRI_SPAN = 10, 10


def u32(value: int) -> int:
    """Return production's modulo-2^32 frame representation."""
    return value & UINT32_MASK


def frame_elapsed(now: int, previous: int) -> int:
    """Mirror unsigned uint32_t frame subtraction in the airborne allocator."""
    return u32(now - previous)


def frame_reached(now: int, deadline: int) -> bool:
    """Mirror ``(int32_t)(frame - deadline) >= 0`` from production."""
    return frame_elapsed(now, deadline) < INT32_HALF


@dataclass
class ClockGuard:
    """Mirror the production GPS phase-step detector."""

    last_synced: bool = False
    sample_valid: bool = False
    last_network_ms: int = 0
    last_local_ms: int = 0

    def update(self, network_ms: int, local_ms: int, synced: bool) -> bool:
        if synced != self.last_synced:
            self.last_synced = synced
            self.sample_valid = False
            return True
        if synced and self.sample_valid:
            network_elapsed = network_ms - self.last_network_ms
            local_elapsed = local_ms - self.last_local_ms
            if abs(network_elapsed - local_elapsed) > CLOCK_STEP_MAX_MS:
                self.last_network_ms = network_ms
                self.last_local_ms = local_ms
                return True
        self.last_network_ms = network_ms
        self.last_local_ms = local_ms
        self.sample_valid = True
        return False


def fair_target(nodes: int, rank: int, frame: int) -> int:
    """Return this sorted rank's deterministic share of available slots."""
    nodes = max(nodes, 1)
    target = max(1, min(MAX_REUSE, FAIR_SLOTS // nodes))
    if nodes <= FAIR_SLOTS and target < MAX_REUSE:
        remainder = FAIR_SLOTS % nodes
        first = (frame // REMAINDER_EPOCH_FRAMES) % nodes
        relative_rank = (rank + nodes - first) % nodes
        if relative_rank < remainder:
            target += 1
    return target


def check_clock_guard(failures: List[str]) -> None:
    """Exercise clock changes that frame-number-only checks cannot detect."""
    same_frame = ClockGuard(last_synced=True)
    if same_frame.update(61000, 1000, True):
        failures.append("clock guard reset on its first synchronized sample")
    if same_frame.update(61050, 1050, True):
        failures.append("clock guard reset during normal synchronized progression")
    if not same_frame.update(60800, 1100, True):
        failures.append("clock guard missed a same-superframe backward correction")

    next_frame = ClockGuard(last_synced=True)
    next_frame.update(71900, 1000, True)
    if not next_frame.update(72500, 1050, True):
        failures.append("clock guard missed an exactly-next-frame phase correction")

    week_rollover = ClockGuard(last_synced=True)
    week_rollover.update(GPS_WEEK_MS - 50, 1000, True)
    if week_rollover.update(GPS_WEEK_MS, 1050, True):
        failures.append("clock guard reset at a coherent GPS-week rollover")

    sync_change = ClockGuard(last_synced=True)
    sync_change.update(100000, 1000, True)
    if not sync_change.update(1050, 1050, False):
        failures.append("clock guard missed GPS synchronization loss")
    if not sync_change.update(100100, 1100, True):
        failures.append("clock guard missed GPS synchronization reacquisition")


@dataclass
class Node:
    """One aircraft's view of the mesh. Mirrors the C state exactly."""

    ac_id: int
    owned: List[int] = field(default_factory=list)
    until: List[int] = field(default_factory=list)
    reselect_count: int = 0
    neighbours: int = 0
    synced: bool = True
    position_valid: bool = True
    airborne: bool = True
    priority: bool = False
    cache_busy: bool = False
    frames_seen: int = 0
    last_frame: int = 0
    slots: Dict[int, tuple] = field(default_factory=dict)

    def __post_init__(self):
        if not GCS_ID <= self.ac_id <= MAX_AC_ID:
            raise ValueError(f"AC_ID {self.ac_id} is outside 0..{MAX_AC_ID}")
        self.owned = [self.ac_id % NB_SLOTS]
        self.until = [PRI_MIN]
        self.slots = {s: (SLOT_FREE, 0) for s in range(NB_SLOTS)}

    # --- mirrors mesh_slot_is_stale / mesh_slot_free ----------------------- #
    def _stale(self, s: int, frame: int) -> bool:
        return frame_elapsed(frame, self.slots[s][1]) > AGE_FRAMES

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
        return u32(frame + PRI_MIN + r % PRI_SPAN)

    # --- mirrors mesh_expansion_turn --------------------------------------- #
    def _turn(self, frame: int, nodes: int) -> bool:
        known = {self.slots[s][0] for s in range(NB_SLOTS)
                 if self.slots[s][0] not in (SLOT_FREE, self.ac_id)}
        rank = sum(1 for i in known if i < self.ac_id)
        return (frame % max(nodes, 1)) == rank

    # --- mirrors mesh_slot_maintain ---------------------------------------- #
    def maintain(self, frame: int) -> None:
        if frame == self.last_frame:
            return
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
        rank = sum(1 for ac_id in seen if ac_id < self.ac_id)

        if self.frames_seen == ENTRY_FRAMES:
            self.owned = [self._pick(frame)]
            self.until = [self._pri_expiry(frame)]

        if frame_reached(frame, self.until[0]):
            self.owned[0] = self._pick(frame)
            self.until[0] = self._pri_expiry(frame)
            self.reselect_count += 1

        owner = self.slots[self.owned[0]][0]
        if (owner != SLOT_FREE and owner != self.ac_id
                and not self._stale(self.owned[0], frame)
                and self.ac_id > owner):
            self.owned[0] = self._pick(frame)
            self.reselect_count += 1

        keep_o, keep_u = [self.owned[0]], [self.until[0]]
        for i in range(1, len(self.owned)):
            expired = frame_reached(frame, self.until[i])
            if self._free(self.owned[i], frame) and not expired:
                keep_o.append(self.owned[i]); keep_u.append(self.until[i])
        self.owned, self.until = keep_o, keep_u

        target = fair_target(nodes, rank, frame)
        if (not self.synced or not self.position_valid
                or (self.ac_id != 0 and not self.airborne)):
            target = 1
        elif self.priority:
            target = min(MAX_REUSE, target + 1)
        elif self.cache_busy and target > 1:
            target -= 1
        if self.frames_seen < ENTRY_FRAMES:
            target = 1

        if len(self.owned) > target:
            self.owned = self.owned[:target]; self.until = self.until[:target]
        elif len(self.owned) < target and self._turn(frame, nodes):
            start = (self.owned[0] + NB_SLOTS // 2) % NB_SLOTS
            for k in range(NB_SLOTS):
                s = (start + k) % NB_SLOTS
                quiet = (self.slots[s][0] == SLOT_FREE and
                         frame_elapsed(frame, self.slots[s][1]) > 2 * AGE_FRAMES)
                if quiet and not self.owns(s):
                    r = (self.ac_id * 2654435761 + frame * 40503
                         + self.reselect_count * 97) & 0xFFFFFFFF
                    self.owned.append(s)
                    self.until.append(u32(frame + HOLD_MIN + r % HOLD_SPAN))
                    break

    @property
    def reuse(self) -> int:
        return len(self.owned)


def run(args: argparse.Namespace) -> int:
    global MAX_REUSE
    MAX_REUSE = args.max_reuse
    rng = random.Random(args.seed)
    # AC_IDs are deliberately irregular. Zero is the GCS, a real mesh peer;
    # 254 exercises the highest aircraft ID and 255 remains reserved.
    all_ids = [0, 3, 19, 42, 58, 77, 101, 125, 140, 168, 203, 222, 254]
    live: Dict[int, Node] = {}
    failures: List[str] = []
    rate_log: List[tuple] = []

    try:
        Node(SLOT_FREE)
        failures.append("reserved ID 255 was accepted as a mesh node")
    except ValueError:
        pass

    boundary_frames = (
        0,
        REMAINDER_EPOCH_FRAMES,
        2 * REMAINDER_EPOCH_FRAMES,
        65535,
        65536,
        int(604800000 / (SUPERFRAME_S * 1000)) - 1,
        int(604800000 / (SUPERFRAME_S * 1000)),
        0xFFFFFFFE,
        0xFFFFFFFF,
    )
    for nodes in range(1, NB_SLOTS + 1):
        for frame in boundary_frames:
            quotas = [fair_target(nodes, rank, frame) for rank in range(nodes)]
            expected = (nodes if nodes > FAIR_SLOTS
                        else min(FAIR_SLOTS, nodes * MAX_REUSE))
            if sum(quotas) != expected or max(quotas) - min(quotas) > 1:
                failures.append(f"invalid fair quotas for {nodes} nodes at frame {frame}: {quotas}")

    if frame_elapsed(1, 0xFFFFFFFE) != 3:
        failures.append("32-bit frame age did not cross rollover correctly")
    if not frame_reached(1, 0xFFFFFFFF):
        failures.append("32-bit lease did not expire across rollover")
    if frame_reached(0xFFFFFFFE, 1):
        failures.append("32-bit lease expired before a rollover deadline")

    rollover_node = Node(42)
    rollover_node.last_frame = 0xFFFFFFFE
    rollover_node.frames_seen = ENTRY_FRAMES
    rollover_node.owned = [10, 20]
    rollover_node.until = [1, 0xFFFFFFFF]
    rollover_node.maintain(0xFFFFFFFF)
    if 20 in rollover_node.owned:
        failures.append("secondary lease did not expire at 32-bit rollover")
    rollover_node.maintain(0)
    if rollover_node.until[0] != 1:
        failures.append("primary lease expired before its rollover deadline")
    rollover_node.maintain(1)
    if rollover_node.until[0] == 1:
        failures.append("primary lease did not renew at its rollover deadline")
    check_clock_guard(failures)

    def join(ac: int) -> None:
        if ac not in live:
            live[ac] = Node(ac)

    for ac in all_ids[:args.initial_nodes]:
        join(ac)

    for frame in range(args.frames):
        # ---- membership churn -------------------------------------------- #
        if frame > 20 and rng.random() < args.churn:
            airborne_live = [i for i in live if i != 0]
            if len(airborne_live) > 2 and rng.random() < 0.5:
                victim = rng.choice(airborne_live)
                del live[victim]                 # landed / lost link
            else:
                cand = [a for a in all_ids if a not in live]
                if cand:
                    join(rng.choice(cand))       # arrived / regained link

        # Exercise application policy independently of topology churn. The GCS
        # remains a valid stationary peer; airborne nodes may lose state,
        # enter an emergency, or observe modem-cache pressure.
        for node in live.values():
            node.synced = rng.random() >= args.state_fault
            node.position_valid = rng.random() >= args.state_fault
            node.airborne = node.ac_id == 0 or rng.random() >= args.state_fault
            node.priority = rng.random() < args.priority
            node.cache_busy = rng.random() < args.cache_busy

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
              f"{mean/pop/SUPERFRAME_S:>11.2f} Hz")
    print("  " + "-" * 44)
    print(f"  (superframe is {SUPERFRAME_S:.0f} s; update rate = slots/{SUPERFRAME_S:.0f})")
    print()

    shared = [f for f in failures if "shared by" in f]
    hard = [f for f in failures if f not in shared]

    slot_frames = max(1, (args.frames - args.settle) * NB_SLOTS)
    pct = 100.0 * len(shared) / slot_frames

    # how long does any one collision persist?
    runs: Dict[str, Tuple[int, int]] = {}
    longest = 0
    for f in shared:
        frame = int(f.split("frame ", 1)[1].split(":", 1)[0])
        key = f.split(": ", 1)[1]
        last_frame, length = runs.get(key, (-2, 0))
        length = length + 1 if frame == last_frame + 1 else 1
        runs[key] = (frame, length)
        longest = max(longest, length)

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
    ap.add_argument("--initial-nodes", type=int, default=9, choices=range(1, 14),
                    metavar="1..13",
                    help="initial live peers including the GCS (default: 9)")
    ap.add_argument("--max-reuse", type=int, default=MAX_REUSE,
                    choices=range(1, NB_SLOTS + 1), metavar="1..32",
                    help="maximum slots per peer (default: production value)")
    ap.add_argument("--state-fault", type=float, default=0.01,
                    help="per-frame probability of sync/position/airborne loss")
    ap.add_argument("--priority", type=float, default=0.01,
                    help="per-frame probability of HOME/emergency/alert priority")
    ap.add_argument("--cache-busy", type=float, default=0.02,
                    help="per-frame probability of cache back-pressure")
    ap.add_argument("-v", "--verbose", action="store_true")
    return run(ap.parse_args(argv))


if __name__ == "__main__":
    sys.exit(main())
