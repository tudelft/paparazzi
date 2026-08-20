#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Validate bounded holdover and randomized GPS-denied mesh access.

The model starts with synchronized TDMA, removes GPS from a configurable subset,
propagates fleet fallback from HOLDOVER advertisements, runs independently
clocked asynchronous deadlines, reboots nodes with generation-tagged event
cancellation, restores GPS, expires the last denied-node lease, and returns to
TDMA. A broadcast occupies the complete modeled E52 flood span; any overlap is
counted as a collision and every modem has an explicit five-entry queue.
"""

from __future__ import annotations

import argparse
import heapq
import random
from dataclasses import dataclass
from typing import List, Sequence


AC_IDS = (0, 3, 19, 42, 58, 77, 101, 125, 140, 168, 203, 222, 251)
HOLDOVER_MS = 60_000
ASYNC_MIN_MS = 16_000
ASYNC_MAX_MS = 24_000
FLOOD_SPAN_MS = 290
MODEM_DRAIN_MS = 60
MODEM_CACHE_SIZE = 5
TASK_PERIOD_MS = 50
TDMA_SLOT_MS = 375
ACQUIRE_AND_ENTRY_MS = 2_000 + 3 * 12_000
PEER_HOLD_MS = 2 * ASYNC_MAX_MS + 12_000
RECOVERY_EPOCH_FRAMES = 8
RECOVERY_MIN_LEAD_FRAMES = PEER_HOLD_MS // 12_000 + 2

GPS = 0
HOLDOVER = 1
ASYNC = 2
ACQUIRING = 3


@dataclass
class Node:
    """One independently clocked fallback originator."""

    ac_id: int
    drift: float
    prng: int
    generation: int = 0
    mode: int = GPS
    gps_valid: bool = True
    announcement_required: bool = False
    peer_until_ms: float = 0.0
    self_until_ms: float = 0.0
    recovery_frame: int = 0
    first_holdover_ms: float = -1.0
    next_local_ms: float = 0.0

    def random_u32(self) -> int:
        value = self.prng
        value ^= (value << 13) & 0xFFFFFFFF
        value ^= value >> 17
        value ^= (value << 5) & 0xFFFFFFFF
        self.prng = value & 0xFFFFFFFF
        return self.prng

    def reboot(self) -> None:
        self.generation += 1
        self.prng = (0x9E3779B9 ^ (self.ac_id * 2654435761)) & 0xFFFFFFFF or 1
        self.mode = ASYNC
        self.announcement_required = False
        self.peer_until_ms = 0.0
        self.self_until_ms = 0.0
        self.recovery_frame = 0
        self.first_holdover_ms = -1.0

    def schedule(self, true_ms: float) -> float:
        local_now = true_ms * self.drift
        span = ASYNC_MAX_MS - ASYNC_MIN_MS + 1
        offset = (self.random_u32() * span) >> 32
        self.next_local_ms = local_now + ASYNC_MIN_MS + offset
        true_deadline = self.next_local_ms / self.drift
        return ((true_deadline + TASK_PERIOD_MS - 1) // TASK_PERIOD_MS
                * TASK_PERIOD_MS)


@dataclass
class Result:
    transmissions: int
    collisions: int
    cache_flushes: int
    fallback_converged_ms: float
    recovery_completed_ms: float
    stale_events: int
    final_modes: tuple[int, int, int, int]


def frame_of(true_ms: float) -> int:
    return int(true_ms // 12_000) & 0xFFFFFFFF


def frame_reached(frame: int, target: int) -> bool:
    return target != 0 and ((frame - target) & 0xFFFFFFFF) < 0x80000000


def frame_later(candidate: int, current: int) -> bool:
    return candidate != 0 and (current == 0 or
                               0 < ((candidate - current) & 0xFFFFFFFF)
                               < 0x80000000)


def next_recovery_frame(frame: int) -> int:
    target = (frame + RECOVERY_EPOCH_FRAMES
              - frame % RECOVERY_EPOCH_FRAMES) & 0xFFFFFFFF
    return target or RECOVERY_EPOCH_FRAMES


def simulate(seed: int, duration_s: int, stress_ppm: int,
             denied_nodes: int) -> Result:
    rng = random.Random(seed)
    nodes = []
    for ac_id in AC_IDS:
        prng = 0x9E3779B9 ^ (ac_id * 2654435761)
        nodes.append(Node(ac_id,
                          1.0 + rng.uniform(-stress_ppm, stress_ppm) * 1e-6,
                          prng & 0xFFFFFFFF or 1))
    end_ms = duration_s * 1000.0
    recovery_started_ms = duration_s * 400.0
    denied = set(range(denied_nodes))
    for index in denied:
        nodes[index].mode = HOLDOVER
        nodes[index].gps_valid = False

    # kind: 0 TDMA, 1 local fallback, 2 async TX, 3 GPS recovery,
    #       4 reboot, 5 recovery check, 6 acquisition complete,
    #       7 collision-cluster resolution.
    events: List[tuple[float, int, int, int]] = []
    heapq.heappush(events, (0.0, 0, 0, 0))
    for index in denied:
        heapq.heappush(events, (HOLDOVER_MS - 12_000, 1, index,
                               nodes[index].generation))
        heapq.heappush(events, (recovery_started_ms, 3, index,
                               nodes[index].generation))
    for index in range(len(nodes)):
        if rng.random() < 0.5:
            reboot_ms = rng.uniform(HOLDOVER_MS,
                                    recovery_started_ms + PEER_HOLD_MS)
            heapq.heappush(events, (reboot_ms, 4, index,
                                   nodes[index].generation))

    collisions = 0
    transmissions = 0
    cache_flushes = 0
    cache_departures: List[List[float]] = [[] for _ in nodes]
    first_async_ms = end_ms + 1.0
    fallback_converged_ms = -1.0
    recovery_completed_ms = -1.0
    stale_events = 0
    cluster_end_ms = 0.0
    cluster_generation = 0
    cluster_count = 0
    cluster_sender = 0
    cluster_mode = GPS
    cluster_target = 0

    def schedule_async(index: int, now_ms: float) -> None:
        node = nodes[index]
        heapq.heappush(events, (node.schedule(now_ms), 2, index,
                               node.generation))

    def queue_flood(now_ms: float) -> None:
        nonlocal cache_flushes
        for queue in cache_departures:
            queue[:] = [departure for departure in queue
                        if departure > now_ms]
            departure = (max(queue) if queue else now_ms) + MODEM_DRAIN_MS
            queue.append(departure)
            if len(queue) > MODEM_CACHE_SIZE:
                cache_flushes += 1
                queue.clear()

    def originate(sender: int, sender_mode: int, target: int,
                  now_ms: float) -> None:
        nonlocal cluster_end_ms, cluster_generation, cluster_count
        nonlocal cluster_sender, cluster_mode, cluster_target
        nonlocal transmissions, collisions
        transmissions += 1
        queue_flood(now_ms)
        if now_ms < cluster_end_ms:
            if cluster_count == 1:
                collisions += 1
            collisions += 1
            cluster_count += 1
            cluster_end_ms = max(cluster_end_ms, now_ms + FLOOD_SPAN_MS)
        else:
            cluster_count = 1
            cluster_end_ms = now_ms + FLOOD_SPAN_MS
        cluster_generation += 1
        cluster_sender = sender
        cluster_mode = sender_mode
        cluster_target = target
        heapq.heappush(events, (cluster_end_ms, 7, cluster_sender,
                               cluster_generation))

    def enter_async(index: int, now_ms: float) -> None:
        nonlocal fallback_converged_ms
        node = nodes[index]
        if node.mode != ASYNC:
            node.mode = ASYNC
            node.announcement_required = True
            node.recovery_frame = 0
            schedule_async(index, now_ms)
            if (fallback_converged_ms < 0.0
                    and all(peer.mode == ASYNC for peer in nodes)):
                fallback_converged_ms = now_ms

    def schedule_recovery(index: int, target: int) -> None:
        target_ms = float(target) * 12_000.0
        heapq.heappush(events, (target_ms, 5, index,
                               nodes[index].generation))

    def receive(index: int, sender_mode: int, target: int,
                now_ms: float) -> None:
        node = nodes[index]
        if sender_mode == HOLDOVER:
            if node.first_holdover_ms < 0.0:
                node.first_holdover_ms = now_ms
                heapq.heappush(events,
                               (now_ms + HOLDOVER_MS - 12_000, 1, index,
                                node.generation))
        elif sender_mode == ASYNC:
            node.peer_until_ms = now_ms + PEER_HOLD_MS
            node.recovery_frame = 0
            enter_async(index, now_ms)
        elif sender_mode == GPS and target != 0:
            if frame_later(target, node.recovery_frame):
                node.recovery_frame = target
                enter_async(index, now_ms)
                schedule_recovery(index, target)

    while events:
        event_ms, kind, index, generation = heapq.heappop(events)
        if kind == 0:
            index %= len(nodes)
        if event_ms > end_ms:
            break
        node = nodes[index]
        if kind not in (0, 3, 7) and generation != node.generation:
            stale_events += 1
            continue

        if kind == 0:
            sender = index % len(nodes)
            sender_node = nodes[sender]
            if sender_node.mode in (GPS, HOLDOVER):
                originate(sender, sender_node.mode, 0, event_ms)
            heapq.heappush(events, (event_ms + TDMA_SLOT_MS, 0,
                                   (index + 1) % len(nodes), 0))
            continue

        if kind == 1:
            if node.mode in (GPS, HOLDOVER):
                enter_async(index, event_ms)
            continue

        if kind == 3:
            node.gps_valid = True
            continue

        if kind == 4:
            node.reboot()
            if index in denied and event_ms < recovery_started_ms:
                node.gps_valid = False
                schedule_async(index, event_ms)
            else:
                node.gps_valid = True
                node.mode = ACQUIRING
                heapq.heappush(events, (event_ms + ACQUIRE_AND_ENTRY_MS,
                                       6, index, node.generation))
            continue

        if kind == 5:
            if (node.mode == ASYNC and node.gps_valid
                    and node.recovery_frame != 0
                    and frame_reached(frame_of(event_ms),
                                      node.recovery_frame)
                    and event_ms >= node.peer_until_ms
                    and event_ms >= node.self_until_ms
                    and not node.announcement_required):
                node.mode = ACQUIRING
                heapq.heappush(events, (event_ms + ACQUIRE_AND_ENTRY_MS,
                                       6, index, node.generation))
            elif node.mode == ASYNC and node.recovery_frame != 0:
                retry_ms = max(event_ms + TASK_PERIOD_MS,
                               node.peer_until_ms, node.self_until_ms)
                heapq.heappush(events, (retry_ms, 5, index,
                                       node.generation))
            continue

        if kind == 6:
            if node.mode == ACQUIRING and node.gps_valid:
                if (node.recovery_frame == 0
                        or frame_reached(frame_of(event_ms),
                                         node.recovery_frame)):
                    node.mode = GPS
                    recovery_completed_ms = max(recovery_completed_ms,
                                                event_ms)
            continue

        if kind == 7:
            if generation == cluster_generation and cluster_count == 1:
                for receiver in range(len(nodes)):
                    if receiver != cluster_sender:
                        receive(receiver, cluster_mode, cluster_target,
                                event_ms)
            continue

        if node.mode != ASYNC:
            continue
        if node.gps_valid and not node.announcement_required \
                and node.recovery_frame == 0:
            node.recovery_frame = next_recovery_frame(
                (frame_of(event_ms) + RECOVERY_MIN_LEAD_FRAMES) & 0xFFFFFFFF)
            schedule_recovery(index, node.recovery_frame)

        first_async_ms = min(first_async_ms, event_ms)

        advertised_mode = ASYNC
        target = 0
        if node.gps_valid and not node.announcement_required:
            advertised_mode = GPS
            target = node.recovery_frame
        if advertised_mode == ASYNC:
            node.self_until_ms = event_ms + PEER_HOLD_MS
            node.announcement_required = False

        originate(index, advertised_mode, target, event_ms)

        schedule_async(index, event_ms)

    if fallback_converged_ms < 0.0:
        fallback_converged_ms = end_ms + 1.0
    if first_async_ms < fallback_converged_ms:
        raise RuntimeError("asynchronous transmission preceded fleet fallback")
    if recovery_completed_ms < 0.0:
        recovery_completed_ms = end_ms + 1.0
    final_modes = tuple(sum(node.mode == mode for node in nodes)
                        for mode in (GPS, HOLDOVER, ASYNC, ACQUIRING))
    return Result(transmissions, collisions, cache_flushes,
                  fallback_converged_ms, recovery_completed_ms, stale_events,
                  final_modes)


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seeds", type=int, default=100)
    parser.add_argument("--duration", type=int, default=3600,
                        help="simulated seconds per seed")
    parser.add_argument("--stress-ppm", type=int, default=100)
    parser.add_argument("--denied-nodes", type=int, default=len(AC_IDS),
                        choices=range(1, len(AC_IDS) + 1),
                        help="GPS-denied nodes; remaining peers retain TDMA")
    args = parser.parse_args(argv)

    total_tx = 0
    total_collisions = 0
    total_flushes = 0
    total_stale_events = 0
    total_unrecovered = 0
    worst_collision_rate = 0.0
    worst_convergence_ms = 0.0
    worst_recovery_ms = 0.0
    for seed in range(1, args.seeds + 1):
        result = simulate(seed, args.duration, args.stress_ppm,
                          args.denied_nodes)
        total_tx += result.transmissions
        total_collisions += result.collisions
        total_flushes += result.cache_flushes
        total_stale_events += result.stale_events
        total_unrecovered += sum(result.final_modes[1:])
        worst_collision_rate = max(worst_collision_rate,
                                   result.collisions
                                   / max(result.transmissions, 1))
        worst_convergence_ms = max(worst_convergence_ms,
                                   result.fallback_converged_ms)
        worst_recovery_ms = max(worst_recovery_ms,
                                result.recovery_completed_ms)

    collision_rate = total_collisions / max(total_tx, 1)
    print("GPS-DENIED RANDOMIZED ACCESS")
    print(f"  seeds / duration : {args.seeds} / {args.duration} s")
    print(f"  denied / total   : {args.denied_nodes} / {len(AC_IDS)}")
    print(f"  oscillator drift : +/-{args.stress_ppm} ppm")
    print(f"  transmissions    : {total_tx}")
    print(f"  collisions       : {total_collisions} ({collision_rate:.3%})")
    print(f"  worst seed       : {worst_collision_rate:.3%}")
    if args.denied_nodes < len(AC_IDS):
        print(f"  worst convergence: {worst_convergence_ms / 1000.0:.3f} s")
    print(f"  worst recovery   : {worst_recovery_ms / 1000.0:.3f} s")
    print(f"  stale reboot evts: {total_stale_events}")
    print(f"  unrecovered nodes: {total_unrecovered}")
    print(f"  cache flushes    : {total_flushes}")

    convergence_limit_ms = HOLDOVER_MS
    recovery_limit_ms = args.duration * 400.0 \
                                + PEER_HOLD_MS \
                                + (RECOVERY_MIN_LEAD_FRAMES
                                    + RECOVERY_EPOCH_FRAMES) * 12_000 \
                        + ACQUIRE_AND_ENTRY_MS
    if (total_flushes != 0 or worst_collision_rate > 0.30
            or worst_convergence_ms > convergence_limit_ms
            or worst_recovery_ms > recovery_limit_ms
            or total_stale_events == 0
            or total_unrecovered != 0):
        print("  FAIL")
        return 1
    print("  PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())