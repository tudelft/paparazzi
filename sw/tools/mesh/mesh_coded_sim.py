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
"""Compare the current TDMA flood with asynchronous coded gossip.

This is a Phase 1 feasibility model, not flight firmware. The coded case
assumes that E52 terminal mode permits application-controlled broadcast
relaying. Confirm that assumption with three physical modems before airborne
integration.

AC_ID 0 is the explicitly configured, mobile GCS. Aircraft IDs are arbitrary
distinct values in 1..254. IDs are dictionary keys only: no slot, rank, array
index, or forwarding decision is derived from their numeric values.
"""

from __future__ import annotations

import argparse
from collections import defaultdict
from dataclasses import dataclass, field
import math
import random
import sys
from typing import Dict, Iterable, List, Optional, Sequence, Set, Tuple

import mesh_link_budget as mlb
from mesh_coding import CodedSymbol, GenerationDecoder, Membership, systematic_symbols
from mesh_link_sim import Drone, Shadowing, receive

GCS_AC_ID = 0
FAIR_SLOTS = 25
SUPERFRAME_S = 16.0
MAX_REUSE = 8


@dataclass
class GroundVehicle:
    x: float
    y: float
    mast_m: float
    speed_ms: float
    next_corner: int = 0

    def step(self, dt_s: float, width: float, height: float) -> None:
        corners = ((width, 0.0), (width, height), (0.0, height), (0.0, 0.0))
        remaining = self.speed_ms * dt_s
        while remaining > 0.0:
            target_x, target_y = corners[self.next_corner]
            delta_x, delta_y = target_x - self.x, target_y - self.y
            distance = math.hypot(delta_x, delta_y)
            if distance <= remaining:
                self.x, self.y = target_x, target_y
                remaining -= distance
                self.next_corner = (self.next_corner + 1) % len(corners)
            elif distance > 0.0:
                self.x += remaining * delta_x / distance
                self.y += remaining * delta_y / distance
                remaining = 0.0
            else:
                self.next_corner = (self.next_corner + 1) % len(corners)


@dataclass
class Pending:
    symbol: CodedSymbol
    ttl: int
    due_ms: int
    expires_ms: int
    priority: int


@dataclass
class Node:
    ac_id: int
    queue: List[Pending] = field(default_factory=list)
    decoders: Dict[Tuple[int, int], GenerationDecoder] = field(default_factory=dict)
    forward_count: Dict[Tuple[int, int], int] = field(default_factory=dict)
    next_tx_ms: int = 0
    queue_drops: int = 0


@dataclass
class Metrics:
    latest_ms: Dict[Tuple[int, int], int] = field(default_factory=dict)
    delivered: Set[Tuple[int, int, int, int]] = field(default_factory=set)
    ages_ms: List[int] = field(default_factory=list)
    gcs_ages_ms: List[int] = field(default_factory=list)
    originated: int = 0
    transmissions: int = 0
    collided_receptions: int = 0
    queue_drops: int = 0

    def note_delivery(self, source_id: int, receiver_id: int, generation: int,
                      sample_index: int, sample_ms: int) -> None:
        key = (source_id, receiver_id, generation, sample_index)
        if key in self.delivered:
            return
        self.delivered.add(key)
        latest_key = (source_id, receiver_id)
        if sample_ms > self.latest_ms.get(latest_key, -1):
            self.latest_ms[latest_key] = sample_ms

    def sample_age(self, now_ms: int, source_ids: Iterable[int],
                   receiver_ids: Iterable[int]) -> None:
        for source_id in source_ids:
            for receiver_id in receiver_ids:
                if receiver_id == source_id:
                    continue
                sample_ms = self.latest_ms.get((source_id, receiver_id))
                if sample_ms is None:
                    continue
                age_ms = now_ms - sample_ms
                self.ages_ms.append(age_ms)
                if receiver_id == GCS_AC_ID:
                    self.gcs_ages_ms.append(age_ms)


class Scenario:
    def __init__(self, args: argparse.Namespace, seed: int):
        self.args = args
        self.rng = random.Random(seed)
        ac_ids = ([int(value) for value in args.ac_ids.split(",")]
                  if args.ac_ids else
                  [GCS_AC_ID] + self.rng.sample(range(1, 255), args.aircraft))
        self.membership = Membership(tuple(ac_ids))
        if not self.membership.contains(GCS_AC_ID):
            raise ValueError("AC_ID 0 must identify the GCS")
        self.aircraft_ids = tuple(ac_id for ac_id in self.membership.ac_ids
                                  if ac_id != GCS_AC_ID)
        if not 1 <= len(self.aircraft_ids) <= 64:
            raise ValueError("aircraft count must be in 1..64")
        self.nodes = {ac_id: Node(ac_id) for ac_id in self.membership.ac_ids}
        self.drones = {
            ac_id: Drone(ac_id=ac_id,
                         x=self.rng.uniform(0.0, args.width),
                         y=self.rng.uniform(0.0, args.height),
                         alt_m=self.rng.uniform(args.alt_min, args.alt_max),
                         speed_ms=self.rng.uniform(12.0, 18.0),
                         wp=(self.rng.uniform(0.0, args.width),
                             self.rng.uniform(0.0, args.height)))
            for ac_id in self.aircraft_ids
        }
        self.gcs = GroundVehicle(0.0, 0.0, args.gcs_mast, args.gcs_speed)
        self.link_budget = mlb.LinkBudget(
            prop=mlb.Propagation(ground=args.ground), gcs_mast_m=args.gcs_mast)
        self.shadow = Shadowing(args.shadow_sigma, args.shadow_tau, self.rng)
        self.sample_times: Dict[Tuple[int, int], List[int]] = {}
        self.next_generation: Dict[int, int] = defaultdict(int)

    def position(self, ac_id: int) -> Tuple[float, float, float]:
        if ac_id == GCS_AC_ID:
            return self.gcs.x, self.gcs.y, self.gcs.mast_m
        drone = self.drones[ac_id]
        return drone.x, drone.y, drone.alt_m

    def bank(self, ac_id: int) -> float:
        return 0.0 if ac_id == GCS_AC_ID else self.drones[ac_id].bank_deg

    def move(self, dt_s: float) -> None:
        self.gcs.step(dt_s, self.args.width, self.args.height)
        for drone in self.drones.values():
            drone.step(dt_s, self.args.width, self.args.height, self.rng)

    def can_receive(self, sender_id: int, receiver_id: int) -> bool:
        sender_x, sender_y, sender_z = self.position(sender_id)
        receiver_x, receiver_y, receiver_z = self.position(receiver_id)
        distance = max(math.hypot(sender_x - receiver_x,
                                  sender_y - receiver_y), 1.0)
        success, _margin = receive(
            self.link_budget, self.shadow, distance, sender_z, receiver_z,
            sender_id, receiver_id, self.bank(sender_id))
        return success


def percentile(values: Sequence[int], fraction: float) -> float:
    if not values:
        return float("inf")
    ordered = sorted(values)
    index = min(len(ordered) - 1, math.ceil(fraction * len(ordered)) - 1)
    return ordered[index] / 1000.0


def coded_source_rate(aircraft_count: int, configured: Optional[float]) -> float:
    if configured is not None:
        return configured
    if aircraft_count <= 9:
        return 0.75
    if aircraft_count <= 16:
        return 0.40
    return 0.10


def legacy_source_rate(member_count: int) -> float:
    return min(float(MAX_REUSE), FAIR_SLOTS / member_count) / SUPERFRAME_S


def coded_forward_probability(aircraft_count: int,
                              configured: Optional[float]) -> float:
    if configured is not None:
        return configured
    if aircraft_count <= 9:
        return 0.15
    if aircraft_count <= 16:
        return 0.10
    return 0.01


def enqueue(node: Node, pending: Pending, queue_limit: int) -> None:
    if len(node.queue) >= queue_limit:
        worst = max(node.queue, key=lambda item: (item.priority, item.due_ms))
        if (worst.priority, worst.due_ms) <= (pending.priority, pending.due_ms):
            node.queue_drops += 1
            return
        node.queue.remove(worst)
        node.queue_drops += 1
    node.queue.append(pending)


def run_legacy(args: argparse.Namespace) -> Metrics:
    scenario = Scenario(args, args.seed)
    metrics = Metrics()
    interval_ms = round(1000.0 / legacy_source_rate(len(scenario.membership.ac_ids)))
    next_emit = {
        ac_id: round(index * interval_ms / len(scenario.aircraft_ids))
        for index, ac_id in enumerate(scenario.aircraft_ids)
    }
    sample_number: Dict[int, int] = defaultdict(int)

    for now_ms in range(0, args.duration * 1000, args.tick_ms):
        if now_ms % 1000 < args.tick_ms:
            scenario.move(1.0)
        for source_id in scenario.aircraft_ids:
            if now_ms < next_emit[source_id]:
                continue
            next_emit[source_id] += interval_ms
            current_sample = sample_number[source_id]
            sample_number[source_id] += 1
            metrics.originated += 1
            reached = {source_id}
            frontier = [source_id]
            while frontier:
                sender_id = frontier.pop(0)
                metrics.transmissions += 1
                for receiver_id in scenario.membership.ac_ids:
                    if receiver_id in reached:
                        continue
                    if scenario.can_receive(sender_id, receiver_id):
                        reached.add(receiver_id)
                        frontier.append(receiver_id)
            for receiver_id in reached:
                if receiver_id != source_id:
                    metrics.note_delivery(source_id, receiver_id, 0,
                                          current_sample, now_ms)
        if now_ms % args.age_sample_ms < args.tick_ms:
            metrics.sample_age(now_ms, scenario.aircraft_ids,
                               scenario.membership.ac_ids)
    return metrics


def add_generation(scenario: Scenario, metrics: Metrics, source_id: int,
                   payloads: List[bytes], sample_times: List[int],
                   now_ms: int, args: argparse.Namespace) -> None:
    generation = scenario.next_generation[source_id]
    scenario.next_generation[source_id] = (generation + 1) & 0xFFFF
    scenario.sample_times[(source_id, generation)] = list(sample_times)
    symbols = systematic_symbols(source_id, generation, payloads, sample_times)
    encoder = GenerationDecoder(source_id, generation, args.generation_size,
                                args.symbol_bytes)
    for symbol in symbols:
        encoder.add(symbol)
    scheduled = symbols + [encoder.recode(scenario.rng, args.code_density)
                           for _ in range(args.repair_symbols)]
    jitter_ms = scenario.rng.randint(0, args.csma_max_ms)
    for offset, symbol in enumerate(scheduled):
        enqueue(scenario.nodes[source_id],
                Pending(symbol=symbol, ttl=args.ttl,
                        due_ms=now_ms + jitter_ms + offset * args.source_spacing_ms,
                        expires_ms=now_ms + args.generation_lifetime_ms,
                        priority=0 if offset < len(symbols) else 1),
                args.queue_limit)


def note_symbol(scenario: Scenario, metrics: Metrics, receiver_id: int,
                pending: Pending, now_ms: int, args: argparse.Namespace) -> None:
    symbol = pending.symbol
    if receiver_id == symbol.source_id:
        return
    key = (symbol.source_id, symbol.generation)
    receiver = scenario.nodes[receiver_id]
    decoder = receiver.decoders.get(key)
    if decoder is None:
        decoder = GenerationDecoder(symbol.source_id, symbol.generation,
                                    args.generation_size, args.symbol_bytes)
        receiver.decoders[key] = decoder
    if not decoder.add(symbol):
        return

    sample_times = scenario.sample_times[key]
    nonzero = [index for index, coefficient in enumerate(symbol.coefficients)
               if coefficient]
    if len(nonzero) == 1 and symbol.coefficients[nonzero[0]] == 1:
        index = nonzero[0]
        metrics.note_delivery(symbol.source_id, receiver_id, symbol.generation,
                              index, sample_times[index])
    if decoder.complete:
        for index, sample_ms in enumerate(sample_times):
            metrics.note_delivery(symbol.source_id, receiver_id,
                                  symbol.generation, index, sample_ms)

    forwarded = receiver.forward_count.get(key, 0)
    probability = coded_forward_probability(len(scenario.aircraft_ids),
                                            args.forward_probability)
    if (pending.ttl > 1 and forwarded < args.max_forwards
            and scenario.rng.random() < probability):
        receiver.forward_count[key] = forwarded + 1
        enqueue(receiver,
                Pending(symbol=decoder.recode(scenario.rng, args.code_density),
                        ttl=pending.ttl - 1,
                        due_ms=now_ms + scenario.rng.randint(
                            args.forward_min_ms, args.forward_max_ms),
                        expires_ms=pending.expires_ms, priority=2),
                args.queue_limit)


def run_coded(args: argparse.Namespace) -> Metrics:
    scenario = Scenario(args, args.seed)
    metrics = Metrics()
    rate = coded_source_rate(len(scenario.aircraft_ids), args.source_rate)
    sample_interval_ms = round(1000.0 / rate)
    next_sample = {ac_id: scenario.rng.randrange(sample_interval_ms)
                   for ac_id in scenario.aircraft_ids}
    payloads: Dict[int, List[bytes]] = defaultdict(list)
    sample_times: Dict[int, List[int]] = defaultdict(list)

    for now_ms in range(0, args.duration * 1000, args.tick_ms):
        if now_ms % 1000 < args.tick_ms:
            scenario.move(1.0)

        for source_id in scenario.aircraft_ids:
            while now_ms >= next_sample[source_id]:
                sample_ms = next_sample[source_id]
                next_sample[source_id] += sample_interval_ms
                sequence = metrics.originated & 0xFFFF
                payloads[source_id].append(bytes(
                    (source_id + sequence + offset) & 0xFF
                    for offset in range(args.symbol_bytes)))
                sample_times[source_id].append(sample_ms)
                metrics.originated += 1
            if len(payloads[source_id]) >= args.generation_size:
                add_generation(scenario, metrics, source_id,
                               payloads[source_id][:args.generation_size],
                               sample_times[source_id][:args.generation_size],
                               now_ms, args)
                del payloads[source_id][:args.generation_size]
                del sample_times[source_id][:args.generation_size]

        contenders: List[Tuple[Node, Pending]] = []
        for node in scenario.nodes.values():
            expired = [item for item in node.queue if item.expires_ms <= now_ms]
            for item in expired:
                node.queue.remove(item)
                node.queue_drops += 1
            ready = [item for item in node.queue if item.due_ms <= now_ms]
            if ready and node.next_tx_ms <= now_ms:
                selected = min(ready, key=lambda item: (item.priority, item.due_ms))
                node.queue.remove(selected)
                contenders.append((node, selected))

        receptions: Dict[int, List[Pending]] = defaultdict(list)
        transmitting_ids = {node.ac_id for node, _pending in contenders}
        for sender, pending in contenders:
            metrics.transmissions += 1
            sender.next_tx_ms = now_ms + args.air_time_ms + scenario.rng.randint(
                args.csma_min_ms, args.csma_max_ms)
            for receiver_id in scenario.membership.ac_ids:
                if receiver_id == sender.ac_id:
                    continue
                if scenario.can_receive(sender.ac_id, receiver_id):
                    receptions[receiver_id].append(pending)

        for receiver_id, heard in receptions.items():
            if receiver_id in transmitting_ids or len(heard) != 1:
                metrics.collided_receptions += 1
                continue
            note_symbol(scenario, metrics, receiver_id, heard[0], now_ms, args)

        if now_ms % args.age_sample_ms < args.tick_ms:
            metrics.sample_age(now_ms, scenario.aircraft_ids,
                               scenario.membership.ac_ids)

    metrics.queue_drops = sum(node.queue_drops for node in scenario.nodes.values())
    return metrics


def report(name: str, metrics: Metrics, args: argparse.Namespace,
           member_count: int) -> None:
    possible = metrics.originated * (member_count - 1)
    if len(metrics.delivered) > possible:
        raise AssertionError("unique deliveries exceed all-to-all opportunities")
    aggregate_airtime = (metrics.transmissions * args.air_time_ms
                         / (args.duration * 1000.0))
    print(f"{name}:")
    print(f"  samples {metrics.originated}, transmissions {metrics.transmissions}, "
          f"unique deliveries {len(metrics.delivered)}")
    print(f"  delivery {len(metrics.delivered) / max(1, possible):.3f}; "
          f"all-peer AoI p50/p95/p99 "
          f"{percentile(metrics.ages_ms, 0.50):.2f}/"
          f"{percentile(metrics.ages_ms, 0.95):.2f}/"
          f"{percentile(metrics.ages_ms, 0.99):.2f} s")
    print(f"  mobile-GCS AoI p95 {percentile(metrics.gcs_ages_ms, 0.95):.2f} s; "
          f"collided receptions {metrics.collided_receptions}; "
          f"queue drops {metrics.queue_drops}")
    print(f"  aggregate radio airtime {aggregate_airtime * 100.0:.1f}% "
          "(sum across all radios, not per-radio duty)")


def parse_args(argv: Optional[Sequence[str]]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--aircraft", type=int, default=9)
    parser.add_argument("--ac-ids", default=None,
                        help="IDs including GCS 0; aircraft use arbitrary "
                             "distinct values in 1..254")
    parser.add_argument("--duration", type=int, default=120)
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--width", type=float, default=3000.0)
    parser.add_argument("--height", type=float, default=3000.0)
    parser.add_argument("--alt-min", type=float, default=50.0)
    parser.add_argument("--alt-max", type=float, default=120.0)
    parser.add_argument("--gcs-mast", type=float, default=4.0)
    parser.add_argument("--gcs-speed", type=float, default=12.0)
    parser.add_argument("--ground", choices=sorted(mlb.GROUND_TYPES),
                        default="average")
    parser.add_argument("--shadow-sigma", type=float, default=3.0)
    parser.add_argument("--shadow-tau", type=float, default=30.0)
    parser.add_argument("--source-rate", type=float, default=None)
    parser.add_argument("--generation-size", type=int, default=2)
    parser.add_argument("--symbol-bytes", type=int, default=22)
    parser.add_argument("--repair-symbols", type=int, default=1)
    parser.add_argument("--code-density", type=float, default=0.5)
    parser.add_argument("--ttl", type=int, default=4)
    parser.add_argument("--max-forwards", type=int, default=1)
    parser.add_argument("--forward-probability", type=float, default=None,
                        help="override adaptive forwarding probability (default "
                             "0.15 <=9, 0.10 <=16, 0.01 above 16 aircraft)")
    parser.add_argument("--forward-min-ms", type=int, default=40)
    parser.add_argument("--forward-max-ms", type=int, default=160)
    parser.add_argument("--source-spacing-ms", type=int, default=40)
    parser.add_argument("--generation-lifetime-ms", type=int, default=3000)
    parser.add_argument("--queue-limit", type=int, default=5)
    parser.add_argument("--csma-min-ms", type=int, default=20)
    parser.add_argument("--csma-max-ms", type=int, default=60)
    parser.add_argument("--air-time-ms", type=int, default=20)
    parser.add_argument("--tick-ms", type=int, default=5)
    parser.add_argument("--age-sample-ms", type=int, default=1000)
    parser.add_argument("--mode", choices=("both", "legacy", "coded"),
                        default="both")
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    try:
        scenario = Scenario(args, args.seed)
        if not 1 <= args.generation_size <= 16:
            raise ValueError("generation size must be in 1..16")
        if args.source_rate is not None and args.source_rate <= 0.0:
            raise ValueError("source rate must be positive")
        if args.duration <= 0 or args.tick_ms <= 0 or args.air_time_ms <= 0:
            raise ValueError("duration, tick, and air time must be positive")
        if args.gcs_speed < 0.0:
            raise ValueError("GCS speed cannot be negative")
        if args.csma_min_ms < 0 or args.csma_max_ms < args.csma_min_ms:
            raise ValueError("invalid CSMA interval")
        if args.forward_max_ms < args.forward_min_ms:
            raise ValueError("invalid forwarding interval")
        if (args.forward_probability is not None
                and not 0.0 <= args.forward_probability <= 1.0):
            raise ValueError("forwarding probability must be in 0..1")
    except ValueError as error:
        print(error, file=sys.stderr)
        return 2

    aircraft_count = len(scenario.aircraft_ids)
    member_count = len(scenario.membership.ac_ids)
    print(f"mesh experiment: {aircraft_count} aircraft + mobile GCS AC_ID 0, "
          f"IDs {scenario.membership.ac_ids}")
    print(f"  coded source rate "
          f"{coded_source_rate(aircraft_count, args.source_rate):.2f} Hz; "
            f"forward probability "
            f"{coded_forward_probability(aircraft_count, args.forward_probability):.2f}; "
            f"legacy fair-share rate {legacy_source_rate(member_count):.3f} Hz")
    if args.mode in ("both", "legacy"):
        report("legacy TDMA flood", run_legacy(args), args, member_count)
    if args.mode in ("both", "coded"):
        report("asynchronous coded gossip", run_coded(args), args, member_count)
    return 0


if __name__ == "__main__":
    sys.exit(main())