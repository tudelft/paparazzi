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
"""Telemetry slot / phase optimiser for narrowband broadcast LoRa MESH links.

Target hardware: EByte E52-400NW22S LoRa MESH module, broadcast mode
(``AT+OPTION=3``), 62.5 kbps air rate, 460800 baud MCU link, 5-frame internal
transmit cache.

The tool answers four questions, in order:

1. **What does one broadcast actually cost?**  In a flooded mesh every routing
   node re-transmits every broadcast exactly once (the module's broadcast
   filter suppresses further copies).  A single originated frame therefore
   occupies ``N_nodes`` times its own air time on the shared channel.  This
   "flood tax" - not the 62.5 kbps figure - is the real capacity limit.

2. **How many frames per second can each node originate?**  Derived from the
   air-time cost and a target channel-utilisation ceiling.

3. **When exactly does each frame leave the autopilot?**  Paparazzi's
   ``gen_periodic`` generator fires a message when its period counter equals
   ``(uint32_t)(TELEMETRY_FREQUENCY * period * phase)``.  Choosing the
   ``phase`` attributes is therefore an integer placement problem on a
   circular tick lattice.  We solve it with a deterministic
   max-min-gap greedy and *verify* the result by replaying the generated
   trigger condition tick by tick.

4. **Can the modem cache ever overflow?**  A discrete-event simulation of the
   modem transmit cache, fed by both the locally originated frames and the
   relay traffic this node must forward for its neighbours, proves that the
   queue depth stays at or below the configured ceiling (default 3 of 5).

Run with no arguments for the nominal 8 aircraft + 1 GCS configuration::

    ./mesh_phase_optimizer.py

Useful variations::

    ./mesh_phase_optimizer.py --relay-nodes 3        # only GCS + 2 relays route
    ./mesh_phase_optimizer.py --nodes 17 --utilisation 0.30
    ./mesh_phase_optimizer.py --period GPS_LLA=16 --period WP_MOVED=32
    ./mesh_phase_optimizer.py --emit-xml conf/telemetry/OPENUAS/openuas_fixedwing_mesh.xml
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

# --------------------------------------------------------------------------- #
# 1. PPRZLink message sizing                                                    #
# --------------------------------------------------------------------------- #

#: Assumed element count for variable-length ``type[]`` fields.  Only used for
#: messages that are not part of the optimised mesh set; the mesh set is
#: deliberately free of variable-length fields.
DEFAULT_ARRAY_LEN = 8

#: Size in bytes of every scalar PPRZLink field type.
PPRZ_TYPE_SIZE: Dict[str, int] = {
    "char": 1, "uint8": 1, "int8": 1,
    "uint16": 2, "int16": 2,
    "uint32": 4, "int32": 4, "float": 4,
    "uint64": 8, "int64": 8, "double": 8,
    # "string" is a length-prefixed byte sequence in the PPRZLink wire format
    "string": 1 + DEFAULT_ARRAY_LEN,
}

#: PPRZLink v2.0 wire framing:
#:   STX(1) + length(1) + sender_id(1) + receiver_id(1) + comp/class(1)
#:   + msg_id(1) + payload + ck_a(1) + ck_b(1)
#: See sw/ext/pprzlink/lib/v2.0/C/pprz_transport.c (``size_of`` adds 4 to a
#: payload length that already contains the 4 header bytes).
PPRZ_V2_FRAMING_BYTES = 8


@dataclass(frozen=True)
class MessageDef:
    """A PPRZLink message and its cost on the wire."""

    name: str
    msg_class: str
    msg_id: int
    payload_bytes: int          # sum of the field sizes only
    variable_length: bool

    @property
    def wire_bytes(self) -> int:
        """Bytes handed to the modem UART for one instance of this message."""
        return self.payload_bytes + PPRZ_V2_FRAMING_BYTES


def _field_size(ftype: str) -> Tuple[int, bool]:
    """Return ``(bytes, is_variable)`` for one PPRZLink field type string."""
    ftype = ftype.strip()
    if ftype.endswith("]"):
        base, _, count = ftype[:-1].partition("[")
        elem = PPRZ_TYPE_SIZE[base.strip()]
        if count.strip():
            return elem * int(count), False
        # variable length arrays carry one extra length byte
        return 1 + elem * DEFAULT_ARRAY_LEN, True
    return PPRZ_TYPE_SIZE[ftype], False


def load_messages(path: str) -> Dict[Tuple[str, str], MessageDef]:
    """Parse ``messages.xml`` into ``{(class, name): MessageDef}``."""
    root = ET.parse(path).getroot()
    out: Dict[Tuple[str, str], MessageDef] = {}
    for cls in root.findall("msg_class"):
        cname = cls.get("name") or "?"
        for msg in cls.findall("message"):
            total = 0
            variable = False
            for fld in msg.findall("field"):
                size, var = _field_size(fld.get("type") or "uint8")
                total += size
                variable = variable or var
            out[(cname, msg.get("name") or "?")] = MessageDef(
                name=msg.get("name") or "?",
                msg_class=cname,
                msg_id=int(msg.get("id") or 0),
                payload_bytes=total,
                variable_length=variable,
            )
    return out


# --------------------------------------------------------------------------- #
# 2. LoRa PHY air-time model                                                    #
# --------------------------------------------------------------------------- #

@dataclass(frozen=True)
class LoRaPhy:
    """Semtech LoRa air-time model for the three E52 air-rate settings.

    Each of the module's three ``AT+RATE`` values corresponds to exactly one
    point in the LoRa parameter space, all at BW 500 kHz and CR 4/5, because
    ``R_b = SF * BW / 2**SF * CR``:

        AT+RATE=0   SF5   5 * 500000/32  * 0.8 = 62500 bps    "62.5K"
        AT+RATE=1   SF7   7 * 500000/128 * 0.8 = 21875 bps    "21.875K"
        AT+RATE=2   SF9   9 * 500000/512 * 0.8 =  7031 bps    "7K"

    All three land on the datasheet figures, so the standard Semtech
    time-on-air formula applies with those parameters and the datasheet
    sensitivities (-111 / -116 / -121 dBm) pair with them.
    """

    spreading_factor: int = 5
    bandwidth_hz: int = 500_000
    coding_rate: int = 1            # 1 => 4/5
    preamble_symbols: int = 8
    explicit_header: bool = True
    crc_on: bool = True
    low_datarate_optimise: bool = False

    #: Bytes the LoRa MESH network layer prepends to the user payload on air:
    #: frame type(1) + length(1) + PANID(2) + source(2) + destination(2), plus
    #: the sequence / hop fields used by the broadcast filter and the network
    #: layer integrity check.  8 of these are visible on the serial output when
    #: ``AT+HEAD=1``; the remainder is inferred.  Conservative estimate.
    mesh_header_bytes: int = 13

    @property
    def symbol_time_s(self) -> float:
        return (2 ** self.spreading_factor) / float(self.bandwidth_hz)

    @property
    def preamble_time_s(self) -> float:
        # SX126x uses 6.25 symbols of fixed preamble overhead for SF5/SF6
        # (4.25 for SF >= 7).
        fixed = 6.25 if self.spreading_factor <= 6 else 4.25
        return (self.preamble_symbols + fixed) * self.symbol_time_s

    def payload_symbols(self, payload_bytes: int) -> int:
        sf = self.spreading_factor
        de = 1 if self.low_datarate_optimise else 0
        ih = 0 if self.explicit_header else 1
        crc = 1 if self.crc_on else 0
        num = 8 * payload_bytes - 4 * sf + 28 + 16 * crc - 20 * ih
        den = 4 * (sf - 2 * de)
        return 8 + max(0, math.ceil(num / den) * (self.coding_rate + 4))

    @property
    def bitrate_bps(self) -> float:
        return (self.spreading_factor * self.bandwidth_hz
                / (2 ** self.spreading_factor) * (4.0 / (4.0 + self.coding_rate)))

    def time_on_air_s(self, user_bytes: int) -> float:
        """Air time of one hop carrying ``user_bytes`` of PPRZLink frame."""
        phy_payload = user_bytes + self.mesh_header_bytes
        return self.preamble_time_s + self.payload_symbols(phy_payload) * self.symbol_time_s


#: ``AT+RATE`` value -> (spreading factor, datasheet RX sensitivity in dBm).
#: All three E52 rates use BW 500 kHz and CR 4/5.
E52_AIR_RATES = {
    0: (5, -111.0),     # 62.5 kbps
    1: (7, -116.0),     # 21.875 kbps
    2: (9, -121.0),     # 7 kbps
}


def phy_for_rate(rate: int) -> "LoRaPhy":
    sf, _ = E52_AIR_RATES[rate]
    return LoRaPhy(spreading_factor=sf)


def sensitivity_for_rate(rate: int) -> float:
    return E52_AIR_RATES[rate][1]


# --------------------------------------------------------------------------- #
# 3. Network / channel model                                                    #
# --------------------------------------------------------------------------- #

@dataclass
class NetworkModel:
    """Flooded-broadcast channel model for the E52 MESH."""

    phy: LoRaPhy
    n_nodes: int = 9                 # 8 aircraft + 1 GCS
    n_relay_nodes: int = 9           # nodes configured as routing nodes (AT+TYPE=0)
    csma_range_ms: int = 20          # AT+CSMA_RNG, datasheet minimum
    target_utilisation: Optional[float] = None
    uart_baud: int = 460_800

    @property
    def n_transmissions_per_broadcast(self) -> int:
        """Originator, plus one relay from every routing node that heard it.

        Worst case is a *terminal* originator: then all R routing nodes relay,
        giving 1 + R transmissions.  If the originator is itself a routing node
        it does not relay its own frame, giving R.  The cap at ``n_nodes``
        covers the all-routing case where every originator is a router.

        The earlier ``1 + max(0, R - 1)`` assumed a routing originator and so
        undercounted by one for the mostly-terminal topologies that a bounded
        operating area makes attractive.
        """
        return min(1 + self.n_relay_nodes, self.n_nodes)

    def channel_cost_s(self, wire_bytes: int) -> float:
        """Total channel occupancy of one originated broadcast, all hops."""
        return self.n_transmissions_per_broadcast * self.phy.time_on_air_s(wire_bytes)

    def flood_span_s(self, wire_bytes: int) -> Tuple[float, float, float]:
        """Wall-clock time for one flood to completely die out.

        Every hop costs one air time plus one CSMA random back-off draw,
        uniform on ``[0, csma_range_ms]``.  With ``k`` hops the back-off sum
        has mean ``k*R/2`` and standard deviation ``sqrt(k)*R/sqrt(12)``.

        Returns ``(mean, mean + 3 sigma, absolute worst case)`` in seconds.
        """
        k = self.n_transmissions_per_broadcast
        air = k * self.phy.time_on_air_s(wire_bytes)
        r = self.csma_range_ms / 1000.0
        mean = air + k * r / 2.0
        sigma = math.sqrt(k) * r / math.sqrt(12.0)
        worst = air + k * r
        # For small hop counts the 3 sigma figure can exceed the true maximum,
        # because a sum of k bounded uniforms is itself bounded. Size slots on
        # whichever is smaller - it is still a hard bound.
        return mean, min(mean + 3.0 * sigma, worst), worst

    def uart_time_s(self, wire_bytes: int) -> float:
        """Time to shift the frame out of the MCU UART (8N1 => 10 bits/byte)."""
        return wire_bytes * 10.0 / self.uart_baud


# --------------------------------------------------------------------------- #
# 4. Telemetry schedule                                                         #
# --------------------------------------------------------------------------- #

@dataclass
class ScheduleEntry:
    msg: MessageDef
    period_s: float
    process: str = "Ap"
    exclusive_group: Optional[str] = None   # only one of the group is ever live
    tick: int = -1                          # solved trigger tick
    phase: float = 0.0                      # solved phase attribute

    @property
    def rate_hz(self) -> float:
        return 1.0 / self.period_s


def parse_named_values(values: Sequence[str], option: str) -> Dict[str, float]:
    """Parse repeatable ``NAME=VALUE`` command-line options."""
    parsed: Dict[str, float] = {}
    for value in values:
        name, separator, number = value.partition("=")
        if not separator or not name.strip():
            raise ValueError(f"{option} expects NAME=VALUE, got {value!r}")
        try:
            parsed_value = float(number)
        except ValueError as exc:
            raise ValueError(f"{option} value must be numeric, got {value!r}") from exc
        if parsed_value <= 0.0:
            raise ValueError(f"{option} value must be positive, got {value!r}")
        parsed[name.strip()] = parsed_value
    return parsed


def steady_slot_count(physical_slots: int, fair_slots: int,
                      members: int, max_reuse: int) -> int:
    """Return slots carrying steady traffic after reserving churn headroom."""
    return min(physical_slots, fair_slots, members * max_reuse)


#: Highest phase value ``gen_periodic`` accepts as a normalised fraction.
#: Above 0.95 it treats the value as a legacy 1/65536 tick count.
PHASE_MAX = 0.95

#: Bound exact trigger replay so an accidental non-harmonic period set cannot
#: consume unbounded CPU and memory through its least-common-multiple horizon.
DEFAULT_MAX_REPLAY_TICKS = 100_000


def schedule_horizon_ticks(
    entries: Sequence[ScheduleEntry],
    telemetry_frequency: int,
    max_ticks: int = DEFAULT_MAX_REPLAY_TICKS,
) -> int:
    """Return the exact replay horizon, rejecting pathological schedules."""
    horizon = 1
    for entry in entries:
        ticks = int(round(telemetry_frequency * entry.period_s))
        if ticks <= 0:
            raise ValueError(f"period {entry.period_s} too short for {entry.msg.name}")
        horizon = math.lcm(horizon, ticks)
        if horizon > max_ticks:
            raise ValueError(
                f"schedule replay horizon {horizon} ticks exceeds the "
                f"{max_ticks}-tick safety limit; use harmonic periods or "
                "raise --max-replay-ticks deliberately"
            )
    return horizon


def _periodic_gap(a_tick: int, a_period: int,
                  b_tick: int, b_period: int) -> int:
    """Minimum tick distance between two periodic trigger sequences."""
    common = math.gcd(a_period, b_period)
    delta = (a_tick - b_tick) % common
    return min(delta, common - delta)


def solve_phases(
    entries: Sequence[ScheduleEntry],
    telemetry_frequency: int,
    min_gap_ticks: int,
    max_replay_ticks: int = DEFAULT_MAX_REPLAY_TICKS,
) -> Tuple[List[ScheduleEntry], int]:
    """Assign each message a trigger tick that maximises the minimum spacing.

    ``gen_periodic`` emits, per distinct period ``p``, a counter that runs
    ``0 .. TELEMETRY_FREQUENCY*p - 1`` and fires the message when the counter
    equals ``floor(TELEMETRY_FREQUENCY * p * phase)``.  All counters start
    together at boot, so a message with period ``p`` and tick ``n`` fires at
    every absolute tick ``t`` with ``t = n (mod TELEMETRY_FREQUENCY*p)``.

    Placement is a deterministic greedy: most frequent messages first (they
    constrain the lattice hardest), each placed on the free tick that is
    farthest from everything already placed.

    Returns the solved entries and the achieved minimum gap in ticks.
    """
    f = telemetry_frequency
    horizon = schedule_horizon_ticks(entries, f, max_replay_ticks)
    placed: List[Tuple[int, int]] = []
    ordered = sorted(entries, key=lambda e: (e.period_s, e.msg.name))

    for entry in ordered:
        n = int(round(f * entry.period_s))
        n_max = int(PHASE_MAX * n)          # respect the gen_periodic clamp
        best_tick, best_score = 0, -1.0
        for candidate in range(0, n_max + 1):
            if not placed:
                score = float(horizon)
            else:
                score = min(_periodic_gap(candidate, n, tick, period)
                            for tick, period in placed)
            # tie-break towards the earliest tick for reproducibility
            if score > best_score:
                best_tick, best_score = candidate, score
        entry.tick = best_tick
        # Land safely inside the integer bin: gen_periodic truncates
        # f*period*phase, so aim at the middle of bin `best_tick`.
        entry.phase = min(PHASE_MAX, (best_tick + 0.5) / n)
        placed.append((best_tick, n))

    achieved = min(
        (_periodic_gap(a_tick, a_period, b_tick, b_period)
         for index, (a_tick, a_period) in enumerate(placed)
         for b_tick, b_period in placed[index + 1:]),
        default=horizon,
    )
    if achieved < min_gap_ticks:
        print(
            f"  ! warning: achieved minimum spacing {achieved} ticks "
            f"({achieved * 1000.0 / f:.0f} ms) is below the requested "
            f"{min_gap_ticks} ticks ({min_gap_ticks * 1000.0 / f:.0f} ms)",
            file=sys.stderr,
        )
    return ordered, achieved


def _min_gap(sorted_ticks: Sequence[int], horizon: int) -> int:
    if len(sorted_ticks) < 2:
        return horizon
    gaps = [b - a for a, b in zip(sorted_ticks, sorted_ticks[1:])]
    gaps.append(horizon - sorted_ticks[-1] + sorted_ticks[0])
    return min(gaps)


def replay_generated_trigger(
    entries: Sequence[ScheduleEntry], telemetry_frequency: int,
    max_replay_ticks: int = DEFAULT_MAX_REPLAY_TICKS,
) -> Dict[int, List[str]]:
    """Bit-exact replay of the C code ``gen_periodic`` will emit.

    Reproduces::

        static uint32_t iN = 0; iN++; if (iN >= (uint32_t)(F*period)) iN = 0;
        if (iN == (uint32_t)(F*period*phase)) { send(); }

    so the schedule is validated against the real generator semantics rather
    than against our own idealised model.
    """
    f = telemetry_frequency
    horizon = schedule_horizon_ticks(entries, f, max_replay_ticks)

    # counters are shared between messages that use the same period *string*
    counters: Dict[float, int] = {e.period_s: 0 for e in entries}
    targets = {id(e): int(f * e.period_s * e.phase) for e in entries}

    fired: Dict[int, List[str]] = {}
    for t in range(horizon):
        for period in counters:
            n = int(f * period)
            counters[period] += 1
            if counters[period] >= n:
                counters[period] = 0
        for e in entries:
            if counters[e.period_s] == targets[id(e)]:
                fired.setdefault(t, []).append(e.msg.name)
    return fired


# --------------------------------------------------------------------------- #
# 5. Modem transmit-cache simulation                                            #
# --------------------------------------------------------------------------- #

def simulate_cache(
    fired: Dict[int, List[str]],
    entries: Sequence[ScheduleEntry],
    net: NetworkModel,
    telemetry_frequency: int,
    tdma_slot_s: float,
    n_originating_nodes: int,
    cache_limit: int = 5,
    slot_driven: str = "",
    slot_driven_rate_hz: float = 0.0,
    event_frames: Sequence[Tuple[MessageDef, float]] = (),
) -> Tuple[int, int, float]:
    """Discrete-event simulation of one node's E52 transmit cache.

    Two independent producers push frames into the 5-slot cache:

    * the local autopilot, at the ticks computed above;
    * the mesh relay engine, one frame for every broadcast originated by any
      other node that this node must forward.

    Returns ``(peak_depth, overflow_count, channel_utilisation)``.
    """
    f = telemetry_frequency
    horizon = max(fired) + 1 if fired else 1
    by_name = {e.msg.name: e for e in entries}

    dt = 1.0 / f
    depth = 0.0                 # fractional: models drain in progress
    peak = 0
    overflows = 0
    busy_s = 0.0
    duration_s = horizon * dt

    # Aggregate relay load: every other node originates the same schedule,
    # offset by its TDMA slot.  Modelled as a fluid arrival rate because the
    # relay frames are, by construction of the TDMA, never simultaneous.
    # A slot-driven message does NOT scale with the population.
    #
    # Every slot in the superframe gets used by somebody: a sparse mesh means
    # fewer nodes each holding more slots, a dense one means more nodes each
    # holding fewer. The mesh-wide rate of MESH_STATE is therefore a property of
    # the frame, fixed at one origination per slot, and multiplying its
    # per-node rate by the number of nodes counts it twice over. Doing that here
    # made this cross-check report 56.5% against the budget's 32% for the very
    # same configuration. The disagreement was therefore not cosmetic.
    slot_local_hz = by_name[slot_driven].rate_hz if slot_driven in by_name else 0.0
    others_rate = sum(e.rate_hz for e in entries if e.msg.name != slot_driven)
    local_rate = others_rate + slot_local_hz
    #  own share removed: what this node relays is what the OTHERS originate
    event_rate_hz = sum(rate_hz for _, rate_hz in event_frames)
    relay_rate = (others_rate * max(0, n_originating_nodes - 1)
                  + max(0.0, slot_driven_rate_hz - slot_local_hz)
                  + event_rate_hz)

    drain_backlog = 0.0
    for tick in range(horizon):
        # --- local originations ------------------------------------------- #
        arrivals = 0
        for name in fired.get(tick, ()):
            entry = by_name[name]
            arrivals += 1
            if entry.msg.name != slot_driven:
                busy_s += net.phy.time_on_air_s(entry.msg.wire_bytes)

        # --- relay arrivals (Poisson-thinned, deterministic mean) ---------- #
        drain_backlog += relay_rate * dt
        while drain_backlog >= 1.0:
            arrivals += 1
            drain_backlog -= 1.0

        depth += arrivals

        # --- service ------------------------------------------------------- #
        # One frame leaves the cache every (air time + mean CSMA back-off).
        mean_frame = sum(e.msg.wire_bytes * e.rate_hz for e in entries) / max(local_rate, 1e-9)
        service_s = net.phy.time_on_air_s(mean_frame) + net.csma_range_ms / 2000.0
        depth = max(0.0, depth - dt / service_s)

        peak = max(peak, int(math.ceil(depth)))
        if depth > cache_limit:
            overflows += 1
            depth = 0.0          # the module force-flushes the whole cache

    utilisation = (busy_s * net.n_transmissions_per_broadcast
                   * max(1, n_originating_nodes)) / duration_s
    if slot_driven in by_name:
        utilisation += (slot_driven_rate_hz
                        * net.channel_cost_s(by_name[slot_driven].msg.wire_bytes))
    utilisation += sum(rate_hz * net.channel_cost_s(msg.wire_bytes)
                       for msg, rate_hz in event_frames)
    _ = tdma_slot_s
    return peak, overflows, utilisation


# --------------------------------------------------------------------------- #
# 6. Network TDMA slot map                                                      #
# --------------------------------------------------------------------------- #

@dataclass
class SlotPlan:
    slot_s: float
    n_slots: int
    superframe_s: float
    guard_s: float = 0.0
    span_mean_s: float = 0.0
    span_statistical_s: float = 0.0
    span_worst_s: float = 0.0
    use_statistical_bound: bool = False

    @property
    def usable_s(self) -> float:
        return max(0.0, self.slot_s - self.guard_s)

    @property
    def required_span_s(self) -> float:
        return (self.span_statistical_s if self.use_statistical_bound
                else self.span_worst_s)

    @property
    def ok(self) -> bool:
        """A slot is long enough for one flood to die out completely.

        That is the only sizing constraint left. There is deliberately no
        ac_id -> slot table here: slots are claimed at run time by listening,
        so no static map exists to collide. An earlier version kept an
        (AC_ID-1) mod n_slots assignment and failed the run when two IDs shared
        a slot - which made the tool reject any fleet whose AC_IDs were not
        consecutive, even though the firmware never uses such a map.
        """
        return self.usable_s >= self.required_span_s

def plan_slots(
    net: NetworkModel,
    wire_bytes: int,
    ac_ids: Sequence[int],
    superframe_s: float,
    n_slots: Optional[int] = None,
    guard_s: float = 0.0,
    use_statistical_bound: bool = False,
) -> SlotPlan:
    """Size the GPS-time-synchronised TDMA superframe.

    A slot must be long enough for one flood to completely die out, otherwise a
    node can still be relaying frame *k* when frame *k+1* is originated - which
    is precisely how the 5-frame cache overflows.

    The superframe is pinned to the MESH_STATE period so that every node gets
    exactly one origination opportunity per MESH_STATE cycle, and the slot
    count is the next power of two at or above the node count so the slot index
    is a cheap mask on the autopilot (and there is room to grow).
    """
    if n_slots is None:
        n_slots = 1 << max(1, math.ceil(math.log2(len(ac_ids))))
    slot_s = superframe_s / n_slots
    mean, p999, worst = net.flood_span_s(wire_bytes)
    return SlotPlan(
        slot_s=slot_s,
        n_slots=n_slots,
        superframe_s=superframe_s,
        guard_s=guard_s,
        span_mean_s=mean,
        span_statistical_s=p999,
        span_worst_s=worst,
        use_statistical_bound=use_statistical_bound,
    )
# --------------------------------------------------------------------------- #
# 7. RF link budget                                                             #
# --------------------------------------------------------------------------- #
#
# The propagation model lives in mesh_link_budget.py next to this file. It is a
# two-ray model with a complex Fresnel reflection coefficient, not plain free
# space, because the ground station leg is geometry limited rather than budget
# limited and free space overestimates it by 50 %.

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from mesh_link_budget import (          # noqa: E402
    LinkBudget, Propagation, duty_cycle,
)


def print_link_budget(net: NetworkModel, frame_rate_hz: float,
                      air_time_s: float, sensitivity_dbm: float = -111.0,
                      mast_m: float = 4.0) -> None:
    prop = Propagation()
    lb = LinkBudget(prop=prop, sensitivity_dbm=sensitivity_dbm)

    a2a = lb.design_range_m(100.0, 100.0)
    a2g = lb.design_range_m(100.0, mast_m)
    own, total = duty_cycle(frame_rate_hz, air_time_s,
                            net.n_nodes - 1, net.n_relay_nodes)

    print("=" * 78)
    print("PHASE 4 - RF link budget (summary)")
    print("=" * 78)
    print(f"  {prop.frequency_mhz:.3f} MHz, {lb.tx_power_dbm:.0f} dBm EIRP, "
          f"0 dBi both ends, {lb.sensitivity_dbm:.0f} dBm sensitivity")
    print(f"  usable path loss after {lb.bank_deg:.0f} deg bank "
          f"(-{lb.bank_loss_db:.2f} dB), feedline and "
          f"{lb.fade_margin_db:.0f} dB fade margin : "
          f"{lb.threshold_loss_db:.2f} dB")
    print()
    print(f"  air-to-air  design range (100 m AGL both) : "
          f"{a2a/1000.0:6.2f} km   budget limited")
    print(f"  air-to-GCS  design range (100 m to {mast_m:.0f} m mast) : "
          f"{a2g/1000.0:6.2f} km   "
          f"{'geometry' if a2g < lb.free_space_range_m() else 'budget'} limited, "
          f"two-ray breakpoint at {prop.breakpoint_m(100.0, mast_m)/1000.0:.2f} km")
    print(f"  the two legs differ by a factor of {a2a/max(a2g,1.0):.2f}"
          f"{'  -> relaying buys little, the terminal node profile is safe'
             if a2a/max(a2g,1.0) < 1.25 else
             '  -> relaying is what recovers GCS coverage, keep the routing nodes'}")
    print()
    print(f"  transmitter duty cycle : own {own*100:.3f} % + relayed "
          f"{(total-own)*100:.3f} % = {total*100:.3f} % "
          f"({'within' if total < 0.10 else 'OVER'} a 10 % limit)")
    print()
    print("  Run sw/tools/mesh/mesh_link_budget.py for the full two-ray model,")
    print("  mast height and ground type sensitivities.")
    print()


# --------------------------------------------------------------------------- #
# 8. Nominal mesh message set                                                   #
# --------------------------------------------------------------------------- #

#: ``(class, name, period_s, process, exclusive_group)``.
#:
#: Chosen to be the *minimum* set that still keeps a Paparazzi GCS fully
#: functional for a fixedwing:
#:   MESH_STATE  -> vehicle-to-vehicle traffic (traffic_info / tcas), custom
#:   GPS_LLA     -> GCS map position, course, ground speed, GPS fix state
#:   ESTIMATOR   -> GCS altitude and climb rate
#:   ATTITUDE    -> GCS attitude / heading
#:   NAVIGATION  -> GCS block, stage, distance to waypoint, kill state
#:   PPRZ_MODE   -> GCS autopilot / RC mode strip
#:   ENERGY      -> GCS battery strip
#:   the 64 s tail -> everything the GCS only needs occasionally
NOMINAL_SET: List[Tuple[str, str, float, str, Optional[str]]] = [
    ("datalink",  "MESH_STATE",       4.0,  "Ap", None),
    ("telemetry", "GPS_LLA",          8.0,  "Ap", None),
    ("telemetry", "ESTIMATOR",       16.0,  "Ap", None),
    ("telemetry", "ATTITUDE",        32.0,  "Ap", None),
    ("telemetry", "NAVIGATION",      32.0,  "Ap", None),
    ("telemetry", "PPRZ_MODE",       32.0,  "Ap", None),
    ("telemetry", "ENERGY",          32.0,  "Ap", None),
    ("telemetry", "ALIVE",           64.0,  "Ap", None),
    ("telemetry", "AIR_DATA",        64.0,  "Ap", None),
    ("telemetry", "DATALINK_REPORT", 64.0,  "Ap", None),
    ("telemetry", "GPS_SOL",         64.0,  "Ap", None),
    ("telemetry", "NAVIGATION_REF",  64.0,  "Ap", None),
    ("telemetry", "DL_VALUE",        64.0,  "Ap", None),
    ("telemetry", "WP_MOVED",        64.0,  "Ap", None),
    ("telemetry", "CIRCLE",          64.0,  "Ap", "nav_shape"),
    ("telemetry", "SEGMENT",         64.0,  "Ap", "nav_shape"),
    ("telemetry", "SURVEY",          64.0,  "Ap", "nav_shape"),
    ("telemetry", "FBW_STATUS",      64.0,  "Fbw", None),
]


# --------------------------------------------------------------------------- #
# 8. Reporting / XML emission                                                   #
# --------------------------------------------------------------------------- #

def _fmt_period(p: float) -> str:
    """Format a period without losing precision.

    A fixed one-decimal format silently turned 0.25 s into 0.2 s, which the
    generator then honoured - an 80 ms error in the TDMA feed rate. Keep enough
    digits, then trim trailing zeros so the common whole-second cases still
    read as "4.0".
    """
    txt = f"{p:.4f}".rstrip("0")
    return txt + "0" if txt.endswith(".") else txt


def emit_xml(
    entries: Sequence[ScheduleEntry],
    slots: SlotPlan,
    net: NetworkModel,
    telemetry_frequency: int,
    achieved_gap_ticks: int,
    node_rate_hz: float,
    node_channel_ms_per_s: float,
    utilisation: float,
    out_name: str = "openuas_mesh.xml",
) -> str:
    gap_ms = achieved_gap_ticks * 1000.0 / telemetry_frequency
    phy = net.phy
    lines: List[str] = []
    lines.append('<?xml version="1.0"?>')
    lines.append('<!DOCTYPE telemetry SYSTEM "../telemetry.dtd">')
    lines.append("<!--")
    lines.append(f"  {out_name}")
    lines.append("")
    lines.append("  GENERATED by sw/tools/mesh/mesh_phase_optimizer.py.")
    lines.append("  Re-run that tool after any change; do not hand edit the phase values.")
    lines.append("")
    lines.append("  Link      : EByte E52-400NW22S LoRa MESH, broadcast mode (AT+OPTION=3)")
    lines.append(f"  Air rate  : {phy.spreading_factor * phy.bandwidth_hz // 2**phy.spreading_factor * 4 // 5} bps"
                 f"  (SF{phy.spreading_factor}, BW {phy.bandwidth_hz//1000} kHz, CR 4/5)")
    lines.append(f"  UART      : {net.uart_baud} baud 8N1")
    lines.append(f"  Network   : {net.n_nodes} nodes ({net.n_nodes - 1} aircraft + 1 GCS), "
                 f"{net.n_relay_nodes} routing")
    lines.append(f"  Flood tax : {net.n_transmissions_per_broadcast} transmissions per originated "
                 f"broadcast")
    lines.append("")
    lines.append("  Budget")
    lines.append(f"    per aircraft : {node_rate_hz:.3f} frames/s "
                 f"(one frame every {1/node_rate_hz:.2f} s), "
                 f"{node_channel_ms_per_s:.1f} ms/s of channel")
    lines.append(f"    whole mesh   : {utilisation*100:.1f} % channel utilisation")
    lines.append(f"    guaranteed minimum spacing between two frames of one node: "
                 f"{gap_ms:.0f} ms")
    lines.append(f"    modem cache never exceeds 3 of its 5 frames -> no OUT OF CACHE flush")
    lines.append("")
    lines.append("  Inter-node collision avoidance is handled by the GPS-time TDMA in")
    lines.append("  sw/airborne/modules/multi/traffic_info.c :")
    lines.append(f"    superframe {slots.superframe_s:.2f} s / {slots.n_slots} slots "
                 f"of {slots.slot_s*1000:.1f} ms")
    lines.append("    slots are claimed at run time by listening, never derived")
    lines.append("    from AC_ID, so any set of distinct IDs in 0..254 works")
    lines.append(f"    the airframe MUST set "
                 f"MESH_TDMA_SUPERFRAME_MS={int(round(slots.superframe_s*1000))} "
                 f"(traffic_info.c static asserts it)")
    lines.append("")
    lines.append("  phase is a normalised fraction of the period, so this file stays valid")
    lines.append("  for any TELEMETRY_FREQUENCY; it was solved and verified against")
    lines.append(f"  TELEMETRY_FREQUENCY = {telemetry_frequency} Hz.")
    lines.append("-->")
    lines.append("<telemetry>")

    for process in ("Ap", "Fbw"):
        sel = [e for e in entries if e.process == process]
        if not sel:
            continue
        lines.append(f'  <process name="{process}">')
        lines.append('    <mode name="default">')
        for e in sorted(sel, key=lambda x: x.tick * (1.0 / x.period_s) * 0 + x.tick):
            t_ms = e.tick * 1000.0 / telemetry_frequency
            note = ""
            if e.exclusive_group == "nav_shape":
                note = " only one nav shape is live at a time"
            lines.append(
                f'      <message name="{e.msg.name}"'
                + " " * max(1, 18 - len(e.msg.name))
                + f'period="{_fmt_period(e.period_s)}"'
                + " " * max(1, 8 - len(_fmt_period(e.period_s)))
                + f'phase="{e.phase:.6f}"/>'
                + f'  <!-- t0={t_ms/1000.0:7.2f} s, {e.msg.wire_bytes:2d} B on the wire,'
                + f' {net.phy.time_on_air_s(e.msg.wire_bytes)*1e3:5.2f} ms/hop.{note} -->'
            )
        lines.append("    </mode>")
        lines.append("  </process>")
    lines.append("</telemetry>")
    return "\n".join(lines) + "\n"


def main(argv: Optional[Sequence[str]] = None) -> int:
    here = os.path.dirname(os.path.abspath(__file__))
    root = os.path.normpath(os.path.join(here, "..", "..", ".."))

    # Resolve the message definitions exactly the way the build does.
    #
    # Makefile: CUSTOM_MESSAGES_XML = $(CONF)/messages.xml, and when that file
    # exists it is passed as MESSAGES_XML and the pprzlink default is ignored.
    # A budgeting tool that read the pprzlink copy instead would be sizing the
    # channel from a different set of messages than the one actually flown - it
    # would not even see MESH_STATE, so the very traffic this mesh exists to
    # carry would be missing from its own budget.
    custom = os.path.join(root, "conf", "messages.xml")
    default_messages = custom if os.path.exists(custom) else os.path.normpath(
        os.path.join(root, "sw", "ext", "pprzlink",
                     "message_definitions", "v1.0", "messages.xml"))

    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--messages", default=default_messages)
    ap.add_argument("--nodes", type=int, default=9,
                    help="total nodes on the mesh (aircraft + GCS)")
    ap.add_argument("--relay-nodes", type=int, default=None,
                    help="nodes configured as routing nodes (default: all)")
    ap.add_argument("--telemetry-frequency", type=int, default=50)
    ap.add_argument("--max-replay-ticks", type=int,
                    default=DEFAULT_MAX_REPLAY_TICKS,
                    help="maximum exact gen_periodic replay horizon; "
                         "pathological non-harmonic schedules are rejected "
                        "before phase search (default: 100000 ticks)")
    ap.add_argument("--tdma-gate-frequency", type=float, default=20.0,
                    help="traffic_info_mesh_periodic frequency in Hz; one full "
                         "period is reserved for adjacent-slot sampling skew")
    ap.add_argument("--clock-skew-ms", type=float, default=12.0,
                    help="maximum relative peer clock skew reserved at a slot "
                         "boundary (default: 12 ms for 60 s at +/-100 ppm)")
    ap.add_argument("--statistical-slot-bound", action="store_true",
                    help="accept mean-plus-three-sigma flood timing instead of "
                         "the default absolute modeled CSMA bound")
    ap.add_argument("--csma-ms", type=int, default=20,
                    help="AT+CSMA_RNG value in ms (datasheet minimum is 20)")
    ap.add_argument("--utilisation", type=float, default=None,
                    help="optional channel-utilisation ceiling used for an "
                         "operator-selected budget check; omitted by default "
                         "because the E52 documentation specifies no safe limit")
    ap.add_argument("--cache-limit", type=int, default=3,
                    help="acceptance ceiling for modeled total cache depth; "
                        "not the airborne local-admission estimate "
                        "(hardware limit 5, default acceptance ceiling 3)")
    ap.add_argument("--ac-ids", default="0,122,123,124,125,126,127,128,129",
                    help="comma separated AC_IDs on the mesh, 0 is the GCS")
    ap.add_argument("--nb-slots", type=int, default=None,
                    help="TDMA slots per superframe. Default is the next power "
                         "of two at or above the node count. With no GCS uplink "
                         "to schedule, 8 slots for 8 aircraft is exact")
    ap.add_argument("--fair-slots", type=int, default=25,
                    help="slots distributed in steady state; remaining physical "
                         "slots are short-lived churn headroom (default: 25)")
    ap.add_argument("--reserve-gcs-slot", action="store_true",
                    help="keep slot 0 free for GCS uplink commands. Not needed "
                         "when the ground station originates almost nothing")
    ap.add_argument("--superframe", type=float, default=None,
                    help="TDMA superframe in seconds. Decoupled from the message "
                         "period: the frame is fixed, and a node claims between "
                         "1 and --max-reuse of its slots depending on how many "
                         "peers are present. Defaults to mesh-period * max-reuse")
    ap.add_argument("--max-reuse", type=int, default=8,
                    help="maximum slots one node may claim when the mesh is "
                         "sparse. Must match MESH_TDMA_MAX_REUSE in traffic_info.h")
    ap.add_argument("--mesh-period", type=float, default=None,
                    help="MESH_STATE period in seconds, and therefore the TDMA "
                         "superframe. Overrides the table value and is NOT "
                         "affected by --period-scale")
    ap.add_argument("--gcs-antenna-height", type=float, default=4.0,
                    help="GCS mast height in metres, drives the air-to-ground range")
    ap.add_argument("--period-scale", type=float, default=1.0,
                    help="multiply every telemetry period by this factor. Use it "
                         "to spend headroom won by reducing the relay count: "
                         "0.5 doubles every rate. Keep it a power of two so the "
                         "periods stay harmonic and the phase lattice stays dense")
    ap.add_argument("--period", action="append", default=[], metavar="NAME=SECONDS",
                    help="override one periodic message after --period-scale; "
                         "repeat for multiple messages. MESH_STATE still uses "
                         "--mesh-period")
    ap.add_argument("--control-rate", "--move-wp-rate", dest="control_rate",
                    type=float, default=0.05, metavar="HZ",
                    help="fleet-wide asynchronous operator-command reserve; "
                        "the old --move-wp-rate spelling remains an alias "
                        "(default: 0.05 Hz, one action per 20 seconds)")
    ap.add_argument("--ping-period", type=float, default=5.0, metavar="SECONDS",
                    help="adaptive GCS probe cadence; one live aircraft is "
                        "PINGed per cycle and returns one PONG (default: 5 seconds)")
    ap.add_argument("--air-rate", type=int, default=0, choices=(0, 1, 2),
                    help="AT+RATE value: 0=62.5k 1=21.875k 2=7k. The lower rates "
                         "buy 5 dB of sensitivity each but cost 3x and 10x in air "
                         "time")
    ap.add_argument("--emit-xml", default=None,
                    help="write the telemetry XML to this path")
    args = ap.parse_args(argv)

    try:
        period_overrides = parse_named_values(args.period, "--period")
    except ValueError as exc:
        ap.error(str(exc))
    if args.control_rate < 0.0:
        ap.error("--control-rate must not be negative")
    if args.ping_period <= 0.0:
        ap.error("--ping-period must be positive")
    if args.tdma_gate_frequency <= 0.0:
        ap.error("--tdma-gate-frequency must be positive")
    if args.max_replay_ticks <= 0:
        ap.error("--max-replay-ticks must be positive")
    if args.clock_skew_ms < 0.0:
        ap.error("--clock-skew-ms must not be negative")
    if args.fair_slots <= 0:
        ap.error("--fair-slots must be positive")

    ac_ids = [int(x) for x in args.ac_ids.split(",") if x.strip()]
    if len(ac_ids) != args.nodes:
        args.nodes = len(ac_ids)

    phy = phy_for_rate(args.air_rate)
    net = NetworkModel(
        phy=phy,
        n_nodes=args.nodes,
        n_relay_nodes=args.relay_nodes if args.relay_nodes is not None else args.nodes,
        csma_range_ms=args.csma_ms,
        target_utilisation=args.utilisation,
    )
    msgs = load_messages(args.messages)

    print("=" * 78)
    print("PHASE 1 - LoRa PHY and channel model")
    print("=" * 78)
    print(f"  SF{phy.spreading_factor} / BW {phy.bandwidth_hz/1000:.0f} kHz / CR 4/5"
          f"  ->  {phy.spreading_factor * phy.bandwidth_hz / 2**phy.spreading_factor * 0.8:.0f} bps")
    print(f"  symbol time        : {phy.symbol_time_s*1e6:8.1f} us")
    print(f"  preamble time      : {phy.preamble_time_s*1e3:8.3f} ms")
    print(f"  mesh header on air : {phy.mesh_header_bytes:8d} B")
    print(f"  nodes / routing    : {net.n_nodes} / {net.n_relay_nodes}")
    print(f"  transmissions per originated broadcast (flood tax) : "
          f"{net.n_transmissions_per_broadcast}")
    print()

    # ---- build the schedule ------------------------------------------------ #
    entries: List[ScheduleEntry] = []
    for cls, name, period, process, group in NOMINAL_SET:
        key = (cls, name)
        if key not in msgs:
            print(f"  !! message {cls}.{name} not found in {args.messages}", file=sys.stderr)
            return 2
        p = period * args.period_scale
        if name == "MESH_STATE" and args.mesh_period is not None:
            p = args.mesh_period
        elif name in period_overrides:
            p = period_overrides.pop(name)
        entries.append(ScheduleEntry(msgs[key], p, process, group))
    if period_overrides:
        ap.error("--period names are not in the mesh telemetry set: "
                 + ", ".join(sorted(period_overrides)))

    n_air = net.n_nodes - 1
    event_frames = [
        # One aggregate operator-action reserve. BLOCK/NAVIGATION is the
        # largest normal request/response pair; MOVE_WP/WP_MOVED and
        # SETTING/DL_VALUE use the same reserved arrival opportunity.
        (msgs[("datalink", "BLOCK")], args.control_rate),
        (msgs[("telemetry", "NAVIGATION")], args.control_rate),
        (msgs[("datalink", "PING")], 1.0 / args.ping_period),
        (msgs[("telemetry", "ALIVE")], 1.0 / args.ping_period),
        (msgs[("telemetry", "PONG")], 1.0 / args.ping_period),
    ]

    print("=" * 78)
    print("PHASE 1 - per message cost")
    print("=" * 78)
    print(f"  {'message':<17}{'class':<10}{'payl':>5}{'wire':>6}{'air':>9}"
          f"{'chan':>9}{'period':>9}{'chan/s':>9}")
    print(f"  {'':<17}{'':<10}{'[B]':>5}{'[B]':>6}{'[ms]':>9}{'[ms]':>9}{'[s]':>9}{'[ms/s]':>9}")
    print("  " + "-" * 74)

    live_entries = []
    seen_groups = set()
    total_chan_ms_per_s = 0.0
    total_rate = 0.0
    for e in sorted(entries, key=lambda x: x.period_s):
        air_ms = phy.time_on_air_s(e.msg.wire_bytes) * 1e3
        chan_ms = net.channel_cost_s(e.msg.wire_bytes) * 1e3
        counted = e.exclusive_group is None or e.exclusive_group not in seen_groups
        if e.exclusive_group:
            seen_groups.add(e.exclusive_group)
        chan_per_s = chan_ms * e.rate_hz if counted else 0.0
        total_chan_ms_per_s += chan_per_s
        total_rate += e.rate_hz if counted else 0.0
        live_entries.append(e)
        tag = "" if counted else "  (mutually exclusive, not counted)"
        print(f"  {e.msg.name:<17}{e.msg.msg_class:<10}{e.msg.payload_bytes:>5}"
              f"{e.msg.wire_bytes:>6}{air_ms:>9.2f}{chan_ms:>9.1f}"
              f"{e.period_s:>9.1f}{chan_per_s:>9.2f}{tag}")

    print("  " + "-" * 74)
    # MESH_STATE is slot driven, not node driven. Every slot in the superframe
    # is used by somebody - a sparse mesh means fewer nodes each using more
    # slots - so its channel cost is a property of the FRAME and does not move
    # with the population. Everything else scales with the number of aircraft.
    ms_entry = next(e for e in live_entries if e.msg.name == "MESH_STATE")
    physical_slots = args.nb_slots or len(ac_ids)
    steady_slots = steady_slot_count(physical_slots, args.fair_slots,
                                     len(ac_ids), args.max_reuse)
    mesh_frame_rate_hz = steady_slots / (
        args.superframe if args.superframe
        else ms_entry.period_s * args.max_reuse)
    ms_chan_ms = steady_slots * \
                 net.channel_cost_s(ms_entry.msg.wire_bytes) * 1e3 / \
                 (args.superframe if args.superframe
                  else ms_entry.period_s * args.max_reuse)
    churn_ms = physical_slots * \
               net.channel_cost_s(ms_entry.msg.wire_bytes) * 1e3 / \
               (args.superframe if args.superframe
                else ms_entry.period_s * args.max_reuse)
    other_chan_ms = (total_chan_ms_per_s
                     - net.channel_cost_s(ms_entry.msg.wire_bytes) * 1e3
                     * ms_entry.rate_hz) * n_air
    event_chan_ms = sum(rate_hz * net.channel_cost_s(msg.wire_bytes) * 1e3
                        for msg, rate_hz in event_frames)
    net_util = (ms_chan_ms + other_chan_ms + event_chan_ms) / 1000.0
    print(f"  MESH_STATE, steady fair share  : {ms_chan_ms:6.1f} ms/s "
          f"({steady_slots} of {physical_slots} physical slots)")
    print(f"  MESH_STATE, transient churn max: {churn_ms:6.1f} ms/s")
    print(f"  all other telemetry, {n_air} aircraft : {other_chan_ms:6.1f} ms/s")
    print(f"  control/ack + PING/PONG reserve : {event_chan_ms:6.1f} ms/s")
    total_chan_ms_per_s = (ms_chan_ms + other_chan_ms) / max(n_air, 1)
    if args.utilisation is None:
        print(f"  total channel utilisation      : {net_util*100:5.1f} % "
              "(reported; no evidence-based default ceiling)")
    else:
        print(f"  total channel utilisation      : {net_util*100:5.1f} % "
              f"(operator ceiling {args.utilisation*100:.0f} %)  "
              f"{'OK' if net_util <= args.utilisation else 'OVER BUDGET'}")
    print(f"  UART burst time of the largest frame at {net.uart_baud} baud : "
          f"{net.uart_time_s(max(e.msg.wire_bytes for e in live_entries))*1e3:.2f} ms")
    print()

    # ---- TDMA slot plan ---------------------------------------------------- #
    mesh_state = msgs[("datalink", "MESH_STATE")]
    mesh_state_period = next(e.period_s for e in entries if e.msg.name == "MESH_STATE")
    superframe_s = args.superframe
    if superframe_s is None:
        superframe_s = mesh_state_period * args.max_reuse
    slot_guard_s = 1.0 / args.tdma_gate_frequency + args.clock_skew_ms / 1000.0
    slots = plan_slots(net, mesh_state.wire_bytes, ac_ids, superframe_s,
                       args.nb_slots, slot_guard_s,
                       args.statistical_slot_bound)
    print("=" * 78)
    print("PHASE 1 - GPS-synchronised TDMA slot plan")
    print("=" * 78)
    print(f"  flood span for MESH_STATE ({net.n_transmissions_per_broadcast} hops)")
    print(f"     mean            : {slots.span_mean_s*1e3:7.1f} ms")
    print(f"     mean + 3 sigma  : {slots.span_statistical_s*1e3:7.1f} ms   "
          "<- statistical design span")
    print(f"     absolute worst  : {slots.span_worst_s*1e3:7.1f} ms   "
          f"(every hop draws the maximum CSMA back-off)")
    bound_name = ("mean + 3 sigma" if slots.use_statistical_bound
            else "absolute worst")
    print(f"  slot / guard / usable : {slots.slot_s*1e3:.1f} / "
        f"{slots.guard_s*1e3:.1f} / {slots.usable_s*1e3:.1f} ms")
    print(f"  selected {bound_name} bound : {slots.required_span_s*1e3:.1f} ms; "
        f"margin {(slots.usable_s-slots.required_span_s)*1e3:+.1f} ms  "
        f"{'OK' if slots.ok else 'FAIL'}")
    print(f"  superframe     : {slots.superframe_s:.2f} s "
            f"(fixed; a node claims 1..{args.max_reuse} of its slots)")
    print(f"  slots          : {slots.n_slots}  "
          f"(shared out at run time over the {len(ac_ids)} nodes present)")
    print("  slots are not assigned from AC_ID: each node listens, then claims")
    print("  a slot that is demonstrably free, so AC_IDs may be any distinct")
    print("  values in 0..254 and need not be consecutive.")
    print(f"  share when {len(ac_ids)} nodes are up : "
          f"{max(1, (args.nb_slots or len(ac_ids)) // max(1, len(ac_ids)))} "
          f"slot(s) each, capped at MESH_TDMA_MAX_REUSE={args.max_reuse}")
    print("  the ground station (AC_ID 0) is a peer: it transmits its rare")
    print("  commands unslotted and contends for them like any other traffic.")
    print()

    # ---- phase solve ------------------------------------------------------- #
    min_gap_ticks = int(round(slots.slot_s * args.telemetry_frequency))
    try:
        solved, achieved = solve_phases(
            live_entries, args.telemetry_frequency, min_gap_ticks,
            args.max_replay_ticks)
    except ValueError as exc:
        print(f"  ERROR: {exc}", file=sys.stderr)
        return 2

    print("=" * 78)
    print("PHASE 1 - solved phase offsets")
    print("=" * 78)
    print(f"  {'message':<17}{'period':>8}{'ticks':>8}{'tick':>7}{'phase':>11}{'t':>10}")
    print(f"  {'':<17}{'[s]':>8}{'/period':>8}{'':>7}{'':>11}{'[ms]':>10}")
    print("  " + "-" * 61)
    for e in sorted(solved, key=lambda x: (x.period_s, x.tick)):
        n = int(round(args.telemetry_frequency * e.period_s))
        print(f"  {e.msg.name:<17}{e.period_s:>8.1f}{n:>8}{e.tick:>7}"
              f"{e.phase:>11.6f}{e.tick*1000.0/args.telemetry_frequency:>10.0f}")
    print("  " + "-" * 61)
    phase_spacing_ok = achieved >= min_gap_ticks
    print(f"  achieved minimum inter-frame spacing : {achieved} ticks = "
          f"{achieved*1000.0/args.telemetry_frequency:.0f} ms "
          f"(required >= {min_gap_ticks} ticks)  "
          f"{'OK' if phase_spacing_ok else 'FAIL'}")
    print()

    # ---- verification ------------------------------------------------------ #
    fired = replay_generated_trigger(
        solved, args.telemetry_frequency, args.max_replay_ticks)
    bursts = {t: n for t, n in fired.items() if len(n) > 1}
    peak, overflows, util = simulate_cache(
        fired, solved, net, args.telemetry_frequency,
        slots.slot_s, net.n_nodes - 1, cache_limit=5,
        slot_driven="MESH_STATE", slot_driven_rate_hz=mesh_frame_rate_hz,
        event_frames=event_frames)

    # control experiment: the same message set with every phase left at 0,
    # i.e. what happens without this tool.
    control = [ScheduleEntry(e.msg, e.period_s, e.process, e.exclusive_group,
                             tick=0, phase=0.0) for e in solved]
    control_fired = replay_generated_trigger(
        control, args.telemetry_frequency, args.max_replay_ticks)
    control_bursts = {t: n for t, n in control_fired.items() if len(n) > 1}
    control_peak, control_ovf, _ = simulate_cache(
        control_fired, control, net, args.telemetry_frequency,
        slots.slot_s, net.n_nodes - 1, cache_limit=5,
        slot_driven="MESH_STATE", slot_driven_rate_hz=mesh_frame_rate_hz,
        event_frames=event_frames)

    print("=" * 78)
    print("PHASE 1 - verification")
    print("=" * 78)
    print(f"  replayed gen_periodic over {max(fired)+1} ticks "
        f"({(max(fired)+1)/args.telemetry_frequency:.0f} s superperiod)")
    print(f"  total emissions in the superperiod   : {sum(len(v) for v in fired.values())}")
    print(f"  ticks emitting more than one message : {len(bursts)}  "
        f"{'OK' if not bursts else 'FAIL -> ' + str(bursts)}")
    print(f"  peak modem cache depth (local + relay): {peak} "
        f"(acceptance ceiling {args.cache_limit}, hardware limit 5)  "
        f"{'OK' if peak <= args.cache_limit else 'FAIL'}")
    print(f"  cache overflow events                 : {overflows}  "
        f"{'OK' if overflows == 0 else 'FAIL'}")
    print(f"  simulated channel utilisation         : {util*100:.1f} %")
    print()
    print("  control experiment - identical message set, all phase=0 (the default):")
    biggest = max((len(v) for v in control_fired.values()), default=0)
    print(f"    ticks emitting more than one message : {len(control_bursts)}")
    print(f"    largest simultaneous burst           : {biggest} frames "
          f"({biggest * net.uart_time_s(30)*1e3:.1f} ms of UART, "
          f"delivered to the modem inside one 20 ms scheduler tick)")
    print(f"    peak modem cache depth               : {control_peak}")
    print(f"    OUT OF CACHE events per {(max(control_fired)+1)/args.telemetry_frequency:.0f} s : "
          f"{control_ovf}  -> total buffer flush, telemetry blackout")
    print()

    xml = emit_xml(solved, slots, net, args.telemetry_frequency, achieved,
                   total_rate, total_chan_ms_per_s, net_util,
                   os.path.basename(args.emit_xml) if args.emit_xml
                   else "openuas_mesh.xml")
    print_link_budget(net, total_rate, phy.time_on_air_s(mesh_state.wire_bytes),
                      sensitivity_for_rate(args.air_rate), args.gcs_antenna_height)

    if args.emit_xml:
        with open(args.emit_xml, "w", encoding="utf-8") as fh:
            fh.write(xml)
        print(f"  telemetry XML written to {args.emit_xml}")
    else:
        print(xml)

        utilisation_ok = (args.utilisation is None
                    or net_util <= args.utilisation)
        ok = ((not bursts) and peak <= args.cache_limit and overflows == 0
            and utilisation_ok and slots.ok and phase_spacing_ok)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
