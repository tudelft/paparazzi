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
"""RF range model for the 433 MHz airborne LoRa MESH.

Free-space path loss on its own is *not* a safe answer for this link.  Both ends
are close to a reflecting surface, so the correct model is the two-ray
(direct + ground-reflected) field, with a complex Fresnel reflection
coefficient.  That matters enormously for the ground station leg: with a 2 m GCS
whip the two-ray breakpoint sits near 1.2 km, and beyond it the path loss grows
at 40 dB/decade instead of 20 dB/decade.  Using free space there overestimates
the usable range by a factor of three.

The air-to-air legs are the opposite case: with both aircraft at 50-120 m AGL
the breakpoint is tens of kilometres away, so free space really is valid and the
link is budget limited rather than geometry limited.

What this tool computes
-----------------------
* two-ray received power with the exact Fresnel reflection coefficient for
  vertical or horizontal polarisation over a chosen ground type;
* surface roughness de-correlation of the specular reflection (Ament);
* the bank-angle polarisation mismatch derived from the geometry rather than
  assumed, plus the half-wave dipole element pattern;
* reliable range by bisection on the received power, with the lobing structure
  reported separately so that nulls are not mistaken for range;
* first Fresnel zone clearance and 4/3-earth radio horizon;
* transmitter duty cycle, including the relay traffic, for the regulatory check.

Run standalone::

    ./mesh_link_budget.py
    ./mesh_link_budget.py --ground wet --gcs-antenna-height 5
"""

from __future__ import annotations

import argparse
import cmath
import math
import sys
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

C_LIGHT = 299_792_458.0

#: Electrical constants of common surfaces at VHF/UHF.
#: ``(relative permittivity, conductivity S/m)`` - ITU-R P.527.
GROUND_TYPES: Dict[str, Tuple[float, float]] = {
    "dry":     (4.0, 0.001),    # dry sandy soil, desert
    "average": (15.0, 0.005),   # medium dry ground, farmland - the default
    "wet":     (25.0, 0.020),   # wet ground after rain
    "fresh":   (81.0, 0.010),   # fresh water
    "sea":     (81.0, 5.000),   # sea water
}

#: RMS surface height in metres, used for the roughness factor.
SURFACE_ROUGHNESS_M: Dict[str, float] = {
    "runway": 0.02,
    "grass": 0.15,
    "crops": 0.40,
    "scrub": 1.00,
}


# --------------------------------------------------------------------------- #
# Propagation                                                                   #
# --------------------------------------------------------------------------- #

@dataclass(frozen=True)
class Propagation:
    frequency_mhz: float = 434.125
    ground: str = "average"
    surface: str = "grass"
    vertical_polarisation: bool = True

    @property
    def wavelength_m(self) -> float:
        return C_LIGHT / (self.frequency_mhz * 1e6)

    @property
    def fspl_constant_db(self) -> float:
        """``FSPL(dB) = constant + 20 log10(d_km)``."""
        return 32.44 + 20.0 * math.log10(self.frequency_mhz)

    def fspl_db(self, d_m: float) -> float:
        return self.fspl_constant_db + 20.0 * math.log10(max(d_m, 1.0) / 1000.0)

    def reflection_coefficient(self, grazing_rad: float) -> complex:
        """Complex Fresnel reflection coefficient at a grazing angle.

        ``eps_c = eps_r - j * 60 * lambda * sigma`` is the complex permittivity
        of the ground at this wavelength.
        """
        eps_r, sigma = GROUND_TYPES[self.ground]
        eps_c = complex(eps_r, -60.0 * self.wavelength_m * sigma)

        sin_t = math.sin(grazing_rad)
        cos_t = math.cos(grazing_rad)
        root = cmath.sqrt(eps_c - cos_t * cos_t)

        if self.vertical_polarisation:
            gamma = (eps_c * sin_t - root) / (eps_c * sin_t + root)
        else:
            gamma = (sin_t - root) / (sin_t + root)

        # Ament roughness factor: a rough surface scatters part of the energy
        # out of the specular direction, which *reduces* the depth of the
        # two-ray nulls.
        sigma_h = SURFACE_ROUGHNESS_M[self.surface]
        g = 2.0 * math.pi * sigma_h * sin_t / self.wavelength_m
        gamma *= math.exp(-2.0 * g * g)
        return gamma

    def two_ray_loss_db(self, d_m: float, h1_m: float, h2_m: float) -> float:
        """Path loss of the coherent direct + ground reflected pair."""
        d_los = math.hypot(d_m, h1_m - h2_m)
        d_ref = math.hypot(d_m, h1_m + h2_m)
        grazing = math.atan2(h1_m + h2_m, d_m)

        gamma = self.reflection_coefficient(grazing)
        phase = 2.0 * math.pi * (d_ref - d_los) / self.wavelength_m

        field = 1.0 + gamma * (d_los / d_ref) * cmath.exp(-1j * phase)
        gain_db = 20.0 * math.log10(max(abs(field), 1e-9))
        return self.fspl_db(d_los) - gain_db

    def breakpoint_m(self, h1_m: float, h2_m: float) -> float:
        """Last two-ray maximum. Beyond it the loss exponent rises to 4."""
        return 4.0 * h1_m * h2_m / self.wavelength_m

    def lobe_period_m(self, d_m: float, h1_m: float, h2_m: float) -> float:
        """Range spacing between consecutive two-ray maxima at ``d_m``.

        For ``d >> h1 + h2`` the excess path length is ``2 h1 h2 / d``, so the
        interference phase is ``phi = 4 pi h1 h2 / (lambda d)`` and consecutive
        maxima are separated by

            delta_d = lambda * d^2 / (2 h1 h2).

        This matters. At 12 km with two aircraft at 100 m the lobes are about
        5 km apart, so an aircraft does *not* fly through the structure quickly
        and the pattern has to be averaged over a full lobe rather than over an
        arbitrarily chosen window.
        """
        return self.wavelength_m * d_m * d_m / (2.0 * h1_m * h2_m)

    def local_mean_loss_db(self, d_m: float, h1_m: float, h2_m: float,
                           n: int = 128) -> float:
        """Two-ray path loss averaged in *power* over one interference lobe.

        This is the quantity to design with. The instantaneous two-ray field
        swings between a +6 dB constructive peak and an arbitrarily deep null;
        neither of those is a range. The local mean is what a receiver sees
        once the geometry has moved a little, and it converges to free space
        below the breakpoint and to a 40 dB/decade slope above it.

        The averaging window is capped at 10 % of the range. Beyond the
        breakpoint the lobe period grows as d^2 and quickly exceeds the range
        itself; averaging over +/- d/2 there would reach back into the much
        stronger sub-breakpoint region and report a loss several dB too
        optimistic. Past the breakpoint the loss is monotonic anyway, so a
        narrow window is both correct and sufficient.
        """
        span = min(self.lobe_period_m(d_m, h1_m, h2_m), 0.1 * d_m)
        acc = 0.0
        for k in range(n):
            d = d_m - 0.5 * span + span * k / (n - 1)
            acc += 10.0 ** (-self.two_ray_loss_db(max(d, 10.0), h1_m, h2_m) / 10.0)
        return -10.0 * math.log10(acc / n)

    def fresnel_radius_m(self, d_m: float, fraction: float = 0.5) -> float:
        """First Fresnel zone radius at ``fraction`` along the path."""
        d1 = d_m * fraction
        d2 = d_m - d1
        return math.sqrt(self.wavelength_m * d1 * d2 / d_m)

    @staticmethod
    def radio_horizon_km(h1_m: float, h2_m: float) -> float:
        """4/3 effective earth radius radio horizon."""
        return 4.12 * (math.sqrt(h1_m) + math.sqrt(h2_m))


# --------------------------------------------------------------------------- #
# Antenna and attitude                                                          #
# --------------------------------------------------------------------------- #

@dataclass(frozen=True)
class Antenna:
    """Half-wave dipole, nominally vertical when the aircraft is wings level."""

    peak_gain_dbi: float = 0.0

    @staticmethod
    def dipole_pattern_db(theta_rad: float) -> float:
        """Half-wave dipole element pattern relative to its own peak.

        ``theta`` is measured from the antenna axis, so ``theta = pi/2`` is the
        broadside peak and ``theta = 0`` is the end-fire null.
        """
        s = math.sin(theta_rad)
        if s < 1e-6:
            return -40.0                      # clamp the mathematical null
        f = math.cos(math.pi / 2.0 * math.cos(theta_rad)) / s
        return 20.0 * math.log10(max(f, 1e-4))

    @staticmethod
    def polarisation_loss_db(bank_rad: float) -> float:
        """Mismatch between a banked airborne dipole and an upright peer.

        Two linearly polarised antennas separated by an angle ``a`` couple as
        ``cos^2(a)``.  A 45 deg bank therefore costs
        ``-10 log10(cos^2 45) = 3.01 dB`` - which is where the 3 dB figure in
        the specification comes from, derived rather than assumed.
        """
        c = abs(math.cos(bank_rad))
        if c < 1e-3:
            return 60.0                       # fully cross polarised
        return -20.0 * math.log10(c)


# --------------------------------------------------------------------------- #
# Link budget                                                                   #
# --------------------------------------------------------------------------- #

@dataclass
class LinkBudget:
    prop: Propagation
    # 10 mW = +10 dBm EIRP. This is the regulatory ceiling for the 433 MHz
    # ISM band and it is a HARD constraint of the assignment, not a default to
    # be tuned. The E52-400NW22S PA is rated 22 dBm, so the module is run well
    # below its capability on purpose; the headroom buys nothing legal and the
    # design must close at 10 dBm or not at all.
    #
    # EIRP, not conducted power: any antenna gain counts against this figure.
    # See sw/tools/mesh/e52_provision.py, which derives the AT+POWER conducted
    # setting from this limit minus the antenna gain.
    tx_power_dbm: float = 10.0
    tx_gain_dbi: float = 0.0
    rx_gain_dbi: float = 0.0
    sensitivity_dbm: float = -111.0         # E52 datasheet at 62.5 kbps
    implementation_loss_db: float = 1.0     # connectors, feedline, detuning
    fade_margin_db: float = 10.0            # residual multipath and shadowing
    bank_deg: float = 45.0
    gcs_mast_m: float = 4.0                 # available ground station mast

    @property
    def bank_loss_db(self) -> float:
        return Antenna.polarisation_loss_db(math.radians(self.bank_deg))

    @property
    def threshold_loss_db(self) -> float:
        """Maximum path loss the link can stand, all fixed terms charged."""
        return (self.tx_power_dbm + self.tx_gain_dbi + self.rx_gain_dbi
                - self.sensitivity_dbm
                - self.bank_loss_db
                - self.implementation_loss_db
                - self.fade_margin_db)

    # -- ranges ------------------------------------------------------------- #

    def free_space_range_m(self) -> float:
        return 1000.0 * 10.0 ** ((self.threshold_loss_db - self.prop.fspl_constant_db) / 20.0)

    def two_ray_range_m(self, h1_m: float, h2_m: float,
                        d_max_m: float = 300_000.0) -> float:
        """Range at which the lobe-averaged two-ray loss reaches the threshold.

        Bisection on ::Propagation.local_mean_loss_db, which is monotonic in
        range once the interference structure has been averaged out.
        """
        def loss(d: float) -> float:
            return self.prop.local_mean_loss_db(d, h1_m, h2_m)

        lo, hi = 10.0, d_max_m
        if loss(lo) > self.threshold_loss_db:
            return 0.0
        if loss(hi) <= self.threshold_loss_db:
            return hi
        for _ in range(100):
            mid = 0.5 * (lo + hi)
            if loss(mid) <= self.threshold_loss_db:
                lo = mid
            else:
                hi = mid
        return 0.5 * (lo + hi)

    def lobe_excursion_db(self, d_m: float, h1_m: float, h2_m: float,
                          n: int = 400) -> Tuple[float, float]:
        """Peak and trough of the interference pattern about the local mean."""
        span = min(self.prop.lobe_period_m(d_m, h1_m, h2_m), d_m)
        mean = self.prop.local_mean_loss_db(d_m, h1_m, h2_m)
        losses = [self.prop.two_ray_loss_db(max(d_m - 0.5 * span + span * k / (n - 1), 10.0),
                                            h1_m, h2_m)
                  for k in range(n)]
        return mean - min(losses), mean - max(losses)

    def design_range_m(self, h1_m: float, h2_m: float) -> float:
        """The range to plan the mission with.

        ``min(free space, lobe averaged two ray)``.

        Capping at free space is deliberate and conservative.  Averaging the
        two-ray field over a lobe yields up to +3 dB over free space, because
        the ground reflection does add power in the mean.  But that is only
        meaningful if the geometry actually sweeps through a lobe on a
        timescale short compared with the mission.  At these altitudes the lobe
        period is of the same order as the range itself (see
        ::Propagation.lobe_period_m), so an aircraft can sit near one null for
        many minutes.  Claiming more than free space on the strength of a
        reflection you cannot control would not be a range, it would be luck.

        Below the breakpoint the cap therefore binds and the link is budget
        limited.  Above the breakpoint the two-ray term binds and the link is
        geometry limited - which is exactly the situation for the ground
        station leg with a low mast.
        """
        return min(self.free_space_range_m(), self.two_ray_range_m(h1_m, h2_m))

    def margin_at_m(self, d_m: float, h1_m: float, h2_m: float) -> float:
        """Spare link margin at a known separation, in dB.

        Positive means the link closes with that much *on top of* the fade
        margin already charged in ::threshold_loss_db.  This is the number that
        matters when the operating area is bounded and known: a range figure
        answers "how far could we go", this answers "how much margin do we
        actually have where we will actually fly".

        Takes the **worse** of free space and the lobe-averaged two-ray loss.
        Beyond the two-ray breakpoint the reflection is destructive on average
        and two-ray is the larger loss; below it, free space is the honest
        bound because the constructive gain is not something to bank on. Taking
        the smaller loss here would flatter the low-altitude, long-range
        geometries - which are exactly the ones that decide whether relaying is
        needed.
        """
        loss = max(self.prop.local_mean_loss_db(d_m, h1_m, h2_m),
                   self.prop.fspl_db(d_m))
        return self.threshold_loss_db - loss


# --------------------------------------------------------------------------- #
# Bounded operating area                                                        #
# --------------------------------------------------------------------------- #

def print_operating_area(lb: LinkBudget, width_m: float, height_m: float,
                         gcs_corner: bool,
                         alt_lo_m: float, alt_hi_m: float) -> bool:
    """Analyse a bounded rectangular operating area.

    When the area is known and bounded, the design question stops being "how
    far can we reach" and becomes "how much margin do we hold at the worst
    geometry inside the box".  If that margin is comfortable everywhere, then
    every node hears every other node directly and multi-hop relaying is dead
    weight: it multiplies the channel occupancy of every packet without adding
    a path that was not already there.

    Returns True when the flat, single-hop topology is supported by the numbers.
    """
    diag = math.hypot(width_m, height_m)
    gcs_worst = diag if gcs_corner else diag / 2.0

    print("=" * 78)
    print("BOUNDED OPERATING AREA")
    print("=" * 78)
    print(f"  area width x height             : {width_m/1000.0:8.3f} x "
          f"{height_m/1000.0:.3f} km")
    print(f"  diagonal (worst drone to drone) : {diag/1000.0:8.3f} km")
    print(f"  GCS position                    : "
          f"{'corner' if gcs_corner else 'centre':>8}")
    print(f"  worst drone to GCS              : {gcs_worst/1000.0:8.3f} km")
    print()
    print(f"  {'link':<40}{'dist':>9}{'margin':>10}")
    print("  " + "-" * 60)

    cases = [
        (f"drone-drone, both {alt_hi_m:.0f} m AGL", diag, alt_hi_m, alt_hi_m),
        (f"drone-drone, both {alt_lo_m:.0f} m AGL", diag, alt_lo_m, alt_lo_m),
        (f"drone-drone, {alt_hi_m:.0f} m to {alt_lo_m:.0f} m", diag, alt_hi_m, alt_lo_m),
        (f"drone-GCS, drone {alt_hi_m:.0f} m AGL", gcs_worst, alt_hi_m, lb.gcs_mast_m),
        (f"drone-GCS, drone {alt_lo_m:.0f} m AGL", gcs_worst, alt_lo_m, lb.gcs_mast_m),
    ]

    worst = 1e9
    for label, d, h1, h2 in cases:
        m = lb.margin_at_m(d, h1, h2)
        worst = min(worst, m)
        flag = "OK" if m >= 6.0 else ("thin" if m >= 0.0 else "FAILS")
        print(f"  {label:<40}{d/1000.0:8.2f}km{m:>9.1f}dB   {flag}")
    print("  " + "-" * 60)
    print(f"  worst case margin in the box    : {worst:.1f} dB")
    print(f"  ...on top of the {lb.fade_margin_db:.0f} dB fade margin already charged,")
    print(f"     so the true fade tolerance at the worst corner is "
          f"{worst + lb.fade_margin_db:.1f} dB")
    print()
    print("  what a pessimistic environment does to that spare margin")
    print("  " + "-" * 60)
    drains = [
        ("co-channel users / raised noise floor in the 433 ISM band", 6.0),
        ("60 deg bank instead of 45 (polarisation)", 3.0),
        ("airframe shadowing, antenna behind the fuselage", 4.0),
    ]
    left = worst
    for label, cost in drains:
        left -= cost
        state = "OK" if left >= 3.0 else ("thin" if left >= 0.0 else "LINK LOST")
        print(f"    -{cost:4.1f} dB  {label:<50} {left:6.1f} dB  {state}")
    print("  " + "-" * 60)
    print()

    direct_ok = worst >= 0.0
    robust_flat_ok = left >= 3.0
    if not direct_ok:
        print(f"  >>> The {diag/1000.0:.2f} km corner geometry does NOT close as a")
        print(f"      direct link ({worst:.1f} dB worst margin). Multi-hop placement")
        print( "      is required; all-router capability alone cannot bridge an empty gap.")
    elif robust_flat_ok:
        print(f"  >>> Every modeled corner link closes directly at "
              f"{diag/1000.0:.2f} km.")
        print( "      The margin also survives a pessimistic interference budget, so a")
        print( "      flat single-hop mesh would be defensible.")
    else:
        print(f"  >>> Every modeled corner link closes directly at "
              f"{diag/1000.0:.2f} km in")
        print( "      nominal conditions, but the reserve does NOT survive the")
        print(f"      pessimistic interference budget ({left:.1f} dB remaining).")
        print( "      All-router flooding adds spatial diversity: copies transmitted")
        print( "      from different positions can bypass a null or local interferer.")
    if direct_ok and not robust_flat_ok:
        print( "      Treat direct coverage as nominal, not guaranteed.")
    print()
    return robust_flat_ok


# --------------------------------------------------------------------------- #
# Duty cycle                                                                    #
# --------------------------------------------------------------------------- #

def duty_cycle(frames_per_s_per_node: float, air_time_s: float,
               n_aircraft: int, n_relay_nodes: int) -> Tuple[float, float]:
    """Transmitter duty cycle of one radio.

    Returns ``(own, total)`` as fractions.  A routing node transmits its own
    traffic plus one relay copy of everything its neighbours originate, so the
    relay share dominates by an order of magnitude.
    """
    own = frames_per_s_per_node * air_time_s
    relayed = 0.0
    if n_relay_nodes > 1:
        relayed = n_aircraft * frames_per_s_per_node * air_time_s
    return own, own + relayed


# --------------------------------------------------------------------------- #
# Report                                                                        #
# --------------------------------------------------------------------------- #

def _fmt_km(m: float) -> str:
    return f"{m/1000.0:8.2f} km"


def report(args: argparse.Namespace) -> int:
    prop = Propagation(frequency_mhz=args.frequency,
                       ground=args.ground,
                       surface=args.surface,
                       vertical_polarisation=not args.horizontal)
    lb = LinkBudget(prop=prop,
                    tx_power_dbm=args.tx_power,
                    sensitivity_dbm=args.sensitivity,
                    fade_margin_db=args.fade_margin,
                    bank_deg=args.bank,
                    gcs_mast_m=args.gcs_antenna_height)

    eps_r, sigma = GROUND_TYPES[prop.ground]

    print("=" * 78)
    print("RF LINK BUDGET")
    print("=" * 78)
    print(f"  frequency                 : {prop.frequency_mhz:9.3f} MHz "
          f"(lambda {prop.wavelength_m*100:.1f} cm)")
    print(f"  polarisation              : "
          f"{'vertical' if prop.vertical_polarisation else 'horizontal':>9}")
    print(f"  ground                    : {prop.ground:>9}  "
          f"(eps_r {eps_r}, sigma {sigma} S/m)")
    print(f"  surface                   : {prop.surface:>9}  "
          f"(rms height {SURFACE_ROUGHNESS_M[prop.surface]} m)")
    print()
    print(f"  TX power                  : {lb.tx_power_dbm:9.1f} dBm EIRP")
    print(f"  antenna gain TX / RX      : {lb.tx_gain_dbi:9.1f} / {lb.rx_gain_dbi:.1f} dBi")
    print(f"  RX sensitivity @ 62.5 kbps: {lb.sensitivity_dbm:9.1f} dBm")
    print(f"  raw system gain           : "
          f"{lb.tx_power_dbm - lb.sensitivity_dbm:9.1f} dB")
    print(f"  bank {lb.bank_deg:.0f} deg polarisation  : "
          f"{-lb.bank_loss_db:9.2f} dB   "
          f"(-20 log10 cos {lb.bank_deg:.0f} deg, derived)")
    print(f"  implementation / feedline : {-lb.implementation_loss_db:9.2f} dB")
    print(f"  multipath / shadowing     : {-lb.fade_margin_db:9.2f} dB")
    print(f"  USABLE PATH LOSS          : {lb.threshold_loss_db:9.2f} dB")
    print()

    print("  bank angle sensitivity (polarisation mismatch alone)")
    for b in (0, 15, 30, 45, 60, 75):
        loss = Antenna.polarisation_loss_db(math.radians(b))
        scale = 10.0 ** (-loss / 20.0)
        print(f"    {b:2d} deg : {-loss:6.2f} dB  ->  range x {scale:4.2f}")
    print()

    print("=" * 78)
    print("RANGE BY GEOMETRY")
    print("=" * 78)
    print(f"  {'case':<32}{'breakpt':>10}{'free sp':>10}{'2-ray':>10}"
          f"{'DESIGN':>10}{'horizon':>10}")
    print("  " + "-" * 74)

    cases: List[Tuple[str, float, float]] = [
        ("air-air, both 120 m AGL", 120.0, 120.0),
        ("air-air, both 100 m AGL", 100.0, 100.0),
        ("air-air, both  50 m AGL", 50.0, 50.0),
        ("air-air, 120 m to 50 m", 120.0, 50.0),
        (f"air-ground, 100 m to {args.gcs_antenna_height:.0f} m mast",
         100.0, args.gcs_antenna_height),
        (f"air-ground,  50 m to {args.gcs_antenna_height:.0f} m mast",
         50.0, args.gcs_antenna_height),
    ]

    results: Dict[str, float] = {}
    for label, h1, h2 in cases:
        bp = prop.breakpoint_m(h1, h2)
        r2 = lb.two_ray_range_m(h1, h2)
        rf = lb.free_space_range_m()
        rd = min(rf, r2)
        rh = prop.radio_horizon_km(h1, h2) * 1000.0
        results[label] = rd
        limit = "geometry" if r2 < rf else "budget"
        print(f"  {label:<32}{bp/1000.0:9.2f}{rf/1000.0:10.2f}{r2/1000.0:10.2f}"
              f"{rd/1000.0:10.2f}{rh/1000.0:10.1f}   {limit}")
    print("  " + "-" * 74)
    print("  all distances in km")
    print("  breakpt = last two-ray maximum; beyond it the loss exponent is 4, not 2")
    print("  free sp = free space only, valid ONLY while range < breakpoint")
    print("  2-ray   = lobe-averaged two-ray solution")
    print("  DESIGN  = min(free space, 2-ray). Never claim more than free space on")
    print("            the strength of a ground reflection you do not control.")
    print()

    a2a_key = "air-air, both 100 m AGL"
    a2g_key = f"air-ground, 100 m to {args.gcs_antenna_height:.0f} m mast"
    a2a = results[a2a_key]
    a2g = results[a2g_key]

    print("  interference structure at the design range")
    for label, d, h1, h2 in ((a2a_key, a2a, 100.0, 100.0),
                             (a2g_key, a2g, 100.0, args.gcs_antenna_height)):
        up, down = lb.lobe_excursion_db(d, h1, h2)
        period = prop.lobe_period_m(d, h1, h2)
        dwell = period / 12.0                 # nominal 12 m/s closure
        print(f"    {label:<32} lobe period {period/1000.0:7.2f} km, "
              f"+{up:.1f} / {down:.1f} dB about the mean")
        print(f"    {'':<32} one lobe takes {dwell/60.0:5.1f} min at 12 m/s closure")
    print("    The lobe period is comparable with the range itself, so an aircraft")
    print("    can sit near a null for minutes. That is why the design range is")
    print("    capped at free space and carries a fade margin, rather than being")
    print("    taken from the two-ray mean.")
    print()
    print(f"  1st Fresnel radius at the air-air range : {prop.fresnel_radius_m(a2a):5.1f} m")
    print(f"    aircraft fly at 50-120 m AGL, so the first Fresnel zone is "
          f"{'partly obstructed' if prop.fresnel_radius_m(a2a) > 50.0 else 'clear'}")
    print(f"    by terrain - this is what the {lb.fade_margin_db:.0f} dB fade margin pays for.")
    print()
    print(f"  >>> AIR-TO-AIR  design range : {a2a/1000.0:6.2f} km  (budget limited)")
    print(f"  >>> AIR-TO-GCS  design range : {a2g/1000.0:6.2f} km  (geometry limited)")
    print(f"  >>> The GCS leg is the weak link, by a factor of "
          f"{a2a/max(a2g, 1.0):.1f}. Multi-hop relaying")
    print(f"      through the swarm is what recovers GCS coverage beyond "
          f"{a2g/1000.0:.1f} km - so the")
    tax = min(1 + args.relay_nodes, args.aircraft + 1)
    print(f"      routing nodes that cost {tax}x in channel time are also "
          f"buying that reach.")
    print()

    print("=" * 78)
    print("GCS MAST HEIGHT SENSITIVITY (aircraft at 100 m AGL)")
    print("=" * 78)
    print(f"  {'mast':>6}{'breakpoint':>13}{'range':>12}")
    print("  " + "-" * 31)
    for h in (1.5, 2.0, 3.0, 5.0, 10.0, 20.0):
        print(f"  {h:5.1f} m{_fmt_km(prop.breakpoint_m(100.0, h)):>13}"
              f"{_fmt_km(lb.design_range_m(100.0, h)):>12}")
    print("  " + "-" * 31)
    print("  Raising the GCS antenna moves the breakpoint out quadratically and is")
    print("  by far the cheapest range improvement available on this link.")
    print()

    print("=" * 78)
    print("GROUND TYPE SENSITIVITY (air to ground, 100 m to "
          f"{args.gcs_antenna_height:.0f} m)")
    print("=" * 78)
    for g in GROUND_TYPES:
        p = Propagation(frequency_mhz=args.frequency, ground=g,
                        surface=prop.surface,
                        vertical_polarisation=prop.vertical_polarisation)
        b = LinkBudget(prop=p, tx_power_dbm=lb.tx_power_dbm,
                       sensitivity_dbm=lb.sensitivity_dbm,
                       fade_margin_db=lb.fade_margin_db, bank_deg=lb.bank_deg)
        print(f"  {g:<9}{_fmt_km(b.design_range_m(100.0, args.gcs_antenna_height))}")
    print()

    width = args.area_width if args.area_width is not None else args.area_side
    height = args.area_height if args.area_height is not None else args.area_side
    print_operating_area(lb, width, height, not args.gcs_centre,
                         args.alt_low, args.alt_high)

    print("=" * 78)
    print("TRANSMITTER DUTY CYCLE (regulatory)")
    print("=" * 78)
    own, total = duty_cycle(args.frame_rate, args.air_time_ms / 1000.0,
                            args.aircraft, args.relay_nodes)
    print(f"  own traffic                  : {own*100:6.3f} %")
    print(f"  plus relayed neighbour frames: {(total-own)*100:6.3f} %")
    print(f"  TOTAL per radio              : {total*100:6.3f} %")
    print(f"  headroom to a 10 % ERC limit : "
          f"{'OK' if total < 0.10 else 'EXCEEDED'} "
          f"(x{0.10/max(total,1e-9):.1f} margin)")
    print()
    print("  The 62.5 kbps mode occupies 500 kHz. Verify that your national")
    print("  sub-band allowance covers a 500 kHz emission at this power and duty")
    print("  cycle before flying; the 433 MHz band is not uniformly permissive.")
    print()

    return 0


def main(argv: Optional[Sequence[str]] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--frequency", type=float, default=434.125,
                    help="carrier in MHz (E52 channel 24)")
    ap.add_argument("--tx-power", type=float, default=10.0,
                    help="dBm EIRP (regulatory limit, NOT the 22 dBm PA rating)")
    ap.add_argument("--sensitivity", type=float, default=-111.0, help="dBm")
    ap.add_argument("--fade-margin", type=float, default=10.0, help="dB")
    ap.add_argument("--bank", type=float, default=45.0, help="worst case bank in deg")
    ap.add_argument("--ground", choices=sorted(GROUND_TYPES), default="average")
    ap.add_argument("--surface", choices=sorted(SURFACE_ROUGHNESS_M), default="grass")
    ap.add_argument("--horizontal", action="store_true",
                    help="horizontally polarised antennas (default is vertical)")
    ap.add_argument("--gcs-antenna-height", type=float, default=4.0,
                    help="GCS mast height in metres")
    ap.add_argument("--area-side", type=float, default=3000.0,
                    help="side of the square operating area in metres")
    ap.add_argument("--area-width", type=float, default=None,
                    help="rectangular area width in metres (overrides --area-side)")
    ap.add_argument("--area-height", type=float, default=None,
                    help="rectangular area height in metres (overrides --area-side)")
    ap.add_argument("--gcs-centre", action="store_true",
                    help="GCS sits in the middle of the area rather than a corner")
    ap.add_argument("--alt-low", type=float, default=50.0,
                    help="lowest operating altitude AGL")
    ap.add_argument("--alt-high", type=float, default=120.0,
                    help="highest operating altitude AGL")
    ap.add_argument("--frame-rate", type=float, default=1.0 / 6.0,
                    help="originated frames per second per node")
    ap.add_argument("--air-time-ms", type=float, default=6.86,
                    help="air time of one frame per hop")
    ap.add_argument("--aircraft", type=int, default=12)
    ap.add_argument("--relay-nodes", type=int, default=13,
                    help="routing radios including the GCS (default 13)")
    return report(ap.parse_args(argv))


if __name__ == "__main__":
    sys.exit(main())
