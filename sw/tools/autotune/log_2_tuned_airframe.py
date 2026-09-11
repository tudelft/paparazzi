#!/usr/bin/env python3
"""
log_2_tuned_airframe.py - Paparazzi UAV airframe auto-tuning from flight logs.

Iteration 1: pitch trim and ruddervator yaw mixing / turn radius.
Iteration 2: roll trim, ETECS (energy control) plant identification, and
             course-loop gains (model based from AUTO1, measured from AUTO2).

Parses a Paparazzi telemetry .log/.data pair, analyzes stabilized flight
segments, and writes a version-incremented, tuned copy of the airframe XML
(<name>_optim_NNN.xml). Optionally verifies the result compiles with the
Paparazzi build system and always emits a human-readable audit report.

Everything except the course-loop *measurement* is derived from AUTO1
(stabilized manual) flight: the controller setpoints in DESIRED, the
commands in COMMANDS and the aircraft response in ATTITUDE / ESTIMATOR /
AIR_DATA are enough to identify trims and the open-loop energy plant. The
course loop is checked against a model when no airborne AUTO2 time exists
and only re-tuned from data when at least 60 s of AUTO2 navigation is found.

Usage:
  log_2_tuned_airframe.py [--log-name 26_09_05__18_42_28] [--ac-id 129]
                          [--log-dir DIR] [--airframe-out DIR] [--base XML]
                          [--turn-radius 30] [--no-build]
"""

from __future__ import annotations

import argparse
import logging
import math
import re
import subprocess
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path

import numpy as np

G = 9.80665
MAX_PPRZ = 9600
TRIM_LIMIT = MAX_PPRZ // 10          # actuators.c clips command_*_trim to +-960
MIN_AUTO2_SECONDS = 60.0             # airborne AUTO2 needed to tune course from data

PPRZ_HOME = Path(__file__).resolve().parents[3] if len(Path(__file__).resolve().parents) > 3 \
    else Path.home() / "paparazzi"

log = logging.getLogger("autotune")
NAN = float("nan")


class AutotuneError(Exception):
    """Fatal autotune failure (bad inputs, malformed XML, failed build)."""


def isnum(x: float) -> bool:
    return x is not None and not math.isnan(x)


# --------------------------------------------------------------------------
# Input parsing
# --------------------------------------------------------------------------

def resolve_existing(path: Path, what: str) -> Path:
    """Resolve symlinks and verify existence; explicit error on broken links."""
    try:
        return path.resolve(strict=True)
    except (OSError, RuntimeError) as exc:
        raise AutotuneError(f"{what} not found or broken symlink: {path} ({exc})") from exc


@dataclass
class FlightLog:
    """Parsed .log/.data pair for one aircraft."""
    log_path: Path
    data_path: Path
    ac_id: int
    aircraft_name: str = ""
    airframe_rel: str = ""            # e.g. airframes/OPENUAS/openuas_zohd_talon_250g.xml
    flown_defines: dict = field(default_factory=dict)  # define name -> value as flown
    msgs: dict = field(default_factory=dict)           # name -> (t[N], vals[N,M])

    def series(self, name: str) -> tuple[np.ndarray, np.ndarray]:
        if name not in self.msgs:
            raise AutotuneError(f"required telemetry message '{name}' absent from {self.data_path.name}")
        return self.msgs[name]

    def flown(self, name: str, default: float) -> float:
        """Numeric value of a define in the airframe that was actually flown."""
        raw = self.flown_defines.get(name)
        if raw is None:
            return default
        try:
            return float(raw.rstrip("f").rstrip("."))
        except ValueError:
            return default


def parse_log_header(log_path: Path, ac_id: int) -> tuple[str, str, dict]:
    """Aircraft name, airframe path and flown defines for ac_id from the .log."""
    header = []
    with log_path.open("r", errors="replace") as f:
        for line in f:
            header.append(line)
            if "</conf>" in line or len(header) > 500000:
                break
    text = "".join(header)
    for m in re.finditer(r"<aircraft\s+[^>]*?>", text, re.S):
        tag = m.group(0)
        m_id = re.search(r'ac_id="(\d+)"', tag)
        if not (m_id and int(m_id.group(1)) == ac_id):
            continue
        name = re.search(r'name="([^"]+)"', tag)
        airframe = re.search(r'airframe="([^"]+)"', tag)
        if not (name and airframe):
            raise AutotuneError(f".log <aircraft> entry for ac_id {ac_id} lacks name/airframe attributes")
        # The embedded airframe copy tells us which values were actually flown.
        # The logger upper-cases and reorders attributes: <define VALUE=".." NAME=".."/>.
        af_start = text.find("<airframe", m.end())
        af_end = text.find("</airframe>", af_start)
        defines = {}
        if af_start > 0 and af_end > af_start:
            for d in re.finditer(r"<define\s+([^>]*?)/?>", text[af_start:af_end]):
                attrs = dict(re.findall(r'(\w+)="([^"]*)"', d.group(1)))
                attrs = {k.lower(): v for k, v in attrs.items()}
                if "name" in attrs and "value" in attrs:
                    defines.setdefault(attrs["name"], attrs["value"])
        return name.group(1), airframe.group(1), defines
    raise AutotuneError(f"no <aircraft ac_id=\"{ac_id}\"> found in {log_path.name}")


def parse_data_file(data_path: Path, ac_id: int, wanted: set[str]) -> dict:
    """Single pass over the .data stream into numpy arrays.

    Format per line: '<time> <ac_id> <MSG_NAME> <field...>'. Array fields
    are comma-separated (e.g. COMMANDS values); commas are separators too.
    Corrupted records are counted and skipped.
    """
    buckets: dict[str, tuple[list, list]] = {m: ([], []) for m in wanted}
    ac = str(ac_id)
    bad = 0
    with data_path.open("r", errors="replace") as f:
        for line in f:
            parts = line.split(" ", 3)
            if len(parts) < 4 or parts[1] != ac:
                continue
            name = parts[2]
            if name not in buckets:
                continue
            try:
                vals = [float(x) for x in parts[3].replace(",", " ").split()]
                t = float(parts[0])
            except ValueError:
                bad += 1
                continue
            if vals:
                buckets[name][0].append(t)
                buckets[name][1].append(vals)
    if bad:
        log.warning("skipped %d corrupted .data records", bad)
    out = {}
    for name, (ts, rows) in buckets.items():
        if not rows:
            continue
        ncol = min(len(r) for r in rows)
        out[name] = (np.asarray(ts), np.asarray([r[:ncol] for r in rows]))
    return out


def load_flight_log(log_dir: Path, log_name: str, ac_id: int) -> FlightLog:
    log_path = resolve_existing(log_dir / f"{log_name}.log", "log file")
    data_path = resolve_existing(log_dir / f"{log_name}.data", "data file")
    name, airframe, defines = parse_log_header(log_path, ac_id)
    wanted = {"ATTITUDE", "GPS", "COMMANDS", "PPRZ_MODE", "AIR_DATA", "AIRSPEED",
              "ESTIMATOR", "DESIRED", "NAVIGATION", "NAVIGATION_REF", "ENERGY", "CIRCLE"}
    msgs = parse_data_file(data_path, ac_id, wanted)
    fl = FlightLog(log_path, data_path, ac_id, name, airframe, defines, msgs)
    log.info("loaded %s: aircraft '%s' (AC_ID %d), airframe %s, %d message types, "
             "%d flown defines", log_name, name, ac_id, airframe, len(msgs), len(defines))
    return fl


# --------------------------------------------------------------------------
# Flight dynamics analysis
# --------------------------------------------------------------------------

@dataclass
class Analysis:
    """Metrics extracted from the flight used to derive XML changes."""
    airspeed_source: str = "AIR_DATA.airspeed"
    airspeed_mean: float = NAN
    airborne_seconds: float = NAN
    n_flights: int = 0
    auto1_seconds: float = NAN
    auto2_seconds: float = NAN
    wind_speed: float = NAN
    wind_from_deg: float = NAN
    wind_ratio: float = NAN               # wind / airspeed
    power_wot_w: float = NAN              # electrical power at full throttle
    volt_sag_wot: float = NAN
    auto2_alt_err_mean: float = NAN       # mean (setpoint - actual) altitude in AUTO2
    auto2_alt_low_frac: float = NAN       # AUTO2 time more than 15 m below setpoint
    auto2_throttle_mean: float = NAN
    pitch_err_rms_auto2: float = NAN
    auto2_level_elevator: float = NAN     # steady elevator in AUTO2 (no RC trim influence)
    n_auto2_level: int = 0
    rollsp_rate_lowgs: float = NAN        # roll setpoint jitter [deg/s] at low groundspeed
    rollsp_rate_highgs: float = NAN
    auto2_bank_sat_frac: float = NAN      # AUTO2 time with roll setpoint at the limit
    auto2_radial_rms: float = NAN         # circle tracking error (m) when CIRCLE msg present
    circle_radius_flown: float = NAN

    # -- pitch (iteration 1)
    n_level: int = 0
    level_theta_deg: float = NAN
    level_desired_pitch_deg: float = NAN
    pitch_bias_deg: float = NAN
    level_elevator_pprz: float = NAN
    level_airspeed: float = NAN

    # -- turns / yaw (iteration 1)
    n_turn: int = 0
    turn_bank_deg: float = NAN
    turn_yaw_cmd: float = NAN
    turn_yaw_frac_high: float = NAN
    turn_coord_ratio: float = NAN
    turn_radius_actual: float = NAN
    yaw_per_bank_pprz_deg: float = NAN

    # -- roll trim (iteration 2)
    level_aileron_pprz: float = NAN
    level_phi_deg: float = NAN
    level_desired_roll_deg: float = NAN
    roll_bias_deg: float = NAN

    # -- ETECS plant (iteration 2)
    n_quasi_steady: int = 0
    cruise_throttle: float = NAN          # throttle fraction at |Vz|<0.3, cruise speed
    cruise_airspeed: float = NAN
    throttle_per_vz: float = NAN          # d(throttle)/d(Vz)      [1/(m/s)]
    throttle_per_airspeed: float = NAN    # d(throttle)/d(V)       [1/(m/s)]
    pitch_per_vz: float = NAN             # d(theta)/d(Vz)         [rad/(m/s)]
    max_climb_full_throttle: float = NAN  # sustained Vz at throttle > 95 %
    full_throttle_frac: float = NAN       # airborne time at throttle > 95 %
    glide_ratio: float = NAN              # V / -Vz at idle
    n_glide: int = 0

    # -- roll / course loop (iteration 2)
    roll_track_lag_s: float = NAN         # DESIRED.roll -> ATTITUDE.phi lag
    roll_track_rmse_deg: float = NAN
    n_course: int = 0
    course_err_rms_deg: float = NAN       # AUTO2 course tracking error
    course_err_mean_deg: float = NAN
    course_osc_frac: float = NAN          # error sign changes per second

    # -- safety checks (iteration 3)
    state_airspeed_dead: bool = False     # AIRSPEED msg zero while AIR_DATA has airspeed
    state_airspeed_max: float = NAN
    sensor_airspeed_max: float = NAN
    ground_alt: float = NAN
    auto2_min_agl: float = NAN            # lowest height above ground in AUTO2
    auto2_max_sink: float = NAN           # most negative Vz in AUTO2
    auto2_pitch_pinned_frac: float = NAN  # AUTO2 time with pitch sp at the min limit
    auto2_alt_err_at_entry: float = NAN   # desired - actual altitude at first AUTO2 entry
    level_theta_at_10: float = NAN        # level pitch predicted at 10 m/s (IMU sanity)


def _lstsq(X: np.ndarray, y: np.ndarray) -> tuple[np.ndarray, float]:
    beta, *_ = np.linalg.lstsq(X, y, rcond=None)
    ss_res = float(((y - X @ beta) ** 2).sum())
    ss_tot = float(((y - y.mean()) ** 2).sum()) or 1.0
    return beta, 1.0 - ss_res / ss_tot


def _fill_gaps(mask: np.ndarray, max_gap: int) -> np.ndarray:
    """Set False runs shorter than max_gap between True runs to True."""
    out = mask.copy()
    idx = np.flatnonzero(mask)
    for a, b in zip(idx[:-1], idx[1:]):
        if 1 < b - a <= max_gap:
            out[a:b] = True
    return out


def _min_run(mask: np.ndarray, min_len: int) -> np.ndarray:
    """Remove True runs shorter than min_len."""
    out = mask.copy()
    padded = np.r_[False, mask, False].astype(int)
    starts = np.flatnonzero(np.diff(padded) == 1)
    ends = np.flatnonzero(np.diff(padded) == -1)
    for s, e in zip(starts, ends):
        if e - s < min_len:
            out[s:e] = False
    return out


def analyze(fl: FlightLog, pitch_min_deg: float = -25.0,
            t_range: tuple[float, float] | None = None) -> Analysis:
    a = Analysis()
    t_att, att = fl.series("ATTITUDE")           # phi psi theta [rad]
    t_gps, gps = fl.series("GPS")                # f3 course decideg, f5 speed cm/s
    t_cmd, cmd = fl.series("COMMANDS")           # THROTTLE ROLL PITCH YAW ...
    t_mode, mode = fl.series("PPRZ_MODE")        # f0: 0 manual, 1 auto1, 2 auto2

    tq = t_att
    interp = lambda t, x: np.interp(tq, t, x)
    dt = float(np.median(np.diff(tq)))
    phi = np.degrees(att[:, 0])
    psi = np.unwrap(att[:, 1])
    theta = np.degrees(att[:, 2])
    gs = interp(t_gps, gps[:, 5] / 100.0)

    if "AIR_DATA" in fl.msgs and fl.msgs["AIR_DATA"][1].shape[1] >= 6 \
            and fl.msgs["AIR_DATA"][1][:, 5].max() > 3.0:
        t_ad, ad = fl.series("AIR_DATA")
        spd = interp(t_ad, ad[:, 5])
    else:
        log.warning("AIR_DATA airspeed unusable; falling back to GPS ground speed")
        spd = gs
        a.airspeed_source = "GPS ground speed"

    zdot = np.zeros_like(tq)
    if "ESTIMATOR" in fl.msgs:
        t_est, est = fl.series("ESTIMATOR")
        zdot = interp(t_est, est[:, 1])

    if cmd.shape[1] < 4:
        raise AutotuneError("COMMANDS message has fewer than 4 axes; cannot analyze yaw")
    thr = interp(t_cmd, cmd[:, 0]) / MAX_PPRZ
    roll_c = interp(t_cmd, cmd[:, 1])
    pitch_c = interp(t_cmd, cmd[:, 2])
    yaw_c = interp(t_cmd, cmd[:, 3])
    m = np.round(interp(t_mode, mode[:, 0]))

    have_des = "DESIRED" in fl.msgs
    des_course = None
    des_alt = None
    if have_des:
        t_des, des = fl.series("DESIRED")
        des_roll = interp(t_des, np.degrees(des[:, 0]))
        des_pitch = interp(t_des, np.degrees(des[:, 1]))
        if des.shape[1] > 2:
            des_course = interp(t_des, des[:, 2])
        if des.shape[1] > 5:
            des_alt = interp(t_des, des[:, 5])

    # State airspeed health: the AIRSPEED message carries stateGetAirspeed(),
    # which every autonomous loop uses; AIR_DATA is only the sensor module.
    if "AIRSPEED" in fl.msgs:
        a.state_airspeed_max = float(fl.msgs["AIRSPEED"][1][:, 0].max())
    if "AIR_DATA" in fl.msgs and fl.msgs["AIR_DATA"][1].shape[1] >= 6:
        a.sensor_airspeed_max = float(fl.msgs["AIR_DATA"][1][:, 5].max())
    if isnum(a.sensor_airspeed_max) and a.sensor_airspeed_max > 3.0 \
            and (not isnum(a.state_airspeed_max) or a.state_airspeed_max < 1.0):
        a.state_airspeed_dead = True
    if "NAVIGATION_REF" in fl.msgs and fl.msgs["NAVIGATION_REF"][1].shape[1] > 3:
        a.ground_alt = float(np.median(fl.msgs["NAVIGATION_REF"][1][:, 3]))

    psidot = np.degrees(np.gradient(psi, tq))

    # Airborne mask per sample (not first..last!): a log can contain several
    # flights and long ground periods. Airborne = moving through the air or
    # clearly above ground; short dips (turn-around, gust) are bridged.
    z_est = np.zeros_like(tq)
    if "ESTIMATOR" in fl.msgs:
        z_est = interp(t_est, est[:, 0])
    else:
        z_est = interp(t_gps, gps[:, 4] / 1000.0)
    agl = z_est - a.ground_alt if isnum(a.ground_alt) else np.full_like(tq, 100.0)
    raw_air = (gs > 6.0) | ((spd > 6.0) & (agl > 3.0))
    airborne = _fill_gaps(raw_air, int(round(8.0 / dt)))
    # Drop ground-handling and hand-carry: require a minimum run length.
    airborne = _min_run(airborne, int(round(15.0 / dt)))
    if t_range is not None:
        airborne &= (tq >= t_range[0]) & (tq <= t_range[1])
    if airborne.sum() < 50:
        raise AutotuneError("insufficient airborne data (need sustained speed > 6 m/s)")
    sel = airborne
    a.airborne_seconds = float(sel.sum() * dt)
    a.n_flights = int(np.sum(np.diff(sel.astype(int)) == 1) + (1 if sel[0] else 0))
    a.auto1_seconds = float((sel & (m == 1)).sum() * dt)
    a.auto2_seconds = float((sel & (m == 2)).sum() * dt)
    a.airspeed_mean = float(spd[sel].mean())

    # Wind estimate from heading vs track (least squares on ground velocity
    # = airspeed * heading + wind), only where both are meaningful.
    wsel = sel & (spd > 5) & (gs > 1)
    if wsel.sum() > 200:
        crs = np.radians(interp(t_gps, gps[:, 3] / 10.0))
        hdg = att[:, 1]
        gE, gN = gs * np.sin(crs), gs * np.cos(crs)
        aE, aN = spd * np.sin(hdg), spd * np.cos(hdg)
        wE = float(np.median((gE - aE)[wsel]))
        wN = float(np.median((gN - aN)[wsel]))
        a.wind_speed = float(math.hypot(wE, wN))
        a.wind_from_deg = float(math.degrees(math.atan2(-wE, -wN)) % 360)
        a.wind_ratio = a.wind_speed / max(a.airspeed_mean, 1.0)

    # ---- straight and level, stabilized --------------------------------
    lvl = sel & (np.abs(phi) < 5) & (np.abs(zdot) < 0.5) & (np.abs(psidot) < 5) \
        & (spd > 0.75 * a.airspeed_mean) & (m >= 1)
    a.n_level = int(lvl.sum())
    if a.n_level >= 30:
        a.level_theta_deg = float(theta[lvl].mean())
        a.level_elevator_pprz = float(pitch_c[lvl].mean())
        a.level_aileron_pprz = float(roll_c[lvl].mean())
        a.level_phi_deg = float(phi[lvl].mean())
        a.level_airspeed = float(spd[lvl].mean())
        if have_des:
            a.level_desired_pitch_deg = float(des_pitch[lvl].mean())
            a.pitch_bias_deg = a.level_theta_deg - a.level_desired_pitch_deg
            a.level_desired_roll_deg = float(des_roll[lvl].mean())
            a.roll_bias_deg = a.level_phi_deg - a.level_desired_roll_deg
    else:
        log.warning("only %d straight-and-level samples; trims not tuned", a.n_level)

    # Level pitch normalised to 10 m/s (theta ~ a + b/V^2) for IMU alignment sanity.
    ql = sel & (m >= 1) & (np.abs(phi) < 8) & (np.abs(zdot) < 0.6) & (spd > 7.5)
    if ql.sum() >= 60:
        v = spd[ql]
        gamma = np.degrees(np.arcsin(np.clip(zdot[ql] / v, -0.3, 0.3)))
        beta_l, _ = _lstsq(np.c_[np.ones(ql.sum()), 1.0 / v ** 2], theta[ql] - gamma)
        a.level_theta_at_10 = float(beta_l[0] + beta_l[1] / 100.0)

    # ---- turns ---------------------------------------------------------
    sgn = np.sign(phi)
    trn = sel & (np.abs(phi) > 15) & (m >= 1) & (spd > 7)
    a.n_turn = int(trn.sum())
    if a.n_turn >= 100:
        bank = np.abs(phi[trn])
        v = np.maximum(spd[trn], 6.0)
        coord_rate = np.degrees(G * np.tan(np.radians(bank)) / v)
        act_rate = np.abs(psidot[trn])
        a.turn_bank_deg = float(bank.mean())
        a.turn_yaw_cmd = float(np.abs(yaw_c[trn]).mean())
        a.turn_yaw_frac_high = float((np.abs(yaw_c[trn]) > MAX_PPRZ / 2).mean())
        a.turn_coord_ratio = float(np.median(act_rate / np.maximum(coord_rate, 1.0)))
        a.turn_radius_actual = float(np.median(v / np.radians(np.maximum(act_rate, 0.5))))
        yb = (yaw_c * sgn)[trn]
        pos = yb > 0
        if pos.sum() > 50:
            a.yaw_per_bank_pprz_deg = float(np.median(yb[pos] / bank[pos]))
    else:
        log.warning("only %d turning samples; yaw mixing tuned from defaults", a.n_turn)

    # ---- ETECS plant identification (quasi-steady, gentle bank) --------
    qs = sel & (m >= 1) & (np.abs(phi) < 10) & (spd > 0.7 * a.airspeed_mean) & (thr > 0.02)
    a.n_quasi_steady = int(qs.sum())
    if a.n_quasi_steady >= 200:
        cr = qs & (np.abs(zdot) < 0.3)
        if cr.sum() >= 40:
            a.cruise_throttle = float(np.median(thr[cr]))
            a.cruise_airspeed = float(spd[cr].mean())
        # throttle = a0 + a1*Vz + a2*(V - Vmean): throttle feedforwards.
        X = np.c_[np.ones(qs.sum()), zdot[qs], spd[qs] - spd[qs].mean()]
        beta, r2 = _lstsq(X, thr[qs])
        if r2 > 0.1:
            a.throttle_per_vz = float(beta[1])
            a.throttle_per_airspeed = float(beta[2])
        # theta = b0 + b1*Vz : pitch per climb rate (flight path angle slope).
        beta_p, r2_p = _lstsq(np.c_[np.ones(qs.sum()), zdot[qs]], np.radians(theta[qs]))
        if r2_p > 0.2:
            a.pitch_per_vz = float(beta_p[1])
        ft = sel & (thr > 0.95) & (np.abs(phi) < 10)
        a.full_throttle_frac = float((thr[sel] > 0.95).mean())
        if ft.sum() >= 40:
            # 75th percentile: sustained capability, not a momentary zoom.
            a.max_climb_full_throttle = float(np.percentile(zdot[ft], 75))
        gl = sel & (thr < 0.03) & (zdot < -0.3) & (spd > 6.5) & (np.abs(phi) < 10)
        a.n_glide = int(gl.sum())
        if a.n_glide >= 30:
            a.glide_ratio = float(np.median(spd[gl] / -zdot[gl]))
    else:
        log.warning("only %d quasi-steady samples; ETECS plant not identified", a.n_quasi_steady)

    # ---- roll tracking (inner loop health, AUTO1) ----------------------
    s1 = sel & (m == 1)
    if have_des and s1.sum() > 200:
        d, p = des_roll[s1], phi[s1]
        a.roll_track_rmse_deg = float(np.sqrt(np.mean((d - p) ** 2)))
        lags = [np.corrcoef(d[:len(d) - L], p[L:])[0, 1] for L in range(0, 15)]
        a.roll_track_lag_s = float(int(np.nanargmax(lags)) * dt)

    # ---- course tracking (AUTO2 only) ----------------------------------
    s2 = sel & (m == 2) & (gs > 6.0)
    s2_air = sel & (m == 2) & (spd > 6.0)
    if s2_air.sum() > 5:
        z = z_est
        if isnum(a.ground_alt):
            a.auto2_min_agl = float((z[s2_air] - a.ground_alt).min())
        a.auto2_max_sink = float(zdot[s2_air].min())
        a.auto2_throttle_mean = float(thr[s2_air].mean())
        if have_des:
            a.auto2_pitch_pinned_frac = float((des_pitch[s2_air] < pitch_min_deg + 4.0).mean())
            a.pitch_err_rms_auto2 = float(np.sqrt(np.mean((des_pitch[s2_air] - theta[s2_air]) ** 2)))
            roll_lim = 40.0
            a.auto2_bank_sat_frac = float((np.abs(des_roll[s2_air]) >= roll_lim - 0.5).mean())
            # Steady elevator in AUTO2: the pilot's RC pitch trim is not in the
            # loop here, so this is the clean CG / mechanical trim indicator.
            lvl2 = s2_air & (np.abs(phi) < 12) & (np.abs(zdot) < 0.8)
            a.n_auto2_level = int(lvl2.sum())
            if a.n_auto2_level >= 30:
                a.auto2_level_elevator = float(pitch_c[lvl2].mean())
            # Roll-setpoint jitter: the course loop's output quality, split by
            # groundspeed (low = upwind, GPS course is noisy there).
            dsp = np.gradient(des_roll, tq)
            lo, hi = s2_air & (gs < 7.0), s2_air & (gs > 11.0)
            if lo.sum() > 30:
                a.rollsp_rate_lowgs = float(dsp[lo].std())
            if hi.sum() > 30:
                a.rollsp_rate_highgs = float(dsp[hi].std())
            if des_alt is not None:
                first = int(np.argmax(s2_air))
                a.auto2_alt_err_at_entry = float(des_alt[first] - z[first])
                aerr = des_alt[s2_air] - z[s2_air]
                a.auto2_alt_err_mean = float(aerr.mean())
                a.auto2_alt_low_frac = float((aerr > 15.0).mean())
        if "CIRCLE" in fl.msgs and "NAVIGATION" in fl.msgs:
            t_c, circ = fl.series("CIRCLE")
            in2 = (t_c > tq[s2_air][0]) & (t_c < tq[s2_air][-1])
            if in2.sum() > 10 and circ.shape[1] >= 3:
                cx, cy = float(np.median(circ[in2, 0])), float(np.median(circ[in2, 1]))
                a.circle_radius_flown = float(abs(np.median(circ[in2, 2])))
                t_n, nav = fl.series("NAVIGATION")
                px, py = interp(t_n, nav[:, 2]), interp(t_n, nav[:, 3])
                dist = np.hypot(px[s2_air] - cx, py[s2_air] - cy)
                a.auto2_radial_rms = float(np.sqrt(np.mean((dist - a.circle_radius_flown) ** 2)))
    if "ENERGY" in fl.msgs and fl.msgs["ENERGY"][1].shape[1] > 2:
        t_en, en = fl.series("ENERGY")
        volt, cur = interp(t_en, en[:, 1]), interp(t_en, en[:, 2])
        wot = sel & (thr > 0.98) & (spd > 6)
        rest = (thr < 0.05) & (volt > 3)
        if wot.sum() > 50 and rest.sum() > 50:
            a.power_wot_w = float((volt * cur)[wot].mean())
            a.volt_sag_wot = float(volt[rest].mean() - volt[wot].mean())
    if des_course is not None and s2.sum() * dt >= MIN_AUTO2_SECONDS:
        course = interp(t_gps, np.radians(gps[:, 3] / 10.0))        # decideg -> rad
        err = np.degrees(np.angle(np.exp(1j * (des_course[s2] - course[s2]))))
        a.n_course = int(s2.sum())
        a.course_err_rms_deg = float(np.sqrt(np.mean(err ** 2)))
        a.course_err_mean_deg = float(np.mean(err))
        sign_changes = int(np.sum(np.diff(np.sign(err - err.mean())) != 0))
        a.course_osc_frac = sign_changes / max(1e-3, a.n_course * dt)
    return a


# --------------------------------------------------------------------------
# Tuning rules -> XML edits
# --------------------------------------------------------------------------

@dataclass
class Change:
    what: str
    before: str
    after: str
    rationale: str


@dataclass
class Advice:
    """Findings that need a human or another flight, not an XML edit."""
    topic: str
    text: str


def required_bank_deg(v: float, radius: float) -> float:
    return math.degrees(math.atan((v * v) / (G * radius)))


class AirframeEditor:
    """Surgical text edits on the airframe XML that keep comments intact."""

    def __init__(self, text: str):
        self.text = text
        self.changes: list[Change] = []

    def get_define(self, name: str) -> str | None:
        m = re.search(r'<define\s+name="%s"\s+value="([^"]*)"' % re.escape(name), self.text)
        return m.group(1) if m else None

    def get_float(self, name: str, default: float) -> float:
        raw = self.get_define(name)
        if raw is None:
            return default
        try:
            return float(raw.rstrip("f").rstrip("."))
        except ValueError:
            return default

    def set_define(self, section: str, name: str, value: str, rationale: str) -> bool:
        pat = re.compile(r'(<define\s+name="%s"\s+value=")([^"]*)(")' % re.escape(name))
        m = pat.search(self.text)
        if not m:
            log.warning("define %s not found; skipped", name)
            return False
        old = m.group(2)
        if old == value:
            return False
        self.text = self.text[:m.start(2)] + value + self.text[m.end(2):]
        self.changes.append(Change(f"{section}: {name}", old, value, rationale))
        return True

    def add_define_to_section(self, section_name: str, name: str, value: str,
                              comment: str, rationale: str) -> bool:
        anchor = re.search(r'(<section name="%s"[^>]*>\n)' % re.escape(section_name), self.text)
        if not anchor:
            log.warning("section %s not found; %s not added", section_name, name)
            return False
        line = f'    <define name="{name}" value="{value}"/><!-- autotune: {comment} -->\n'
        self.text = self.text[:anchor.end(1)] + line + self.text[anchor.end(1):]
        self.changes.append(Change(f"{section_name}: {name}", "(absent)", value, rationale))
        return True


def fmt(x: float, nd: int = 3) -> str:
    return f"{round(x, nd):g}"


def tune_airframe(xml_text: str, a: Analysis, fl: FlightLog,
                  target_radius: float,
                  overrides: dict[str, str] | None = None) -> tuple[str, list[Change], list[Advice]]:
    ed = AirframeEditor(xml_text)
    advice: list[Advice] = []
    v_ref = a.level_airspeed if isnum(a.level_airspeed) else \
        (a.airspeed_mean if isnum(a.airspeed_mean) else 12.0)

    # Manual overrides (--set NAME=VALUE) are applied first and win.
    for name, value in (overrides or {}).items():
        if not ed.set_define("override", name, value, "set explicitly on the command line"):
            advice.append(Advice("override", f"--set {name}: define not found in the airframe"))

    # =====================================================================
    # 0. Flight-critical configuration faults found in the log
    # =====================================================================
    if a.state_airspeed_dead:
        # USE_AIRSPEED must reach the ap target; USE_AIRSPEED_SDP3X alone does
        # not imply it. Insert into the ap target's airspeed module block.
        ap_block = re.search(r'(<target name="ap"[^>]*>)(.*?)(</target>)', ed.text, re.S)
        fixed = False
        if ap_block:
            body = ap_block.group(2)
            live = re.sub(r"<!--.*?-->", "", body, flags=re.S)
            if not re.search(r'<define\s+name="USE_AIRSPEED"\s', live):
                new_line = ('<define name="USE_AIRSPEED" value="TRUE"/>'
                            '<!-- autotune: state airspeed was 0 in flight, ETECS flew blind -->')
                # Prefer replacing the misleading commented-out define in place.
                m_c = re.search(r'<!--\s*<define name="USE_AIRSPEED" value="TRUE"/>\s*-->'
                                r'(<!--[^\n]*-->)?', body)
                mod = re.search(r'(<module name="airspeed"[^>]*>\n)', body)
                if m_c:
                    s, e = ap_block.start(2) + m_c.start(), ap_block.start(2) + m_c.end()
                    ed.text = ed.text[:s] + new_line + ed.text[e:]
                    fixed = True
                elif mod:
                    ins = ap_block.start(2) + mod.end(1)
                    ed.text = ed.text[:ins] + "        " + new_line + "\n" + ed.text[ins:]
                    fixed = True
        ed.changes.append(Change(
            "firmware/ap: USE_AIRSPEED", "(not defined for ap target)",
            "TRUE" if fixed else "(NOT FIXED: insert manually in <target name=\"ap\">)",
            f"AIRSPEED telemetry (= stateGetAirspeed, what the autopilot uses) stayed 0 while "
            f"AIR_DATA.airspeed reached {a.sensor_airspeed_max:.1f} m/s: the sensor works but never "
            f"reaches the state. The generated Makefile only had -DUSE_AIRSPEED in the nps target. "
            f"With airspeed=0 the ETECS speed error is a constant +{v_ref:.0f} m/s, so on AUTO2 "
            f"entry it pitches to PITCH_MIN and floors the throttle regardless of altitude."))
        advice.append(Advice(
            "AUTO2 dive explained",
            f"in AUTO2 the pitch setpoint sat at the minimum limit {a.auto2_pitch_pinned_frac:.0%} "
            f"of the time, sink reached {a.auto2_max_sink:.1f} m/s and the aircraft got down to "
            f"{a.auto2_min_agl:.0f} m above ground (altitude error at entry was only "
            f"{a.auto2_alt_err_at_entry:+.0f} m). This is the signature of a speed loop that "
            f"believes it is far too slow, i.e. the missing state airspeed above, not a waypoint "
            f"or navigation problem. After the fix, verify on the bench that Messages->AIRSPEED "
            f"shows a non-zero airspeed when you blow into the pitot BEFORE any AUTO2 attempt."))
    elif isnum(a.auto2_min_agl) and (a.auto2_min_agl < 15 or
                                     (isnum(a.auto2_pitch_pinned_frac) and a.auto2_pitch_pinned_frac > 0.5)):
        advice.append(Advice(
            "AUTO2 anomaly",
            f"AUTO2 reached {a.auto2_min_agl:.0f} m AGL with pitch pinned at the minimum "
            f"{a.auto2_pitch_pinned_frac:.0%} of the time and sink {a.auto2_max_sink:.1f} m/s; "
            f"state airspeed looks alive (max {a.state_airspeed_max:.1f} m/s), so check the "
            f"airspeed setpoint vs achievable speed and the ETECS speed gains before retrying."))

    # Power wall: AUTO2 could not hold altitude although the throttle was
    # effectively saturated. No gain fixes that; say so and stop the plant
    # rules from "learning" a cruise throttle of 100 %.
    power_limited = (isnum(a.auto2_alt_low_frac) and a.auto2_alt_low_frac > 0.5
                     and isnum(a.auto2_throttle_mean) and a.auto2_throttle_mean > 0.85)
    if power_limited:
        advice.append(Advice(
            "POWER LIMITED",
            f"in AUTO2 the aircraft was more than 15 m below its altitude setpoint "
            f"{a.auto2_alt_low_frac:.0%} of the time (mean deficit {a.auto2_alt_err_mean:+.0f} m) at "
            f"{a.auto2_throttle_mean:.0%} mean throttle; full throttle sustained only "
            f"{a.max_climb_full_throttle if isnum(a.max_climb_full_throttle) else float('nan'):.1f} m/s "
            f"of climb. Electrical power at full throttle was {a.power_wot_w:.0f} W with "
            f"{a.volt_sag_wot:.2f} V battery sag. ETECS correctly protected airspeed by giving up "
            f"altitude (this is the 'throttle saturated -> climb setpoint to 0' rule in "
            f"energy_ctrl.c). The fix is thrust, not tuning: higher-pitch or larger propeller, "
            f"3S battery (check ESC/motor limits), fresh cells, or less weight/drag. Until then "
            f"keep AUTO2 to calm days and set the flight-plan altitude to what it can hold: it "
            f"flew about {a.auto2_alt_err_mean:.0f} m below the current setpoint, so lower the "
            f"flight plan ALT by that much (or raise it only after the propulsion change)."))
    if isnum(a.wind_ratio) and a.wind_ratio > 0.5:
        advice.append(Advice(
            "wind",
            f"estimated wind {a.wind_speed:.1f} m/s from {a.wind_from_deg:.0f} deg = "
            f"{a.wind_ratio:.0%} of the {a.airspeed_mean:.1f} m/s airspeed. Course tracking, "
            f"circle shape and ground-speed based metrics are dominated by wind in this log; "
            f"course-loop gains were NOT changed from these data. Groundspeed fell to "
            f"{max(0.0, a.airspeed_mean - a.wind_speed):.1f} m/s upwind, so AUTO_GROUNDSPEED_SETPOINT "
            f"pushed the airspeed target up, costing more throttle."))

    # IMU alignment sanity: level pitch normalised to 10 m/s should be a few
    # degrees for a small foam wing. Compare against the flown mount angle.
    b2i_flown = fl.flown("BODY_TO_IMU_THETA", ed.get_float("BODY_TO_IMU_THETA", 0.0))
    if isnum(a.level_theta_at_10) and a.level_theta_at_10 > 4.5:
        suggested = b2i_flown + math.radians(a.level_theta_at_10 - 2.5)
        advice.append(Advice(
            "IMU pitch alignment",
            f"level-flight body pitch normalised to 10 m/s is {a.level_theta_at_10:+.1f} deg with "
            f"IMU_BODY_TO_IMU_THETA={b2i_flown:.3f} rad. A cruise alpha of 2-3 deg is expected; "
            f"if the FC was moved, raise BODY_TO_IMU_THETA to about {suggested:.3f} rad. Confirm on "
            f"the bench: hold the wing chord level and read ATTITUDE.theta in the GCS; add that "
            f"reading (in rad) to BODY_TO_IMU_THETA. Flight data alone cannot separate IMU tilt "
            f"from a genuinely higher angle of attack (heavier aircraft or slower flight)."))

    # =====================================================================
    # 1. Pitch and roll trim (COMMAND_*_TRIM: added to commands in
    #    actuators.c, so new_trim = trim flown + steady command observed)
    # =====================================================================
    # In AUTO1 the pilot's RC pitch trim biases the setpoint, so the steady
    # elevator there can be misleading. AUTO2 has no stick in the loop: prefer it.
    if isnum(a.auto2_level_elevator) and a.n_auto2_level >= 30:
        elev_src, elev_val = f"AUTO2 (n={a.n_auto2_level})", a.auto2_level_elevator
    else:
        elev_src, elev_val = "AUTO1 level", a.level_elevator_pprz
    if isnum(elev_val) and abs(elev_val) > 100:
        flown = fl.flown("PITCH_TRIM", ed.get_float("PITCH_TRIM", 0.0))
        want = flown + elev_val
        new = int(max(-TRIM_LIMIT, min(TRIM_LIMIT, round(want / 10) * 10)))
        ed.set_define("TRIM", "PITCH_TRIM", str(new),
                      f"the pitch loop held a steady {elev_val:+.0f} pprz elevator in {elev_src} "
                      f"flight; moving that into the command trim centres the servo and frees "
                      f"the integrator (flown trim {flown:+.0f})")
        if abs(want) > TRIM_LIMIT:
            advice.append(Advice("pitch trim / CG",
                                 f"required pitch trim {want:+.0f} pprz exceeds the firmware clip of "
                                 f"+-{TRIM_LIMIT}. The aircraft is nose-heavy: move the CG aft "
                                 f"(battery back ~5 mm per step, dive test at altitude) or add "
                                 f"up-elevator at the ruddervator clevises. Target after the change: "
                                 f"AUTO2 steady elevator between +100 and +400 pprz."))
        if isnum(a.level_elevator_pprz) and isnum(a.auto2_level_elevator) \
                and abs(a.level_elevator_pprz - a.auto2_level_elevator) > 400:
            advice.append(Advice("RC trim",
                                 f"AUTO1 steady elevator ({a.level_elevator_pprz:+.0f}) differs from AUTO2 "
                                 f"({a.auto2_level_elevator:+.0f}) by more than 400 pprz: the transmitter "
                                 f"pitch trim was not centred in AUTO1. Keep TX trims at zero; trim in the "
                                 f"airframe file instead so MANUAL, AUTO1 and AUTO2 agree."))

    if isnum(a.level_aileron_pprz) and abs(a.level_aileron_pprz) > 100:
        flown = fl.flown("ROLL_TRIM", ed.get_float("ROLL_TRIM", 0.0))
        want = flown + a.level_aileron_pprz
        new = int(max(-TRIM_LIMIT, min(TRIM_LIMIT, round(want / 10) * 10)))
        ed.set_define("TRIM", "ROLL_TRIM", str(new),
                      f"the roll loop held a steady {a.level_aileron_pprz:+.0f} pprz aileron to fly "
                      f"wings level (bank {a.level_phi_deg:+.1f} deg vs demanded "
                      f"{a.level_desired_roll_deg:+.1f} deg); a lateral imbalance or aileron "
                      f"misalignment is absorbed in the trim (flown trim {flown:+.0f})")

    # Nominal cruise pitch follows the measured level attitude.
    if isnum(a.level_theta_deg):
        theta_rad = math.radians(a.level_theta_deg)
        if 0.0 < theta_rad < 0.2:
            ed.set_define("V_CTL", "AUTO_THROTTLE_NOMINAL_CRUISE_PITCH", fmt(theta_rad),
                          f"measured level-flight pitch at {a.level_airspeed:.1f} m/s is "
                          f"{a.level_theta_deg:.1f} deg = {theta_rad:.3f} rad")

    # =====================================================================
    # 2. Turn coordination and turn radius
    # =====================================================================
    # The firmware already has a coordinated-turn yaw loop
    # (stabilization_adaptive.c: yaw_rate_ref = g/V*sin(roll_sp), rudder =
    # DGAIN*(ref - r)). It is correct in sign by construction, works in AUTO1
    # and AUTO2, and turns the pilot's rudder stick into a yaw-rate command.
    # A hand-made "roll into rudder" command-law mixer cannot know the sign of
    # @ROLL (the roll loop outputs -aileron_setpoint) and turned out inverted
    # in flight, so it is removed in favour of the yaw loop.
    bank_needed = required_bank_deg(v_ref, target_radius)

    if "$ruddervons" in ed.text:
        ed.text = re.sub(r'\n\s*<let var="ruddervons"[^\n]*\n', "\n", ed.text, count=1)
        ed.text = re.sub(r'(<set\s+servo="S_RUDDERVATOR_LEFT"\s+value=")1\.8\*@PITCH\+\$ruddervons(")',
                         r'\g<1>1.8*@PITCH+@YAW\2', ed.text, count=1)
        ed.text = re.sub(r'(<set\s+servo="S_RUDDERVATOR_RIGHT"\s+value=")1\.8\*@PITCH-\$ruddervons(")',
                         r'\g<1>1.8*@PITCH-@YAW\2', ed.text, count=1)
        ed.text = re.sub(r'\n\s*<define name="RUDDERVONS_OF_ROLL"[^\n]*', "", ed.text, count=1)
        ed.changes.append(Change(
            "command_laws: ruddervon mixer", "$ruddervons = @YAW + k*@ROLL", "removed (plain @YAW)",
            f"with the mixer active the turn coordination ratio dropped to {a.turn_coord_ratio:.2f} "
            f"(was 0.88 without it): in AUTO the roll command is -aileron_setpoint, so the mixer "
            f"added rudder AGAINST the turn. Coordination is now done by the firmware yaw loop."))

    if not re.search(r'<define\s+name="YAW_LOOP"', ed.text):
        # Enable the adaptive-stabilization yaw loop with conservative gains.
        # DGAIN: pprz per rad/s of yaw-rate error. 5000 gives half rudder for
        # a 1 rad/s error; NY_IGAIN 0 disables lateral-g trimming until the
        # sign/authority is verified in flight.
        anchor = re.search(r'(<section name="HORIZONTAL CONTROL" prefix="H_CTL_">\n)', ed.text)
        if anchor:
            block = ('    <!-- autotune: coordinated-turn yaw loop (rudder = YAW_DGAIN*(g/V*sin(roll_sp) - r)).\n'
                     '         Replaces the command-law roll->rudder mixer, which was sign-inverted in AUTO. -->\n'
                     '    <define name="YAW_LOOP" value="TRUE"/>\n'
                     '    <define name="YAW_DGAIN" value="5000."/>\n'
                     '    <define name="YAW_TRIM_NY" value="FALSE"/>\n'
                     '    <define name="YAW_NY_IGAIN" value="0."/>\n')
            ed.text = ed.text[:anchor.end(1)] + block + ed.text[anchor.end(1):]
            ed.changes.append(Change(
                "H_CTL: YAW_LOOP / YAW_DGAIN", "(absent)", "TRUE / 5000",
                f"turns were uncoordinated (achieved/ideal yaw rate {a.turn_coord_ratio:.2f}, pilot held "
                f"{a.yaw_per_bank_pprz_deg:.0f} pprz rudder per deg of bank in AUTO1). The yaw loop "
                f"commands rudder for the coordinated yaw rate g*sin(phi)/V using the gyro, in every "
                f"AUTO mode, with the correct sign by construction."))
            ed.add_define_to_section("AUTO1", "MAX_YAW_RATE", "60",
                                     "deg/s full rudder stick in AUTO1 with the yaw loop",
                                     "with YAW_LOOP the AUTO1 rudder stick commands a yaw rate; 60 deg/s "
                                     "at full stick keeps the pilot's rudder authority")
            # unit attribute so the generator converts deg/s -> rad/s
            ed.text = ed.text.replace('<define name="MAX_YAW_RATE" value="60"/>',
                                      '<define name="MAX_YAW_RATE" value="60" unit="deg/s"/>', 1)

    # The AUTO passthrough of the RC yaw channel leaks any transmitter rudder
    # trim into AUTO2 as a constant rudder offset; with the yaw loop the stick
    # is already a yaw-rate setpoint, so drop the passthrough. Only inside
    # <auto_rc_commands>: the MANUAL mapping in <rc_commands> must stay.
    blk = re.search(r"<auto_rc_commands>.*?</auto_rc_commands>", ed.text, re.S)
    pat_auto = re.compile(r'\n[ \t]*<set command="YAW" value="@YAW(?:\*0\.9)?"/>[^\n]*')
    if blk and pat_auto.search(blk.group(0)):
        new_blk = pat_auto.sub("\n    <!-- autotune: RC yaw passthrough removed, COMMAND_YAW is owned by the yaw loop -->",
                               blk.group(0), count=1)
        ed.text = ed.text[:blk.start()] + new_blk + ed.text[blk.end():]
        ed.changes.append(Change("auto_rc_commands: YAW passthrough", "@YAW", "removed",
                                 "a constant rudder offset (transmitter trim) was visible in AUTO2 "
                                 "telemetry; the yaw loop now owns COMMAND_YAW in AUTO modes"))

    # Circle radius sized for the wind actually encountered: downwind
    # groundspeed is V+W and the bank needed is atan(V*(V+W)/(g R)); keep it
    # at or below 25 deg so the roll limit and the drag stay in reserve.
    w_design = a.wind_speed if isnum(a.wind_speed) else 0.0
    r_wind = v_ref * (v_ref + w_design) / (G * math.tan(math.radians(25.0)))
    r_use = max(target_radius, math.ceil(r_wind / 5.0) * 5.0)
    if isnum(a.auto2_bank_sat_frac) and a.auto2_bank_sat_frac > 0.1 and r_use > target_radius:
        for name in ("DEFAULT_CIRCLE_RADIUS", "MIN_CIRCLE_RADIUS", "LANDING_CIRCLE_RADIUS"):
            ed.set_define("MISC", name, fmt(r_use, 1),
                          f"roll setpoint sat at its limit {a.auto2_bank_sat_frac:.0%} of AUTO2 and the "
                          f"circle tracking error was {a.auto2_radial_rms:.0f} m RMS: a "
                          f"{a.circle_radius_flown:.0f} m circle needs "
                          f"{math.degrees(math.atan(v_ref * (v_ref + w_design) / (G * a.circle_radius_flown))):.0f} deg "
                          f"bank downwind in {w_design:.0f} m/s wind; {r_use:.0f} m needs 25 deg")
    else:
        ed.set_define("MISC", "MIN_CIRCLE_RADIUS", fmt(target_radius, 1),
                      f"requested minimum autonomous turn radius is {target_radius:g} m (needs "
                      f"{bank_needed:.0f} deg bank at {v_ref:.1f} m/s, within the roll limit)")
        if target_radius > 25.0:
            ed.set_define("MISC", "LANDING_CIRCLE_RADIUS", fmt(target_radius, 1),
                          "landing circle must not be tighter than the guaranteed minimum radius")

    # Course loop jitter at low groundspeed: the roll setpoint moved much
    # faster upwind than downwind although roll tracking was equal, i.e. the
    # course loop was chasing GPS-course noise. Remove the derivative term.
    if isnum(a.rollsp_rate_lowgs) and isnum(a.rollsp_rate_highgs) \
            and a.rollsp_rate_lowgs > 2.0 * a.rollsp_rate_highgs and a.rollsp_rate_lowgs > 6.0:
        if ed.get_float("COURSE_DGAIN", 0.0) > 0.0:
            ed.set_define("H_CTL", "COURSE_DGAIN", "0.0",
                          f"roll setpoint jitter was {a.rollsp_rate_lowgs:.1f} deg/s at low groundspeed "
                          f"(upwind) vs {a.rollsp_rate_highgs:.1f} deg/s downwind while roll tracking "
                          f"error was the same: the course derivative term amplifies GPS course noise "
                          f"when groundspeed is low")
        advice.append(Advice("upwind roll wobble",
                             f"the wobble you saw upwind is real and comes from the course loop, not "
                             f"the roll loop: setpoint rate {a.rollsp_rate_lowgs:.1f} deg/s upwind vs "
                             f"{a.rollsp_rate_highgs:.1f} downwind, roll error RMS equal. COURSE_DGAIN "
                             f"was zeroed; the larger circle radius also halves the course rate needed."))

    # =====================================================================
    # 3. ETECS energy-control plant (from AUTO1 quasi-steady flight)
    # =====================================================================
    if isnum(a.cruise_throttle) and a.cruise_throttle >= 0.95:
        # A saturated throttle is not a cruise point; learning it would remove
        # all climb authority from the feedforward. Keep the previous value.
        advice.append(Advice(
            "no cruise point",
            f"altitude could only be held at {a.cruise_throttle:.0%} throttle in this log; that "
            f"is not a cruise condition, so the ETECS cruise/feedforward values were left as "
            f"they were (from the previous run) rather than learned from a saturated flight."))
        a.cruise_throttle = NAN

    if isnum(a.cruise_throttle):
        ed.set_define("V_CTL", "AUTO_THROTTLE_NOMINAL_CRUISE_THROTTLE", fmt(a.cruise_throttle, 2),
                      f"median throttle to hold altitude at {a.cruise_airspeed:.1f} m/s was "
                      f"{a.cruise_throttle:.0%}; the ETECS adaptive cruise term starts here "
                      f"instead of hunting up from the model value")
        v_c = round(a.cruise_airspeed, 1)
        ed.set_define("MISC", "NOMINAL_AIRSPEED", fmt(v_c, 1),
                      "measured cruise airspeed; also scales the course-loop speed compensation")
        ed.set_define("V_CTL", "AUTO_AIRSPEED_SETPOINT", fmt(v_c, 1),
                      "autonomous airspeed target set to the demonstrated cruise speed")
        ed.set_define("MISC", "TRACKING_AIRSPEED", fmt(v_c + 1.0, 1),
                      "one m/s above cruise, consistent with the new nominal airspeed")
        if a.cruise_throttle > 0.7:
            advice.append(Advice(
                "propulsion margin",
                f"holding altitude needs {a.cruise_throttle:.0%} throttle and the aircraft spent "
                f"{a.full_throttle_frac:.0%} of the flight at full throttle. That leaves little "
                f"climb authority for ETECS; check propeller, motor kV, battery sag "
                f"(ENERGY message) and airframe drag before an AUTO2 flight."))

    if isnum(a.max_climb_full_throttle) and isnum(a.cruise_throttle):
        climb_avail = max(0.5, a.max_climb_full_throttle)
        incr = max(0.05, min(1.0, (1.0 - a.cruise_throttle) / climb_avail))
        ed.set_define("V_CTL", "AUTO_THROTTLE_CLIMB_THROTTLE_INCREMENT", fmt(incr, 2),
                      f"full throttle gave a sustained {climb_avail:.1f} m/s climb from a "
                      f"{a.cruise_throttle:.0%} cruise: (1 - {a.cruise_throttle:.2f}) / "
                      f"{climb_avail:.1f} = {incr:.2f} throttle per m/s of climb demand")
        max_climb = max(0.5, round(0.8 * climb_avail, 1))
        if max_climb < ed.get_float("ALTITUDE_MAX_CLIMB", 2.0):
            ed.set_define("V_CTL", "ALTITUDE_MAX_CLIMB", fmt(max_climb, 1),
                          f"only {climb_avail:.1f} m/s sustained climb is available at full throttle; "
                          f"commanding more would starve the speed loop (80 % margin applied)")

    if isnum(a.pitch_per_vz) and 0.02 < a.pitch_per_vz < 0.2:
        ed.set_define("V_CTL", "AUTO_THROTTLE_PITCH_OF_VZ_PGAIN", fmt(a.pitch_per_vz, 3),
                      f"regression of body pitch on climb rate over {a.n_quasi_steady} quasi-steady "
                      f"samples gives {a.pitch_per_vz:.3f} rad per m/s")

    if isnum(a.throttle_per_airspeed) and 0.01 < a.throttle_per_airspeed < 0.2:
        ed.set_define("V_CTL", "AUTO_THROTTLE_OF_AIRSPEED_PGAIN", fmt(a.throttle_per_airspeed, 3),
                      f"measured cruise-throttle slope {a.throttle_per_airspeed:.3f} per m/s of "
                      f"airspeed; the proportional speed term matches the drag curve")

    if isnum(a.glide_ratio) and 3.0 < a.glide_ratio < 25.0:
        ed.set_define("V_CTL", "GLIDE_RATIO", fmt(a.glide_ratio, 1),
                      f"idle-throttle descent over {a.n_glide} samples gave V/Vz = {a.glide_ratio:.1f}; "
                      f"used for the pitch-down when throttle is killed")

    # =====================================================================
    # 3b. Safety envelope for an aircraft with little power margin
    # =====================================================================
    v_stall = ed.get_float("STALL_AIRSPEED", 6.7)
    v_cruise = a.cruise_airspeed if isnum(a.cruise_airspeed) else v_ref
    # Bank limit: keep the accelerated stall speed (Vs*sqrt(n)) 15 % under cruise.
    n_max = (v_cruise / (1.15 * v_stall)) ** 2
    if n_max > 1.0:
        bank_max = math.degrees(math.acos(1.0 / n_max))
        bank_max = max(20.0, min(45.0, math.floor(bank_max / 5.0) * 5.0))
        for name in ("ROLL_MAX_SETPOINT", "MAX_ROLL"):
            cur = ed.get_float(name, 45.0)
            if bank_max < cur:
                ed.set_define("H_CTL/AUTO1", name, fmt(bank_max, 0),
                              f"at {v_cruise:.1f} m/s cruise with {v_stall:.1f} m/s stall, a "
                              f"{cur:.0f} deg bank raises stall speed to "
                              f"{v_stall * math.sqrt(1 / math.cos(math.radians(cur))):.1f} m/s; "
                              f"{bank_max:.0f} deg keeps a 15 % margin (turns in the log were flown "
                              f"at {a.turn_bank_deg:.0f} deg and {a.airspeed_mean:.1f} m/s, "
                              f"i.e. at the stall boundary)")
    # Pitch-down limit: no steeper than the idle glide path plus a small margin.
    if isnum(a.glide_ratio):
        pitch_min = -math.ceil(math.degrees(math.atan(1.0 / a.glide_ratio)) + 3.0)
        pitch_min = max(-25.0, min(-8.0, pitch_min))
        cur = ed.get_float("PITCH_MIN_SETPOINT", -25.0)
        if pitch_min > cur:
            ed.set_define("H_CTL", "PITCH_MIN_SETPOINT", fmt(pitch_min, 0),
                          f"idle glide path is {math.degrees(math.atan(1.0 / a.glide_ratio)):.0f} deg "
                          f"down (L/D {a.glide_ratio:.1f}); allowing {cur:.0f} deg let the "
                          f"controller dive at {a.auto2_max_sink if isnum(a.auto2_max_sink) else -6:.0f} m/s")
    # Speed-loop aggressiveness: a 2 m/s speed error must not command more
    # pitch-down than a few degrees on an aircraft that cannot out-climb it.
    if isnum(a.cruise_throttle) and a.cruise_throttle > 0.7:
        ed.set_define("V_CTL", "MAX_ACCELERATION", "0.15",
                      "with no throttle margin, speed errors must be corrected gently: "
                      "0.15 g caps the energy-distribution pitch-down at ~5 deg")
        ed.set_define("V_CTL", "AUTO_PITCH_OF_AIRSPEED_IGAIN", "0.001",
                      "slow the adaptive cruise-pitch integrator: at 0.004 a sustained speed "
                      "error winds the pitch reference to its limit within ~10 s")
        ed.set_define("V_CTL", "ENERGY_DIFF_IGAIN", "0.02",
                      "energy-distribution integrator reduced for the same reason")

    # =====================================================================
    # 4. Course loop: measured from AUTO2 when available, else model check
    # =====================================================================
    pgain = ed.get_float("COURSE_PGAIN", 0.9)
    windy = isnum(a.wind_ratio) and a.wind_ratio > 0.5
    if a.n_course > 0 and not windy:
        if a.course_osc_frac >= 0.3:
            new = round(max(0.4, pgain * 0.8), 2)
            ed.set_define("H_CTL", "COURSE_PGAIN", fmt(new, 2),
                          f"AUTO2 course error changes sign {a.course_osc_frac:.2f} times per second: "
                          f"oscillatory tracking, gain reduced 20 %")
        elif a.course_err_rms_deg > 15:
            new = round(min(1.5, pgain * 1.2), 2)
            ed.set_define("H_CTL", "COURSE_PGAIN", fmt(new, 2),
                          f"AUTO2 course error RMS {a.course_err_rms_deg:.1f} deg without oscillation "
                          f"over {a.auto2_seconds:.0f} s: sluggish tracking, gain raised 20 %")
        else:
            advice.append(Advice("course loop",
                                 f"AUTO2 course tracking is healthy (RMS {a.course_err_rms_deg:.1f} deg, "
                                 f"{a.course_osc_frac:.2f} sign changes/s); COURSE_PGAIN {pgain} kept"))
    elif isnum(a.roll_track_lag_s) and not windy:
        # Model check: course-loop bandwidth ~ pgain*g/V must stay well below
        # the roll-tracking bandwidth (1/lag) or the outer loop oscillates.
        bw_course = pgain * G / v_ref
        bw_roll = 1.0 / max(a.roll_track_lag_s, 0.05)
        ratio = bw_roll / bw_course
        if ratio < 3.0:
            new = round(max(0.4, pgain * ratio / 3.0), 2)
            ed.set_define("H_CTL", "COURSE_PGAIN", fmt(new, 2),
                          f"roll loop lags its setpoint by {a.roll_track_lag_s:.2f} s "
                          f"({bw_roll:.1f} rad/s) while course bandwidth is {bw_course:.1f} rad/s; "
                          f"separation {ratio:.1f}x is below the safe 3x, gain reduced")
        advice.append(Advice(
            "course loop",
            f"no airborne AUTO2 time >= {MIN_AUTO2_SECONDS:.0f} s in this log "
            f"({a.auto2_seconds:.0f} s found). Model check: roll tracks its setpoint with "
            f"{a.roll_track_lag_s:.2f} s lag and {a.roll_track_rmse_deg:.1f} deg RMS, course "
            f"bandwidth at {v_ref:.1f} m/s is {bw_course:.1f} rad/s -> separation {ratio:.1f}x "
            f"({'OK' if ratio >= 3 else 'tight'}). A 1-2 minute AUTO2 circle or line at safe "
            f"altitude is enough to tune COURSE_PGAIN from data on the next run."))
    return ed.text, ed.changes, advice


# --------------------------------------------------------------------------
# Output, versioning, build verification
# --------------------------------------------------------------------------

def next_version_path(out_dir: Path, base_name: str) -> Path:
    existing = sorted(out_dir.glob(f"{base_name}_optim_[0-9][0-9][0-9].xml"))
    last = int(existing[-1].stem.rsplit("_", 1)[1]) if existing else 0
    return out_dir / f"{base_name}_optim_{last + 1:03d}.xml"


def validate_xml(path: Path) -> None:
    try:
        ET.parse(str(path))
    except ET.ParseError as exc:
        raise AutotuneError(f"generated XML is not well-formed: {path}: {exc}") from exc


def verify_build(pprz_home: Path, aircraft: str, airframe_rel: str, tuned_xml: Path,
                 target: str = "ap") -> tuple[bool, str]:
    """Compile with the tuned XML substituted for the original; always restore."""
    orig = resolve_existing(pprz_home / "conf" / airframe_rel, "base airframe")
    backup = orig.read_bytes()
    try:
        orig.write_bytes(tuned_xml.read_bytes())
        cmd = ["make", "-C", str(pprz_home), "-f", "Makefile.ac",
               f"AIRCRAFT={aircraft}", f"{target}.compile"]
        log.info("build verification: %s", " ".join(cmd))
        res = subprocess.run(cmd, capture_output=True, text=True, timeout=1800)
        tail = "\n".join((res.stdout + "\n" + res.stderr).splitlines()[-40:])
        return res.returncode == 0, tail
    finally:
        orig.write_bytes(backup)


# --------------------------------------------------------------------------
# Audit report
# --------------------------------------------------------------------------

def _f(x: float, spec: str = "6.2f", unit: str = "") -> str:
    return (format(x, spec) + unit) if isnum(x) else "n/a"


def write_report(report_path: Path, fl: FlightLog, a: Analysis, changes: list[Change],
                 advice: list[Advice], out_xml: Path, base_xml: Path, target_radius: float,
                 build_status: str, build_tail: str) -> str:
    v_ref = a.level_airspeed if isnum(a.level_airspeed) else a.airspeed_mean
    L = [
        "=" * 74,
        "PAPARAZZI AIRFRAME AUTO-TUNE REPORT (Iteration 2)",
        f"generated: {datetime.now().isoformat(timespec='seconds')}",
        "=" * 74,
        f"flight log : {fl.data_path}",
        f"aircraft   : {fl.aircraft_name} (AC_ID {fl.ac_id})",
        f"base XML   : {base_xml}",
        f"output XML : {out_xml}",
        f"build check: {build_status}",
        "",
        "-- FLIGHT " + "-" * 64,
        f"  airborne {a.airborne_seconds:.0f} s in {a.n_flights} flight(s) | AUTO1 {a.auto1_seconds:.0f} s | AUTO2 {a.auto2_seconds:.0f} s",
        f"  wind estimate          : {_f(a.wind_speed, '.1f', ' m/s')} from {_f(a.wind_from_deg, '.0f', ' deg')}"
        f"  ({_f(a.wind_ratio, '.0%')} of airspeed)",
        f"  power @ full throttle  : {_f(a.power_wot_w, '.0f', ' W')}, battery sag {_f(a.volt_sag_wot, '.2f', ' V')}",
        f"  AUTO2 altitude deficit : mean {_f(a.auto2_alt_err_mean, '+.0f', ' m')}, >15 m low {_f(a.auto2_alt_low_frac, '.0%')} of time,"
        f" throttle {_f(a.auto2_throttle_mean, '.0%')}, pitch RMS {_f(a.pitch_err_rms_auto2, '.1f', ' deg')}",
        f"  airspeed source        : {a.airspeed_source}",
        f"  mean airspeed          : {_f(a.airspeed_mean, '.2f', ' m/s')}",
        f"  STATE airspeed max     : {_f(a.state_airspeed_max, '.1f', ' m/s')}  (sensor AIR_DATA max "
        f"{_f(a.sensor_airspeed_max, '.1f', ' m/s')})" + ("   <<< STATE AIRSPEED DEAD" if a.state_airspeed_dead else ""),
        f"  ground alt (nav ref)   : {_f(a.ground_alt, '.0f', ' m')}",
        f"  AUTO2 min AGL / sink   : {_f(a.auto2_min_agl, '.0f', ' m')} / {_f(a.auto2_max_sink, '.1f', ' m/s')}"
        f"  pitch at min limit {_f(a.auto2_pitch_pinned_frac, '.0%')} of AUTO2, alt err at entry "
        f"{_f(a.auto2_alt_err_at_entry, '+.0f', ' m')}",
        f"  level pitch @10 m/s    : {_f(a.level_theta_at_10, '+.1f', ' deg')} (IMU alignment sanity)",
        "",
        f"-- PITCH / ROLL TRIM  (straight+level, n={a.n_level}) " + "-" * 25,
        f"  body pitch / demanded  : {_f(a.level_theta_deg, '+.2f')} / {_f(a.level_desired_pitch_deg, '+.2f')} deg"
        f"  -> error {_f(a.pitch_bias_deg, '+.2f')} deg",
        f"  steady elevator cmd    : {_f(a.level_elevator_pprz, '+.0f', ' pprz')}",
        f"  bank / demanded        : {_f(a.level_phi_deg, '+.2f')} / {_f(a.level_desired_roll_deg, '+.2f')} deg"
        f"  -> error {_f(a.roll_bias_deg, '+.2f')} deg",
        f"  steady aileron cmd     : {_f(a.level_aileron_pprz, '+.0f', ' pprz')}",
        "",
        f"-- TURNS / YAW  (|bank|>15 deg, n={a.n_turn}) " + "-" * 32,
        f"  mean |bank|            : {_f(a.turn_bank_deg, '.1f', ' deg')}",
        f"  mean |yaw cmd|         : {_f(a.turn_yaw_cmd, '.0f')} pprz (of {MAX_PPRZ})",
        f"  time above half rudder : {_f(a.turn_yaw_frac_high, '.0%')}",
        f"  turn coordination ratio: {_f(a.turn_coord_ratio, '.2f')} (achieved/ideal)",
        f"  median achieved radius : {_f(a.turn_radius_actual, '.1f', ' m')} (with pilot rudder)",
        f"  rudder per deg of bank : {_f(a.yaw_per_bank_pprz_deg, '.0f', ' pprz/deg')}",
        f"  target turn radius     : {target_radius:.1f} m (= {required_bank_deg(v_ref, target_radius):.0f} deg bank at {v_ref:.1f} m/s)",
        "",
        f"-- ETECS PLANT  (quasi-steady, n={a.n_quasi_steady}) " + "-" * 33,
        f"  cruise throttle        : {_f(a.cruise_throttle, '.0%')} at {_f(a.cruise_airspeed, '.1f', ' m/s')}",
        f"  throttle per m/s climb : {_f(a.throttle_per_vz, '.3f')} (regression)",
        f"  throttle per m/s speed : {_f(a.throttle_per_airspeed, '.3f')}",
        f"  pitch per m/s climb    : {_f(a.pitch_per_vz, '.3f', ' rad/(m/s)')}",
        f"  sustained climb @ WOT  : {_f(a.max_climb_full_throttle, '.2f', ' m/s')}"
        f"  ({_f(a.full_throttle_frac, '.0%')} of flight at full throttle)",
        f"  glide ratio (idle)     : {_f(a.glide_ratio, '.1f')} (n={a.n_glide})",
        "",
        "-- ROLL / COURSE LOOPS " + "-" * 51,
        f"  roll setpoint tracking : lag {_f(a.roll_track_lag_s, '.2f', ' s')}, RMS {_f(a.roll_track_rmse_deg, '.1f', ' deg')} (AUTO1)",
        f"  AUTO2 course samples   : {a.n_course}",
        f"  course error RMS/mean  : {_f(a.course_err_rms_deg, '.1f')} / {_f(a.course_err_mean_deg, '+.1f')} deg",
        f"  course sign changes    : {_f(a.course_osc_frac, '.2f', ' /s')}",
        f"  roll-sp jitter lo/hi gs: {_f(a.rollsp_rate_lowgs, '.1f')} / {_f(a.rollsp_rate_highgs, '.1f')} deg/s  (upwind vs downwind)",
        f"  AUTO2 circle           : R {_f(a.circle_radius_flown, '.0f', ' m')}, radial err RMS {_f(a.auto2_radial_rms, '.0f', ' m')},"
        f" roll sp at limit {_f(a.auto2_bank_sat_frac, '.0%')}",
        f"  AUTO2 steady elevator  : {_f(a.auto2_level_elevator, '+.0f', ' pprz')} (n={a.n_auto2_level}, no RC trim in loop)",
        "",
        "-- XML CHANGES " + "-" * 59,
    ]
    if changes:
        for i, c in enumerate(changes, 1):
            L += [f"{i}. {c.what}", f"   before : {c.before}", f"   after  : {c.after}",
                  f"   why    : {c.rationale}", ""]
    else:
        L.append("(no changes were necessary)")
    if advice:
        L.append("-- ADVICE / NEXT FLIGHT " + "-" * 50)
        for ad in advice:
            L += [f"* [{ad.topic}] {ad.text}", ""]
    if build_tail:
        L += ["-- BUILD OUTPUT (tail) " + "-" * 51, build_tail]
    L.append("=" * 74)
    report = "\n".join(L)
    report_path.write_text(report)
    return report


# --------------------------------------------------------------------------
# Main
# --------------------------------------------------------------------------

def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="Tune a Paparazzi airframe XML from flight logs.")
    ap.add_argument("--log-name", default="26_09_05__18_42_28", help="log basename without extension")
    ap.add_argument("--log-dir", type=Path, default=PPRZ_HOME / "var" / "logs")
    ap.add_argument("--airframe-out", type=Path, default=PPRZ_HOME / "conf" / "airframes" / "OPENUAS")
    ap.add_argument("--base", type=Path, default=None,
                    help="airframe XML to tune (default: the one referenced by the log)")
    ap.add_argument("--ac-id", type=int, default=129)
    ap.add_argument("--turn-radius", type=float, default=30.0,
                    help="target minimum autonomous turn radius [m]")
    ap.add_argument("--set", action="append", default=[], metavar="NAME=VALUE",
                    help="force an airframe define (repeatable), e.g. --set BODY_TO_IMU_THETA=0.16")
    ap.add_argument("--t-range", type=float, nargs=2, metavar=("T0", "T1"), default=None,
                    help="only analyse log time [T0, T1] seconds (e.g. one of several flights)")
    ap.add_argument("--target", default="ap", help="build target for verification")
    ap.add_argument("--no-build", action="store_true", help="skip compile verification")
    ap.add_argument("-v", "--verbose", action="store_true")
    args = ap.parse_args(argv)

    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO,
                        format="%(levelname)s %(message)s")
    try:
        log_dir = resolve_existing(args.log_dir, "log directory")
        out_dir = resolve_existing(args.airframe_out, "airframe output directory")

        fl = load_flight_log(log_dir, args.log_name, args.ac_id)
        base_xml = resolve_existing(args.base or (PPRZ_HOME / "conf" / fl.airframe_rel),
                                    "base airframe XML")
        base_name = re.sub(r"_optim_\d{3}$", "", base_xml.stem)

        overrides = {}
        for item in args.set:
            if "=" not in item:
                raise AutotuneError(f"--set expects NAME=VALUE, got '{item}'")
            k, v = item.split("=", 1)
            overrides[k.strip()] = v.strip()

        base_text = base_xml.read_text()
        m_pmin = re.search(r'name="PITCH_MIN_SETPOINT"\s+value="([^"]+)"', base_text)
        pitch_min = float(m_pmin.group(1)) if m_pmin else -25.0
        analysis = analyze(fl, pitch_min, tuple(args.t_range) if args.t_range else None)
        tuned_text, changes, advice = tune_airframe(base_text, analysis, fl,
                                                    args.turn_radius, overrides)

        out_xml = next_version_path(out_dir, base_name)
        out_xml.write_text(tuned_text)
        validate_xml(out_xml)

        build_status, build_tail = "SKIPPED", ""
        if not args.no_build:
            ok, build_tail = verify_build(PPRZ_HOME, fl.aircraft_name, fl.airframe_rel,
                                          out_xml, args.target)
            build_status = "PASS" if ok else "FAIL"

        report_path = out_xml.with_suffix(".report.txt")
        report = write_report(report_path, fl, analysis, changes, advice, out_xml, base_xml,
                              args.turn_radius, build_status, build_tail)
        print(report)
        print(f"\nreport saved: {report_path}")

        if build_status == "FAIL":
            raise AutotuneError(f"generated airframe failed compilation; see {report_path}")
        return 0
    except AutotuneError as exc:
        log.error("%s", exc)
        return 1


if __name__ == "__main__":
    sys.exit(main())
