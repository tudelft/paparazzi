#!/usr/bin/env python3
"""Fly the EARcam loud-spot mission in NPS against CATIA with a virtual loudspeaker.

Starts server, link, CATIA (--local --earcam-sim) and the NPS simulator of a
generated aircraft, launches it, jumps to the find_loudspot block and follows
the flight plan until the Drop block has been flown. Reports block timing,
DC_SHOT count and the position error between the loudspeaker and the DROP
waypoint returned by CATIA.

Example:
  python3 sw/simulator/nps/nps_earcam_mission.py --aircraft EasystarEar --ac-id 235 \
      --speaker 48.81050,7.85160 --time-factor 4
"""

import argparse
import csv
import math
import os
from pathlib import Path
import random
import subprocess
import sys
import threading
import time

sys.path.insert(0, str(Path(__file__).resolve().parent))
from nps_fixedwing_tuning import (  # noqa: E402
    PAPARAZZI_HOME,
    PAPARAZZI_SRC,
    flight_plan_data,
    send_setting,
    setting_indexes,
    start_process,
    stop_process,
)
from pprzlink.ivy import IvyMessagesInterface  # noqa: E402
from pprzlink.message import PprzMessage  # noqa: E402

CATIA_DIR = PAPARAZZI_HOME / "sw/airborne/modules/digital_cam/catia"
LOCAL_SIM_DEVICE = Path("/tmp/catia-sim")
DEFAULT_SURFACE_GRID = PAPARAZZI_HOME / "data/terrain/imav2026_m4_ign_lidar_hd_10m.csv"


def flight_plan_ground_alt(flight_plan_path):
    import xml.etree.ElementTree as ET
    root = ET.parse(flight_plan_path).getroot()
    plan = root if root.tag == "flight_plan" else root.find("flight_plan")
    return float(plan.get("ground_alt"))


def geofence_polygon(flight_plan_path):
    """Corners (lat, lon) of the flight plan's geofence_sector from the generated flight_plan.xml."""
    import xml.etree.ElementTree as ET
    root = ET.parse(flight_plan_path).getroot()
    plan = root if root.tag == "flight_plan" else root.find("flight_plan")
    name = plan.get("geofence_sector")
    if not name:
        return None
    waypoints = {wp.get("name"): wp for wp in plan.iter("waypoint")}
    for sector in plan.iter("sector"):
        if sector.get("name") == name:
            corners = []
            for corner in sector.findall("corner"):
                wp = waypoints[corner.get("name")]
                corners.append((float(wp.get("lat")), float(wp.get("lon"))))
            return corners
    return None


def fence_report(polygon, track, block_names):
    """Print the closest approach of the track to the geofence and every fix outside it."""
    lat0 = sum(p[0] for p in polygon) / len(polygon)
    m_lat = 111320.0
    m_lon = 111320.0 * math.cos(math.radians(lat0))
    pts = [((lon - polygon[0][1]) * m_lon, (lat - polygon[0][0]) * m_lat) for lat, lon in polygon]

    def inside(x, y):
        c = False
        j = len(pts) - 1
        for i in range(len(pts)):
            xi, yi = pts[i]
            xj, yj = pts[j]
            if (yi > y) != (yj > y) and x < (xj - xi) * (y - yi) / (yj - yi) + xi:
                c = not c
            j = i
        return c

    def edge_distance(x, y):
        best = None
        for i in range(len(pts)):
            ax, ay = pts[i]
            bx, by = pts[(i + 1) % len(pts)]
            dx, dy = bx - ax, by - ay
            t = max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / (dx * dx + dy * dy)))
            d = math.hypot(x - ax - t * dx, y - ay - t * dy)
            best = d if best is None or d < best else best
        return best

    closest = None
    outside = []
    for stamp, lat, lon, alt, block in track:
        x, y = (lon - polygon[0][1]) * m_lon, (lat - polygon[0][0]) * m_lat
        d = edge_distance(x, y)
        name = block_names.get(block, str(block))
        if not inside(x, y):
            outside.append((stamp, name, d))
        elif closest is None or d < closest[0]:
            closest = (d, stamp, name)
    if closest is not None:
        print(f"geofence_min_distance_m={closest[0]:.0f} (t={closest[1]:.0f} s, {closest[2]})")
    if outside:
        print(f"GEOFENCE BREACH: {len(outside)} fixes outside, first t={outside[0][0]:.0f} s in {outside[0][1]}, "
              f"up to {max(o[2] for o in outside):.0f} m beyond the edge")
    return len(outside)


class SurfaceGrid:
    """Terrain (MNT) and surface = terrain + trees/buildings (MNS) on a regular local grid,
    as written by ign_lidar_grid.py. NPS flies over a flat world at the take-off ground, so
    the clearance of a fix is (height above that ground) - (surface height above the field)."""

    def __init__(self, path):
        rows = []
        with path.open() as handle:
            for row in csv.DictReader(handle):
                if row["mnt_m"] in ("", "None") or row["mns_m"] in ("", "None"):
                    continue
                rows.append((float(row["x_east_m"]), float(row["y_north_m"]), float(row["lat"]),
                             float(row["lon"]), float(row["mnt_m"]), float(row["mns_m"])))
        if not rows:
            raise RuntimeError(f"empty surface grid {path}")
        centre = min(rows, key=lambda r: abs(r[0]) + abs(r[1]))
        self.lat0, self.lon0, self.field_m = centre[2], centre[3], centre[4]
        xs = sorted({r[0] for r in rows})
        self.step = min(b - a for a, b in zip(xs, xs[1:]) if b > a)
        north = max(rows, key=lambda r: r[1])
        east = max(rows, key=lambda r: r[0])
        self.m_per_deg_lat = north[1] / (north[2] - self.lat0)
        self.m_per_deg_lon = east[0] / (east[3] - self.lon0)
        self.cells = {(round(r[0] / self.step), round(r[1] / self.step)): (r[4], r[5]) for r in rows}

    def local_xy(self, lat, lon):
        return (lon - self.lon0) * self.m_per_deg_lon, (lat - self.lat0) * self.m_per_deg_lat

    def surface_above_field(self, lat, lon):
        """Highest surface (m above the field at the centre) of the 4 cells around the point; None off-grid."""
        x, y = self.local_xy(lat, lon)
        tops = []
        for i in (math.floor(x / self.step), math.ceil(x / self.step)):
            for j in (math.floor(y / self.step), math.ceil(y / self.step)):
                cell = self.cells.get((int(i), int(j)))
                if cell is not None:
                    tops.append(cell[1] - self.field_m)
        return max(tops) if tops else None


def clearance_report(grid, track, ground, block_names):
    """Print the minimum clearance above the LiDAR surface per block and every fix below 5 m."""
    per_block = {}
    worst = None
    off_grid = 0
    hits = []
    for stamp, lat, lon, alt, block in track:
        top = grid.surface_above_field(lat, lon)
        if top is None:
            off_grid += 1
            continue
        clearance = (alt - ground) - top
        x, y = grid.local_xy(lat, lon)
        name = block_names.get(block, str(block))
        entry = per_block.get(name)
        if entry is None or clearance < entry[0]:
            per_block[name] = (clearance, top, x, y, stamp)
        if worst is None or clearance < worst[0]:
            worst = (clearance, top, x, y, stamp, name)
        if clearance < 5.0 and top > 2.0:
            hits.append((stamp, name, x, y, alt - ground, top, clearance))
    print(f"surface_grid={grid.step:.0f} m cells, field {grid.field_m:.1f} m MSL, {off_grid} fixes off-grid")
    for name, (clearance, top, x, y, stamp) in per_block.items():
        print(f"  clearance {name:<18s} min {clearance:6.1f} m  (surface +{top:4.1f} m at x={x:+5.0f} y={y:+5.0f}, t={stamp:.0f} s)")
    if worst is not None:
        print(f"min_surface_clearance_m={worst[0]:.1f} block={worst[5]} at x={worst[2]:+.0f} y={worst[3]:+.0f}")
    if hits:
        print(f"OBSTACLE WARNING: {len(hits)} fixes less than 5 m above a surface higher than 2 m:")
        for stamp, name, x, y, height, top, clearance in hits[:12]:
            print(f"  t={stamp:6.1f} {name:<16s} x={x:+5.0f} y={y:+5.0f} height {height:5.1f} m, surface +{top:4.1f} m, clearance {clearance:5.1f} m")
    return len(hits)


def jump_to_block(interface, aircraft_id, block_id):
    # Datalink BLOCK is forwarded by link directly; ground JUMP_TO_BLOCK needs the server.
    message = PprzMessage("datalink", "BLOCK")
    message["ac_id"] = aircraft_id
    message["block_id"] = block_id
    interface.send(message)


def utm_to_latlon(easting, northing, zone, northern=True):
    k0 = 0.9996
    a = 6378137.0
    e2 = 0.00669438
    e1 = (1.0 - math.sqrt(1.0 - e2)) / (1.0 + math.sqrt(1.0 - e2))
    x = easting - 500000.0
    y = northing if northern else northing - 10000000.0
    lon_origin = math.radians((zone - 1) * 6 - 180 + 3)
    ep2 = e2 / (1.0 - e2)
    m = y / k0
    mu = m / (a * (1.0 - e2 / 4.0 - 3.0 * e2 * e2 / 64.0 - 5.0 * e2 ** 3 / 256.0))
    phi1 = (mu + (3.0 * e1 / 2.0 - 27.0 * e1 ** 3 / 32.0) * math.sin(2.0 * mu)
            + (21.0 * e1 ** 2 / 16.0 - 55.0 * e1 ** 4 / 32.0) * math.sin(4.0 * mu)
            + (151.0 * e1 ** 3 / 96.0) * math.sin(6.0 * mu))
    sin1 = math.sin(phi1)
    cos1 = math.cos(phi1)
    n1 = a / math.sqrt(1.0 - e2 * sin1 * sin1)
    t1 = math.tan(phi1) ** 2
    c1 = ep2 * cos1 * cos1
    r1 = a * (1.0 - e2) / (1.0 - e2 * sin1 * sin1) ** 1.5
    d = x / (n1 * k0)
    lat = phi1 - (n1 * math.tan(phi1) / r1) * (
        d * d / 2.0
        - (5.0 + 3.0 * t1 + 10.0 * c1 - 4.0 * c1 * c1 - 9.0 * ep2) * d ** 4 / 24.0
        + (61.0 + 90.0 * t1 + 298.0 * c1 + 45.0 * t1 * t1 - 252.0 * ep2 - 3.0 * c1 * c1) * d ** 6 / 720.0)
    lon = lon_origin + (
        d - (1.0 + 2.0 * t1 + c1) * d ** 3 / 6.0
        + (5.0 - 2.0 * c1 + 28.0 * t1 - 3.0 * c1 * c1 + 8.0 * ep2 + 24.0 * t1 * t1) * d ** 5 / 120.0) / cos1
    return math.degrees(lat), math.degrees(lon)


def ground_distance_m(lat_a, lon_a, lat_b, lon_b):
    north = math.radians(lat_b - lat_a) * 6378137.0
    east = math.radians(lon_b - lon_a) * 6378137.0 * math.cos(math.radians(lat_a))
    return math.hypot(north, east)


class MissionMonitor:
    def __init__(self, aircraft_id, time_factor, drop_wp_id, final_block=None, release_block=None, impact_wp_id=None):
        self.aircraft_id = aircraft_id
        self.time_factor = time_factor
        self.drop_wp_id = drop_wp_id
        self.impact_wp_id = impact_wp_id
        self.impact_wp = None
        self.waypoints = {}
        self.final_block = final_block
        self.release_block = release_block
        self.lock = threading.Lock()
        self.started_at = time.monotonic()
        self.block = None
        self.block_history = []
        self.altitude = None
        self.shots = 0
        self.drop_wp = None
        self.drop_wp_updates = 0
        self.final_phase = False
        self.closest_to_drop_m = None
        self.last_gps = None
        self.release_gps = None
        self.min_final_altitude = None
        self.track = []   # (sim_time_s, east, north, alt_m, zone, block) at GPS rate
        self.wind_est = None   # last WIND_INFO_RET from the aircraft (east, north, airspeed)
        self.wind_truth = None   # last NPS_WIND from the FDM (north, east, down)

    def nps_wind(self, aircraft_id, message):
        if int(aircraft_id) != self.aircraft_id:
            return
        with self.lock:
            self.wind_truth = (float(message["vx"]), float(message["vy"]), float(message["vz"]))

    def wind_info(self, aircraft_id, message):
        if int(aircraft_id) != self.aircraft_id:
            return
        if int(message["flags"]) & 1:
            with self.lock:
                self.wind_est = (float(message["east"]), float(message["north"]), float(message["airspeed"]))

    def gps(self, aircraft_id, message):
        if int(aircraft_id) != self.aircraft_id:
            return
        with self.lock:
            east = float(message["utm_east"]) / 100.0
            north = float(message["utm_north"]) / 100.0
            self.last_gps = (east, north, float(message["alt"]) / 1000.0,
                             float(message["speed"]) / 100.0, math.radians(float(message["course"]) / 10.0),
                             int(message["utm_zone"]))
            self.track.append((self.sim_time(), east, north, self.last_gps[2], self.last_gps[5], self.block))
            if not self.final_phase or self.drop_wp is None:
                return
            distance = math.hypot(east - self.drop_wp[0], north - self.drop_wp[1])
            if self.closest_to_drop_m is None or distance < self.closest_to_drop_m:
                self.closest_to_drop_m = distance

    def sim_time(self):
        return (time.monotonic() - self.started_at) * self.time_factor

    def navigation(self, aircraft_id, message):
        if int(aircraft_id) != self.aircraft_id:
            return
        block = int(message["cur_block"])
        with self.lock:
            if block != self.block:
                self.block = block
                self.block_history.append((self.sim_time(), block))
                if self.release_block is not None and block == self.release_block and self.release_gps is None:
                    self.release_gps = self.last_gps
            if self.final_block is not None and block >= self.final_block:
                self.final_phase = True

    def position(self, aircraft_id, message):
        if int(aircraft_id) != self.aircraft_id:
            return
        with self.lock:
            self.altitude = -float(message["ltpp_z"])
            if self.final_phase and (self.min_final_altitude is None or self.altitude < self.min_final_altitude):
                self.min_final_altitude = self.altitude

    def dc_shot(self, aircraft_id, message):
        if int(aircraft_id) != self.aircraft_id:
            return
        with self.lock:
            self.shots += 1

    def wp_moved(self, aircraft_id, message):
        if int(aircraft_id) != self.aircraft_id:
            return
        wp_id = int(message["wp_id"])
        with self.lock:
            self.waypoints[wp_id] = (float(message["utm_east"]), float(message["utm_north"]), int(message["utm_zone"]))
            if wp_id == self.impact_wp_id:
                self.impact_wp = (float(message["utm_east"]), float(message["utm_north"]), int(message["utm_zone"]))
            if wp_id != self.drop_wp_id:
                return
            self.drop_wp = (float(message["utm_east"]), float(message["utm_north"]),
                            float(message["alt"]), int(message["utm_zone"]))
            self.drop_wp_updates += 1

    def snapshot(self):
        with self.lock:
            return {
                "block": self.block,
                "history": list(self.block_history),
                "altitude": self.altitude,
                "shots": self.shots,
                "drop_wp": self.drop_wp,
                "drop_wp_updates": self.drop_wp_updates,
                "closest_to_drop_m": self.closest_to_drop_m,
                "release_gps": self.release_gps,
                "min_final_altitude": self.min_final_altitude,
                "impact_wp": self.impact_wp,
                "waypoints": dict(self.waypoints),
                "track": list(self.track),
                "wind_est": self.wind_est,
                "wind_truth": self.wind_truth,
            }


def wait_until(predicate, monitor, timeout_sim_s, description):
    deadline = time.monotonic() + timeout_sim_s / monitor.time_factor
    while time.monotonic() < deadline:
        state = monitor.snapshot()
        if predicate(state):
            return state
        time.sleep(0.1)
    state = monitor.snapshot()
    history = " -> ".join(str(block) for _, block in state["history"])
    raise RuntimeError(f"Timed out waiting for {description} (block {state['block']}, altitude {state['altitude']}, history {history})")


def wait_for_path(path, timeout_s):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if path.exists():
            return
        time.sleep(0.1)
    raise RuntimeError(f"{path} did not appear")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--aircraft", required=True)
    parser.add_argument("--ac-id", required=True, type=int)
    parser.add_argument("--speaker", required=True, help="virtual loudspeaker LAT,LON[,DB_AT_1M]")
    parser.add_argument("--speaker-radius", type=float, default=0.0,
                        help="place the speaker at a random point within this many metres of --speaker "
                             "(rulebook: mannequins within 25 m of the given point); 0 = exactly there")
    parser.add_argument("--seed", type=int, help="random seed for --speaker-radius (default: time based)")
    parser.add_argument("--wind", default="0,0",
                        help="SPEED_MPS,FROM_DEG steady wind in the simulation (meteorological direction)")
    parser.add_argument("--surface-grid", type=Path, default=DEFAULT_SURFACE_GRID,
                        help="LiDAR terrain/surface grid CSV (ign_lidar_grid.py) for the obstacle clearance check")
    parser.add_argument("--time-factor", type=float, default=4.0)
    parser.add_argument("--mission-timeout", type=float, default=1800.0, help="simulated seconds")
    parser.add_argument("--bus", default="127.255.255.255:2010")
    parser.add_argument("--output-dir", type=Path, default=Path("var/nps_earcam"))
    parser.add_argument("--start-block", default="find_loudspot", help="mission entry block")
    parser.add_argument("--release-block", default="drop_climbout",
                        help="block entered at the release command (Mission 4 plan)")
    args = parser.parse_args()

    speaker = [float(value) for value in args.speaker.split(",")]
    if len(speaker) < 2:
        raise RuntimeError("--speaker expects LAT,LON[,DB_AT_1M]")
    circle_centre = (speaker[0], speaker[1])
    if args.speaker_radius > 0.0:
        # Uniform over the disc: sqrt on the radius, so the centre is not favoured.
        seed = args.seed if args.seed is not None else int(time.time())
        generator = random.Random(seed)
        distance = args.speaker_radius * math.sqrt(generator.random())
        bearing = generator.random() * 2.0 * math.pi
        speaker[0] += distance * math.cos(bearing) / 111320.0
        speaker[1] += distance * math.sin(bearing) / (111320.0 * math.cos(math.radians(circle_centre[0])))
        print(f"speaker placed {distance:.1f} m at {math.degrees(bearing):.0f} deg from the given point (seed {seed}): "
              f"{speaker[0]:.7f},{speaker[1]:.7f}")
    speaker_arg = ",".join(f"{value:.7f}" if index < 2 else str(value) for index, value in enumerate(speaker))
    wind = [float(value) for value in args.wind.split(",")]
    if len(wind) != 2:
        raise RuntimeError("--wind expects SPEED_MPS,FROM_DEG")

    aircraft_dir = PAPARAZZI_HOME / "var/aircrafts" / args.aircraft
    simulator = aircraft_dir / "nps/simsitl"
    settings_header = aircraft_dir / "nps/generated/settings.h"
    flight_plan_header = aircraft_dir / "nps/generated/flight_plan.h"
    flight_plan_path = aircraft_dir / "flight_plan.xml"
    catia = CATIA_DIR / "catia"
    for required in (simulator, settings_header, flight_plan_header, flight_plan_path, catia):
        if not required.exists():
            raise RuntimeError(f"Missing artifact: {required}")

    settings = setting_indexes(settings_header)
    flight_plan = flight_plan_data(flight_plan_path)
    blocks = flight_plan["blocks"]
    block_names = {index: name for name, index in blocks.items()}
    for name in ("Takeoff", "Standby", args.start_block, "goto_loudspot"):
        if name not in blocks:
            raise RuntimeError(f"Flight plan must provide block '{name}'")
    release_block = blocks.get(args.release_block)
    drop_wp_id = None
    impact_wp_id = None
    wp_ids = {}
    for line in flight_plan_header.read_text().splitlines():
        if line.startswith("#define WP_"):
            parts = line.split()
            if len(parts) >= 3 and parts[2].isdigit():
                wp_ids[parts[1][3:]] = int(parts[2])
        if line.startswith("#define WP_DROP "):
            drop_wp_id = int(line.split()[2])
        if line.startswith("#define WP__IMPACT "):
            impact_wp_id = int(line.split()[2])
    if drop_wp_id is None:
        raise RuntimeError("Flight plan must provide waypoint DROP")

    args.output_dir.mkdir(parents=True, exist_ok=True)
    prefix = args.output_dir / f"{args.aircraft}_earcam_mission"
    environment = os.environ.copy()
    environment["PAPARAZZI_HOME"] = str(PAPARAZZI_HOME)
    environment["PAPARAZZI_SRC"] = str(PAPARAZZI_SRC)

    processes = []
    logs = []
    interface = None
    monitor = MissionMonitor(args.ac_id, args.time_factor, drop_wp_id, blocks["goto_loudspot"], release_block, impact_wp_id)
    result = 1
    try:
        ground_tools = [
            ("link", [str(PAPARAZZI_HOME / "sw/ground_segment/tmtc/link"), "-udp", "-udp_broadcast", "-b", args.bus]),
        ]
        if wind[0] > 0.0:
            # GAIA answers the WORLD_ENV_REQ of NPS; it is a Qt app, so run it without a display.
            environment.setdefault("QT_QPA_PLATFORM", "offscreen")
            ground_tools.append(("gaia", [str(PAPARAZZI_HOME / "sw/simulator/gaia"), "-b", args.bus,
                                          "-t", str(args.time_factor), "-w", str(wind[0]), "-d", str(wind[1])]))
        for name, command in ground_tools:
            process, log = start_process(command, environment, prefix.with_suffix(f".{name}.log"))
            processes.append(process)
            logs.append(log)

        catia_log_path = prefix.with_suffix(".catia.log")
        catia_log = catia_log_path.open("w")
        catia_process = subprocess.Popen(
            [str(catia), "--local", "--earcam-sim", speaker_arg, "--debug"],
            cwd=CATIA_DIR,
            env=environment,
            stdout=catia_log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        processes.append(catia_process)
        logs.append(catia_log)
        wait_for_path(LOCAL_SIM_DEVICE, 10.0)
        time.sleep(1.0)

        interface = IvyMessagesInterface("nps_earcam_mission", ivy_bus=args.bus)
        interface.subscribe(monitor.navigation, PprzMessage("telemetry", "NAVIGATION"))
        interface.subscribe(monitor.position, PprzMessage("telemetry", "NPS_SPEED_POS"))
        interface.subscribe(monitor.dc_shot, PprzMessage("telemetry", "DC_SHOT"))
        interface.subscribe(monitor.wp_moved, PprzMessage("telemetry", "WP_MOVED"))
        interface.subscribe(monitor.gps, PprzMessage("telemetry", "GPS"))
        interface.subscribe(monitor.wind_info, PprzMessage("telemetry", "WIND_INFO_RET"))
        interface.subscribe(monitor.nps_wind, PprzMessage("telemetry", "NPS_WIND"))

        sim_process, sim_log = start_process(
            [str(simulator), "--rc_script", "0", "--time_factor", str(args.time_factor), "--ivy_bus", args.bus],
            environment,
            prefix.with_suffix(".simulator.log"),
        )
        processes.append(sim_process)
        logs.append(sim_log)

        wait_until(lambda s: s["altitude"] is not None and s["block"] is not None, monitor, 60.0, "telemetry")
        # Geo init runs 10 s after GPS fix and then Holding point kills the throttle; command after that.
        wait_until(lambda s: s["block"] == blocks["Holding point"], monitor, 90.0, "Holding point")
        time.sleep(2.0 / args.time_factor)
        ground = monitor.snapshot()["altitude"]

        send_setting(interface, args.ac_id, settings["flight_altitude"], flight_plan["altitude"])
        if wind[0] > 0.0:
            print(f"wind set via gaia: {wind[0]:.1f} m/s from {wind[1]:.0f} deg")
        send_setting(interface, args.ac_id, settings["autopilot.mode"], 2)
        send_setting(interface, args.ac_id, settings["autopilot.kill_throttle"], 0)
        jump_to_block(interface, args.ac_id, blocks["Takeoff"])
        time.sleep(0.5)
        send_setting(interface, args.ac_id, settings["autopilot.launch"], 1)
        wait_until(lambda s: s["altitude"] >= ground + 20.0, monitor, 180.0, "takeoff")
        wait_until(lambda s: s["block"] == blocks["Standby"], monitor, 180.0, "Standby after climb")
        time.sleep(20.0 / args.time_factor)

        mission_start = monitor.sim_time()
        jump_to_block(interface, args.ac_id, blocks[args.start_block])
        wait_until(lambda s: s["block"] is not None and s["block"] >= blocks[args.start_block],
                   monitor, 30.0, args.start_block)

        def drop_done(state):
            # Blocks can be transient (one nav cycle), so key on the final phase having
            # started and the plan being back in Standby.
            seen_final = any(block >= blocks["goto_loudspot"] for _, block in state["history"])
            return seen_final and state["block"] == blocks["Standby"]

        wait_until(drop_done, monitor, args.mission_timeout, "Drop block to finish")
        mission_end = monitor.sim_time()
        state = monitor.snapshot()

        print("block sequence (sim time s):")
        for stamp, block in state["history"]:
            if stamp >= mission_start - 1.0:
                print(f"  {stamp - mission_start:7.1f}  {block_names.get(block, block)}")
        print(f"mission_duration_s={mission_end - mission_start:.1f}")
        if state["wind_truth"] is not None:
            t_north, t_east, t_down = state["wind_truth"]
            print(f"wind_fdm_mps={math.hypot(t_east, t_north):.1f} "
                  f"wind_fdm_from_deg={(math.degrees(math.atan2(-t_east, -t_north)) + 360.0) % 360.0:.0f}")
        if state["wind_est"] is not None:
            w_east, w_north, w_airspeed = state["wind_est"]
            est_speed = math.hypot(w_east, w_north)
            est_from = (math.degrees(math.atan2(-w_east, -w_north)) + 360.0) % 360.0
            true_east = -wind[0] * math.sin(math.radians(wind[1]))
            true_north = -wind[0] * math.cos(math.radians(wind[1]))
            vec_error = math.hypot(w_east - true_east, w_north - true_north)
            print(f"wind_onboard_mps={est_speed:.1f} wind_onboard_from_deg={est_from:.0f} "
                  f"fit_airspeed_mps={w_airspeed:.1f} wind_vector_error_mps={vec_error:.2f}")
        else:
            print("wind_onboard: no estimate received")
        print(f"dc_shot_count={state['shots']}")
        print(f"drop_wp_updates={state['drop_wp_updates']}")
        if state["closest_to_drop_m"] is not None:
            print(f"closest_approach_to_drop_m={state['closest_to_drop_m']:.1f}")
        if state["drop_wp"] is None:
            print("DROP waypoint was never moved")
        else:
            east, north, alt, zone = state["drop_wp"]
            lat, lon = utm_to_latlon(east, north, zone, northern=speaker[0] >= 0.0)
            error = ground_distance_m(speaker[0], speaker[1], lat, lon)
            print(f"speaker_lat={speaker[0]:.7f} speaker_lon={speaker[1]:.7f}")
            print(f"drop_lat={lat:.7f} drop_lon={lon:.7f} drop_alt={alt:.1f}")
            print(f"position_error_m={error:.2f}")
            result = 0 if error <= 10.0 else 1
            if state["release_gps"] is not None:
                # Release height and speed from the GPS report nearest the release command.
                r_east, r_north, r_alt, r_speed, r_course, r_zone = state["release_gps"]
                height = r_alt - alt
                print(f"release_height_m={height:.2f} release_speed_mps={r_speed:.1f} (GPS at 2 Hz)")
                if height > 2.0:
                    print("RULE: release above 2 m, no points")
                    result = 1
            if state["impact_wp"] is not None and state["release_gps"] is not None:
                # nav_drop's expected impact (aircraft position at the release command plus
                # hatch delay travel and the target-release offset) against the speaker.
                i_east, i_north, i_zone = state["impact_wp"]
                i_lat, i_lon = utm_to_latlon(i_east, i_north, i_zone, northern=speaker[0] >= 0.0)
                impact_error = ground_distance_m(speaker[0], speaker[1], i_lat, i_lon)
                print(f"impact_error_m={impact_error:.2f}")
                start_wp = state["waypoints"].get(wp_ids.get("START"))
                if start_wp is not None:
                    # Along-track (+ long) and cross-track (+ left) parts of impact - DROP.
                    dir_e, dir_n = east - start_wp[0], north - start_wp[1]
                    run = math.hypot(dir_e, dir_n) or 1.0
                    dir_e, dir_n = dir_e / run, dir_n / run
                    off_e, off_n = i_east - east, i_north - north
                    along = off_e * dir_e + off_n * dir_n
                    cross = -off_e * dir_n + off_n * dir_e
                    print(f"impact_vs_drop_along_m={along:.2f} impact_vs_drop_cross_m={cross:.2f}")
            elif release_block is not None:
                print("no release (hatch not commanded)")
                result = 1
            if state["min_final_altitude"] is not None:
                print(f"min_altitude_final_phase_m={state['min_final_altitude'] - ground:.2f} (above takeoff ground)")
        catia_log.flush()
        heatmap = None
        for line in catia_log_path.read_text(errors="replace").splitlines():
            if "EAR_RESULT" in line or "loudest" in line.lower() or "Photo take" in line or ".jpg" in line:
                print(f"catia: {line.strip()}")
            for word in line.split():
                if word.endswith(".jpg") and "/e" in word:
                    heatmap = Path(word.strip("'\","))

        # Full aircraft track (GPS rate) of the mission, then the heatmap over satellite
        # imagery with samples and track: <prefix>.track.csv and photos/eNNNNNN_sat.jpg.
        track_path = prefix.with_suffix(".track.csv")
        track_rows = []
        with track_path.open("w") as track_file:
            track_file.write("t_s,lat_deg,lon_deg,alt_m,block\n")
            for stamp, east, north, alt, zone, block in state["track"]:
                if stamp < mission_start - 1.0:
                    continue
                lat, lon = utm_to_latlon(east, north, zone, northern=speaker[0] >= 0.0)
                track_rows.append((stamp - mission_start, lat, lon, alt, block))
                track_file.write(f"{stamp - mission_start:.2f},{lat:.7f},{lon:.7f},{alt:.1f},{block_names.get(block, block)}\n")
        print(f"track={track_path}")
        surface_grid = None
        if args.surface_grid and args.surface_grid.exists():
            # Trees and buildings from the LiDAR surface model against the flat NPS world
            # (GPS altitudes in the track are above MSL, the NPS ground is the plan's ground_alt).
            surface_grid = SurfaceGrid(args.surface_grid)
            if clearance_report(surface_grid, track_rows, flight_plan_ground_alt(flight_plan_path), block_names) > 0:
                result = 1
        fence = geofence_polygon(flight_plan_path)
        if fence:
            if fence_report(fence, track_rows, block_names) > 0:
                result = 1
        if "HOME" in blocks and any(block == blocks["HOME"] for _, block in state["history"]):
            print("GEOFENCE/HOME: the autopilot entered HOME mode during the mission")
            result = 1
        if heatmap is not None and heatmap.exists() and heatmap.with_suffix(".geo").exists():
            ear_logs = sorted((CATIA_DIR / "earlogs").glob("ear_*.csv"))
            overlay = [sys.executable, str(CATIA_DIR / "ear_heatmap_overlay.py"), str(heatmap),
                       "--track", str(track_path), "--truth", f"{speaker[0]:.7f},{speaker[1]:.7f}",
                       "--search", f"{circle_centre[0]:.7f},{circle_centre[1]:.7f},{max(args.speaker_radius, 25.0):.0f}",
                       "--quiet"]
            if surface_grid is not None:
                overlay += ["--obstacles", str(args.surface_grid)]
            if ear_logs:
                overlay += ["--log", str(ear_logs[-1])]
            completed = subprocess.run(overlay, capture_output=True, text=True)
            print(f"overlay: {(completed.stdout + completed.stderr).strip()}")
    finally:
        for process in reversed(processes):
            stop_process(process)
        if interface is not None:
            interface.shutdown()
        for log_file in logs:
            log_file.close()
    return result


if __name__ == "__main__":
    sys.exit(main())
