#!/usr/bin/env python3

import argparse
import csv
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time
import re
import xml.etree.ElementTree as ET


PAPARAZZI_HOME = Path(os.environ.get("PAPARAZZI_HOME", Path(__file__).resolve().parents[3]))
PAPARAZZI_SRC = Path(os.environ.get("PAPARAZZI_SRC", PAPARAZZI_HOME))
PPRZLINK_PYTHON = PAPARAZZI_SRC / "sw/ext/pprzlink/lib/v2.0/python/src"
sys.path.insert(0, str(PPRZLINK_PYTHON))

from pprzlink.ivy import IvyMessagesInterface
from pprzlink.message import PprzMessage

LANDING_FIELDS = ("flight_time", "landing_agl", "landing_sink_rate", "remaining_m", "cross_track_m",
                  "predicted_error_m", "predicted_cross_track_m", "brake_fraction", "agl_fresh",
                  "landing_abort", "commit_flare")


class FlightRecorder:
    def __init__(self, aircraft_id, time_factor):
        self.aircraft_id = aircraft_id
        self.time_factor = time_factor
        self.lock = threading.Lock()
        self.started_at = time.monotonic()
        self.latest_position = None
        self.latest_attitude = None
        self.latest_control = {}
        self.samples = []

    def position(self, aircraft_id, message):
        if int(aircraft_id) != self.aircraft_id:
            return
        with self.lock:
            self.latest_position = {
                "time": (time.monotonic() - self.started_at) * self.time_factor,
                "north": float(message["ltpp_x"]),
                "east": float(message["ltpp_y"]),
                "down": float(message["ltpp_z"]),
                "north_speed": float(message["ltpp_xd"]),
                "east_speed": float(message["ltpp_yd"]),
                "down_speed": float(message["ltpp_zd"]),
            }
            self._record()

    def attitude(self, aircraft_id, message):
        if int(aircraft_id) != self.aircraft_id:
            return
        with self.lock:
            self.latest_attitude = {
                "roll_rate": float(message["p"]),
                "pitch_rate": float(message["q"]),
                "yaw_rate": float(message["r"]),
                "roll": float(message["phi"]),
                "pitch": float(message["theta"]),
                "yaw": float(message["psi"]),
            }

    def control(self, aircraft_id, message):
        if int(aircraft_id) != self.aircraft_id:
            return
        with self.lock:
            if message.name == "ENERGY":
                self.latest_control["throttle"] = float(message["throttle"])
            elif message.name == "NAVIGATION":
                self.latest_control["nav_block"] = int(message["cur_block"])
                self.latest_control["nav_stage"] = int(message["cur_stage"])
            elif message.name == "SONAR":
                self.latest_control["agl"] = float(message["sonar_distance"])
            elif message.name == "DEBUG_VECT" and message["name"].strip('"') == "precision_landing":
                values = message["vector"]
                if len(values) == len(LANDING_FIELDS):
                    self.latest_control.update(zip(LANDING_FIELDS, map(float, values)))
            elif message.name == "DESIRED":
                self.latest_control.update({
                    "desired_roll": math.degrees(float(message["roll"])),
                    "desired_pitch": math.degrees(float(message["pitch"])),
                    "desired_altitude": float(message["altitude"]),
                    "desired_climb": float(message["climb"]),
                    "desired_airspeed": float(message["airspeed"]),
                })
            elif message.name == "AIRSPEED":
                self.latest_control.update({
                    "measured_airspeed": float(message["airspeed"]),
                    "airspeed_setpoint": float(message["airspeed_sp"]),
                })
            elif message.name == "COMMANDS":
                values = message["values"]
                if len(values) >= 4:
                    self.latest_control.update({
                        "command_throttle": float(values[0]),
                        "command_roll": float(values[1]),
                        "command_pitch": float(values[2]),
                        "command_yaw": float(values[3]),
                    })
                if len(values) >= 5:
                    self.latest_control["command_brake"] = float(values[4])
            elif message.name == "H_CTL_A":
                self.latest_control.update({
                    "loop_roll_setpoint": math.degrees(float(message["roll_sp"])),
                    "loop_roll_reference": math.degrees(float(message["roll_ref"])),
                    "loop_roll_measured": math.degrees(float(message["phi"])),
                    "loop_roll_error_sum": float(message["roll_sum_err"]),
                    "aileron_setpoint": float(message["aileron_sp"]),
                    "loop_pitch_setpoint": math.degrees(float(message["pitch_sp"])),
                    "loop_pitch_reference": math.degrees(float(message["pitch_ref"])),
                    "loop_pitch_measured": math.degrees(float(message["theta"])),
                    "loop_pitch_error_sum": float(message["pitch_sum_err"]),
                    "elevator_setpoint": float(message["elevator_sp"]),
                })

    def _record(self):
        if self.latest_position is None or self.latest_attitude is None:
            return
        sample = dict(self.latest_position)
        sample.update(self.latest_attitude)
        sample.update(self.latest_control)
        sample["altitude"] = -sample["down"]
        sample["airspeed"] = math.sqrt(
            sample["north_speed"] ** 2
            + sample["east_speed"] ** 2
            + sample["down_speed"] ** 2
        )
        self.samples.append(sample)

    def snapshot(self):
        with self.lock:
            return list(self.samples)


def setting_indexes(settings_header_path):
    source = settings_header_path.read_text()
    match = re.search(r"#define SETTINGS_NAMES \{ \\\n(.*?)\n\};", source, re.DOTALL)
    if match is None:
        raise RuntimeError(f"SETTINGS_NAMES not found in {settings_header_path}")
    names = re.findall(r'\{ "([^"]+)" \}', match.group(1))
    return {name: index for index, name in enumerate(names)}


def flight_plan_data(flight_plan_path, flight_plan_header_path=None):
    indexes = {}
    root = ET.parse(flight_plan_path).getroot()
    flight_plan = root if root.tag == "flight_plan" else root.find("flight_plan")
    blocks = flight_plan.find("blocks") if flight_plan is not None else None
    if blocks is None:
        raise RuntimeError(f"No blocks found in {flight_plan_path}")
    for index, block in enumerate(blocks.findall("block")):
        indexes[block.get("name")] = int(block.get("no", index))
    aliases = {}
    ambiguous = set()
    for name, index in indexes.items():
        short_name = name.rsplit(".", 1)[-1]
        for alias in (short_name, short_name.replace("_", "-")):
            if alias in aliases and aliases[alias] != index:
                ambiguous.add(alias)
            else:
                aliases[alias] = index
    indexes.update({name: index for name, index in aliases.items()
                    if name not in ambiguous and name not in indexes})
    waypoints = {waypoint.get("name"): {axis: float(waypoint.get(axis, "0")) for axis in ("x", "y", "alt")}
                 for waypoint in flight_plan.find("waypoints")}
    if flight_plan_header_path is not None:
        source = flight_plan_header_path.read_text()
        waypoint_ids = {name: int(index) for name, index in
                        re.findall(r"^#define WP_(\S+) (\d+)$", source, re.MULTILINE)}
        match = re.search(r"#define WAYPOINTS_ENU \{ \\\n(.*?)\n\};", source, re.DOTALL)
        if match is None:
            raise RuntimeError(f"WAYPOINTS_ENU not found in {flight_plan_header_path}")
        enu = re.findall(r"\{\s*([-+\d.eE]+),\s*([-+\d.eE]+),\s*[-+\d.eE]+\s*\}", match.group(1))
        for name, waypoint in waypoints.items():
            if name not in waypoint_ids or waypoint_ids[name] >= len(enu):
                raise RuntimeError(f"Waypoint {name!r} absent from {flight_plan_header_path}")
            waypoint["x"], waypoint["y"] = map(float, enu[waypoint_ids[name]])
    sectors = flight_plan.find("sectors")
    landing_sector = None
    if sectors is not None:
        sector = sectors.find("./sector[@name='Takeoff_Landing_zone_Fixedwing']")
        if sector is not None:
            landing_sector = [waypoints[corner.get("name")] for corner in sector.findall("corner")]
    return {
        "blocks": indexes,
        "waypoints": waypoints,
        "landing_sector": landing_sector,
        "altitude": float(flight_plan.get("alt")),
        "ground_altitude": float(flight_plan.get("ground_alt")),
    }


def send_setting(interface, aircraft_id, index, value):
    message = PprzMessage("ground", "DL_SETTING")
    message["ac_id"] = str(aircraft_id)
    message["index"] = index
    message["value"] = value
    interface.send(message)


def parse_setting_overrides(assignments, settings):
    overrides = []
    for assignment in assignments:
        name, value = assignment.split("=", 1)
        if name not in settings:
            raise ValueError(f"Setting {name!r} absent from generated settings; regenerate the aircraft settings header")
        numeric_value = float(value)
        if not math.isfinite(numeric_value):
            raise ValueError(f"Setting {name!r} must be finite")
        overrides.append((settings[name], numeric_value))
    return overrides


def jump_to_block(interface, aircraft_id, block_id):
    message = PprzMessage("ground", "JUMP_TO_BLOCK")
    message["ac_id"] = str(aircraft_id)
    message["block_id"] = block_id
    interface.send(message)


def start_process(command, environment, log_path):
    log_file = log_path.open("w")
    process = subprocess.Popen(
        command,
        cwd=PAPARAZZI_HOME,
        env=environment,
        stdout=log_file,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    return process, log_file


def stop_process(process):
    if process.poll() is not None:
        return
    os.killpg(process.pid, signal.SIGTERM)
    try:
        process.wait(timeout=3)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGKILL)
        process.wait()


def wait_for_samples(recorder, minimum, timeout):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if len(recorder.snapshot()) >= minimum:
            return
        time.sleep(0.1)
    raise RuntimeError("NPS truth telemetry did not arrive")


def wait_for(predicate, recorder, timeout, description):
    deadline = time.monotonic() + timeout
    samples = []
    while time.monotonic() < deadline:
        samples = recorder.snapshot()
        if samples and predicate(samples[-1]):
            return samples[-1]
        time.sleep(0.1)
    observed = ", ".join(sorted(samples[-1])) if samples else "no sample fields"
    raise RuntimeError(f"Timed out waiting for {description}; observed: {observed}")


def altitude_metrics(samples, target, start_time):
    active = [sample for sample in samples if sample["time"] >= start_time]
    if not active:
        raise RuntimeError("No samples recorded during altitude step")
    errors = [sample["altitude"] - target for sample in active]
    capture_time = None
    for index, sample in enumerate(active):
        tail = active[index:]
        if len(tail) >= 20 and all(abs(item["altitude"] - target) <= 1.0 for item in tail[:20]):
            capture_time = sample["time"] - start_time
            break
    return {
        "capture_time": capture_time,
        "overshoot": max(errors),
        "undershoot": min(errors),
        "airspeed_min": min(sample["airspeed"] for sample in active),
        "airspeed_max": max(sample["airspeed"] for sample in active),
    }


def oval_metrics(samples, target_altitude, requested_radius, start_time):
    active = [sample for sample in samples if sample["time"] >= start_time]
    if not active:
        raise RuntimeError("No samples recorded during oval")
    turn_samples = [sample for sample in active if abs(sample["roll"]) >= 15.0]
    straight_samples = [sample for sample in active if abs(sample["roll"]) < 15.0]
    if not turn_samples:
        raise RuntimeError("No banked-turn samples recorded during oval")
    speeds = [sample["airspeed"] for sample in turn_samples]
    radii = []
    for sample in turn_samples:
        bank = math.radians(abs(sample["roll"]))
        if abs(math.tan(bank)) > 1e-3:
            radii.append(sample["airspeed"] ** 2 / (9.80665 * math.tan(bank)))
    turn_errors = [sample["altitude"] - target_altitude for sample in turn_samples]
    straight_errors = [sample["altitude"] - target_altitude for sample in straight_samples]
    return {
        "requested_radius": requested_radius,
        "estimated_radius_mean": sum(radii) / len(radii),
        "estimated_radius_min": min(radii),
        "bank_max": max(abs(sample["roll"]) for sample in turn_samples),
        "turn_dip": min(turn_errors),
        "turn_balloon": max(turn_errors),
        "turn_mean": sum(turn_errors) / len(turn_errors),
        "straight_abs_max": max((abs(error) for error in straight_errors), default=float("nan")),
        "airspeed_min": min(speeds),
        "airspeed_max": max(speeds),
    }


def write_csv(path, samples):
    if not samples:
        return
    fieldnames = []
    for sample in samples:
        for name in sample:
            if name not in fieldnames:
                fieldnames.append(name)
    with path.open("w", newline="") as output:
        writer = csv.DictWriter(output, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(samples)


def read_contacts(path):
    contacts = []
    for line in path.read_text(errors="replace").splitlines():
        if line.startswith("NPS_LANDING_CONTACT "):
            contacts.append(json.loads(line.partition(" ")[2]))
    return contacts


LANDING_PITCH_MIN_DEG = -10.0
LANDING_PITCH_MAX_DEG = 20.0
LANDING_PRECISION_RADIUS_M = 1.0

def point_inside_polygon(east, north, polygon):
    if polygon is None or len(polygon) < 3 or not math.isfinite(east) or not math.isfinite(north):
        return False
    inside = False
    previous = len(polygon) - 1
    for corner, point in enumerate(polygon):
        preceding = polygon[previous]
        north_delta = preceding["y"] - point["y"]
        if (point["y"] > north) != (preceding["y"] > north) and abs(north_delta) > 1e-9:
            boundary_east = (preceding["x"] - point["x"]) * (north - point["y"]) / north_delta + point["x"]
            if east < boundary_east:
                inside = not inside
        previous = corner
    return inside


def belly_contact_indexes(model_path):
    contacts = ET.parse(model_path).getroot().findall("./ground_reactions/contact")
    indexes = tuple(index for index, contact in enumerate(contacts)
                    if contact.get("type") == "STRUCTURE" and contact.get("name", "").startswith("BELLY"))
    if not indexes:
        raise ValueError(f"No named belly contacts in {model_path}")
    return indexes


def wingtip_contact_indexes(model_path):
    contacts = ET.parse(model_path).getroot().findall("./ground_reactions/contact")
    return tuple(index for index, contact in enumerate(contacts)
                 if contact.get("type") == "STRUCTURE"
                 and contact.get("name") in ("LEFT_WINGTIP", "RIGHT_WINGTIP"))


def gentle_wingtip_contact(contact, first_contact):
    values = [contact.get(key, float("nan")) for key in ("time", "groundspeed", "sink", "pitch", "roll")]
    first_time = first_contact.get("time", float("nan"))
    return all(math.isfinite(value) for value in values) and math.isfinite(first_time) \
        and contact["time"] >= first_time and 0.0 <= contact["groundspeed"] <= 2.0 \
        and abs(contact["sink"]) <= 0.3 \
        and LANDING_PITCH_MIN_DEG <= contact["pitch"] <= LANDING_PITCH_MAX_DEG and abs(contact["roll"]) <= 8.0


def wingtip_settled_after_belly(contact, first_contact):
    contact_time = contact.get("time", float("nan"))
    first_time = first_contact.get("time", float("nan"))
    return math.isfinite(contact_time) and math.isfinite(first_time) and contact_time >= first_time


def landing_metrics(samples, flight_plan, start_time, ground_altitude, contacts=None, belly_contacts=(0, 1),
                    max_sample_gap=0.3, wingtip_contacts=(), allow_wingtip_settling=False):
    active = [sample for sample in samples if sample["time"] >= start_time]
    final_block = flight_plan["blocks"]["final"]
    flare_block = flight_plan["blocks"]["flare"]
    final_samples = [sample for sample in active if sample.get("nav_block") in (final_block, flare_block)]
    if not final_samples:
        raise RuntimeError("Landing never reached final")
    contact = next((sample for sample in final_samples
                    if sample["altitude"] <= ground_altitude + 0.15), None)
    if contact is None:
        raise RuntimeError("No near-ground crossing recorded during final/flare")
    method = "NPS truth crossing preflight ground altitude + 0.15 m; not exact first contact"
    if contacts:
        first_contact = contacts[0]
        contact = dict(contact, **{key: first_contact[key] for key in ("east", "north", "altitude", "pitch", "roll")})
        contact["down_speed"] = first_contact["sink"]
        contact["north_speed"] = 0.0
        contact["east_speed"] = first_contact["groundspeed"]
        method = "JSBSim first structural contact at physics timestep (CG position)"
    touchdown = flight_plan["waypoints"]["TD"]
    approach = flight_plan["waypoints"]["AF"]
    landing_sector = flight_plan.get("landing_sector")
    if landing_sector is None:
        raise ValueError("Takeoff_Landing_zone_Fixedwing is required for precision-landing validation")
    east = touchdown["x"] - approach["x"]
    north = touchdown["y"] - approach["y"]
    length = math.hypot(east, north)
    if not math.isfinite(length) or length <= 1.0:
        raise ValueError("AF and TD must be distinct")
    delta_east = contact["east"] - touchdown["x"]
    delta_north = contact["north"] - touchdown["y"]
    longitudinal = (delta_east * east + delta_north * north) / length
    lateral = (delta_east * north - delta_north * east) / length
    finite_contact = all(math.isfinite(contact[key]) for key in
                         ("east", "north", "pitch", "roll", "down_speed", "east_speed", "north_speed"))
    contact_quality = finite_contact and 0.0 <= contact["down_speed"] <= 1.5 and abs(contact["roll"]) <= 8.0 \
                      and LANDING_PITCH_MIN_DEG <= contact["pitch"] <= LANDING_PITCH_MAX_DEG
    if contacts:
        contact_quality = contact_quality and contacts[0].get("contact_index") in belly_contacts
    after_contact = [sample for sample in active if sample["time"] >= contact["time"]]
    stop_start = len(after_contact) - 1
    while stop_start > 0 and after_contact[-1]["time"] - after_contact[stop_start]["time"] < 2.0:
        stop_start -= 1
    stop_samples = after_contact[stop_start:]
    stopped = len(stop_samples) >= 2 and stop_samples[-1]["time"] - stop_samples[0]["time"] >= 2.0
    stopped = stopped and all(0 < following["time"] - preceding["time"] <= max_sample_gap
                              for preceding, following in zip(stop_samples, stop_samples[1:]))
    rollout_safe = bool(after_contact)
    for sample in after_contact:
        forward = ((sample["east"] - touchdown["x"]) * east + (sample["north"] - touchdown["y"]) * north) / length
        across = ((sample["east"] - touchdown["x"]) * north - (sample["north"] - touchdown["y"]) * east) / length
        rollout_safe = rollout_safe and point_inside_polygon(sample["east"], sample["north"], landing_sector) \
            and abs(sample["roll"]) <= 8.0 and LANDING_PITCH_MIN_DEG <= sample["pitch"] <= LANDING_PITCH_MAX_DEG \
            and sample["altitude"] <= ground_altitude + 0.3
    for sample in stop_samples:
        stopped = stopped and math.hypot(sample["east_speed"], sample["north_speed"]) <= 0.5 \
            and abs(sample["down_speed"]) <= 0.2 and sample["altitude"] <= ground_altitude + 0.2
    final_sample = after_contact[-1]
    stop_data_valid = all(math.isfinite(sample[key]) for sample in stop_samples
                          for key in ("time", "east", "north", "altitude", "east_speed", "north_speed", "down_speed"))
    stopped = stopped and stop_data_valid and all(
        math.hypot(sample["east"] - final_sample["east"], sample["north"] - final_sample["north"]) <= 0.2
        for sample in stop_samples)
    engine_off = stopped and all(math.isfinite(sample.get("throttle", float("nan")))
                                 and sample["throttle"] <= 0.01
                                 and math.isfinite(sample.get("command_throttle", float("nan")))
                                 and abs(sample["command_throttle"]) <= 1.0 for sample in stop_samples)
    stop_confirmed = bool(contacts) and finite_contact and stopped and engine_off
    final_longitudinal = ((final_sample["east"] - touchdown["x"]) * east
                          + (final_sample["north"] - touchdown["y"]) * north) / length if stop_confirmed else None
    final_lateral = ((final_sample["east"] - touchdown["x"]) * north
                     - (final_sample["north"] - touchdown["y"]) * east) / length if stop_confirmed else None
    final_inside_sector = stop_confirmed \
        and point_inside_polygon(final_sample["east"], final_sample["north"], landing_sector)
    go_around_samples = sum(sample.get("nav_block") == flight_plan["blocks"]["go-around"] for sample in active)
    uninterrupted_landing = go_around_samples == 0
    wing_touches = [item for item in (contacts or []) if item.get("contact_index") in wingtip_contacts]
    gentle_touches = [item for item in wing_touches
                      if contacts[0].get("contact_index") in belly_contacts
                      and gentle_wingtip_contact(item, contacts[0])]
    accepted_wingtip_touches = [item for item in wing_touches
                                if contacts[0].get("contact_index") in belly_contacts
                                and (gentle_wingtip_contact(item, contacts[0])
                                     or (allow_wingtip_settling
                                         and wingtip_settled_after_belly(item, contacts[0])))]
    unacceptable_contacts = sum(item.get("contact_index") not in belly_contacts
                                and not (item.get("contact_index") in wingtip_contacts
                                         and contacts[0].get("contact_index") in belly_contacts
                                         and (gentle_wingtip_contact(item, contacts[0])
                                              or (allow_wingtip_settling
                                                  and wingtip_settled_after_belly(item, contacts[0]))))
                                for item in (contacts or []))
    no_bad_contact = bool(contacts) and unacceptable_contacts == 0
    contact_inside_sector = point_inside_polygon(contact["east"], contact["north"], landing_sector)
    touchdown_distance = math.hypot(longitudinal, lateral)
    final_distance = math.hypot(final_longitudinal, final_lateral) if stop_confirmed else None
    touchdown_inside_precision = contact_inside_sector and touchdown_distance <= LANDING_PRECISION_RADIUS_M
    final_inside_precision = final_inside_sector and final_distance <= LANDING_PRECISION_RADIUS_M
    return {
        "contact_method": method,
        "touchdown_longitudinal_m": longitudinal,
        "touchdown_cross_track_m": lateral,
        "touchdown_groundspeed_mps": math.hypot(contact["north_speed"], contact["east_speed"]),
        "touchdown_sink_mps": contact["down_speed"],
        "touchdown_pitch_deg": contact["pitch"],
        "touchdown_roll_deg": contact["roll"],
        "touchdown_inside_landing_sector": contact_inside_sector,
        "touchdown_inside_precision_box": touchdown_inside_precision,
        "touchdown_inside_precision_radius": touchdown_inside_precision,
        "touchdown_inside_internal_margin": contact_inside_sector and abs(lateral) <= 1.2,
        "final_longitudinal_m": final_longitudinal,
        "final_cross_track_m": final_lateral,
        "final_distance_to_td_m": final_distance,
        "touchdown_to_stop_distance_m": math.hypot(final_sample["east"] - contact["east"],
                               final_sample["north"] - contact["north"]) if stop_confirmed else None,
        "inside_landing_sector": final_inside_sector,
        "inside_precision_box": final_inside_precision,
        "inside_precision_radius": final_inside_precision,
        "inside_internal_margin": final_inside_sector and abs(final_lateral) <= 1.2,
        "contact_quality_pass": contact_quality,
        "stopped": stopped,
        "engine_off": engine_off,
        "stop_confirmed": stop_confirmed,
        "rollout_quality_pass": rollout_safe and no_bad_contact,
        "wingtip_touch_count": len(wing_touches),
        "gentle_wingtip_touch_count": len(gentle_touches),
        "accepted_wingtip_touch_count": len(accepted_wingtip_touches),
        "unacceptable_contact_count": unacceptable_contacts,
        "contact_review_required": not (contact_quality and rollout_safe and no_bad_contact),
        "strict_quality_pass": uninterrupted_landing and contact_inside_sector and contact_quality
                       and stop_confirmed and rollout_safe and no_bad_contact,
        "landing_pass": uninterrupted_landing and final_inside_sector,
        "preferred_landing_pass": uninterrupted_landing and final_inside_precision and contact_inside_sector,
        "max_brake_fraction": max(sample.get("brake_fraction", 0.0) for sample in final_samples),
        "go_around_samples": go_around_samples,
    }


def main():
    parser = argparse.ArgumentParser(description="Run repeatable fixed-wing NPS tuning tests")
    parser.add_argument("--aircraft", required=True, help="Generated Paparazzi aircraft name")
    parser.add_argument("--ac-id", required=True, type=int)
    parser.add_argument("--scenario", choices=("launch-check", "altitude-step", "oval", "precision-landing"), required=True)
    parser.add_argument("--radius", type=float, default=40.0)
    parser.add_argument("--altitude-step", type=float, default=30.0)
    parser.add_argument("--settle-seconds", type=float, default=30.0)
    parser.add_argument("--measure-seconds", type=float, default=60.0)
    parser.add_argument("--preflight-seconds", type=float, default=10.0)
    parser.add_argument("--time-factor", type=float, default=4.0)
    parser.add_argument("--bus", default="127.255.255.255:2010")
    parser.add_argument("--udp-port", type=int, default=4242)
    parser.add_argument("--udp-uplink-port", type=int, default=4243)
    parser.add_argument("--landing-timeout", type=float, default=300.0, help="Maximum simulated seconds for landing")
    parser.add_argument("--landing-block", default="Land Right AF-TD")
    parser.add_argument("--expect-go-around", action="store_true", help="Validate rejection and climb-out instead of touchdown")
    parser.add_argument("--wind-speed", type=float, default=0.0)
    parser.add_argument("--wind-direction", type=float, default=0.0)
    parser.add_argument("--setting", action="append", default=[], metavar="NAME=VALUE")
    parser.add_argument("--output-dir", type=Path, default=Path("var/nps_tuning"))
    args = parser.parse_args()

    aircraft_dir = PAPARAZZI_HOME / "var/aircrafts" / args.aircraft
    simulator = aircraft_dir / "nps/simsitl"
    settings_header_path = aircraft_dir / "nps/generated/settings.h"
    flight_plan_path = aircraft_dir / "flight_plan.xml"
    for required in (simulator, settings_header_path, flight_plan_path):
        if not required.exists():
            raise RuntimeError(f"Missing generated aircraft artifact: {required}")

    settings = setting_indexes(settings_header_path)
    overrides = parse_setting_overrides(args.setting, settings)
    flight_plan = flight_plan_data(flight_plan_path, aircraft_dir / "nps/generated/flight_plan.h")
    belly_contacts = ()
    wingtip_contacts = ()
    allow_wingtip_settling = False
    if args.scenario == "precision-landing":
        airframe_header = (aircraft_dir / "nps/generated/airframe.h").read_text()
        model = re.search(r'^#define NPS_JSBSIM_MODEL "([^"]+)"', airframe_header, re.MULTILINE)
        if model is None:
            raise ValueError("Precision landing requires a named JSBSim model")
        model_path = PAPARAZZI_HOME / "conf/simulator/jsbsim/aircraft" / (model[1] + ".xml")
        belly_contacts = belly_contact_indexes(model_path)
        wingtip_contacts = wingtip_contact_indexes(model_path)
        allow_wingtip_settling = model[1] in ("openuas_jsbsim_multiplex_easystar3",
                              "openuas_jsbsim_zohd_talon_250g")
    blocks = flight_plan["blocks"]
    required_settings = ("autopilot.mode", "autopilot.launch", "autopilot.kill_throttle", "flight_altitude", "nav_radius")
    missing_settings = [name for name in required_settings if name not in settings]
    if missing_settings:
        raise RuntimeError(f"Missing settings: {', '.join(missing_settings)}")
    if args.scenario == "oval" and "Oval 1-2" not in blocks:
        raise RuntimeError("Flight plan must provide an 'Oval 1-2' block")
    launch_block = blocks.get("Takeoff")
    if launch_block is None:
        raise RuntimeError("Flight plan must provide a 'Takeoff' block")

    args.output_dir.mkdir(parents=True, exist_ok=True)
    suffix = ""
    if args.scenario == "oval":
        suffix = f"_{args.radius:g}m"
    elif args.scenario == "altitude-step":
        suffix = f"_{args.altitude_step:+g}m"
    prefix = args.output_dir / f"{args.aircraft}_{args.scenario}{suffix}"
    environment = os.environ.copy()
    environment["PAPARAZZI_HOME"] = str(PAPARAZZI_HOME)
    environment["PAPARAZZI_SRC"] = str(PAPARAZZI_SRC)

    processes = []
    logs = []
    interface = None
    recorder = FlightRecorder(args.ac_id, args.time_factor)
    samples_path = prefix.with_suffix(".csv")
    try:
        server, server_log = start_process(
            [str(PAPARAZZI_HOME / "sw/ground_segment/tmtc/server"), "-n", "-b", args.bus],
            environment,
            prefix.with_suffix(".server.log"),
        )
        processes.append(server)
        logs.append(server_log)
        link, link_log = start_process(
            [str(PAPARAZZI_HOME / "sw/ground_segment/tmtc/link"), "-udp", "-udp_broadcast", "-b", args.bus,
             "-udp_port", str(args.udp_port), "-udp_uplink_port", str(args.udp_uplink_port)],
            environment,
            prefix.with_suffix(".link.log"),
        )
        processes.append(link)
        logs.append(link_log)
        time.sleep(1.0)
        for process, log_file in zip(processes, logs):
            if process.poll() is not None:
                raise RuntimeError(f"Test process exited with {process.returncode}; see {log_file.name}")

        interface = IvyMessagesInterface("nps_fixedwing_tuning", ivy_bus=args.bus)
        interface.subscribe(recorder.position, PprzMessage("telemetry", "NPS_SPEED_POS"))
        interface.subscribe(recorder.attitude, PprzMessage("telemetry", "NPS_RATE_ATTITUDE"))
        for message_name in ("ENERGY", "DESIRED", "AIRSPEED", "COMMANDS", "H_CTL_A", "NAVIGATION", "SONAR", "DEBUG_VECT"):
            interface.subscribe(recorder.control, PprzMessage("telemetry", message_name))

        simulator_process, simulator_log = start_process(
            [str(simulator), "--rc_script", "0", "--time_factor", str(args.time_factor), "--ivy_bus", args.bus],
            environment,
            prefix.with_suffix(".simulator.log"),
        )
        processes.append(simulator_process)
        logs.append(simulator_log)
        wait_for_samples(recorder, 10, 15.0)
        wait_for(lambda sample: "nav_block" in sample and "command_throttle" in sample,
             recorder, 15.0, "autopilot telemetry (check UDP ports)")
        time.sleep(args.preflight_seconds / args.time_factor)
        ground_altitude = recorder.snapshot()[-1]["altitude"]

        for index, value in overrides:
            send_setting(interface, args.ac_id, index, value)
        send_setting(interface, args.ac_id, settings["nps_atmosphere.wind_speed"], args.wind_speed)
        send_setting(interface, args.ac_id, settings["nps_atmosphere.wind_dir"], math.radians(args.wind_direction))
        send_setting(interface, args.ac_id, settings["flight_altitude"], flight_plan["altitude"])
        send_setting(interface, args.ac_id, settings["autopilot.mode"], 2)
        send_setting(interface, args.ac_id, settings["autopilot.kill_throttle"], 0)
        jump_to_block(interface, args.ac_id, launch_block)
        time.sleep(0.5)
        send_setting(interface, args.ac_id, settings["autopilot.launch"], 1)
        airborne = wait_for(
            lambda sample: sample["altitude"] >= ground_altitude + 20.0,
            recorder,
            30.0,
            "takeoff",
        )

        if args.scenario == "launch-check":
            start_time = airborne["time"]
            time.sleep(args.measure_seconds / args.time_factor)
            active = [sample for sample in recorder.snapshot() if sample["time"] >= start_time]
            metrics = {
                "climb_achieved": max(sample["altitude"] for sample in active) - ground_altitude,
                "airspeed_min": min(sample["airspeed"] for sample in active),
                "airspeed_max": max(sample["airspeed"] for sample in active),
                "pitch_abs_max": max(abs(sample["pitch"]) for sample in active),
                "roll_abs_max": max(abs(sample["roll"]) for sample in active),
            }
            samples = recorder.snapshot()
            write_csv(samples_path, samples)
            for name, value in metrics.items():
                print(f"{name}={value:.3f}")
            return

        time.sleep(args.settle_seconds / args.time_factor)
        if args.scenario == "precision-landing":
            start_time = recorder.snapshot()[-1]["time"]
            contact_log = prefix.with_suffix(".simulator.log")
            if read_contacts(contact_log):
                raise RuntimeError("Structural contact occurred before the landing scenario")
            jump_to_block(interface, args.ac_id, blocks[args.landing_block])
            wait_for(lambda sample: sample.get("nav_block") == blocks["final"], recorder,
                     args.landing_timeout / args.time_factor, "landing final")
            if args.expect_go_around:
                wait_for(lambda sample: sample.get("nav_block") == blocks["go-around"], recorder,
                         args.landing_timeout / args.time_factor, "approach rejection")
                climbout = wait_for(lambda sample: sample.get("nav_block") == blocks["go-around"]
                                    and sample["altitude"] > ground_altitude + 25.0,
                                    recorder, args.landing_timeout / args.time_factor, "go-around climb-out")
                if read_contacts(contact_log):
                    raise RuntimeError("Go-around contacted the ground")
                metrics = {"go_around_pass": True, "climbout_altitude_m": climbout["altitude"],
                           "contact_count": 0}
                prefix.with_suffix(".json").write_text(json.dumps(metrics, indent=2) + "\n")
                print(json.dumps(metrics, indent=2))
                return
            wait_for(lambda sample: sample.get("nav_block") in (blocks["final"], blocks["flare"])
                     and sample["altitude"] <= ground_altitude + 0.15,
                     recorder, args.landing_timeout / args.time_factor, "landing near-ground crossing")
            time.sleep(args.measure_seconds / args.time_factor)
            contacts = read_contacts(prefix.with_suffix(".simulator.log"))
            if not contacts:
                raise RuntimeError("First-contact record missing; the aircraft NPS target must define NPS_JSBSIM_CONTACT_LOG=1")
            metrics = landing_metrics(recorder.snapshot(), flight_plan, start_time, ground_altitude, contacts,
                                      belly_contacts, max_sample_gap=0.3 * args.time_factor,
                                      wingtip_contacts=wingtip_contacts,
                                      allow_wingtip_settling=allow_wingtip_settling)
            prefix.with_suffix(".json").write_text(json.dumps(metrics, indent=2) + "\n")
            print(json.dumps(metrics, indent=2))
            if not metrics["stop_confirmed"]:
                raise RuntimeError("Final stopping position is unconfirmed: contact log and stable two-second stop required")
            if metrics["go_around_samples"]:
                raise RuntimeError("Normal landing entered the go-around block")
            if not metrics["landing_pass"]:
                raise RuntimeError("Aircraft did not stop with engine off inside Takeoff_Landing_zone_Fixedwing")
            if not metrics["preferred_landing_pass"]:
                raise RuntimeError(f"Aircraft stopped more than {LANDING_PRECISION_RADIUS_M:.1f} m from TD")
            if not metrics["strict_quality_pass"]:
                raise RuntimeError("Landing contact or rollout failed strict quality checks")
            return
        settled_sample = recorder.snapshot()[-1]
        desired_altitude = settled_sample.get("desired_altitude")
        target_altitude = (
            desired_altitude - flight_plan["ground_altitude"]
            if desired_altitude is not None
            else settled_sample["altitude"]
        )

        if args.scenario == "altitude-step":
            baseline = recorder.snapshot()[-1]["altitude"]
            if args.altitude_step < 0.0:
                staged_altitude = baseline - args.altitude_step
                staged_msl = flight_plan["ground_altitude"] + staged_altitude
                send_setting(interface, args.ac_id, settings["flight_altitude"], staged_msl)
                wait_for(
                    lambda sample: abs(sample["altitude"] - staged_altitude) <= 1.0,
                    recorder,
                    45.0,
                    "descent staging altitude",
                )
                time.sleep(args.settle_seconds / args.time_factor)
                baseline = recorder.snapshot()[-1]["altitude"]
            target_altitude = baseline + args.altitude_step
            start_time = recorder.snapshot()[-1]["time"]
            target_msl = flight_plan["ground_altitude"] + target_altitude
            send_setting(interface, args.ac_id, settings["flight_altitude"], target_msl)
            time.sleep(args.measure_seconds / args.time_factor)
            metrics = altitude_metrics(recorder.snapshot(), target_altitude, start_time)
        else:
            send_setting(interface, args.ac_id, settings["nav_radius"], args.radius)
            start_time = recorder.snapshot()[-1]["time"]
            time.sleep(args.measure_seconds / args.time_factor)
            metrics = oval_metrics(recorder.snapshot(), target_altitude, args.radius, start_time)

        samples = recorder.snapshot()
        write_csv(samples_path, samples)
        for name, value in metrics.items():
            if isinstance(value, float):
                print(f"{name}={value:.3f}")
            else:
                print(f"{name}={value}")
    finally:
        write_csv(samples_path, recorder.snapshot())
        for process in reversed(processes):
            stop_process(process)
        if interface is not None:
            interface.shutdown()
        for log_file in logs:
            log_file.close()


if __name__ == "__main__":
    main()