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


def flight_plan_data(flight_plan_path):
    indexes = {}
    root = ET.parse(flight_plan_path).getroot()
    flight_plan = root if root.tag == "flight_plan" else root.find("flight_plan")
    blocks = flight_plan.find("blocks") if flight_plan is not None else None
    if blocks is None:
        raise RuntimeError(f"No blocks found in {flight_plan_path}")
    for index, block in enumerate(blocks.findall("block")):
        indexes[block.get("name")] = int(block.get("no", index))
    return {
        "blocks": indexes,
        "waypoints": {waypoint.get("name"): {axis: float(waypoint.get(axis, "0")) for axis in ("x", "y", "alt")}
                      for waypoint in flight_plan.find("waypoints")},
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
    while time.monotonic() < deadline:
        samples = recorder.snapshot()
        if samples and predicate(samples[-1]):
            return samples[-1]
        time.sleep(0.1)
    raise RuntimeError(f"Timed out waiting for {description}")


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


def landing_metrics(samples, flight_plan, start_time, ground_altitude, contacts=None):
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
    east = touchdown["x"] - approach["x"]
    north = touchdown["y"] - approach["y"]
    length = math.hypot(east, north)
    if length <= 1.0:
        raise ValueError("AF and TD must be distinct")
    delta_east = contact["east"] - touchdown["x"]
    delta_north = contact["north"] - touchdown["y"]
    longitudinal = (delta_east * east + delta_north * north) / length
    lateral = (delta_east * north - delta_north * east) / length
    finite_contact = all(math.isfinite(contact[key]) for key in
                         ("east", "north", "pitch", "roll", "down_speed", "east_speed", "north_speed"))
    contact_quality = finite_contact and 0.0 <= contact["down_speed"] <= 1.5 and abs(contact["roll"]) <= 8.0 \
                      and 0.0 <= contact["pitch"] <= 15.0
    if contacts:
        contact_quality = contact_quality and contacts[0].get("contact_index", 0) in (0, 1)
    return {
        "contact_method": method,
        "touchdown_longitudinal_m": longitudinal,
        "touchdown_cross_track_m": lateral,
        "touchdown_groundspeed_mps": math.hypot(contact["north_speed"], contact["east_speed"]),
        "touchdown_sink_mps": contact["down_speed"],
        "touchdown_pitch_deg": contact["pitch"],
        "touchdown_roll_deg": contact["roll"],
        "inside_precision_box": abs(longitudinal) <= 10.0 and abs(lateral) <= 1.5,
        "inside_internal_margin": abs(longitudinal) <= 8.0 and abs(lateral) <= 1.2,
        "contact_quality_pass": contact_quality,
        "max_brake_fraction": max(sample.get("brake_fraction", 0.0) for sample in final_samples),
        "go_around_samples": sum(sample.get("nav_block") == flight_plan["blocks"]["go-around"] for sample in active),
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
    flight_plan = flight_plan_data(flight_plan_path)
    blocks = flight_plan["blocks"]
    required_settings = ("autopilot.mode", "autopilot.launch", "autopilot.kill_throttle", "flight_altitude", "nav_radius")
    missing_settings = [name for name in required_settings if name not in settings]
    if missing_settings:
        raise RuntimeError(f"Missing settings: {', '.join(missing_settings)}")
    if "Oval 1-2" not in blocks:
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
                raise RuntimeError("First-contact record missing; build NPS with USER_CFLAGS=-DNPS_JSBSIM_CONTACT_LOG=1")
            metrics = landing_metrics(recorder.snapshot(), flight_plan, start_time, ground_altitude, contacts)
            prefix.with_suffix(".json").write_text(json.dumps(metrics, indent=2) + "\n")
            print(json.dumps(metrics, indent=2))
            if not metrics["inside_precision_box"]:
                raise RuntimeError("Landing outside the 20 x 3 m precision box")
            if not metrics["contact_quality_pass"]:
                raise RuntimeError("Contact outside provisional sink/attitude limits; inspect the recorded first contact")
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