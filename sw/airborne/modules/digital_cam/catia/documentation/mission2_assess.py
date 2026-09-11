#!/usr/bin/env python3
"""Offline IMAV 2026 Mission 2 assessment; never controls an aircraft."""

import argparse
import csv
import itertools
import json
import math
from pathlib import Path


AUTONOMY = {"onboard": 1.0, "offboard": 0.7, "manual": 0.4}
LANDING = {"none": 0, "zone": 1, "precision": 2}


def read_points(path, reference=False):
    identifier = "target_id" if reference else "report_id"
    fields = [identifier, "latitude_deg", "longitude_deg"]
    if reference:
        fields.append("uncertainty_m")
    points = []
    identifiers = set()
    coordinates = set()
    with Path(path).open(newline="", encoding="utf-8-sig") as file:
        reader = csv.DictReader(file)
        if reader.fieldnames is None or sorted(reader.fieldnames) != sorted(fields):
            raise ValueError(f"{path}: expected exactly these CSV columns: {', '.join(fields)}")
        for row_number, row in enumerate(reader, 2):
            if None in row or any(value is None or not value.strip() for value in row.values()):
                raise ValueError(f"{path}:{row_number}: incomplete or extra CSV values")
            identity = row[identifier].strip()
            latitude = finite_number(row["latitude_deg"], "latitude", -90, 90)
            longitude = finite_number(row["longitude_deg"], "longitude", -180, 180)
            if identity in identifiers or (latitude, longitude) in coordinates:
                raise ValueError(f"{path}:{row_number}: duplicate identifier or coordinates")
            identifiers.add(identity)
            coordinates.add((latitude, longitude))
            uncertainty = finite_number(row["uncertainty_m"], "reference uncertainty", 0) if reference else 0
            points.append(dict(identifier=identity, latitude=latitude, longitude=longitude,
                               uncertainty_m=uncertainty))
    if reference and len(points) != 2:
        raise ValueError("Provide exactly two independently surveyed reference targets")
    if not reference and len(points) > 2:
        raise ValueError("Provide the actual final submission of at most two targets, not a candidate list")
    return points


def validate_submission(reference_path, submission_path, submission_delay_s, design_limit_m=4.0):
    from geographiclib.geodesic import Geodesic

    delay = finite_number(submission_delay_s, "submission delay (s)", 0)
    design_limit = finite_number(design_limit_m, "design limit (m)", 0, 5, positive=True)
    references = read_points(reference_path, reference=True)
    reports = read_points(submission_path)
    on_time = delay <= 300
    assignments = []
    for ordering in itertools.permutations(references, len(reports)):
        matches = []
        for reference, report in zip(ordering, reports):
            distance = Geodesic.WGS84.Inverse(reference["latitude"], reference["longitude"],
                                             report["latitude"], report["longitude"])["s12"]
            upper = distance + reference["uncertainty_m"]
            matches.append({"report_id": report["identifier"], "target_id": reference["identifier"],
                            "error_m": distance, "reference_uncertainty_m": reference["uncertainty_m"],
                            "conservative_error_m": upper, "within_5m": distance <= 5,
                            "within_design_budget": upper <= design_limit})
        assignments.append(matches)
    best = min(assignments, key=lambda matches: (-sum(match["within_5m"] for match in matches),
                                                sum(match["error_m"] for match in matches)))
    within_tolerance = sum(match["within_5m"] for match in best)
    ambiguous = within_tolerance > 0 and sum(
        sum(match["within_5m"] for match in matches) == within_tolerance for matches in assignments
    ) > 1
    matched = {match["target_id"] for match in best}
    return {
        "status": "offline_validation_not_official_judging",
        "submission_delay_s": delay, "submitted_on_time": on_time,
        "reference_requirement": "Independent WGS84 survey; uncertainty_m must be a defensible horizontal bound.",
        "matching": "One-to-one; maximize targets within 5 m, then minimize total distance.",
        "assignment_ambiguous": ambiguous, "matches": best,
        "unreported_targets": [point["identifier"] for point in references if point["identifier"] not in matched],
        "within_5m_count": within_tolerance,
        "provisional_valid_hotspots": within_tolerance if on_time else 0,
        "conservative_within_5m_count": sum(match["conservative_error_m"] <= 5 for match in best),
        "both_targets_within_5m_on_time": on_time and within_tolerance == 2,
        "design_limit_m": design_limit,
        "design_gate_passed": on_time and len(best) == 2 and not ambiguous
                              and all(match["within_design_budget"] for match in best),
    }


def finite_number(value, name, minimum, maximum=None, positive=False):
    number = float(value)
    if not math.isfinite(number) or number < minimum or (positive and number == 0):
        raise ValueError(f"Invalid {name}: {value}")
    if maximum is not None and number > maximum:
        raise ValueError(f"Invalid {name}: {value}")
    return number


def score_mission(*, mass_kg, voltage_v, flight_seconds, consumed_mah,
                  valid_hotspots, autonomy, landing, self_made_points, safety_review):
    if safety_review not in ("not_reviewed", "rejected", "passed"):
        raise ValueError("Unknown safety-review status")
    if safety_review != "passed":
        return {"status": "blocked_by_safety_review", "score": None,
                "safety_review": safety_review,
                "reason": "Review terrain/trees, flight envelope, geofence, and return/landing reserve first."}
    mass = finite_number(mass_kg, "takeoff mass (kg)", 0, 5, positive=True)
    voltage = finite_number(voltage_v, "fully charged voltage (V)", 0, positive=True)
    duration = finite_number(flight_seconds, "launch-to-stop duration (s)", 0, positive=True)
    charge = finite_number(consumed_mah, "consumed charge (mAh)", 0, positive=True)
    if valid_hotspots not in (0, 1, 2):
        raise ValueError("Valid hotspot count must be 0, 1 or 2")
    if autonomy not in AUTONOMY or landing not in LANDING or self_made_points not in (0, 1, 2):
        raise ValueError("Invalid autonomy, landing or self-made category")
    weight_factor = -2 * math.expm1(-5 / (1.4 * mass))
    power_factor = min(3.0, duration / charge / voltage * 50)
    multiplier = 8 / 11.75 * weight_factor * AUTONOMY[autonomy]
    hotspot_points = 3.5 * valid_hotspots
    bonus = LANDING[landing] + self_made_points
    return {
        "status": "offline_estimate_not_flight_authorization",
        "rules": "IMAV2026 V4-1 sections 5.4.3 and 5.4.4, Table 8",
        "configuration": "fixed_wing", "configuration_factor": 1.0,
        "safety_review": safety_review,
        "weight_factor": weight_factor, "autonomy_factor": AUTONOMY[autonomy],
        "power_factor": power_factor, "hotspot_base_points": hotspot_points,
        "mission_points": multiplier * (hotspot_points + power_factor),
        "landing_points": LANDING[landing], "self_made_points": self_made_points,
        "score": multiplier * (hotspot_points + power_factor) + bonus,
        "gain_per_additional_valid_hotspot": multiplier * 3.5 if valid_hotspots < 2 else 0.0,
        "remaining_power_score_headroom": multiplier * (3 - power_factor),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    score = commands.add_parser("score", help="Evaluate the fixed-wing Table 8 formula")
    score.add_argument("--mass-kg", type=float, required=True)
    score.add_argument("--voltage-v", type=float, required=True)
    score.add_argument("--flight-seconds", type=float, required=True)
    score.add_argument("--consumed-mah", type=float, required=True)
    score.add_argument("--valid-hotspots", type=int, choices=(0, 1, 2), required=True,
                       help="Distinct accepted targets within 5 m and submitted within 300 s of landing")
    score.add_argument("--autonomy", choices=AUTONOMY, required=True)
    score.add_argument("--landing", choices=LANDING, required=True,
                       help="Landing bonus category; 'none' means no bonus, not no landing or recovery")
    score.add_argument("--self-made-points", type=int, choices=(0, 1, 2), required=True)
    score.add_argument("--safety-review", choices=("not_reviewed", "rejected", "passed"),
                       default="not_reviewed", help="Human review record, not a safety check performed by this tool")
    validate = commands.add_parser("validate", help="Check a final two-target submission against independent survey")
    validate.add_argument("--reference", type=Path, required=True)
    validate.add_argument("--submission", type=Path, required=True)
    validate.add_argument("--submission-delay-s", type=float, required=True,
                          help="Elapsed time from landing to actual submission, including recovery, shutdown, SD-card access and copying")
    validate.add_argument("--design-limit-m", type=float, default=4.0)
    args = parser.parse_args()
    try:
        if args.command == "score":
            values = vars(args).copy()
            values.pop("command")
            result = score_mission(**values)
            status = 2 if result["score"] is None else 0
        else:
            result = validate_submission(args.reference, args.submission, args.submission_delay_s, args.design_limit_m)
            status = 0 if result["design_gate_passed"] else 2
    except ImportError:
        parser.error("Validation requires GeographicLib; install mission2-assessment-requirements.txt in a desktop venv")
    except (ValueError, OverflowError, OSError, csv.Error) as error:
        parser.error(str(error))
    print(json.dumps(result, indent=2, allow_nan=False))
    return status


if __name__ == "__main__":
    raise SystemExit(main())