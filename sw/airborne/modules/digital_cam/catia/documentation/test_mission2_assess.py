import csv
import importlib.util
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

from mission2_assess import read_points, score_mission, validate_submission


class MissionScoreTests(unittest.TestCase):
    def setUp(self):
        self.inputs = dict(mass_kg=2.0, voltage_v=16.8, flight_seconds=600,
                           consumed_mah=1000, valid_hotspots=2, autonomy="onboard",
                           landing="zone", self_made_points=0, safety_review="passed")

    def test_literal_rulebook_formula(self):
        result = score_mission(**self.inputs)
        weight = 2 * (1 - math.exp(-5 / (1.4 * 2)))
        power = 600 * 50 / (16.8 * 1000)
        expected = 8 / 11.75 * (7 + power) * weight + 1
        self.assertAlmostEqual(result["score"], expected)
        self.assertAlmostEqual(result["power_factor"], power)
        self.assertEqual(result["configuration_factor"], 1)

    def test_power_cap_and_missing_hotspot_gain(self):
        self.inputs.update(consumed_mah=100, valid_hotspots=1)
        result = score_mission(**self.inputs)
        self.assertEqual(result["power_factor"], 3)
        self.assertEqual(result["remaining_power_score_headroom"], 0)
        self.inputs["valid_hotspots"] = 2
        complete = score_mission(**self.inputs)
        self.assertAlmostEqual(complete["score"] - result["score"],
                               result["gain_per_additional_valid_hotspot"])

    def test_autonomy_does_not_multiply_bonuses(self):
        self.inputs.update(landing="precision", self_made_points=2)
        onboard = score_mission(**self.inputs)
        self.inputs["autonomy"] = "offboard"
        offboard = score_mission(**self.inputs)
        self.assertAlmostEqual(offboard["score"] - 4, (onboard["score"] - 4) * .7)

    def test_unsafe_or_unreviewed_is_not_scored(self):
        for status in ("not_reviewed", "rejected"):
            self.inputs["safety_review"] = status
            result = score_mission(**self.inputs)
            self.assertIsNone(result["score"])
            self.assertEqual(result["status"], "blocked_by_safety_review")

    def test_invalid_or_missing_measurements(self):
        for key, value in (("mass_kg", 5.1), ("mass_kg", 0), ("voltage_v", 0),
                           ("flight_seconds", -1), ("consumed_mah", math.nan),
                           ("voltage_v", math.inf), ("valid_hotspots", 3)):
            with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                score_mission(**(self.inputs | {key: value}))

    def test_cli_defaults_to_no_safety_approval(self):
        script = Path(__file__).with_name("mission2_assess.py")
        command = [sys.executable, str(script), "score", "--mass-kg", "2",
                   "--voltage-v", "16.8", "--flight-seconds", "600", "--consumed-mah", "1000",
                   "--valid-hotspots", "2", "--autonomy", "onboard", "--landing", "zone",
                   "--self-made-points", "0"]
        result = subprocess.run(command, capture_output=True, text=True, check=False)
        self.assertEqual(result.returncode, 2)
        self.assertIn('"score": null', result.stdout)

    def test_landing_without_bonus_is_still_a_completed_flight(self):
        with_bonus = score_mission(**self.inputs)
        self.inputs["landing"] = "none"
        without_bonus = score_mission(**self.inputs)
        self.assertEqual(without_bonus["landing_points"], 0)
        self.assertAlmostEqual(with_bonus["score"] - without_bonus["score"], 1)

    def test_cli_explains_sd_retrieval_deadline(self):
        script = Path(__file__).with_name("mission2_assess.py")
        result = subprocess.run([sys.executable, str(script), "validate", "--help"],
                                capture_output=True, text=True, check=False)
        self.assertEqual(result.returncode, 0)
        self.assertIn("SD-card", result.stdout)
        self.assertNotIn("pre-landing", result.stdout)


@unittest.skipUnless(importlib.util.find_spec("geographiclib"), "Install assessment requirements for geodesic tests")
class SubmissionTests(unittest.TestCase):
    def setUp(self):
        from geographiclib.geodesic import Geodesic

        self.geodesic = Geodesic.WGS84
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.root = Path(directory.name)
        self.reference = self.root / "reference.csv"
        self.submission = self.root / "submission.csv"
        self.references = [("first", 48.81, 7.85, .2), ("second", 48.811, 7.85, .2)]
        self.write(self.reference, ["target_id", "latitude_deg", "longitude_deg", "uncertainty_m"], self.references)

    def write(self, path, fields, rows):
        with path.open("w", newline="", encoding="utf-8") as file:
            writer = csv.writer(file)
            writer.writerow(fields)
            writer.writerows(rows)

    def reports(self, offsets, reverse=False):
        rows = []
        for reference, distance in zip(self.references, offsets):
            point = self.geodesic.Direct(reference[1], reference[2], 90, distance)
            rows.append((f"report-{len(rows)}", point["lat2"], point["lon2"]))
        self.write(self.submission, ["report_id", "latitude_deg", "longitude_deg"],
                   list(reversed(rows)) if reverse else rows)

    def test_reversed_ids_distance_and_uncertainty(self):
        self.reports([3, 2], reverse=True)
        result = validate_submission(self.reference, self.submission, 300)
        self.assertTrue(result["design_gate_passed"])
        self.assertTrue(result["both_targets_within_5m_on_time"])
        self.assertFalse(result["assignment_ambiguous"])
        self.assertEqual(result["matches"][0]["target_id"], "second")
        self.assertAlmostEqual(result["matches"][1]["error_m"], 3, places=6)
        self.assertAlmostEqual(result["matches"][1]["conservative_error_m"], 3.2, places=6)

    def test_reference_uncertainty_and_design_margin(self):
        self.reports([4.9, 3.9])
        result = validate_submission(self.reference, self.submission, 0)
        self.assertEqual(result["provisional_valid_hotspots"], 2)
        self.assertEqual(result["conservative_within_5m_count"], 1)
        self.assertFalse(result["design_gate_passed"])

    def test_late_and_outside_limit(self):
        self.reports([5.01, 3])
        result = validate_submission(self.reference, self.submission, 301)
        self.assertEqual(result["within_5m_count"], 1)
        self.assertEqual(result["provisional_valid_hotspots"], 0)
        self.assertFalse(result["design_gate_passed"])

    def test_deadline_includes_all_post_landing_retrieval_time(self):
        self.reports([2, 2])
        on_time = validate_submission(self.reference, self.submission, 300)
        late = validate_submission(self.reference, self.submission, 300.001)
        self.assertTrue(on_time["design_gate_passed"])
        self.assertFalse(late["submitted_on_time"])
        self.assertEqual(late["provisional_valid_hotspots"], 0)

    def test_missing_and_empty_submission(self):
        for offsets in ([1], []):
            self.reports(offsets)
            result = validate_submission(self.reference, self.submission, 10)
            self.assertEqual(result["provisional_valid_hotspots"], len(offsets))
            self.assertEqual(len(result["unreported_targets"]), 2 - len(offsets))
            self.assertFalse(result["design_gate_passed"])

    def test_duplicate_invalid_and_extra_reports(self):
        for rows in ([("same", 1, 2), ("same", 3, 4)],
                     [("one", 1, 2), ("two", 1, 2)], [("bad", "nan", 2)],
                     [("one", 1, 2), ("two", 3, 4), ("three", 5, 6)]):
            self.write(self.submission, ["report_id", "latitude_deg", "longitude_deg"], rows)
            with self.assertRaises(ValueError):
                read_points(self.submission)
        self.write(self.reference, ["target_id", "latitude_deg", "longitude_deg", "uncertainty_m"],
                   self.references[:1])
        with self.assertRaises(ValueError):
            read_points(self.reference, reference=True)

    def test_overlapping_acceptance_circles_are_ambiguous(self):
        self.write(self.reference, ["target_id", "latitude_deg", "longitude_deg", "uncertainty_m"],
                   [("first", 0, 0, .2), ("second", 0, .00001, .2)])
        self.write(self.submission, ["report_id", "latitude_deg", "longitude_deg"],
                   [("one", 0, .000003), ("two", 0, .000007)])
        result = validate_submission(self.reference, self.submission, 10)
        self.assertEqual(result["provisional_valid_hotspots"], 2)
        self.assertTrue(result["assignment_ambiguous"])
        self.assertFalse(result["design_gate_passed"])

    def test_cli_acceptance_and_failure_status(self):
        script = Path(__file__).with_name("mission2_assess.py")
        for distance, expected_status in ((2, 0), (6, 2)):
            self.reports([distance, 2])
            command = [sys.executable, str(script), "validate", "--reference", str(self.reference),
                       "--submission", str(self.submission), "--submission-delay-s", "20"]
            result = subprocess.run(command, capture_output=True, text=True, check=False)
            self.assertEqual(result.returncode, expected_status, result.stderr)
            parsed = json.loads(result.stdout)
            self.assertEqual(parsed["design_gate_passed"], expected_status == 0)


if __name__ == "__main__":
    unittest.main()