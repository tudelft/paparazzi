import tempfile
import unittest
from dataclasses import replace
from pathlib import Path

import numpy as np

import log_2_tuned_airframe as tune


class AutotuneProvenanceTest(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.log_path = Path(self.directory.name) / "flight.log"
        self.airframe = """<airframe>
          <firmware NAME="fixedwing">
            <target NAME="nps"><define NAME="PITCH_TRIM" VALUE="999"/></target>
            <target NAME="ap"><define NAME="USE_AIRSPEED" VALUE="TRUE"/></target>
          </firmware>
          <section PREFIX="IMU_"><define NAME="BODY_TO_IMU_THETA" VALUE="0.13"/></section>
          <section PREFIX="COMMAND_"><define NAME="PITCH_TRIM" VALUE="400"/></section>
          <section><define NAME="BRAKE_MAX_PCT" VALUE="0.75f"/></section>
          <command_laws>
            <let VAR="brake_value_nofilt" VALUE="((autopilot.mode == AP_MODE_AUTO2) ? Clip(-@BRAKE, 0, MAX_PPRZ) : 0)"/>
          </command_laws>
        </airframe>"""
        self.log_path.write_text(
            '<configuration><conf><aircraft ac_id="129" name="Adam" airframe="airframes/talon.xml">'
            + self.airframe + '</aircraft><aircraft ac_id="135" name="Other" airframe="other.xml">'
            '<airframe><define NAME="PITCH_TRIM" VALUE="-500"/></airframe>'
            '</aircraft></conf></configuration>')
        self.base = """<airframe>
          <section><define name="PITCH_TRIM" value="400"/>
          <define name="BODY_TO_IMU_THETA" value="0.13"/>
          <define name="COURSE_PGAIN" value="0.9"/></section>
        </airframe>"""
        name, relative, defines = tune.parse_log_header(self.log_path, 129)
        self.flight = tune.FlightLog(self.log_path, self.log_path.with_suffix(".data"),
                                    129, name, relative, defines)

    def test_hardware_values_belong_to_selected_aircraft(self):
        self.assertEqual(self.flight.aircraft_name, "Adam")
        self.assertEqual(self.flight.flown_defines["PITCH_TRIM"], "400")
        self.assertEqual(self.flight.flown_defines["BODY_TO_IMU_THETA"], "0.13")
        self.assertEqual(self.flight.flown_defines["USE_AIRSPEED"], "TRUE")

    def test_missing_embedded_airframe_cannot_borrow_next_aircraft(self):
        self.log_path.write_text(self.log_path.read_text().replace(self.airframe, ""))
        with self.assertRaises(tune.AutotuneError):
            tune.parse_log_header(self.log_path, 129)

    def test_logged_brakes_hold_tuning_even_when_current_mixer_removed(self):
        analysis = tune.Analysis(auto2_negative_brake_frac=1.0, level_elevator_pprz=500)
        output, changes, advice = tune.tune_airframe(self.base, analysis, self.flight, 40)
        self.assertEqual(output, self.base)
        self.assertFalse(changes)
        self.assertIn("AUTO2 AIRBRAKES / TUNING HOLD", [item.topic for item in advice])

    def test_explicit_trim_wins_even_when_equal_to_base(self):
        analysis = tune.Analysis(auto1_only=True, level_elevator_pprz=500)
        for value in ("400", "600"):
            with self.subTest(value=value):
                output, _, advice = tune.tune_airframe(self.base, analysis, self.flight, 40,
                                                      {"PITCH_TRIM": value})
                self.assertEqual(tune.AirframeEditor(output).get_define("PITCH_TRIM"), value)
                self.assertNotIn("override", [item.topic for item in advice])

    def test_auto1_mode_excludes_auto2_and_transition_interpolation(self):
        times = np.arange(0.0, 120.0, 0.2)
        count = len(times)
        attitude = np.zeros((count, 3))
        attitude[:, 0] = 0.01 * np.sin(times)
        attitude[:, 2] = 0.06
        gps = np.zeros((count, 6))
        gps[:, 4] = 320000
        gps[:, 5] = 1000
        air_data = np.zeros((count, 7))
        air_data[:, 5] = 10
        desired = np.zeros((count, 8))
        desired[:, 0] = 0.01 * np.sin(times)
        desired[:, 1] = 0.06
        desired[:, 5] = 320
        command_time = np.arange(0.0, 120.0, 0.8)
        commands = np.zeros((len(command_time), 5))
        commands[:, 0] = 7000
        commands[:, 2] = 500
        messages = {
            "ATTITUDE": (times, attitude), "GPS": (times, gps),
            "AIR_DATA": (times, air_data), "AIRSPEED": (times, np.full((count, 1), 10.0)),
            "DESIRED": (times, desired), "COMMANDS": (command_time, commands),
            "ESTIMATOR": (times, np.c_[np.full(count, 320.0), np.zeros(count)]),
            "NAVIGATION_REF": (np.array([0.0]), np.array([[0, 0, 0, 280]])),
            "PPRZ_MODE": (np.array([0.0, 40.0, 60.0, 100.0]), np.array([[1], [2], [1], [2]])),
        }
        flight = replace(self.flight, msgs=messages)
        first = tune.analyze(flight, -20, auto1_only=True)
        self.assertEqual(first.auto2_seconds, 0)
        self.assertEqual(first.n_course, 0)
        self.assertAlmostEqual(first.level_elevator_pprz, 500)
        commands[((command_time >= 40) & (command_time < 60)) | (command_time >= 100), 2] = 9000
        second = tune.analyze(flight, -20, auto1_only=True)
        self.assertAlmostEqual(second.level_elevator_pprz, first.level_elevator_pprz)
        output, _, _ = tune.tune_airframe(self.base, second, flight, 40)
        self.assertEqual(tune.AirframeEditor(output).get_define("COURSE_PGAIN"), "0.9")


if __name__ == "__main__":
    unittest.main()