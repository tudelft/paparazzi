import unittest

from nps_fixedwing_tuning import FlightRecorder, PprzMessage, landing_metrics, parse_setting_overrides


class LandingMetricsTest(unittest.TestCase):
    def test_validate_settings_before_startup(self):
        self.assertEqual(parse_setting_overrides(["flare_brake=0.4"], {"flare_brake": 3}), [(3, 0.4)])
        with self.assertRaises(ValueError):
            parse_setting_overrides(["flare_brake=0.4"], {})
        with self.assertRaises(ValueError):
            parse_setting_overrides(["flare_brake=nan"], {"flare_brake": 3})

    def test_landing_vector_from_ivy(self):
        recorder = FlightRecorder(135, 1)
        message = PprzMessage("telemetry", "DEBUG_VECT")
        message.ivy_string_to_payload('"precision_landing" 1,2,3,4,5,6,7,0.5,1,0,0')
        recorder.control(135, message)
        self.assertEqual(recorder.latest_control["brake_fraction"], 0.5)
        self.assertEqual(recorder.latest_control["landing_agl"], 2)

    def setUp(self):
        self.plan = {"blocks": {"final": 2, "flare": 3, "go-around": 4},
                     "waypoints": {"AF": {"x": -100, "y": 0}, "TD": {"x": 0, "y": 0}}}
        self.sample = {"time": 5, "nav_block": 3, "altitude": 0.1, "east": -6, "north": 0.5,
                       "east_speed": 8, "north_speed": 0, "down_speed": 0.8, "pitch": 3, "roll": 0}

    def test_td_projection(self):
        result = landing_metrics([self.sample], self.plan, 4, 0)
        self.assertEqual(result["touchdown_longitudinal_m"], -6)
        self.assertEqual(result["touchdown_cross_track_m"], -0.5)
        self.assertTrue(result["inside_precision_box"])

    def test_crosswind_miss(self):
        self.sample["north"] = 2
        self.assertFalse(landing_metrics([self.sample], self.plan, 4, 0)["inside_precision_box"])

    def test_bad_contact_is_not_accepted(self):
        self.sample["down_speed"] = 2.0
        self.assertFalse(landing_metrics([self.sample], self.plan, 4, 0)["contact_quality_pass"])
        self.sample["down_speed"] = float("nan")
        self.assertFalse(landing_metrics([self.sample], self.plan, 4, 0)["contact_quality_pass"])

    def test_first_contact_overrides_bounce(self):
        contact = {"east": -7, "north": 0, "altitude": 0.08, "pitch": 2, "roll": 1,
                   "sink": 0.8, "groundspeed": 9}
        result = landing_metrics([self.sample], self.plan, 4, 0, [contact])
        self.assertEqual(result["touchdown_longitudinal_m"], -7)
        self.assertEqual(result["touchdown_sink_mps"], 0.8)

    def test_no_contact_is_not_success(self):
        self.sample["altitude"] = 3
        with self.assertRaises(RuntimeError):
            landing_metrics([self.sample], self.plan, 4, 0)

    def test_preflight_and_go_around_are_not_touchdown(self):
        for block in (0, 4):
            self.sample["nav_block"] = block
            with self.assertRaises(RuntimeError):
                landing_metrics([self.sample], self.plan, 4, 0)


if __name__ == "__main__":
    unittest.main()