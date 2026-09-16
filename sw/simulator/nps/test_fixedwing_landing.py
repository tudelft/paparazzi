import unittest
import subprocess
import tempfile
from pathlib import Path
import xml.etree.ElementTree as ET

from nps_fixedwing_tuning import FlightRecorder, PprzMessage, landing_metrics, parse_setting_overrides, belly_contact_indexes, wingtip_contact_indexes


class LandingBrakeOwnershipTest(unittest.TestCase):
    def test_manual_rc_cannot_latch_autonomous_brakes(self):
        home = Path(__file__).resolve().parents[3]
        for name in ("openuas_zohd_talon_250g", "openuas_multiplex_easystar_3"):
            with self.subTest(airframe=name):
                airframe = ET.parse(home / "conf/airframes/OPENUAS" / f"{name}.xml").getroot()
                self.assertEqual(airframe.find("./rc_commands/set[@command='BRAKE']").get("value"), "0")
                self.assertIsNone(airframe.find("./auto_rc_commands/set[@command='BRAKE']"))
                self.assertEqual(airframe.find("./commands/axis[@name='BRAKE']").get("failsafe_value"), "0")
                mixer = airframe.find("./command_laws/let[@var='brake_value_nofilt']").get("value")
                self.assertEqual(mixer, "((autopilot.mode == AP_MODE_AUTO2) ? Clip(-@BRAKE, 0, MAX_PPRZ) : $manual_crow)")

    def test_talon_slider_is_isolated_from_autonomous_brakes(self):
        self.check_slider_isolation("openuas_zohd_talon_250g")

    def test_easystar_slider_is_isolated_from_autonomous_brakes(self):
        self.check_slider_isolation("openuas_multiplex_easystar_3")

    def check_slider_isolation(self, name):
        home = Path(__file__).resolve().parents[3]
        airframe = ET.parse(home / "conf/airframes/OPENUAS" / f"{name}.xml").getroot()
        self.assertIsNone(airframe.find(".//module[@name='auto1_commands']"))
        manual = airframe.find("./command_laws/let[@var='manual_crow']").get("value")
        selected = airframe.find("./command_laws/let[@var='brake_value_nofilt']").get("value")
        selected = selected.replace("@BRAKE", "autonomous_brake").replace("$manual_crow", "manual_crow")
        source = """
#include <assert.h>
#define AP_MODE_MANUAL 0
#define AP_MODE_AUTO1 1
#define AP_MODE_AUTO2 2
#define RC_OK 0
#define RADIO_BRAKE 8
#define MAX_PPRZ 9600
#define Clip(value,lower,upper) ((value)<(lower)?(lower):((value)>(upper)?(upper):(value)))
struct { int mode; } autopilot;
struct { int status; } radio_control;
static int slider;
static int radio_control_get(int channel) { (void)channel; return slider; }
static int demand(int mode, int status, int rc, int autonomous_brake) {
    autopilot.mode = mode;
    radio_control.status = status;
    slider = rc;
    int manual_crow = MANUAL_EXPR;
    return SELECT_EXPR;
}
int main(void) {
    for (int mode = 0; mode <= 1; mode++) {
        assert(demand(mode, 0, -9600, 0) == 9600);
        assert(demand(mode, 0, -4800, 0) == 4800);
        assert(demand(mode, 0, 0, -9600) == 0);
        assert(demand(mode, 0, 9600, 0) == 0);
        assert(demand(mode, 1, -9600, 0) == 0);
        assert(demand(mode, 2, -9600, 0) == 0);
    }
    assert(demand(1, 0, -9600, 0) == 9600);
    assert(demand(2, 0, -9600, 0) == 0);
    assert(demand(2, 2, -9600, 0) == 0);
    assert(demand(2, 0, 9600, -4800) == 4800);
    assert(demand(2, 2, 9600, -4800) == 4800);
    assert(demand(2, 0, -9600, 0) == 0);
    assert(demand(3, 0, -9600, -9600) == 0);
    int remaining_crow = 9600;
    int target = demand(2, 0, -9600, 0);
    for (int tick = 0; tick < 23; tick++) {
        remaining_crow += Clip(target - remaining_crow, -RETRACT_RATE, EXTEND_RATE);
    }
    assert(remaining_crow == 0);
    return 0;
}
"""
        source = source.replace("MANUAL_EXPR", manual).replace("SELECT_EXPR", selected)
        for placeholder, name in (("RETRACT_RATE", "BRAKE_ENABLED_SLOWDOWN_FACTOR_RETRACT"),
                                  ("EXTEND_RATE", "BRAKE_ENABLED_SLOWDOWN_FACTOR_EXTEND")):
            source = source.replace(placeholder, airframe.find(f".//define[@name='{name}']").get("value"))
        with tempfile.TemporaryDirectory() as directory:
            binary = str(Path(directory) / "crow-test")
            subprocess.run(["gcc", "-std=c11", "-Wall", "-Wextra", "-Werror", "-x", "c", "-", "-o", binary],
                           input=source, text=True, capture_output=True, check=True)
            subprocess.run([binary], check=True)

    def test_easystar_auto1_hatch_mixer(self):
        home = Path(__file__).resolve().parents[3]
        airframe = ET.parse(home / "conf/airframes/OPENUAS/openuas_multiplex_easystar_3.xml").getroot()
        selected = airframe.find("./command_laws/set[@servo='S_HATCH']").get("value")
        closed = airframe.find(".//define[@name='SERVO_HATCH_CLOSED']").get("value")
        source = """
#include <assert.h>
#define AP_MODE_AUTO1 1
#define RC_OK 0
#define MIN_PPRZ -9600
#define SERVO_HATCH_CLOSED CLOSED_EXPR
#define RADIO_HATCH 1
struct { int mode; } autopilot;
struct { int status; } radio_control;
static int slider;
static int radio_control_get(int channel) { (void)channel; return slider; }
static int demand(int stored_hatch) { return SELECT_EXPR; }
int main(void) {
    for (autopilot.mode = 0; autopilot.mode < 4; autopilot.mode++) {
        for (radio_control.status = 0; radio_control.status < 3; radio_control.status++) {
            for (slider = -9600; slider <= 9600; slider += 4800) {
                int expected = autopilot.mode == AP_MODE_AUTO1
                    ? (radio_control.status == RC_OK ? slider : SERVO_HATCH_CLOSED) : -2400;
                assert(demand(-2400) == expected);
            }
        }
    }
    autopilot.mode = AP_MODE_AUTO1;
    radio_control.status = RC_OK;
    slider = 9600;
    assert(demand(-9600) == 9600);
    autopilot.mode = 2;
    assert(demand(-9600) == -9600);
    assert(demand(0) == 0);
    return 0;
}
"""
        source = source.replace("SELECT_EXPR", selected.replace("@HATCH", "stored_hatch"))
        source = source.replace("CLOSED_EXPR", closed)
        with tempfile.TemporaryDirectory() as directory:
            binary = str(Path(directory) / "hatch-test")
            subprocess.run(["gcc", "-std=c11", "-Wall", "-Wextra", "-Werror",
                            "-x", "c", "-", "-o", binary],
                           input=source, text=True, capture_output=True, check=True)
            subprocess.run([binary], check=True)

    def test_braking_blocks_retract_on_exit(self):
        home = Path(__file__).resolve().parents[3]
        plan = ET.parse(home / "conf/flight_plans/TUDELFT/tudelft_imav2026_o_test_pricise_landing.xml").getroot()
        for name in ("Crow brake bench test", "flare"):
            with self.subTest(block=name):
                self.assertEqual(plan.find(f"./blocks/block[@name='{name}']").get("on_exit"),
                                 "precision_landing_stop()")
                final = plan.find("./blocks/block[@name='final']")
                self.assertEqual(final.get("on_exit"), "precision_landing_release()")
                self.assertEqual(final.get("pre_call"), "precision_landing_run()")
                self.assertEqual(plan.find("./blocks/block[@name='go-around']").get("on_enter"),
                         "precision_landing_check_abort()")
                self.assertIsNone(plan.find("./blocks/block[@name='flare']/set[@var='autopilot.kill_throttle']"))
                self.assertNotIn("NavSetGroundReferenceHere()", ET.tostring(plan, encoding="unicode"))
                for name in ("Wait GPS", "Geo init", "Holding point", "Takeoff", "Standby",
                             "Oval 1-2", "XOval 1-2", "Survey S1-S2"):
                    self.assertEqual(plan.find(f"./blocks/block[@name='{name}']").get("on_enter"),
                                     "precision_landing_stop()")
                for name in ("land", "retry decision"):
                    self.assertEqual(plan.find(f"./blocks/block[@name='{name}']/exception").get("cond"),
                                     "!precision_landing_is_active()")


class LandingMetricsTest(unittest.TestCase):
    def test_gentle_wingtip_settling_for_both_models(self):
        home = Path(__file__).resolve().parents[3]
        for model_name in ("zohd_talon_250g", "multiplex_easystar3"):
            allow_wingtip_settling = True
            model = home / f"conf/simulator/jsbsim/aircraft/openuas_jsbsim_{model_name}.xml"
            belly = belly_contact_indexes(model)
            wings = wingtip_contact_indexes(model)
            self.assertEqual(len(wings), 2)
            self.assertFalse(set(belly) & set(wings))
            first = dict(time=5, east=-6, north=0.5, altitude=0.1, pitch=3, roll=0,
                         sink=0.8, groundspeed=8, contact_index=belly[0])
            touch = dict(first, time=6, contact_index=wings[0], groundspeed=0.9, sink=0.01, roll=3)
            samples = [dict(self.sample, time=5 + tick * 0.1, east_speed=0, north_speed=0, down_speed=0)
                       for tick in range(31)]
            result = landing_metrics(samples, self.plan, 4, 0, [first, touch], belly, wingtip_contacts=wings,
                                     allow_wingtip_settling=allow_wingtip_settling)
            self.assertTrue(result["landing_pass"])
            self.assertEqual(result["gentle_wingtip_touch_count"], 1)
            self.assertEqual(result["unacceptable_contact_count"], 0)
            repeated_strike = dict(touch, time=7, sink=1.0)
            result = landing_metrics(samples, self.plan, 4, 0, [first, touch, repeated_strike], belly,
                                     wingtip_contacts=wings, allow_wingtip_settling=allow_wingtip_settling)
            self.assertEqual(result["strict_quality_pass"], allow_wingtip_settling)
            self.assertTrue(result["landing_pass"])
            self.assertEqual(result["wingtip_touch_count"], 2)
            self.assertEqual(result["accepted_wingtip_touch_count"], 2 if allow_wingtip_settling else 1)
            self.assertEqual(result["unacceptable_contact_count"], 0 if allow_wingtip_settling else 1)
            for change in ({"groundspeed": 15}, {"sink": 1}, {"roll": 20}, {"pitch": 20.01},
                           {"sink": float("nan")}, {"time": 4}, {"contact_index": 99}):
                with self.subTest(model=model_name, change=change):
                    result = landing_metrics(samples, self.plan, 4, 0, [first, dict(touch, **change)], belly,
                                             wingtip_contacts=wings,
                                             allow_wingtip_settling=allow_wingtip_settling)
                    accepted_for_foam = allow_wingtip_settling and change not in ({"time": 4},
                                                                                  {"contact_index": 99})
                    self.assertEqual(result["strict_quality_pass"], accepted_for_foam)
                    self.assertEqual(result["unacceptable_contact_count"], 0 if accepted_for_foam else 1)
            missing = dict(touch)
            del missing["sink"]
            self.assertEqual(landing_metrics(samples, self.plan, 4, 0, [first, missing], belly,
                                             wingtip_contacts=wings,
                                             allow_wingtip_settling=allow_wingtip_settling)["strict_quality_pass"],
                             allow_wingtip_settling)
            self.assertFalse(landing_metrics(samples, self.plan, 4, 0, [touch], belly,
                                             wingtip_contacts=wings,
                                             allow_wingtip_settling=allow_wingtip_settling)["contact_quality_pass"])
            samples[-1]["north"] = 3
            self.assertFalse(landing_metrics(samples, self.plan, 4, 0, [first, touch], belly,
                                             wingtip_contacts=wings,
                                             allow_wingtip_settling=allow_wingtip_settling)["landing_pass"])

    def test_accelerated_telemetry_stop(self):
        contact = dict(east=-6, north=0.5, altitude=0.1, pitch=3, roll=0, sink=0.8, groundspeed=8, contact_index=1)
        samples = [dict(self.sample, time=5 + tick * 0.4, east_speed=0, north_speed=0, down_speed=0)
                   for tick in range(11)]
        self.assertTrue(landing_metrics(samples, self.plan, 4, 0, [contact], (1, 2),
                                        max_sample_gap=1.2)["stopped"])
        self.assertFalse(landing_metrics([samples[0], samples[-1]], self.plan, 4, 0, [contact], (1, 2),
                                         max_sample_gap=1.2)["stopped"])
        samples = [dict(self.sample, time=5 + tick * 0.43, east_speed=0, north_speed=0, down_speed=0)
               for tick in range(11)]
        self.assertTrue(landing_metrics(samples, self.plan, 4, 0, [contact], (1, 2),
                        max_sample_gap=1.2)["stopped"])
        samples[-6]["east_speed"] = 1.0
        self.assertFalse(landing_metrics(samples, self.plan, 4, 0, [contact], (1, 2),
                         max_sample_gap=1.2)["stopped"])

    def test_ground_settling_is_not_nose_contact(self):
        contact = dict(east=-6, north=0.5, altitude=0.1, pitch=3, roll=0, sink=0.8, groundspeed=8, contact_index=1)
        samples = [dict(self.sample, time=5 + tick * 0.1, east_speed=0, north_speed=0, down_speed=0)
                   for tick in range(31)]
        samples[4]["pitch"] = -3
        self.assertTrue(landing_metrics(samples, self.plan, 4, 0, [contact], (1, 2))["landing_pass"])
        self.assertFalse(landing_metrics(samples, self.plan, 4, 0,
                                         [contact, dict(contact, contact_index=0)], (1, 2))["strict_quality_pass"])
        samples[4]["pitch"] = -16
        self.assertFalse(landing_metrics(samples, self.plan, 4, 0, [contact], (1, 2))["strict_quality_pass"])
        samples[4]["pitch"] = 3
        contact["pitch"] = -3
        self.assertTrue(landing_metrics(samples, self.plan, 4, 0, [contact], (1, 2))["contact_quality_pass"])

    def test_slight_nose_down_quality_gate(self):
        contact = dict(east=-6, north=0.5, altitude=0.1, pitch=-3, roll=0,
                       sink=0.8, groundspeed=8, contact_index=1)
        samples = [dict(self.sample, time=5 + tick * 0.1, east_speed=0, north_speed=0, down_speed=0)
                   for tick in range(31)]
        for pitch, accepted in ((-10, True), (-5, True), (0, True), (15, True), (20, True),
                    (-10.01, False), (20.01, False), (float("nan"), False),
                    (float("inf"), False), (-float("inf"), False)):
            with self.subTest(pitch=pitch):
                result = landing_metrics(samples, self.plan, 4, 0, [dict(contact, pitch=pitch)], (1, 2))
                self.assertEqual(result["contact_quality_pass"], accepted)
                self.assertEqual(result["strict_quality_pass"], accepted)
        for change in ({"sink": 4.4}, {"roll": 20}, {"contact_index": 0}):
            with self.subTest(change=change):
                self.assertFalse(landing_metrics(samples, self.plan, 4, 0,
                                                 [dict(contact, **change)], (1, 2))["strict_quality_pass"])

    def test_rollout_and_wingtip_pitch_boundaries(self):
        contact = dict(time=5, east=-6, north=0.5, altitude=0.1, pitch=3, roll=0,
                       sink=0.8, groundspeed=8, contact_index=1)
        for pitch, accepted in ((-10, True), (20, True), (-10.01, False), (20.01, False),
                                (float("nan"), False), (float("inf"), False), (-float("inf"), False)):
            samples = [dict(self.sample, time=5 + tick * 0.1, east_speed=0, north_speed=0, down_speed=0)
                       for tick in range(31)]
            with self.subTest(pitch=pitch, phase="rollout"):
                samples[4]["pitch"] = pitch
                result = landing_metrics(samples, self.plan, 4, 0, [contact], (1, 2))
                self.assertEqual(result["rollout_quality_pass"], accepted)
                self.assertEqual(result["strict_quality_pass"], accepted)
            with self.subTest(pitch=pitch, phase="wingtip"):
                samples[4]["pitch"] = 3
                touch = dict(contact, time=6, pitch=pitch, groundspeed=0.9, sink=0.01, contact_index=4)
                result = landing_metrics(samples, self.plan, 4, 0, [contact, touch], (1, 2), wingtip_contacts=(4, 5))
                self.assertEqual(result["gentle_wingtip_touch_count"], int(accepted))
                self.assertEqual(result["strict_quality_pass"], accepted)

    def test_talon_contact_identity(self):
        home = Path(__file__).resolve().parents[3]
        model = home / "conf/simulator/jsbsim/aircraft/openuas_jsbsim_zohd_talon_250g.xml"
        model_contacts = ET.parse(model).getroot().findall("./ground_reactions/contact")
        contacts = belly_contact_indexes(model)
        self.assertEqual({model_contacts[index].get("name") for index in contacts},
                         {"BELLY_FRONT", "BELLY_REAR", "BELLY_FRONT_RIGHT", "BELLY_REAR_RIGHT",
                          "BELLY_SAND_DRAG"})
        contact = dict(east=-6, north=0.5, altitude=0.1, pitch=3, roll=0, sink=0.8, groundspeed=8)
        for index in range(len(model_contacts)):
            result = landing_metrics([self.sample], self.plan, 4, 0,
                                     [dict(contact, contact_index=index)], contacts)
            self.assertEqual(result["contact_quality_pass"], index in contacts)

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
                       "east_speed": 8, "north_speed": 0, "down_speed": 0.8, "pitch": 3, "roll": 0,
                       "throttle": 0, "command_throttle": 0}

    def test_td_projection(self):
        result = landing_metrics([self.sample], self.plan, 4, 0)
        self.assertEqual(result["touchdown_longitudinal_m"], -6)
        self.assertEqual(result["touchdown_cross_track_m"], -0.5)
        self.assertTrue(result["touchdown_inside_precision_box"])
        self.assertFalse(result["inside_precision_box"])
        self.assertIsNone(result["final_distance_to_td_m"])

    def test_crosswind_miss(self):
        self.sample["north"] = 3
        self.assertFalse(landing_metrics([self.sample], self.plan, 4, 0)["touchdown_inside_precision_box"])

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

    def test_stopped_landing_and_later_damage(self):
        contacts = [{"east": -6, "north": 0.5, "altitude": 0.1, "pitch": 3, "roll": 0,
                     "sink": 0.8, "groundspeed": 8, "contact_index": 0}]
        samples = [dict(self.sample, time=5 + tick * 0.1, east_speed=0, north_speed=0, down_speed=0)
                   for tick in range(31)]
        self.assertTrue(landing_metrics(samples, self.plan, 4, 0, contacts)["landing_pass"])
        samples[-1]["east_speed"] = 2
        self.assertFalse(landing_metrics(samples, self.plan, 4, 0, contacts)["landing_pass"])
        samples[-1]["east_speed"] = 0
        samples[-1]["north"] = 3
        self.assertFalse(landing_metrics(samples, self.plan, 4, 0, contacts)["landing_pass"])
        samples[-1]["north"] = 0.5
        contacts.append(dict(contacts[0], contact_index=2))
        result = landing_metrics(samples, self.plan, 4, 0, contacts)
        self.assertFalse(result["strict_quality_pass"])
        self.assertTrue(result["landing_pass"])
        self.assertTrue(result["contact_review_required"])

    def test_first_contact_alone_is_not_success(self):
        result = landing_metrics([self.sample], self.plan, 4, 0)
        self.assertFalse(result["landing_pass"])

    def test_stationary_with_engine_on_is_not_success(self):
        contacts = [{"east": -6, "north": 0.5, "altitude": 0.1, "pitch": 3, "roll": 0,
                     "sink": 0.8, "groundspeed": 8, "contact_index": 0}]
        samples = [dict(self.sample, time=5 + tick * 0.1, east_speed=0, north_speed=0, down_speed=0)
                   for tick in range(31)]
        samples[-1]["command_throttle"] = 100
        result = landing_metrics(samples, self.plan, 4, 0, contacts)
        self.assertFalse(result["engine_off"])
        self.assertFalse(result["landing_pass"])

    def test_missing_samples_and_outside_first_contact_fail(self):
        contacts = [{"east": -6, "north": 0.5, "altitude": 0.1, "pitch": 3, "roll": 0,
                     "sink": 0.8, "groundspeed": 8, "contact_index": 0}]
        samples = [dict(self.sample, time=5 + tick * 0.1, east_speed=0, north_speed=0, down_speed=0)
                   for tick in range(31)]
        self.assertFalse(landing_metrics([samples[0], samples[-1]], self.plan, 4, 0, contacts)["stopped"])
        contacts[0]["east"] = -11
        result = landing_metrics(samples, self.plan, 4, 0, contacts)
        self.assertFalse(result["strict_quality_pass"])
        self.assertTrue(result["landing_pass"])

    def test_final_spot_is_primary(self):
        contact = dict(time=5, east=-12, north=0, altitude=0.1, pitch=-8, roll=0,
                       sink=2, groundspeed=8, contact_index=1)
        samples = [dict(self.sample, time=5 + tick * 0.1, east=1, north=0.5,
                        east_speed=0, north_speed=0, down_speed=0) for tick in range(31)]
        result = landing_metrics(samples, self.plan, 4, 0, [contact], (1, 2))
        self.assertFalse(result["touchdown_inside_precision_box"])
        self.assertFalse(result["contact_quality_pass"])
        self.assertTrue(result["landing_pass"])
        self.assertTrue(result["contact_review_required"])
        self.assertEqual(result["final_longitudinal_m"], 1)
        self.assertFalse(result["preferred_landing_pass"])
        self.assertAlmostEqual(result["touchdown_to_stop_distance_m"], (13 ** 2 + 0.5 ** 2) ** 0.5)
        self.assertEqual(result["final_cross_track_m"], -0.5)
        self.assertAlmostEqual(result["final_distance_to_td_m"], 1.25 ** 0.5)
        contact["east"] = 0
        contact["sink"] = 0.8
        for sample in samples:
            sample["east"] = 11
        result = landing_metrics(samples, self.plan, 4, 0, [contact], (1, 2))
        self.assertTrue(result["touchdown_inside_precision_box"])
        self.assertTrue(result["stop_confirmed"])
        self.assertFalse(result["landing_pass"])
        self.assertFalse(result["preferred_landing_pass"])

    def test_both_touch_and_stop_inside_is_preferred(self):
        contact = dict(time=5, east=-6, north=0.5, altitude=0.1, pitch=-8, roll=0,
                       sink=2, groundspeed=8, contact_index=1)
        samples = [dict(self.sample, time=5 + tick * 0.1, east=1, north=0.5,
                        east_speed=0, north_speed=0, down_speed=0) for tick in range(31)]
        result = landing_metrics(samples, self.plan, 4, 0, [contact], (1, 2))
        self.assertTrue(result["landing_pass"])
        self.assertTrue(result["preferred_landing_pass"])
        self.assertFalse(result["strict_quality_pass"])
        self.assertTrue(result["contact_review_required"])
        self.assertEqual(result["touchdown_to_stop_distance_m"], 7)
        samples[-1]["east_speed"] = 1
        result = landing_metrics(samples, self.plan, 4, 0, [contact], (1, 2))
        self.assertFalse(result["preferred_landing_pass"])
        self.assertIsNone(result["touchdown_to_stop_distance_m"])

    def test_creep_or_invalid_final_position_is_not_standstill(self):
        contact = dict(time=5, east=-6, north=0.5, altitude=0.1, pitch=3, roll=0,
                       sink=0.8, groundspeed=8, contact_index=1)
        samples = [dict(self.sample, time=5 + tick * 0.1, east=-6 + tick * 0.04,
                        east_speed=0.4, north_speed=0, down_speed=0) for tick in range(31)]
        result = landing_metrics(samples, self.plan, 4, 0, [contact], (1, 2))
        self.assertFalse(result["stop_confirmed"])
        self.assertIsNone(result["final_distance_to_td_m"])
        for sample in samples:
            sample["east"] = -6
            sample["east_speed"] = 0
        samples[-1]["east"] = float("nan")
        self.assertFalse(landing_metrics(samples, self.plan, 4, 0, [contact], (1, 2))["landing_pass"])


class LandingControllerTest(unittest.TestCase):
    def test_rangefinder_async_buffer_safety(self):
        home = Path(__file__).resolve().parents[3]
        with tempfile.TemporaryDirectory() as directory:
            for sensor_mode, periodic, debug in ((mode, periodic, debug) for mode in (0, 81)
                                                for periodic in (0, 1) for debug in (None, 0, 1)):
                with self.subTest(sensor_mode=sensor_mode, periodic=periodic, debug=debug):
                    binary = str(Path(directory) / f"rangefinder-{sensor_mode}-{periodic}-{debug}")
                    debug_flags = [] if debug is None else [f"-DRANGEFINDER_I2C_SYNC_SEND={debug}"]
                    subprocess.run(["gcc", "-std=c11", "-Wall", "-Wextra", "-Werror", "-g", "-O1",
                                    "-fsanitize=address,undefined,float-cast-overflow", "-fno-sanitize-recover=all", "-no-pie",
                                    "-Itests/modules/rangefinder_i2c_stubs", "-Itests/modules/precision_landing_stubs",
                                    "-Isw/include", "-Isw/airborne", "-DRANGEFINDER_I2C_PORT=test_bus",
                                    "-DRANGEFINDER_I2C_ADDR=0xA4", f"-DRANGEFINDER_I2C_READ_MODE_SINGLE={sensor_mode}",
                                    f"-DPERIODIC_TELEMETRY={periodic}", *debug_flags,
                                    "-DRANGEFINDER_I2C_USE_FILTER=0", "-DRANGEFINDER_I2C_SCALE=0.001f",
                                    "-DRANGEFINDER_I2C_MIN_RANGE=0.005f", "-DRANGEFINDER_I2C_MAX_RANGE=6.f",
                                    "-DRANGEFINDER_I2C_USE_FOR_AGL=1", "tests/modules/test_rangefinder_i2c_safety.c",
                                    "sw/airborne/modules/sensors/rangefinder_i2c.c", "-lm", "-o", binary], cwd=home, check=True)
                    subprocess.run([binary], check=True)

    def test_talon_aim_stays_within_plan_limits(self):
        home = Path(__file__).resolve().parents[3]
        airframe = ET.parse(home / "conf/airframes/OPENUAS/openuas_zohd_talon_250g.xml").getroot()
        plan = ET.parse(home / "conf/flight_plans/TUDELFT/tudelft_imav2026_o_test_pricise_landing.xml").getroot()
        aim = float(airframe.find("./section[@name='PRECISION_LANDING']/define[@name='AIM_BEFORE_TD']").get("value"))
        setting = plan.find("./variables/variable[@var='aim_before_td']")
        self.assertEqual(aim, 7.0)
        self.assertLessEqual(float(setting.get("min")), aim)
        self.assertGreaterEqual(float(setting.get("max")), aim)

    def test_agl_configuration_and_reacquisition(self):
        home = Path(__file__).resolve().parents[3]
        with tempfile.TemporaryDirectory() as directory:
            binary = str(Path(directory) / "agl")
            subprocess.run(["gcc", "-std=c11", "-Wall", "-Wextra", "-Werror",
                            "-Itests/modules/agl_dist_stubs", "-Itests/modules/precision_landing_stubs",
                            "-Isw/include", "-Isw/airborne", "-DAGL_DIST_SONAR_ID=42",
                            "-DAGL_DIST_SONAR_MAX_RANGE=6.f", "-DAGL_DIST_SONAR_MIN_RANGE=0.005f",
                            "-DAGL_DIST_SONAR_FILTER=0.4f", "tests/modules/test_agl_dist_landing.c",
                            "sw/airborne/modules/sonar/agl_dist.c", "-lm", "-o", binary], cwd=home, check=True)
            subprocess.run([binary], check=True)

    def test_controller(self):
        home = Path(__file__).resolve().parents[3]
        with tempfile.TemporaryDirectory() as directory:
            binary = str(Path(directory) / "controller")
            subprocess.run(["gcc", "-std=c11", "-Wall", "-Wextra", "-Werror",
                            "-O1", "-g", "-fsanitize=address,undefined,float-cast-overflow", "-fno-sanitize-recover=all", "-no-pie",
                            "-Itests/modules/precision_landing_stubs", "-Isw/include", "-Isw/airborne",
                            "tests/modules/test_precision_landing_controller.c",
                            "sw/airborne/modules/nav/precision_landing.c", "-lm", "-o", binary],
                           cwd=home, check=True)
            subprocess.run([binary], check=True)

    def test_crow_output(self):
        home = Path(__file__).resolve().parents[3]
        source = """
#include <assert.h>
#include <math.h>
#include "sw/simulator/nps/nps_crow.h"
int main(void) {
  struct NpsCrowCommands result = nps_crow_commands(-7200, 7200, 9600);
  assert(result.roll == 0 && result.brake == -0.75);
  result = nps_crow_commands(-2400, 9600, 9600);
  assert(result.roll == 0.375 && result.brake == -0.625);
  result = nps_crow_commands(0, 0, 9600);
  assert(result.roll == 0 && result.brake == 0);
  return 0;
}
"""
        with tempfile.TemporaryDirectory() as directory:
            binary = str(Path(directory) / "crow")
            subprocess.run(["gcc", "-std=c11", "-Wall", "-Wextra", "-Werror", "-I.",
                            "-x", "c", "-", "-o", binary], input=source, text=True, cwd=home, check=True)
            subprocess.run([binary], check=True)


if __name__ == "__main__":
    unittest.main()