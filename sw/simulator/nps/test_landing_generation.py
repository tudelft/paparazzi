import os
import subprocess
import tempfile
import unittest
from pathlib import Path
import xml.etree.ElementTree as ET


class LandingGenerationTest(unittest.TestCase):
    def test_module_removal_and_validated_mode_setting(self):
        home = Path(__file__).resolve().parents[3]
        with tempfile.TemporaryDirectory(prefix="landing-generation-", dir=home / "var") as directory:
            work = Path(directory)
            airframe = ET.parse(home / "conf/airframes/OPENUAS/openuas_zohd_talon_250g.xml")
            firmware = airframe.getroot().find("./firmware[@name='fixedwing']")
            extra = ET.SubElement(firmware, "module", name="auto1_commands")
            airframe_path = work / "airframe.xml"
            airframe.write(airframe_path)
            conf = ET.parse(home / "conf/userconf/OPENUAS/openuas_precision_landing_test_conf.xml")
            aircraft = conf.getroot().find("aircraft")
            aircraft.set("name", "LandingGenerationTest")
            aircraft.set("airframe", os.path.relpath(airframe_path, home / "conf"))
            conf_path = work / "conf.xml"
            conf.write(conf_path)
            environment = dict(os.environ, PAPARAZZI_HOME=str(home), PAPARAZZI_SRC=str(home),
                               PPRZLINK_DIR=str(home / "sw/ext/pprzlink/message_definitions/v1.0"))
            command = [str(home / "sw/tools/generators/gen_aircraft.out"), "-name", "LandingGenerationTest",
                       "-target", "nps", "-conf", str(conf_path), "-all",
                       "-ac_dir", str(work / "build"), "-conf_dir", str(work / "config")]
            subprocess.run(command, cwd=home, env=environment, capture_output=True, text=True, check=True)
            generated = work / "build/generated"
            self.assertIn("periodic_auto1_commands();", (generated / "modules.h").read_text())
            self.assertIn("auto1_commands", (generated / "settings.h").read_text())
            firmware.remove(extra)
            airframe.write(airframe_path)
            subprocess.run(command, cwd=home, env=environment, capture_output=True, text=True, check=True)
            self.assertNotIn("auto1_commands", (generated / "modules.h").read_text())
            settings = (generated / "settings.h").read_text()
            self.assertNotIn("auto1_commands", settings)
            self.assertIn("autopilot_SetModeHandler( _value )", settings)
            self.assertNotIn("autopilot.mode = _value", settings)
            plan = (generated / "flight_plan.h").read_text()
            self.assertNotIn("extern float landing_max_retries", plan)
            self.assertNotIn("extern uint8_t landing_retry_count", plan)
            self.assertIn("precision_landing_bench_run()", plan)
            self.assertIn("precision_landing_reset_retries()", plan)
            self.assertIn("precision_landing_record_retry()", plan)
            self.assertIn("precision_landing_retry_allowed()", plan)
            settings = (generated / "settings.h").read_text()
            self.assertIn("precision_landing_approach_airspeed", settings)
            self.assertIn("precision_landing_max_retries", settings)
            standby = plan[plan.index("Block(5) // Standby"):plan.index("Block(6) // Oval")]
            self.assertIn("v_ctl_auto_airspeed_setpoint = NOMINAL_AIRSPEED", standby)
            self.assertIn("nav_radius = DEFAULT_CIRCLE_RADIUS", standby)
            self.assertLess(standby.index("nav_radius ="), standby.index("NavCircleWaypoint"))
            self.assertLess(plan.index("!(precision_landing_parameters_valid())"),
                            plan.index("WaypointAlt(WP_AF) ="))


if __name__ == "__main__":
    unittest.main()