import math
import unittest
from pathlib import Path
import xml.etree.ElementTree as ET

import jsbsim


class EasyStar3GroundContactTest(unittest.TestCase):
    def test_belly_landing_stops_without_rebound(self):
        root = Path(__file__).resolve().parents[3] / "conf/simulator/jsbsim"
        name = "openuas_jsbsim_multiplex_easystar3"
        contacts = ET.parse(root / "aircraft" / f"{name}.xml").getroot().findall("./ground_reactions/contact")
        belly = {index for index, contact in enumerate(contacts) if contact.get("name", "").startswith("BELLY")}
        model = jsbsim.FGFDMExec(str(root))
        model.set_debug_level(0)
        model.set_aircraft_path(str(root / "aircraft"))
        self.assertTrue(model.load_model(name, False))
        timestep = 0.0005
        model.set_dt(timestep)
        initial = {"ic/h-agl-ft": 0.20 / 0.3048, "ic/u-fps": 11.5 / 0.3048,
                   "ic/w-fps": 1.36 / 0.3048, "ic/theta-deg": -1.4, "ic/terrain-elevation-ft": 0}
        for key, value in initial.items():
            model[key] = value
        self.assertTrue(model.run_ic())
        bases = [f"contact/unit[{index}]" if index else "contact/unit" for index in range(len(contacts))]
        first_contact = None
        peak_height_after_contact = 0.0
        peak_pitch_after_contact = 0.0
        peak_roll_after_contact = 0.0
        seen = set()
        for step in range(math.ceil(5 / timestep)):
            self.assertTrue(model.run())
            touching = {index for index, base in enumerate(bases) if model[base + "/WOW"]}
            seen.update(touching)
            if touching and first_contact is None:
                first_contact = (model["position/h-agl-ft"] * 0.3048,
                                 model["position/distance-from-start-lat-mt"],
                                 model["position/distance-from-start-lon-mt"])
                peak_height_after_contact = first_contact[0]
            if first_contact is not None:
                peak_height_after_contact = max(peak_height_after_contact,
                                                model["position/h-agl-ft"] * 0.3048)
                peak_pitch_after_contact = max(peak_pitch_after_contact, abs(model["attitude/theta-deg"]))
                peak_roll_after_contact = max(peak_roll_after_contact, abs(model["attitude/phi-deg"]))
        self.assertIsNotNone(first_contact)
        self.assertTrue(seen & belly)
        self.assertLessEqual(peak_height_after_contact - first_contact[0], 0.01)
        stop_distance = math.hypot(model["position/distance-from-start-lat-mt"] - first_contact[1],
                                   model["position/distance-from-start-lon-mt"] - first_contact[2])
        self.assertLessEqual(stop_distance, 0.8)
        self.assertLess(model["velocities/vg-fps"] * 0.3048, 0.1)
        self.assertLess(peak_pitch_after_contact, 10.0)
        self.assertLess(peak_roll_after_contact, 8.0)


if __name__ == "__main__":
    unittest.main()