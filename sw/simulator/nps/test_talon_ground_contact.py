import math
import unittest
from pathlib import Path
import xml.etree.ElementTree as ET

import jsbsim


class TalonGroundContactTest(unittest.TestCase):
    def test_drop_and_slide(self):
        root = Path(__file__).resolve().parents[3] / "conf/simulator/jsbsim"
        name = "openuas_jsbsim_zohd_talon_250g"
        contacts = ET.parse(root / "aircraft" / f"{name}.xml").getroot().findall("./ground_reactions/contact")
        belly = {index for index, contact in enumerate(contacts) if contact.get("name").startswith("BELLY")}
        for timestep in (0.0001, 0.0005, 1 / 550):
            for speed in (0.0, 8.5):
                with self.subTest(timestep=timestep, speed=speed):
                    model = jsbsim.FGFDMExec(str(root))
                    model.set_debug_level(0)
                    model.set_aircraft_path(str(root / "aircraft"))
                    self.assertTrue(model.load_model(name, False))
                    model.set_dt(timestep)
                    initial = {"ic/h-agl-ft": 0.15 / 0.3048, "ic/u-fps": speed / 0.3048,
                               "ic/w-fps": (1.1 if speed else 0) / 0.3048,
                               "ic/theta-deg": 1.0, "ic/terrain-elevation-ft": 0}
                    for key, value in initial.items():
                        model[key] = value
                    self.assertTrue(model.run_ic())
                    bases = [f"contact/unit[{index}]" if index else "contact/unit"
                             for index in range(len(contacts))]
                    seen = set()
                    peak_compression = 0.0
                    for step in range(math.ceil(8 / timestep)):
                        self.assertTrue(model.run())
                        for index, base in enumerate(bases):
                            if model[base + "/WOW"]:
                                seen.add(index)
                            peak_compression = max(peak_compression, model[base + "/compression-ft"] * 0.3048)
                    self.assertTrue(seen)
                    self.assertTrue(seen <= belly, f"Unexpected contacts: {seen - belly}")
                    self.assertLess(peak_compression, 0.012)
                    self.assertLess(model["velocities/vg-fps"] * 0.3048, 0.1)
                    self.assertLess(abs(model["attitude/phi-deg"]), 0.5)
                    self.assertTrue(math.isfinite(model["position/h-agl-ft"]))
                    for index in belly:
                        self.assertLess(model[bases[index] + "/compression-ft"] * 0.3048, 0.002)


if __name__ == "__main__":
    unittest.main()