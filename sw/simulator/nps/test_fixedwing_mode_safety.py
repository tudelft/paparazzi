import re
import subprocess
import tempfile
import unittest
from pathlib import Path


class FixedwingModeSafetyTest(unittest.TestCase):
    def test_ins_reset_reprojects_position_before_home_capture(self):
        home = Path(__file__).resolve().parents[3]
        source = (home / "sw/airborne/modules/ins/ins_alt_float.c").read_text()
        reset = source[source.index("static void reset_ref(void)\n{"):source.index("static void reset_vertical_ref(void)")]
        program = """
#include <assert.h>
#include <math.h>
#include "state.h"
#define MODULE_INS_ALT_FLOAT_ID 1
struct { float alt; bool origin_initialized, reset_alt_ref; } ins_altf;
struct LlaCoor_f gps;
static struct UtmCoor_f utm_float_from_gps(struct LlaCoor_f *position, uint8_t zone) {
  struct UtmCoor_f result = {.zone = zone};
  utm_of_lla_f(&result, position);
  return result;
}
RESET
int main(void) {
  stateInit();
  gps = (struct LlaCoor_f){.lat = RadOfDeg(50.72737), .lon = RadOfDeg(7.56648), .alt = 293.f};
  struct UtmCoor_f old_position = utm_float_from_gps(&gps, 31);
  struct UtmCoor_f new_position = utm_float_from_gps(&gps, 32);
  stateSetLocalUtmOrigin_f(1, &old_position);
  stateSetPositionUtm_f(1, &old_position);
  assert(fabsf(old_position.east - new_position.east) > 400000.f);
  ins_altf.alt = 290.f;
  reset_ref();
  const struct EnuCoor_f *local = stateGetPositionEnu_f();
  assert(hypotf(local->x, local->y) < 0.1f);
  assert(stateGetPositionUtm_f()->zone == stateGetUtmOrigin_f()->zone);
  assert(stateGetPositionUtm_f()->alt == 290.f);
  assert(ins_altf.origin_initialized && ins_altf.reset_alt_ref);
  reset_ref();
  assert(hypotf(stateGetPositionEnu_f()->x, stateGetPositionEnu_f()->y) < 0.1f);
  return 0;
}
""".replace("RESET\n", reset)
        with tempfile.TemporaryDirectory() as directory:
            binary = str(Path(directory) / "utm-reset")
            subprocess.run(["gcc", "-std=c11", "-Wall", "-Wextra", "-O1", "-g",
                            "-ffunction-sections", "-fdata-sections", "-Wl,--gc-sections",
                            "-Isw/airborne", "-Isw/include", "sw/airborne/state.c",
                            "sw/airborne/math/pprz_geodetic_float.c", "sw/airborne/math/pprz_geodetic_double.c",
                            "sw/airborne/math/pprz_geodetic_int.c", "sw/airborne/math/pprz_algebra_float.c",
                            "sw/airborne/math/pprz_algebra_int.c",
                            "-x", "c", "-", "-lm", "-o", binary], cwd=home, input=program, text=True, check=True)
            subprocess.run([binary], check=True)

    def test_real_mode_setters_and_gps_recovery(self):
        home = Path(__file__).resolve().parents[3]
        source = (home / "sw/airborne/firmwares/fixedwing/autopilot_static.c").read_text()
        setters = source[source.index("void autopilot_static_set_mode("):source.index("void autopilot_static_set_motors_on(")]
        failsafe = source[source.index("void autopilot_failsafe_checks(void)"):source.index("void attitude_loop(void)")]
        rc_update = source[source.index("static inline uint8_t pprz_mode_update(void)\n{"):source.index("#else // not RADIO_CONTROL")]
        modes = (home / "sw/airborne/firmwares/fixedwing/autopilot_static.h").read_text()
        definitions = "\n".join(re.findall(r"^#define\s+AP_MODE_.*$", modes, re.MULTILINE))
        program = """
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#define USE_GPS 1
#define FAILSAFE_DELAY_WITHOUT_GPS 5
#define USE_PRECISION_LANDING 1
#define RADIO_CONTROL 1
#define UNLOCKED_HOME_MODE 1
#define SITL 1
#define RC_LOST_MODE AP_MODE_AUTO2
#define RADIO_MODE 0
#define AP_MODE_OF_PULSE(value) (value)
MODE_DEFINITIONS
struct { uint8_t mode; bool launch; } autopilot;
struct { uint32_t nb_sec; } sys_time;
struct { uint32_t last_3dfix_time; } gps;
static uint8_t gps_failsafe_restore_mode;
static bool gps_failsafe_active;
static unsigned mode_changes;
static bool too_far_from_home;
static uint8_t rc_mode = AP_MODE_AUTO2;
static bool rc_lost;
static bool RadioControlIsLost(void) { return rc_lost; }
static uint8_t radio_control_get(uint8_t channel) { (void)channel; return rc_mode; }
static bool datalink_lost(void) { return false; }
static bool higher_than_max_altitude(void) { return false; }
static uint8_t autopilot_get_mode(void) { return autopilot.mode; }
static void precision_landing_on_mode_change(uint8_t mode) { (void)mode; mode_changes++; }
SETTERS
static bool autopilot_set_mode(uint8_t mode) {
  uint8_t previous = autopilot.mode;
  autopilot_static_set_mode(mode);
  return previous != autopilot.mode;
}
RC_UPDATE
FAILSAFE
int main(void) {
  autopilot.launch = true;
  autopilot.mode = AP_MODE_AUTO2;
  sys_time.nb_sec = 20;
  gps.last_3dfix_time = 0;
  autopilot_failsafe_checks();
  assert(autopilot.mode == AP_MODE_GPS_OUT_OF_ORDER && gps_failsafe_active);
  gps.last_3dfix_time = 20;
  autopilot_failsafe_checks();
  assert(autopilot.mode == AP_MODE_AUTO2 && !gps_failsafe_active);
  for (uint8_t selected = AP_MODE_MANUAL; selected <= AP_MODE_HOME; selected++) {
    autopilot_static_set_mode(AP_MODE_AUTO2);
    gps.last_3dfix_time = 0;
    autopilot_failsafe_checks();
    autopilot_static_set_mode(selected);
    rc_mode = selected;
    gps.last_3dfix_time = 20;
    autopilot_failsafe_checks();
    assert(autopilot.mode == selected && !gps_failsafe_active);
  }
  autopilot_static_set_mode(AP_MODE_MANUAL);
  unsigned previous_changes = mode_changes;
  const float invalid[] = {NAN, INFINITY, -INFINITY, -1.f, 0.5f, 5.f, 256.f, 1e30f};
  for (unsigned index = 0; index < sizeof(invalid) / sizeof(invalid[0]); index++) {
    autopilot_static_SetModeHandler(invalid[index]);
    assert(autopilot.mode == AP_MODE_MANUAL && mode_changes == previous_changes);
  }
  autopilot_static_set_mode(255);
  assert(autopilot.mode == AP_MODE_MANUAL);
  autopilot_static_SetModeHandler(AP_MODE_AUTO2);
  assert(autopilot.mode == AP_MODE_AUTO2 && mode_changes == previous_changes + 1);
  too_far_from_home = true;
  gps.last_3dfix_time = sys_time.nb_sec;
  for (uint8_t selected = AP_MODE_MANUAL; selected <= AP_MODE_AUTO2; selected++) {
    rc_mode = selected;
    autopilot_failsafe_checks();
    const uint8_t expected = selected == AP_MODE_AUTO2 ? AP_MODE_HOME : selected;
    assert(autopilot.mode == expected);
    unsigned settled_changes = mode_changes;
    for (unsigned cycle = 0; cycle < 1000; cycle++) {
      autopilot_failsafe_checks();
      assert(autopilot.mode == expected && mode_changes == settled_changes);
    }
  }
  rc_mode = AP_MODE_AUTO1;
  autopilot_failsafe_checks();
  rc_lost = true;
  autopilot_failsafe_checks();
  assert(autopilot.mode == AP_MODE_HOME);
  rc_lost = false;
  autopilot_failsafe_checks();
  assert(autopilot.mode == AP_MODE_AUTO1);
  too_far_from_home = false;
  rc_mode = AP_MODE_AUTO2;
  gps.last_3dfix_time = 0;
  autopilot_failsafe_checks();
  assert(autopilot.mode == AP_MODE_GPS_OUT_OF_ORDER);
  previous_changes = mode_changes;
  for (unsigned cycle = 0; cycle < 1000; cycle++) { autopilot_failsafe_checks(); }
  assert(mode_changes == previous_changes);
  rc_mode = AP_MODE_MANUAL;
  autopilot_failsafe_checks();
  gps.last_3dfix_time = sys_time.nb_sec;
  autopilot_failsafe_checks();
  assert(autopilot.mode == AP_MODE_MANUAL);
  return 0;
}
""".replace("MODE_DEFINITIONS", definitions).replace("SETTERS", setters).replace("RC_UPDATE", rc_update).replace("FAILSAFE\n", failsafe)
        with tempfile.TemporaryDirectory() as directory:
            for landing_enabled, two_switches in ((True, False), (False, False), (True, True), (False, True)):
                with self.subTest(landing_enabled=landing_enabled, two_switches=two_switches):
                    variant = program
                    if two_switches:
                        variant = variant.replace("#define RADIO_MODE 0", "#define RADIO_MODE 0\n#define RADIO_AUTO_MODE 1\n#define THRESHOLD2 0\n#define INFO(message)")
                        variant = variant.replace("(void)channel; return rc_mode;", "return channel == RADIO_AUTO_MODE ? (rc_mode == AP_MODE_AUTO2) : rc_mode;")
                        variant = variant.replace("selected <= AP_MODE_HOME", "selected <= AP_MODE_AUTO2")
                    if not landing_enabled:
                        variant = variant.replace("#define USE_PRECISION_LANDING 1", "#define USE_PRECISION_LANDING 0")
                        variant = variant.replace("static void precision_landing_on_mode_change(uint8_t mode) { (void)mode; mode_changes++; }", "")
                        variant = variant.replace("autopilot.mode = new_autopilot_mode;", "mode_changes++; autopilot.mode = new_autopilot_mode;")
                    binary = str(Path(directory) / f"mode-test-{landing_enabled}-{two_switches}")
                    subprocess.run(["gcc", "-std=c11", "-Wall", "-Wextra", "-Werror",
                                    "-fsanitize=undefined,float-cast-overflow", "-fno-sanitize-recover=all",
                                    "-x", "c", "-", "-lm", "-o", binary], input=variant, text=True, check=True)
                    subprocess.run([binary], check=True)


if __name__ == "__main__":
    unittest.main()