"""Build mission4_orbit_strasbourg.xml from mission4_star_strasbourg.xml.
Every edit must match exactly once, or the script stops."""
import os, re, sys

MS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))  # conf/flight_plans/MS
SRC = os.path.join(MS, "mission4_star_strasbourg.xml")
DST = os.path.join(MS, "mission4_orbit_strasbourg.xml")
s = open(SRC).read()

def rep(old, new, count=1):
    global s
    n = s.count(old)
    if n != count:
        sys.exit(f"expected {count} match(es), found {n}: {old[:80]!r}")
    s = s.replace(old, new)

def splice(start_marker, end_marker, new):
    """Replace from start_marker (inclusive) to end_marker (exclusive)."""
    global s
    a = s.find(start_marker); b = s.find(end_marker, a + 1)
    if a == -1 or b == -1 or s.count(start_marker) != 1:
        sys.exit(f"splice markers not found/unique: {start_marker[:60]!r} .. {end_marker[:60]!r}")
    s = s[:a] + new + s[b:]

def c_safe(chunk):
    bad = [c for c in "<>&" if c in chunk]
    if bad:
        sys.exit(f"header C chunk contains {bad}: xml-light cannot take these")
    return chunk

# ---------------------------------------------------------------- top comment
TOP = """<!--
  IMAV Mission4 (Strasbourg) - ORBIT search, obstacle-aware drop.

  A copy of mission4_star_strasbourg.xml with the star search replaced by an
  orbit. The site model, obstacle sectors, geofence and the whole drop (lane
  run-in, release gate, climb-out) are that file's - see its header for the
  LiDAR survey, the lane, and the fence discrepancy still to be settled with
  the organisers. mission4_star_strasbourg.xml is left untouched so either
  plan can be flown (aircraft CherryStar vs CherryOrbit).

  WHY AN ORBIT INSTEAD OF THE STAR
    Tangent turns force the star's legs out to 60 m either side of the
    centre and its loops to 90 m, to search a 25 m-radius area, and only
    about 37% of its time is spent listening. At 45 m, directly overhead is
    also where SPL says least about horizontal position. A 25 m circle round
    the organisers' point stays inside the area, listens about 70% of the
    time, and never needs recentring: by rule the source is within 25 m.
    Monte Carlo with the noise model below, source anywhere in the 25 m
    disc, median estimate error: star 9.3 m, orbit 5.5 m after 75 s; star
    7.2 m, orbit 4.2 m after 150 s. Neither reaches 2 m from 45 m.

  HOW IT FLIES
    Each cycle is MISSION4_ORBIT_QUIET_M of motor-off glide (listening,
    after MISSION4_ORBIT_SPINDOWN s of prop spin-down, ended early at
    orbit_floor) then powered flight for at least MISSION4_ORBIT_POWER_M,
    until back at orbit_height (climbing back the ~10 m lost) and until the
    next start bearing: each arc starts MISSION4_ORBIT_STEP_DEG further round
    than the last, so the arcs sweep the whole circle whatever the climb
    rate. Listening continues past orbit_time until every 10 deg sector has
    been heard (at most MISSION4_ORBIT_EXTRA_S more). Then it solves while
    still circling, checks plausibility, and hands over to the drop.

  WHAT ELSE CHANGED FROM THE STAR FILE
    - Motor genuinely off while listening. The star zeroed only the energy
      controller's nominal term and the controller added 3-33% back
      (measured in a CherryStar log). Here every throttle term is zeroed for
      the arc and restored after it, from the flight plan alone - no
      firmware change. A GPS-driven watchdog restores them if the plan stops
      renewing the request, which also covers HOME mode, where the flight
      plan does not run at all. Samples are only kept while the throttle
      command actually is zero.
    - Sampling timed on get_sys_time_float(). autopilot.flight_time is a
      whole-second counter, so the star's 20 Hz was really 1 Hz.
    - The solver fits a free dB offset per listening arc (absorbs slow mic
      drift and an uncalibrated beeper); the linear solver is the fallback.
    - A failed search holds after one retry. The star's retry counter was
      reset by the retry itself, so it could loop forever.
    - The base-turn exit tests the bearing of START from the turn centre.
      The star's test was 90 deg off and left the turn a quarter-turn late.
    - Drop Exit climbs at the climb-out rate and is excluded from the
      obstacle escape: it was already climbing, and the escape only turned it
      back towards _CLIMBOUT (seen in a CherryStar log).

  STILL PLACEHOLDER
    mission4_sample_signal() synthesises SPL from TRUE_SOUND_SOURCE with the
    star file's noise model: 1.5 dB white + 1.5 dB drift, assumed, not
    measured, and no prop or wind noise. DropOpen() is a no-op.
-->
"""
assert "--" not in TOP[4:-4], "double hyphen inside XML comment"
splice("<!--\n  IMAV Mission4 (Strasbourg) - CLOVERLEAF STAR search", "<flight_plan alt=", TOP)
rep('name="Mission4_Star_Strasbourg"', 'name="Mission4_Orbit_Strasbourg"')

# ---------------------------------------------------------------- includes, sonar clock
rep('#include "modules/core/abi.h"\n',
    '#include "modules/core/abi.h"\n#include "mcu_periph/sys_time.h"\n#include "modules/gps/gps.h"\n')
rep("  mission4_sonar_agl_time = autopilot.flight_time;",
    "  mission4_sonar_agl_time = get_sys_time_float();")
rep("  if (!signbit(autopilot.flight_time - mission4_sonar_agl_time - 0.5f)) {",
    "  if (!signbit(get_sys_time_float() - mission4_sonar_agl_time - 0.5f)) {")
rep("Above this height nothing at the site can be hit. The star flies here.",
    "Above this height nothing at the site can be hit. The orbit flies here.")

# ---------------------------------------------------------------- SITL noise scale (default 1 = unchanged)
rep("static float mission4_spl_drift = 0.0f;\n",
    "static float mission4_spl_drift = 0.0f;\n"
    "/* SITL only: M4_NOISE_SCALE multiplies the whole SPL error, 0 = noiseless,\n"
    "   so a run can separate estimate error from drop error. Default 1 keeps\n"
    "   this block equivalent to mission4_strasbourg.xml's. */\n"
    "static float mission4_noise_scale = 1.0f;\n"
    "/* SITL only: M4_NO_SOURCE=1 removes the beeper entirely - the microphone\n"
    "   hears only background at M4_BACKGROUND_DB (default 50 dB) plus the same\n"
    "   noise - to see what the search does when there is nothing to find. */\n"
    "static bool mission4_no_source = FALSE;\n"
    "static float mission4_background_db = 50.0f;\n")
rep("  else { seed = (uint32_t)time(NULL); }\n",
    "  else { seed = (uint32_t)time(NULL); }\n"
    "  const char *sc = getenv(\"M4_NOISE_SCALE\");\n"
    "  if (sc != NULL) { mission4_noise_scale = (float)atof(sc); }\n"
    "  const char *ns = getenv(\"M4_NO_SOURCE\");\n"
    "  if (ns != NULL) { mission4_no_source = (atoi(ns) != 0); }\n"
    "  const char *bg = getenv(\"M4_BACKGROUND_DB\");\n"
    "  if (bg != NULL) { mission4_background_db = (float)atof(bg); }\n")
rep('tau=%.1f s\\n",', 'tau=%.1f s scale=%.2f no_source=%d background=%.0f dB\\n",')
rep("MISSION4_SPL_DRIFT_TAU);", "MISSION4_SPL_DRIFT_TAU, (double)mission4_noise_scale, (int)mission4_no_source, (double)mission4_background_db);")
rep("  return mission4_spl_drift + MISSION4_SPL_NOISE_DB * mission4_randn();",
    "  return mission4_noise_scale * (mission4_spl_drift + MISSION4_SPL_NOISE_DB * mission4_randn());")

# ---------------------------------------------------------------- sample buffer
SAMPLES = c_safe("""/* ===================================================================
   Acoustic sampling. SPL model and noise block unchanged from
   mission4_star_strasbourg.xml so the two plans stay comparable. Changed:
   timed on get_sys_time_float() (autopilot.flight_time counts whole
   seconds, which made the old 20 Hz really 1 Hz); every sample records the
   listening arc it came from, for the per-arc solver; and there is no
   best-sample tracking into DROP_POINT any more - only the solver writes it.

   2000 samples is 34 KB. 150 s of orbit keeps about 1780 at 20 Hz, and
   orbit_time is capped at 160 s so the buffer is never the limit.

   The buffer lives in the F4's 64 KB core-coupled RAM (.ram4), which
   nothing else in this build uses: in main RAM it left the heap only
   26 KB (ap build, checked), against about 50 KB with the star's
   600-sample buffer. CCM is CPU-only, which is all these arrays need, and
   NOINIT is fine because mission4_sample_count gates every read.
   =================================================================== */
#define MISSION4_MAX_SAMPLES 2000
#define MISSION4_SAMPLE_PERIOD 0.05f
#define MISSION4_KTR_REF_SPL 95.0f
#define MISSION4_KTR_REF_DIST 3.0f
#define MISSION4_MAX_ARCS 40

#ifdef SITL
#define MISSION4_CCM(var) var
#else
#include "mcu_periph/ram_arch.h"
#define MISSION4_CCM(var) IN_FAST_SECTION_NOINIT(var)
#endif
MISSION4_CCM(static float mission4_sample_x[MISSION4_MAX_SAMPLES]);
MISSION4_CCM(static float mission4_sample_y[MISSION4_MAX_SAMPLES]);
MISSION4_CCM(static float mission4_sample_spl[MISSION4_MAX_SAMPLES]);
MISSION4_CCM(static float mission4_sample_z[MISSION4_MAX_SAMPLES]);
MISSION4_CCM(static uint8_t mission4_sample_arc[MISSION4_MAX_SAMPLES]);
static uint16_t mission4_sample_count = 0;
static float mission4_last_sample_time = -1000.0f;
""")
splice("/* ===================================================================\n   Acoustic sampling and the multilateration solver.",
       "\n/* ---- SPL MEASUREMENT NOISE", SAMPLES)

# ---------------------------------------------------------------- sampler + quiet + orbit (replaces star)
ORBIT = c_safe(r"""static inline bool mission4_sample_signal(uint8_t source_wp, uint8_t arc)
{
  float now = get_sys_time_float();
  if (!signbit(now - mission4_last_sample_time - MISSION4_SAMPLE_PERIOD)) {
    mission4_last_sample_time = now;
    float dx = GetPosX() - waypoints[source_wp].x;
    float dy = GetPosY() - waypoints[source_wp].y;
    float dz = GetPosAlt() - GetAltRef();
    float dist = sqrtf(dx * dx + dy * dy + dz * dz);
    float spl = MISSION4_KTR_REF_SPL - 20.0f * log10f(dist / MISSION4_KTR_REF_DIST);
#ifdef SITL
    /* M4_NO_SOURCE=1: no beeper at all, only background. */
    if (mission4_no_source) { spl = mission4_background_db; }
#endif
    /* Corrupt the reading BEFORE anything downstream sees it. */
    spl += mission4_spl_noise();
    if (signbit((float)mission4_sample_count - (float)MISSION4_MAX_SAMPLES)) {
      mission4_sample_x[mission4_sample_count] = GetPosX();
      mission4_sample_y[mission4_sample_count] = GetPosY();
      mission4_sample_spl[mission4_sample_count] = spl;
      mission4_sample_z[mission4_sample_count] = dz;
      mission4_sample_arc[mission4_sample_count] = arc;
      mission4_sample_count++;
    }
  }
  return false;
}

/* ===================================================================
   MOTOR-OFF LISTENING, FROM THE FLIGHT PLAN ALONE

   The energy controller's throttle is nominal + climb increment x climb
   setpoint + airspeed P + energy P + two bank terms, and its integrator
   rewrites nominal every control step. Zeroing nominal alone (the star
   file's approach) left 3-33% throttle in the listening window of a
   CherryStar log. Zeroing every term makes the output exactly 0, which
   takes the controller's existing out-of-throttle branch: pitch holds
   airspeed and the climb demand is dropped - a proper glide.

   The terms are saved when an arc starts and restored when it ends,
   integrator state included, so power comes back where it was.

   Safety net: the request must be renewed at least every
   MISSION4_QUIET_HOLD_S (mission4_quiet_on, every nav cycle). A GPS Abi
   callback restores the gains once the request is stale. GPS arrives in
   every autopilot mode, so this also covers HOME mode (geofence breach, RC
   loss), which runs nav_home() instead of the flight plan and would
   otherwise leave the gains at zero. The global exception
   mission4_quiet_guard() does the same from every flight-plan block, in
   case GPS stops. A gain changed from the GCS during an arc is overwritten
   when the arc ends.
   =================================================================== */
#define MISSION4_QUIET_HOLD_S 0.3f
#define MISSION4_QUIET_NGAINS 8

static float mission4_quiet_saved[MISSION4_QUIET_NGAINS];
static bool mission4_quiet_active = FALSE;
static float mission4_quiet_until = 0.0f;

static inline void mission4_quiet_off(void)
{
  if (!mission4_quiet_active) { return; }
  v_ctl_auto_throttle_nominal_cruise_throttle = mission4_quiet_saved[0];
  v_ctl_auto_throttle_climb_throttle_increment = mission4_quiet_saved[1];
  v_ctl_auto_throttle_of_airspeed_pgain = mission4_quiet_saved[2];
  v_ctl_auto_throttle_of_airspeed_igain = mission4_quiet_saved[3];
  v_ctl_energy_total_pgain = mission4_quiet_saved[4];
  v_ctl_energy_total_igain = mission4_quiet_saved[5];
  v_ctl_energy_bank_throttle_gain = mission4_quiet_saved[6];
  v_ctl_energy_bank_washout_gain = mission4_quiet_saved[7];
  mission4_quiet_active = FALSE;
}

static inline void mission4_quiet_on(void)
{
  if (!mission4_quiet_active) {
    mission4_quiet_saved[0] = v_ctl_auto_throttle_nominal_cruise_throttle;
    mission4_quiet_saved[1] = v_ctl_auto_throttle_climb_throttle_increment;
    mission4_quiet_saved[2] = v_ctl_auto_throttle_of_airspeed_pgain;
    mission4_quiet_saved[3] = v_ctl_auto_throttle_of_airspeed_igain;
    mission4_quiet_saved[4] = v_ctl_energy_total_pgain;
    mission4_quiet_saved[5] = v_ctl_energy_total_igain;
    mission4_quiet_saved[6] = v_ctl_energy_bank_throttle_gain;
    mission4_quiet_saved[7] = v_ctl_energy_bank_washout_gain;
    v_ctl_auto_throttle_nominal_cruise_throttle = 0.f;
    v_ctl_auto_throttle_climb_throttle_increment = 0.f;
    v_ctl_auto_throttle_of_airspeed_pgain = 0.f;
    v_ctl_auto_throttle_of_airspeed_igain = 0.f;
    v_ctl_energy_total_pgain = 0.f;
    v_ctl_energy_total_igain = 0.f;
    v_ctl_energy_bank_throttle_gain = 0.f;
    v_ctl_energy_bank_washout_gain = 0.f;
    mission4_quiet_active = TRUE;
  }
  mission4_quiet_until = get_sys_time_float() + MISSION4_QUIET_HOLD_S;
}

static inline void mission4_quiet_check(void)
{
  if (mission4_quiet_active) {
    if (!signbit(get_sys_time_float() - mission4_quiet_until)) { mission4_quiet_off(); }
  }
}

static abi_event mission4_gps_ev[1];

static void mission4_quiet_gps_cb(uint8_t sender_id __attribute__((unused)),
                                  uint32_t stamp __attribute__((unused)),
                                  struct GpsState *gps_s __attribute__((unused)))
{
  mission4_quiet_check();
}

static inline void mission4_quiet_init(void)
{
  AbiBindMsgGPS(ABI_BROADCAST, mission4_gps_ev, mission4_quiet_gps_cb);
}

/* Side-effecting condition for the global exception: always false. */
static inline bool mission4_quiet_guard(void)
{
  mission4_quiet_check();
  return FALSE;
}

/* ===================================================================
   ORBIT SEARCH

   A circle of MISSION4_ORBIT_R round the organisers' point, cycling
   MISSION4_ORBIT_QUIET_M of motor-off glide and MISSION4_ORBIT_POWER_M of
   powered climb-back. The 160 m cycle is not a whole lap (157 m), so the
   listening arcs creep round, and with each arc covering 70% of a lap every
   bearing gets listened to. 25 m at 10 m/s is about 22 deg of bank: the
   same as the star's turns, and below the airframe's declared
   MIN_CIRCLE_RADIUS of 30 m, as the star already was.

   Distance round the orbit is integrated from ground speed in the block's
   pre_call, so the cycle does not depend on how the circle was joined. It
   starts powered, which gives the aircraft 50 m to settle onto the circle.
   Arc ids start at 1 and step per listening arc; the solver gives each arc
   its own dB offset.
   =================================================================== */
#define MISSION4_ORBIT_R 25.0f
#define MISSION4_ORBIT_QUIET_M 110.0f
#define MISSION4_ORBIT_POWER_M 50.0f
#define MISSION4_ORBIT_SPINDOWN 1.5f
#define MISSION4_ORBIT_AIRSPEED 10.0f
#define MISSION4_CRUISE_THR 0.16f
#define MISSION4_PLAUSIBLE_M 35.0f
#define MISSION4_SOLVE_MAX_ITER 20
#define MISSION4_SOLVE_MIN_SAMPLES 40.0f
/* 20/ln(10): for SPL = c - 10 log10(d^2), dSPL/dX = K (x - X) / d^2. */
#define MISSION4_K_DB 8.6858896f

static float mission4_orbit_t0 = 0.0f;
static float mission4_orbit_last_t = -1.0f;
static float mission4_orbit_dist = 0.0f;
static float mission4_orbit_run = 0.0f;
static float mission4_orbit_quiet_t = 0.0f;
static bool mission4_orbit_quiet = FALSE;
static uint8_t mission4_arc = 0;
static bool mission4_orbit_done = FALSE;
static bool mission4_orbit_ok = FALSE;
static uint16_t mission4_thr_rejects = 0;

/* Sweep: each listening arc starts MISSION4_ORBIT_STEP_DEG further round
   than the last, whatever the climb rate, and the orbit is not solved
   until every one of MISSION4_COV_BINS sectors has been heard (or
   MISSION4_ORBIT_EXTRA_S after orbit_time). Left to the climb-back alone
   the shift was ~50 deg per cycle in SITL, but a stronger climb could cut
   it to ~7 deg and leave a sector unheard for the whole search. The
   simulated noise is the same in every direction, so a gap cost little
   there; real noise is not (upwind side, reflections, attitude to the
   beeper), and an always-missed sector could bias the estimate. */
#define MISSION4_ORBIT_STEP_DEG 40.0f
#define MISSION4_ORBIT_START_WINDOW_DEG 20.0f
#define MISSION4_COV_BINS 36
#define MISSION4_ORBIT_EXTRA_S 60.0f
#define MISSION4_PI 3.14159265f

static float mission4_orbit_cx = 0.0f;
static float mission4_orbit_cy = 0.0f;
static float mission4_orbit_target = 0.0f;
static bool mission4_orbit_target_set = FALSE;
static uint8_t mission4_cov[MISSION4_COV_BINS];

/* Bearing of the aircraft from the orbit centre: radians, clockwise from
   north, in [0, 2 pi). */
static inline float mission4_orbit_bearing(void)
{
  float b = atan2f(GetPosX() - mission4_orbit_cx, GetPosY() - mission4_orbit_cy);
  return fmodf(b + 2.0f * MISSION4_PI, 2.0f * MISSION4_PI);
}

static inline uint8_t mission4_cov_count(void)
{
  uint8_t k, c = 0;
  for (k = 0; k != MISSION4_COV_BINS; k++) { c += mission4_cov[k]; }
  return c;
}

/* True when a listening arc may start here: at the planned start bearing,
   or up to MISSION4_ORBIT_START_WINDOW_DEG past it. If the aircraft is
   already further past, the plan steps on to the next start bearing ahead
   of it instead of waiting a whole lap. The first arc starts wherever the
   aircraft is and fixes the sequence. The circle is flown clockwise
   (positive radius), so bearings increase. */
static inline bool mission4_orbit_at_start_bearing(void)
{
  float brg = mission4_orbit_bearing();
  float step = MISSION4_ORBIT_STEP_DEG * MISSION4_PI / 180.0f;
  float win = MISSION4_ORBIT_START_WINDOW_DEG * MISSION4_PI / 180.0f;
  if (!mission4_orbit_target_set) {
    mission4_orbit_target = brg;
    mission4_orbit_target_set = TRUE;
  }
  /* Signed angle the aircraft is past the target, in [-pi, pi). */
  float d = fmodf(brg - mission4_orbit_target + 5.0f * MISSION4_PI, 2.0f * MISSION4_PI) - MISSION4_PI;
  uint8_t guard = 0;
  while (!signbit(d - win)) {
    mission4_orbit_target = fmodf(mission4_orbit_target + step, 2.0f * MISSION4_PI);
    d = fmodf(brg - mission4_orbit_target + 5.0f * MISSION4_PI, 2.0f * MISSION4_PI) - MISSION4_PI;
    guard++;
    if (guard == 12) { break; }
  }
  if (signbit(d)) { return FALSE; }
  if (!signbit(d - win)) { return FALSE; }
  mission4_orbit_target = fmodf(mission4_orbit_target + step, 2.0f * MISSION4_PI);
  return TRUE;
}

static float mission4_est_x = 0.0f;
static float mission4_est_y = 0.0f;
static uint8_t mission4_solve_iter = 0;
static bool mission4_solving = FALSE;
static bool mission4_solve_good = FALSE;
static float mission4_arc_r[MISSION4_MAX_ARCS];
static float mission4_arc_jx[MISSION4_MAX_ARCS];
static float mission4_arc_jy[MISSION4_MAX_ARCS];
static uint16_t mission4_arc_n[MISSION4_MAX_ARCS];

/* One Gauss-Newton step of the per-arc fit
     spl_i = c_arc(i) - 10 log10((x_i - X)^2 + (y_i - Y)^2 + z_i^2)
   with the offsets eliminated exactly: for a given X,Y each c is its arc's
   mean residual, so residuals and Jacobian are centred per arc and only a
   2x2 remains. The offsets make the fit blind to a constant level within an
   arc - slow mic drift, an uncalibrated beeper - so it works from how the
   level CHANGES along each arc. One step per nav cycle keeps the cost on
   the autopilot to two passes over the buffer per cycle.
   Returns TRUE while another step is wanted. */
static inline bool mission4_solve_step(void)
{
  uint16_t n = mission4_sample_count;
  uint16_t i;
  uint8_t k;
  for (k = 0; k != MISSION4_MAX_ARCS; k++) {
    mission4_arc_r[k] = 0.f;
    mission4_arc_jx[k] = 0.f;
    mission4_arc_jy[k] = 0.f;
    mission4_arc_n[k] = 0;
  }
  for (i = 0; i != n; i++) {
    k = mission4_sample_arc[i];
    float dx = mission4_sample_x[i] - mission4_est_x;
    float dy = mission4_sample_y[i] - mission4_est_y;
    float z = mission4_sample_z[i];
    float d2 = dx * dx + dy * dy + z * z;
    mission4_arc_r[k] += mission4_sample_spl[i] + 10.0f * log10f(d2);
    mission4_arc_jx[k] += MISSION4_K_DB * dx / d2;
    mission4_arc_jy[k] += MISSION4_K_DB * dy / d2;
    mission4_arc_n[k]++;
  }
  for (k = 0; k != MISSION4_MAX_ARCS; k++) {
    if (mission4_arc_n[k] != 0) {
      float c = (float)mission4_arc_n[k];
      mission4_arc_r[k] /= c;
      mission4_arc_jx[k] /= c;
      mission4_arc_jy[k] /= c;
    }
  }
  double a11 = 0.0, a12 = 0.0, a22 = 0.0, b1 = 0.0, b2 = 0.0;
  for (i = 0; i != n; i++) {
    k = mission4_sample_arc[i];
    float dx = mission4_sample_x[i] - mission4_est_x;
    float dy = mission4_sample_y[i] - mission4_est_y;
    float z = mission4_sample_z[i];
    float d2 = dx * dx + dy * dy + z * z;
    double jx = (double)(MISSION4_K_DB * dx / d2 - mission4_arc_jx[k]);
    double jy = (double)(MISSION4_K_DB * dy / d2 - mission4_arc_jy[k]);
    double r = (double)(mission4_sample_spl[i] + 10.0f * log10f(d2) - mission4_arc_r[k]);
    a11 += jx * jx;
    a12 += jx * jy;
    a22 += jy * jy;
    b1 += jx * r;
    b2 += jy * r;
  }
  double det = a11 * a22 - a12 * a12;
  if (signbit((float)(det - 1.0e-9))) {
    mission4_solve_good = FALSE;
    return FALSE;
  }
  /* Steps clipped to 8 m so a poor start cannot throw the estimate away. */
  float sx = fmaxf(-8.0f, fminf(8.0f, (float)((a22 * b1 - a12 * b2) / det)));
  float sy = fmaxf(-8.0f, fminf(8.0f, (float)((a11 * b2 - a12 * b1) / det)));
  mission4_est_x += sx;
  mission4_est_y += sy;
  mission4_solve_iter++;
  mission4_solve_good = TRUE;
  if (signbit(sx * sx + sy * sy - 1.0e-4f)) { return FALSE; }
  if (mission4_solve_iter == MISSION4_SOLVE_MAX_ITER) { return FALSE; }
  return TRUE;
}

/* Write DROP_POINT from the per-arc fit, or from the linear solver if the
   fit failed, and decide plausibility: the rules put the alarm within 25 m
   of the given point, so an estimate beyond MISSION4_PLAUSIBLE_M is another
   source and must not be flown to. */
static inline void mission4_orbit_finish(uint8_t centre_wp, uint8_t drop_wp)
{
  bool solved = mission4_solve_good;
  if (solved) {
    waypoints[drop_wp].x = mission4_est_x;
    waypoints[drop_wp].y = mission4_est_y;
  } else {
    solved = mission4_solve_source(drop_wp);
  }
  float dx = waypoints[drop_wp].x - waypoints[centre_wp].x;
  float dy = waypoints[drop_wp].y - waypoints[centre_wp].y;
  mission4_orbit_ok = FALSE;
  if (solved) {
    if (signbit(dx * dx + dy * dy - MISSION4_PLAUSIBLE_M * MISSION4_PLAUSIBLE_M)) { mission4_orbit_ok = TRUE; }
  }
#ifdef SITL
  /* M4_NO_SOURCE=1: there is nothing to find, so drop on the organisers'
     point instead of on an estimate fitted to background noise. SITL only:
     the competition always has a beeper, and the plan has no detection check
     (by decision), so on the aircraft the estimate is always used. */
  if (mission4_no_source) {
    waypoints[drop_wp].x = waypoints[centre_wp].x;
    waypoints[drop_wp].y = waypoints[centre_wp].y;
    mission4_orbit_ok = TRUE;
    printf("[M4ORBIT] M4_NO_SOURCE set: ignoring the estimate, dropping on SEARCH_CENTER\n");
    fflush(stdout);
  }
#endif
  mission4_orbit_done = TRUE;
}

static inline void mission4_orbit_reset(uint8_t centre_wp, uint8_t drop_wp)
{
  mission4_quiet_off();
  mission4_sample_count = 0;
  mission4_orbit_t0 = get_sys_time_float();
  mission4_orbit_last_t = -1.0f;
  mission4_orbit_dist = 0.0f;
  mission4_orbit_run = 0.0f;
  mission4_orbit_quiet = FALSE;
  mission4_arc = 0;
  mission4_orbit_done = FALSE;
  mission4_orbit_ok = FALSE;
  mission4_solving = FALSE;
  mission4_solve_good = FALSE;
  mission4_solve_iter = 0;
  mission4_thr_rejects = 0;
  mission4_orbit_cx = waypoints[centre_wp].x;
  mission4_orbit_cy = waypoints[centre_wp].y;
  mission4_orbit_target_set = FALSE;
  uint8_t k;
  for (k = 0; k != MISSION4_COV_BINS; k++) { mission4_cov[k] = 0; }
  waypoints[drop_wp].x = waypoints[centre_wp].x;
  waypoints[drop_wp].y = waypoints[centre_wp].y;
}

/* Orbit Search pre_call: owns the quiet/powered cycle for orbit_time, then
   powers up and runs the solver one step per nav cycle while the circle
   keeps flying. mission4_orbit_done hands over.

   A listening arc only starts once the climb-back has actually reached
   height (within MISSION4_ORBIT_REGAIN_TOL), and ends early if the glide
   reaches floor_m. A fixed 50 m of power is not enough on its own: the
   climb is capped near 2 m/s, so it regained ~9 m per cycle against
   ~10.5 m lost, and a SITL flight ratcheted from 45 m down to 20 m over
   nine arcs - under the floor, which only limits the COMMANDED height and
   cannot stop a glide with the motor off. */
#define MISSION4_ORBIT_REGAIN_TOL 1.0f
static inline void mission4_orbit_update(float orbit_time, float height, float floor_m,
                                         uint8_t centre_wp, uint8_t drop_wp)
{
  float now = get_sys_time_float();
  v_ctl_auto_airspeed_setpoint = MISSION4_ORBIT_AIRSPEED;
  if (mission4_orbit_done) { return; }

  /* Listening ends once orbit_time has passed AND every sector has been
     heard, or at the hard limit MISSION4_ORBIT_EXTRA_S later. */
  bool listen_done = mission4_solving;
  if (!signbit(now - mission4_orbit_t0 - orbit_time)) {
    if (mission4_cov_count() == MISSION4_COV_BINS) { listen_done = TRUE; }
    if (!signbit(now - mission4_orbit_t0 - orbit_time - MISSION4_ORBIT_EXTRA_S)) { listen_done = TRUE; }
  }
  if (listen_done) {
    mission4_quiet_off();
    mission4_orbit_quiet = FALSE;
    if (!mission4_solving) {
      mission4_solving = TRUE;
      mission4_solve_iter = 0;
      mission4_solve_good = FALSE;
      mission4_est_x = waypoints[centre_wp].x;
      mission4_est_y = waypoints[centre_wp].y;
      if (signbit((float)mission4_sample_count - MISSION4_SOLVE_MIN_SAMPLES)) {
        mission4_orbit_finish(centre_wp, drop_wp);
        return;
      }
    }
    if (!mission4_solve_step()) { mission4_orbit_finish(centre_wp, drop_wp); }
    return;
  }

  float dt = now - mission4_orbit_last_t;
  if (signbit(mission4_orbit_last_t)) { dt = 0.0f; }
  if (!signbit(dt - 0.5f)) { dt = 0.0f; }
  mission4_orbit_last_t = now;
  float ds = stateGetHorizontalSpeedNorm_f() * dt;
  mission4_orbit_dist += ds;

  float h = GetPosAlt() - GetAltRef();
  if (mission4_orbit_quiet) {
    mission4_orbit_run += ds;
    if (!signbit(mission4_orbit_dist - MISSION4_ORBIT_QUIET_M) || signbit(h - floor_m)) {
      mission4_orbit_quiet = FALSE;
      mission4_orbit_dist = 0.0f;
      mission4_quiet_off();
    } else {
      mission4_quiet_on();
    }
  } else if (!signbit(mission4_orbit_dist - MISSION4_ORBIT_POWER_M)) {
    if (!signbit(h - height + MISSION4_ORBIT_REGAIN_TOL)) {
      if (mission4_orbit_at_start_bearing()) {
        mission4_orbit_quiet = TRUE;
        mission4_orbit_dist = 0.0f;
        mission4_orbit_run = 0.0f;
        mission4_orbit_quiet_t = now;
        if (mission4_arc != MISSION4_MAX_ARCS - 1) { mission4_arc++; }
        mission4_quiet_on();
      }
    }
  }
}

/* Sample only on a listening arc, past the spin-down, and only while the
   throttle actually sent to the motor is zero - checked, not assumed, so a
   running prop can never again end up in the data unnoticed. */
static inline bool mission4_orbit_sample(uint8_t src_wp)
{
  if (!mission4_orbit_quiet) { return FALSE; }
  if (signbit(get_sys_time_float() - mission4_orbit_quiet_t - MISSION4_ORBIT_SPINDOWN)) { return FALSE; }
  if (v_ctl_throttle_slewed != 0) {
    mission4_thr_rejects++;
    return FALSE;
  }
  if (signbit((float)mission4_sample_count - (float)MISSION4_MAX_SAMPLES)) {
    uint8_t bin = (uint8_t)(mission4_orbit_bearing() / (2.0f * MISSION4_PI) * (float)MISSION4_COV_BINS);
    if (bin == MISSION4_COV_BINS) { bin = MISSION4_COV_BINS - 1; }
    mission4_cov[bin] = 1;
  }
  return mission4_sample_signal(src_wp, mission4_arc);
}

/* Level at the orbit height when powered; the natural glide line on a
   listening arc, where commanding level flight would only fight a motor
   that is off. Floored so a lagging climb-back cannot ratchet it down. */
static inline float mission4_orbit_alt(float height, float floor_m)
{
  if (!mission4_orbit_quiet) { return GetAltRef() + height; }
  float h = height - mission4_orbit_run / (float)V_CTL_GLIDE_RATIO;
  if (signbit(h - floor_m)) { h = floor_m; }
  return GetAltRef() + h;
}

static inline void mission4_log_estimate(uint8_t drop_wp, uint8_t true_wp)
{
  char msg[100];
  float ex = waypoints[drop_wp].x - waypoints[true_wp].x;
  float ey = waypoints[drop_wp].y - waypoints[true_wp].y;
  float err = sqrtf(ex * ex + ey * ey);
  int nc = snprintf(msg, sizeof(msg), "M4EST n=%u arcs=%u it=%u fit=%d ok=%d err=%.1f cov=%u/36",
                    (unsigned)mission4_sample_count, (unsigned)mission4_arc,
                    (unsigned)mission4_solve_iter, (int)mission4_solve_good,
                    (int)mission4_orbit_ok, err, (unsigned)mission4_cov_count());
  if (!signbit((float)nc)) {
    DOWNLINK_SEND_INFO_MSG(DefaultChannel, DefaultDevice, (uint8_t)nc, msg);
  }
#ifdef SITL
  printf("[M4ORBIT] solve: %u samples, %u arcs, %u iterations, per-arc fit %s, plausible %s\n"
         "          estimate (%.1f, %.1f), %.1f m from TRUE source; cycles rejected for throttle: %u; sectors heard %u/36\n",
         (unsigned)mission4_sample_count, (unsigned)mission4_arc, (unsigned)mission4_solve_iter,
         mission4_solve_good ? "ok" : "FAILED (linear fallback)", mission4_orbit_ok ? "yes" : "NO",
         waypoints[drop_wp].x, waypoints[drop_wp].y, err, (unsigned)mission4_thr_rejects, (unsigned)mission4_cov_count());
  fflush(stdout);
#endif
}

""")
splice("static inline bool mission4_sample_signal(uint8_t source_wp, uint8_t drop_wp) {",
       "/* ===================================================================\n   OBSTACLE-AWARE RUN-IN, from their v2.", ORBIT)

# ---------------------------------------------------------------- release log: star vars -> orbit vars
rep('"STARDROP agl=%.2f dTrue=%.1f MISS=%.2f SIM=%.2f n=%u it=%u sh=%.1f lane=%.0f",',
    '"ORBITDROP agl=%.2f dTrue=%.1f MISS=%.2f SIM=%.2f n=%u arcs=%u lane=%.0f",')
rep('printf("[M4STAR] release:', 'printf("[M4ORBIT] release:')
rep("search: %u legs/star, %u stars, final shift %.2f m, lane course %.0f deg",
    "search: %u samples, %u arcs, lane course %.0f deg")
n_args = len(re.findall(r"\(unsigned\)mission4_star_nlegs, \(unsigned\)mission4_star_iter,\n\s+mission4_star_shift_m, _tilt\);", s))
if n_args != 2: sys.exit(f"release-log args: expected 2, found {n_args}")
s = re.sub(r"\(unsigned\)mission4_star_nlegs, \(unsigned\)mission4_star_iter,\n(\s+)mission4_star_shift_m, _tilt\);",
           r"(unsigned)mission4_sample_count, (unsigned)mission4_arc,\n\1_tilt);", s)

# ---------------------------------------------------------------- SITL: place the simulated beeper from the environment
rep(r"""static inline float mission4_spl_noise(void)""",
    r"""#ifdef SITL
/* SITL only: M4_SOURCE_E / M4_SOURCE_N (metres east / north of SEARCH_CENTER)
   move the simulated beeper, so a batch can test positions across the whole
   search area without editing the flight plan. Unset: the waypoint as
   declared. */
static inline void mission4_sim_place_source(uint8_t src_wp, uint8_t centre_wp)
{
  const char *e = getenv("M4_SOURCE_E");
  const char *n = getenv("M4_SOURCE_N");
  if (e == NULL) { return; }
  if (n == NULL) { return; }
  waypoints[src_wp].x = waypoints[centre_wp].x + (float)atof(e);
  waypoints[src_wp].y = waypoints[centre_wp].y + (float)atof(n);
  printf("[M4SOURCE] beeper placed %.1f m east, %.1f m north of SEARCH_CENTER\n", atof(e), atof(n));
  fflush(stdout);
}
#else
static inline void mission4_sim_place_source(uint8_t src_wp __attribute__((unused)),
                                             uint8_t centre_wp __attribute__((unused))) {}
#endif

static inline float mission4_spl_noise(void)""")

# ---------------------------------------------------------------- lead quantisation: GPS update period, not the nav tick
rep(r"""static inline float mission4_release_lead(void)""",
    r"""/* How often the horizontal position actually changes. ins_alt_float moves
   it only when GPS data arrives, so the release gate can only fire on the
   first update inside the lead: on average half an update period of travel
   late, not the half 20 Hz nav tick the formula used to assume. Traced in
   SITL (10 Hz GPS) the position changed every 2-3 nav cycles in uneven
   1-2 m steps; with this term the kit landed -0.37..+0.50 m long (mean
   +0.12) over five noiseless runs. Extrapolating between updates did not
   help: the GPS timestamps tick every nav cycle, and the updates' own
   timing jitter still made the extrapolated range jump. What remains is
   about +-0.5 m of release scatter at 10 m/s; only an INS that propagates
   position between fixes removes it. Set this to the aircraft's real GPS
   rate: the ublox rate is not configured in the airframe, and at 1-5 Hz the
   scatter grows to 2-10 m, so configure the receiver for 10 Hz. */
#define MISSION4_POS_UPDATE_HZ 10.0f

static inline float mission4_release_lead(void)""")
rep(r"""  float _tick = _v / (2.0f * (float)NAVIGATION_FREQUENCY);""",
    r"""  float _tick = _v / (2.0f * MISSION4_POS_UPDATE_HZ);""")

# ---------------------------------------------------------------- SITL per-cycle release gate trace (prints only)
rep(r"""#ifdef SITL
static inline void mission4_sim_payload(uint8_t impact_wp)""",
    r"""#ifdef SITL
/* Every nav cycle of Drop Final within 12 m of DROP_POINT: range, lead, AGL,
   whether the estimated position changed since the previous cycle, and the
   age of the last GPS fix computed from the fix time and from the message
   time. Shows whether the gate can only fire right after a GPS update and
   whether a between-fix projection could be timed reliably. SITL only;
   always false, so it never deroutes. */
static float mission4_trace_x = 0.0f;
static float mission4_trace_y = 0.0f;

static inline bool mission4_gate_trace(uint8_t drop_wp)
{
  float px = GetPosX(), py = GetPosY();
  float dx = px - waypoints[drop_wp].x, dy = py - waypoints[drop_wp].y;
  float d = sqrtf(dx * dx + dy * dy);
  if (signbit(d - 12.0f)) {
    float now = get_sys_time_float();
    float fix_t = gps_time_since_last_3dfix();
    float msg_t = (float)gps.last_msg_time + (float)gps.last_msg_ticks / (float)sys_time.cpu_ticks_per_sec;
    int moved = (px != mission4_trace_x) || (py != mission4_trace_y);
    printf("[M4GATE] t=%.3f d=%.2f lead=%.2f agl=%.2f moved=%d fix_age=%.3f msg_age=%.3f\n",
           now, d, mission4_release_lead(), mission4_agl(), moved, now - fix_t, now - msg_t);
  }
  mission4_trace_x = px;
  mission4_trace_y = py;
  return FALSE;
}
#else
static inline bool mission4_gate_trace(uint8_t drop_wp __attribute__((unused))) { return FALSE; }
#endif

#ifdef SITL
static inline void mission4_sim_payload(uint8_t impact_wp)""")
rep(r"""      <exception cond="(mission4_agl() @LT 2.0) @AND (mission4_agl() @GT 0.5)""",
    r"""      <exception cond="mission4_gate_trace(WP_DROP_POINT)" deroute="Drop Missed"/>
      <exception cond="(mission4_agl() @LT 2.0) @AND (mission4_agl() @GT 0.5)""")

# ---------------------------------------------------------------- SITL release diagnostics (prints only)
rep(r"""(unsigned)mission4_sample_count, (unsigned)mission4_arc,
         _tilt);
  fflush(stdout);""", r"""(unsigned)mission4_sample_count, (unsigned)mission4_arc,
         _tilt);
  {
    /* Where the drop error comes from, relative to the run-in line through
       DROP_POINT: how far out the gate fired (mission4_prev_d2 is the range
       at the gate tick), how far out the release actually happened, the
       aircraft's cross-track offset and course error, and the impact split
       into long/short and left/right. SITL only. */
    float _ux = mission4_run_in_ux, _uy = mission4_run_in_uy;
    float _rx = _px - waypoints[drop_wp].x, _ry = _py - waypoints[drop_wp].y;
    float _along = -(_rx * _ux + _ry * _uy);
    float _cross = _rx * _uy - _ry * _ux;
    float _cerr = fmodf(DegOfRad(_dir - atan2f(_ux, _uy)) + 540.0f, 360.0f) - 180.0f;
    float _jx = waypoints[impact_wp].x - waypoints[drop_wp].x;
    float _jy = waypoints[impact_wp].y - waypoints[drop_wp].y;
    printf("[M4DIAG] gate fired %.2f m out; released %.2f m to go (lead %.2f); cross-track %+.2f m (+ right); course error %+.2f deg\n"
           "         impact %+.2f m long, %+.2f m right of DROP_POINT\n",
           sqrtf(mission4_prev_d2), _along, _lead, _cross, _cerr,
           _jx * _ux + _jy * _uy, _jx * _uy - _jy * _ux);
  }
  fflush(stdout);""")

# ---------------------------------------------------------------- waypoints: drop star scratch
before = len(s)
s = re.sub(r"    <!-- Star scratch: leg start.*?<waypoint name=\"_S_TURN\" x=\"0.0\" y=\"0.0\"/>\n\n", "", s, count=1, flags=re.S)
if len(s) == before: sys.exit("star scratch waypoints not found")

# ---------------------------------------------------------------- variables
VARS = """    <!-- 45 m, as the star: 27 m above the tallest tree at this site. A 110 m
         listening arc sheds about 10 m, climbed back on the 50 m powered
         part; orbit_floor stops a lagging climb-back ratcheting the orbit
         down lap after lap. DO NOT LOWER THIS: the orbit is over the lane
         here, but the organisers' point on the day may not be. -->
    <variable init="45." var="orbit_height" min="30." max="70." step="1."/>
    <variable init="30." var="orbit_floor" min="25." max="60." step="1."/>
    <!-- Minimum listening time, extended by at most 60 s until every 10 deg
         sector round the orbit has been heard. Monte Carlo median error
         4.2 m at 150 s, 5.5 m at 75 s. Capped at 160 s; samples beyond the
         2000-sample buffer are not kept and do not count as heard. -->
    <variable init="150." var="orbit_time" min="40." max="160." step="5."/>
    <variable init="0" type="uint8_t" var="orbit_fails"/>
"""
before = len(s)
s = re.sub(r"    <!-- 45 m, their v2's M4_SAFE_HEIGHT_M.*?var=\"star_floor\" min=\"25.\" max=\"60.\" step=\"1.\"/>\n", VARS, s, count=1, flags=re.S)
if 'var="orbit_height"' not in s or 'var="star_floor"' in s: sys.exit("star height variables not replaced")

# ---------------------------------------------------------------- exceptions
rep("autopilot to HOME mode on breach. -->",
    "autopilot to HOME mode on breach. Drop Exit is excluded: it is already a\n"
    "         full-rate climb, and diverting it only turned it back towards\n"
    "         _CLIMBOUT. -->")
rep("@AND !(nav_block == IndexOfBlock('Drop Go Around'))",
    "@AND !(nav_block == IndexOfBlock('Drop Go Around')) @AND !(nav_block == IndexOfBlock('Drop Exit'))")
rep("  </exceptions>",
    "    <!-- Restores the throttle gains if a listening arc stopped renewing its\n"
    "         request (see mission4_quiet_on). Always false; the deroute target\n"
    "         only has to be a block the aircraft is never in during flight. -->\n"
    "    <exception cond=\"mission4_quiet_guard()\" deroute=\"Wait GPS\"/>\n"
    "  </exceptions>")
rep('      <call_once fun="mission4_noise_init()"/>\n',
    '      <call_once fun="mission4_noise_init()"/>\n      <call_once fun="mission4_quiet_init()"/>\n')

# ---------------------------------------------------------------- blocks: star search -> orbit search
BLOCKS = """    <!-- ── Orbit search ─""" + "─" * 51 + """ -->
    <block name="Start Mission4 Orbit" strip_button="Start Mission4 Orbit" strip_icon="survey.png" group="home">
      <set var="sound_detected" value="FALSE"/>
      <set var="drop_approaches" value="0"/>
      <set var="orbit_fails" value="0"/>
      <deroute block="Climb to Orbit Height"/>
    </block>

    <!-- Takeoff hands over at +25 m; climb the rest over the centre, powered. -->
    <block name="Climb to Orbit Height" group="mission">
      <set var="v_ctl_auto_throttle_nominal_cruise_throttle" value="MISSION4_CRUISE_THR"/>
      <set var="v_ctl_auto_airspeed_setpoint" value="MISSION4_ORBIT_AIRSPEED"/>
      <exception cond="GetPosAlt() @GT GetAltRef() + orbit_height - 2.0" deroute="Orbit Begin"/>
      <circle climb="2.0" radius="DEFAULT_CIRCLE_RADIUS" vmode="climb" wp="SEARCH_CENTER"/>
    </block>

    <!-- Clean buffer, arc counter and timer. DROP_POINT is parked on the
         centre so a manual Start Drop Mission4 before any solve still has a
         sane target. Also where the one retry comes back in. -->
    <block name="Orbit Begin" group="mission">
      <call_once fun="mission4_sim_place_source(WP_TRUE_SOUND_SOURCE, WP_SEARCH_CENTER)"/>
      <call_once fun="DownlinkSendWpNr(WP_TRUE_SOUND_SOURCE)"/>
      <call_once fun="mission4_orbit_reset(WP_SEARCH_CENTER, WP_DROP_POINT)"/>
      <deroute block="Orbit Search"/>
    </block>

    <!-- The orbit. The pre_call owns the quiet/powered cycle and, after
         orbit_time, runs the solver one step per nav cycle while the circle
         keeps flying; mission4_orbit_done then hands over. The sampling
         exception is the side-effecting-condition idiom, so it must deroute
         to a DIFFERENT block (see the star file's leg block). -->
    <block name="Orbit Search" group="mission" pre_call="mission4_orbit_update(orbit_time, orbit_height, orbit_floor, WP_SEARCH_CENTER, WP_DROP_POINT)">
      <exception cond="sound_detected" deroute="Drop Setup"/>
      <exception cond="mission4_orbit_done" deroute="Orbit Decide"/>
      <exception cond="mission4_orbit_sample(WP_TRUE_SOUND_SOURCE)" deroute="Orbit Decide"/>
      <circle radius="MISSION4_ORBIT_R" vmode="alt" alt="mission4_orbit_alt(orbit_height, orbit_floor)" wp="SEARCH_CENTER"/>
    </block>

    <block name="Orbit Decide" group="mission">
      <call_once fun="DownlinkSendWpNr(WP_DROP_POINT)"/>
      <call_once fun="mission4_log_estimate(WP_DROP_POINT, WP_TRUE_SOUND_SOURCE)"/>
      <deroute block="Orbit Check"/>
    </block>

    <!-- Separate block: exceptions are tested before stages, so the verdict
         the solver left behind is acted on here. -->
    <block name="Orbit Check" group="mission">
      <exception cond="!mission4_orbit_ok" deroute="Orbit Failed"/>
      <set var="sound_detected" value="TRUE"/>
      <deroute block="Drop Setup"/>
    </block>

    <!-- No fit, or an implausible one: one fresh orbit, then hold. Better no
         drop than the wrong mannequin. orbit_fails is outside
         mission4_orbit_reset() so the retry cannot clear it - the star file's
         retry did exactly that and could loop forever. -->
    <block name="Orbit Failed" group="mission">
      <set var="orbit_fails" value="orbit_fails + 1"/>
      <deroute block="Orbit Retry"/>
    </block>

    <block name="Orbit Retry" group="mission">
      <exception cond="orbit_fails @GT 1" deroute="Standby"/>
      <deroute block="Orbit Begin"/>
    </block>

"""
splice("    <!-- ── Star search", '    <block name="Start Drop Mission4"', BLOCKS)
rep("Start Mission4 Star", "Start Mission4 Orbit", count=2)   # the two Takeoff hand-overs

# ---------------------------------------------------------------- base-turn exit
rep("""      <!-- Hold the base turn until BOTH the heading is 15 deg short of the
           run-in and the aircraft is within 5 m of the START height. -->""",
    """      <!-- Hold the base turn until BOTH the aircraft is 15 deg of arc short
           of START and within 5 m of the START height. NavQdrCloseTo tests
           the BEARING of the aircraft from the circle centre, not its
           heading. START sits at course - 90 deg from the centre of a
           right-hand circle (M4_DROPTURN_SIDE = +1), course + 90 for a
           left-hand one, so exit at course -+ 105. The star file tests
           course - 15, a quarter-turn late. -->""")
rep("NavQdrCloseTo(DegOfRad(atan2f(mission4_run_in_ux, mission4_run_in_uy))-15)",
    "NavQdrCloseTo(DegOfRad(atan2f(mission4_run_in_ux, mission4_run_in_uy)) - M4_DROPTURN_SIDE * 105.)")

# ---------------------------------------------------------------- Drop Exit
rep("""    <!-- Climbing circle around _EXIT, placed by construction so a full turn
         plus M4_FENCE_MARGIN_M stays inside the geofence. Leaves once above
         the trees. -->""",
    """    <!-- Climbing circle around _EXIT, placed by construction so a full turn
         plus M4_FENCE_MARGIN_M stays inside the geofence. Leaves once above
         the trees. Climbs at the climb-out rate, not 2 m/s: for most
         estimates the exit circle overlaps the east tree row and the aircraft
         arrives at about 20 m, under M4_ESCAPE_HEIGHT_M. -->""")
rep("""    <block name="Drop Exit" group="mission">
      <set var="v_ctl_max_climb" value="V_CTL_ALTITUDE_MAX_CLIMB"/>
      <set var="v_ctl_auto_airspeed_setpoint" value="12."/>
      <circle radius="M4_DROPTURN_SIDE * M4_EXIT_TURN_RADIUS_M"
              until="GetPosHeight() @GT M4_SAFE_HEIGHT_M - 5" vmode="climb" climb="2.0" wp="_EXIT"/>
      <deroute block="Drop Decide"/>""",
    """    <block name="Drop Exit" group="mission">
      <set var="v_ctl_max_climb" value="M4_CLIMBOUT_RATE_MPS"/>
      <set var="v_ctl_auto_airspeed_setpoint" value="M4_CLIMBOUT_AIRSPEED_MPS"/>
      <circle radius="M4_DROPTURN_SIDE * M4_EXIT_TURN_RADIUS_M"
              until="GetPosHeight() @GT M4_SAFE_HEIGHT_M - 5" vmode="climb" climb="M4_CLIMBOUT_RATE_MPS" wp="_EXIT"/>
      <set var="v_ctl_max_climb" value="V_CTL_ALTITUDE_MAX_CLIMB"/>
      <deroute block="Drop Decide"/>""")

# ---------------------------------------------------------------- final checks
for leftover in (r"mission4_star_(?!strasbourg)", r"WP__S_", r"Star Leg", r"Star Turn", r"star_height"):
    m = re.search(leftover, s)
    if m: sys.exit(f"leftover star reference {leftover!r}: {s[m.start() - 40:m.end() + 40]!r}")
open(DST, "w").write(s)
print(f"wrote {DST}: {s.count(chr(10))} lines")
