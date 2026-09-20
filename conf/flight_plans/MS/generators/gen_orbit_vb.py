"""Build the Valkenburg replica mission4_orbit.xml from mission4_orbit_strasbourg.xml.
Same pattern as mission4.xml vs mission4_strasbourg.xml: identical mission
logic, Valkenburg site. Every edit must match exactly once, or the script stops."""
import os, re, sys

MS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))  # conf/flight_plans/MS
SRC = os.path.join(MS, "mission4_orbit_strasbourg.xml")
DST = os.path.join(MS, "mission4_orbit.xml")
s = open(SRC).read()

def rep(old, new, count=1):
    global s
    n = s.count(old)
    if n != count:
        sys.exit(f"expected {count} match(es), found {n}: {old[:90]!r}")
    s = s.replace(old, new)

def splice(start, end, new):
    """Replace from start (inclusive) to end (inclusive)."""
    global s
    a = s.find(start); b = s.find(end, a)
    if a == -1 or b == -1 or s.count(start) != 1:
        sys.exit(f"splice markers not found/unique: {start[:60]!r}")
    s = s[:a] + new + s[b + len(end):]

# ---------------------------------------------------------------- top comment: Valkenburg note first
rep("<!--\n  IMAV Mission4 (Strasbourg) - ORBIT search, obstacle-aware drop.\n",
    """<!--
  IMAV Mission4 - ORBIT search, VALKENBURG replica.

  Generated from mission4_orbit_strasbourg.xml, the way mission4.xml is the
  Valkenburg replica of mission4_strasbourg.xml: the orbit search, solver,
  motor-off listening and the whole drop are identical, only the site
  differs. Flown by CherryOrbitVB (session mission4_orbit_valkenburg).

  VALKENBURG SITE
    - Frame, launch/land, standby, swarm waypoints, SEARCH_CENTER and
      TRUE_SOUND_SOURCE placeholders: the same as mission4.xml.
    - Flyzone FZ1-FZ7 from MS/valkenburg_flyzone.kml, as sector M4_FENCE so
      the drop code's corridor, climb-out and exit-circle tests use it.
    - Geofence: mission4.xml's buffered exception (mission4_in_valkenburg_
      safe, 60 m outside margin) to Standby, NOT geofence_sector. HOME is
      ~31 m outside this polygon, so a hard fence cannot be used.
    - NO OBSTACLE SECTORS. There is no tree or building survey for
      Valkenburg, so the only thing the low run-in is checked against is
      leaving the flyzone. Look at the field before trusting a low run-in.
    - Run-in course 270 (westbound). From the placeholder SEARCH_CENTER a
      300 m run-in from the south starts outside the flyzone; from the east
      it fits with ~90 m to spare, and it is usually into the prevailing
      westerly wind.

  Everything below this section is the Strasbourg file's own header. Its
  site notes (LiDAR survey, lane, fence discrepancy) describe Strasbourg.

  ORIGINAL HEADER - IMAV Mission4 (Strasbourg) - ORBIT search, obstacle-aware drop.
""")
rep('name="Mission4_Orbit_Strasbourg"', 'name="Mission4_Orbit_Valkenburg"')
rep('<flight_plan alt="190" ground_alt="140" lat0="48.8086670" lon0="7.8517466"',
    '<flight_plan alt="50" ground_alt="0" lat0="52.1681239" lon0="4.4124784"')
rep('security_height="5" geofence_sector="M4_FENCE" geofence_max_alt="220">',
    'security_height="5" geofence_max_alt="80">')

# ---------------------------------------------------------------- header C: obstacles, geofence helper, run-in course
rep('#include "modules/gps/gps.h"\n',
    '#include "modules/gps/gps.h"\n#include "modules/nav/nav_mission4_geofence.h"\n')
rep("static inline bool InsideM4_OW(float _x, float _y);\n"
    "static inline bool InsideM4_OE(float _x, float _y);\n"
    "static inline bool InsideM4_ON(float _x, float _y);\n", "")
rep("/* Trees, or outside the geofence: both are places the low run-in corridor\n"
    "   may not touch. Logical OR only - see the header note on operators. */",
    "/* Valkenburg: no obstacle survey, so leaving the flyzone is the only thing\n"
    "   the low run-in corridor is tested against. */")
rep("  return InsideM4_OW(x, y) || InsideM4_OE(x, y) || InsideM4_ON(x, y) || !InsideM4_FENCE(x, y);",
    "  return !InsideM4_FENCE(x, y);")
rep("""/* Run-in lane, from their v2. Course is south to north between the tree
   lines; the tilt search about it is what actually finds a clear corridor
   (course 0 itself is blocked 111 m south of the target - see the file
   header), so MAX_TILT is load bearing, not a refinement. */
#define M4_LANE_COURSE_DEG 0.f""",
    """/* Run-in course, westbound at Valkenburg. From the placeholder
   SEARCH_CENTER a 300 m run-in from the south starts outside the flyzone;
   from the east it fits with ~90 m to spare and is usually into the
   prevailing westerly wind. The tilt search still tries +-20 deg about it,
   testing the corridor against the flyzone only (no obstacle sectors). */
#define M4_LANE_COURSE_DEG 270.f""")
rep("""/* +1 = base turn and exit circle on the right (east) side, over the open
   meadow rather than towards the west forest. */""",
    """/* +1 = base turn and exit circle on the right of the run-in: north of a
   westbound run-in at Valkenburg. */""")

# ---------------------------------------------------------------- waypoints
WPTS = """  <waypoints>
    <!-- Valkenburg launch / land / standby infrastructure, as in mission4.xml
         (the CherryVB replica). TD keeps its explicit height="0". -->
    <waypoint name="HOME" x="0.0" y="0.0"/>
    <waypoint name="AF" x="11.438" y="17.313" alt="35.0"/>
    <waypoint name="_BASELEG" x="-0.709" y="10.397"/>
    <waypoint height="0" name="TD" x="-0.238" y="23.784"/>
    <waypoint name="_HERE" x="0.0" y="0.0"/>
    <waypoint name="_STDBY" x="150.000" y="60.000"/>

    <!-- Kept only because nav_couzin_fw.c hard-references these in C. -->
    <waypoint name="SWARM_TGT" x="180.000" y="90.000"/>
    <waypoint name="S1" x="380.000" y="-60.000"/>
    <waypoint name="S2" x="640.000" y="220.000"/>

    <!-- >>> REPOSITION ON THE DAY <<< Placeholder, the same point mission4.xml
         uses. Height is the orbit height. -->
    <waypoint name="SEARCH_CENTER" x="270.000" y="40.000" height="45."/>

    <!-- PLACEHOLDER ground truth for the simulated sensor only, the same point
         mission4.xml uses (~10.6 m from the centre). -->
    <waypoint name="TRUE_SOUND_SOURCE" x="278.000" y="47.000"/>

    <!-- Drop geometry, all recomputed at Drop Setup. START is 300 m back on
         the chosen run-in course; _CLIMBOUT and _EXIT are fence limited. -->
    <waypoint name="DROP_POINT" x="0.0" y="0.0"/>
    <waypoint name="DROP_APPROACH" x="0.0" y="0.0"/>
    <waypoint name="DROP_LEVEL" x="0.0" y="0.0"/>
    <waypoint name="_DROPTURN" x="0.0" y="0.0"/>
    <waypoint name="_CLIMBOUT" x="0.0" y="0.0"/>
    <waypoint name="_EXIT" x="0.0" y="0.0"/>
    <waypoint name="IMPACT" x="0.0" y="0.0"/>

    <!-- Valkenburg flyzone FZ1-FZ7, MS/valkenburg_flyzone.kml in this frame -
         the same corners as mission4.xml and mission4_in_valkenburg() in
         nav_mission4_geofence.h. -->
    <waypoint name="FZ1" x="-13.546" y="138.715"/>
    <waypoint name="FZ2" x="96.380" y="-189.677"/>
    <waypoint name="FZ3" x="275.267" y="-258.695"/>
    <waypoint name="FZ4" x="603.682" y="-1.547"/>
    <waypoint name="FZ5" x="876.109" y="207.733"/>
    <waypoint name="FZ6" x="775.741" y="545.031"/>
    <waypoint name="FZ7" x="310.089" y="344.656"/>
  </waypoints>
"""
splice("  <waypoints>\n", "  </waypoints>\n", WPTS)

SECT = """  <sectors>
    <!-- The Valkenburg flyzone. Named M4_FENCE because the drop code tests
         InsideM4_FENCE() for its corridor, climb-out and exit-circle checks.
         Not wired to geofence_sector - see the file header. -->
    <sector name="M4_FENCE" color="green">
      <corner name="FZ1"/><corner name="FZ2"/><corner name="FZ3"/><corner name="FZ4"/>
      <corner name="FZ5"/><corner name="FZ6"/><corner name="FZ7"/>
    </sector>
    <kml file="MS/valkenburg_flyzone.kml"/>
  </sectors>
"""
splice("  <sectors>\n", "  </sectors>\n", SECT)

# ---------------------------------------------------------------- exceptions
rep("""The hard fence itself is
         geofence_sector on the flight_plan element, which switches the
         autopilot to HOME mode on breach.""",
    """At Valkenburg there are no
         obstacle sectors, so this only fires low OUTSIDE the flyzone; the
         fence proper is the buffered exception below.""")
rep("  </exceptions>",
    "    <!-- Geofence, exactly as mission4.xml: outside the Valkenburg flyzone plus\n"
    "         a 60 m margin, deroute to Standby. The margin is what lets HOME and\n"
    "         the landing points, just outside the polygon, not trip it. -->\n"
    "    <exception cond=\"!mission4_in_valkenburg_safe(GetPosX(), GetPosY()) @AND "
    "!(nav_block == IndexOfBlock('Wait GPS')) @AND !(nav_block == IndexOfBlock('Geo init')) @AND "
    "!(nav_block == IndexOfBlock('Holding point')) @AND !(nav_block == IndexOfBlock('Takeoff'))\" "
    "deroute=\"Standby\"/>\n"
    "  </exceptions>")
rep("45 m, as the star: 27 m above the tallest tree at this site.",
    "45 m, kept from the Strasbourg file (27 m above its tallest tree).")

# ---------------------------------------------------------------- final checks
for leftover in (r"InsideM4_O[WEN]", r"_OW1", r"_F1\"", r"strasbourg_flyzone\.kml", r"geofence_sector=", r"lat0=\"48\."):
    m = re.search(leftover, s)
    if m: sys.exit(f"leftover Strasbourg item {leftover!r}: {s[max(0, m.start() - 40):m.end() + 40]!r}")
open(DST, "w").write(s)
print(f"wrote {DST}: {s.count(chr(10))} lines")
