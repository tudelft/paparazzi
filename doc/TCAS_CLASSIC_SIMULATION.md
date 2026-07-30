# Retesting TCAS in Classic Simulation

This guide explains how to reproduce the two TCAS scenarios used to verify the
fix:

1. two fixed-wing aircraft flying toward each other;
2. one fixed-wing aircraft passing an ARDrone 2 rotorcraft in a stable hover.

Both scenarios start automatically. No flight-plan block selection, takeoff
button, or threshold adjustment is required.

## Quick retest in Paparazzi Center

Use these files in Paparazzi Center:

- configuration: `conf/userconf/OPENUAS/openuas_test_tcas_conf.xml`;
- control panel: `conf/userconf/OPENUAS/openuas_control_panel.xml`.

### Two fixed wings

1. Select `Adam_TCAS` and build the `sim` target.
2. Select `Easystar_3_TCAS` and build the `sim` target.
3. Start the session **OUAS TCAS - Two Fixedwings (Automatic)**.
4. Watch the Messages window and the aircraft altitudes in the GCS.

The aircraft repeatedly fly the same opposing route. A successful encounter
shows:

```text
129 TCAS_TA 135
135 TCAS_TA 129
129 TCAS_RA 135 3
135 TCAS_RA 129 2
129 TCAS_RESOLVED 135
135 TCAS_RESOLVED 129
```

The exact repetition count and timing can vary. Resolution value `2` is climb
and `3` is descend. The two aircraft must choose opposite directions and their
altitudes must separate while the RA is active.

### Fixed wing and ARDrone 2

1. Select `Easystar_3_TCAS` and build the `sim` target.
2. Select `ARDrone_2_TCAS` and build the `sim` target.
3. Start the session **OUAS TCAS - Fixedwing and ARDrone (Automatic)**.
4. Watch aircraft 161 climb to a 12 m hover at the route midpoint.
5. Watch the Messages window as aircraft 135 approaches it.

A successful encounter shows both aircraft participating:

```text
135 TCAS_TA 161
161 TCAS_TA 135
135 TCAS_RA 161 3
161 TCAS_RA 135 2
135 TCAS_RESOLVED 161
161 TCAS_RESOLVED 135
```

Aircraft 161 should climb while its RA is active, then return to its nominal
12 m hover after `TCAS_RESOLVED`. In the acceptance run it climbed from about
12.02 m to 12.70 m before returning to 12 m.

## Command-line build checks

From the repository root, build the same targets with:

```sh
make CONF_XML=conf/userconf/OPENUAS/openuas_test_tcas_conf.xml \
  AIRCRAFT=Adam_TCAS sim.compile
make CONF_XML=conf/userconf/OPENUAS/openuas_test_tcas_conf.xml \
  AIRCRAFT=Easystar_3_TCAS sim.compile
make CONF_XML=conf/userconf/OPENUAS/openuas_test_tcas_conf.xml \
  AIRCRAFT=ARDrone_2_TCAS sim.compile
```

Run the focused policy regression suite with:

```sh
make -C tests/utils test_tcas_policy.run
tests/utils/test_tcas_policy.run
```

The expected result is `1..34` followed by 34 passing checks.

## Why these scenarios are deterministic

The fixed-wing plans use opposing, looping routes with a 5 m security floor:

- `conf/flight_plans/OPENUAS/openuas_fixedwing_tcas_west.xml`;
- `conf/flight_plans/OPENUAS/openuas_fixedwing_tcas_east.xml`.

The ARDrone plan starts at the midpoint of the fixed-wing route, climbs
vertically, and holds position at 12 m:

- `conf/flight_plans/OPENUAS/openuas_rotorcraft_tcas_test.xml`.

The ARDrone `sim` target uses Paparazzi's `fdm_rotorcraft_sim` model. Its older
JSBSim model is still available through the `nps` target, but it is not used by
this regression scenario because its lateral instability made encounter timing
unrepeatable.

The saved sessions run the server without `-n`. This is important: the server
must receive and rebroadcast classic traffic messages for each aircraft to see
the other one.

## What was fixed

### Classic traffic with a frozen TOW

The legacy simulation path can repeatedly publish `itow=1`. Treating every
equal-TOW packet as an old duplicate made a moving aircraft go stale in the
traffic table.

Traffic ingestion now compares the observation payload when TOW is equal:

- changed position or velocity is accepted as a new observation;
- an identical repeat remains a duplicate and does not refresh freshness;
- older TOW values are still rejected.

The classic `ACINFO_LLA` velocity conversion was also corrected to encode
meters per second as centimeters per second.

### Fixed-wing ownship state validation

Paparazzi state coordinate getters perform conversions lazily. TCAS previously
tested state status bits before all required ENU values had been materialized.
In classic simulation this left ownship geometry permanently unavailable even
though all coordinates and velocities were finite.

TCAS now obtains one ENU position and velocity snapshot first, then validates
the resulting status and values. A finite zero velocity remains valid, which is
required for hovering rotorcraft.

### Rotorcraft altitude representation

The ARDrone had valid local ENU position and velocity but did not expose
`POS_UTM_F`. Requiring a UTM altitude therefore disabled its TCAS before traffic
was evaluated.

The shared TCAS core now obtains MSL altitude from the firmware's native state:

- fixed wing: UTM altitude;
- rotorcraft: MSL origin plus local ENU altitude.

The same MSL value is used for the security-height gate, intruder altitude
conversion, and altitude command refresh.

### Initial RA coordination

Before a peer resolution message arrives, each aircraft independently selects
a complementary direction from vertical geometry and aircraft-ID ordering.
The initial command is no longer reversed from the peer's observed vertical
speed: that unconfirmed-motion heuristic could briefly make both aircraft
descend. Once a fresh peer resolution is available, the existing explicit
coordination rule still resolves any same-direction choice by aircraft ID.

### Reproducible test geometry

The original routes did not guarantee a conflict, and unstable flight-dynamics
models could fail before closest approach. Dedicated automatic plans and stable
simulation targets now isolate TCAS behavior from manual operation and model
instability.

## Investigation path

The repair was completed in this order:

1. traced classic traffic through server rebroadcast and airborne ingestion;
2. reproduced the frozen-TOW stale-track failure;
3. added payload-aware equal-TOW acceptance and velocity unit checks;
4. replaced non-conflicting manual routes with deterministic opposing routes;
5. selected stable fixed-wing simulation profiles for both aircraft IDs;
6. verified that traffic slots 0 and 1 are intentionally reserved for GCS and
   ownship, ruling out a slot-index regression;
7. traced the TCAS early-return gates and found ownship geometry unavailable;
8. fixed lazy state conversion ordering and obtained live fixed-wing TA/RA;
9. built an automatic midpoint-hover rotorcraft scenario;
10. retained the ARDrone `nps` target and added a stable `sim` target;
11. found that rotorcraft had valid ENU geometry without UTM altitude;
12. added firmware-neutral MSL altitude handling;
13. verified TA, coordinated RA, altitude response, and resolution for both
  scenarios;
14. repeated the live fixed-wing encounter after the final cleanup, found a
  one-cycle same-direction RA, removed the uncoordinated speed reversal, and
  verified complementary first commands through resolution.

## Troubleshooting

If no advisory appears:

- confirm both required simulator processes are running and moving;
- confirm the session's Server entry does not contain `-n`;
- rebuild after changing a flight plan because generated constants are compiled
  into the simulator;
- make sure the aircraft altitude is above the plan's security-height floor;
- ignore stale telemetry from aircraft not included in the selected session;
- stop old simulators before starting another acceptance run.

For a clean restart on Linux:

```sh
pkill -f 'var/aircrafts/(Easystar_3_TCAS|Adam_TCAS|ARDrone_2_TCAS)/(sim|nps)/simsitl' || true
```

Build success alone is not acceptance. The required evidence is live TA, live
coordinated RA, an altitude response, and `TCAS_RESOLVED` from the participating
aircraft.