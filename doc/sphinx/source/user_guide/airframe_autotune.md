# Airframe Auto-Tuning from Flight Logs

Manual test flights produce a lot of numbers. Most of them end up in a log file that nobody opens again. The `log_2_tuned_airframe.py` tool opens them for you: it reads a Paparazzi telemetry log, measures how the aircraft actually flew, derives a small set of airframe parameter corrections from physics, writes a **new, version-numbered airframe file**, compiles it to be sure it is valid, and prints a report explaining every change in plain language.

The tool never edits your original airframe file. It only ever creates `<airframe>_optim_001.xml`, `_optim_002.xml`, and so on next to it, so every tuning round stays reviewable and reversible.

```{image} ../../../images/tools/autotune/process_overview.svg
:alt: Autotune pipeline from log files to a verified airframe XML
:align: center
:width: 100%
```

## What It Solves

The tool targets the complaints that show up most often after the first stabilized (AUTO1) flights of a new fixed-wing airframe, and prepares the energy and navigation loops for the first autonomous (AUTO2) flight:

| Symptom in flight | What the log shows | What is corrected |
| --- | --- | --- |
| Elevator sits permanently off-centre in level flight; the pitch integrator is doing the trimming | The pitch loop holds a constant `COMMANDS` elevator while body pitch $\theta$ from `ATTITUDE` differs from the `DESIRED` pitch | `COMMAND_PITCH_TRIM` absorbs the steady elevator; the nominal cruise pitch is set to the measured value |
| One wing heavy; aileron never centred | The roll loop holds a constant `COMMANDS` aileron to fly wings level | `COMMAND_ROLL_TRIM` absorbs the steady aileron |
| Turns are wide, the pilot has to hold a lot of rudder to keep them coordinated | `COMMANDS` yaw is large and saturating whenever `ATTITUDE` roll is large | Roll is mixed into the ruddervators (`RUDDERVONS_OF_ROLL`), rudder attenuation in AUTO is removed, navigation radii are set to the requested minimum |
| The ETECS energy controller was tuned from a simulator model and the real aircraft flies at a different throttle, speed or climb capability | Throttle, climb rate and airspeed in quasi-steady flight describe the real plant | Cruise throttle, cruise airspeed, throttle-per-climb, pitch-per-climb, throttle-per-airspeed, maximum climb and glide ratio are set from measurements |
| Unknown whether the course loop will oscillate in AUTO2 | Roll-setpoint tracking lag in AUTO1; course error statistics if AUTO2 exists | `H_CTL_COURSE_PGAIN` is reduced when the bandwidth separation to the roll loop is unsafe, or trimmed &plusmn;20 % from measured AUTO2 tracking |

:::{important}
Everything except the *measured* course-loop retune is derived from **AUTO1** flight. You do not need to risk an autonomous flight to get trims and a realistic energy-controller starting point: the aircraft only has to fly stabilized straight legs, turns, a full-throttle climb and an idle glide. When the log contains at least 60 s of airborne AUTO2, the course loop is tuned from data as well; otherwise the report explains what a short AUTO2 flight would add.
:::

## Requirements

- Python 3.8 or newer with `numpy` (already required by the Paparazzi Python tools).
- A `.log`/`.data` pair produced by the Paparazzi server in `var/logs`. The `.log` file must contain the `<aircraft>` entry for the aircraft you tune; it always does when the log was recorded with the normal ground segment.
- The aircraft must be defined in `conf/conf.xml` under the same name and `ac_id` as in the log, because the build verification calls `make -f Makefile.ac AIRCRAFT=<name> ap.compile`.
- A working `ap` build of the aircraft. The tool does not fix build problems unrelated to the airframe.

:::{tip}
A good tuning flight has a few minutes of stabilized **AUTO1** with: some straight and level legs, sustained turns in both directions, at least one 10 s full-throttle climb at constant attitude, and one idle-throttle glide of 10 s or more. Pure MANUAL flight is ignored, because the controller setpoints needed for the comparison are not available in that mode.
:::

## Quick Start

```bash
cd ~/paparazzi
./sw/tools/autotune/log_2_tuned_airframe.py --log-name 26_09_05__18_42_28 --ac-id 129
```

This reads `var/logs/26_09_05__18_42_28.{log,data}`, finds the airframe path for AC_ID 129 inside the log, writes `conf/airframes/OPENUAS/<airframe>_optim_NNN.xml`, compiles the `ap` target with that file, and prints the report.

To only look at the proposed changes without compiling:

```bash
./sw/tools/autotune/log_2_tuned_airframe.py --log-name 26_09_05__18_42_28 --ac-id 129 --no-build
```

### Command Line Options

| Option | Default | Meaning |
| --- | --- | --- |
| `--log-name NAME` | `26_09_05__18_42_28` | Log basename without extension; both `NAME.log` and `NAME.data` must exist |
| `--log-dir DIR` | `$PAPARAZZI_HOME/var/logs` | Where to find the log pair (symlinks are resolved) |
| `--airframe-out DIR` | `$PAPARAZZI_HOME/conf/airframes/OPENUAS` | Where the `_optim_NNN.xml` and report are written |
| `--base XML` | airframe referenced in the log | Airframe file to tune; pass a previous `_optim_NNN.xml` to iterate on it |
| `--ac-id N` | `129` | Aircraft ID whose messages are analysed |
| `--turn-radius R` | `30.0` | Requested minimum autonomous turn radius in metres |
| `--set NAME=VALUE` | none | Force an airframe define (repeatable), e.g. `--set BODY_TO_IMU_THETA=0.16` after a bench IMU check |
| `--target T` | `ap` | Firmware target used for the compile check |
| `--no-build` | off | Skip the compile check (report says `SKIPPED`) |
| `-v` | off | Verbose logging |

All paths are resolved with `pathlib.Path.resolve()`, so a `var/logs` or `conf/airframes` symbolic link pointing somewhere else on the disk works, and a broken link produces a clear error rather than a traceback.

## How the Tuning Works

```{image} ../../../images/tools/autotune/tuning_logic.svg
:alt: Iteration 1 - mapping from flight segments and measurements to trim, mixing and radius parameters
:align: center
:width: 100%
```

```{image} ../../../images/tools/autotune/tuning_logic_iter2.svg
:alt: Iteration 2 - roll trim, ETECS plant identification and course loop
:align: center
:width: 100%
```

### Step 1: Reading the log

The `.data` file is a plain text stream with one message per line: `<time> <ac_id> <MESSAGE> <fields...>`. The tool makes a single pass over the file, keeps only the messages of the requested `ac_id` that it needs (`ATTITUDE`, `GPS`, `COMMANDS`, `PPRZ_MODE`, `AIR_DATA`, `ESTIMATOR`, `DESIRED`, `NAVIGATION`, `ENERGY`), and converts them to `numpy` arrays. Array-type fields such as `COMMANDS.values` are comma-separated in the log; both separators are accepted. Corrupt lines are counted and skipped, not fatal.

The `.log` file carries a copy of the airframe **as it was flown**. The tool reads its `<define>` values so that a trim correction is always added to the trim that was actually in effect during the flight, even when you tune a different base file with `--base`.

Airspeed comes from `AIR_DATA.airspeed` (equivalent airspeed from the pitot sensor). If that field is missing or never above 3 m/s, GPS ground speed is used instead and the report says so.

### Step 2: Segmenting the flight

All signals are resampled onto the `ATTITUDE` timeline. The airborne window is the span where GPS ground speed exceeds 6 m/s. Inside it these segments are selected:

- **Straight and level**: |roll| < 5°, |vertical speed| < 0.5 m/s, |yaw rate| < 5°/s, airspeed at least 75 % of the flight mean, mode AUTO1 or AUTO2. At least 30 samples.
- **Banked turns**: |roll| > 15°, airspeed above 7 m/s, mode AUTO1 or AUTO2. At least 100 samples.
- **Quasi-steady**: |roll| < 10°, airspeed above 70 % of the mean, throttle above 2 %, any climb rate, AUTO1 or AUTO2. At least 200 samples. Sub-sets: *cruise* (|Vz| < 0.3 m/s), *full throttle* (> 95 %), *idle glide* (< 3 % and sinking).
- **AUTO2 navigation**: mode AUTO2 and airborne, at least 60 s in total.

If a segment is too short the corresponding correction is skipped and the report explains why.

### Step 3: Configuration faults that make autonomous flight unsafe

Before any tuning, the tool checks things that no gain can compensate for:

- **State airspeed alive?** `AIRSPEED.airspeed` is `stateGetAirspeed()`, the value every autonomous loop uses; `AIR_DATA.airspeed` is only the sensor module's output. If the sensor reports speed but the state stays at zero, `USE_AIRSPEED` never reached the `ap` target (setting `USE_AIRSPEED_SDP3X` alone does *not* imply it). The tool inserts `<define name="USE_AIRSPEED" value="TRUE"/>` into the `ap` target's airspeed module and explains the consequence: with airspeed = 0 the ETECS speed error is a constant +10 m/s, so on AUTO2 entry the controller pitches to `PITCH_MIN_SETPOINT` and floors the throttle regardless of altitude. This exact fault produced a dive to ground level on the Talon 250G's first AUTO2 attempt.
- **AUTO2 anomaly detector**: minimum height above ground, maximum sink, fraction of AUTO2 time with the pitch setpoint at its lower limit, and the altitude error at AUTO2 entry, so a "why did it dive" question is answered by the report.
- **IMU alignment sanity**: level-flight body pitch normalised to 10 m/s ($\theta \sim a + b/V^2$). A small foam wing cruises at 2–3° alpha; a value above 4.5° with the flown `IMU_BODY_TO_IMU_THETA` produces a bench-check instruction and a suggested value. Flight data alone cannot separate IMU tilt from a heavier or slower aircraft, so this is advice, not an automatic edit; apply the bench result with `--set`.
- **Safety envelope** for aircraft with little power margin (cruise throttle > 70 %): `ROLL_MAX_SETPOINT`/`AUTO1_MAX_ROLL` are limited so that the accelerated stall speed $V_s\sqrt{1/\cos\varphi}$ stays 15 % below cruise speed; `PITCH_MIN_SETPOINT` is limited to the idle glide angle plus 3°; `MAX_ACCELERATION`, `AUTO_PITCH_OF_AIRSPEED_IGAIN` and `ENERGY_DIFF_IGAIN` are reduced so a speed error cannot wind the pitch reference to its limit.

### Step 4: Pitch and roll trim

In straight and level flight the attitude loops should have nothing left to do. If they hold a constant elevator or aileron command, that steady deflection is a trim the airframe needs (a heavy wing, a slightly bent ruddervator, a misaligned servo arm). Paparazzi adds `COMMAND_PITCH_TRIM` and `COMMAND_ROLL_TRIM` to the commands *after* the control loops (in `actuators.c`), so the correct new trim is simply

$$\text{trim}_{\text{new}} = \text{trim}_{\text{flown}} + \overline{\text{command}_{\text{level}}}$$

rounded to 10 pprz and clipped to the firmware limit of &plusmn;960 pprz (10 % of full scale). If the required trim exceeds that limit the report tells you to move the mechanical linkage instead. Steady commands below 100 pprz are treated as noise. The report also prints the body attitude versus the `DESIRED` attitude so you can see the loop error that the trim was hiding.

Because the measured level pitch is the real cruise attitude, `V_CTL_AUTO_THROTTLE_NOMINAL_CRUISE_PITCH` is set to $\theta$ in radians when that lies in a sensible range (0 to 0.2 rad).

:::{note}
`INS_PITCH_NEUTRAL_DEFAULT` / `INS_ROLL_NEUTRAL_DEFAULT` are **not** touched. Those defines are only read by a few legacy INS drivers (Xsens, VN100, ArduIMU, `ahrs_sim`); with `ahrs float_cmpl_quat` + `ins alt_float` they have no effect. The IMU-to-body mounting angle lives in `IMU_BODY_TO_IMU_THETA` and should be set from a bench measurement, not from flight data.
:::

### Step 5: Ruddervator mixing and turn radius

A coordinated turn at bank angle $\varphi$ and airspeed $V$ has a yaw rate of $g\tan\varphi / V$. In the turning segment the tool compares the achieved yaw rate with that ideal value and, more importantly, measures how much rudder the pilot had to hold per degree of bank. On a V-tail (ruddervator) aircraft with only pitch and yaw in the tail mixer, that rudder must come from the pilot's thumb in every turn. The tool converts the measured slope (pprz per degree of bank) into a roll-to-rudder gain

$$k = \frac{\text{pprz/deg} \cdot 45^\circ}{9600}$$

clamped to [0.3, 1.0], and rewrites the command laws so the ruddervators receive `@YAW + k·@ROLL` instead of `@YAW` alone. The gain is exposed as `RUDDERVONS_OF_ROLL` in the `MIXER` section so it can be trimmed by hand later. The `auto_rc_commands` attenuation `@YAW*0.9` is removed, since attenuating a channel that was already saturating is counter-productive once the mixer provides coordination.

Finally, `MIN_CIRCLE_RADIUS` and `LANDING_CIRCLE_RADIUS` are set to the requested `--turn-radius`. The report states the bank angle required for that radius at the measured airspeed, $\varphi = \arctan\left(V^2 / (gR)\right)$, so you can confirm it is well inside `H_CTL_ROLL_MAX_SETPOINT`.

:::{note}
All edits are made as surgical text substitutions on the original XML, not through an XML library rewrite. Your comments, indentation, and commented-out experiments are preserved exactly; only the touched attribute values and the two inserted lines change. Every inserted line carries an `autotune` XML comment.
:::

### Step 6: ETECS energy-controller plant

The `energyadaptive` (ETECS) vertical controller computes throttle and pitch as

$$\delta_T = T_{\text{cruise}} + k_{T,\dot z}\,\dot z_{\text{sp}} + k_{T,V}\,(V_{\text{sp}} - V) + \ldots \qquad
\theta = \theta_{\text{cruise}} + k_{\theta,\dot z}\,\dot z_{\text{sp}} + \ldots$$

The four feedforward coefficients are properties of the airframe and propulsion, not of the controller, so they can be identified from **open-loop AUTO1 data** without ever engaging the vertical loop. On the quasi-steady segment the tool computes:

| Parameter | How it is measured |
| --- | --- |
| `AUTO_THROTTLE_NOMINAL_CRUISE_THROTTLE` | median throttle while $\lvert \dot z \rvert < 0.3$ m/s |
| `NOMINAL_AIRSPEED`, `AUTO_AIRSPEED_SETPOINT`, `TRACKING_AIRSPEED` | mean airspeed in that same cruise set (tracking = cruise + 1 m/s) |
| `AUTO_THROTTLE_OF_AIRSPEED_PGAIN` | slope of throttle on airspeed from a least-squares fit `throttle ~ 1 + Vz + V` |
| `AUTO_THROTTLE_PITCH_OF_VZ_PGAIN` | slope of body pitch on climb rate from `theta ~ 1 + Vz` |
| `AUTO_THROTTLE_CLIMB_THROTTLE_INCREMENT` | $(1 - T_{\text{cruise}}) / \dot z_{\text{WOT}}$, where $\dot z_{\text{WOT}}$ is the 75th percentile climb rate at more than 95 % throttle |
| `ALTITUDE_MAX_CLIMB` | $0.8\,\dot z_{\text{WOT}}$, only lowered, never raised |
| `GLIDE_RATIO` | median $V / (-\dot z)$ at idle throttle |

Each regression is only used if its $R^2$ is reasonable and the coefficient lies in a physically plausible band; otherwise the existing value stays and the report shows `n/a`. When the cruise throttle is above 70 % the report adds a **propulsion margin** advice: the aircraft has little climb authority left, which is worth knowing before the first AUTO2 flight.

### Step 7: Course loop

The navigation course loop turns a course error into a roll setpoint: $\phi_{\text{sp}} = K_c\,(V/V_{\text{nom}})\,e_{\text{course}}$. Its bandwidth is roughly $K_c\,g/V$, and it must stay well below the bandwidth of the inner roll loop or the aircraft will weave.

- **Without AUTO2 data** (the normal case for a first flight) the tool measures how quickly `ATTITUDE.phi` follows `DESIRED.roll` in AUTO1 (cross-correlation lag and RMS). If the roll bandwidth is less than three times the course bandwidth, `H_CTL_COURSE_PGAIN` is reduced to restore the margin. Either way the report states the separation ratio and recommends a 1–2 minute AUTO2 circle at safe altitude as the next data source.
- **With at least 60 s of airborne AUTO2** the course error `DESIRED.course - GPS.course` is evaluated directly. More than 0.3 sign changes per second means the loop is oscillating and the gain is cut by 20 %; an RMS error above 15° without oscillation means it is sluggish and the gain is raised by 20 %; otherwise it is reported as healthy and left alone.

### Step 8: Versioning, validation, and compile check

```{image} ../../../images/tools/autotune/build_verification.svg
:alt: Build verification flow with guaranteed restore of the base airframe
:align: center
:width: 60%
```

The output name is `<base>_optim_NNN.xml` where `NNN` is one higher than the highest existing number in the output directory. The file is first checked for well-formedness. Then, unless `--no-build` is given, the tuned file is temporarily copied over the base airframe path, `make -C $PAPARAZZI_HOME -f Makefile.ac AIRCRAFT=<name> ap.compile` is run, and the base airframe is restored in a `finally` block so it is untouched even if the build crashes or is interrupted. A failed build raises an exception after writing the report, and the last 40 lines of build output are included in it.

## Reading the Report

The report is printed to the terminal and saved as `<output>.report.txt` next to the generated XML. It has these blocks:

1. **Header**: log, aircraft, base and output airframe paths, and `build check: PASS | FAIL | SKIPPED`.
2. **Flight**: airborne time split into AUTO1 and AUTO2, airspeed source and mean.
3. **Pitch / roll trim**: body vs demanded attitude, steady elevator and aileron commands.
4. **Turns / yaw**: bank, rudder usage and saturation, coordination ratio, achieved radius, rudder-per-bank slope, bank needed for the target radius.
5. **ETECS plant**: cruise throttle and speed, the three regression slopes, sustained full-throttle climb, glide ratio.
6. **Roll / course loops**: roll setpoint tracking lag and RMS, AUTO2 course statistics (or `n/a`).
7. **XML changes**: numbered list with `before`, `after`, and `why` for every parameter touched.
8. **Advice / next flight**: findings that need a human decision or more data rather than an XML edit.
9. **Build output tail** when a compile was run.

Example excerpt from the Talon 250G run on an AUTO1-only log:

```text
-- FLIGHT ----------------------------------------------------------------
  airborne 541 s | AUTO1 503 s | AUTO2 19 s
-- PITCH / ROLL TRIM  (straight+level, n=77) -------------------------
  body pitch / demanded  : +2.43 / -3.35 deg  -> error +5.78 deg
  steady elevator cmd    : +903 pprz
  bank / demanded        : -1.00 / +1.95 deg  -> error -2.95 deg
  steady aileron cmd     : -353 pprz
-- ETECS PLANT  (quasi-steady, n=970) ---------------------------------
  cruise throttle        : 84% at 10.3 m/s
  pitch per m/s climb    : 0.057 rad/(m/s)
  sustained climb @ WOT  : 1.34 m/s  (28% of flight at full throttle)
  glide ratio (idle)     : 4.9 (n=114)

1. TRIM: PITCH_TRIM
   before : 0.0
   after  : 900
   why    : the pitch loop held a steady +903 pprz elevator in level flight (body
            pitch +2.4 deg vs demanded -3.3 deg); moving that into the command trim
            centres the servo and frees the integrator (flown trim +0)

-- ADVICE / NEXT FLIGHT --------------------------------------------------
* [propulsion margin] holding altitude needs 84% throttle and the aircraft spent
  28% of the flight at full throttle. That leaves little climb authority for ETECS;
  check propeller, motor kV, battery sag (ENERGY message) and airframe drag before
  an AUTO2 flight.
* [course loop] no airborne AUTO2 time >= 60 s in this log (19 s found). Model
  check: roll tracks its setpoint with 0.20 s lag ... separation 5.7x (OK).
```

That report is a good example of why the tool exists: the airframe file said cruise was 28 % throttle at 12 m/s with 4.3 m/s of climb available (from the simulator model), while the aircraft actually needed 84 % at 10.3 m/s and could only climb 1.3 m/s. Flying AUTO2 with the model values would have made the energy controller demand climbs it could not deliver.

## Putting the Result into Service

1. Read the report, the **advice** block, and the `diff` between the base airframe and the `_optim_NNN.xml` file. The tool is deliberately conservative, but you know your aircraft.
2. Point the aircraft entry in `conf/conf.xml` at the new file (or copy the accepted values into the base airframe), rebuild, and upload.
3. Fly the same kind of AUTO1 test again and run the tool on the new log with `--base` pointing at the file you flew. A well-trimmed aircraft produces near-zero steady commands and a near-zero rudder-per-bank slope, so only the ETECS and radius parameters are refreshed.
4. When the trims and the ETECS numbers are stable, fly a short AUTO2 circle at safe altitude. The next run then tunes `H_CTL_COURSE_PGAIN` from measured tracking instead of the model check.

:::{warning}
Auto-tuning changes control mixing and attitude references. Always review the generated XML before flight, keep the first flight with a new file at safe altitude, and be ready to switch to MANUAL. The compile check proves the file is valid, not that it flies well.
:::

## Files

| Path | Purpose |
| --- | --- |
| `sw/tools/autotune/log_2_tuned_airframe.py` | The tool |
| `conf/airframes/<group>/<airframe>_optim_NNN.xml` | Generated, version-numbered airframes |
| `conf/airframes/<group>/<airframe>_optim_NNN.report.txt` | Audit report of each run |
| `doc/sphinx/source/user_guide/airframe_autotune.md` | This page (Sphinx user guide) |
| `doc/images/tools/autotune/*.dot` | Graphviz sources of the diagrams on this page; render with `dot -Tsvg X.dot -o X.svg` |
| `doc/tools/autotune/airframe_autotune.html` | Standalone, self-contained HTML copy of this page (diagrams inlined) |

## Extending the Tool

The tuning rules live in `tune_airframe()`. Each rule calls `AirframeEditor.set_define()` (or `add_define_to_section()`), which records a `Change(what, before, after, rationale)` that feeds the report, or appends an `Advice(topic, text)` when the finding needs a human rather than an edit. Adding a rule for a new parameter means measuring the relevant quantity in `analyze()`, storing it in the `Analysis` dataclass, and adding one `set_define()` call with a plausibility check and a rationale string.

Candidates for a third iteration, once AUTO2 logs are available: `H_CTL_COURSE_DGAIN` and `CARROT` from cross-track statistics in `NAVIGATION`, the ETECS `ENERGY_TOT_PGAIN` / `ENERGY_DIFF_PGAIN` closed-loop gains from altitude and airspeed error spectra, and the bank-angle energy feedforwards from throttle behaviour in autonomous circles.
