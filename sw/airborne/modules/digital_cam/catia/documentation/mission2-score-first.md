# Mission 2: Maximize Points, Keep The Airframe

A fixed-hardware implementation plan for the Talon, Tiny1-C, M10N and MORA.

[Documentation Hub](index.html) | [Camera Pipeline](catia_camera_pipeline.html) |
[AI Camera](raspberry_pi_ai_camera.html) | [LWIR Calibration](lwir-calibration.html) |
[EARcam Guide](earcam-loudest-spot-explained.html) | [EARcam Data Flow](earcam-dataflow.html) |
[Mission 2 Plan](mission2-score-first.html)

## Quick 'n Dirty Checklist

**Short on time? Start here.** Assume current photo timing is usable for a
first attempt, not proven accurate. No timing experiment, pose logging, clock
handshake, new firmware just for diagnostics, or new algorithm is required for
this quick path. Keep the normal approximately four-second photo interval.

**Prerequisite TODO:** complete the installed lens/mount calibration using the
[calibration guide](lwir-calibration.html) and put the reviewed calibration file
on MORA before testing. The checklist below assumes
this is done; calibration has not been marked complete by this document.

1. **Start the real camera setup.** Use the established CATIA launch on MORA with
  `--lwircam`, not mock/test mode. Check the intended SD storage is mounted,
  writable and has room. Pass the completed calibration file with
  `--lwir-calibration`. Leave optional motion compensation off for this test.
2. **Take three photos on the ground.** With propulsion disabled, use the normal
  FC photo command and a clearly visible warm target. Check that all three
  JPEGs open and that each reports temperatures when analyzed; captures store
  the temperature plane inside the JPEG. With `--lwir-raw`, check
  instead for a matching `.jpg.raw` file of **196608 bytes** in the current
  256 x 192 image mode.
3. **Check the ground-shot results.** Confirm the calibration loads and inspect
  the JPEG hotspot result, not just its camera GPS tags. Check that the warm
  target is detected. Ground shots need not produce coordinates if AGL/geometry
  is invalid; check the actual target coordinates after the flight.
4. **Make one ordinary short test flight.** Only after the capture check passes,
  use the already-reviewed route and normal margins over a known warm target.
  Keep the existing altitude, speed and turns. No extra low or tight pass.
  Aim to obtain at least one usable target observation, not perfect timing data.
5. **Land, stop cleanly, recover the card.** Wait for capture/writes to finish,
  use the established shutdown/unmount procedure, then copy the JPEGs, plus any
  raw companions if those were enabled. Keep partial results even if some shots failed.
6. **Make one practical decision.** Does an onboard hotspot coordinate fall
  near the known target, and can you retrieve it promptly? If yes, retain this
  setup for the next test. If no, first check detections, whether the intended
  calibration file was loaded, mount orientation and AGL; do not start by
  adding timing algorithms.

**Quick pass:** real images and raw data recovered, at least one plausible
onboard hotspot coordinate, and a working retrieval process. An ordinary map
comparison is a sanity check, not proof of the competition's 5 m tolerance.

On a development PC with ExifTool installed, inspect a recovered image with:

```sh
exiftool -s -ImageDescription -UserComment /path/to/recovered-photo.jpg
```

Look in `LWIR_HOTSPOTS_V1` for the result status, count and per-hotspot
latitude/longitude. ExifTool is only for this optional inspection, not a MORA
dependency. Do not confuse multiple observations with distinct targets.

**Before competition:** identify how the final two onboard coordinates will be
selected and handed over within **300 seconds of landing**. Automatic final
two-target export is not implemented. This quick test does not solve that gap;
manual inspection here is development work, not a claim of onboard autonomy.
Detailed timing and survey checks below can wait until this basic test works.

> **Objective:** maximize our chance of winning with the existing hardware.
> For Mission 2, the practical consequence of airframe loss is that we may not
> recover all the data needed for the submission. Land and retrieve the SD card;
> assess the usable results actually recovered rather than assume that either
> all data or all points are lost. Missions 3 and 4 use separate aircraft.
> Improve the software first; change flight behaviour only when there is no
> good alternative and the aircraft's operating margins remain intact.
> No score calculation authorizes a flight.

## Agreed Constraints

- Keep the existing aircraft, camera, installed optics and GNSS hardware.
- Plan to land after Mission 2 to retrieve the SD card. No in-flight access to
  its results is assumed; onboard result readiness is not submission to the judges.
- Prefer calibration, exposure/pose timing, onboard detection and processing
  improvements that do not change the trajectory or aircraft settings.
- Consider flight-behaviour changes only if those alternatives are inadequate.
- Do not lower the aircraft toward trees, reduce stall margins, tighten turns,
  breach the geofence or compromise return and landing to improve a measurement.
- Keep safety supervision and override authority regardless of autonomy points.
- No changes to flight code, routes, airframe configuration or deployment are
  made by the offline tools described here.

The aircraft allocation is **Mission 2: Talon; Mission 3: a separate quadcopter;
Mission 4: EasyStar**. Do not model loss of the Talon as loss of the Mission 3
or Mission 4 airframe or their availability.

The earlier 40 m AGL example is an error-budget calculation, **not a prescribed
flight altitude**. Use the existing approved flight envelope and clearance
requirements. The rulebook's 80 m AGL maximum is an upper limit, not evidence
that the space below it is clear of obstacles.

## Safety Gate

Here, the safety gate also serves a concrete competitive purpose: avoid losing
the airframe to trees, terrain, loss of control or an exhausted landing reserve.
Protecting people and complying with the flight rules remain mandatory.

Review the existing flight as well as any proposed alternative. A numerical
GNSS accuracy estimate, a geofence, or a terrain-only elevation map cannot
establish tree clearance on its own.

| Review | Evidence required before accepting the flight |
| --- | --- |
| Trees, terrain and structures | A current obstacle assessment, including canopy height and uncertain or unmapped objects; no unverified low pass |
| Vertical clearance | A consistent height datum and a margin covering height-estimation error, obstacle uncertainty and tracking error over the complete route |
| Lateral clearance | The aircraft's swept path and wingspan, turn geometry, navigation uncertainty, tracking error, wind drift and space for a safe abort |
| Flight envelope | Previously validated airspeed, bank and turn limits for this aircraft, load and wind; ground speed is not stall margin |
| Energy and escape | Validated return, missed-approach and landing reserve; a safe continuation if a target or extra observation is abandoned |
| Geofence and people | All route segments, turns, launch, approach and abort paths remain within the approved operating area |
| Software isolation | Image processing must not starve navigation, delay safety commands or exhaust storage; degrade image collection before flight safety |

**If any required clearance is unknown, the affected precision pass is not
approved.** If safe clearance conflicts with the height limit, omit that route
or sector and accept the lost opportunity. Never make the problem disappear
by changing a software margin. Safety intervention takes precedence over any
autonomy-factor penalty.

Neither the score calculator nor the coordinate validator computes obstacle
clearance or models aircraft dynamics. Their safety-review field is a human
review record, not a safety certificate.

## What Earns Points

Sections 5.4.3 and 5.4.4 require two distinct heat-point positions, decimal
degrees, within 5 m, delivered within 300 seconds of landing. Confirm with the
organizers the reference point on each source, survey datum/uncertainty, the
distance convention and the treatment of ambiguous or duplicate submissions.
Table 9's sample coordinates are not actual mission targets.

For the fixed-wing aircraft, Table 8 gives:

```text
HP = 3.5 * number_of_valid_hotspots                  (0, 1 or 2)
CF = 1.0                                          (fixed wing, not adjustable)
CP = min(3, flight_seconds * 50 / (voltage_v * consumed_mah))
W2 = 2 * (1 - exp(-5 / (1.4 * takeoff_mass_kg)))
A2 = 1.0 onboard; 0.7 offboard computation; 0.4 manual actions
S2 = (8 / 11.75) * (CF * HP + CP) * W2 * A2 + landing_points + self_made_points
```

Use actual launch-to-landing-and-stop duration, maximum voltage after charging,
and consumed charge measured by recharge, as described in the rulebook. Do not
substitute battery nameplate capacity for consumed charge. Confirm measurement
practice and the printed formula with the judges; this is an offline estimate,
not an official scoring decision.

For otherwise identical safe fixed-wing flights, one additional accepted
hotspot contributes 3.5 base points, more than the whole 3-point power-factor
range. Reliability on both targets is therefore a strong priority. The factor
uses **time divided by consumed charge**, so it does not simply reward the
shortest flight. At constant average electrical consumption rate, time and
charge rise together. Do not extend a flight to manipulate this factor or
spend safety reserve on another observation.

Keep mass and configuration fixed in comparisons. Preserve onboard computation
and automatic result generation where practical. Evaluate precision landing
points only using an already validated safe landing procedure; they do not
justify a risky approach. Self-made points are a documentation/judging input,
not a reason to modify the aircraft.

Landing is also how we access Mission 2 data, not merely an optional bonus.
The calculator's `--landing none` means zero landing-bonus points after a
completed flight, not an airborne aircraft or an unrecovered SD card.

Optimize the competition outcome, not just the score conditional on a perfect
flight. Include the chance of recovering the Mission 2 data and delivering
usable coordinates on time. Incomplete data does not automatically mean zero
valid hotspots: assess whatever results were recovered, without claiming missing
observations or a complete two-target result. The calculator does not model
data-recovery probabilities or the cost of airframe loss; do not invent them
to justify a higher nominal score. Its review gate preserves the agreed flight
constraints while we compare measured improvements.

## First-Test Checklist

**Software checkpoint, 2026-09-10:** native capture integration, actual-SDK
callback regression, native raw/EXIF preservation, pose sender (eight host
configurations), logger, clock bounds and UART partial-write tests pass. The
current ARM64 build also passes. These are hardware-free checks, not a configured
Talon firmware build or a camera/flight test. Existing vendor `fread` and static
OpenCV `dlopen` build warnings remain; execution on MORA must still be checked.

Freeze features here. The first test is an operational bench test, followed by
a baseline accuracy test when measured calibration and independent references
are available. Keep the Talon, Tiny1-C, M10N and MORA, the existing validated
flight behaviour and the usual approximately four-second shot interval.

### Prepare The Bench

1. Record the FC/MORA software revisions, camera mode, installed lens and mount.
  Build and review the **actual Talon configuration** with
  `DIGITAL_CAM_UART_POSE_STREAM=1` before installing it. Verify the configured
  UART TX buffer can provide at least **162 free bytes** for clocked poses plus
  command headroom; a 128-byte ring skips every sample. No airframe has been
  enabled or firmware installed by this software checkpoint.
2. Use the established bench setup with propulsion disabled. Deploy the reviewed
  matching ARM64 binaries through the existing installation procedure. Confirm
  CATIA uses the real `--lwircam` backend, actual FC serial device and intended
  SD photo directory, not `--test`, mock transforms or a local simulator.
3. Add the following to CATIA's **actual start command**. Create the
  directory first on the intended mounted storage, writable by CATIA's user;
  this example path does not itself prove that an SD card is mounted there.

  ```text
  --pose-log /home/air/digital_cam/pose-logs
  --clock-align
  ```

  Omit `--lwir-motion-compensation` for this test. Keep the established launch
  arguments and add `--debug`; save stdout/stderr
  through shutdown. Avoid starting a second CATIA instance beside its service.
  Add `--lwir-calibration` only with the measured, reviewed installed-camera
  calibration when available. Missing/unverified calibration may block real
  coordinates, but does not prevent testing capture and evidence storage.

### Run And Check

1. Allow normal camera warm-up, then request at least 15 shots about four seconds
  apart using the existing FC command path. Stop requests before shutdown.
  Confirm every acknowledged capture has a readable JPEG whose stored
  temperature plane still yields hotspot results. If `--lwir-raw` is
  set, check the matching untouched `.jpg.raw` file instead; in the current
  256 x 192 image mode it is 196608 bytes, generally `4 * width * height` bytes,
  and two file renames are not a crash-atomic pair, so check both recovered files.
2. Check image EXIF for `capture_time_kind=sdk_callback_not_exposure`, nonzero
  request time, strictly later frame-arrival time and increasing callback
  sequence within the same capture-server run. Sequence gaps between shots
  are normal; one SDK delivery must not be reused for separate requests.
3. Check that a schema-2 pose CSV contains continuing samples, that accepted
  clock replies increase, and that mapped sample intervals continue after
  startup. JPEG and CSV must share a known `mora_boot_id`. Inspect UART skips,
  queue/callback drops, rejected data and clock rejections; unexplained loss,
  persistent unmapped rows or stalled capture fails the timing-evidence gate.
  Mapping is not sensor exposure synchronization and is not used by projection.
4. Stop CATIA cleanly using the established service stop or SIGINT/SIGTERM.
  Wait for capture workers and the pose writer to finish. Require final pose
  `accepted == synced`, `error=0`, no incomplete-log report and no unexplained
  drops/rejections. Do not pull power to shorten a blocked SD shutdown.
5. Complete the established clean unmount/shutdown and recover the card. Retain
  JPEG/raw pairs, pose CSV, full console/final status, software identifiers,
  camera settings and calibration together under one test identifier. Verify
  the recovered copies, not just files still visible on MORA.

**Bench pass:** captures, timing evidence and cleanly recovered files meet all
checks above without disrupting ordinary commands. Record any failure and fix
that observed issue before adding algorithms. This does not certify five-metre
accuracy or complete competition submission readiness.

### First Accuracy Baseline

Use measured lens/mount calibration and independently surveyed target positions
with stated uncertainty. Do not turn an example calibration's `verified` flag
on to bypass measurement. Keep compensation off for the baseline and use the
existing reviewed route, clearance and landing margins. Preserve recovered
onboard coordinates, reference truth and the evaluator's report, including
partial results. Check height/AGL assumptions and GNSS quality explicitly.

Rehearse landing, shutdown, card access, copying and handover against the full
300-second deadline (180-second rehearsal target). The first bench/baseline
test may archive individual image results; automatic final two-target export
is **not implemented** and a timed archive copy is not a completed scored
submission. Pose interpolation, effective-lag estimation, prediction, fusion,
replanning and final export are deferred beyond this first-test scope.

## Implementation Order

| Stage | Work | Gate before proceeding |
| --- | --- | --- |
| 1. Baseline evidence | Record the present route, capture mode, measured mass, energy, timing and final submissions; establish independent reference positions | Safety review complete; measurements and their uncertainty identified |
| 2. Same flight, better measurements | Validate existing lens/mount calibration, native-frame integrity, timing, navigation age, height datum and detector thresholds | Held-out results improve without aircraft or trajectory changes |
| 3. Same flight, better onboard decisions | Select well-timed, well-conditioned observations already available; associate two targets and combine observations robustly | No double counting; uncertainty includes common GNSS bias; deadline and compute/storage budgets met |
| 4. Conditional flight review | Only if stages 2-3 cannot meet the target, assess the smallest useful change inside the validated envelope | Obstacle/terrain review, simulation/replay and supervised test evidence; no automatically generated low or tight pursuit manoeuvre |
| 5. Blind mission validation | Freeze calibration and algorithms, then rehearse capture, landing, SD-card retrieval and actual submission | Both targets within 5 m on time, with measured reliability, recovery and safety evidence |

The current implementation measures server-request-to-frame-arrival time, not
true exposure time. The optional constant-velocity correction is a partial
estimate and is off by default. Test sensor buffering, FC pose age, clock offset
and changing attitude before attributing better coordinates to synchronization.
The eight-frame stability counter now runs between requests; its remaining
startup/recovery criteria still need camera evidence. Do not remove quality
checks solely to make a timing statistic look better.

The next evidence component is now implemented: an opt-in 10 Hz timestamped FC
pose stream, a bounded CATIA CSV logger on MORA, and monotonic request/frame-arrival times
in image EXIF with the same boot identity. The usual four-second shot interval
is unchanged. See [pose and image timing evidence](catia_camera_pipeline.html#pose-and-image-timing-evidence)
for enabling, buffer requirements, units, loss counters and SD-card retrieval.
An opt-in request/reply exchange now maps FC sample times to bounded MORA
intervals in schema-2 CSV. It does not synchronize camera exposures or feed
mapped poses into geolocation. No airframe configuration enables it
automatically; no hardware was changed.

Still captures now use timestamped SDK callbacks and a latest-frame mailbox,
avoiding reproduced duplicate reads from the bundled SDK's polling buffer.
No evidence currently establishes variable Tiny1-C integration time. First
collect the bench/baseline evidence above. A mode-specific effective lag and
pose interpolation are later candidates if the measured error requires them;
do not assume zero sensor latency. See the
[investigation and prediction model](catia_camera_pipeline.html#lwir-integration-and-effective-lag).

For GNSS, distinguish dimensionless HDOP from a horizontal-accuracy estimate in
metres. Use existing M10N data and independent reference measurements. Never
use the same receiver's average as proof of its absolute accuracy. Combining
neighboring frames does not remove a shared GNSS bias. If the existing hardware
cannot satisfy the budget in the tested conditions, report that limitation and
accept fewer points; hardware replacement is outside this plan.

The 4 m engineering target leaves 1 m below the rulebook tolerance. It must be
tested at the **actual safe altitude and viewing angles**, not carried over
unchanged from a 40 m calculation. If a well-conditioned image is unavailable,
skip it or retain an explicitly uncertain estimate rather than inventing
confidence or steering toward an obstacle.

## Land, Retrieve, Submit

The intended sequence is **Mission 2 capture and onboard processing -> return
and land -> retrieve SD-card results -> submit**. Plan the return and landing
after Mission 2 rather than depend on an in-flight download or postpone access
for an unrelated mission leg. Use the existing validated return/landing sequence;
this plan does not add a new landing controller or change the route automatically.
End acquisition in time to retain the required return, approach and landing
reserve, even when target evidence is incomplete.

The onboard software should prepare a small, clearly named, flight-identified
submission table with the two final coordinates and retain supporting JPEG/raw
pairs separately. Keep a recoverable result as processing progresses, and finish
pending writes before storage removal. Durable result publication and automatic
final two-target export are implementation requirements, **not capabilities
provided by the offline evaluator**. Do not rely on a successful rename alone
as proof that data has reached the SD card.

Rehearse the complete handover with the actual equipment and team:

1. Record the landing event used by the judges for the five-minute deadline;
  confirm its definition in advance. The timer does not start again at card removal.
2. Stop and disarm, recover the aircraft, and use the established clean shutdown
  or unmount procedure before disconnecting power or removing storage. Include
  walking, access and shutdown time; do not rush card removal or recovery.
3. Read the card, confirm the flight identifier, and copy the small finalized
  table first. Do not wait for a full image archive copy before handing it over.
4. Deliver the unchanged onboard result through the judges' agreed submission
  method and record that time. File-copy completion is not yet judge submission.
5. Archive the supporting data and prepare the Talon for any further Talon
  flights after completing this handover. Missions 3 and 4 use their own aircraft.

Use **180 seconds from landing to submission as a rehearsal target**, leaving
120 seconds below the 300-second limit. This is a planning target, not a measured
retrieval time or permission to skip proper shutdown. Test card mounting, file
identification and delivery as well as coordinate accuracy. Fix a slow handover
through preparation and software, not a rushed recovery or riskier landing.

Onboard calculation and SD-card delivery are different operations. Plan for
copying and submitting already-computed coordinates, not manual target selection
or localization on the laptop after landing. Confirm the organizers' treatment
of result transfer under the autonomy rules; SD delivery by itself does not
establish the awarded autonomy factor. Do not claim a completed two-target
result when the onboard file is missing, incomplete or from an earlier flight.

## Offline Assessment Tool

[mission2_assess.py](mission2_assess.py) is a development-PC tool. It has no
flight-control, deployment, telemetry or network output. Its JSON is an offline
report on standard output, not a new runtime JPEG sidecar. It does not introduce
Python or GeographicLib into the flight application. Development validation on
a PC is separate from doing scored mission computation offboard.

From the Paparazzi repository root, prepare an isolated environment:

```sh
python3 -m venv .venv-mission2
.venv-mission2/bin/pip install -r \
  sw/airborne/modules/digital_cam/catia/documentation/mission2-assessment-requirements.txt
.venv-mission2/bin/python \
  sw/airborne/modules/digital_cam/catia/documentation/test_mission2_assess.py
```

### Compare Scores

The following is an **illustrative scenario**, not measured Talon performance
and not an approved flight. By default, a safety-unreviewed scenario produces
`score: null` and exits with status 2:

```sh
.venv-mission2/bin/python \
  sw/airborne/modules/digital_cam/catia/documentation/mission2_assess.py score \
  --mass-kg 2 --voltage-v 16.8 --flight-seconds 600 --consumed-mah 1000 \
  --valid-hotspots 2 --autonomy onboard --landing zone --self-made-points 0
```

After a real human review, `--safety-review passed` records that review and
allows calculation. `--safety-review rejected` blocks the score. Setting the
field does not inspect obstacles, authorize flight or resolve safety concerns.
For actual trials, use measured values and valid hotspot counts; for scenario
analysis, label the inputs as hypothetical. No reliability probabilities or
actual receiver accuracy are inferred from a score.

Compare `gain_per_additional_valid_hotspot` and
`remaining_power_score_headroom` across otherwise comparable safe trials. A
deterministic scenario score is not expected mission score: also compare failure
rates, target misses, report delays and energy-reserve outcomes across trials.

### Validate Final Coordinates

Keep reference truth separate from the localization algorithm. Create a reference
CSV with exactly two rows and these columns:

```csv
target_id,latitude_deg,longitude_deg,uncertainty_m
```

`uncertainty_m` must be a defensible horizontal bound for the independently
surveyed physical reference point, not an unexamined HDOP or one-sigma value.
Use existing independently surveyed control or suitable reference data; do not
alter the aircraft hardware. Without independent truth, report repeatability
only and do not claim absolute 5 m validation.

The actual final submission CSV has at most two rows:

```csv
report_id,latitude_deg,longitude_deg
```

A header-only submission records a missed mission. Do not give the validator
every candidate and let it select the best against known truth. Duplicate IDs,
duplicate coordinates, invalid numbers and extra reports are rejected.

```sh
.venv-mission2/bin/python \
  sw/airborne/modules/digital_cam/catia/documentation/mission2_assess.py validate \
  --reference mission-test/reference.csv --submission mission-test/submitted.csv \
  --submission-delay-s 120
```

Use measured seconds from landing to actual submission, including aircraft
recovery, storage shutdown, SD-card access, copying and handover. Coordinates
computed before landing are not already submitted. The validator cannot verify
those events; it relies on the recorded elapsed time. It computes WGS84 horizontal geodesic distances and matches
reports one-to-one, maximizing the number within 5 m before minimizing total
distance. Each reference can earn at most one provisional hotspot result. An
ambiguous match is flagged and blocks the stricter engineering gate. Coordinate
matching alone does not prove the detector found two distinct physical sources;
inspect the retained image/frame evidence too.

The report distinguishes:

- `provisional_valid_hotspots`: reports within 5 m, provided the deadline was met;
  an offline interpretation, not a judge's award.
- `conservative_error_m`: measured separation plus reference uncertainty.
- `design_gate_passed`: both conservatively within 4 m, unambiguous matching,
  and submission on time. This is **only a localization gate**, never flight safety.

Exit status is 0 when the design gate passes, or 2 when it fails or input is
invalid. Invalid input produces a diagnostic rather than a score. A single
passing submission does not establish statistical reliability; archive all
trials, including failures, and keep tuning and held-out evaluation sets separate.

## Evidence To Keep

For each trial, retain the safety review reference, unchanged airframe/route
version, target survey and uncertainty, native JPEG/raw pairs, available FC and
MORA logs, timing method, calibration version, final submitted coordinates,
landing time, SD-card retrieval time, submission time, takeoff mass, consumed
charge, voltage, flight time and landing
outcome. Record interventions and failures instead of discarding them.

The evaluator implements the first assessment step only. It does not claim
five-metre accuracy, obstacle avoidance, exposure synchronization or a fully
autonomous Mission 2 solution. Subsequent changes should be justified by this
evidence, keeping the fixed-hardware and safety constraints intact.
