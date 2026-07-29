# TCAS for Fixed-Wing and Rotorcraft

## Architecture, Safety Refactoring, and Validation Report

> **Project baseline**
>
> This document describes the Paparazzi TCAS refactoring completed in commit
> `5ad5d0782cb24c40834ee55a25db251b107b590f`, **“TCAS now works for both
> Rotorcraft and Fixedwing”**, dated 2026-07-29. The comparison baseline is
> `968474c84069ab845bd881896dbb0822bdff3c11`.

> **Scope note**
>
> In this document, “TCAS” refers to Paparazzi's onboard traffic-alert and
> vertical collision-avoidance module. It is not a claim of certification or
> equivalence to a certified civil-aviation TCAS II installation. Operational
> deployment still requires scenario testing, scheduler and loss-burst testing,
> hardware-in-the-loop qualification, and aircraft-specific flight testing.

---

## 1. Executive Summary & The Core Challenge

### What TCAS does

A Traffic Collision Avoidance System continuously compares the ownship state
with reported traffic. It estimates whether another aircraft is approaching a
protected volume and escalates through three broad outcomes:

- **No alarm:** represented surveillance was evaluated and no advisory is needed.
- **Traffic advisory (TA):** another aircraft requires immediate attention.
- **Resolution advisory (RA):** a vertical avoidance command is required.

The simple version sounds like geometry: subtract two positions, compare two
velocities, and decide whether the paths conflict. The real engineering problem
is harder. The system must also decide whether its inputs are complete, current,
in the same reference frame, and safe to act on. If information disappears, it
must distinguish **“no conflict”** from **“I can no longer prove there is no
conflict.”**

> **Safety takeaway:** Missing traffic data is not empty air. The refactored
> module reports `TCAS_UNAVAILABLE` or temporarily holds an existing advisory;
> it never converts uncertainty into a synthetic `TCAS_NO_ALARM` result.

### Why rotorcraft support is not a parameter adjustment

A rotorcraft is not merely a fixed-wing aircraft with a lower speed setting.
The two firmware families expose different control contracts and operate under
different assumptions:

| Characteristic | Fixed-wing | Rotorcraft / helicopter |
|---|---|---|
| Normal horizontal motion | Usually continuous | May hover at zero ground velocity |
| Vertical control | Fixed-wing altitude controller | Rotorcraft vertical guidance modes |
| Navigation altitude | MSL-oriented fixed-wing setpoint | Local altitude above an MSL origin |
| Valid stationary state | Unusual but possible in strong wind | Normal hover condition |
| Low-altitude behavior | Forward-flight security constraints | Hover, climb-rate, guided, manual-throttle, failsafe, and kill modes |
| Control authority | `V_CTL_MODE_AUTO_ALT` | `NAV_VERTICAL_MODE_ALT` inside vertical guidance |

The legacy module directly depended on fixed-wing navigation state such as
`ground_alt`, `flight_altitude`, and `v_ctl_altitude_setpoint`. Rotorcraft instead
uses a local vertical frame and produces thrust through `guidance_v`. Applying a
fixed-wing MSL command directly as a rotorcraft local altitude would introduce a
potentially large offset equal to the local origin altitude.

Rotorcraft also makes a previously rare condition routine: **zero velocity**.
A hovering quadcopter reports a valid velocity vector of `(0, 0, 0)`. A
fixed-wing aircraft can also be stationary over the ground in a strong headwind.
Velocity magnitude therefore cannot be used as a validity flag.

### Refactoring objective

The high-level objective was:

1. Keep **one shared surveillance, conflict, and RA policy core**.
2. Produce a **firmware-neutral avoidance altitude in meters MSL**.
3. Add thin, explicit bindings for fixed-wing and rotorcraft control authority.
4. Preserve the fixed-wing hook and altitude-resolution contract while keeping
  rotorcraft application behind its existing mode authority.
5. Harden traffic validity, freshness, prediction, and command caching so both
  firmware families reject unavailable geometry and commands explicitly.
6. Keep all airborne state statically allocated and avoid wire-format widening.

The result is not two TCAS implementations. It is one safety core with two small
control-system adapters.

---

## 2. Key Refactorings & The “WHY” Behind Them

### 2.1 Shared core, thin firmware bindings

The core remains in:

- `sw/airborne/modules/multi/tcas.c`
- `sw/airborne/modules/multi/tcas.h`

It owns:

- traffic-track acquisition;
- relative ENU geometry;
- TA/RA state transitions;
- advisory prioritization;
- peer RA coordination;
- freshness and bounded-hold decisions;
- security-height enforcement; and
- generation of an avoidance altitude in meters MSL.

Firmware-specific code owns only the final question: **may this firmware apply
that altitude command in its current control mode?**

**Why:** Duplicating TCAS logic for rotorcraft would eventually produce two
safety policies with different fixes, thresholds, and edge cases. Keeping the
geometry and advisory policy shared ensures a defect corrected for one aircraft
class is corrected for both.

### 2.2 Firmware-neutral altitude command API

The refactoring introduced:

```c
bool tcas_get_altitude_command(float nominal_altitude_msl,
                               float *altitude_msl);
```

The function does not directly mutate navigation state. It returns an MSL
command only when:

- an RA is active;
- ownship geometry is valid;
- the cached intruder boundary is valid;
- all numerical inputs are finite; and
- the result respects the security-height floor.

**Why:** The old fixed-wing hook mixed advisory policy with one controller's
state. A pure query API creates a stable boundary: TCAS decides the safe altitude,
while each firmware decides whether its current mode grants TCAS authority.

### 2.3 Explicit altitude-frame conversion

The shared policy helper defines the frame conversions:

```text
intruder_msl = ownship_msl + intruder_enu_z - ownship_enu_z
local_altitude = commanded_msl - hmsl_origin
```

Fixed-wing continues to use its MSL navigation altitude. Rotorcraft performs:

1. local NAV altitude → MSL;
2. shared TCAS resolution in MSL; and
3. resolved MSL altitude → local NAV altitude.

**Why:** Position values are meaningful only with their reference frame. Without
this conversion, a mathematically reasonable avoidance command could be applied
in the wrong vertical datum. That is a unit-safe-looking bug with an aircraft-
scale consequence.

> **Critical invariant:** The TCAS core exchanges altitude commands in meters
> MSL. Rotorcraft converts only at the guidance boundary; the conflict algorithm
> itself does not branch on aircraft type.

### 2.4 Rotorcraft integration at the correct authority boundary

Rotorcraft consumes the command in `guidance_v_from_nav()` only when:

```c
nav.vertical_mode == NAV_VERTICAL_MODE_ALT
```

It does not override climb-rate, guided, hover, manual-throttle, failsafe, or
kill behavior.

**Why:** Writing directly into rotorcraft thrust or vertical-reference internals
would bypass the firmware's mode ownership and safety transitions. Integrating
at NAV altitude mode lets the existing reference generator and position
controller execute the command normally. TCAS supplies an altitude, not a
surprise thrust command.

### 2.5 Fixed-wing behavior retained behind an explicit gate

The existing `callTCAS()` compatibility hook remains for fixed-wing. It applies
an RA only when:

- throttle is not killed; and
- `v_ctl_mode == V_CTL_MODE_AUTO_ALT`.

**Why:** The refactor must add rotorcraft support without changing fixed-wing
control authority. Keeping the established hook also avoids forcing unrelated
fixed-wing code through a new integration model.

### 2.6 Pure, testable policy helpers

`sw/airborne/modules/multi/tcas_policy.h` now contains stateless helpers for:

- finite velocity validation;
- surveillance freshness decisions;
- ENU-to-MSL conversion;
- MSL-to-local conversion; and
- climb/descend altitude resolution with a minimum floor.

**Why:** Safety policy buried inside periodic tasks is difficult to test at exact
boundaries. Pure functions permit deterministic tests for `fresh - 1`, `fresh`,
`fresh + 1`, drop thresholds, invalid commands, `NaN`, zero velocity, and frame
conversion without constructing a complete autopilot runtime.

### 2.7 Zero velocity treated as data, not absence

Velocity is usable when all three components are finite:

```c
return isfinite(east) && isfinite(north) && isfinite(up);
```

The magnitude may be zero. Observation presence is tracked separately with
traffic status and freshness metadata.

**Why:** A zero vector is a physical measurement. It represents hover, landed
state, or a fixed-wing aircraft stationary over ground in strong wind. Rejecting
it would make the collision system blind precisely when a moving aircraft could
approach a stationary one.

### 2.8 Checked traffic snapshots for the safety-critical consumer

TCAS uses checked copy-out APIs:

- `traffic_info_get_snapshot()` for legacy traffic; and
- `traffic_info_get_mesh_snapshot()` for mesh traffic.

A snapshot is returned only when the traffic record is registered, position and
velocity observations exist, required coordinate conversion succeeds, and all
ENU values are finite.

The established legacy pointer getters remain compatible for other modules.

**Why:** Globally changing old getters to nullable pointers would force a risky
workspace-wide API migration. TCAS needs stronger guarantees than legacy
consumers, so the safer design is a dedicated checked boundary rather than an
unrelated redesign of formation, follow, camera, and navigation modules.

### 2.9 Monotonic freshness and bounded prediction

Mesh observations use a constant-velocity projection:

$$
\hat{p}(t) = p_0 + v_0 \min(\Delta t, T_{prediction})
$$

Prediction always starts from the stored observation and is clamped to the TA
horizon. It never repeatedly predicts from an already predicted position.
Freshness uses local monotonic receipt time rather than GPS time-of-week.

**Why:** Unbounded extrapolation eventually becomes confident fiction,
especially for maneuverable rotorcraft. GPS time-of-week also wraps and packets
may arrive out of order. Monotonic local age provides a reliable safety clock,
while bounded projection improves short-term geometry without pretending a stale
track remains predictable forever.

### 2.10 Evaluate, hold, or declare unavailable

The policy layer returns one of three actions:

| Action | Meaning | Permitted behavior |
|---|---|---|
| `EVALUATE` | Complete geometry is fresh | Open, update, or clear advisories |
| `HOLD` | Existing advisory, bounded data outage | Preserve it; do not recompute from stale geometry |
| `UNAVAILABLE` | Geometry cannot support evaluation or hold | Do not claim no conflict |

A stale track may hold an existing advisory for a bounded interval, but stale
data cannot open a new advisory or clear an existing conflict as “resolved.”

**Why:** Immediately clearing an RA on a missed packet is unsafe. Continuing to
maneuver forever on stale data is also unsafe. Bounded hold provides the
conservative middle path.

### 2.11 Explicit command validity

`tcas_command_valid` separates these concepts:

- an RA state exists;
- a direction has been selected; and
- a usable intruder altitude boundary has actually been computed.

The cache is invalidated when ownship geometry fails, the selected intruder or
RA direction changes, or the RA disappears. Firmware application gates are
separate: fixed-wing mode/throttle checks and rotorcraft NAV-mode checks prevent
application when the controller has not granted TCAS authority, without
necessarily invalidating the cached boundary.

**Why:** An enum value alone does not prove that every input needed for a flight
command was initialized and current. This prevents an advisory state from
accidentally applying stale or uninitialized altitude data.

### 2.12 Module metadata made firmware-neutral

`conf/modules/tcas.xml` no longer schedules TCAS only under
`FIXEDWING_FIRMWARE`. The same initialization, periodic tasks, and datalink
handlers are available to both supported firmware families.

**Why:** Correct C code is useless if the module generator omits it from the
rotorcraft build. Build metadata is part of the architecture, not administrative
decoration.

---

## 3. Uncovered Lurking Bugs (Legacy Defect Analysis)

Rotorcraft support acted as a stress test for assumptions that were easy to miss
in a fixed-wing-only environment. Some findings were direct compatibility gaps;
others were genuine safety defects in shared traffic and advisory behavior.

### 3.1 Fixed-wing altitude coupling masqueraded as general TCAS logic

**Defect:** Ground reference and altitude application were directly tied to
fixed-wing navigation variables.

**Why it stayed hidden:** Every original caller used the same fixed-wing MSL
conventions, so the frame dependency looked natural rather than architectural.

**How rotorcraft exposed it:** Rotorcraft NAV altitude is local to an MSL origin.
The first integration attempt immediately raises the question: “MSL or local?”
The answer could no longer remain implicit.

**Resolution:** A shared MSL command API plus explicit local/MSL conversion at
the rotorcraft guidance boundary.

### 3.2 Zero velocity could be confused with missing velocity

**Defect class:** Magnitude-based validity assumptions can reject a real
stationary observation.

**Why it stayed hidden:** Fixed-wing aircraft normally move over the ground, so
an all-zero vector is rare. Rotorcraft hover makes it routine. Strong wind also
makes zero ground velocity physically possible for fixed-wing.

**Resolution:** Validity depends on observation-presence bits and finite numeric
components, never on non-zero magnitude. A dedicated regression test requires
`(0, 0, 0)` to remain valid.

> **Safety takeaway:** “Not moving” and “not measured” are different states.
> Encoding both as zero is convenient; treating both as equivalent is not.

### 3.3 Missing or invalid surveillance could look harmless

**Defect:** A status model centered on `NO_ALARM`, `TA`, and `RA` had no explicit
way to state that surveillance could not support a conclusion.

**Why it stayed hidden:** Reliable bench telemetry and short fixed-wing tests
do not reproduce every loss burst, conversion failure, or startup state.

**Resolution:** `TCAS_UNAVAILABLE` is now distinct from `TCAS_NO_ALARM`.
Invalid ownship geometry, absent surveillance history, unusable traffic,
expired tracks, and traffic-table overflow fail closed.

### 3.4 Invalid snapshots could clear an active advisory

**Defect:** Treating failed geometry acquisition as ordinary no-conflict geometry
can erase an RA precisely when communication is lost.

**Why it stayed hidden:** Nominal tests tend to stop sending traffic cleanly or
never enter an RA before data loss. The dangerous sequence is conflict first,
loss second.

**Resolution:** Existing advisories enter bounded hold. They are neither updated
from invalid geometry nor immediately cleared as resolved.

### 3.5 Stale command data was not the same as a valid RA command

**Defect:** Advisory state and command-data validity were insufficiently
separated. A selected RA direction could outlive the intruder boundary from
which its altitude should be calculated.

**Why it stayed hidden:** In fixed-wing nominal operation, the 1 Hz state machine
and 4 Hz altitude refresh usually run with continuous traffic, so the cache
appears continuously valid.

**Resolution:** `tcas_command_valid` explicitly controls whether the cached
boundary can produce a flight command.

### 3.6 Traffic coordinate conversion could appear successful when it was not

**Defect:** Lazy coordinate getters historically returned storage even when the
required source frame or ownship origin was unavailable. A caller could consume
zeroed or stale structure contents as converted geometry.

**Why it stayed hidden:** Fixed-wing configurations usually initialize their
navigation frames early and consistently. Mixed firmware and startup timing
exercise more combinations of ENU, UTM, LLA, and local-origin readiness.

**Resolution:** Conversion helpers set validity bits only after successful
conversion. The TCAS snapshot API verifies those bits before copying data.

### 3.7 Traffic-table saturation could freeze known tracks silently

**Defect:** Capacity handling could prevent updates broadly rather than refusing
only a new arrival, leaving an apparently populated but increasingly stale
traffic picture.

**Why it stayed hidden:** Small test fleets rarely fill `NB_ACS`. A real mesh can
see churn, relayed peers, and more IDs over time.

**Resolution:** Known aircraft continue updating when the table is full; only a
new record is refused. `traffic_info_capacity_exceeded` makes the loss of
coverage visible to TCAS, which then reports unavailable surveillance.

### 3.8 ID and slot identity were easy to conflate

**Defect:** Aircraft IDs are protocol identities; `ti_acs[]` indexes are compact
storage slots. Using one as the other risks out-of-bounds access or updating the
wrong track, especially with sparse IDs.

**Why it stayed hidden:** Sequential, low-numbered test aircraft accidentally
make ID and slot values look interchangeable.

**Resolution:** ID validation and registered-slot checks confirm both bounds and
identity. The supported domain is explicit: GCS `0`, aircraft `1..254`, and
`255` reserved for broadcast/sentinel use.

### 3.9 GPS time-of-week was unsuitable as the sole freshness clock

**Defect:** GPS time wraps weekly, may jump during synchronization, and describes
source time rather than local receipt age.

**Why it stayed hidden:** Short flights rarely cross rollover and usually have
stable GPS before traffic evaluation begins.

**Resolution:** Safety freshness uses an extended local monotonic timestamp.
Source TOW remains useful for rejecting duplicate or out-of-order source data,
with rollover-aware comparison.

### 3.10 Unbounded prediction would reward old data with false precision

**Defect:** Constant-velocity extrapolation becomes less credible with time but,
without a limit, continues producing exact-looking coordinates.

**Why it stayed hidden:** Fixed-wing trajectories are often smoother over short
intervals. Rotorcraft can change horizontal and vertical motion rapidly.

**Resolution:** Prediction is capped at the advisory horizon. Beyond the fresh
window, the policy holds an existing advisory or declares surveillance
unavailable instead of evolving conflict geometry from stale data.

---

## 4. Impacted Files & Risk Assessment

The dual-firmware commit changed **nine tracked files**, with 688 insertions and
167 deletions. The table below covers both primary and secondary components.

| File / component | Adjustment made | Technical rationale: why | Risk of inaction |
|---|---|---|---|
| `sw/airborne/modules/multi/tcas.c` | Shared fixed-wing/rotorcraft core; ownship validation; checked track acquisition; fail-closed state; bounded hold; command-valid cache; firmware-specific ground reference | Keep geometry and advisory policy identical while isolating controller-specific behavior | **Critical:** rotorcraft cannot use TCAS correctly; wrong altitude frame, stale commands, or missing data could produce unsafe guidance or false no-alarm states |
| `sw/airborne/modules/multi/tcas.h` | Firmware-neutral public API; `TCAS_UNAVAILABLE`; shared resolution types and documented contracts | Give both firmware families one stable interface and represent uncertainty explicitly | **High:** rotorcraft has no safe command interface; consumers may confuse unavailable surveillance with evaluated no-conflict |
| `sw/airborne/modules/multi/tcas_policy.h` | Pure freshness, velocity, frame-conversion, and altitude-resolution helpers | Make safety boundaries deterministic, reviewable, and independently testable | **High:** policy stays tangled in periodic control flow, increasing divergence and making boundary defects difficult to detect |
| `sw/airborne/firmwares/rotorcraft/guidance/guidance_v.c` | Thin TCAS overlay in `NAV_VERTICAL_MODE_ALT`; local ↔ MSL conversion | Apply RAs through normal rotorcraft vertical guidance without bypassing mode authority | **High:** if left unchanged, rotorcraft receives no TCAS altitude command. The chosen design also avoids hazards that a less constrained integration could introduce, such as mode override or frame confusion |
| `sw/airborne/modules/multi/traffic_info.c` | Monotonic observation age; checked snapshots; source arbitration; success-only conversions; bounded mesh prediction; finite-value and ordering checks | TCAS decisions are only as safe as their traffic state and freshness metadata | **Critical:** stale, unconverted, non-finite, or frozen traffic can enter collision geometry and create missed or false advisories |
| `sw/airborne/modules/multi/traffic_info.h` | Explicit ID domain; safe slot resolution; checked snapshot declarations; traffic-capacity and surveillance state | Separate protocol IDs from compact table slots and expose safety-grade acquisition without breaking legacy getters | **High:** sparse/reserved IDs or table saturation can corrupt lookup semantics or hide surveillance loss |
| `conf/modules/tcas.xml` | Removed fixed-wing-only scheduling/build conditions; documented control authority for both firmware families | Ensure module generation actually initializes and schedules TCAS on rotorcraft | **High:** rotorcraft code may compile in isolation but never initialize, run periodic evaluation, or process peer messages |
| `tests/utils/test_tcas_policy.c` | Expanded to 29 policy tests covering freshness boundaries, rollover, zero velocity, altitude conversion, climb/descend, invalid inputs, and security floor | Cover selected pure-policy boundaries with executable regression tests | **Medium:** future cleanup can silently reintroduce zero-velocity rejection, boundary errors, or frame mistakes; track selection, cache lifetime, peer coordination, and controller integration still require broader tests |
| `tests/utils/Makefile` | Declared policy/time headers as test dependencies and enabled strict warnings | Ensure policy edits rebuild the test and compile under `-Wextra -Werror` | **Medium:** stale test binaries may report success after a header-only policy change |

### Related files intentionally not redesigned

Several existing modules consume legacy traffic getters, including formation,
follow, camera, potential-field, and navigation code. They were deliberately not
forced through the new TCAS snapshot contract.

**Why:** A safety improvement for TCAS should not trigger an uncontrolled API
migration and behavior redesign across unrelated flight algorithms. The checked
snapshot is an additive safety boundary for TCAS; legacy getters retain their
established contract.

### Runtime and protocol impact

| Concern | Result |
|---|---|
| Dynamic allocation | None introduced |
| TCAS core copies | One shared implementation |
| `MESH_STATE` wire payload | Unchanged; 17-byte payload |
| Traffic ID/table field width | Unchanged; byte-sized |
| PPRZLink requirement for mesh | Version 2 enforced at compile time |
| Fixed-wing control hook | Preserved |
| Rotorcraft control modes affected | NAV altitude mode only |

### Validation evidence

During implementation, the following checks were run in the working repository:

- fixed-wing Adam AP compilation;
- fixed-wing Easystar mesh NPS compilation and linking;
- rotorcraft Quadshot AP compilation;
- rotorcraft Quadshot mesh NPS compilation and linking;
- 29 focused TCAS policy checks;
- the complete utility suite: **2 test executables, 52 assertions**;
- TCAS and traffic module metadata validation;
- XML validation for module, fixture, and flight-plan files;
- five deterministic 4,000-frame mesh simulations using seeds
  `1`, `3`, `5`, `7`, and `11`;
- Doxygen 1.15 generation with zero diagnostics attributed to the three TCAS
  source/header files; and
- `git diff --check` plus focused code and documentation reviews.

Representative reproducible commands include:

```bash
make AIRCRAFT=Adam ap.compile
make CONF_XML=conf/userconf/OPENUAS/openuas_test_tcas_conf.xml \
  AIRCRAFT=Easystar_TCAS USER_CFLAGS=-DTRAFFIC_INFO_USE_MESH=1 nps.compile
make CONF_XML=conf/userconf/OPENUAS/openuas_all_ac_conf.xml \
  AIRCRAFT=Quadshot_W_Negative ap.compile
make CONF_XML=conf/userconf/OPENUAS/openuas_test_tcas_conf.xml \
  AIRCRAFT=Quadshot_TCAS USER_CFLAGS=-DTRAFFIC_INFO_USE_MESH=1 nps.compile
make -C tests/utils clean test
```

The mixed-fleet configuration and its two flight plans must be present for the
`Easystar_TCAS` and `Quadshot_TCAS` commands. Simulation and documentation
commands depend on their corresponding local tools. Build artifacts and logs
were not committed as proof; therefore these results describe the checks run
during this refactoring rather than a permanent CI record.

The rotorcraft builds retain one pre-existing compile-time caution from
`stabilization_indi_simple.c` regarding adaptive INDI. No TCAS, traffic-info, or
rotorcraft-guidance warning was introduced.

> **Validation boundary:** Passing builds and deterministic tests establish
> implementation consistency; they do not replace aircraft-specific HIL,
> radio-loss, latency, encounter-scenario, and flight-test qualification.

---

## 5. Value Proposition: Why This Refactoring Matters

### Reduction of identified software hazards

The largest engineering gain is not simply that a quadcopter can now climb or
descend for an RA. It is that the system has a much clearer definition of when
it is allowed to believe its own geometry. These changes reduce identified
software hazards, pending operational qualification.

- Missing data no longer means clear air.
- Zero velocity remains a valid physical observation.
- Stale tracks cannot open or clear advisories.
- Existing advisories survive bounded communication gaps.
- Invalid coordinate conversion cannot masquerade as valid ENU geometry.
- A selected RA cannot apply an uninitialized or obsolete altitude boundary.
- Security height remains enforced in the common MSL command frame.

### One policy across a heterogeneous fleet

A fixed-wing aircraft and a hovering rotorcraft now evaluate the same encounter
with the same geometry, freshness rules, tie-breaking, and vertical separation
policy. Aircraft type metadata remains useful for observability, but it does not
fork the avoidance algorithm.

That matters operationally: mixed fleets are difficult enough without each
vehicle carrying a slightly different interpretation of “stale,” “stationary,”
or “resolve upward.”

### Better modularity without abstraction theatre

The design introduces only the boundaries that carry real meaning:

- shared safety policy;
- checked traffic snapshots;
- MSL command contract; and
- thin firmware-specific application.

There is no dynamic object hierarchy, no duplicated controller, and no general
traffic API rewrite. Embedded constraints and existing fixed-wing interfaces are
preserved.

### Easier review and maintenance

The most subtle rules now live in pure helpers with boundary tests. A developer
can review the complete zero-velocity rule in one function, the entire freshness
state transition in another, and altitude resolution independently of firmware
state.

This reduces the chance that a future “small cleanup” accidentally changes one
firmware's safety behavior while leaving the other untouched.

### Future-proofing

The architecture provides a practical path for additional firmware families or
new traffic transports:

1. produce a checked ENU traffic snapshot with monotonic age;
2. use the shared TCAS core to obtain an MSL command; and
3. bind that command only at an appropriate controller authority boundary.

New transports can improve update rate or precision without rewriting advisory
policy. New aircraft classes can integrate without cloning the conflict
algorithm.

> **Bottom line:** This refactoring turns a fixed-wing-specific feature into a
> shared avionics capability while simultaneously correcting assumptions that
> could hide surveillance loss, stale commands, frame errors, and stationary
> traffic. It reduces concrete software risks and makes the next integration
> substantially less risky, while operational safety claims remain subject to
> HIL, encounter-scenario, radio-loss, and aircraft flight qualification.

---

## Reference Map

| Topic | Primary implementation |
|---|---|
| Shared conflict and advisory core | `sw/airborne/modules/multi/tcas.c` |
| Public TCAS contract | `sw/airborne/modules/multi/tcas.h` |
| Pure safety policy | `sw/airborne/modules/multi/tcas_policy.h` |
| Rotorcraft altitude binding | `sw/airborne/firmwares/rotorcraft/guidance/guidance_v.c` |
| Traffic state and freshness | `sw/airborne/modules/multi/traffic_info.c` |
| Traffic IDs and snapshot API | `sw/airborne/modules/multi/traffic_info.h` |
| Module scheduling/build metadata | `conf/modules/tcas.xml` |
| Policy regression tests | `tests/utils/test_tcas_policy.c` |
| Test build integration | `tests/utils/Makefile` |

## Change Provenance

- **Foundation:** `968474c84069ab845bd881896dbb0822bdff3c11` — “TCAS fixes and more for ream mesh”
- **Dual-firmware refactor:** `5ad5d0782cb24c40834ee55a25db251b107b590f` — “TCAS now works for both Rotorcraft and Fixedwing”
- **Tracked refactor scope:** 9 files, 688 insertions, 167 deletions
