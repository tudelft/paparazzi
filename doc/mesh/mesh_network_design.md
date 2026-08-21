# Self-Organising E52 LoRa Mesh for Paparazzi UAV

**Radio:** EByte E52-400NW22S, channel 24 (434.125 MHz), rate 0
(62.5 kbit/s), 460800 baud UART, 10 dBm EIRP, 0 dBi antennas.

**Fleet:** eight aircraft nominal, up to twelve aircraft, plus the ground
station as AC_ID 0. Aircraft use distinct AC_IDs in 1..254; ID 255 remains
reserved for broadcast and internal sentinels. No ordering or sequential
numbering is assumed.

**Priority:** fault tolerance and multi-hop coverage first, then the highest
state update rate that fits the E52 channel and five-frame transmit cache.

**Modeled and build-validated envelope:** 13 routing peers, 32 self-organised
slots, 12 s superframe, 1.5 s `MESH_STATE` scheduler ceiling, 38.3% modeled channel
utilisation. This is a software acceptance baseline, not a substitute for
thirteen-modem bench testing or flight qualification.

---

## 1. Architecture

### 1.1 Every node is a router

Every E52 modem, including the GCS, is provisioned with `AT+TYPE=0`. There is
no fixed relay aircraft and no single point of failure. A drone may land for a
battery swap, disappear, or rejoin without changing any other modem.

This distinction matters:

* **Paparazzi decides when a node originates a new frame.** `traffic_info.c`
  uses GPS-synchronised self-organising TDMA and modem-cache back-pressure.
* **The E52 decides how the frame is forwarded.** In broadcast mode
  (`AT+OPTION=3`) every routing modem that first receives a new frame forwards
  it once. Its duplicate filter prevents endless forwarding.
* **E52 RSSI-based best-router election applies to routed unicast.** The manual
  says the network layer automatically chooses the best routing nodes using
  RSSI. Broadcast mode does not expose a selective-relay API: it is a
  duplicate-suppressed flood. The implementation does not claim otherwise.

For position exchange, broadcast is retained because every aircraft must learn
about every other aircraft and because it continues working without a
coordinator or acknowledgement exchange. Directed, high-volume application
traffic should use E52 unicast when practical so the modem can use its automatic
route discovery and best-next-hop selection instead of flooding it.

The two control layers must not be confused:

```mermaid
flowchart LR
  S["Aircraft state"] --> T["traffic_info: choose an origination slot"]
  T -->|"one new PPRZLink frame"| L["Local E52"]
  L -->|"broadcast copy"| A["Routing E52 A"]
  L -->|"broadcast copy"| B["Routing E52 B"]
  A -->|"forward first copy once"| C["Routing E52 C"]
  B -. "later duplicate is discarded" .-> C
  C --> G["GCS E52, AC_ID 0"]
```

The TDMA allocator controls only the arrow from `traffic_info` to the local
E52. Forwarded copies are created inside the modems and do not require a
Paparazzi slot of their own. Their airtime is nevertheless charged to the
shared channel and is the reason one origination needs a flood-sized slot.

### 1.2 Why all-routing is necessary

The original 3 x 3 km area is direct-link friendly: its 4.24 km diagonal closes
with at least 7.3 dB spare margin above the already charged 10 dB fade margin.
The future operating areas are not:

| Area | Diagonal | Worst direct margin |
| --- | ---: | ---: |
| 3 x 3 km | 4.24 km | +7.3 dB |
| 11 x 11 km | 15.56 km | -14.7 dB |
| 17 x 3 km | 17.26 km | -16.9 dB |

These values come from `mesh_link_budget.py` using the exact two-ray model,
434.125 MHz, a 4 m GCS mast, 50-120 m aircraft altitude, 45 degree bank, 10 dB
fade margin, and the hard 10 dBm EIRP limit. The expanded areas need short
air-to-air hops through the swarm; a terminal-only aircraft at an edge can
partition the topology.

The deterministic design ranges from the same model are 12.30 km for an
air-to-air link at 100 m AGL and 9.17 km for an aircraft-to-GCS link with the
4 m mast. These are not universal radio ranges: they include the stated
antenna, bank, feedline, terrain-clearance, and fade assumptions. Mission
spacing must use the weaker link applicable to each edge and retain positive
margin; “all nodes route” does not make an out-of-range edge exist.

### 1.3 The bandwidth cost

All-router broadcast is expensive. One 30-byte `MESH_STATE` frame occupies
7.50 ms per RF transmission. The conservative capacity model charges one
transmission at every one of the 13 routing peers for each originated
broadcast:

$$13 \times 7.50\ \text{ms} = 97.5\ \text{ms of charged channel time}$$

An individual flood can use fewer transmissions when some peers are
unreachable or receive only duplicates, but capacity is not budgeted on that
optimistic case. The 13-transmission “flood tax” is a planning upper bound for
the connected all-router fleet, not a claim that every packet always produces
exactly 13 observable RF transmissions.

The E52 also has only five transmit-cache entries. On overflow it clears the
whole cache, creating a network-wide telemetry gap. The schedule is therefore
designed around the flood cost and cache behavior, not around the nominal
62.5 kbit/s PHY rate.

`mesh_phase_optimizer.py` proves the delivered profile:

| Check | Result |
| --- | ---: |
| Nodes / routing nodes | 13 / 13 |
| Flood tax | 13 transmissions |
| Channel utilisation | 38.3% (40% ceiling) |
| Flood span, mean + 3 sigma | 290.0 ms |
| Slot length | 375.0 ms (+29%) |
| Peak E52 cache depth | 1 of 5 |
| Cache overflow events | 0 |

The 40% limit here is an engineering ceiling on aggregate shared-channel
occupancy, chosen to retain contention and model-error headroom. It is not a
statement of legal transmitter duty cycle. The optimizer separately estimates
the busiest all-router radio at 3.985% transmit duty for the full telemetry
mix. Applicable national limits on frequency, bandwidth, power, and duty cycle
remain authoritative and must be checked for the flight location.

The optimizer's deliberately unscheduled control experiment reaches 17 cached
frames and overflows. Phase placement and TDMA are safety mechanisms, not
cosmetic tuning.

The 375 ms slot is much longer than one 6.86 ms radio transmission because it
contains the complete multi-hop forwarding episode, CSMA delay, and statistical
headroom:

```mermaid
flowchart LR
  O["Origination"] --> H1["Hop 1"] --> H2["Hop 2"] --> H3["Later hops"]
  H3 --> Q["Flood becomes quiet"]
  Q --> N["Next TDMA origination"]
```

The optimizer's mean-plus-three-sigma flood span is 290.0 ms. A 375 ms slot
leaves 85.0 ms, or 29%, beyond that modeled bound. This is statistical
engineering headroom, not a hard proof that an RF flood can never run longer.

---

## 2. Dynamic Topology

### 2.1 Self-organising slots

The superframe is 12 s with 32 slots of 375 ms. Slots are not derived from
AC_ID. A joining peer:

1. listens for three complete superframes;
2. learns slot occupancy from ordinary `MESH_STATE` arrival times;
3. claims a demonstrably free slot;
4. yields deterministically if a lower AC_ID is already using that slot;
5. leases secondary and primary slots so mutually deaf collisions eventually
   separate even when nobody can decode the collision.

`MESH_SLOT_FREE` is `0xFF`, not zero. AC_ID 0 is the GCS and is a legitimate
mesh slot owner, while aircraft IDs are limited to 1..254. The identity domain
therefore addresses at most 255 members (GCS plus 254 aircraft), so byte-sized
member counts remain sufficient without changing the over-air format. The
delivered 32-slot profile supports at most 32 simultaneous full-membership
peers. The GCS participates in routing and traffic visibility but is
deliberately excluded from TCAS collision avoidance; a future
stationary-obstacle policy could use its track explicitly.

Thirty-two slots are intentional. Thirteen peers could fit in sixteen, but the
churn test showed that sixteen slots were too tight when up to six aircraft
leave and rejoin. With 32 slots, eight deterministic 4000-frame simulations at
5% churn all pass; collision occupancy is 2.18-3.75%, every collision is
transient, and no node loses its primary slot.

```mermaid
stateDiagram-v2
  [*] --> Listening: power on
  Listening --> Primary: observe 3 superframes\nand claim a free slot
  Primary --> Expanded: fair share allows another slot\nand quiet-slot turn arrives
  Expanded --> Expanded: renew or rotate\nsecondary leases
  Expanded --> Primary: peer returns, cache rises,\nor state becomes low priority
  Primary --> Reselect: lower AC_ID owns the slot\nor primary lease expires
  Reselect --> Primary: choose an observed-free slot
  Primary --> [*]: node leaves
```

There is no reservation packet. Each receiver infers the sender's slot from
GPS-aligned or bounded-holdover arrival time. Listen-before-claim avoids cold-start over-allocation;
round-robin expansion prevents two nodes from grabbing the same apparently
free slot in one superframe; leases eventually break collisions that are
otherwise invisible because collided frames cannot be decoded.

When GPS time disappears, the node retains the last GPS-to-monotonic anchor for
up to 60 seconds instead of jumping to boot-relative time. The primary
reservation and absolute superframe phase therefore survive a short outage;
position-invalid policy contracts opportunistic slots. The bound
assumes at most 100 ppm error per node: two worst-case clocks separate by about
12 ms in one minute, leaving margin inside the 375 ms slot after the modeled
290.0 ms flood span and 50 ms scheduler sampling interval.

One superframe before that bound, the denied node and peers which hear its
`HOLDOVER` advertisements clear slot authority and enter `MESH_CLOCK_ASYNC`.
This fleet-wide downgrade prevents synchronized and asynchronous MACs from
competing indefinitely during partial GPS loss. Each peer then originates one
frame at an independently randomized 16-24 second interval; asynchronous frames
remain valid traffic but never reserve a TDMA slot. GPS-ready fallback peers
advertise `RECOVERY` without extending the denied-peer lease. `RECOVERY` carries
a deterministic absolute GPS-frame target: every peer selects the same next
eight-frame epoch at least seven frames ahead and adopts any later target it
receives. This lets rebooted and late-joining peers converge before TDMA resumes.
At that target, two seconds of continuous valid GPS and the normal
listen-before-claim cycle are required before transmission. Coherent recovery
during early holdover preserves slots, while a correction over 10 ms clears
them.

### 2.2 State-aware origination

All radios route continuously at the E52 layer. Paparazzi adapts only the rate
at which a node originates its own state:

* a healthy airborne node with GPS or bounded holdover and valid position
  receives its fair share of available slots;
* a landed or position-invalid node keeps one heartbeat slot;
* a node without bounded network time uses low-rate randomized access instead
  of pretending that boot-relative slots are synchronized;
* HOME, emergency, failsafe, and low-battery states request one extra slot;
* when the estimated modem cache reaches the high-water mark, the node gives
  up one opportunistic slot before the E52 can overflow;
* when aircraft leave, healthy peers gradually claim quiet slots; when they
  return, peers contract immediately.

Of the 32 physical slots, 28 are distributed as steady-state fair share and
four remain as headroom for joins, partitions merging, and temporarily
different membership views. For $S=28$ fair-share slots and $N$ live nodes,
each healthy node receives a baseline of
$q=\lfloor S/N\rfloor$ slots. The first $r=S\bmod N$ sorted AC_ID ranks receive
one remainder slot, so quotas differ by at most one and sum to all available
slots. The winner window advances by one rank every 81 superframes, about
16.2 minutes, giving long-term fairness without fleet-wide claim churn.

At maximum population, two of the 13 peers are entitled to three slots and
eleven to two slots per 12 s. The average entitlement is therefore
$28/(13\times12)=0.179$ Hz instead of the floor-only 0.167 Hz. At the nominal
nine-peer population, one peer receives four slots and eight receive three,
for an average entitlement of 0.259 Hz. Collision-healing leases and policy contraction
can make observed rates lower than these steady-state entitlements. Sparse
fleets may now use up to `MESH_TDMA_MAX_REUSE=8` slots. The cap does not add
slots or shorten them: it lets fewer live peers use slots that would otherwise
remain empty. Measured steady-state simulator rates changed as follows; counts
include the GCS because it is a real mesh peer:

| Live peers | Reuse 4 | Reuse 8 | Improvement |
| ---: | ---: | ---: | ---: |
| 1 | 0.31 Hz | 0.56 Hz | 81% |
| 2 | 0.29 Hz | 0.48 Hz | 66% |
| 3 | 0.27 Hz | 0.35 Hz | 30% |
| 4 | 0.27 Hz | 0.28 Hz | 4% |
| 9 | 0.17 Hz | 0.17 Hz | unchanged |
| 13 | 0.15 Hz | 0.14 Hz | 0.01 Hz reserved for churn |

Collision-healing leases deliberately limit dense-fleet convergence, so the
measured rates are lower than the mathematical entitlement. This revision
does not weaken those leases merely to report a higher number. Emergency state
still receives priority within the same bounded channel budget.

The 1.5 s telemetry period is an enabling ceiling, not a promise that every
node transmits every 1.5 s. Actual emission remains controlled by live slot
ownership, position validity, synchronization state, and cache pressure.

### 2.3 Time and lease constants

| Parameter | Value | Reason |
| --- | ---: | --- |
| `MESH_TDMA_SUPERFRAME_MS` | 12000 | 32 flood-safe slots |
| `MESH_TDMA_NB_SLOTS` | 32 | 13 peers plus churn headroom |
| `MESH_TDMA_FAIR_SLOTS` | 28 | steady quota; four slots absorb membership disagreement |
| `MESH_TDMA_MAX_REUSE` | 8 | bounded sparse-fleet acceleration |
| `MESH_ENTRY_FRAMES` | 3 | listen before transmitting |
| Secondary lease | 6-13 frames | break secondary collisions |
| Primary lease | 10-19 frames | preserve a 2-4 minute recovery window with a 12 s frame |
| Slot age | 4 frames | tolerate loss, reclaim departed nodes |
| Remainder epoch | 81 frames | rotate one rank after ageing and serialized expansion settle |
| Holdover | 60 s | preserve GPS epoch while worst-case relative drift stays bounded |
| GPS acquisition | 2 s | reject fix flapping before entering TDMA |
| Asynchronous interval | 16-24 s | bounded degraded-mode load below the track-drop horizon |
| Recovery epoch | 8 frames | deterministic common target beyond all fallback leases |
| Cache high water | 3 of 5 | leave two E52 cache entries as hard margin |

The static assert in `traffic_info.c` forces the generated `MESH_STATE` period
to equal `superframe / max reuse` (12 / 8 = 1.5 s) in either supported telemetry
mode layout. A mismatched telemetry file therefore fails the aircraft build
instead of failing in flight.

The cap and telemetry period are a fleet-wide protocol profile. Update every
participating aircraft before using the faster profile; do not mix reuse-four
and reuse-eight firmware in one mesh.

### 2.4 Automatic one-aircraft telemetry mode

The common one-aircraft test case uses an ordinary `mesh_solo` telemetry mode.
There is no custom wire protocol and no additional ground application. Both E52
modules stay in the normal routing/broadcast profile; the feature never rewrites
`AT+TYPE`, `AT+OPTION`, destination addresses, air rate, power, channel, or UART
settings in flight.

The existing OCaml link already sends a targeted `PING` to every live aircraft
at five-second intervals. Because E52 broadcast traffic is heard by every mesh
member, each aircraft can answer two questions locally:

* has the GCS recently pinged me;
* has the GCS recently pinged another aircraft.

The airborne selector chooses `mesh_solo` only when the mesh clock is safe, the
GCS has recently pinged this aircraft, no recent PING targeted another aircraft,
no peer owns a live mesh slot, and no `MESH_STATE` peer frame has been received
for 12 seconds. Any failed condition returns immediately to `mesh`. A manual
selection of any other telemetry mode is preserved and disables automatic
switching until `mesh` or `mesh_solo` is selected again.

The ground link was tightened to PING only aircraft that are still live. Without
that small correction, an aircraft which landed hours earlier would remain in
the link table and suppress solo mode forever.

Only existing messages are used. Fixed-wing sends `MINIMAL_COM` at 4 Hz,
`ATTITUDE` at 2 Hz, `ENERGY` at 1 Hz, `DATALINK_REPORT` at 0.5 Hz, and `ALIVE`
at 0.2 Hz. Rotorcraft uses the same schedule with native `ROTORCRAFT_FP` instead
of `MINIMAL_COM`. `MESH_STATE` remains active as the safety/discovery canary in
both modes. One standard `ALIVE` is sent when the mesh transport first becomes
ready and retried every 30 seconds until the first GCS PING, allowing the normal
server/link discovery cycle to start without a custom handshake.

The two-router conservative budget for the fixed-wing standard-message profile
is about 15.9% aggregate channel occupancy and 8.0% transmit duty per modem.
Live NPS measurements were 4.16 Hz for fixed-wing `MINIMAL_COM` and 3.98 Hz for
rotorcraft `ROTORCRAFT_FP`, roughly seven times the measured 0.56 Hz sparse mesh
rate. A simultaneous fixed-wing/rotorcraft test observed both aircraft in the
ordinary link table and zero solo-rate frames over the measured six-second
window.

The design deliberately accepts less throughput than a custom compact packet.
In return it uses standard Paparazzi messages, standard generated telemetry
modes, the existing PING/PONG path, one small firmware-neutral selector, and no
new process for the operator to start.

---

## 3. E52-400NW22S Configuration

The delivered profile is tailored to the E52 manual:

```text
AT+PANID=250,1
AT+SRC_ADDR=1000+AC_ID,1
AT+TYPE=0                 all nodes route, including AC_ID 0
AT+RATE=0                 62.5 kbit/s only
AT+CHANNEL=24,1           434.125 MHz
AT+POWER=10,1             10 dBm conducted with 0 dBi antenna
AT+OPTION=3,1             broadcast
AT+DST_ADDR=65535,1
AT+ROUTER_SAVE=0          do not persist a moving topology
AT+ROUTER_CLR=1
AT+ROUTER_SCORE=3
AT+HEAD=0                 PPRZLink already carries framing and sender ID
AT+BACK=0                 do not inject modem status text into PPRZLink
AT+CSMA_RNG=20            manual minimum; TDMA separates originations
AT+RESET_TIME=0           no periodic RF reset in flight
AT+FILTER_TIME=3000       duplicate suppression
AT+UART=460800,8N1
AT+RESET
```

`AT+TYPE` must follow `AT+SRC_ADDR` because it rewrites the address type bit.
The provisioning tool enforces this ordering, checks every response, reopens
the serial port after changing baud, and reads the configuration back.

Program an aircraft or the GCS:

```bash
python3 sw/tools/mesh/e52_provision.py \
  --port /dev/ttyUSB0 --ac-id 129 --factory-reset

python3 sw/tools/mesh/e52_provision.py \
  --port /dev/ttyUSB0 --ac-id 0 --factory-reset
```

Both commands default to `AT+TYPE=0`. A constrained terminal/router experiment
is still possible with `--relay-ids 125`, but it is not the delivered flight
architecture.

The power rule is absolute: **10 dBm EIRP maximum**. The NW22S PA can produce
22 dBm, but that setting must never be used here. `e52_provision.py` derives
conducted power from the EIRP ceiling minus antenna gain and refuses an illegal
combination.

---

## 4. RF and Simulation Tools

All tools are in `sw/tools/mesh/`. Run them from the repository root. They use
Python 3 and return nonzero when an acceptance gate fails.

### 4.1 `mesh_link_budget.py`: does one hop close?

This is the deterministic RF calculator. It uses free-space loss where valid,
two-ray propagation with a complex ground reflection coefficient, surface
roughness, bank-angle polarisation loss, fade margin, Fresnel clearance, and
radio horizon.

```bash
# Original area
python3 sw/tools/mesh/mesh_link_budget.py \
  --area-side 3000 --gcs-antenna-height 4

# Future square
python3 sw/tools/mesh/mesh_link_budget.py \
  --area-width 11000 --area-height 11000 --gcs-antenna-height 4

# Future strip
python3 sw/tools/mesh/mesh_link_budget.py \
  --area-width 17000 --area-height 3000 --gcs-antenna-height 4
```

Read **worst case margin** first. Positive margin means a direct corner-to-corner
link closes after the modeled fade reserve; negative margin means multi-hop
placement is required. A negative corner margin does not mean the mesh fails:
it means no mission plan may leave a gap between neighboring routing aircraft
larger than the applicable one-hop design range. For this profile that is at
most 12.30 km air-to-air and 9.17 km to the 4 m GCS mast, before adding any
mission-specific margin.

### 4.2 `mesh_phase_optimizer.py`: does the traffic fit?

This tool parses the real PPRZLink XML, computes each message's LoRa airtime,
multiplies it by the E52 flood tax, solves Paparazzi phase offsets, replays the
generated tick condition, simulates the five-frame modem cache, and writes the
telemetry XML.

The exact delivered 13-router gate is:

```bash
python3 sw/tools/mesh/mesh_phase_optimizer.py \
  --ac-ids 0,3,19,42,58,77,101,125,140,168,203,222,251 \
  --relay-nodes 13 --nb-slots 32 \
  --mesh-period 1.5 --superframe 12 --max-reuse 8 \
  --period-scale 4 \
  --emit-xml conf/telemetry/OPENUAS/openuas_mesh_swarm.xml
```

The AC_ID list is deliberately irregular. Its length drives population; the
numeric values do not drive slot assignment. Do not hand-edit generated phase
values. Change the model inputs and rerun the tool.

### 4.3 `mesh_slot_sim.py`: do joins and leaves converge?

This is a line-by-line Python mirror of the airborne slot allocator. It includes
AC_ID 0, arbitrary IDs, 12 aircraft, dynamic state faults, emergency priority,
cache pressure, and random land/rejoin churn.

```bash
for seed in 1 2 3 4 5 6 7 8; do
  python3 sw/tools/mesh/mesh_slot_sim.py \
    --frames 4000 --churn 0.05 --seed "$seed" --max-reuse 8 || exit 1
done
```

Acceptance means every node retains a slot, nobody exceeds the reuse cap,
collision occupancy stays below 4%, every collision is bounded, and no
deadlock survives a lease.

### 4.4 `mesh_link_sim.py`: does the full moving mesh deliver?

This packet-level Monte Carlo simulator imports the same RF model, moves the
aircraft between waypoints, applies bank loss and correlated log-normal
shadowing, and propagates each frame through an arbitrary number of routing
hops. The reached set models the E52 duplicate filter.

```bash
# Nominal fleet and area
python3 sw/tools/mesh/mesh_link_sim.py \
  --ac-ids 0,3,19,42,77,101,125,168,251 \
  --width 3000 --height 3000 --duration 600 --seed 1

# Maximum fleet, future square
python3 sw/tools/mesh/mesh_link_sim.py \
  --ac-ids 0,3,19,42,58,77,101,125,140,168,203,222,251 \
  --width 11000 --height 11000 --duration 1200 --seed 7

# Maximum fleet, future strip
python3 sw/tools/mesh/mesh_link_sim.py \
  --ac-ids 0,3,19,42,58,77,101,125,140,168,203,222,251 \
  --width 17000 --height 3000 --duration 1200 --seed 7
```

The report shows direct and multi-hop PDR per aircraft, worst instantaneous
margin, and busiest-radio duty. In one 17 x 3 km baseline seed, direct fleet
PDR was 0.951 while modeled all-router multi-hop PDR was 1.000. Across the
sixteen recorded expanded-area regression runs, all modeled relayed PDRs were
1.000; this is evidence that multi-hop helps under those seeds and assumptions,
not a guarantee for arbitrary geometry or terrain.

Simulation is not a proof that every random placement is connected. Before an
expanded-area flight, run many seeds and enforce mission geometry that keeps a
chain of aircraft inside one-hop range.

### 4.5 `e52_provision.py`: is hardware identical?

Use `--dry-run` to inspect a profile without hardware, `--verify-only` to read
back a modem, and `--factory-reset` before programming a known profile.

```bash
python3 sw/tools/mesh/e52_provision.py --ac-id 0 --dry-run
python3 sw/tools/mesh/e52_provision.py --ac-id 251 --dry-run
python3 sw/tools/mesh/e52_provision.py \
  --port /dev/ttyUSB0 --ac-id 251 --verify-only
```

Check that every modem reports a unique `SRC_ADDR=1000+AC_ID`, `TYPE=0`, rate
0, channel 24, 10 dBm conducted power for a 0 dBi antenna, broadcast option 3,
and 460800 8N1.

### 4.6 What each tool proves

No single tool validates the network. Each removes a different failure mode:

```mermaid
flowchart LR
  B["Link budget\nmean one-hop margin"] --> M["Link simulation\nmobility and fading"]
  P["Phase optimizer\nairtime and cache"] --> M
  S["Slot simulation\njoin, leave, collision recovery"] --> M
  M --> H["13-modem bench\nreal firmware and RF"]
  E["Provisioning readback\nidentical legal settings"] --> H
  H --> F["Controlled flight qualification"]
```

Passing a box means only that box's assumptions held. In particular, the link
simulator assumes collision-free originations after `mesh_slot_sim.py` has
validated the allocator; it does not reproduce E52 firmware timing, adjacent
channel interference, terrain blockage, UART faults, or antenna installation.

---

## 5. Code Design

### 5.1 `MESH_STATE`

The compact datalink message is 22 payload bytes and 30 PPRZLink wire bytes:

* flags: unified flight mode, rotorcraft, valid position, airborne, alert,
  emergency;
* clock mode: GPS, bounded holdover, or asynchronous fallback;
* latitude and longitude at 1e-7 degree;
* ellipsoid altitude in centimeters;
* packed course, ground speed, and climb rate.
* absolute recovery frame, zero outside `RECOVERY`.

The PPRZLink sender header is the source of truth for AC_ID. The payload does
not duplicate it, so a relayed frame cannot claim a different aircraft without
rewriting and rechecksumming the whole PPRZLink frame.

Message edits belong in `conf/messages_mesh_new.xml`. Do not edit
`sw/ext/pprzlink/message_definitions/v1.0/messages.xml`; it is external source,
and the active `conf/messages.xml` resolves to the mesh definition.

### 5.2 Existing Paparazzi state reused

No parallel estimator or flight-mode model was invented. `traffic_info` uses:

* current `gps.fix` and `gps_tow_from_sys_ticks()` for GPS time; configured
  fix-grace time is deliberately excluded from clock authority;
* an anchored monotonic clock for bounded holdover;
* `state.pos_status` and `stateGetPositionLla_i()` for position validity/state;
* `stateGetHorizontalSpeedDir_f()`, `stateGetHorizontalSpeedNorm_f()`, and
  `stateGetSpeedEnu_f()` for motion;
* `autopilot_in_flight()`, `autopilot_get_mode()`, and
  `autopilot_throttle_killed()` for flight state;
* `electrical.vsupply` and `LOW_BAT_LEVEL` where available;
* Paparazzi `Min`/`Max`, geodetic conversion, and periodic telemetry APIs.

All mesh storage is static. There is no heap allocation, recursion, or
variable-length array in the new path.

Clock validity and kinematic validity are separate. Losing GPS time does not
immediately end TDMA because holdover preserves the epoch, but the current
position contract still requires a valid 3D GPS fix. A future vision, UWB, or
SLAM source may set position valid only after it provides a globally shared
frame and bounded uncertainty; local coordinates must never be encoded as LLA.

### 5.3 Hardware back-pressure

`mesh_local_tx_in_flight()` is a leaky-bucket estimate of locally submitted
frames which may not yet have drained. `traffic_info_mesh_periodic()` refuses a
new origination at depth three and increments `throttled_count`; state-aware
reuse also contracts by one. E52-internal relay frames are not observable on
the UART API, so this is local admission control, not a physical cache-depth
measurement. Worst-case relay load remains an optimizer and HIL acceptance
constraint.

### 5.4 Predicted traffic snapshots and TCAS freshness

Each valid `MESH_STATE` stores its immutable position, velocity, local monotonic
receive time, and mesh provenance in `traffic_info`. Safety consumers can ask
for a caller-owned ENU snapshot projected with constant velocity:

$$\hat{p}(t)=p_{observation}+v_{observation}\min(\Delta t,T_{prediction})$$

Repeated calls always start from the original observation, so prediction never
accumulates drift in the traffic table. Invalid-position heartbeats remain
visible to TDMA membership but invalidate old kinematics immediately. Legacy
traffic records keep their existing behavior and are never mesh-predicted.

TCAS bounds prediction and fresh decision-making to `TCAS_TAU_TA`, currently
4 s on the proposed Talon. After that interval an existing TA or RA and its
altitude target are held, but no stale state opens, closes, or changes an
advisory. After two complete superframes plus one slot and one 1 Hz task
interval, currently 25.375 s, the track becomes `TCAS_UNAVAILABLE`; data loss
is never reported as geometric resolution or `TCAS_NO_ALARM`. Clock arithmetic uses local
monotonic age, while TDMA epochs use GPS week plus TOW. A synchronization
change or a GPS-to-monotonic phase correction over 10 ms clears learned slot
ownership and starts a new listen-before-claim cycle, including corrections
that remain inside the same superframe or land in exactly the next one.

This policy bridges short update gaps without pretending that constant-velocity
prediction is certain during maneuvers. It is not TCAS flight qualification.
Measured packet latency, maneuver envelopes, loss bursts, and hardware-in-the-
loop conflict scenarios remain required before operational use.

---

## 6. Acceptance Procedure

Run these before flight after any message, telemetry, timing, routing, or radio
change:

```bash
# 1. Channel, flood span, phases, and cache
python3 sw/tools/mesh/mesh_phase_optimizer.py \
  --ac-ids 0,3,19,42,58,77,101,125,140,168,203,222,251 \
  --relay-nodes 13 --nb-slots 32 \
  --mesh-period 1.5 --superframe 12 --max-reuse 8 --period-scale 4

# 2. Dynamic topology, fair quotas, clock steps, and state-aware contraction
for seed in 1 2 3 4 5 6 7 8; do
  python3 sw/tools/mesh/mesh_slot_sim.py \
    --frames 4000 --churn 0.05 --seed "$seed" --max-reuse 8 || exit 1
done

# 2a. Automatic mesh/mesh_solo mode policy
tests/utils/test_mesh_mode_policy.run

# 2b. Independent clocks, bounded holdover, and randomized fallback
python3 sw/tools/mesh/mesh_gps_denied_sim.py \
  --seeds 100 --duration 3600 --stress-ppm 100 --denied-nodes 13
python3 sw/tools/mesh/mesh_gps_denied_sim.py \
  --seeds 100 --duration 3600 --stress-ppm 100 --denied-nodes 6

# 3. RF geometry and moving multi-hop delivery
python3 sw/tools/mesh/mesh_link_budget.py \
  --area-width 17000 --area-height 3000 --gcs-antenna-height 4
python3 sw/tools/mesh/mesh_link_sim.py \
  --ac-ids 0,3,19,42,58,77,101,125,140,168,203,222,251 \
  --width 17000 --height 3000 --duration 1200 --seed 7

# 4. Embedded build
make CONF_XML=conf/userconf/OPENUAS/openuas_swarm_conf.xml \
  AIRCRAFT=Haydn ap.compile
make CONF_XML=conf/userconf/OPENUAS/openuas_swarm_conf.xml \
  AIRCRAFT=Adam ap.compile

# 5. Modem profiles
python3 sw/tools/mesh/e52_provision.py --ac-id 0 --dry-run
python3 sw/tools/mesh/e52_provision.py --ac-id 251 --dry-run
```

The slot simulator also checks 32-bit frame rollover, GPS-week rollover,
same-frame and next-frame phase corrections, and GPS synchronization loss and
reacquisition before each churn run.

Bench acceptance must also verify all thirteen modems together. Start them in
different orders, remove up to six aircraft modems, restore them, and confirm:

* all present AC_IDs continue updating;
* the GCS (AC_ID 0) can transmit and route;
* `throttled_count` remains near zero in steady state;
* no modem returns `OUT OF CACHE`;
* removing any one routing node does not partition the remaining connected
  topology;
* modem readback matches the provisioning profile.

Record packet timestamps, source AC_ID, duplicate count where observable,
end-to-end latency, modem errors, and `throttled_count`. A pass/fail result
without these traces cannot distinguish RF loss, a slot collision, UART loss,
cache flushing, or a topology partition.

---

## 7. Known Limits

1. **Broadcast has no selective relay election.** All `TYPE=0` nodes forward a
   new broadcast once. Automatic RSSI route selection is available for E52
   unicast, not for this broadcast state stream.
2. **Connectivity still depends on geometry.** All-router capability cannot
   bridge an empty 17 km gap. The mission planner must keep a connected chain
   of aircraft with adequate one-hop margin.
3. **Dense-fleet update rate is deliberately lower.** Maximum-population
  average `MESH_STATE` entitlement is about 0.205 Hz per node; nominal
  population reaches about 0.296 Hz before lease and policy contraction.
  This is the cost of 13-way broadcast redundancy
   at 10 dBm and 62.5 kbit/s.
4. **The GCS command tail is unslotted.** Commands and normal PINGs remain rare
  CSMA traffic. Solo mode reduces downlink contention but does not turn the
  command path into a sustained bulk uplink.
5. **The RF model is terrain-agnostic.** The 10 dB fade reserve covers generic
   shadowing, not a ridge, building, or forest wall. Survey the real site.
6. **All nodes must share protocol constants and message layout.** The clock
  clock and recovery fields change `MESH_STATE` to 30 bytes on wire. Mixed old/new firmware
  can decode shifted fields incorrectly, so rebuild and deploy the whole fleet
  atomically. A mixed 16-slot/32-slot fleet will also collide.
7. **Asynchronous fallback is degraded operation.** It preserves low-rate
  discovery without false TDMA authority; it does not guarantee collision-free
  delivery. The 100-seed software gate must be followed by thirteen-modem HIL.

The design preference is explicit: when speed and redundancy conflict, keep
the all-routing topology and reduce originated telemetry first. The optimizer
is the authority for that trade, and it must exit zero before flight.

---

## 8. End-User Operation

No modem reprovisioning, additional application, or manual telemetry-mode change
is needed. Build and flash aircraft with a telemetry profile containing both
`mesh` and `mesh_solo`, then start the normal link/server/GCS session. The E52
link must use 460800 baud, as provisioned.

The aircraft starts in `mesh`. Once the standard link has discovered it and
PINGs show that no other aircraft is live, it switches to `mesh_solo`
automatically after the 12-second quiet interval. Starting another aircraft or
losing the GCS/clock immediately restores `mesh`. Stopping an aircraft removes
it from the link's PING set after the normal live-aircraft timeout, allowing the
remaining aircraft to return to `mesh_solo` automatically.

The supplied fixed-wing profiles use the generated `Ap` process. The dedicated
`openuas_mesh_rotorcraft.xml` profile uses `Main` and native `ROTORCRAFT_FP`, so
the selector itself is firmware-neutral. Configurations without both mode names
continue using their existing telemetry unchanged.

With `digital_cam_uart` disabled in the Talon airframe, clean `ap` and `nps`
targets build successfully. Validation includes fixed-wing and rotorcraft builds,
pure mode-policy tests, ordinary mesh churn/GPS-loss simulations, single-aircraft
rate measurements, and a real two-aircraft fallback run through the unchanged
OCaml UDP link.

---

## 9. Future Improvements

These are the three highest-value next steps. They are deliberately not
described as current capabilities.

### 9.1 Hardware-in-the-loop qualification

The software models now agree, but the largest remaining uncertainty is the
E52 firmware itself: forwarding jitter, duplicate-filter behavior, cache
flushes, UART buffering, CSMA interaction, and recovery after a modem resets.
Build a repeatable thirteen-modem test fixture before increasing area or
traffic rate.

The fixture should use conducted RF paths, attenuators or a channel emulator
where possible, rather than thirteen nearby antennas at full signal. Exercise:

* simultaneous and staggered power-up;
* one through six node removals and rejoins;
* asymmetric links and a forced three- or four-hop chain;
* fading near receiver sensitivity;
* emergency-state bursts, GCS commands, and sustained maximum telemetry;
* UART interruption, modem reset, and duplicate-filter expiry.

Acceptance should bound per-node PDR, 95th and 99th percentile latency, longest
outage, convergence time after churn, cache high-water events, and recovery
without manual reprovisioning. Use the measured forwarding-delay distribution
to rerun `mesh_phase_optimizer.py`; replace the current statistical slot
assumption only after the hardware data supports it.

### 9.2 Hybrid broadcast and routed unicast

Keep compact safety state on broadcast because every peer needs it and it must
survive without a coordinator. Move traffic with one intended recipient, such
as commands, parameter exchange, logs, or bulk telemetry, toward E52 routed
unicast so it can use RSSI-based route selection without paying the 13-way
broadcast flood tax.

```mermaid
flowchart TB
  X{"Who needs this frame?"}
  X -->|"all peers; safety state"| B["Broadcast flood\nrobust, high airtime cost"]
  X -->|"one peer; command or bulk data"| U["Routed unicast\nselected route, lower flood cost"]
  B --> C["Budget with worst-case flood tax"]
  U --> R["Require route discovery, retry,\nand failover measurements"]
```

This is an architectural experiment, not a provisioning-only change. Determine
first whether the E52 can switch destination and option safely at runtime at
the required rate. If mode changes disrupt routing state or duplicate filters,
use time-separated traffic classes or a second radio rather than rapidly
rewriting modem configuration. Add sequence numbers, delivery policy, and
bounded retries at the application layer; never allow bulk unicast to starve
`MESH_STATE` or emergency traffic. Re-run airtime, cache, churn, and hardware
tests for the mixed profile.

### 9.3 Topology-aware connectivity protection

All-router flooding provides a forwarding opportunity, but connectivity is a
property of the instantaneous RF graph. In a long strip, one aircraft can be
an articulation node: removing it divides the graph even though every remaining
radio is configured as a router.

```mermaid
flowchart LR
  subgraph Vulnerable["One critical bridge"]
    G1["GCS"] --- A1["A"] --- B1["B: articulation"] --- C1["C"] --- D1["edge"]
  end
  subgraph Protected["Two independent paths"]
    G2["GCS"] --- A2["A"]
    A2 --- B2["B"] --- D2["edge"]
    A2 --- C2["C"] --- D2
  end
```

A future topology service should estimate link quality, construct the live
connectivity graph, identify articulation nodes and weak edges, and expose a
margin-to-partition warning to the GCS and mission planner. Position-derived
range is a useful first estimate, but measured RSSI/PDR history is preferable
because antenna shadowing and interference are not visible in geometry alone.

The response must match what the current radio can actually control. In the
broadcast design Paparazzi cannot elect which E52 forwards a packet. It can:

* warn or constrain mission geometry before a critical bridge disappears;
* protect the state-update opportunity of aircraft that maintain connectivity;
* request repositioning or loiter points that create a second path;
* feed route quality into the future unicast traffic class.

The target is at least two node-disjoint GCS paths for safety-critical mission
regions where geometry permits. When that is impossible, the system should
report the single-node failure explicitly instead of treating “all nodes route”
as proof of redundancy.
