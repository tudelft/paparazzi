/*
 * Copyright (C) Pascal Brisset, Antoine Drouin (2008), Kirk Scheper (2016)
 *
 * This file is part of paparazzi
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file modules/multi/traffic_info.c
 * @brief Maintains traffic-aircraft state and optional MESH_STATE exchange.
 * @author Pascal Brisset
 * @author Antoine Drouin
 * @author Kirk Scheper
 *
 * The legacy path parses and stores standard traffic messages. When
 * TRAFFIC_INFO_USE_MESH is enabled, the same table also receives compact
 * MESH_STATE frames and originates local state through self-organising TDMA.
 */

#include "modules/multi/traffic_info.h"

#include "generated/airframe.h"     // AC_ID
#include "generated/flight_plan.h"  // NAV_MSL0

#include "modules/datalink/datalink.h"
#include "modules/datalink/telemetry.h"
#include "pprzlink/dl_protocol.h"   // datalink messages
#include "pprzlink/messages.h"     // telemetry messages

#include "state.h"
#include "math/pprz_geodetic_utm.h"
#include "math/pprz_geodetic_wgs84.h"

#if TRAFFIC_INFO_USE_MESH
#include "autopilot.h"
#endif

#if TRAFFIC_INFO_USE_MESH && defined(LOW_BAT_LEVEL)
#include "modules/energy/electrical.h"
#endif

#if TRAFFIC_INFO_USE_LOG
#include "modules/loggers/logger_utils.h"
#if !USE_CHIBIOS_RTOS
static FILE* pprzLogFile = NULL;

/* Set the default log path to bebop storage */
#ifndef TRAFFIC_INFO_FILE_PATH
#define TRAFFIC_INFO_FILE_PATH /data/ftp/internal_000/acinfo
#endif

#endif // !USE_CHIBIOS_RTOS
#endif // TRAFFIC_INFO_USE_LOG

/* Number of occupied entries in ti_acs, including GCS and this aircraft. */
uint8_t ti_acs_idx;
/* AC_ID-to-ti_acs index map; zero means unknown except for GCS AC_ID 0. */
uint8_t ti_acs_id[NB_ACS_ID];
/* Fixed-capacity state table shared by legacy and optional mesh messages. */
struct acInfo ti_acs[NB_ACS];

/* Geoid height (msl) over ellipsoid [mm] */
int32_t geoid_height;

/* Send ACINFO_LLA message from the datalink class */
static void send_acinfo_lla(struct transport_tx *trans, struct link_device *dev)
{
  int16_t course = (int16_t)DeciDegOfRad(stateGetHorizontalSpeedDir_f());
  struct LlaCoor_i* lla = stateGetPositionLla_i();
  uint32_t itow = gps.tow;
  uint16_t speed = (uint16_t)(stateGetHorizontalSpeedNorm_f()*10.f);
  int16_t climb = (int16_t)(stateGetSpeedEnu_f()->z*10.f);
  int32_t alt = (int32_t)(lla->alt/10);
  uint8_t ac_id = AC_ID;

  // broadcast GPS message
  struct pprzlink_msg msg;
  msg.trans = trans;
  msg.dev = dev;
  msg.sender_id = AC_ID;
  msg.receiver_id = PPRZLINK_MSG_BROADCAST;
  msg.component_id = 0;
  pprzlink_msg_send_ACINFO_LLA(&msg,
      &course,
      &lla->lat, &lla->lon, &alt,
      &itow, &speed, &climb, &ac_id);

#if TRAFFIC_INFO_USE_LOG
  struct LlaCoor_f* lla_f = stateGetPositionLla_f();
  struct EnuCoor_f* enu_f = stateGetPositionEnu_f();
  if (LogFileIsOpen(pprzLogFile)) {
    LogWrite(pprzLogFile, "S,%d,%d,%.7f,%.7f,%.3f,%.3f,%.3f,%.3f,%d,0\n", // 0 at the end to have same length than receive log lines
        msg.sender_id, msg.receiver_id,
        DegOfRad(lla_f->lat), DegOfRad(lla_f->lon), lla_f->alt,
        enu_f->x, enu_f->y, enu_f->z,
        itow);
  }
#endif
}

#if TRAFFIC_INFO_USE_MESH
/* ------------------------------------------------------------------------- *
 * MESH_STATE: compact broadcast state vector for narrowband LoRa MESH links
 *
 * Everything below is statically allocated and O(1). The only mutable state is
 * ::mesh_link plus two pointers to the transport and device objects owned by
 * the telemetry subsystem, captured on the first telemetry callback.
 * ------------------------------------------------------------------------- */

struct MeshLinkState mesh_link;

/* The MESH_STATE period in the telemetry file is what the bandwidth budget was
 * computed from; the superframe and reuse cap are what the node will actually
 * do. A node emits once per owned slot, so its ceiling is MESH_TDMA_MAX_REUSE
 * frames per superframe. If the declared period is longer than that interval
 * the budget understates the real channel load - the mesh would be provisioned
 * for less traffic than it generates, which is the failure mode that shows up
 * as OUT OF CACHE in flight rather than as anything visible on a bench.
 *
 * gen_periodic emits PERIOD_<MSG>_<process>_<mode index> for every scheduled
 * message, so the two files can be forced to agree at build time. Compared in
 * whole milliseconds because a cast of a parenthesised floating constant is an
 * integer constant expression while a floating multiply is not.
 */
#if defined(PERIOD_MESH_STATE_Ap_0)
/* The gate can only send what the telemetry scheduler offers it. To use up to
 * MESH_TDMA_MAX_REUSE slots per superframe the request has to arrive at least
 * that often, so the MESH_STATE period must be superframe / MAX_REUSE.
 * Compared in milliseconds; a cast of a parenthesised floating constant is an
 * integer constant expression, a floating multiply is a GCC extension that all
 * supported toolchains accept. */
_Static_assert((MESH_TDMA_SUPERFRAME_MS / MESH_TDMA_MAX_REUSE) ==
                 (unsigned)(PERIOD_MESH_STATE_Ap_0 * 1000.0 + 0.5),
               "The MESH_STATE telemetry period must equal "
               "MESH_TDMA_SUPERFRAME_MS / MESH_TDMA_MAX_REUSE, otherwise the "
               "bandwidth budget is computed for a different rate than the "
               "node will actually emit.");
#endif

#if defined(TRAFFIC_INFO_MESH_PERIODIC_FREQ)
/* The transmit gate samples the clock; it is not interrupt driven. A slot is
 * only ever used if at least one run of the periodic task falls inside it, so
 * the task period must be strictly shorter than one slot. The delivered 20 Hz
 * task and 12 s / 32-slot profile give 50 ms sampling inside a 375 ms slot.
 * Cross-multiplied to stay in integers if either profile value changes.
 */
_Static_assert(MESH_TDMA_NB_SLOTS * 1000u <
                 MESH_TDMA_SUPERFRAME_MS *
                   (unsigned)(TRAFFIC_INFO_MESH_PERIODIC_FREQ),
               "traffic_info_mesh_periodic() runs more slowly than one TDMA "
               "slot, so some owned slots would never be sampled. Raise the "
               "periodic frequency in conf/modules/traffic_info.xml or reduce "
               "MESH_TDMA_NB_SLOTS.");
#endif

/* Transport and device the periodic telemetry dispatcher handed us. They point
 * at statically allocated singletons (DefaultChannel / DefaultDevice); we only
 * cache them so that the deferred emission can reuse the same downlink. */
static struct transport_tx *mesh_trans;
static struct link_device *mesh_dev;

static uint8_t mesh_modem_in_flight(uint32_t now_ms);

/** GPS time of week in ms when the receiver is synchronised, otherwise the
 *  local monotonic clock. All nodes that hold a 3D fix therefore agree on the
 *  slot boundaries to within the GPS time transfer error (well under 1 ms),
 *  and nodes without a fix still spread themselves out by AC_ID. */
static uint32_t mesh_network_time_ms(void)
{
  if (gps.fix >= GPS_FIX_3D) {
    mesh_link.synced = true;
    return gps_tow_from_sys_ticks(sys_time.nb_tick);
  }
  mesh_link.synced = false;
  return get_sys_time_msec();
}

/** Pack course, ground speed and climb rate into the 32 bit multiplex field.
 * @param[in] course_ddeg course over ground in decidegrees, any range
 * @param[in] gspeed_cms  ground speed in cm/s, saturated at 204.7 m/s
 * @param[in] climb_cms   climb rate in cm/s, saturated at +/- 25.5 m/s
 */
static uint32_t mesh_multiplex_encode(int32_t course_ddeg, int32_t gspeed_cms, int32_t climb_cms)
{
  int32_t course = course_ddeg % 3600;
  if (course < 0) {
    course += 3600;
  }

  /* cm/s -> dm/s. Eleven bits of cm/s would saturate at 20.47 m/s, and this
   * airframe is set up for AIRSPEED_MAX 16 m/s, so any useful tailwind puts the
   * ground speed over the clip and a conflicting aircraft reads a neighbour as
   * slower than it really is - the one error a collision-avoidance input must
   * not make. Decimetres per second reach 204.7 m/s, and 0.1 m/s is still finer
   * than the GPS ground-speed noise. */
  int32_t gspeed = (gspeed_cms + 5) / 10;
  if (gspeed < 0) { gspeed = 0; }
  if (gspeed > 2047) { gspeed = 2047; }

  /* cm/s -> dm/s, rounded away from zero so that a slow climb never reads as
   * level flight to a conflicting aircraft */
  int32_t climb_dms = (climb_cms >= 0) ? (climb_cms + 5) / 10 : (climb_cms - 5) / 10;
  if (climb_dms > 255) { climb_dms = 255; }
  if (climb_dms < -256) { climb_dms = -256; }

  return (((uint32_t)course & 0x0FFFu) << 20)
         | (((uint32_t)gspeed & 0x07FFu) << 9)
         | ((uint32_t)climb_dms & 0x01FFu);
}

/** Inverse of ::mesh_multiplex_encode. Outputs use the units of
 *  ::set_ac_info_lla, i.e. decidegrees and cm/s. */
static void mesh_multiplex_decode(uint32_t multiplex, int16_t *course_ddeg,
                                  uint16_t *gspeed_cms, int16_t *climb_cms)
{
  const uint32_t course = (multiplex >> 20) & 0x0FFFu;
  const uint32_t gspeed = (multiplex >> 9) & 0x07FFu;
  /* branch free sign extension of a 9 bit two's complement field */
  const int32_t climb_dms = (int32_t)((multiplex & 0x01FFu) ^ 0x0100u) - 0x0100;

  *course_ddeg = (int16_t)((course < 3600u) ? course : 0u);
  *gspeed_cms = (uint16_t)(gspeed * 10);      /* dm/s on the wire -> cm/s */
  *climb_cms = (int16_t)(climb_dms * 10);
}

/** Map the firmware specific autopilot mode onto the unified 3 bit mesh mode,
 *  so that a fixedwing and a rotorcraft describe themselves identically. */
static uint8_t mesh_unified_mode(void)
{
  if (autopilot_throttle_killed()) {
    return MESH_MODE_KILL;
  }

  switch (autopilot_get_mode()) {
#ifdef AP_MODE_FAILSAFE
    case AP_MODE_FAILSAFE:          return MESH_MODE_FAILSAFE;
#endif
#ifdef AP_MODE_KILL
    case AP_MODE_KILL:              return MESH_MODE_KILL;
#endif
#ifdef AP_MODE_HOME
    case AP_MODE_HOME:              return MESH_MODE_HOME;
#endif
#ifdef AP_MODE_NOGPS
    case AP_MODE_NOGPS:             return MESH_MODE_NOGPS;
#endif
#if FIXEDWING_FIRMWARE
#ifdef AP_MODE_MANUAL
    case AP_MODE_MANUAL:            return MESH_MODE_MANUAL;
#endif
#ifdef AP_MODE_AUTO1
    case AP_MODE_AUTO1:             return MESH_MODE_ASSISTED;
#endif
#ifdef AP_MODE_AUTO2
    case AP_MODE_AUTO2:             return MESH_MODE_AUTO;
#endif
#else /* rotorcraft and hybrid */
#ifdef AP_MODE_NAV
    case AP_MODE_NAV:               return MESH_MODE_AUTO;
#endif
#ifdef AP_MODE_GUIDED
    case AP_MODE_GUIDED:            return MESH_MODE_AUTO;
#endif
#ifdef AP_MODE_RATE_DIRECT
    case AP_MODE_RATE_DIRECT:       return MESH_MODE_MANUAL;
#endif
#ifdef AP_MODE_ATTITUDE_DIRECT
    case AP_MODE_ATTITUDE_DIRECT:   return MESH_MODE_MANUAL;
#endif
#ifdef AP_MODE_RC_DIRECT
    case AP_MODE_RC_DIRECT:         return MESH_MODE_MANUAL;
#endif
#endif
    default:                        return MESH_MODE_UNKNOWN;
  }
}

static uint8_t mesh_state_flags(void)
{
  uint8_t flags = mesh_unified_mode() & MESH_FLAG_MODE_MASK;

#if !FIXEDWING_FIRMWARE
  flags |= MESH_FLAG_ROTORCRAFT;
#endif

  if (gps.fix >= GPS_FIX_3D && state.pos_status != 0) {
    flags |= MESH_FLAG_POS_VALID;
  }
  if (autopilot_in_flight()) {
    flags |= MESH_FLAG_AIRBORNE;
  }
  if ((flags & MESH_FLAG_MODE_MASK) >= MESH_MODE_NOGPS) {
    /* NOGPS, FAILSAFE and KILL all mean "not following the flight plan" */
    flags |= MESH_FLAG_EMERGENCY;
  }
#ifdef LOW_BAT_LEVEL
  if (electrical.vsupply < LOW_BAT_LEVEL) {
    flags |= MESH_FLAG_ALERT;
  }
#endif

  return flags;
}

/** How often this node should originate its own state.
 *
 * The base share still comes from the live population size, but the mesh now
 * biases the claim count with state that already exists onboard:
 *
 *  - a synchronized, airborne node with a valid position can use spare slots;
 *  - landed or unsynchronized nodes keep one heartbeat slot;
 *  - emergency, HOME and low-battery states get the available spare slots;
 *  - modem-cache pressure contracts the share before the E52 can overflow.
 *
 * This is traffic shaping, not relay election. Every E52 is provisioned as a
 * routing node and performs forwarding below Paparazzi. The application only
 * controls when it originates a fresh MESH_STATE frame.
 */
static uint8_t mesh_redundancy_target(uint8_t nodes, uint32_t now_ms)
{
  const uint8_t flags = mesh_state_flags();
  uint8_t target = (uint8_t)(MESH_TDMA_NB_SLOTS / Max(nodes, 1u));
  if (target < 1) {
    target = 1;
  }
  if (target > MESH_TDMA_MAX_REUSE) {
    target = MESH_TDMA_MAX_REUSE;
  }

  /* A node which cannot contribute a current position is a poor source of
   * repeated state. Keep one heartbeat slot so it remains discoverable, but
   * let healthy peers use the spare capacity. A landed aircraft behaves the
   * same way; AC_ID 0 is exempt because the GCS is a real stationary peer. */
  if (!mesh_link.synced || (flags & MESH_FLAG_POS_VALID) == 0
      || (AC_ID != 0 && (flags & MESH_FLAG_AIRBORNE) == 0)) {
    return 1;
  }

  /* Emergency, failsafe and low-battery states are the frames the rest of the
   * swarm must see first. Give them one extra slot, not the whole reuse cap:
   * several aircraft can enter HOME together, and a 1->4 jump by all of them
   * creates needless claim churn even though expansion itself is serialized. */
  if ((flags & (MESH_FLAG_EMERGENCY | MESH_FLAG_ALERT)) != 0
      || (flags & MESH_FLAG_MODE_MASK) == MESH_MODE_HOME) {
    return Min(MESH_TDMA_MAX_REUSE, target + 1u);
  }

  /* Back-pressure outranks update rate. The E52 force-clears all five cached
   * frames on overflow, so a node seeing a busy local cache contracts before
   * it asks for another opportunistic slot. */
  if (mesh_modem_in_flight(now_ms) >= MESH_CACHE_HIGH_WATER && target > 1) {
    target--;
  }
  return target;
}

/** Hand one MESH_STATE frame to the modem. Snapshots the state at call time,
 *  not at request time, so the deferral introduced by the TDMA slot costs
 *  latency but never accuracy. */
static void mesh_state_emit(void)
{
  const struct LlaCoor_i *lla = stateGetPositionLla_i();

  uint8_t flags = mesh_state_flags();
  int32_t lat = lla->lat;
  int32_t lon = lla->lon;
  int32_t alt = lla->alt / 10;                 /* mm above ellipsoid -> cm */
  uint32_t multiplex = mesh_multiplex_encode(
                         (int32_t)DeciDegOfRad(stateGetHorizontalSpeedDir_f()),
                         (int32_t)(stateGetHorizontalSpeedNorm_f() * 100.f),
                         (int32_t)(stateGetSpeedEnu_f()->z * 100.f));

  struct pprzlink_msg msg;
  msg.trans = mesh_trans;
  msg.dev = mesh_dev;
  msg.sender_id = AC_ID;
  msg.receiver_id = PPRZLINK_MSG_BROADCAST;
  msg.component_id = 0;
  pprzlink_msg_send_MESH_STATE(&msg, &flags, &lat, &lon, &alt, &multiplex);
}

/** Periodic telemetry callback for MESH_STATE.
 *
 * It does *not* transmit. The periodic telemetry counters are free running
 * from boot and are therefore uncorrelated between aircraft; transmitting here
 * would let several nodes originate at the same instant, which floods every
 * modem cache in the mesh with simultaneous relay frames. Instead the request
 * is latched and ::traffic_info_mesh_periodic releases it inside this node's
 * own GPS synchronised slot.
 */
static void request_mesh_state(struct transport_tx *trans, struct link_device *dev)
{
  /* This callback exists only to learn which transport and device the telemetry
   * subsystem wants us to use. It deliberately does not transmit and does not
   * pace anything.
   *
   * An earlier version latched a "pending" flag here and let the transmit gate
   * consume it. That silently threw away transmit opportunities: the telemetry
   * scheduler fires on a counter that starts at boot, whereas slots are aligned
   * to GPS time, so the two clocks have an arbitrary and drifting offset. A
   * node owning four slots per superframe would find the flag already consumed
   * on some of them and skip. Emission is now driven purely by slot ownership,
   * which is the only clock that matters here. */
  mesh_trans = trans;
  mesh_dev = dev;
  mesh_link.ready = true;
}

/** Estimated number of frames still queued inside the modem.
 *  Leaky bucket: every handover pushes the "cache empty" instant one drain
 *  time further into the future. */
static uint8_t mesh_modem_in_flight(uint32_t now_ms)
{
  const int32_t remaining = (int32_t)(mesh_link.modem_free_ms - now_ms);
  if (remaining <= 0) {
    return 0;
  }
  return (uint8_t)Min(255, ((uint32_t)remaining + MESH_MODEM_DRAIN_MS - 1) / MESH_MODEM_DRAIN_MS);
}

/* ------------------------------------------------------------------------- *
 * Self-organising slot allocation
 *
 * The mesh has no coordinator, nodes join and leave at will, and the ground
 * station is just another mobile member. A slot map fixed at compile time
 * cannot survive that, so slots are claimed, defended and released at run time
 * - the approach marine AIS and VDL Mode 4 use for exactly this problem
 * (many mobile peers, no central authority, periodic position broadcast).
 *
 * Three properties make it cheap here:
 *
 *  1. a frame's slot is implied by its GPS-aligned arrival time, so the
 *     occupancy map is learned from ordinary MESH_STATE traffic and costs
 *     nothing on air - no reservation protocol, no extra message, no bytes;
 *  2. collisions are resolved by a deterministic rule (higher AC_ID yields),
 *     so the two nodes involved never both move and never both stay;
 *  3. spare membership is converted into update rate, but now with a state
 *     bias: the base claim count still scales with N/k, then the node uses its
 *     own flight state and local cache pressure to decide whether it is healthy
 *     enough to originate extra state. This never changes modem forwarding:
 *     every routing E52 still forwards each newly received broadcast once.
 *
 * All state is static: one MeshSlot per slot, nothing allocated, no recursion.
 * ------------------------------------------------------------------------- */

static struct MeshSlot mesh_slots[MESH_TDMA_NB_SLOTS];

/** Slots this node currently claims. Index 0 is the primary, which it always
 *  holds; the rest are opportunistic and are surrendered the moment a real
 *  owner appears. */
static uint8_t mesh_owned[MESH_TDMA_MAX_REUSE];
static uint16_t mesh_owned_until[MESH_TDMA_MAX_REUSE];
static uint8_t mesh_owned_n;

/** Superframes observed since boot. Used for network entry: a node listens
 *  before it expands beyond its primary slot. */
static uint16_t mesh_frames_seen;
static uint16_t mesh_last_frame;
static bool mesh_entered;          ///< network entry slot choice has been made

/** Superframe index of a network timestamp. Wraps harmlessly in 16 bits. */
static inline uint16_t mesh_frame_of(uint32_t net_ms)
{
  return (uint16_t)(net_ms / MESH_TDMA_SUPERFRAME_MS);
}

/** Slot index of a network timestamp.
 *
 * Scaling up before dividing keeps this exact for any superframe length: the
 * slot need not be a whole number of milliseconds (1000 ms over 16 slots is
 * 62.5 ms), and truncating the slot length first would map a sliver of every
 * superframe to a slot nobody owns.
 */
static inline uint8_t mesh_slot_of(uint32_t net_ms)
{
  const uint32_t frame_ms = net_ms % MESH_TDMA_SUPERFRAME_MS;
  return (uint8_t)((frame_ms * MESH_TDMA_NB_SLOTS) / MESH_TDMA_SUPERFRAME_MS);
}

static inline bool mesh_slot_is_stale(uint8_t s, uint16_t frame)
{
  return (uint16_t)(frame - mesh_slots[s].last_frame) > MESH_SLOT_AGE_FRAMES;
}

static inline bool mesh_slot_free(uint8_t s, uint16_t frame)
{
  return mesh_slots[s].ac_id == MESH_SLOT_FREE || mesh_slots[s].ac_id == AC_ID
         || mesh_slot_is_stale(s, frame);
}

/** Check whether @p slot has been silent long enough for a secondary claim.
 *
 * Twice the ageing window. A slot whose owner has only just aged out, or which
 * a newly entered node has claimed but whose first transmission we have not
 * processed yet, still reads as free in the map. Claiming it immediately is
 * how an expanding node lands on top of a newcomer's primary - and once they
 * are transmitting together neither can hear the other. Requiring a longer
 * quiet period than the ageing window closes that race.
 *
 * The primary is exempt: it is chosen from a genuinely free slot at entry and
 * defended by the AC_ID rule thereafter.
 */
static bool mesh_slot_quiet(uint8_t slot, uint16_t frame)
{
  return mesh_slots[slot].ac_id == MESH_SLOT_FREE
         && (uint16_t)(frame - mesh_slots[slot].last_frame) > (2 * MESH_SLOT_AGE_FRAMES);
}

static bool mesh_owns(uint8_t slot)
{
  /* Silent during network entry. A node that starts transmitting on its AC_ID
   * hint before it has heard anybody will sooner or later land on a slot that
   * is already in use, and the two are then mutually deaf - neither can apply
   * the "higher AC_ID yields" rule because neither can hear the other. So:
   * listen for a few complete superframes, build the map, and only then pick a
   * slot that is demonstrably free. Costs a few seconds of silence at power up
   * and removes the entire class of join-time collisions. */
  if (mesh_frames_seen < MESH_ENTRY_FRAMES) {
    return false;
  }
  for (uint8_t i = 0; i < mesh_owned_n; i++) {
    if (mesh_owned[i] == slot) {
      return true;
    }
  }
  return false;
}

/** Record the slot in which @p sender was heard transmitting.
 *
 * Called for every received MESH_STATE. A node that reappears after a landing
 * or a dropout simply starts being heard again and reclaims a slot; nothing
 * has to notice that it left.
 */
void mesh_slot_observe(uint8_t sender, uint32_t net_ms)
{
  /* Sender 0 is the ground station, a legitimate member of this mesh, not a
   * null value. Only our own relayed frames are discarded. */
  if (sender == AC_ID) {
    return;
  }
  const uint8_t s = mesh_slot_of(net_ms);
  const uint16_t frame = mesh_frame_of(net_ms);

  if (mesh_slots[s].ac_id == MESH_SLOT_FREE || mesh_slots[s].ac_id == sender
      || mesh_slot_is_stale(s, frame)) {
    mesh_slots[s].ac_id = sender;
    mesh_slots[s].last_frame = frame;
  }
}

/** Pick a free slot for the primary, biased by AC_ID so that two nodes
 *  reselecting in the same frame rarely land on the same one. */
static uint8_t mesh_pick_slot(uint16_t frame)
{
  const uint8_t start = (uint8_t)((AC_ID * 7u + mesh_link.reselect_count * 3u)
                                  % MESH_TDMA_NB_SLOTS);
  for (uint8_t k = 0; k < MESH_TDMA_NB_SLOTS; k++) {
    const uint8_t s = (uint8_t)((start + k) % MESH_TDMA_NB_SLOTS);
    if (mesh_slot_free(s, frame) && !mesh_owns(s)) {
      return s;
    }
  }
  return start;   /* mesh full: keep transmitting, the modem CSMA copes */
}

/** Randomised lease length for an opportunistic slot.
 *
 * A secondary slot is held for MESH_SLOT_HOLD_MIN..+SPAN superframes and then
 * surrendered, exactly as an AIS station times out its slot reservations.
 *
 * This is what breaks the one deadlock the occupancy map cannot see. If node A
 * is using a slot as a secondary and node B claims the same slot as its
 * primary, the two transmit on top of each other; neither can hear the other,
 * and no third party can decode the collision either, so the "higher AC_ID
 * yields" rule never fires and both sit there forever. Because the leases are
 * seeded per node they expire at different frames: whichever secondary lapses
 * first leaves the slot, the other node becomes audible, and the map repairs
 * itself.
 */
static uint16_t mesh_lease_expiry(uint16_t frame)
{
  /* cheap deterministic scatter, distinct per node and per claim */
  const uint32_t r = (uint32_t)(AC_ID * 2654435761u
                                + frame * 40503u
                                + mesh_link.reselect_count * 97u);
  return (uint16_t)(frame + MESH_SLOT_HOLD_MIN
                    + (uint16_t)(r % MESH_SLOT_HOLD_SPAN));
}

/** Randomised lease length for the primary slot. Much longer than a
 *  secondary's: this exists only to break a mutual-deafness deadlock, not to
 *  share capacity, so it should almost never fire in a healthy mesh. */
static uint16_t mesh_primary_expiry(uint16_t frame)
{
  const uint32_t r = (uint32_t)(AC_ID * 1103515245u
                                + frame * 12345u
                                + mesh_link.reselect_count * 7919u);
  return (uint16_t)(frame + MESH_PRIMARY_HOLD_MIN
                    + (uint16_t)(r % MESH_PRIMARY_HOLD_SPAN));
}

/** May this node take an extra slot in this superframe?
 *
 * Expansion is serialised round-robin over the known membership: a node may
 * only grow when @c frame modulo @p nodes equals its rank in the sorted
 * list of live AC_IDs.
 *
 * This is not tidiness, it is necessary. A node cannot hear a collision in a
 * slot it is transmitting in, and neither can anyone else - a collision
 * delivers nothing to anybody. So if two nodes claim the same free slot in the
 * same frame, they can sit on top of each other indefinitely with no evidence
 * available to either. Serialising expansion means the first claimant is heard
 * and recorded by everyone else before the next node gets its turn, so the
 * second never picks that slot in the first place.
 */
static bool mesh_expansion_turn(uint16_t frame, uint8_t nodes)
{
  uint8_t rank = 0;
  for (uint8_t s = 0; s < MESH_TDMA_NB_SLOTS; s++) {
    const uint8_t id = mesh_slots[s].ac_id;
    if (id != MESH_SLOT_FREE && id != AC_ID && id < AC_ID) {
      bool dup = false;
      for (uint8_t t = 0; t < s; t++) {
        if (mesh_slots[t].ac_id == id) { dup = true; break; }
      }
      if (!dup) { rank++; }
    }
  }
  return (uint8_t)(frame % (nodes ? nodes : 1)) == rank;
}

/** Age the map, defend the primary slot, and size this node's share.
 *
 * Two rules keep this stable, and the churn simulation in
 * sw/tools/mesh/mesh_slot_sim.py exists because getting them wrong is not
 * obvious from reading the code:
 *
 *  * **Listen before claiming.** A node that has just booted sees an empty map
 *    and would otherwise conclude it is alone and grab the maximum share. If
 *    every node does that simultaneously they collide on every slot, and
 *    because a collision delivers nothing, none of them ever learns the others
 *    exist - a permanent deadlock. So the share stays at one slot until the
 *    node has watched a couple of complete superframes.
 *
 *  * **Expand slowly, contract immediately.** At most one extra slot is taken
 *    per superframe, and only one that has been observed free; any slot a real
 *    owner appears in is surrendered at once. Growth is therefore always
 *    slower than the detection of a conflict.
 */
static void mesh_slot_maintain(uint16_t frame, uint32_t now_ms)
{
  /* --- age the map, count distinct live nodes ---------------------------- */
  uint8_t nodes = 1;                                   /* ourselves */
  uint8_t seen[MESH_TDMA_NB_SLOTS];
  uint8_t seen_n = 0;
  for (uint8_t s = 0; s < MESH_TDMA_NB_SLOTS; s++) {
    if (mesh_slots[s].ac_id == MESH_SLOT_FREE) {
      continue;
    }
    if (mesh_slot_is_stale(s, frame)) {
      mesh_slots[s].ac_id = MESH_SLOT_FREE;            /* owner left or landed */
      continue;   /* last_frame is deliberately kept: see mesh_slot_quiet() */
    }
    if (mesh_slots[s].ac_id == AC_ID) {
      continue;
    }
    bool dup = false;
    for (uint8_t i = 0; i < seen_n; i++) {
      if (seen[i] == mesh_slots[s].ac_id) { dup = true; break; }
    }
    if (!dup) {
      seen[seen_n++] = mesh_slots[s].ac_id;            /* one node, many slots */
      nodes++;
    }
  }
  mesh_link.neighbours = (uint8_t)(nodes - 1);

  /* --- network entry: listen first, then choose a slot that is really free */
  if (mesh_frames_seen == MESH_ENTRY_FRAMES && !mesh_entered) {
    mesh_entered = true;
    mesh_owned[0] = mesh_pick_slot(frame);
    mesh_owned_until[0] = mesh_primary_expiry(frame);
    mesh_owned_n = 1;
  }

  /* --- the primary has a lease too --------------------------------------- *
   * Two nodes whose PRIMARY slots coincide are the one case that nothing else
   * can resolve: they are mutually deaf, so the "higher AC_ID yields" rule
   * never fires, and no third party can decode the collision to report it
   * either. Leaving the primary permanent therefore leaves a genuine deadlock.
   * A long randomised lease removes it - the two lapse at different frames,
   * whichever re-picks first becomes audible, and the map repairs. Long enough
   * (tens of superframes) that a healthy node effectively keeps its slot. */
  if ((int16_t)(frame - mesh_owned_until[0]) >= 0) {
    mesh_owned[0] = mesh_pick_slot(frame);
    mesh_owned_until[0] = mesh_primary_expiry(frame);
    mesh_link.reselect_count++;
  }

  /* --- defend the primary ------------------------------------------------ */
  const uint8_t owner = mesh_slots[mesh_owned[0]].ac_id;
    if (owner != MESH_SLOT_FREE && owner != AC_ID
      && !mesh_slot_is_stale(mesh_owned[0], frame)
      && AC_ID > owner) {
    mesh_owned[0] = mesh_pick_slot(frame);             /* higher AC_ID yields */
    mesh_link.reselect_count++;
  }
  mesh_link.slot = mesh_owned[0];

  /* --- drop secondaries that a real owner took, or whose lease expired ---- */
  uint8_t keep = 1;
  for (uint8_t i = 1; i < mesh_owned_n; i++) {
    const uint8_t s = mesh_owned[i];
    const bool expired = (int16_t)(frame - mesh_owned_until[i]) >= 0;
    if (mesh_slot_free(s, frame) && !expired) {
      mesh_owned[keep] = s;
      mesh_owned_until[keep] = mesh_owned_until[i];
      keep++;
    }
  }
  mesh_owned_n = keep;

  /* --- size the fair share ----------------------------------------------- */
  uint8_t target = mesh_redundancy_target(nodes, now_ms);
  if (mesh_frames_seen < MESH_ENTRY_FRAMES) {
    target = 1;                                        /* listen before claiming */
  }

  if (mesh_owned_n > target) {
    mesh_owned_n = target;                             /* contract immediately */
  } else if (mesh_owned_n < target && mesh_expansion_turn(frame, nodes)) {
    /* Expand by at most one slot, and only on this node's turn - see
     * ::mesh_expansion_turn for why that serialisation is essential. */
    const uint8_t start = (uint8_t)((mesh_owned[0] + MESH_TDMA_NB_SLOTS / 2u)
                                    % MESH_TDMA_NB_SLOTS);
    for (uint8_t k = 0; k < MESH_TDMA_NB_SLOTS; k++) {
      const uint8_t s = (uint8_t)((start + k) % MESH_TDMA_NB_SLOTS);
      if (mesh_slot_quiet(s, frame) && !mesh_owns(s)) {
        mesh_owned[mesh_owned_n] = s;
        mesh_owned_until[mesh_owned_n] = mesh_lease_expiry(frame);
        mesh_owned_n++;
        break;
      }
    }
  }

  mesh_link.reuse = mesh_owned_n;
}

void traffic_info_mesh_periodic(void)
{
  const uint32_t net_ms = mesh_network_time_ms();
  const uint32_t now_ms = get_sys_time_msec();
  const uint16_t frame = mesh_frame_of(net_ms);
  const uint8_t slot = mesh_slot_of(net_ms);

  /* Slot bookkeeping is a per-SUPERFRAME activity, not a per-tick one.
   *
   * This task runs at 20 Hz, so a superframe covers about twenty calls. Running
   * the maintenance on every one of them would let a node take its whole share
   * of slots inside a single frame: mesh_expansion_turn() is a function of the
   * frame number alone, so it stays true for every tick of that frame, and the
   * "expand by at most one slot per superframe" rule - which is what keeps two
   * nodes from claiming the same free slot before either has been heard -
   * would be defeated twenty times over. */
  if (frame != mesh_last_frame) {
    mesh_last_frame = frame;
    if (mesh_frames_seen < 0xFFFF) {
      mesh_frames_seen++;
    }
    mesh_slot_maintain(frame, now_ms);
  }

  if (!mesh_link.ready || mesh_trans == NULL || mesh_dev == NULL) {
    return;
  }
  if (!mesh_owns(slot)) {
    mesh_link.defer_ticks++;
    return;
  }

  /* One origination per (superframe, slot) pair. */
  const uint32_t key = (uint32_t)frame * MESH_TDMA_NB_SLOTS + slot;
  if (key == mesh_link.last_emit_key) {
    return;
  }

  if (mesh_modem_in_flight(now_ms) >= MESH_CACHE_HIGH_WATER) {
    mesh_link.throttled_count++;
    return;
  }

  mesh_state_emit();

  mesh_link.last_emit_key = key;
  mesh_link.tx_count++;
  mesh_link.modem_free_ms = Max(mesh_link.modem_free_ms, now_ms) + MESH_MODEM_DRAIN_MS;
}
#endif /* TRAFFIC_INFO_USE_MESH */

void traffic_info_init(void)
{
  memset(ti_acs_id, 0, NB_ACS_ID);

  ti_acs_id[0] = 0;  // ground station
  ti_acs_id[AC_ID] = 1;
  ti_acs[ti_acs_id[AC_ID]].ac_id = AC_ID;
  ti_acs_idx = 2;

  geoid_height = NAV_MSL0;

#if TRAFFIC_INFO_USE_MESH
  for (uint8_t s = 0; s < MESH_TDMA_NB_SLOTS; s++) {
    mesh_slots[s].ac_id = MESH_SLOT_FREE;   /* not 0: that is the GCS */
    mesh_slots[s].last_frame = 0;
  }
  mesh_owned[0] = (uint8_t)(MESH_TDMA_SLOT_HINT % MESH_TDMA_NB_SLOTS);
  mesh_owned_until[0] = MESH_PRIMARY_HOLD_MIN;
  mesh_owned_n = 1;
  mesh_entered = false;
  mesh_frames_seen = 0;
  mesh_link.slot = mesh_owned[0];
  mesh_link.reuse = 1;
  mesh_link.modem_free_ms = get_sys_time_msec();
  mesh_link.last_emit_key = UINT32_MAX;
#endif

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_ACINFO_LLA, send_acinfo_lla);
#if TRAFFIC_INFO_USE_MESH
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_MESH_STATE, request_mesh_state);
#endif
#endif
}

/**
 * Update estimate of the geoid height
 * Requires an available hsml and/or lla measurement, if not available value isn't updated
 */
static void update_geoid_height(void) {
  if(bit_is_set(gps.valid_fields, GPS_VALID_HMSL_BIT) && bit_is_set(gps.valid_fields, GPS_VALID_POS_LLA_BIT))
  {
    geoid_height = gps.lla_pos.alt - gps.hmsl;
  } else if(bit_is_set(gps.valid_fields, GPS_VALID_POS_LLA_BIT))
  {
    geoid_height = wgs84_ellipsoid_to_geoid_i(gps.lla_pos.lat, gps.lla_pos.lon);
  } /* default is just keep last available height */
}

bool parse_acinfo_dl(uint8_t *buf)
{
  uint8_t sender_id = SenderIdOfPprzMsg(buf);
  uint8_t msg_id = IdOfPprzMsg(buf);
  uint8_t class_id = pprzlink_get_msg_class_id(buf);
  uint32_t itow = 0;

  /* handle telemetry message */
#if PPRZLINK_DEFAULT_VER == 2
  if (class_id == DL_telemetry_CLASS_ID) {
#else
  if (sender_id > 0) {
#endif
    switch (msg_id) {
      case DL_GPS_SMALL: {
        uint32_t multiplex_speed = DL_GPS_SMALL_multiplex_speed(buf);

        // decode compressed values
        int16_t course = (int16_t)((multiplex_speed >> 21) & 0x7FF); // bits 31-21 course in decideg
        if (course & 0x400) {
          course |= 0xF800;  // fix for twos complements
        }
        course *= 2; // scale course by resolution
        int16_t gspeed = (int16_t)((multiplex_speed >> 10) & 0x7FF); // bits 20-10 ground speed cm/s
        if (gspeed & 0x400) {
          gspeed |= 0xF800;  // fix for twos complements
        }
        int16_t climb = (int16_t)(multiplex_speed & 0x3FF); // bits 9-0 z climb speed in cm/s
        if (climb & 0x200) {
          climb |= 0xFC00;  // fix for twos complements
        }
        itow = gps_tow_from_sys_ticks(sys_time.nb_tick);

        set_ac_info_lla(sender_id,
                        DL_GPS_SMALL_lat(buf),
                        DL_GPS_SMALL_lon(buf),
                        (int32_t)DL_GPS_SMALL_alt(buf) * 10,
                        course,
                        gspeed,
                        climb,
                        itow);
      }
      break;
      case DL_GPS: {
        itow = DL_GPS_itow(buf);
        set_ac_info_utm(sender_id,
                    DL_GPS_utm_east(buf),
                    DL_GPS_utm_north(buf),
                    DL_GPS_alt(buf),
                    DL_GPS_utm_zone(buf),
                    DL_GPS_course(buf),
                    DL_GPS_speed(buf),
                    DL_GPS_climb(buf),
                    itow);
      }
      break;
      case DL_GPS_LLA: {
        itow = DL_GPS_LLA_itow(buf);
        set_ac_info_lla(sender_id,
                        DL_GPS_LLA_lat(buf),
                        DL_GPS_LLA_lon(buf),
                        DL_GPS_LLA_alt(buf),
                        DL_GPS_LLA_course(buf),
                        DL_GPS_LLA_speed(buf),
                        DL_GPS_LLA_climb(buf),
                        itow);
      }
      break;
      case DL_GPS_INT: {
        struct LtpDef_i def = {0};
        struct EcefCoor_i ecef_pos = {
          .x = DL_GPS_INT_ecef_x(buf),
          .y = DL_GPS_INT_ecef_y(buf),
          .z = DL_GPS_INT_ecef_z(buf),
        };
        struct EcefCoor_i ecef_vel = {
          .x = DL_GPS_INT_ecef_xd(buf),
          .y = DL_GPS_INT_ecef_yd(buf),
          .z = DL_GPS_INT_ecef_zd(buf),
        };
        struct NedCoor_i ned_vel;
        ltp_def_from_ecef_i(&def, &ecef_pos);
        ned_of_ecef_vect_i(&ned_vel, &def, &ecef_vel);
        struct NedCoor_f ned_vel_f;
        VECT3_FLOAT_OF_CM(ned_vel_f, ned_vel);
        int16_t course = (int16_t)(10.f * DegOfRad(atan2f(ned_vel_f.y, ned_vel_f.x))); // decideg
        uint16_t gspeed = (uint16_t)(CM_OF_M(FLOAT_VECT2_NORM(ned_vel_f)));
        int16_t climb = (int16_t)(CM_OF_M(-ned_vel_f.z));
        itow = DL_GPS_INT_tow(buf);
        set_ac_info_lla(sender_id,
                        DL_GPS_INT_lat(buf),
                        DL_GPS_INT_lon(buf),
                        DL_GPS_INT_alt(buf),
                        course,
                        gspeed,
                        climb,
                        itow);
      }
      break;
      default:
        return FALSE;
    }
  /* handle datalink message */
  } else if (class_id == DL_datalink_CLASS_ID) {
    switch (msg_id) {
#if TRAFFIC_INFO_USE_MESH
      case DL_MESH_STATE: {
        /* Flooded broadcasts return to their originator. Keep the local
         * estimator, rather than a relayed copy, as this aircraft's state. */
        if (sender_id == AC_ID) {
          return TRUE;
        }
        int16_t course;
        uint16_t gspeed;
        int16_t climb;
        mesh_multiplex_decode(DL_MESH_STATE_multiplex_speed(buf), &course, &gspeed, &climb);

        /* The originator is the PPRZLink sender, not a payload field: there is
         * no ac_id in MESH_STATE, so a relayed frame cannot claim to be from
         * someone else without also rewriting the frame header. */
        itow = gps_tow_from_sys_ticks(sys_time.nb_tick);

        /* The arrival time also tells us which slot the sender is using, which
         * is how the occupancy map is built without spending a single byte on
         * a reservation protocol. */
        mesh_slot_observe(sender_id, mesh_network_time_ms());

        set_ac_info_lla(sender_id,
                        DL_MESH_STATE_lat(buf),
                        DL_MESH_STATE_lon(buf),
                        DL_MESH_STATE_alt(buf) * 10,   /* cm -> mm */
                        course,
                        gspeed,
                        climb,
                        itow);
      }
      break;
    #endif
      case DL_ACINFO: {
        sender_id = DL_ACINFO_ac_id(buf); // may overwrite GCS id
        itow = DL_ACINFO_itow(buf);
        set_ac_info_utm(sender_id,
                        DL_ACINFO_utm_east(buf),
                        DL_ACINFO_utm_north(buf),
                        DL_ACINFO_alt(buf) * 10,
                        DL_ACINFO_utm_zone(buf),
                        DL_ACINFO_course(buf),
                        DL_ACINFO_speed(buf),
                        DL_ACINFO_climb(buf),
                        itow);
      }
      break;
      case DL_ACINFO_LLA: {
        sender_id = DL_ACINFO_LLA_ac_id(buf); // may overwrite GCS id
        itow = DL_ACINFO_LLA_itow(buf);
        set_ac_info_lla(sender_id,
                  DL_ACINFO_LLA_lat(buf),
                  DL_ACINFO_LLA_lon(buf),
                  DL_ACINFO_LLA_alt(buf) * 10,
                  DL_ACINFO_LLA_course(buf),
                  DL_ACINFO_LLA_speed(buf),
                  DL_ACINFO_LLA_climb(buf),
                  itow);
      }
      break;
      default:
        return FALSE;
    }
  } else {
    return FALSE; // unsupported class
  }
#if TRAFFIC_INFO_USE_LOG
  struct LlaCoor_f* lla_f = acInfoGetPositionLla_f(sender_id);
  struct EnuCoor_f* enu_f = acInfoGetPositionEnu_f(sender_id);
  if (LogFileIsOpen(pprzLogFile)) {
    LogWrite(pprzLogFile, "R,%d,%d,%.7f,%.7f,%.3f,%.3f,%.3f,%.3f,%d,%d\n",
        sender_id, AC_ID,
        DegOfRad(lla_f->lat), DegOfRad(lla_f->lon), lla_f->alt,
        enu_f->x, enu_f->y, enu_f->z,
        itow, gps.tow);
  }
#endif
  return TRUE;
}


void set_ac_info_utm(uint8_t id, uint32_t utm_east, uint32_t utm_north, uint32_t alt, uint8_t utm_zone, uint16_t course,
                 uint16_t gspeed, uint16_t climb, uint32_t itow)
{
  const uint8_t slot = ti_acs_slot(id);
  if (slot == TI_ACS_NONE) {
    return;
  }

  if (itow < ti_acs[slot].itow) {
    return; // don't update on old data
  }

  ti_acs[slot].status = 0;

  uint16_t my_zone = stateGetUtmOrigin_f()->zone;
  if (utm_zone == my_zone) {
    ti_acs[slot].utm_pos_i.east = utm_east;
    ti_acs[slot].utm_pos_i.north = utm_north;
    ti_acs[slot].utm_pos_i.alt = alt;
    ti_acs[slot].utm_pos_i.zone = utm_zone;
    SetBit(ti_acs[slot].status, AC_INFO_POS_UTM_I);

  } else { // store other uav in utm extended zone
    struct UtmCoor_i utm = {.east = utm_east, .north = utm_north, .alt = alt, .zone = utm_zone};
    struct LlaCoor_i lla;
    lla_of_utm_i(&lla, &utm);
    LLA_COPY(ti_acs[slot].lla_pos_i, lla);
    SetBit(ti_acs[slot].status, AC_INFO_POS_LLA_I);

    utm.zone = my_zone;
    utm_of_lla_i(&utm, &lla);

    UTM_COPY(ti_acs[slot].utm_pos_i, utm);
    SetBit(ti_acs[slot].status, AC_INFO_POS_UTM_I);
  }

  ti_acs[slot].course = RadOfDeciDeg(course);
  ti_acs[slot].gspeed = MOfCm(gspeed);
  ti_acs[slot].climb = MOfCm(climb);
  SetBit(ti_acs[slot].status, AC_INFO_VEL_LOCAL_F);

  ti_acs[slot].itow = itow;
}

void set_ac_info_lla(uint8_t id, int32_t lat, int32_t lon, int32_t alt,
                     int16_t course, uint16_t gspeed, int16_t climb, uint32_t itow)
{
  const uint8_t slot = ti_acs_slot(id);
  if (slot == TI_ACS_NONE) {
    return;
  }

  if (itow < ti_acs[slot].itow) {
    return; // don't update on old data
  }

  ti_acs[slot].status = 0;

  struct LlaCoor_i lla = {.lat = lat, .lon = lon, .alt = alt};
  LLA_COPY(ti_acs[slot].lla_pos_i, lla);
  SetBit(ti_acs[slot].status, AC_INFO_POS_LLA_I);

  ti_acs[slot].course = RadOfDeciDeg(course);
  ti_acs[slot].gspeed = MOfCm(gspeed);
  ti_acs[slot].climb = MOfCm(climb);
  SetBit(ti_acs[slot].status, AC_INFO_VEL_LOCAL_F);

  ti_acs[slot].itow = itow;
}

/* Lazy reference-frame conversions. Each function computes only a missing
 * representation and marks it valid in the aircraft's status bit field. */

/* compute UTM position of aircraft with ac_id (int) */
void acInfoCalcPositionUtm_i(uint8_t ac_id)
{
  uint8_t ac_nr = ti_acs_id[ac_id];
  if (bit_is_set(ti_acs[ac_nr].status, AC_INFO_POS_UTM_I))
  {
    return;
  }

  /* LLA_i -> UTM_i is more accurate than from UTM_f */
  if (bit_is_set(ti_acs[ac_nr].status, AC_INFO_POS_LLA_I))
  {
    // use my zone as reference, i.e zone extend
    ti_acs[ac_nr].utm_pos_i.zone = stateGetUtmOrigin_f()->zone;
    utm_of_lla_i(&ti_acs[ac_nr].utm_pos_i, &ti_acs[ac_nr].lla_pos_i);
    update_geoid_height();
    ti_acs[ac_nr].utm_pos_i.alt -= geoid_height;
  } else if (bit_is_set(ti_acs[ac_nr].status, AC_INFO_POS_UTM_F))
  {
    UTM_BFP_OF_REAL(ti_acs[ac_nr].utm_pos_i, ti_acs[ac_nr].utm_pos_f);
  } else if (bit_is_set(ti_acs[ac_nr].status, AC_INFO_POS_LLA_F))
  {
    // use my zone as reference, i.e zone extend
    ti_acs[ac_nr].utm_pos_i.zone = stateGetUtmOrigin_f()->zone;
    utm_of_lla_f(&ti_acs[ac_nr].utm_pos_f, &ti_acs[ac_nr].lla_pos_f);
    update_geoid_height();
    ti_acs[ac_nr].utm_pos_f.alt -= geoid_height/1000.;
    SetBit(ti_acs[ac_nr].status, AC_INFO_POS_UTM_F);
    UTM_BFP_OF_REAL(ti_acs[ac_nr].utm_pos_i, ti_acs[ac_nr].utm_pos_f);
  }
  SetBit(ti_acs[ac_nr].status, AC_INFO_POS_UTM_I);
}

/* compute LLA position of aircraft with ac_id (int) */
void acInfoCalcPositionLla_i(uint8_t ac_id)
{
  uint8_t ac_nr = ti_acs_id[ac_id];
  if (bit_is_set(ti_acs[ac_nr].status, AC_INFO_POS_LLA_I))
  {
    return;
  }

  if (bit_is_set(ti_acs[ac_nr].status, AC_INFO_POS_LLA_F))
  {
    LLA_BFP_OF_REAL(ti_acs[ac_nr].lla_pos_i, ti_acs[ac_nr].lla_pos_f);
  } else if (bit_is_set(ti_acs[ac_nr].status, AC_INFO_POS_UTM_I))
  {
    lla_of_utm_i(&ti_acs[ac_nr].lla_pos_i, &ti_acs[ac_nr].utm_pos_i);
    update_geoid_height();
    ti_acs[ac_nr].lla_pos_i.alt += geoid_height;
  } else if (bit_is_set(ti_acs[ac_nr].status, AC_INFO_POS_UTM_F))
  {
    lla_of_utm_f(&ti_acs[ac_nr].lla_pos_f, &ti_acs[ac_nr].utm_pos_f);
    update_geoid_height();
    ti_acs[ac_nr].lla_pos_f.alt += geoid_height/1000.;
    SetBit(ti_acs[ac_nr].status, AC_INFO_POS_LLA_F);
    LLA_BFP_OF_REAL(ti_acs[ac_nr].lla_pos_i, ti_acs[ac_nr].lla_pos_f);
  }
  SetBit(ti_acs[ac_nr].status, AC_INFO_POS_LLA_I);
}

/* compute ENU position of aircraft with ac_id (int) */
void acInfoCalcPositionEnu_i(uint8_t ac_id)
{
  uint8_t ac_nr = ti_acs_id[ac_id];
  if (bit_is_set(ti_acs[ac_nr].status, AC_INFO_POS_ENU_I))
  {
    return;
  }

  if (bit_is_set(ti_acs[ac_nr].status, AC_INFO_POS_ENU_F))
  {
    ENU_BFP_OF_REAL(ti_acs[ac_nr].enu_pos_i, ti_acs[ac_nr].enu_pos_f);
  }
  else if (state.ned_initialized_i)
  {
    if (bit_is_set(ti_acs[ac_nr].status, AC_INFO_POS_LLA_I) || bit_is_set(ti_acs[ac_nr].status, AC_INFO_POS_UTM_I))
    {
      struct EnuCoor_i enu;
      enu_of_lla_point_i(&enu, stateGetNedOrigin_i(), acInfoGetPositionLla_i(ac_id));
      // convert ENU pos from cm to BFP with INT32_POS_FRAC
      enu.x = POS_BFP_OF_REAL(enu.x) / 100;
      enu.y = POS_BFP_OF_REAL(enu.y) / 100;
      enu.z = POS_BFP_OF_REAL(enu.z) / 100;
      ti_acs[ac_nr].enu_pos_i = enu;
    } else if (bit_is_set(ti_acs[ac_nr].status, AC_INFO_POS_LLA_F) || bit_is_set(ti_acs[ac_nr].status, AC_INFO_POS_UTM_F))
    {
      enu_of_lla_point_f(&ti_acs[ac_nr].enu_pos_f, stateGetNedOrigin_f(), acInfoGetPositionLla_f(ac_id));
      SetBit(ti_acs[ac_nr].status, AC_INFO_POS_ENU_F);
      ENU_BFP_OF_REAL(ti_acs[ac_nr].enu_pos_i, ti_acs[ac_nr].enu_pos_f)
    }
  } else if (state.utm_initialized_f)
  {
    /* if utm origin is initialized we use the ENU = UTM - UTM_ORIGIN as in state to facilitate comparison */
    ENU_OF_UTM_DIFF(ti_acs[ac_nr].enu_pos_f, *acInfoGetPositionUtm_f(ac_id), *stateGetUtmOrigin_f());
    SetBit(ti_acs[ac_nr].status, AC_INFO_POS_ENU_F);
    ENU_BFP_OF_REAL(ti_acs[ac_nr].enu_pos_i, ti_acs[ac_nr].enu_pos_f);
  }
  SetBit(ti_acs[ac_nr].status, AC_INFO_POS_ENU_I);
}

/* compute UTM position of aircraft with ac_id (float) */
void acInfoCalcPositionUtm_f(uint8_t ac_id)
{
  uint8_t ac_nr = ti_acs_id[ac_id];
  if (bit_is_set(ti_acs[ac_nr].status, AC_INFO_POS_UTM_F))
  {
    return;
  }

  if (bit_is_set(ti_acs[ac_nr].status, AC_INFO_POS_UTM_I))
  {
    UTM_FLOAT_OF_BFP(ti_acs[ac_nr].utm_pos_f, ti_acs[ac_nr].utm_pos_i);
  } else if (bit_is_set(ti_acs[ac_nr].status, AC_INFO_POS_LLA_I))
  {
    // use my zone as reference, i.e zone extend
    ti_acs[ac_nr].utm_pos_i.zone = stateGetUtmOrigin_f()->zone;
    utm_of_lla_i(&ti_acs[ac_nr].utm_pos_i, &ti_acs[ac_nr].lla_pos_i);
    update_geoid_height();
    ti_acs[ac_nr].utm_pos_i.alt -= geoid_height;
    SetBit(ti_acs[ac_nr].status, AC_INFO_POS_UTM_I);
    UTM_FLOAT_OF_BFP(ti_acs[ac_nr].utm_pos_f, ti_acs[ac_nr].utm_pos_i);
  } else if (bit_is_set(ti_acs[ac_nr].status, AC_INFO_POS_LLA_F))
  {
    /* not very accurate with float ~5cm */
    ti_acs[ac_nr].utm_pos_f.zone = stateGetUtmOrigin_f()->zone;
    utm_of_lla_f(&ti_acs[ac_nr].utm_pos_f, &ti_acs[ac_nr].lla_pos_f);
    update_geoid_height();
    ti_acs[ac_nr].utm_pos_f.alt -= geoid_height/1000.;
  }
  SetBit(ti_acs[ac_nr].status, AC_INFO_POS_UTM_F);
}

/* compute LLA position of aircraft with ac_id (float) */
void acInfoCalcPositionLla_f(uint8_t ac_id)
{
  uint8_t ac_nr = ti_acs_id[ac_id];
  if (bit_is_set(ti_acs[ac_nr].status, AC_INFO_POS_LLA_F))
  {
    return;
  }

  if (bit_is_set(ti_acs[ac_nr].status, AC_INFO_POS_LLA_I))
  {
    LLA_FLOAT_OF_BFP(ti_acs[ac_nr].lla_pos_f, ti_acs[ac_nr].lla_pos_i);
  } else if (bit_is_set(ti_acs[ac_nr].status, AC_INFO_POS_UTM_I))
  {
    lla_of_utm_i(&ti_acs[ac_nr].lla_pos_i, &ti_acs[ac_nr].utm_pos_i);
    update_geoid_height();
    ti_acs[ac_nr].lla_pos_i.alt += geoid_height;
    SetBit(ti_acs[ac_nr].status, AC_INFO_POS_LLA_I);
    LLA_FLOAT_OF_BFP(ti_acs[ac_nr].lla_pos_f, ti_acs[ac_nr].lla_pos_i);
  } else if (bit_is_set(ti_acs[ac_nr].status, AC_INFO_POS_UTM_F))
  {
    lla_of_utm_f(&ti_acs[ac_nr].lla_pos_f, &ti_acs[ac_nr].utm_pos_f);
    update_geoid_height();
    ti_acs[ac_nr].lla_pos_f.alt += geoid_height/1000.;
  }
  SetBit(ti_acs[ac_nr].status, AC_INFO_POS_LLA_F);
}

/* Compute ENU velocity of aircraft with ac_id (float). */
void acInfoCalcPositionEnu_f(uint8_t ac_id)
{
  uint8_t ac_nr = ti_acs_id[ac_id];
  if (bit_is_set(ti_acs[ac_nr].status, AC_INFO_POS_ENU_F))
  {
    return;
  }

  if (bit_is_set(ti_acs[ac_nr].status, AC_INFO_POS_ENU_I))
  {
    ENU_FLOAT_OF_BFP(ti_acs[ac_nr].enu_pos_f, ti_acs[ac_nr].enu_pos_i);
  }
  else if (state.ned_initialized_i)
  {
    if (bit_is_set(ti_acs[ac_nr].status, AC_INFO_POS_LLA_F) || bit_is_set(ti_acs[ac_nr].status, AC_INFO_POS_UTM_F))
    {
      enu_of_lla_point_f(&ti_acs[ac_nr].enu_pos_f, stateGetNedOrigin_f(), acInfoGetPositionLla_f(ac_id));
    } else if (bit_is_set(ti_acs[ac_nr].status, AC_INFO_POS_LLA_I) || bit_is_set(ti_acs[ac_nr].status, AC_INFO_POS_UTM_I))
    {
      struct EnuCoor_i enu;
      enu_of_lla_point_i(&enu, stateGetNedOrigin_i(), acInfoGetPositionLla_i(ac_id));
      // convert ENU pos from cm to BFP with INT32_POS_FRAC
      enu.x = POS_BFP_OF_REAL(enu.x) / 100;
      enu.y = POS_BFP_OF_REAL(enu.y) / 100;
      enu.z = POS_BFP_OF_REAL(enu.z) / 100;
      ti_acs[ac_nr].enu_pos_i = enu;
      SetBit(ti_acs[ac_nr].status, AC_INFO_POS_ENU_I);
      ENU_FLOAT_OF_BFP(ti_acs[ac_nr].enu_pos_f, ti_acs[ac_nr].enu_pos_i);
    }
  } else if (state.utm_initialized_f)
  {
    /* if utm origin is initialized we use the ENU = UTM - UTM_ORIGIN as in state to facilitate comparison */
    ENU_OF_UTM_DIFF(ti_acs[ac_nr].enu_pos_f, *acInfoGetPositionUtm_f(ac_id), *stateGetUtmOrigin_f());
  }
  SetBit(ti_acs[ac_nr].status, AC_INFO_POS_ENU_F);
}

/* compute ENU velocity of aircraft with ac_id (int) */
void acInfoCalcVelocityEnu_i(uint8_t ac_id)
{
  uint8_t ac_nr = ti_acs_id[ac_id];
  if (bit_is_set(ti_acs[ac_nr].status, AC_INFO_VEL_ENU_I))
  {
    return;
  }

  if (bit_is_set(ti_acs[ac_nr].status, AC_INFO_VEL_ENU_F))
  {
    SPEEDS_BFP_OF_REAL(ti_acs[ac_nr].enu_vel_i, ti_acs[ac_nr].enu_vel_f);
  } else {
    ti_acs[ac_nr].enu_vel_f.x = ti_acs[ac_nr].gspeed * sinf(ti_acs[ac_nr].course);
    ti_acs[ac_nr].enu_vel_f.y = ti_acs[ac_nr].gspeed * cosf(ti_acs[ac_nr].course);
    ti_acs[ac_nr].enu_vel_f.z = ti_acs[ac_nr].climb;
    SetBit(ti_acs[ac_nr].status, AC_INFO_VEL_ENU_F);
    SPEEDS_BFP_OF_REAL(ti_acs[ac_nr].enu_vel_i, ti_acs[ac_nr].enu_vel_f);
  }
  SetBit(ti_acs[ac_nr].status, AC_INFO_VEL_ENU_I);
}

/* compute ENU position of aircraft with ac_id (float) */
void acInfoCalcVelocityEnu_f(uint8_t ac_id)
{
  uint8_t ac_nr = ti_acs_id[ac_id];
  if (bit_is_set(ti_acs[ac_nr].status, AC_INFO_VEL_ENU_F))
  {
    return;
  }

  if (bit_is_set(ti_acs[ac_nr].status, AC_INFO_VEL_ENU_I))
  {
    SPEEDS_FLOAT_OF_BFP(ti_acs[ac_nr].enu_vel_f, ti_acs[ac_nr].enu_vel_i);
  } else if (bit_is_set(ti_acs[ac_nr].status, AC_INFO_VEL_LOCAL_F)) {
    ti_acs[ac_nr].enu_vel_f.x = ti_acs[ac_nr].gspeed * sinf(ti_acs[ac_nr].course);
    ti_acs[ac_nr].enu_vel_f.y = ti_acs[ac_nr].gspeed * cosf(ti_acs[ac_nr].course);
    ti_acs[ac_nr].enu_vel_f.z = ti_acs[ac_nr].climb;
  }
  SetBit(ti_acs[ac_nr].status, AC_INFO_VEL_ENU_F);
}

void traffic_info_log_start(void)
{
#if TRAFFIC_INFO_USE_LOG
  LogOpen(pprzLogFile, STRINGIFY(TRAFFIC_INFO_FILE_PATH), NULL);
#endif
}

void traffic_info_log_stop(void)
{
#if TRAFFIC_INFO_USE_LOG
  if (LogFileIsOpen(pprzLogFile)) {
    LogClose(pprzLogFile);
  }
#endif
}

