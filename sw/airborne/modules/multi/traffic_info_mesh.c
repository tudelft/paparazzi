/*
 * Copyright (C) OpenUAS (2026)
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
 * @file modules/multi/traffic_info_mesh.c
 * @brief Optional mesh transport for traffic information.
 */

#include "modules/multi/traffic_info_mesh.h"

#if TRAFFIC_INFO_USE_MESH

#include "modules/multi/traffic_info.h"
#include "modules/multi/traffic_info_internal.h"
#include "modules/multi/traffic_info_mesh_policy.h"
#include "modules/multi/traffic_info_policy.h"

#include "generated/airframe.h"

#include "autopilot.h"
#include "modules/datalink/datalink.h"
#include "modules/datalink/telemetry.h"
#include "pprzlink/dl_protocol.h"
#include "pprzlink/messages.h"
#include "state.h"

#include <math.h>

#if PPRZLINK_DEFAULT_VER != 2
#error "TRAFFIC_INFO_USE_MESH requires PPRZLink v2 sender and class headers"
#endif

#if defined(LOW_BAT_LEVEL)
#include "modules/energy/electrical.h"
#endif

#if MESH_AUTO_TELEMETRY
#if defined(TELEMETRY_MODE_Ap_mesh) &&                                         \
    defined(TELEMETRY_MODE_Ap_mesh_manifold) &&                                \
    defined(TELEMETRY_MODE_Ap_mesh_solo)
#define MESH_AUTO_TELEMETRY_AVAILABLE 1
#define MESH_AUTO_TELEMETRY_HAS_MANIFOLD 1
#define MESH_TELEMETRY_MODE telemetry_mode_Ap
#define MESH_TELEMETRY_MODE_MESH TELEMETRY_MODE_Ap_mesh
#define MESH_TELEMETRY_MODE_MANIFOLD TELEMETRY_MODE_Ap_mesh_manifold
#define MESH_TELEMETRY_MODE_SOLO TELEMETRY_MODE_Ap_mesh_solo
#elif defined(TELEMETRY_MODE_Main_mesh) &&                                     \
    defined(TELEMETRY_MODE_Main_mesh_manifold) &&                              \
    defined(TELEMETRY_MODE_Main_mesh_solo)
#define MESH_AUTO_TELEMETRY_AVAILABLE 1
#define MESH_AUTO_TELEMETRY_HAS_MANIFOLD 1
#define MESH_TELEMETRY_MODE telemetry_mode_Main
#define MESH_TELEMETRY_MODE_MESH TELEMETRY_MODE_Main_mesh
#define MESH_TELEMETRY_MODE_MANIFOLD TELEMETRY_MODE_Main_mesh_manifold
#define MESH_TELEMETRY_MODE_SOLO TELEMETRY_MODE_Main_mesh_solo
#elif defined(TELEMETRY_MODE_Ap_mesh) && defined(TELEMETRY_MODE_Ap_mesh_solo)
#define MESH_AUTO_TELEMETRY_AVAILABLE 1
#define MESH_AUTO_TELEMETRY_HAS_MANIFOLD 0
#define MESH_TELEMETRY_MODE telemetry_mode_Ap
#define MESH_TELEMETRY_MODE_MESH TELEMETRY_MODE_Ap_mesh
#define MESH_TELEMETRY_MODE_SOLO TELEMETRY_MODE_Ap_mesh_solo
#elif defined(TELEMETRY_MODE_Main_mesh) &&                                     \
    defined(TELEMETRY_MODE_Main_mesh_solo)
#define MESH_AUTO_TELEMETRY_AVAILABLE 1
#define MESH_AUTO_TELEMETRY_HAS_MANIFOLD 0
#define MESH_TELEMETRY_MODE telemetry_mode_Main
#define MESH_TELEMETRY_MODE_MESH TELEMETRY_MODE_Main_mesh
#define MESH_TELEMETRY_MODE_SOLO TELEMETRY_MODE_Main_mesh_solo
#elif defined(TELEMETRY_MODE_Ap_mesh) ||                                       \
    defined(TELEMETRY_MODE_Ap_mesh_solo) ||                                    \
    defined(TELEMETRY_MODE_Main_mesh) ||                                       \
    defined(TELEMETRY_MODE_Main_mesh_solo)
#error                                                                         \
    "MESH_AUTO_TELEMETRY requires matching mesh and mesh_solo modes in one telemetry process"
#else
#define MESH_AUTO_TELEMETRY_AVAILABLE 0
#define MESH_AUTO_TELEMETRY_HAS_MANIFOLD 0
#endif
#else
#define MESH_AUTO_TELEMETRY_AVAILABLE 0
#define MESH_AUTO_TELEMETRY_HAS_MANIFOLD 0
#endif

struct MeshSlot {
  uint8_t ac_id;
  uint32_t last_frame;
};

struct MeshLinkState {
  uint64_t local_tx_free_ms;
  uint32_t last_emit_key;
  uint16_t reselect_count;
  uint8_t neighbours;
  bool ready;
};

/* Keep request rate equal to maximum slot consumption so configured telemetry
 * bandwidth cannot understate actual mesh traffic. */
#define TRAFFIC_INFO_ASSERT_MESH_PERIOD(period)                                \
  _Static_assert((MESH_TDMA_SUPERFRAME_MS / MESH_TDMA_MAX_REUSE) ==            \
                     (unsigned)((period) * 1000.0 + 0.5),                      \
                 "Every MESH_STATE telemetry period must equal "               \
                 "MESH_TDMA_SUPERFRAME_MS / MESH_TDMA_MAX_REUSE")

#if defined(PERIOD_MESH_STATE_Ap_0)
TRAFFIC_INFO_ASSERT_MESH_PERIOD(PERIOD_MESH_STATE_Ap_0);
#endif
#if defined(PERIOD_MESH_STATE_Ap_1)
TRAFFIC_INFO_ASSERT_MESH_PERIOD(PERIOD_MESH_STATE_Ap_1);
#endif
#if defined(PERIOD_MESH_STATE_Ap_2)
TRAFFIC_INFO_ASSERT_MESH_PERIOD(PERIOD_MESH_STATE_Ap_2);
#endif
#if defined(PERIOD_MESH_STATE_Main_0)
TRAFFIC_INFO_ASSERT_MESH_PERIOD(PERIOD_MESH_STATE_Main_0);
#endif
#if defined(PERIOD_MESH_STATE_Main_1)
TRAFFIC_INFO_ASSERT_MESH_PERIOD(PERIOD_MESH_STATE_Main_1);
#endif
#if defined(PERIOD_MESH_STATE_Main_2)
TRAFFIC_INFO_ASSERT_MESH_PERIOD(PERIOD_MESH_STATE_Main_2);
#endif

#undef TRAFFIC_INFO_ASSERT_MESH_PERIOD

#if defined(TRAFFIC_INFO_MESH_PERIODIC_FREQ)
_Static_assert(MESH_TDMA_NB_SLOTS * 1000u <
                   MESH_TDMA_SUPERFRAME_MS *
                       (unsigned)(TRAFFIC_INFO_MESH_PERIODIC_FREQ),
               "traffic_info_mesh_periodic() runs more slowly than one TDMA "
               "slot, so some owned slots would never be sampled. Raise the "
               "periodic frequency in conf/modules/traffic_info.xml or reduce "
               "MESH_TDMA_NB_SLOTS.");
#endif

static uint64_t mesh_received_ms[NB_ACS];
static bool mesh_has_valid_observation[NB_ACS];
static bool mesh_peer_seen[NB_ACS];
static uint8_t mesh_peer_clock_mode[NB_ACS];
static uint64_t mesh_peer_clock_received_ms[NB_ACS];
static uint64_t mesh_peer_holdover_started_ms[NB_ACS];

static struct MeshLinkState mesh_link;
static struct transport_tx *mesh_trans;
static struct link_device *mesh_dev;
static struct MeshClockState mesh_clock;
static bool mesh_clock_sample_valid;
static uint64_t mesh_last_network_ms;
static uint64_t mesh_last_local_ms;
static uint64_t mesh_async_next_emit_ms;
static uint64_t mesh_async_peer_until_ms;
static uint64_t mesh_async_self_until_ms;
static uint32_t mesh_async_prng;
static uint32_t mesh_recovery_frame;
static bool mesh_async_announcement_required;
static bool mesh_recovery_required;
static bool mesh_async_population_known;
static uint64_t mesh_last_position_fix_ms;
static bool mesh_position_fix_seen;

static struct MeshSlot mesh_slots[MESH_TDMA_NB_SLOTS];
static uint8_t mesh_owned[MESH_TDMA_MAX_REUSE];
static uint32_t mesh_owned_until[MESH_TDMA_MAX_REUSE];
static uint8_t mesh_owned_n;
static uint16_t mesh_frames_seen;
static uint32_t mesh_last_frame;
static bool mesh_entered;

#if MESH_AUTO_TELEMETRY_AVAILABLE
static uint64_t mesh_last_peer_ms;
static uint64_t mesh_alive_retry_ms;
static uint64_t mesh_boot_announce_ms;
static uint64_t mesh_boot_state_ms;
static uint64_t mesh_boot_state_recovery_ms;
static bool mesh_boot_announce_pending;
static bool mesh_boot_state_pending;
static bool mesh_boot_state_recovery_pending;
#endif

static uint32_t mesh_multiplex_encode(int32_t course_ddeg, int32_t gspeed_cms,
                                      int32_t climb_cms);
static void mesh_multiplex_decode(uint32_t multiplex, int16_t *course_ddeg,
                                  uint16_t *gspeed_cms, int16_t *climb_cms);
static uint8_t mesh_state_flags(uint64_t local_ms);
static uint8_t mesh_local_tx_in_flight(uint64_t now_ms);
static void mesh_async_schedule_next(uint64_t local_ms);
static uint32_t mesh_frame_of(uint64_t net_ms);
static void mesh_slot_observe(uint8_t sender, uint64_t net_ms);
static void mesh_slot_reset(uint32_t frame);

#if MESH_AUTO_TELEMETRY_AVAILABLE
#define MESH_ALIVE_RETRY_MS 30000u
#define MESH_BOOT_ANNOUNCE_WINDOW_MS 250u
#define MESH_BOOT_STATE_WINDOW_MS 160u
#define MESH_BOOT_STATE_RECOVERY_WINDOW_MS 4000u
#define MESH_MANIFOLD_ENTER_NEIGHBOURS 12u
#define MESH_MANIFOLD_EXIT_NEIGHBOURS 10u

#if FIXEDWING_FIRMWARE
#define MESH_BOOT_STATE_MSG_ID PPRZ_MSG_ID_MINIMAL_COM
#elif ROTORCRAFT_FIRMWARE
#define MESH_BOOT_STATE_MSG_ID PPRZ_MSG_ID_ROTORCRAFT_FP
#else
#define MESH_BOOT_STATE_MSG_ID 0u
#endif

static void mesh_auto_telemetry_periodic(uint64_t now_ms) {
  if (!mesh_link.ready || mesh_trans == NULL || mesh_dev == NULL) {
    return;
  }

  const bool automatic_mode = MESH_TELEMETRY_MODE == MESH_TELEMETRY_MODE_MESH ||
                              MESH_TELEMETRY_MODE == MESH_TELEMETRY_MODE_SOLO;
#if MESH_AUTO_TELEMETRY_HAS_MANIFOLD
  const bool manifold_mode =
      MESH_TELEMETRY_MODE == MESH_TELEMETRY_MODE_MANIFOLD;
#else
  const bool manifold_mode = false;
#endif
  if (automatic_mode || manifold_mode) {
    const struct mesh_mode_policy_input input = {
        .self_ping_fresh =
            datalink_gcs_self_ping_is_fresh(MESH_GCS_PING_TIMEOUT_MS),
        .other_ping_fresh =
            datalink_gcs_other_ping_is_fresh(MESH_GCS_PING_TIMEOUT_MS),
        .peer_present = mesh_link.neighbours != 0,
        .peer_quiet_ms = now_ms - mesh_last_peer_ms,
        .required_quiet_ms = MESH_SOLO_QUIET_MS};
    const bool select_solo = mesh_mode_should_use_solo(&input);
    if (select_solo) {
      MESH_TELEMETRY_MODE = MESH_TELEMETRY_MODE_SOLO;
    } else {
#if MESH_AUTO_TELEMETRY_HAS_MANIFOLD
      const bool select_manifold = mesh_mode_should_use_manifold(
          mesh_link.neighbours, manifold_mode, MESH_MANIFOLD_ENTER_NEIGHBOURS,
          MESH_MANIFOLD_EXIT_NEIGHBOURS);
      MESH_TELEMETRY_MODE = select_manifold ? MESH_TELEMETRY_MODE_MANIFOLD
                                            : MESH_TELEMETRY_MODE_MESH;
#else
      MESH_TELEMETRY_MODE = MESH_TELEMETRY_MODE_MESH;
#endif
    }
  }

  const bool any_gcs_ping_fresh =
      datalink_gcs_self_ping_is_fresh(MESH_GCS_PING_TIMEOUT_MS) ||
      datalink_gcs_other_ping_is_fresh(MESH_GCS_PING_TIMEOUT_MS);
  if (mesh_boot_announce_pending && now_ms >= mesh_boot_announce_ms &&
      mesh_local_tx_in_flight(now_ms) == 0) {
    pprz_msg_send_ALIVE(mesh_trans, mesh_dev, AC_ID, 16, MD5SUM);
    mesh_link.local_tx_free_ms = now_ms + MESH_MODEM_DRAIN_MS;
    mesh_alive_retry_ms = now_ms + MESH_ALIVE_RETRY_MS;
    mesh_boot_announce_pending = false;
    mesh_boot_state_ms =
        mesh_link.local_tx_free_ms +
        mesh_mode_boot_spread_ms((uint32_t)AC_ID ^ UINT32_C(0xa5),
                                 MESH_BOOT_STATE_WINDOW_MS);
    mesh_boot_state_recovery_ms =
        mesh_boot_state_ms + MESH_MODEM_DRAIN_MS +
        mesh_mode_boot_spread_ms((uint32_t)AC_ID ^ UINT32_C(0x5a),
                                 MESH_BOOT_STATE_RECOVERY_WINDOW_MS);
  } else if (!mesh_boot_announce_pending && !any_gcs_ping_fresh &&
             now_ms >= mesh_alive_retry_ms &&
             mesh_local_tx_in_flight(now_ms) == 0) {
    pprz_msg_send_ALIVE(mesh_trans, mesh_dev, AC_ID, 16, MD5SUM);
    mesh_link.local_tx_free_ms = now_ms + MESH_MODEM_DRAIN_MS;
    mesh_alive_retry_ms = now_ms + MESH_ALIVE_RETRY_MS;
  }

  if (mesh_boot_state_pending && !mesh_boot_announce_pending &&
      now_ms >= mesh_boot_state_ms && mesh_local_tx_in_flight(now_ms) == 0) {
    const uint8_t sent = periodic_telemetry_send_message(
        DefaultPeriodic, MESH_BOOT_STATE_MSG_ID, mesh_trans, mesh_dev);
    if (sent != 0) {
      mesh_link.local_tx_free_ms =
          now_ms + (uint64_t)sent * MESH_MODEM_DRAIN_MS;
    }
    mesh_boot_state_pending = false;
  }

  if (mesh_boot_state_recovery_pending &&
      now_ms >= mesh_boot_state_recovery_ms &&
      mesh_local_tx_in_flight(now_ms) == 0) {
    const uint8_t sent = periodic_telemetry_send_message(
        DefaultPeriodic, MESH_BOOT_STATE_MSG_ID, mesh_trans, mesh_dev);
    if (sent != 0) {
      mesh_link.local_tx_free_ms =
          now_ms + (uint64_t)sent * MESH_MODEM_DRAIN_MS;
    }
    mesh_boot_state_recovery_pending = false;
  }
}

static void mesh_auto_telemetry_note_peer(uint64_t now_ms) {
  mesh_last_peer_ms = now_ms;
  if (MESH_TELEMETRY_MODE == MESH_TELEMETRY_MODE_SOLO) {
    MESH_TELEMETRY_MODE = MESH_TELEMETRY_MODE_MESH;
  }
}
#endif

static uint64_t mesh_gps_time_ms(void) {
  const uint32_t tow_ms = gps_tow_from_sys_ticks(sys_time.nb_tick);
  uint32_t week = gps.week;
  if (tow_ms < gps.tow && gps.tow - tow_ms > 302400000u) {
    week++;
  }
  return (uint64_t)week * 604800000ULL + tow_ms;
}

static uint64_t mesh_clock_project_ms(uint64_t local_ms) {
  return mesh_clock.mode == MESH_CLOCK_ASYNC
             ? local_ms
             : mesh_clock_project(&mesh_clock, local_ms);
}

static uint32_t mesh_async_random(void) {
  uint32_t value = mesh_async_prng;
  value ^= value << 13;
  value ^= value >> 17;
  value ^= value << 5;
  mesh_async_prng = value;
  return value;
}

static void mesh_async_remix(uint64_t local_ms) {
  mesh_async_prng ^= (uint32_t)local_ms ^ (uint32_t)(local_ms >> 32) ^
                     (uint32_t)mesh_clock.anchor_network_ms ^
                     (uint32_t)(mesh_clock.anchor_network_ms >> 32);
  if (mesh_async_prng == 0) {
    mesh_async_prng = 1;
  }
  (void)mesh_async_random();
}

static uint8_t mesh_async_known_peers(void) {
  uint8_t peers = 0;
  for (uint8_t slot = 0; slot < ti_acs_idx; slot++) {
    if (ti_acs[slot].ac_id != AC_ID && mesh_peer_seen[slot]) {
      peers++;
    }
  }
  return peers;
}

static void mesh_async_schedule_next(uint64_t local_ms) {
  /* Unknown population is treated as dense. Packet loss must never make a
   * partition increase its aggregate transmission rate. */
  const struct mesh_async_interval interval = mesh_async_interval_for_peers(
      mesh_async_known_peers(), mesh_async_population_known,
      MESH_ASYNC_MIN_INTERVAL_MS, MESH_ASYNC_MAX_INTERVAL_MS);
  const uint64_t span = (uint64_t)interval.max_ms - interval.min_ms + 1u;
  const uint64_t offset = ((uint64_t)mesh_async_random() * span) >> 32;
  mesh_async_next_emit_ms = local_ms + interval.min_ms + offset;
}

static bool mesh_peer_requires_async(uint64_t local_ms) {
  /* A peer nearing holdover expiry moves the whole observed fleet to async
   * early. Mixed clock domains cannot safely share deterministic TDMA slots. */
  if (local_ms < mesh_async_peer_until_ms ||
      local_ms < mesh_async_self_until_ms) {
    return true;
  }
  for (uint8_t slot = 0; slot < ti_acs_idx; slot++) {
    if (mesh_peer_clock_mode[slot] != MESH_CLOCK_HOLDOVER) {
      continue;
    }
    if (local_ms - mesh_peer_clock_received_ms[slot] >
        MESH_ASYNC_PEER_HOLD_MS) {
      mesh_peer_clock_mode[slot] = MESH_CLOCK_RECOVERY;
      mesh_peer_holdover_started_ms[slot] = 0;
      continue;
    }
    if (local_ms - mesh_peer_holdover_started_ms[slot] >=
        MESH_CLOCK_HOLDOVER_MAX_MS - MESH_TDMA_SUPERFRAME_MS) {
      return true;
    }
  }
  return false;
}

static inline int64_t mesh_time_delta_ms(uint64_t now_ms,
                                         uint64_t previous_ms) {
  return now_ms >= previous_ms ? (int64_t)(now_ms - previous_ms)
                               : -(int64_t)(previous_ms - now_ms);
}

static uint32_t mesh_multiplex_encode(int32_t course_ddeg, int32_t gspeed_cms,
                                      int32_t climb_cms) {
  int32_t course = course_ddeg % 3600;
  if (course < 0) {
    course += 3600;
  }

  int32_t gspeed = (gspeed_cms + 5) / 10;
  if (gspeed < 0) {
    gspeed = 0;
  }
  if (gspeed > 2047) {
    gspeed = 2047;
  }

  int32_t climb_dms =
      (climb_cms >= 0) ? (climb_cms + 5) / 10 : (climb_cms - 5) / 10;
  if (climb_dms > 255) {
    climb_dms = 255;
  }
  if (climb_dms < -256) {
    climb_dms = -256;
  }

  return (((uint32_t)course & 0x0FFFu) << 20) |
         (((uint32_t)gspeed & 0x07FFu) << 9) | ((uint32_t)climb_dms & 0x01FFu);
}

static void mesh_multiplex_decode(uint32_t multiplex, int16_t *course_ddeg,
                                  uint16_t *gspeed_cms, int16_t *climb_cms) {
  const uint32_t course = (multiplex >> 20) & 0x0FFFu;
  const uint32_t gspeed = (multiplex >> 9) & 0x07FFu;
  const int32_t climb_dms = (int32_t)((multiplex & 0x01FFu) ^ 0x0100u) - 0x0100;

  *course_ddeg = (int16_t)((course < 3600u) ? course : 0u);
  *gspeed_cms = (uint16_t)(gspeed * 10);
  *climb_cms = (int16_t)(climb_dms * 10);
}

static uint8_t mesh_unified_mode(void) {
  if (autopilot_throttle_killed()) {
    return MESH_MODE_KILL;
  }

  switch (autopilot_get_mode()) {
#ifdef AP_MODE_FAILSAFE
  case AP_MODE_FAILSAFE:
    return MESH_MODE_FAILSAFE;
#endif
#ifdef AP_MODE_KILL
  case AP_MODE_KILL:
    return MESH_MODE_KILL;
#endif
#ifdef AP_MODE_HOME
  case AP_MODE_HOME:
    return MESH_MODE_HOME;
#endif
#ifdef AP_MODE_NOGPS
  case AP_MODE_NOGPS:
    return MESH_MODE_NOGPS;
#endif
#if FIXEDWING_FIRMWARE
#ifdef AP_MODE_MANUAL
  case AP_MODE_MANUAL:
    return MESH_MODE_MANUAL;
#endif
#ifdef AP_MODE_AUTO1
  case AP_MODE_AUTO1:
    return MESH_MODE_ASSISTED;
#endif
#ifdef AP_MODE_AUTO2
  case AP_MODE_AUTO2:
    return MESH_MODE_AUTO;
#endif
#else
#ifdef AP_MODE_NAV
  case AP_MODE_NAV:
    return MESH_MODE_AUTO;
#endif
#ifdef AP_MODE_GUIDED
  case AP_MODE_GUIDED:
    return MESH_MODE_AUTO;
#endif
#ifdef AP_MODE_RATE_DIRECT
  case AP_MODE_RATE_DIRECT:
    return MESH_MODE_MANUAL;
#endif
#ifdef AP_MODE_ATTITUDE_DIRECT
  case AP_MODE_ATTITUDE_DIRECT:
    return MESH_MODE_MANUAL;
#endif
#ifdef AP_MODE_RC_DIRECT
  case AP_MODE_RC_DIRECT:
    return MESH_MODE_MANUAL;
#endif
#endif
  default:
    return MESH_MODE_UNKNOWN;
  }
}

static uint8_t mesh_state_flags(uint64_t local_ms) {
  uint8_t flags = mesh_unified_mode() & MESH_FLAG_MODE_MASK;

#if !FIXEDWING_FIRMWARE
  flags |= MESH_FLAG_ROTORCRAFT;
#endif

  const struct LlaCoor_i *position = stateGetPositionLla_i();
  const struct EnuCoor_f *velocity = stateGetSpeedEnu_f();
  if (gps.fix >= GPS_FIX_3D) {
    mesh_last_position_fix_ms = local_ms;
    mesh_position_fix_seen = true;
  }
  const bool position_valid = stateIsGlobalCoordinateValid();
  const bool velocity_valid = (state.speed_status & SPEED_LOCAL_COORD) != 0 &&
                              isfinite(stateGetHorizontalSpeedDir_f()) &&
                              isfinite(stateGetHorizontalSpeedNorm_f()) &&
                              isfinite(velocity->x) && isfinite(velocity->y) &&
                              isfinite(velocity->z);
  const bool position_holdover_valid = mesh_position_holdover_valid(
      mesh_position_fix_seen, local_ms - mesh_last_position_fix_ms,
      MESH_POSITION_HOLDOVER_MS);
  if (position_holdover_valid && position_valid && velocity_valid &&
      position != NULL) {
    flags |= MESH_FLAG_POS_VALID;
  }
  if (autopilot_in_flight()) {
    flags |= MESH_FLAG_AIRBORNE;
  }
  if ((flags & MESH_FLAG_MODE_MASK) >= MESH_MODE_NOGPS) {
    flags |= MESH_FLAG_EMERGENCY;
  }
#ifdef LOW_BAT_LEVEL
  if (electrical.vsupply < LOW_BAT_LEVEL) {
    flags |= MESH_FLAG_ALERT;
  }
#endif

  return flags;
}

static uint8_t mesh_redundancy_target(uint8_t nodes, uint8_t rank,
                                      uint32_t frame, uint64_t now_ms) {
  const uint8_t flags = mesh_state_flags(now_ms);
  nodes = Max(nodes, 1u);
  uint8_t target = (uint8_t)(MESH_TDMA_FAIR_SLOTS / nodes);
  if (target < 1) {
    target = 1;
  }
  if (target > MESH_TDMA_MAX_REUSE) {
    target = MESH_TDMA_MAX_REUSE;
  } else if (nodes <= MESH_TDMA_FAIR_SLOTS && target < MESH_TDMA_MAX_REUSE) {
    const uint8_t remainder = MESH_TDMA_FAIR_SLOTS % nodes;
    const uint8_t first =
        (uint8_t)((frame / MESH_REMAINDER_EPOCH_FRAMES) % nodes);
    const uint8_t relative_rank = (uint8_t)((rank + nodes - first) % nodes);
    if (relative_rank < remainder) {
      target++;
    }
  }

  if (mesh_clock.mode == MESH_CLOCK_ASYNC ||
      (flags & MESH_FLAG_POS_VALID) == 0 ||
      (AC_ID != TRAFFIC_INFO_GCS_ID && (flags & MESH_FLAG_AIRBORNE) == 0)) {
    return 1;
  }

  if ((flags & (MESH_FLAG_EMERGENCY | MESH_FLAG_ALERT)) != 0 ||
      (flags & MESH_FLAG_MODE_MASK) == MESH_MODE_HOME) {
    return Min(MESH_TDMA_MAX_REUSE, target + 1u);
  }

  if (mesh_local_tx_in_flight(now_ms) >= MESH_CACHE_HIGH_WATER && target > 1) {
    target--;
  }
  return target;
}

static void mesh_state_emit(void) {
  const uint64_t local_ms = traffic_monotonic_time_ms();
  const struct LlaCoor_i *lla = stateGetPositionLla_i();

  uint8_t flags = mesh_state_flags(local_ms);
  uint8_t clock_mode = (uint8_t)mesh_clock.mode;
  uint32_t recovery_frame = 0;
  if (mesh_clock.mode == MESH_CLOCK_ASYNC && gps.fix >= GPS_FIX_3D &&
      !mesh_async_announcement_required) {
    clock_mode = MESH_CLOCK_RECOVERY;
    recovery_frame = mesh_recovery_frame;
  }
  int32_t lat = 0;
  int32_t lon = 0;
  int32_t alt = 0;
  uint32_t multiplex = 0;
  if ((flags & MESH_FLAG_POS_VALID) != 0) {
    lat = lla->lat;
    lon = lla->lon;
    alt = lla->alt / 10;
    multiplex = mesh_multiplex_encode(
        (int32_t)DeciDegOfRad(stateGetHorizontalSpeedDir_f()),
        (int32_t)(stateGetHorizontalSpeedNorm_f() * 100.f),
        (int32_t)(stateGetSpeedEnu_f()->z * 100.f));
  }

  struct pprzlink_msg msg;
  msg.trans = mesh_trans;
  msg.dev = mesh_dev;
  msg.sender_id = AC_ID;
  msg.receiver_id = PPRZLINK_MSG_BROADCAST;
  msg.component_id = 0;
  pprzlink_msg_send_MESH_STATE(&msg, &flags, &clock_mode, &lat, &lon, &alt,
                               &multiplex, &recovery_frame);
  if (clock_mode == MESH_CLOCK_ASYNC) {
    mesh_async_self_until_ms = local_ms + MESH_ASYNC_PEER_HOLD_MS;
    mesh_async_announcement_required = false;
    mesh_recovery_required = true;
  }
}

#if PERIODIC_TELEMETRY
static void request_mesh_state(struct transport_tx *trans,
                               struct link_device *dev) {
  mesh_trans = trans;
  mesh_dev = dev;
  mesh_link.ready = true;
}

void traffic_info_mesh_register_telemetry(void) {
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_MESH_STATE,
                              request_mesh_state);
}
#endif

static uint8_t mesh_local_tx_in_flight(uint64_t now_ms) {
  if (mesh_link.local_tx_free_ms <= now_ms) {
    return 0;
  }
  const uint64_t remaining = mesh_link.local_tx_free_ms - now_ms;
  return (uint8_t)Min(UINT64_C(255), (remaining + MESH_MODEM_DRAIN_MS - 1u) /
                                         MESH_MODEM_DRAIN_MS);
}

static void mesh_slot_reset(uint32_t frame) {
  for (uint8_t slot = 0; slot < MESH_TDMA_NB_SLOTS; slot++) {
    mesh_slots[slot].ac_id = MESH_SLOT_FREE;
    mesh_slots[slot].last_frame = frame;
  }
  mesh_owned[0] = 0;
  mesh_owned_until[0] = frame + MESH_PRIMARY_HOLD_MIN;
  mesh_owned_n = 1;
  mesh_entered = false;
  mesh_frames_seen = 0;
  mesh_last_frame = frame;
  mesh_link.neighbours = 0;
}

static uint32_t mesh_frame_of(uint64_t net_ms) {
  return (uint32_t)(net_ms / MESH_TDMA_SUPERFRAME_MS);
}

static uint8_t mesh_slot_of(uint64_t net_ms) {
  const uint32_t frame_ms = (uint32_t)(net_ms % MESH_TDMA_SUPERFRAME_MS);
  return (uint8_t)(((uint64_t)frame_ms * MESH_TDMA_NB_SLOTS) /
                   MESH_TDMA_SUPERFRAME_MS);
}

static bool mesh_slot_is_stale(uint8_t slot, uint32_t frame) {
  return frame - mesh_slots[slot].last_frame > MESH_SLOT_AGE_FRAMES;
}

static bool mesh_slot_free(uint8_t slot, uint32_t frame) {
  return mesh_slots[slot].ac_id == MESH_SLOT_FREE ||
         mesh_slots[slot].ac_id == AC_ID || mesh_slot_is_stale(slot, frame);
}

static bool mesh_slot_quiet(uint8_t slot, uint32_t frame) {
  return mesh_slots[slot].ac_id == MESH_SLOT_FREE &&
         frame - mesh_slots[slot].last_frame > (2 * MESH_SLOT_AGE_FRAMES);
}

static bool mesh_owns(uint8_t slot) {
  /* Simultaneous starters otherwise see empty maps, claim aggressively, and
   * become mutually unable to observe and repair their collisions. */
  if (mesh_frames_seen < MESH_ENTRY_FRAMES) {
    return false;
  }
  for (uint8_t index = 0; index < mesh_owned_n; index++) {
    if (mesh_owned[index] == slot) {
      return true;
    }
  }
  return false;
}

static void mesh_slot_observe(uint8_t sender, uint64_t net_ms) {
  if (sender == AC_ID || !traffic_info_id_valid(sender)) {
    return;
  }
  const uint8_t slot = mesh_slot_of(net_ms);
  const uint32_t frame = mesh_frame_of(net_ms);

  if (mesh_slots[slot].ac_id == MESH_SLOT_FREE ||
      mesh_slots[slot].ac_id == sender || mesh_slot_is_stale(slot, frame)) {
    mesh_slots[slot].ac_id = sender;
    mesh_slots[slot].last_frame = frame;
  }
}

static uint32_t mesh_slot_hash(uint32_t value) {
  value ^= value >> 16;
  value *= 0x7FEB352Du;
  value ^= value >> 15;
  value *= 0x846CA68Bu;
  return value ^ (value >> 16);
}

static uint8_t mesh_pick_slot(uint32_t frame) {
  const uint32_t mixed =
      mesh_slot_hash((uint32_t)AC_ID |
                     ((uint32_t)mesh_link.reselect_count << 8) | (frame << 16));
  const uint8_t start = (uint8_t)(mixed % MESH_TDMA_NB_SLOTS);
  for (uint8_t offset = 0; offset < MESH_TDMA_NB_SLOTS; offset++) {
    const uint8_t slot = (uint8_t)((start + offset) % MESH_TDMA_NB_SLOTS);
    if (mesh_slot_free(slot, frame) && !mesh_owns(slot)) {
      return slot;
    }
  }
  return start;
}

static uint32_t mesh_lease_expiry(uint32_t frame) {
  const uint32_t random =
      mesh_slot_hash((uint32_t)AC_ID |
                     ((uint32_t)mesh_link.reselect_count << 8) | (frame << 16));
  return frame + MESH_SLOT_HOLD_MIN + random % MESH_SLOT_HOLD_SPAN;
}

static uint32_t mesh_primary_expiry(uint32_t frame) {
  const uint32_t random = mesh_slot_hash(
      (uint32_t)AC_ID | ((uint32_t)mesh_link.reselect_count << 8) |
      (frame << 16) | 0x80000000u);
  return frame + MESH_PRIMARY_HOLD_MIN + random % MESH_PRIMARY_HOLD_SPAN;
}

static bool mesh_expansion_turn(uint32_t frame, uint8_t nodes) {
  uint8_t rank = 0;
  for (uint8_t slot = 0; slot < MESH_TDMA_NB_SLOTS; slot++) {
    const uint8_t id = mesh_slots[slot].ac_id;
    if (id != MESH_SLOT_FREE && id != AC_ID && id < AC_ID) {
      bool duplicate = false;
      for (uint8_t previous = 0; previous < slot; previous++) {
        if (mesh_slots[previous].ac_id == id) {
          duplicate = true;
          break;
        }
      }
      if (!duplicate) {
        rank++;
      }
    }
  }
  return (uint8_t)(frame % (nodes ? nodes : 1)) == rank;
}

static void mesh_slot_maintain(uint32_t frame, uint64_t now_ms) {
  /* Growth stays slower than conflict detection while contraction is
   * immediate, keeping membership churn self-stabilizing. */
  uint8_t nodes = 1;
  uint8_t rank = 0;
  uint8_t seen[MESH_TDMA_NB_SLOTS];
  uint8_t seen_n = 0;
  for (uint8_t slot = 0; slot < MESH_TDMA_NB_SLOTS; slot++) {
    if (mesh_slots[slot].ac_id == MESH_SLOT_FREE) {
      continue;
    }
    if (mesh_slot_is_stale(slot, frame)) {
      mesh_slots[slot].ac_id = MESH_SLOT_FREE;
      continue;
    }
    if (mesh_slots[slot].ac_id == AC_ID) {
      continue;
    }
    bool duplicate = false;
    for (uint8_t index = 0; index < seen_n; index++) {
      if (seen[index] == mesh_slots[slot].ac_id) {
        duplicate = true;
        break;
      }
    }
    if (!duplicate) {
      seen[seen_n++] = mesh_slots[slot].ac_id;
      nodes++;
      if (mesh_slots[slot].ac_id < AC_ID) {
        rank++;
      }
    }
  }
  mesh_link.neighbours = (uint8_t)(nodes - 1);

  if (mesh_frames_seen == MESH_ENTRY_FRAMES && !mesh_entered) {
    mesh_entered = true;
    mesh_owned[0] = mesh_pick_slot(frame);
    mesh_owned_until[0] = mesh_primary_expiry(frame);
    mesh_owned_n = 1;
  }

  if ((int32_t)(frame - mesh_owned_until[0]) >= 0) {
    mesh_owned[0] = mesh_pick_slot(frame);
    mesh_owned_until[0] = mesh_primary_expiry(frame);
    mesh_link.reselect_count++;
  }

  const uint8_t owner = mesh_slots[mesh_owned[0]].ac_id;
  if (owner != MESH_SLOT_FREE && owner != AC_ID &&
      !mesh_slot_is_stale(mesh_owned[0], frame) && AC_ID > owner) {
    mesh_owned[0] = mesh_pick_slot(frame);
    mesh_link.reselect_count++;
  }

  uint8_t keep = 1;
  for (uint8_t index = 1; index < mesh_owned_n; index++) {
    const uint8_t slot = mesh_owned[index];
    const bool expired = (int32_t)(frame - mesh_owned_until[index]) >= 0;
    if (mesh_slot_free(slot, frame) && !expired) {
      mesh_owned[keep] = slot;
      mesh_owned_until[keep] = mesh_owned_until[index];
      keep++;
    }
  }
  mesh_owned_n = keep;

  uint8_t target = mesh_redundancy_target(nodes, rank, frame, now_ms);
  if (mesh_frames_seen < MESH_ENTRY_FRAMES) {
    target = 1;
  }

  if (mesh_owned_n > target) {
    mesh_owned_n = target;
  } else if (mesh_owned_n < target && mesh_expansion_turn(frame, nodes)) {
    const uint8_t start = (uint8_t)((mesh_owned[0] + MESH_TDMA_NB_SLOTS / 2u) %
                                    MESH_TDMA_NB_SLOTS);
    for (uint8_t offset = 0; offset < MESH_TDMA_NB_SLOTS; offset++) {
      const uint8_t slot = (uint8_t)((start + offset) % MESH_TDMA_NB_SLOTS);
      if (mesh_slot_quiet(slot, frame) && !mesh_owns(slot)) {
        mesh_owned[mesh_owned_n] = slot;
        mesh_owned_until[mesh_owned_n] = mesh_lease_expiry(frame);
        mesh_owned_n++;
        break;
      }
    }
  }
}

void traffic_info_mesh_periodic(void) {
  /* Snapshot at actual TDMA emission time so scheduling delay adds latency
   * without adding avoidable state age. */
  const uint64_t local_ms = traffic_monotonic_time_ms();
  const enum MeshClockMode previous_mode = mesh_clock.mode;
  const bool gps_valid = gps.fix >= GPS_FIX_3D;
  const uint32_t gps_frame = gps_valid ? mesh_frame_of(mesh_gps_time_ms()) : 0;
  const bool local_holdover_expiring = mesh_clock_holdover_expiring(
      &mesh_clock, local_ms, MESH_CLOCK_HOLDOVER_MAX_MS,
      MESH_TDMA_SUPERFRAME_MS);
  const bool denied_fallback = local_holdover_expiring ||
                               mesh_peer_requires_async(local_ms) ||
                               mesh_async_announcement_required;
  if (mesh_clock.mode == MESH_CLOCK_ASYNC && gps_valid &&
      !mesh_async_announcement_required && mesh_recovery_frame == 0) {
    if (mesh_recovery_required) {
      mesh_recovery_frame = mesh_clock_next_recovery_frame(
          gps_frame + MESH_RECOVERY_MIN_LEAD_FRAMES,
          MESH_RECOVERY_EPOCH_FRAMES);
    }
  }
  if (denied_fallback && (local_holdover_expiring || !gps_valid)) {
    mesh_recovery_frame = 0;
  }
  const bool recovery_pending =
      mesh_recovery_frame != 0 &&
      !mesh_clock_frame_reached(gps_frame, mesh_recovery_frame);
  const bool force_async = denied_fallback || recovery_pending;
  bool reset_slots = false;
  const uint64_t net_ms = mesh_clock_update(
      &mesh_clock, local_ms, gps_valid, gps_valid ? mesh_gps_time_ms() : 0,
      force_async, MESH_CLOCK_HOLDOVER_MAX_MS, MESH_CLOCK_ACQUIRE_MS,
      MESH_CLOCK_STEP_MAX_MS, &reset_slots);
  const uint32_t frame = mesh_frame_of(net_ms);
  const uint8_t slot = mesh_slot_of(net_ms);

  if (mesh_clock.mode != previous_mode) {
    mesh_clock_sample_valid = false;
    if (mesh_clock.mode == MESH_CLOCK_ASYNC) {
      if (previous_mode != MESH_CLOCK_ASYNC) {
        mesh_async_population_known = mesh_frames_seen >= MESH_ENTRY_FRAMES;
      }
      mesh_async_announcement_required = true;
      mesh_async_remix(local_ms);
      mesh_async_schedule_next(local_ms);
    } else if (mesh_clock.mode == MESH_CLOCK_GPS) {
      mesh_recovery_frame = 0;
      mesh_recovery_required = false;
    }
  }
  if (reset_slots) {
    mesh_slot_reset(frame);
#if MESH_AUTO_TELEMETRY_AVAILABLE
    mesh_auto_telemetry_periodic(local_ms);
#endif
    return;
  }
  if (mesh_clock.mode == MESH_CLOCK_GPS && mesh_clock_sample_valid) {
    const int64_t network_elapsed =
        mesh_time_delta_ms(net_ms, mesh_last_network_ms);
    const int64_t local_elapsed =
        mesh_time_delta_ms(local_ms, mesh_last_local_ms);
    const int64_t correction = network_elapsed - local_elapsed;
    if (correction > MESH_CLOCK_STEP_MAX_MS ||
        correction < -MESH_CLOCK_STEP_MAX_MS) {
      mesh_last_network_ms = net_ms;
      mesh_last_local_ms = local_ms;
      mesh_slot_reset(frame);
#if MESH_AUTO_TELEMETRY_AVAILABLE
      mesh_auto_telemetry_periodic(local_ms);
#endif
      return;
    }
  }
  mesh_last_network_ms = net_ms;
  mesh_last_local_ms = local_ms;
  mesh_clock_sample_valid = mesh_clock.mode == MESH_CLOCK_GPS;
  if (mesh_clock.mode != MESH_CLOCK_ASYNC && mesh_frames_seen > 0 &&
      frame != mesh_last_frame && frame != mesh_last_frame + 1u) {
    mesh_slot_reset(frame);
#if MESH_AUTO_TELEMETRY_AVAILABLE
    mesh_auto_telemetry_periodic(local_ms);
#endif
    return;
  }

  if (mesh_clock.mode != MESH_CLOCK_ASYNC && frame != mesh_last_frame) {
    mesh_last_frame = frame;
    if (mesh_frames_seen < 0xFFFF) {
      mesh_frames_seen++;
    }
    mesh_slot_maintain(frame, local_ms);
  }

  if (!mesh_link.ready || mesh_trans == NULL || mesh_dev == NULL) {
#if MESH_AUTO_TELEMETRY_AVAILABLE
    mesh_auto_telemetry_periodic(local_ms);
#endif
    return;
  }
  if (mesh_clock.mode == MESH_CLOCK_ASYNC) {
    if (local_ms < mesh_async_next_emit_ms) {
#if MESH_AUTO_TELEMETRY_AVAILABLE
      mesh_auto_telemetry_periodic(local_ms);
#endif
      return;
    }
    if (mesh_local_tx_in_flight(local_ms) >= MESH_CACHE_HIGH_WATER) {
      mesh_async_schedule_next(local_ms);
#if MESH_AUTO_TELEMETRY_AVAILABLE
      mesh_auto_telemetry_periodic(local_ms);
#endif
      return;
    }
    mesh_state_emit();
    mesh_link.local_tx_free_ms =
        Max(mesh_link.local_tx_free_ms, local_ms) + MESH_MODEM_DRAIN_MS;
    mesh_async_schedule_next(local_ms);
#if MESH_AUTO_TELEMETRY_AVAILABLE
    mesh_auto_telemetry_periodic(local_ms);
#endif
    return;
  }
  if (!mesh_owns(slot)) {
#if MESH_AUTO_TELEMETRY_AVAILABLE
    mesh_auto_telemetry_periodic(local_ms);
#endif
    return;
  }

  const uint32_t key = frame * MESH_TDMA_NB_SLOTS + slot;
  if (key == mesh_link.last_emit_key) {
#if MESH_AUTO_TELEMETRY_AVAILABLE
    mesh_auto_telemetry_periodic(local_ms);
#endif
    return;
  }

  if (mesh_local_tx_in_flight(local_ms) >= MESH_CACHE_HIGH_WATER) {
#if MESH_AUTO_TELEMETRY_AVAILABLE
    mesh_auto_telemetry_periodic(local_ms);
#endif
    return;
  }

  mesh_state_emit();
  mesh_link.last_emit_key = key;
  mesh_link.local_tx_free_ms =
      Max(mesh_link.local_tx_free_ms, local_ms) + MESH_MODEM_DRAIN_MS;
#if MESH_AUTO_TELEMETRY_AVAILABLE
  mesh_auto_telemetry_periodic(local_ms);
#endif
}

void traffic_info_mesh_init(void) {
  mesh_slot_reset(0);
  for (uint8_t slot = 0; slot < NB_ACS; slot++) {
    mesh_received_ms[slot] = 0;
    mesh_has_valid_observation[slot] = false;
    mesh_peer_seen[slot] = false;
    mesh_peer_clock_mode[slot] = MESH_CLOCK_RECOVERY;
    mesh_peer_clock_received_ms[slot] = 0;
    mesh_peer_holdover_started_ms[slot] = 0;
  }
  mesh_clock_init(&mesh_clock);
  mesh_clock_sample_valid = false;
  mesh_last_network_ms = 0;
  mesh_last_local_ms = 0;
  mesh_async_prng = 0x9E3779B9u ^ ((uint32_t)AC_ID * 2654435761u);
  if (mesh_async_prng == 0) {
    mesh_async_prng = 1;
  }
  mesh_async_schedule_next(traffic_monotonic_time_ms());
  mesh_async_peer_until_ms = 0;
  mesh_async_self_until_ms = 0;
  mesh_recovery_frame = 0;
  mesh_async_announcement_required = false;
  mesh_recovery_required = false;
  mesh_async_population_known = false;
  mesh_last_position_fix_ms = 0;
  mesh_position_fix_seen = false;
  mesh_link.local_tx_free_ms = traffic_monotonic_time_ms();
  mesh_link.last_emit_key = UINT32_MAX;
#if MESH_AUTO_TELEMETRY_AVAILABLE
  MESH_TELEMETRY_MODE = MESH_TELEMETRY_MODE_MESH;
  mesh_last_peer_ms = traffic_monotonic_time_ms();
  mesh_alive_retry_ms = UINT64_MAX;
  mesh_boot_announce_ms =
      traffic_monotonic_time_ms() +
      mesh_mode_boot_spread_ms(AC_ID, MESH_BOOT_ANNOUNCE_WINDOW_MS);
  mesh_boot_state_ms = UINT64_MAX;
  mesh_boot_state_recovery_ms = UINT64_MAX;
  mesh_boot_announce_pending = true;
  mesh_boot_state_pending = MESH_BOOT_STATE_MSG_ID != 0u;
  mesh_boot_state_recovery_pending = MESH_BOOT_STATE_MSG_ID != 0u;
#endif
}

void traffic_info_mesh_reset_slot(uint8_t slot) {
  if (slot < NB_ACS) {
    mesh_received_ms[slot] = 0;
    mesh_has_valid_observation[slot] = false;
    mesh_peer_seen[slot] = false;
    mesh_peer_clock_mode[slot] = MESH_CLOCK_RECOVERY;
    mesh_peer_clock_received_ms[slot] = 0;
    mesh_peer_holdover_started_ms[slot] = 0;
  }
}

bool traffic_info_mesh_source_active(uint8_t slot) {
  return slot < NB_ACS &&
         bit_is_set(ti_acs[slot].status, AC_INFO_SOURCE_MESH) &&
         mesh_has_valid_observation[slot] &&
         traffic_monotonic_time_ms() - mesh_received_ms[slot] <=
             TRAFFIC_INFO_MESH_DROP_MS;
}

bool traffic_info_mesh_clear_source(uint8_t slot) {
  if (slot >= NB_ACS || !bit_is_set(ti_acs[slot].status, AC_INFO_SOURCE_MESH)) {
    return false;
  }
  ClearBit(ti_acs[slot].status, AC_INFO_SOURCE_MESH);
  mesh_has_valid_observation[slot] = false;
  return true;
}

bool traffic_info_mesh_parse_telemetry(uint8_t sender_id, uint8_t msg_id) {
#if MESH_AUTO_TELEMETRY_AVAILABLE
  if (msg_id == DL_ALIVE) {
    if (mesh_mode_is_peer_sender(sender_id, AC_ID)) {
      mesh_auto_telemetry_note_peer(traffic_monotonic_time_ms());
    }
    return true;
  }
#else
  (void)sender_id;
  (void)msg_id;
#endif
  return false;
}

enum traffic_info_mesh_parse_result
traffic_info_mesh_parse_state(uint8_t *buf, uint8_t sender_id, uint32_t *itow) {
  /* Flooded broadcasts may return to their originator; local estimator state
   * remains authoritative for our own identity. */
  if (sender_id == AC_ID) {
    return TRAFFIC_INFO_MESH_PARSE_HANDLED;
  }
  if (!traffic_info_id_valid(sender_id)) {
    return TRAFFIC_INFO_MESH_PARSE_REJECTED;
  }

  const uint8_t flags = DL_MESH_STATE_flags(buf);
  int32_t altitude_mm = 0;
  if ((flags & MESH_FLAG_POS_VALID) != 0 &&
      !traffic_info_cm_to_mm(DL_MESH_STATE_alt(buf), &altitude_mm)) {
    return TRAFFIC_INFO_MESH_PARSE_REJECTED;
  }
#if MESH_AUTO_TELEMETRY_AVAILABLE
  mesh_auto_telemetry_note_peer(traffic_monotonic_time_ms());
#endif
  *itow = gps_tow_from_sys_ticks(sys_time.nb_tick);

  const uint8_t clock_mode = DL_MESH_STATE_clock_mode(buf);
  const uint32_t recovery_frame = DL_MESH_STATE_recovery_frame(buf);
  const uint64_t local_ms = traffic_monotonic_time_ms();
  if (mesh_clock.mode != MESH_CLOCK_ASYNC &&
      clock_mode <= MESH_CLOCK_HOLDOVER) {
    mesh_slot_observe(sender_id, mesh_clock_project_ms(local_ms));
  }
  const uint8_t slot = ti_acs_slot(sender_id);
  if (slot == TI_ACS_NONE) {
    return TRAFFIC_INFO_MESH_PARSE_HANDLED_LOG;
  }
  traffic_info_internal_note_slot_activity(slot, local_ms);
  if (clock_mode <= MESH_CLOCK_RECOVERY) {
    mesh_peer_seen[slot] = true;
    if (clock_mode == MESH_CLOCK_ASYNC) {
      mesh_async_peer_until_ms = local_ms + MESH_ASYNC_PEER_HOLD_MS;
      mesh_recovery_frame = 0;
      mesh_recovery_required = true;
    } else if (clock_mode == MESH_CLOCK_RECOVERY && gps.fix >= GPS_FIX_3D) {
      const uint32_t current_frame = mesh_frame_of(mesh_gps_time_ms());
      const uint32_t safe_recovery_frame = mesh_clock_sanitize_recovery_frame(
          current_frame, recovery_frame, MESH_RECOVERY_EPOCH_FRAMES,
          MESH_RECOVERY_MIN_LEAD_FRAMES);
      if (mesh_clock_frame_is_later(safe_recovery_frame, mesh_recovery_frame)) {
        mesh_recovery_frame = safe_recovery_frame;
        mesh_recovery_required = true;
      }
    }
    if (clock_mode == MESH_CLOCK_HOLDOVER &&
        mesh_peer_clock_mode[slot] != MESH_CLOCK_HOLDOVER) {
      mesh_peer_holdover_started_ms[slot] = local_ms;
      mesh_recovery_frame = 0;
      mesh_recovery_required = true;
    } else if (clock_mode != MESH_CLOCK_HOLDOVER) {
      mesh_peer_holdover_started_ms[slot] = 0;
    }
    mesh_peer_clock_mode[slot] = clock_mode;
    mesh_peer_clock_received_ms[slot] = local_ms;
  }

  if ((flags & MESH_FLAG_POS_VALID) != 0) {
    mesh_received_ms[slot] = traffic_monotonic_time_ms();
    mesh_has_valid_observation[slot] = true;
    traffic_info_touch(slot);
    int16_t course;
    uint16_t gspeed;
    int16_t climb;
    mesh_multiplex_decode(DL_MESH_STATE_multiplex_speed(buf), &course, &gspeed,
                          &climb);
    traffic_info_internal_store_mesh(slot, DL_MESH_STATE_lat(buf),
                                     DL_MESH_STATE_lon(buf), altitude_mm,
                                     course, gspeed, climb, *itow);
    return TRAFFIC_INFO_MESH_PARSE_HANDLED_LOG;
  }

  /* Preserve presence without refreshing stale kinematics or displacing a
   * complete legacy fallback observation. */
  traffic_info_internal_store_mesh_heartbeat(slot, *itow);
  return TRAFFIC_INFO_MESH_PARSE_HANDLED;
}

bool traffic_info_is_mesh_track(uint8_t ac_id) {
  if (!traffic_info_id_valid(ac_id)) {
    return false;
  }
  const uint8_t slot = ti_acs_id[ac_id];
  return slot < ti_acs_idx && ti_acs[slot].ac_id == ac_id &&
         bit_is_set(ti_acs[slot].status, AC_INFO_SOURCE_MESH);
}

bool traffic_info_get_mesh_snapshot(uint8_t ac_id, uint32_t max_prediction_ms,
                                    struct EnuCoor_f *position,
                                    struct EnuCoor_f *velocity,
                                    uint32_t *age_ms) {
  if (position == NULL || velocity == NULL || age_ms == NULL ||
      !traffic_info_id_valid(ac_id)) {
    return false;
  }
  const uint8_t slot = ti_acs_id[ac_id];
  if (!traffic_info_is_mesh_track(ac_id) ||
      !bit_is_set(ti_acs[slot].status, AC_INFO_POS_LLA_I) ||
      !bit_is_set(ti_acs[slot].status, AC_INFO_VEL_LOCAL_F)) {
    return false;
  }

  *position = *acInfoGetPositionEnu_f(ac_id);
  if (!bit_is_set(ti_acs[slot].status, AC_INFO_POS_ENU_F)) {
    return false;
  }
  *velocity = *acInfoGetVelocityEnu_f(ac_id);
  if (!isfinite(position->x) || !isfinite(position->y) ||
      !isfinite(position->z) || !isfinite(velocity->x) ||
      !isfinite(velocity->y) || !isfinite(velocity->z)) {
    return false;
  }
  const uint64_t age = traffic_monotonic_time_ms() - mesh_received_ms[slot];
  *age_ms = age > UINT32_MAX ? UINT32_MAX : (uint32_t)age;

  const float prediction_s = Min(*age_ms, max_prediction_ms) * 0.001f;
  position->x += velocity->x * prediction_s;
  position->y += velocity->y * prediction_s;
  position->z += velocity->z * prediction_s;
  return true;
}

bool traffic_info_get_mesh_valid_age(uint8_t ac_id, uint32_t *age_ms) {
  if (age_ms == NULL || !traffic_info_is_mesh_track(ac_id)) {
    return false;
  }
  const uint8_t slot = ti_acs_id[ac_id];
  if (!mesh_has_valid_observation[slot]) {
    return false;
  }
  const uint64_t age = traffic_monotonic_time_ms() - mesh_received_ms[slot];
  *age_ms = age > UINT32_MAX ? UINT32_MAX : (uint32_t)age;
  return true;
}

#endif /* TRAFFIC_INFO_USE_MESH */