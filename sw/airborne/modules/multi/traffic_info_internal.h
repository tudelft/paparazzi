/*
 * Copyright (C) OpenUAS (2026)
 *
 * This file is part of paparazzi
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 */

/**
 * @file modules/multi/traffic_info_internal.h
 * @brief Narrow services shared by traffic-info source implementations.
 */

#ifndef TRAFFIC_INFO_INTERNAL_H
#define TRAFFIC_INFO_INTERNAL_H

#include "modules/multi/traffic_info.h"

/** Return the module's wrapping-safe monotonic timestamp. */
extern uint64_t traffic_monotonic_time_ms(void);

#if TRAFFIC_INFO_USE_MESH
/** Result of delegated MESH_STATE parsing.
 *
 * The distinction preserves legacy receive logging: valid kinematics continue
 * through the common log tail, while rejected frames fail and presence-only
 * heartbeats return without pretending that usable traffic data was logged.
 */
enum traffic_info_mesh_parse_result {
  TRAFFIC_INFO_MESH_PARSE_REJECTED,   /**< Invalid frame; report failure. */
  TRAFFIC_INFO_MESH_PARSE_HANDLED,    /**< Handled; return without logging. */
  TRAFFIC_INFO_MESH_PARSE_HANDLED_LOG /**< Handled; run common receive log. */
};

/** Initialize all mesh sidecars after the shared table has been initialized. */
extern void traffic_info_mesh_init(void);
/** Register the deferred MESH_STATE telemetry transport callback. */
extern void traffic_info_mesh_register_telemetry(void);
/** Clear every mesh sidecar before a shared slot receives a new owner. */
extern void traffic_info_mesh_reset_slot(uint8_t slot);
/** Handle mesh-specific telemetry presence messages such as ALIVE. */
extern bool traffic_info_mesh_parse_telemetry(uint8_t sender_id,
                                              uint8_t msg_id);
/** Parse MESH_STATE while leaving shared logging policy to traffic_info.c. */
extern enum traffic_info_mesh_parse_result
traffic_info_mesh_parse_state(uint8_t *buf, uint8_t sender_id, uint32_t *itow);
/** Return whether fresh valid mesh kinematics still block legacy replacement. */
extern bool traffic_info_mesh_source_active(uint8_t slot);
/** Drop mesh ownership and report whether an ownership bit was cleared. */
extern bool traffic_info_mesh_clear_source(uint8_t slot);

/** Update reclamation activity for a valid mesh presence observation. */
extern void traffic_info_internal_note_slot_activity(uint8_t slot,
                                                     uint64_t received_ms);
/** Store decoded mesh kinematics in the shared representation. */
extern void traffic_info_internal_store_mesh(uint8_t slot, int32_t lat,
                                             int32_t lon, int32_t altitude_mm,
                                             int16_t course, uint16_t gspeed,
                                             int16_t climb, uint32_t itow);
/** Store presence without replacing a complete legacy fallback track. */
extern void traffic_info_internal_store_mesh_heartbeat(uint8_t slot,
                                                       uint32_t itow);
#endif

#endif /* TRAFFIC_INFO_INTERNAL_H */