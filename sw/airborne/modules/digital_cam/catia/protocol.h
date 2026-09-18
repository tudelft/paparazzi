/*
 * Copyright (C) 2014 OpenUAS
 *
 * This file is part of paparazzi.
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
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

/**
 * @file protocol.h
 * @brief CATIA's byte-stream UART framing, messages, and wire-compatible payload layouts.
 * @details All multi-byte fields use the historic CATIA native layout and must remain wire
 * compatible with deployed flight-controller firmware. Callers should treat the unions as
 * serialization storage, not a portable network byte-order protocol.
 *
 * |STX|length|... payload=(length-4) bytes ...|Checksum A|Checksum B|
 *
 * where checksum is computed over length and payload:
 * @code
 * catia_ck_a = catia_ck_b = length
 * for each byte b in payload
 *     catia_ck_a += b;
 *     catia_ck_b += catia_ck_a;
 * @endcode
 */

#ifndef CATIA_TRANSPORT_H
#define CATIA_TRANSPORT_H

#include <inttypes.h>
#include <stdbool.h>

/////////////////////////////////////////////////////////////////////
// MESSAGES

#define CATIA_SHOOT              1
#define CATIA_SHOOT_MSG_SIZE     (4*10)

// 7 * 4 bytes int32_t
// nr, lat, lon, h, phi, theta, psi

union dc_shot_union {
  struct {
    int32_t nr;
    int32_t lat;
    int32_t lon;
    int32_t alt;
    int32_t phi;
    int32_t theta;
    int32_t psi;
    int32_t vground;
    int32_t course;
    int32_t groundalt;
  } data;
  uint8_t bin[CATIA_SHOOT_MSG_SIZE];
  int32_t i[10];
};

#define CATIA_BUFFER_EMPTY       2

// 0 bytes payload: null

#define CATIA_PAYLOAD            3
#define CATIA_PAYLOAD_MSG_SIZE   70


// 72 bytes

#define CATIA_STATUS             4
#define CATIA_STATUS_MSG_SIZE    (4*2)

// 4*2 bytes
union catia_status_union {
  struct catia_status_struct {
    uint16_t cpu;
    uint16_t threads;
    uint16_t shots;
    uint16_t extra;
  } data;
  uint8_t bin[CATIA_STATUS_MSG_SIZE];
};

/////////////////////////////////////////////////////////////////////
// CAMERA SELECTION (additive; legacy CATIA_SHOOT means CATIA_CAMERA_ALL)

#define CATIA_CAMERA_ALL         0
#define CATIA_CAMERA_CHDK        1
#define CATIA_CAMERA_AICAM       2
#define CATIA_CAMERA_LWIRCAM     3
#define CATIA_CAMERA_EARCAM      4

#define CATIA_CAMERA_MASK_NONE   0x00U
#define CATIA_CAMERA_MASK_CHDK   0x01U
#define CATIA_CAMERA_MASK_AICAM  0x02U
#define CATIA_CAMERA_MASK_LWIR   0x04U
#define CATIA_CAMERA_MASK_EAR    0x08U
#define CATIA_CAMERA_MASK_ALL    0xFFU
#define CATIA_CAMERA_MASK_SUPPORTED 0x0FU

#define CATIA_SHOOT_MASK          12
#define CATIA_SHOOT_MASK_MSG_SIZE  (CATIA_SHOOT_MSG_SIZE + 4)

union dc_shot_mask_union {
  struct {
    union dc_shot_union shot;
    uint32_t camera_mask;
  } data;
  uint8_t bin[CATIA_SHOOT_MASK_MSG_SIZE];
};

#define CATIA_SHOOT_TARGETED          5
#define CATIA_SHOOT_TARGETED_MSG_SIZE (CATIA_SHOOT_MSG_SIZE + 4)

union dc_shot_targeted_union {
  struct {
    union dc_shot_union shot;
    int32_t camera_id;
  } data;
  uint8_t bin[CATIA_SHOOT_TARGETED_MSG_SIZE];
};

#define CATIA_STOP_TARGETED           6
#define CATIA_STOP_TARGETED_MSG_SIZE  4
// camera_id low byte selects the camera; CATIA_STOP_FLAG_KEEP requests an
// intermediate result while the sample session continues (refinement stages).
#define CATIA_STOP_FLAG_KEEP          0x100

#define CATIA_EAR_RESULT              7
#define CATIA_EAR_RESULT_MSG_SIZE     (4*8)

#define CATIA_EAR_RESULT_INVALID      0
#define CATIA_EAR_RESULT_VALID        1

union catia_ear_result_union {
  struct {
    int32_t status;        // CATIA_EAR_RESULT_VALID or CATIA_EAR_RESULT_INVALID
    int32_t lat;           // 1e7 deg
    int32_t lon;           // 1e7 deg
    int32_t agl_mm;        // listening height above the spot (median AGL of the loudest windows)
    int32_t alt_mm;        // ellipsoid altitude of the spot (ground), same reference as dc_shot alt
    int32_t level_cdb;     // loudest level in centi-dB
    int32_t confidence;    // 0..1000
    int32_t sample_count;
  } data;
  uint8_t bin[CATIA_EAR_RESULT_MSG_SIZE];
};

/////////////////////////////////////////////////////////////////////
// SENDING

#define CATIA_POSE_SAMPLE             8
#define CATIA_POSE_SAMPLE_MSG_SIZE    100
#define CATIA_POSE_SAMPLE_GPS_PRESENT 1U

union catia_pose_sample_union {
  struct {
    uint32_t sequence;
    uint32_t sample_begin_us;
    uint32_t sample_end_us;
    union dc_shot_union shot;
    int32_t velocity_north_bfp;
    int32_t velocity_east_bfp;
    int32_t velocity_down_bfp;
    uint32_t gps_tow_ms;
    uint32_t gps_week;
    uint32_t gps_hacc_cm;
    uint32_t gps_vacc_cm;
    uint32_t gps_sacc_cm_s;
    uint32_t gps_fix;
    uint32_t gps_num_sv;
    uint32_t gps_valid_fields;
    uint32_t flags;
  } data;
  uint8_t bin[CATIA_POSE_SAMPLE_MSG_SIZE];
};

#define CATIA_CLOCK_REQUEST           9
#define CATIA_CLOCK_REQUEST_MSG_SIZE  8
#define CATIA_CLOCK_REPLY             10
#define CATIA_CLOCK_REPLY_MSG_SIZE    16
#define CATIA_POSE_CLOCKED            11
#define CATIA_POSE_CLOCKED_MSG_SIZE   (CATIA_POSE_SAMPLE_MSG_SIZE + 8)

union catia_clock_request_union {
  struct {
    uint32_t token_low;
    uint32_t token_high;
  } data;
  uint8_t bin[CATIA_CLOCK_REQUEST_MSG_SIZE];
};

union catia_clock_reply_union {
  struct {
    union catia_clock_request_union request;
    uint32_t receive_us;
    uint32_t transmit_us;
  } data;
  uint8_t bin[CATIA_CLOCK_REPLY_MSG_SIZE];
};

union catia_pose_clocked_union {
  struct {
    union catia_pose_sample_union sample;
    union catia_clock_request_union request;
  } data;
  uint8_t bin[CATIA_POSE_CLOCKED_MSG_SIZE];
};

// Each platform supplies CameraLinkTransmit; this shared header stays OS-independent.

extern uint8_t catia_ck_a, catia_ck_b;

#define STX  0x99

#define CatiaSizeOf(_payload) (_payload+5)

#define CatiaPutUint8( _byte) {     \
    catia_ck_a += _byte;              \
    catia_ck_b += catia_ck_a;          \
    CameraLinkTransmit(_byte);     \
  }

#define CatiaHeader(msg_id, payload_len) {           \
    CameraLinkTransmit(STX);                        \
    uint8_t msg_len = CatiaSizeOf( payload_len);       \
    CameraLinkTransmit(msg_len);                    \
    catia_ck_a = msg_len; catia_ck_b = msg_len;         \
    CatiaPutUint8(msg_id);                             \
  }

#define CatiaTrailer() {               \
    CameraLinkTransmit(catia_ck_a);    \
    CameraLinkTransmit(catia_ck_b);    \
  }

#define CatiaPut1ByteByAddr( _byte) {  \
    uint8_t _x = *(_byte);              \
    CatiaPutUint8( _x);                  \
  }

/////////////////////////////////////////////////////////////////////
// PARSING

/**
 * @brief Persistent state for incremental CATIA UART frame parsing.
 * @details One instance must remain associated with one ordered byte stream. A completed
 * message remains in @c payload until its consumer clears @c msg_received; feeding later
 * bytes before that acknowledgement is counted as an error to avoid silently overwriting it.
 */
struct catia_transport {
  /** Storage for the validated payload of the most recently completed message. */
  uint8_t payload[256];
  /** Count of framing or checksum faults observed while consuming this stream. */
  uint8_t error;
  /** Message identifier from the current or most recently completed frame. */
  uint8_t msg_id;
  /** True when @c payload contains one complete validated message awaiting dispatch. */
  bool    msg_received;
  /** Number of payload bytes expected in the current frame. */
  uint8_t payload_len;
  /** Internal parser state: the next expected position in the frame. */
  uint8_t status;
  /** Internal offset into @c payload while a frame is being assembled. */
  uint8_t payload_idx;
  /** Running Fletcher-style checksum accumulators for the in-progress frame. */
  uint8_t ck_a, ck_b;
};

extern struct catia_transport catia_protocol;

/**
 * @brief Consume one byte from a CATIA UART stream.
 * @param t Persistent parser state for that stream.
 * @param c Newly received byte.
 * @details Frames may be arbitrarily split across calls. On a valid complete frame this sets
 * @c t->msg_received and leaves the decoded bytes in @c t->payload. The caller must dispatch
 * the message and clear that flag before supplying bytes for the next frame.
 * @warning This function is not thread-safe. Serialize access to each @p t instance and do not
 * alter its fields while a frame is in progress.
 */
void parse_catia(struct catia_transport *t, uint8_t c);


#endif

