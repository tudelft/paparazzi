
#include "protocol.h"

/**
 * @file protocol.c
 * @brief Incremental parser for CATIA's UART framing protocol.
 * @details The UART transport is a byte stream, so this parser keeps state across
 * calls and accepts arbitrarily fragmented frames. It validates both Fletcher-style
 * checksum bytes before publishing a message. On malformed input it discards only
 * the current partial frame and resumes searching for the next start byte; this is
 * deliberately resilient to line noise and lost bytes.
 */

uint8_t catia_ck_a, catia_ck_b;


/** Parser states; each corresponds to the next byte expected on the wire. */
#define UNINIT      0
#define GOT_STX     1
#define GOT_LENGTH  2
#define GOT_MSGID   3
#define GOT_PAYLOAD 4
#define GOT_CRC1    5

struct catia_transport catia_protocol;

/**
 * @brief Consume one UART byte and advance a CATIA transport parser.
 * @param t Persistent parser state owned by the receiving transport.
 * @param c Newly received byte.
 * @details Checksums cover the length, message ID, and payload bytes. A complete,
 * valid frame sets @c t->msg_received; the caller must clear that flag after
 * dispatching. Any framing or checksum error increments @c t->error and resets the
 * state machine, avoiding a poisoned partial frame affecting later traffic.
 * @warning @p t must remain valid and unchanged by another thread for the duration
 * of a frame; this parser intentionally contains no synchronization.
 */
void parse_catia(struct catia_transport *t, uint8_t c)
{
//printf("%02X %d %d\n",c, t->status, t->error);

  switch (t->status) {
    case UNINIT:
      if (c == STX) {
        t->status++;
      }
      break;
    case GOT_STX:
      if (t->msg_received) {
        t->error++;
        goto error;
      }
      t->payload_len = c - 5; /* Counting STX, LENGTH and CRC1 and CRC2 */
      t->ck_a = t->ck_b = c;
      t->status++;
      t->payload_idx = 0;
      break;
    case GOT_LENGTH:
      t->msg_id = c;
      t->ck_a += c; t->ck_b += t->ck_a;
      t->status++;
      if (t->payload_len == 0) {
        t->status++;
      }
      break;
    case GOT_MSGID:
      t->payload[t->payload_idx] = c;
      t->ck_a += c; t->ck_b += t->ck_a;
      t->payload_idx++;
      if (t->payload_idx == t->payload_len) {
        t->status++;
      }
      break;
    case GOT_PAYLOAD:
      if (c != t->ck_a) {
        goto error;
      }
      t->status++;
      break;
    case GOT_CRC1:
      if (c != t->ck_b) {
        goto error;
      }
      t->msg_received = true;
      goto restart;
    default:
      goto error;
  }
  return;
error:
  t->error++;
restart:
  t->status = UNINIT;
  return;
}
