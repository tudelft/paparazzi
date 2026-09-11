#include <stddef.h>
#include "clock_alignment.h"

_Static_assert(sizeof(union catia_clock_reply_union) == CATIA_CLOCK_REPLY_MSG_SIZE, "Clock reply wire size");
_Static_assert(sizeof(union catia_pose_clocked_union) == CATIA_POSE_CLOCKED_MSG_SIZE, "Clocked pose wire size");

static bool same_token(union catia_clock_request_union first, union catia_clock_request_union second)
{
  return first.data.token_low == second.data.token_low && first.data.token_high == second.data.token_high;
}

bool clock_alignment_request(struct clock_alignment *clock, union catia_clock_request_union token, uint64_t sent_us)
{
  if (clock == NULL || sent_us == 0 || sent_us > UINT64_MAX - CLOCK_ALIGNMENT_MAX_RTT_US
      || (token.data.token_low == 0 && token.data.token_high == 0)
      || same_token(token, clock->pending_token) || same_token(token, clock->mapped_token)) return false;
  clock->pending_token = token;
  clock->sent_us = sent_us;
  clock->pending = true;
  return true;
}

bool clock_alignment_reply(struct clock_alignment *clock, const uint8_t *payload, size_t length, uint64_t received_us)
{
  if (clock == NULL || !clock->pending || payload == NULL || length != CATIA_CLOCK_REPLY_MSG_SIZE) return false;
  union catia_clock_reply_union reply = {0};
  for (size_t index = 0; index < sizeof(reply.bin); ++index) reply.bin[index] = payload[index];
  if (!same_token(reply.data.request, clock->pending_token)) return false;
  clock->pending = false;
  clock->valid = false;
  if (received_us < clock->sent_us || received_us > UINT64_MAX - CLOCK_ALIGNMENT_MAX_AGE_US - 10000
      || received_us - clock->sent_us > CLOCK_ALIGNMENT_MAX_RTT_US) return false;
  const uint32_t service_us = reply.data.transmit_us - reply.data.receive_us;
  if (service_us > CLOCK_ALIGNMENT_MAX_RTT_US) return false;
  const uint64_t service_min_us = (uint64_t)service_us * 1000000 / (1000000 + CLOCK_ALIGNMENT_DRIFT_PPM);
  const uint64_t lower = clock->sent_us + (service_min_us > 2 ? service_min_us - 2 : 0);
  if (lower > received_us + 2) return false;
  clock->anchor = (struct clock_interval){lower, received_us + 2};
  clock->anchor_fc_us = reply.data.transmit_us;
  clock->receive_fc_us = reply.data.receive_us;
  clock->mapped_sent_us = clock->sent_us;
  clock->received_us = received_us;
  clock->mapped_token = reply.data.request;
  clock->valid = true;
  return true;
}

bool clock_alignment_map(const struct clock_alignment *clock, union catia_clock_request_union token,
                         uint32_t fc_us, uint64_t now_us, struct clock_interval *interval)
{
  if (clock == NULL || interval == NULL || !clock->valid || !same_token(token, clock->mapped_token)
      || now_us < clock->received_us || now_us - clock->received_us > CLOCK_ALIGNMENT_MAX_AGE_US) return false;
  const uint32_t elapsed = fc_us - clock->anchor_fc_us;
  if (elapsed > CLOCK_ALIGNMENT_MAX_AGE_US) return false;
  const uint64_t minimum = (uint64_t)elapsed * 1000000 / (1000000 + CLOCK_ALIGNMENT_DRIFT_PPM);
  const uint64_t maximum = ((uint64_t)elapsed * 1000000 + 999999 - CLOCK_ALIGNMENT_DRIFT_PPM)
                           / (1000000 - CLOCK_ALIGNMENT_DRIFT_PPM);
  const uint64_t earliest = clock->anchor.earliest_us + (minimum > 2 ? minimum - 2 : 0);
  uint64_t latest = clock->anchor.latest_us + maximum + 2;
  if (earliest > now_us) return false;
  if (latest > now_us) latest = now_us;
  *interval = (struct clock_interval){earliest, latest};
  return true;
}