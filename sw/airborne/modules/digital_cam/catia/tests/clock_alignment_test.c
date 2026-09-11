#include "../clock_alignment.h"
#include <assert.h>
#include <stddef.h>
#include <stdio.h>

static uint32_t fc_time(uint64_t relative_us, uint32_t origin, uint32_t rate)
{
  return origin + (uint32_t)(relative_us * rate / 1000000);
}

static void drift_bounds(void)
{
  const uint32_t origins[] = {0, UINT32_MAX - 800};
  const uint32_t rates[] = {999000, 999500, 1000000, 1000500, 1001000};
  const uint32_t forward_delays[] = {1, 1000, 40000};
  const uint32_t return_delays[] = {1, 2000, 45000};
  const uint32_t service_delays[] = {0, 13, 2000};
  for (size_t origin = 0; origin < sizeof(origins) / sizeof(origins[0]); ++origin) {
    for (size_t rate = 0; rate < sizeof(rates) / sizeof(rates[0]); ++rate) {
      for (size_t forward = 0; forward < sizeof(forward_delays) / sizeof(forward_delays[0]); ++forward) {
        for (size_t backward = 0; backward < sizeof(return_delays) / sizeof(return_delays[0]); ++backward) {
          for (size_t service = 0; service < sizeof(service_delays) / sizeof(service_delays[0]); ++service) {
            struct clock_alignment clock = {0};
            union catia_clock_request_union token = {.data = {4, 7}};
            const uint64_t base = 10000000;
            const uint64_t receive = forward_delays[forward];
            const uint64_t transmit = receive + service_delays[service];
            const uint64_t returned = transmit + return_delays[backward];
            union catia_clock_reply_union reply = {.data = {
              token, fc_time(receive, origins[origin], rates[rate]), fc_time(transmit, origins[origin], rates[rate])
            }};
            assert(clock_alignment_request(&clock, token, base));
            assert(clock_alignment_reply(&clock, reply.bin, sizeof(reply.bin), base + returned));
            for (uint64_t elapsed = 0; elapsed < 1990000; elapsed += 9973) {
              const uint64_t sample = transmit + elapsed;
              const uint64_t now = base + (sample > returned ? sample : returned);
              struct clock_interval interval;
              assert(clock_alignment_map(&clock, token, fc_time(sample, origins[origin], rates[rate]), now, &interval));
              assert(interval.earliest_us <= base + sample && interval.latest_us >= base + sample);
            }
            union catia_clock_request_union next = {.data = {5, 7}};
            assert(clock_alignment_request(&clock, next, base + returned + 1000000));
            assert(clock.mapped_sent_us == base && clock.sent_us != clock.mapped_sent_us);
            struct clock_interval interval;
            assert(clock_alignment_map(&clock, token, fc_time(transmit + 1000000, origins[origin], rates[rate]),
                                         base + returned + 1000000, &interval));
          }
        }
      }
    }
  }
}

int main(void)
{
  drift_bounds();
  for (uint32_t forward = 1000; forward <= 50000; forward += 7000) {
    for (uint32_t backward = 1000; backward <= 40000; backward += 9000) {
      struct clock_alignment clock = {0};
      union catia_clock_request_union token = {.data = {1, 2}};
      const uint64_t sent = 10000000;
      const uint64_t received = sent + forward + 100 + backward;
      union catia_clock_reply_union reply = {.data = {token, UINT32_MAX - 50, 49}};
      assert(clock_alignment_request(&clock, token, sent));
      assert(clock_alignment_reply(&clock, reply.bin, sizeof(reply.bin), received));
      struct clock_interval interval;
      assert(clock_alignment_map(&clock, token, 100049, received + 100000, &interval));
      const uint64_t actual = sent + forward + 100 + 100000;
      assert(interval.earliest_us <= actual && interval.latest_us >= actual);
      assert(!clock_alignment_map(&clock, token, 48, received, &interval));
      assert(!clock_alignment_map(&clock, token, 100049, received + CLOCK_ALIGNMENT_MAX_AGE_US + 1, &interval));
      token.data.token_low++;
      assert(!clock_alignment_map(&clock, token, 100049, received + 100000, &interval));
      assert(clock_alignment_request(&clock, token, received + 100000));
      assert(!clock_alignment_reply(&clock, reply.bin, sizeof(reply.bin), received + 101000));
      reply.data.request = token;
      assert(!clock_alignment_reply(&clock, reply.bin, sizeof(reply.bin) - 1, received + 101000));
      assert(!clock_alignment_reply(&clock, reply.bin, sizeof(reply.bin), received + 200001));
      assert(!clock.valid);
    }
  }
  struct clock_alignment clock = {0};
  union catia_clock_request_union token = {.data = {1, 2}};
  assert(!clock_alignment_request(&clock, token, UINT64_MAX));
  assert(clock_alignment_request(&clock, token, 100));
  assert(!clock_alignment_request(&clock, token, 200));
  union catia_clock_reply_union bad = {.data = {token, 10000, 0}};
  assert(!clock_alignment_reply(&clock, bad.bin, sizeof(bad.bin), 1000));
  puts("Clock interval: bounded drift, asymmetric delay, wrap, pending-probe independence and rejection passed");
}