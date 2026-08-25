#include <stdio.h>
#include <stdlib.h>

#include "modules/multi/traffic_info_mesh_policy.h"

#define HOLDOVER_MS 60000u
#define ACQUIRE_MS 2000u
#define STEP_MS 10u

static void expect(bool condition, const char *message)
{
  if (!condition) {
    fprintf(stderr, "%s\n", message);
    exit(EXIT_FAILURE);
  }
}

static uint64_t update(struct MeshClockState *clock, uint64_t local_ms,
                       bool gps_valid, uint64_t gps_ms, bool *reset_slots)
{
       return mesh_clock_update(clock, local_ms, gps_valid, gps_ms, false,
                           HOLDOVER_MS, ACQUIRE_MS, STEP_MS, reset_slots);
}

int main(void)
{
       puts("1..37");
  struct MeshClockState clock;
  bool reset_slots;
  mesh_clock_init(&clock);
  expect(clock.mode == MESH_CLOCK_ASYNC, "boot did not start asynchronously");
  puts("ok 1 - boot without GPS is asynchronous");

  expect(update(&clock, 1000, false, 0, &reset_slots) == 1000 && !reset_slots,
         "GPS-denied boot did not retain local time");
  puts("ok 2 - denied boot uses monotonic time");

  update(&clock, 2000, true, 604802000ULL, &reset_slots);
  expect(clock.mode == MESH_CLOCK_ASYNC && !reset_slots,
         "GPS acquisition entered TDMA immediately");
  puts("ok 3 - acquisition requires stable GPS");

  update(&clock, 3999, true, 604803999ULL, &reset_slots);
  expect(clock.mode == MESH_CLOCK_ASYNC, "GPS acquisition completed early");
  puts("ok 4 - acquisition interval is enforced");

  expect(update(&clock, 4000, true, 604804000ULL, &reset_slots) == 604804000ULL
         && clock.mode == MESH_CLOCK_GPS && reset_slots,
         "stable GPS did not enter TDMA with a reset");
  puts("ok 5 - stable GPS enters TDMA");

  expect(update(&clock, 5000, false, 0, &reset_slots) == 604805000ULL
         && clock.mode == MESH_CLOCK_HOLDOVER && !reset_slots,
         "GPS loss did not preserve the network epoch");
  puts("ok 6 - GPS loss enters holdover without reset");

  expect(update(&clock, 5000 + HOLDOVER_MS, false, 0, &reset_slots)
         == 604805000ULL + HOLDOVER_MS && clock.mode == MESH_CLOCK_HOLDOVER,
         "holdover expired before its inclusive deadline");
  puts("ok 7 - holdover deadline is inclusive");

  expect(update(&clock, 5001 + HOLDOVER_MS, false, 0, &reset_slots)
         == 5001 + HOLDOVER_MS && clock.mode == MESH_CLOCK_ASYNC && reset_slots,
         "expired holdover did not enter asynchronous mode");
  puts("ok 8 - expired holdover enters asynchronous mode");

  update(&clock, 190000, true, 604990000ULL, &reset_slots);
  update(&clock, 191000, false, 0, &reset_slots);
  expect(clock.mode == MESH_CLOCK_ASYNC,
         "GPS flapping preserved an interrupted acquisition");
  puts("ok 9 - GPS flapping restarts acquisition");

  update(&clock, 192000, true, 604992000ULL, &reset_slots);
  update(&clock, 194000, true, 604994000ULL, &reset_slots);
  expect(clock.mode == MESH_CLOCK_GPS && reset_slots,
         "asynchronous reacquisition did not reset slots");
  puts("ok 10 - asynchronous reacquisition resets slots");

  update(&clock, 195000, false, 0, &reset_slots);
  update(&clock, 196000, true, 604996005ULL, &reset_slots);
  update(&clock, 198000, true, 604998005ULL, &reset_slots);
  expect(clock.mode == MESH_CLOCK_GPS && !reset_slots,
         "coherent holdover reacquisition discarded slots");
  puts("ok 11 - coherent holdover reacquisition preserves slots");

  update(&clock, 199000, false, 0, &reset_slots);
  update(&clock, 200000, true, 605000050ULL, &reset_slots);
  update(&clock, 202000, true, 605002050ULL, &reset_slots);
  expect(clock.mode == MESH_CLOCK_GPS && reset_slots,
         "large holdover correction preserved stale slots");
  puts("ok 12 - large holdover correction resets slots");

  expect(mesh_clock_difference_ms(3, 10) == 7,
         "clock difference failed descending timestamps");
  puts("ok 13 - clock difference handles descending values");
  expect(mesh_clock_difference_ms(10, 3) == 7,
         "clock difference failed ascending timestamps");
  puts("ok 14 - clock difference handles ascending values");
  expect(mesh_clock_difference_ms(10, 10) == 0,
         "clock difference failed equal timestamps");
  puts("ok 15 - clock difference handles equal values");
       expect(mesh_clock_project(&clock, 202000) == 605002050ULL,
         "GPS epoch was not refreshed after reacquisition");
  puts("ok 16 - reacquisition refreshes the epoch");

         expect(mesh_clock_update(&clock, 203000, true, 605003050ULL, true,
                                  HOLDOVER_MS, ACQUIRE_MS, STEP_MS, &reset_slots)
                == 203000 && clock.mode == MESH_CLOCK_ASYNC && reset_slots,
                "peer fallback did not force asynchronous mode");
         puts("ok 17 - peer fallback forces asynchronous mode");

         mesh_clock_update(&clock, 206000, true, 605006050ULL, true,
                           HOLDOVER_MS, ACQUIRE_MS, STEP_MS, &reset_slots);
         expect(clock.mode == MESH_CLOCK_ASYNC && !clock.acquiring,
                "forced fallback allowed GPS acquisition");
         puts("ok 18 - forced fallback blocks acquisition");

         update(&clock, 207000, true, 605007050ULL, &reset_slots);
         update(&clock, 209000, true, 605009050ULL, &reset_slots);
         expect(clock.mode == MESH_CLOCK_GPS && reset_slots,
                "GPS did not recover after peer fallback expired");
         puts("ok 19 - GPS recovers after peer fallback expires");

         clock.mode = MESH_CLOCK_HOLDOVER;
         clock.holdover_started_ms = 1000;
         expect(!mesh_clock_holdover_expiring(&clock, 1000 + HOLDOVER_MS - 12001,
                                              HOLDOVER_MS, 12000),
                "holdover entered fallback before the lead boundary");
         puts("ok 20 - holdover remains TDMA before fallback lead");
         expect(mesh_clock_holdover_expiring(&clock, 1000 + HOLDOVER_MS - 12000,
                                             HOLDOVER_MS, 12000),
                "holdover missed the fallback lead boundary");
         puts("ok 21 - holdover enters fallback at lead boundary");

         expect(!mesh_clock_frame_reached(10, 0),
                "zero recovery target was treated as active");
         puts("ok 22 - zero recovery target is inactive");
         expect(!mesh_clock_frame_reached(9, 10),
                "recovery target was reached early");
         puts("ok 23 - recovery frame waits for target");
         expect(mesh_clock_frame_reached(10, 10),
                "recovery target was not reached exactly");
         puts("ok 24 - recovery frame reaches exact target");
         expect(mesh_clock_frame_reached(1, UINT32_MAX),
                "recovery frame did not cross rollover");
         puts("ok 25 - recovery frame crosses rollover");
         expect(mesh_clock_frame_is_later(11, 10),
                "later recovery target was rejected");
         puts("ok 26 - later recovery target is accepted");
         expect(mesh_clock_frame_is_later(1, UINT32_MAX),
                "later recovery target was rejected across rollover");
         puts("ok 27 - recovery target ordering crosses rollover");
       expect(mesh_clock_next_recovery_frame(1, 8) == 8,
                "recovery epoch did not round to the shared boundary");
         puts("ok 28 - recovery target rounds to epoch");
       expect(mesh_clock_next_recovery_frame(8, 8) == 16,
                "recovery epoch reused the current boundary");
         puts("ok 29 - recovery target selects next epoch");
       expect(mesh_clock_next_recovery_frame(UINT32_MAX, 8) == 8,
                "recovery epoch returned the zero sentinel at rollover");
         puts("ok 30 - recovery epoch avoids zero at rollover");

         mesh_clock_init(&clock);
         update(&clock, 1000, true, 100000, &reset_slots);
         expect(clock.acquiring, "test setup did not begin GPS acquisition");
         mesh_clock_update(&clock, 1500, true, 100500, true,
                           HOLDOVER_MS, ACQUIRE_MS, STEP_MS, &reset_slots);
         expect(!clock.acquiring && clock.mode == MESH_CLOCK_ASYNC,
                "forced fallback did not cancel in-progress acquisition");
         puts("ok 31 - fallback cancels acquisition");
         update(&clock, 3000, true, 102000, &reset_slots);
         expect(clock.mode == MESH_CLOCK_ASYNC,
                "cancelled acquisition resumed without a full stable interval");
         puts("ok 32 - cancelled acquisition restarts timing");
       expect(mesh_clock_next_recovery_frame(7 + 7, 8) == 16,
                "minimum lead did not skip an unsafe nearby epoch");
         puts("ok 33 - recovery lead skips nearby epoch");
       expect(mesh_clock_sanitize_recovery_frame(10, 17, 8, 7) == 0,
                "unaligned recovery target was accepted");
         puts("ok 34 - unaligned recovery target is rejected");
       expect(mesh_clock_sanitize_recovery_frame(10, 32, 8, 7) == 0,
                "implausibly distant recovery target was accepted");
         puts("ok 35 - distant recovery target is rejected");
       expect(mesh_clock_sanitize_recovery_frame(15, 16, 8, 7) == 24,
                "unsafe nearby target was not advanced");
         puts("ok 36 - nearby recovery target is advanced");
       expect(mesh_clock_sanitize_recovery_frame(10, 24, 8, 7) == 24,
                "valid bounded recovery target was changed");
         puts("ok 37 - bounded recovery target is accepted");
  return EXIT_SUCCESS;
}