#include <stdio.h>
#include <stdlib.h>

#include "modules/multi/traffic_info_mesh_mode.h"

static void expect(bool condition, const char *message)
{
  if (!condition) {
    fprintf(stderr, "%s\n", message);
    exit(EXIT_FAILURE);
  }
}

int main(void)
{
  puts("1..20");
  struct mesh_mode_policy_input input = {
    .self_ping_fresh = true,
    .other_ping_fresh = false,
    .peer_present = false,
    .peer_quiet_ms = 12000,
    .required_quiet_ms = 12000
  };

  expect(mesh_mode_should_use_solo(&input), "healthy solo link was rejected");
  puts("ok 1 - healthy one-aircraft link selects solo");
  input.self_ping_fresh = false;
  expect(!mesh_mode_should_use_solo(&input), "missing GCS ping selected solo");
  puts("ok 2 - missing self ping selects mesh");
  input.self_ping_fresh = true;
  input.other_ping_fresh = true;
  expect(!mesh_mode_should_use_solo(&input), "other targeted ping selected solo");
  puts("ok 3 - other aircraft ping selects mesh");
  input.other_ping_fresh = false;
  input.peer_present = true;
  expect(!mesh_mode_should_use_solo(&input), "mesh peer selected solo");
  puts("ok 4 - received peer state selects mesh");
  input.peer_present = false;
  input.peer_quiet_ms = input.required_quiet_ms - 1;
  expect(!mesh_mode_should_use_solo(&input), "quiet interval ended early");
  puts("ok 5 - quiet interval is enforced");
  input.peer_quiet_ms = input.required_quiet_ms;
  expect(mesh_mode_should_use_solo(&input), "inclusive quiet boundary failed");
  puts("ok 6 - quiet boundary selects solo");
  input.peer_quiet_ms = input.required_quiet_ms + 60000;
  expect(mesh_mode_should_use_solo(&input), "long GPS-denied solo run was rejected");
  puts("ok 7 - solo selection is independent of mesh clock state");
    expect(!mesh_mode_should_use_dense(11, false, 12, 10),
      "dense mode entered below its threshold");
    puts("ok 8 - common mode remains below dense threshold");
    expect(mesh_mode_should_use_dense(12, false, 12, 10),
      "dense mode did not enter at its threshold");
    puts("ok 9 - dense mode enters at threshold");
    expect(mesh_mode_should_use_dense(11, true, 12, 10),
      "dense mode left inside the hysteresis band");
    puts("ok 10 - dense mode holds inside hysteresis band");
    expect(!mesh_mode_should_use_dense(10, true, 12, 10),
      "dense mode did not leave at its exit threshold");
    puts("ok 11 - dense mode leaves at exit threshold");
      struct mesh_async_interval interval =
        mesh_async_interval_for_peers(0, false, 16000, 24000);
      expect(interval.min_ms == 16000 && interval.max_ms == 24000,
        "unknown population accelerated asynchronous traffic");
      puts("ok 12 - unknown population keeps conservative interval");
      interval = mesh_async_interval_for_peers(0, true, 16000, 24000);
      expect(interval.min_ms == 4000 && interval.max_ms == 8000,
        "known solo population did not accelerate");
      puts("ok 13 - known solo population uses fast interval");
      interval = mesh_async_interval_for_peers(1, true, 16000, 24000);
      expect(interval.min_ms == 8000 && interval.max_ms == 16000,
        "two-aircraft population did not use sparse interval");
      puts("ok 14 - two-aircraft population uses sparse interval");
      interval = mesh_async_interval_for_peers(3, true, 16000, 24000);
      expect(interval.min_ms == 8000 && interval.max_ms == 16000,
        "four-aircraft population did not use sparse interval");
      puts("ok 15 - four-aircraft population uses sparse interval");
      interval = mesh_async_interval_for_peers(4, true, 16000, 24000);
      expect(interval.min_ms == 16000 && interval.max_ms == 24000,
        "five-aircraft population accelerated dense fallback");
      puts("ok 16 - five-aircraft population keeps dense interval");
      interval = mesh_async_interval_for_peers(0, true, 1, 1);
      expect(interval.min_ms == 1 && interval.max_ms == 1,
        "small configured interval underflowed");
      puts("ok 17 - asynchronous interval cannot underflow");
        expect(!mesh_position_holdover_valid(false, 0, 12000),
          "position holdover started without a prior GNSS fix");
        puts("ok 18 - position holdover requires prior GNSS fix");
        expect(mesh_position_holdover_valid(true, 12000, 12000),
          "position holdover rejected its inclusive boundary");
        puts("ok 19 - position holdover includes configured boundary");
        expect(!mesh_position_holdover_valid(true, 12001, 12000),
          "position holdover accepted stale estimator state");
        puts("ok 20 - stale estimator state is rejected");
  return EXIT_SUCCESS;
}