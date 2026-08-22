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
  puts("1..7");
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
  return EXIT_SUCCESS;
}