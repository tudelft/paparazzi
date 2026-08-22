#ifndef TRAFFIC_INFO_MESH_MODE_H
#define TRAFFIC_INFO_MESH_MODE_H

#include <stdbool.h>
#include <stdint.h>

struct mesh_mode_policy_input {
  bool self_ping_fresh;
  bool other_ping_fresh;
  bool peer_present;
  uint64_t peer_quiet_ms;
  uint32_t required_quiet_ms;
};

static inline bool mesh_mode_should_use_solo(
  const struct mesh_mode_policy_input *input)
{
  return input->self_ping_fresh
         && !input->other_ping_fresh
         && !input->peer_present
         && input->peer_quiet_ms >= input->required_quiet_ms;
}

#endif /* TRAFFIC_INFO_MESH_MODE_H */