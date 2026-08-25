#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "modules/multi/traffic_info_policy.h"

static void expect(const char *name, bool actual, bool expected)
{
  if (actual != expected) {
    fprintf(stderr, "%s: expected %d, got %d\n", name, expected, actual);
    exit(EXIT_FAILURE);
  }
}

int main(void)
{
  puts("1..6");
  expect("fresh slot", traffic_info_reclaim_candidate_preferred(
           1000u, 501u, 500u, false, 0u), false);
  expect("exact stale boundary", traffic_info_reclaim_candidate_preferred(
           1000u, 500u, 500u, false, 0u), true);
  expect("first stale candidate", traffic_info_reclaim_candidate_preferred(
           1000u, 100u, 500u, false, 0u), true);
  expect("older stale candidate", traffic_info_reclaim_candidate_preferred(
           1000u, 100u, 500u, true, 200u), true);
  expect("newer stale candidate", traffic_info_reclaim_candidate_preferred(
           1000u, 300u, 500u, true, 200u), false);
  expect("large monotonic time", traffic_info_reclaim_candidate_preferred(
           UINT64_MAX - 1u, UINT64_MAX - 501u, 500u, false, 0u), true);
  for (unsigned test = 1; test <= 6; test++) {
    printf("ok %u - traffic slot reclamation policy\n", test);
  }
  return EXIT_SUCCESS;
}