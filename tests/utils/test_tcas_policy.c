#include <stdio.h>
#include <stdlib.h>

#include "modules/multi/tcas_policy.h"
#include "modules/multi/traffic_info_time.h"

#define FRESH_MS 4000u
#define DROP_MS 25375u
#define LEGACY_MS 1500u

static void expect_action(const char *name, enum tcas_surveillance_action actual,
                          enum tcas_surveillance_action expected)
{
  if (actual != expected) {
    fprintf(stderr, "%s: expected action %d, got %d\n", name, expected, actual);
    exit(EXIT_FAILURE);
  }
}

static enum tcas_surveillance_action action(bool valid, bool mesh,
                                             bool advisory, uint32_t age)
{
  return tcas_surveillance_action(valid, mesh, advisory, age,
                                  FRESH_MS, DROP_MS, LEGACY_MS);
}

int main(void)
{
  puts("1..29");
  expect_action("mesh fresh-1", action(true, true, false, FRESH_MS - 1u),
                TCAS_SURVEILLANCE_EVALUATE);
  expect_action("mesh fresh exact", action(true, true, false, FRESH_MS),
                TCAS_SURVEILLANCE_EVALUATE);
  expect_action("mesh fresh+1 no advisory", action(true, true, false, FRESH_MS + 1u),
                TCAS_SURVEILLANCE_UNAVAILABLE);
  expect_action("mesh fresh+1 active advisory", action(true, true, true, FRESH_MS + 1u),
                TCAS_SURVEILLANCE_HOLD);
  expect_action("mesh drop exact active advisory", action(false, true, true, DROP_MS),
                TCAS_SURVEILLANCE_HOLD);
  expect_action("mesh drop+1 active advisory", action(false, true, true, DROP_MS + 1u),
                TCAS_SURVEILLANCE_UNAVAILABLE);
  expect_action("mesh invalid without advisory", action(false, true, false, 0),
                TCAS_SURVEILLANCE_UNAVAILABLE);
  expect_action("mesh invalid no valid history", action(false, true, true, UINT32_MAX),
                TCAS_SURVEILLANCE_UNAVAILABLE);

  expect_action("legacy invalid drop exact", action(false, false, true, 3u * LEGACY_MS),
                TCAS_SURVEILLANCE_HOLD);
  expect_action("legacy invalid drop+1", action(false, false, true, 3u * LEGACY_MS + 1u),
                TCAS_SURVEILLANCE_UNAVAILABLE);

  expect_action("legacy hold exact", action(true, false, false, LEGACY_MS),
                TCAS_SURVEILLANCE_EVALUATE);
  expect_action("legacy hold+1 active advisory", action(true, false, true, LEGACY_MS + 1u),
                TCAS_SURVEILLANCE_HOLD);
  expect_action("legacy hold+1 no advisory", action(true, false, false, LEGACY_MS + 1u),
                TCAS_SURVEILLANCE_UNAVAILABLE);
  expect_action("legacy drop exact active advisory", action(true, false, true, 3u * LEGACY_MS),
                TCAS_SURVEILLANCE_HOLD);
  expect_action("legacy drop+1 active advisory", action(true, false, true, 3u * LEGACY_MS + 1u),
                TCAS_SURVEILLANCE_UNAVAILABLE);

  const float intruder_msl = tcas_intruder_altitude_msl(123.0f, 35.0f, 20.0f);
  const float altitude_error = intruder_msl - 138.0f;
  if (altitude_error < -1e-6f || altitude_error > 1e-6f) {
    fprintf(stderr, "altitude conversion: expected 138, got %.6f\n", intruder_msl);
    return EXIT_FAILURE;
  }

  if (!traffic_info_itow_is_newer(0u, TRAFFIC_INFO_GPS_WEEK_MS - 1u)) {
    fputs("week rollover was not accepted as newer\n", stderr);
    return EXIT_FAILURE;
  }
  if (traffic_info_itow_is_newer(100u, 100u)) {
    fputs("duplicate TOW was accepted as newer\n", stderr);
    return EXIT_FAILURE;
  }
  if (traffic_info_itow_is_newer(100u, 101u)) {
    fputs("older TOW was accepted as newer\n", stderr);
    return EXIT_FAILURE;
  }
  if (traffic_info_itow_is_newer(TRAFFIC_INFO_GPS_HALF_WEEK_MS, 0u)) {
    fputs("ambiguous half-week TOW was accepted as newer\n", stderr);
    return EXIT_FAILURE;
  }

  if (!tcas_velocity_is_usable(0.0f, 0.0f, 0.0f)) {
    fputs("finite zero velocity was rejected\n", stderr);
    return EXIT_FAILURE;
  }
  if (tcas_velocity_is_usable(NAN, 0.0f, 0.0f)) {
    fputs("non-finite velocity was accepted\n", stderr);
    return EXIT_FAILURE;
  }

  float resolved_msl = 0.f;
  if (!tcas_resolve_altitude_msl(TCAS_RESOLUTION_CLIMB, true, 100.f, 95.f, 15.f, 25.f, &resolved_msl)
      || resolved_msl != 110.f) {
    fputs("climb resolution did not preserve fixed-wing behavior\n", stderr);
    return EXIT_FAILURE;
  }
  if (!tcas_resolve_altitude_msl(TCAS_RESOLUTION_DESCEND, true, 100.f, 105.f, 15.f, 25.f, &resolved_msl)
      || resolved_msl != 90.f) {
    fputs("descend resolution did not preserve fixed-wing behavior\n", stderr);
    return EXIT_FAILURE;
  }
  if (!tcas_resolve_altitude_msl(TCAS_RESOLUTION_DESCEND, true, 30.f, 20.f, 15.f, 25.f, &resolved_msl)
      || resolved_msl != 25.f) {
    fputs("security floor was not applied\n", stderr);
    return EXIT_FAILURE;
  }
    if (tcas_resolve_altitude_msl(TCAS_RESOLUTION_NONE, true, 100.f, 95.f, 15.f, 25.f, &resolved_msl)
      || tcas_resolve_altitude_msl(TCAS_RESOLUTION_CLIMB, false, 100.f, 95.f, 15.f, 25.f, &resolved_msl)
      || tcas_resolve_altitude_msl(TCAS_RESOLUTION_CLIMB, true, NAN, 95.f, 15.f, 25.f, &resolved_msl)) {
    fputs("inactive or invalid altitude resolution was accepted\n", stderr);
    return EXIT_FAILURE;
  }
  if (tcas_altitude_local_from_msl(138.f, 23.f) != 115.f) {
    fputs("MSL-to-local altitude conversion failed\n", stderr);
    return EXIT_FAILURE;
  }

  for (unsigned test = 1; test <= 29; test++) {
    printf("ok %u - TCAS policy boundary\n", test);
  }
  return EXIT_SUCCESS;
}