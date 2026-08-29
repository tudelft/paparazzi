#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "modules/sensors/airspeed_sdp3x.h"

static unsigned test_number;

static void expect(bool condition, const char *name)
{
  test_number++;
  if (!condition) {
    fprintf(stderr, "not ok %u - %s\n", test_number, name);
    exit(EXIT_FAILURE);
  }
  printf("ok %u - %s\n", test_number, name);
}

static void expect_near(float actual, float expected, float tolerance, const char *name)
{
  expect(isfinite(actual) && fabsf(actual - expected) <= tolerance, name);
}

int main(void)
{
  puts("1..23");

  const uint8_t datasheet_crc_example[] = {0xbe, 0xef};
  expect(sdp3x_crc_valid(datasheet_crc_example, 2, 0x92), "datasheet CRC vector");
  expect(!sdp3x_crc_valid(datasheet_crc_example, 2, 0x93), "corrupt CRC rejected");

  expect(sdp3x_decode_int16(0x00, 0x00) == 0, "decode zero");
  expect(sdp3x_decode_int16(0x7f, 0xff) == INT16_MAX, "decode positive limit");
  expect(sdp3x_decode_int16(0x80, 0x00) == INT16_MIN, "decode negative limit");
  expect(sdp3x_decode_int16(0xff, 0xec) == -20, "decode negative one Pa at SDP33 scale");

  expect(sdp3x_scale_is_valid(20), "SDP33 scale accepted");
  expect(sdp3x_scale_is_valid(60), "SDP31 scale accepted");
  expect(sdp3x_scale_is_valid(240), "SDP32 scale accepted");
  expect(!sdp3x_scale_is_valid(0), "zero scale rejected");
  expect(!sdp3x_scale_is_valid(21), "unknown scale rejected");

  expect_near(sdp3x_pressure_from_raw(1764, 20.f, false), 88.2f, 1e-5f,
              "normal polarity converts counts to Pa");
  expect_near(sdp3x_pressure_from_raw(-1764, 20.f, true), 88.2f, 1e-5f,
              "reversed polarity converts forward pressure to positive Pa");

  expect_near(sdp3x_pressure_for_airspeed(88.2f, false), 88.2f, 1e-6f,
              "unidirectional mode preserves positive pressure");
  expect_near(sdp3x_pressure_for_airspeed(-88.2f, false), 0.f, 1e-6f,
              "unidirectional mode clamps negative pressure");
  expect_near(sdp3x_pressure_for_airspeed(88.2f, true), 88.2f, 1e-6f,
              "bidirectional mode preserves positive pressure");
  expect_near(sdp3x_pressure_for_airspeed(-88.2f, true), -88.2f, 1e-6f,
              "bidirectional mode preserves negative pressure");

  const float eas_scale = 2.0f / 1.225f;
  expect_near(sdp3x_eas_from_pressure(0.f, eas_scale), 0.f, 1e-6f, "zero pressure is zero EAS");
  expect_near(sdp3x_eas_from_pressure(88.2f, eas_scale), 12.f, 1e-5f, "88.2 Pa is 12 m/s EAS");
  expect_near(sdp3x_eas_from_pressure(177.0125f, eas_scale), 17.f, 1e-5f, "177.0125 Pa is 17 m/s EAS");
  expect_near(sdp3x_eas_from_pressure(382.8125f, eas_scale), 25.f, 1e-5f, "382.8125 Pa is 25 m/s EAS");
  expect_near(sdp3x_eas_from_pressure(1500.f, eas_scale), 49.4872f, 1e-4f, "SDP33 full scale is 49.49 m/s EAS");
  expect_near(sdp3x_eas_from_pressure(-88.2f, eas_scale), -12.f, 1e-5f, "negative pressure preserves sign");

  return EXIT_SUCCESS;
}