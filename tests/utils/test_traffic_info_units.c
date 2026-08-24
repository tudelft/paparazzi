#include <limits.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "modules/multi/traffic_info_units.h"

static void expect_conversion(const char *name, int32_t centimeters,
                              bool expected_valid, int32_t expected_mm)
{
  int32_t millimeters = INT32_C(123456789);
  const bool valid = traffic_info_cm_to_mm(centimeters, &millimeters);
  if (valid != expected_valid || (valid && millimeters != expected_mm)) {
    fprintf(stderr, "%s: input %ld, valid %d, result %ld\n",
            name, (long)centimeters, valid, (long)millimeters);
    exit(EXIT_FAILURE);
  }
}

int main(void)
{
  puts("1..9");
  expect_conversion("zero", 0, true, 0);
  expect_conversion("positive", 12345, true, 123450);
  expect_conversion("negative", -12345, true, -123450);
  expect_conversion("positive boundary", INT32_MAX / 10, true, 2147483640);
  expect_conversion("negative boundary", INT32_MIN / 10, true, -2147483640);
  expect_conversion("positive overflow", INT32_MAX / 10 + 1, false, 0);
  expect_conversion("negative overflow", INT32_MIN / 10 - 1, false, 0);
  expect_conversion("maximum input", INT32_MAX, false, 0);
  expect_conversion("minimum input", INT32_MIN, false, 0);
  for (unsigned test = 1; test <= 9; test++) {
    printf("ok %u - checked traffic unit conversion\n", test);
  }
  return EXIT_SUCCESS;
}