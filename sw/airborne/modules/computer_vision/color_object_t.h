#ifndef COLOR_OBJECT_T_H
#define COLOR_OBJECT_T_H

#include <stdint.h>
#include <stdbool.h>

struct color_object_t {
  int32_t Nobs;
  int32_t obstacle_data[100][4];
  uint32_t y_max;
  uint32_t x_max;
  bool updated;
};

#endif
