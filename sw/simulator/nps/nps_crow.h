#ifndef NPS_CROW_H
#define NPS_CROW_H

struct NpsCrowCommands {
  double roll;
  double brake;
};

static inline struct NpsCrowCommands nps_crow_commands(double left, double right, double full_scale)
{
  const struct NpsCrowCommands result = {
    .roll = (left + right) / (2.0 * full_scale),
    .brake = (left - right) / (2.0 * full_scale)
  };
  return result;
}

#endif