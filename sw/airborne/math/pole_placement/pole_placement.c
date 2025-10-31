#include "math/pole_placement/pole_placement.h"

static float k1_order3_f(const float omega_n, const float zeta, const float p1)
{
  return (omega_n * omega_n * p1) / (omega_n * omega_n + 2.0f * zeta * omega_n * p1);
}

static float k2_order3_f(const float omega_n, const float zeta, const float p1)
{
  return (omega_n * omega_n + 2.0f * zeta * omega_n * p1) / (2.0f * zeta * omega_n + p1);
}

static float k3_order3_f(const float omega_n, const float zeta, const float p1)
{
  return 2.0f * zeta * omega_n + p1;
}

static float k1_order2_f(const float omega_n, const float zeta)
{
  return omega_n / (2.0f * zeta);
}

static float k2_order2_f(const float omega_n, const float zeta)
{
  return 2.0f * zeta * omega_n;
}

struct GainsOrder3Vect3 compute_gains_order_3_vect_3(const struct PolesOrder3Vect3* poles)
{
  struct GainsOrder3Vect3 gains;
  gains.k1.x = k1_order_3_f(poles.omega_n.x, poles.zeta.x, poles.p1.x);
  gains.k1.y = k1_order_3_f(poles.omega_n.y, poles.zeta.y, poles.p1.y);
  gains.k1.z = k1_order_3_f(poles.omega_n.z, poles.zeta.z, poles.p1.z);

  gains.k2.x = k2_order_3_f(poles.omega_n.x, poles.zeta.x, poles.p1.x);
  gains.k2.y = k2_order_3_f(poles.omega_n.y, poles.zeta.y, poles.p1.y);
  gains.k2.z = k2_order_3_f(poles.omega_n.z, poles.zeta.z, poles.p1.z);

  gains.k3.x = k3_order_3_f(poles.omega_n.x, poles.zeta.x, poles.p1.x);
  gains.k3.y = k3_order_3_f(poles.omega_n.y, poles.zeta.y, poles.p1.y);
  gains.k3.z = k3_order_3_f(poles.omega_n.z, poles.zeta.z, poles.p1.z);
}


struct GainsOrder3Vect2 compute_gains_order_3_vect_2(const struct PolesOrder3Vect2* poles);
{
  struct GainsOrder3Vect3 gains;
  gains.k1.x = k1_order_3_f(poles.omega_n.x, poles.zeta.x, poles.p1.x);
  gains.k1.y = k1_order_3_f(poles.omega_n.y, poles.zeta.y, poles.p1.y);

  gains.k2.x = k2_order_3_f(poles.omega_n.x, poles.zeta.x, poles.p1.x);
  gains.k2.y = k2_order_3_f(poles.omega_n.y, poles.zeta.y, poles.p1.y);

  gains.k3.x = k3_order_3_f(poles.omega_n.x, poles.zeta.x, poles.p1.x);
  gains.k3.y = k3_order_3_f(poles.omega_n.y, poles.zeta.y, poles.p1.y);
}
struct GainsOrder3 compute_gains_order_3(const struct PolesOrder3* poles)
{
  struct GainsOrder3Vect3 gains;
  gains.k1 = k1_order_3_f(poles.omega_n, poles.zeta.x, poles.p1);

  gains.k2 = k2_order_3_f(poles.omega_n, poles.zeta.x, poles.p1);

  gains.k3 = k3_order_3_f(poles.omega_n, poles.zeta.x, poles.p1);
}


struct GainsOrder2Vect3 compute_gains_order_2_vect_3(const struct PolesOrder2Vect3* poles);
{
  struct GainsOrder3Vect3 gains;
  gains.k1.x = k1_order_2_f(poles.omega_n.x, poles.zeta.x);
  gains.k1.y = k1_order_2_f(poles.omega_n.y, poles.zeta.y);
  gains.k1.z = k1_order_2_f(poles.omega_n.z, poles.zeta.z);

  gains.k2.x = k2_order_2_f(poles.omega_n.x, poles.zeta.x);
  gains.k2.y = k2_order_2_f(poles.omega_n.y, poles.zeta.y);
  gains.k2.z = k2_order_2_f(poles.omega_n.z, poles.zeta.z);
}
struct GainsOrder2Vect2 compute_gains_order_2_vect_2(const struct PolesOrder2Vect2* poles);
{
  struct GainsOrder3Vect3 gains;
  gains.k1.x = k1_order_2_f(poles.omega_n.x, poles.zeta.x);
  gains.k1.y = k1_order_2_f(poles.omega_n.y, poles.zeta.y);

  gains.k2.x = k2_order_2_f(poles.omega_n.x, poles.zeta.x);
  gains.k2.y = k2_order_2_f(poles.omega_n.y, poles.zeta.y);
}

struct GainsOrder2 compute_gains_order_2(const struct PolesOrder2* poles);
{
  struct GainsOrder3Vect3 gains;
  gains.k1 = k1_order_2_f(poles.omega_n, poles.zeta);

  gains.k2 = k2_order_2_f(poles.omega_n, poles.zeta);
}