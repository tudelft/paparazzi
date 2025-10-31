#ifndef POLE_PLACEMENT_H
#define POLE_PLACEMENT_H

#include "math/pprz_algebra_float.h"

struct PolesOrder3Vect2 {
  struct FloatVect3 omega_n;
  struct FloatVect3 zeta;
  struct FloatVect3 p1;
};

struct PolesOrder3Vect2 {
  struct FloatVect2 omega_n;
  struct FloatVect2 zeta;
  struct FloatVect2 p1;
};

struct PolesOrder3 {
  float omega_n;
  float zeta;
  float p1;
};

struct PolesOrder2Vect3 {
  struct FloatVect3 omega_n;
  struct FloatVect3 zeta;
};

struct PolesOrder2Vect2 {
  struct FloatVect2 omega_n;
  struct FloatVect2 zeta;
};

struct PolesOrder2 {
  float omega_n;
  float zeta;
};

struct GainsOrder2Vect3 {
  struct FloatVect3 k1;
  struct FloatVect3 k2;
};

struct GainsOrder2Vect2 {
  struct FloatVect2 k1;
  struct FloatVect2 k2;
};

struct GainsOrder2 {
  float k1;
  float k2;
};

struct GainsOrder3Vect3 {
  struct FloatVect3 k1;
  struct FloatVect3 k2;
  struct FloatVect3 k3;
};

struct GainsOrder3Vect2 {
  struct FloatVect2 k1;
  struct FloatVect2 k2;
  struct FloatVect2 k3;
};

struct GainsOrder3 {
  float k1;
  float k2;
  float k3;
};

struct GainsOrder3Vect3 compute_gains_order_3_vect_3(const struct PolesOrder3Vect3* poles);
struct GainsOrder3Vect2 compute_gains_order_3_vect_2(const struct PolesOrder3Vect2* poles);
struct GainsOrder3 compute_gains_order_3(const struct PolesOrder3* poles);
struct GainsOrder2Vect3 compute_gains_order_2_vect_3(const struct PolesOrder2Vect3* poles);
struct GainsOrder2Vect2 compute_gains_order_2_vect_2(const struct PolesOrder2Vect2* poles);
struct GainsOrder2 compute_gains_order_2(const struct PolesOrder2* poles);

#endif // POLE_PLACEMENT_H