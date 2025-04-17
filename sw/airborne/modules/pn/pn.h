// pn.h

#ifndef PN_H
#define PN_H

#include "math/pprz_algebra_float.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"

typedef enum {
  PN_MODE_FRPN = 0,
  PN_MODE_GRTPN
} pn_mode_t;


void pn_init(void);
void pn_start(void);
void pn_run(void);
void pn_stop(void);

void pn_set_mode(pn_mode_t m);

struct Proportional_nav {
  struct FloatVect3 pos_target;
  struct FloatVect3 vel_target;
  struct FloatVect3 accel_command;
};
const struct Proportional_nav *pn_info_logger(void);

#endif // PN_H
