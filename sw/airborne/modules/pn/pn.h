// pn.h

#ifndef PN_H
#define PN_H

#include "math/pprz_algebra_float.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "pprzlink/pprz_transport.h"
#include "pprzlink/pprzlink_device.h"

struct pnmessage {
  struct link_device *device;       ///< Device used for communication
  struct pprz_transport transport;  ///< Transport over communication line (PPRZ)
  uint8_t time_since_last_frame;    ///< Time since last frame
  bool enabled;                     ///< If the InterMCU communication is enabled
  bool msg_available;               ///< If we have an InterMCU message
};

typedef enum {
  PN_MODE_FRPN,
  PN_MODE_GRTPN,
  PN_MODE_NEURAL
} pn_mode_t;


void pn_init(void);
void pn_start(void);
void pn_run(void);
void pn_stop(void);
extern void pn_parse_REMOTE_GPS_LOCAL(uint8_t *buf);
extern void pn_event(void);
void pn_set_mode(pn_mode_t m);

struct Proportional_nav {
  struct FloatVect3 pos_target;
  struct FloatVect3 vel_target;
  struct FloatVect3 accel_command;
};
struct Proportional_nav *pn_info_logger(void);

#endif // PN_H
