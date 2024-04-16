#ifndef PN_H
#define PN_H

#include "std.h"
#include "math/pprz_algebra_int.h"
#include "math/pprz_algebra_float.h"
#include "pprzlink/pprz_transport.h"
#include "pprzlink/pprzlink_device.h"

struct pnmessage {
  struct link_device *device;       ///< Device used for communication
  struct pprz_transport transport;  ///< Transport over communication line (PPRZ)
  uint8_t time_since_last_frame;    ///< Time since last frame
  bool enabled;                     ///< If the InterMCU communication is enabled
  bool msg_available;               ///< If we have an InterMCU message
};

struct Proportional_nav {
  struct FloatVect3 speed_des;
  struct FloatVect3 accel_des;
  struct FloatVect3 accel_des_pn;
  struct FloatVect3 los_rate;
  struct FloatVect3 pos_target;
  struct FloatVect3 pos_des;
  struct FloatVect3 speed_target;
};

extern void pn_init(void); 
extern void pn_run(void); 
extern void pn_start(void);
extern void pn_stop(void);
extern void pn_event(void);
extern void pn_parse_REMOTE_GPS_LOCAL(uint8_t *buf);
extern struct Proportional_nav *pn_info_logger(void);


#endif


