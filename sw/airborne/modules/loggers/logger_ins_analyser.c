#include "state.h"

#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"

static void send_ins_analyser(struct transport_tx *trans, struct link_device *dev)
{
  struct NedCoor_f *pned = stateGetPositionNed_f();
  struct NedCoor_f *vned = stateGetSpeedNed_f();
  struct NedCoor_f *aned = stateGetAccelNed_f();
  struct Int32Vect3 *abody = stateGetAccelBody_i();
 
  pprz_msg_send_INS_ANALYSER(trans, dev, AC_ID,
                         &(pned->x),
                         &(pned->y),
                         &(pned->z),
                         &(vned->x),
                         &(vned->y),
                         &(vned->z),
                         &(aned->x),
                         &(aned->y),                         
                         &(aned->z),
                         &(abody->x),
                         &(abody->y),
                         &(abody->z)                                                  
                         );
};

#endif

void logger_ins_analyser()
{
  #if PERIODIC_TELEMETRY
    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_INS_ANALYSER, send_ins_analyser);
  #endif
};