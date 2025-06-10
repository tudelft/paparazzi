#ifndef CTBR_PN_H
#define CTBR_PN_H

#include "math/pprz_algebra_float.h"
#include "pprzlink/pprz_transport.h"
#include "pprzlink/pprzlink_device.h"

// Target info handling
void pn_parse_TARGET_INFO(uint8_t *buf);

// internal function declaration
void ctbr_run(void);

// Core PN interface (same as acc_pn)
void pn_init(void);
void pn_start(void);
void pn_stop(void);
void pn_run(void);
void pn_event(void);

#endif // CTBR_PN_H
