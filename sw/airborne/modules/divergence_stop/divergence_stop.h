#ifndef DIVERGENCE_STOP_H
#define DIVERGENCE_STOP_H

extern void divergence_stop_init(void);
extern void divergence_stop_periodic(void);
#ifndef DIVERGENCE_MODE_FORWARD
#define DIVERGENCE_MODE_FORWARD 1
#endif

#ifndef DIVERGENCE_MODE_STOP
#define DIVERGENCE_MODE_STOP 0
#endif

extern int navigation_state;

#endif
