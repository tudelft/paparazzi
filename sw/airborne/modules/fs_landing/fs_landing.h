//
// Created by matteo on 09/05/2020.
//

#ifndef PAPARAZZI_FS_LANDING_H
#define PAPARAZZI_FS_LANDING_H

#include "paparazzi.h"
#include "generated/airframe.h"

// Servo names must match in the xml
#ifndef S_THROTTLE_LEFT
// TODO Complain during compilation, something might cause controller to brake
#endif
#ifndef S_THROTTLE_RIGHT
// TODO Complain during compilation, something might cause controller to brake
#endif
#ifndef S_ELEVON_LEFT
// TODO Complain during compilation, something might cause controller to brake
#endif
#ifndef S_ELEVON_RIGHT
// TODO Complain during compilation, something might cause controller to brake
#endif

#define NB_ACT_FS 4

struct fs_landing_t {
    int32_t commands[NB_ACT_FS];
};
extern struct fs_landing_t fs_landing;

/* External used functions */
extern void fs_landing_init(void);
extern void fs_landing_run(void);

extern void fs_landing_set_actuator_values(void);

#endif //PAPARAZZI_FS_LANDING_H
