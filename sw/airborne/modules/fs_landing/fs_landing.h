//
// Created by matteo on 09/05/2020.
//

#ifndef PAPARAZZI_FS_LANDING_H
#define PAPARAZZI_FS_LANDING_H

#include "paparazzi.h"
#include "generated/airframe.h"
#include "subsystems/radio_control.h"
#include "mcu_periph/sys_time.h"

#ifndef SERVO_S_THROTTLE_LEFT
#error "Airframe servo name must match module source (S_THROTTLE_LEFT)"
#endif
#ifndef SERVO_S_THROTTLE_RIGHT
#error "Airframe servo name must match module source (S_THROTTLE_RIGHT)"
#endif
#ifndef SERVO_S_ELEVON_LEFT
#error "Airframe servo name must match module source (S_ELEVON_LEFT)"
#endif
#ifndef SERVO_S_ELEVON_RIGHT
#error "Airframe servo name must match module source (S_ELEVON_RIGHT)"
#endif

struct fs_landing_t {
    int32_t commands[ACTUATORS_NB];
};
extern struct fs_landing_t fs_landing;

/* External used functions */
extern void fs_landing_init(void);
extern void fs_landing_run(void);

extern void fs_landing_set_actuator_values(void);

bool is_fs_landing_active(void);

#endif //PAPARAZZI_FS_LANDING_H
