//
// Created by matteo on 09/05/2020.
//

#ifndef PAPARAZZI_FS_LANDING_H
#define PAPARAZZI_FS_LANDING_H

#include "paparazzi.h"
#include "std.h"
#include "state.h"
#include "generated/airframe.h"
#include "modules/radio_control/radio_control.h"
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

#if PERIODIC_TELEMETRY
#define N_DBG_VALUES 4
extern float fs_landing_dbg_values[N_DBG_VALUES];
#endif

struct fs_landing_t {
    int32_t commands[ACTUATORS_NB];
};
extern struct fs_landing_t fs_landing;

// GCS Variables
//extern uint8_t pilot_has_control;
//extern uint8_t act_identification_active;
extern uint8_t cyclic_control_active;
//extern uint8_t impulse_control_active;

extern uint8_t use_pre_spin;
extern float pre_spin_pitch_coeff;
extern float pre_spin_speed_setpoint;
extern float pre_spin_trim_percentage;
extern float err_test;

extern float v_filt;
extern float my_psi;

/* External used functions */
extern void fs_landing_init(void);
extern void fs_landing_run(void);

void horizontal_velocity_filter(void);
void my_psi_from_mag(void);
void mag_psi_offset_correction(void);
float get_matching_motl_val(int32_t val);

// Handlers for changing gcs variables
//extern void fs_landing_pilot_control_handler(uint8_t active);
//extern void fs_landing_actuator_id_handler(uint8_t active);

extern void fs_landing_set_actuator_values(void);

bool is_fs_landing_active(void);
bool pre_spin_actuator_values(void);

#endif //PAPARAZZI_FS_LANDING_H
