//
// Created by matteo on 06/08/2021.
//

#ifndef PAPARAZZI_CYCLIC_CONTROLLER_H
#define PAPARAZZI_CYCLIC_CONTROLLER_H

struct cyclic_controller_t {
  float cyclic_phase;
  float cyclic_amplitude;
  float d;
  float vel;
};

extern struct cyclic_controller_t cyclic_controller;

extern float cc_kp_gain;
extern float cc_kd_gain;
extern float max_amplitude;

void cyclic_controller_init(void);
void cyclic_controller_run(void);

#endif //PAPARAZZI_CYCLIC_CONTROLLER_H
