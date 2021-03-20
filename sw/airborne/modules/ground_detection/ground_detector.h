//
// Created by anish on 14-03-21.
//

#ifndef PAPARAZZI_GROUND_DETECTOR_H
#define PAPARAZZI_GROUND_DETECTOR_H



extern float GD_MAX_SPEED;
extern float GD_HEADING_RATE;
extern float GD_SLOW_FACTOR_FOUND;
extern float GD_SLOW_FACTOR_SEARCH;


extern void get_signal_init(void);
extern void get_signal_periodic(void);

#endif //PAPARAZZI_GROUND_DETECTOR_H