//
// Created by anish on 17-03-21.
//

#ifndef PAPARAZZI_GROUND_DETECTION_TESTING_H
#define PAPARAZZI_GROUND_DETECTION_TESTING_H

#include <stdint.h>
#include <stdbool.h>



extern void image_width_printer_init(void);
extern volatile int go_no_go; /*create a volatile integer to be read by navigation. 1 means it is okay to go straight, 0 means
                        it is not*/
//We want to be able to change the size of the rectangle, therefore we have to define them here such that they are
//linked with the XML

extern uint8_t lum_min;
extern uint8_t lum_max;
extern uint8_t cb_min;
extern uint8_t cb_max;
extern uint8_t cr_min;
extern uint8_t cr_max;

extern double BOTTOM_LENGTH_PERCENTAGE;
extern double TOP_LENGTH_PERCENTAGE;
extern double TOP_WIDTH_PERCENTAGE;
extern int WIDTH_RECT;

#endif //PAPARAZZI_GROUND_DETECTION_TESTING_H