//
// Created by anish on 17-03-21.
//

#ifndef PAPARAZZI_GROUND_DETECTION_TESTING_H
#define PAPARAZZI_GROUND_DETECTION_TESTING_H

#endif //PAPARAZZI_GROUND_DETECTION_TESTING_H

extern void image_width_printer_init(void);
extern volatile int go_no_go; /*create a volatile integer to be read by navigation. 1 means it is okay to go straight, 0 means
                        it is not*/
//We want to be able to change the size of the rectangle, therefore we have to define them here such that they are
//linked with the XML

extern double BOTTOM_LENGTH_PERCENTAGE;
extern double TOP_LENGTH_PERCENTAGE;
extern double TOP_WIDTH_PERCENTAGE;
extern int WIDTH_RECT;