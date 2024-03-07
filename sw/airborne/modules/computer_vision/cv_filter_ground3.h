/**
 * @file modules/computer_vision/cv_filter_ground3.h
 * Divides the lower part of the image in 3 and returns the number of pixels counted in each area 
 */

#ifndef FILTER_GROUND3_CV_H
#define FILTER_GROUND3_CV_H

#include <stdint.h>
#include <stdbool.h>

// Module settings
extern uint8_t cod_lum_min;
extern uint8_t cod_lum_max;
extern uint8_t cod_cb_min;
extern uint8_t cod_cb_max;
extern uint8_t cod_cr_min;
extern uint8_t cod_cr_max;

// takes a look to the "lower_pix" pixels
extern uint8_t lower_pix;

extern bool cod_draw;


// Module functions
extern void filter_ground_init(void);
extern void filter_ground_periodic(void);

#endif /* FILTER_GROUND3_CV_H */
