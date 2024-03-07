/*
 * @file modules/computer_vision/simple_cam_test.h
 * Takes half of the img and sets it to black
 * if you are over the defined color_object or not
 */

#ifndef SIMPLE_CAM_TEST_CV_H
#define SIMPLE_CAM_TEST_CV_H

#include <stdint.h>
#include <stdbool.h>

// Module settings

// defines how much of the image is inspected
extern float height_ratio; // i guess we can also use float here


// can we also communicate the other way?
// and get this delete_flag as input?
// extern bool delete_flag;


// Module functions
extern void cam_test_init(void);
extern void cam_test_periodic(void);

#endif /* SIMPLE_CAM_TEST_CV_H */