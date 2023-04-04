#ifndef FARNEBACK_CALCULATOR_H
#define FARNEBACK_CALCULATOR_H

#include "modules/computer_vision/cv.h"

#ifdef __cplusplus
extern "C" {
#endif

int opencv_example(char *img, int width, int height);
void farneback(char *img, float* output_flow, int width, int height, int width_img, int height_img);

#ifdef __cplusplus
}
#endif

#endif // FARNEBACK_CALCULATOR_H

