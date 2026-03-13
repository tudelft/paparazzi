#ifndef CUSTOM_DETECT_COLOR_OBJECT_H
#define CUSTOM_DETECT_COLOR_OBJECT_H

#include "std.h"
#include "modules/computer_vision/cv.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initialize the custom color obstacle detector module.
 */
void custom_detect_color_object_init(void);

/**
 * Periodic function that publishes the latest CUSTOM_DETECTION ABI message.
 */
void custom_detect_color_object_periodic(void);

#ifdef __cplusplus
}
#endif

#endif // CUSTOM_DETECT_COLOR_OBJECT_H