/**
 * This file is created by TEAM 2 from 2023 for the AFMAV course
 */

#include "modules/computer_vision/cv.h"
#include "of_avoidance.h"
#include "modules/core/abi.h"
#include "pthread.h"
#include <stdio.h>

#ifndef OPENCVDEMO_FPS
#define OPENCVDEMO_FPS 0       // Default FPS (zero means run at camera fps)
#endif
PRINT_CONFIG_VAR(OPENCVDEMO_FPS)

static pthread_mutex_t mutex; // Handles the lock of the memory

// Stores the output from the image processing which needs to be sent to the autopilot
struct ABI_message_type { // Define struct
    int lowest_detection_index;
    bool new_result;
};
struct ABI_message_type global_ABI_message; // Make a global var of type ABI_message_type (our custom struct)

// Function
struct image_t *opencv_func(struct image_t *img, uint8_t camera_id)
{
  if (img->type == IMAGE_YUV422) {
    // Call the `opencv_main` function in `opencv_optical_flow.cpp`. This is where all the image processing happens
    // The index of the best direction to go is returned
    int lowest_index_tmp = opencv_main((char *) img->buf, img->w, img->h);

    // Safe the output of the image processing in a global variable
    pthread_mutex_lock(&mutex);
    global_ABI_message.lowest_detection_index = lowest_index_tmp;
    global_ABI_message.new_result = true;
    pthread_mutex_unlock(&mutex);
  }
  return NULL;
}

void OF_init(void)
{
    pthread_mutex_init(&mutex, NULL);
    global_ABI_message.lowest_detection_index = 0;
    global_ABI_message.new_result = false;

    cv_add_to_device(&OPENCVDEMO_CAMERA, opencv_func, OPENCVDEMO_FPS, 0);
}

// This function is periodically called to send the output of the image processing with an ABI messages to the autopilot
// The frequency can be set in conf/modules/opencv_optical_flow.xml
void OF_periodic(void) {
    // Copy the global var to a local var
    struct ABI_message_type local_ABI_message;
    pthread_mutex_lock(&mutex);
    local_ABI_message.new_result = global_ABI_message.new_result;
    local_ABI_message.lowest_detection_index = global_ABI_message.lowest_detection_index;
    pthread_mutex_unlock(&mutex);

    // If there is a new message from the image processing
    if (local_ABI_message.new_result) {
        // ABI broadcast
        // DIVERGENCE_SAFE_HEADING_OF_AVOIDANCE_ID defined in sw/airborne/modules/core/abi_sender_divs.h
        AbiSendMsgDIVERGENCE_SAFE_HEADING(DIVERGENCE_SAFE_HEADING_OF_AVOIDANCE_ID, local_ABI_message.lowest_detection_index);

        pthread_mutex_lock(&mutex);
        global_ABI_message.new_result = false;
        pthread_mutex_unlock(&mutex);

    }

}