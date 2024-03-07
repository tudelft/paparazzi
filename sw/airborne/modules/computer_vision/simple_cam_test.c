// Copyright 2024 @ivrolan
// Own header
#include "modules/computer_vision/simple_cam_test.h"
#include "modules/computer_vision/cv.h"
#include "modules/core/abi.h"
#include "std.h"

#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include "pthread.h"


#define PRINT(string,...) fprintf(stderr, "[simple_cam_test->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)


// mutex to read from the cam
static pthread_mutex_t mutex;

// info to send
bool test_flag_to_send;


static struct image_t *cam_callback(struct image_t *img);
static struct image_t *cam_callback(struct image_t *img __attribute__((unused)))
{
  uint8_t *buffer = img->buf;
  
  // Go through all the pixels
  // TODO use the setting height_ratio
  // to screen only through the part of the img
  for (uint16_t y = 0; y < img->h; y++) {
    for (uint16_t x = 0; x < img->w; x ++) {
      // Check if the color is inside the specified values
      uint8_t *yp, *up, *vp;
      // get color YUV
      if (x % 2 == 0) {
        // Even x
        up = &buffer[y * 2 * img->w + 2 * x];      // U
        yp = &buffer[y * 2 * img->w + 2 * x + 1];  // Y1
        vp = &buffer[y * 2 * img->w + 2 * x + 2];  // V
        //yp = &buffer[y * 2 * img->w + 2 * x + 3]; // Y2
      } else {
        // Uneven x
        up = &buffer[y * 2 * img->w + 2 * x - 2];  // U
        //yp = &buffer[y * 2 * img->w + 2 * x - 1]; // Y1
        vp = &buffer[y * 2 * img->w + 2 * x];      // V
        yp = &buffer[y * 2 * img->w + 2 * x + 1];  // Y2
      }

      // set brightness to a very low value
      *yp = 30;
      // 
    }
  }

  PRINT("CameraCallback");
  return img;
}

// in init define and initialize what is needed
// e.g. the callbacks of the cam
extern void cam_test_init(void) {
  test_flag_to_send = false;

  #define FPS 0
  #define CALLBACK_ID 0
  // CAM_TEST_CAM will be defined in the xml file
  cv_add_to_device(&CAM_TEST_CAM, cam_callback, 0, 0);
  PRINT("INIT\n");

}


// depending on the data computed in the callbacks, send info
extern void cam_test_periodic(void) {
  //AbiSendMsgTEST_DETECTION(CAM_TEST_ID, )
  PRINT("PERIODIC");
}

