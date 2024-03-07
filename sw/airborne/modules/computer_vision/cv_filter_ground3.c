
#include "modules/computer_vision/cv_filter_ground3.h"
#include "modules/computer_vision/cv.h"
#include "modules/core/abi.h"
#include "std.h"

#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include "pthread.h"


#define PRINT(string,...) fprintf(stderr, "[cv_filter_ground3->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)


#ifndef FILTER_GROUND3_FPS
#define FILTER_GROUND3_FPS 0 ///< Default FPS (zero means run at camera fps)
#endif

static pthread_mutex_t mutex;

// filter settings
uint8_t cod_lum_min = 0;
uint8_t cod_lum_max = 0;
uint8_t cod_cb_min = 0;
uint8_t cod_cb_max = 0;
uint8_t cod_cr_min = 0;
uint8_t cod_cr_max = 0;


uint8_t lower_pix = 50;

bool cod_draw = false;

struct ground_filter_msg_t {
  uint16_t count_left;
  uint16_t count_center;
  uint16_t count_right;
  bool updated;
};

struct ground_filter_msg_t ground_filter_msg;

void get_pix(uint8_t **buffer_ptr,const uint16_t x, const uint16_t y, const uint16_t w, const uint16_t h, uint8_t **yp,uint8_t **up, uint8_t **vp) {
    if (x % 2 == 0) {
        // Even x
        *up = buffer_ptr[y * 2 * w + 2 * x];      // U
        *yp = buffer_ptr[y * 2 * w + 2 * x + 1];  // Y1
        *vp = buffer_ptr[y * 2 * w + 2 * x + 2];  // V
        //yp = &buffer[y * 2 * img->w + 2 * x + 3]; // Y2
      } else {
        // Uneven x
        *up = buffer_ptr[y * 2 * w + 2 * x - 2];  // U
        //yp = &buffer[y * 2 * img->w + 2 * x - 1]; // Y1
        *vp = buffer_ptr[y * 2 * w + 2 * x];      // V
        *yp = buffer_ptr[y * 2 * w + 2 * x + 1];  // Y2
      }
}


static struct image_t *cam_callback(struct image_t *img);
static struct image_t *cam_callback(struct image_t *img __attribute__((unused))) {

  uint16_t cnt_center = 0;
  uint16_t cnt_left = 0;
  uint16_t cnt_right = 0;
  uint8_t *buffer = img->buf;

  // Go through all the pixels
  uint8_t *yp, *up, *vp;
  //PRINT("lower_pix = %d, img->h /3: %d", lower_pix, img->h/3);
  for (uint16_t x = 0; x < lower_pix; x++) {
    for (uint8_t i = 0; i < 3; i++){
      for (uint16_t y = i*(img->h/3); y < (i+1) * (img->h/3); y++) {  
        
        //get_pix(&buffer, x, y,img->w, img->h, &yp, &up, &vp);
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
        if ( (*yp >= cod_lum_min) && (*yp <= cod_lum_max) &&
            (*up >= cod_cb_min ) && (*up <= cod_cb_max ) &&
            (*vp >= cod_cr_min ) && (*vp <= cod_cr_max )) {
          
          if (cod_draw) {
            *yp = 255; // set bright color to test it
          }
          switch (i)
          {
          case 0: // add to left
            cnt_left++;
            break;
          case 1: // add to center
            cnt_center++;
            break;
          case 2: // add to right
            cnt_right++;
            break;
          default:
            break;
          }
        }
      }
    }
  }

  if (cod_draw) {
    //PRINT("drawing black");
    // for (uint16_t y = 0; y < img->h - lower_pix; y++) {

    //   for (uint16_t x = 0; x < img->w; x ++) {
    //     PRINT("pix x:%d y:%d, w:%d,h:%d", x,y, img->w, img->h);
    //     get_pix(&buffer, x, y,img->w, img->h, &yp, &up, &vp);
        
    //     // draw dark the ignored area
    //     *yp = 0;
    //     // *up = 0;
    //     // *vp = 0;
    //   }
    // }
    for (uint16_t x = lower_pix; x < img->w; x++) {
      for (uint16_t y = 0; y < img->h; y++) {
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

        // set to black
        *yp = 16;
        *up = 128;
        *vp = 128;
      }
    }
  }

  pthread_mutex_lock(&mutex);
  ground_filter_msg.count_left = cnt_left;
  ground_filter_msg.count_center = cnt_center;
  ground_filter_msg.count_right = cnt_right;
  ground_filter_msg.updated = true;
  pthread_mutex_unlock(&mutex);

  //PRINT("updated, lower pix is %d", lower_pix);
  return img;
}


extern void filter_ground_init(void) {
  ground_filter_msg.count_center = 0;
  ground_filter_msg.count_center = 0;
  ground_filter_msg.count_right = 0;
  ground_filter_msg.updated = false;

  pthread_mutex_init(&mutex, NULL);

  cod_lum_min = FILTER_GROUND3_LUM_MIN;
  cod_lum_max = FILTER_GROUND3_LUM_MAX;
  cod_cb_min = FILTER_GROUND3_CB_MIN;
  cod_cb_max = FILTER_GROUND3_CB_MAX;
  cod_cr_min = FILTER_GROUND3_CR_MIN;
  cod_cr_max = FILTER_GROUND3_CR_MAX;
  lower_pix = FILTER_GROUND3_LOWER_PIX;
  cod_draw = FILTER_GROUND3_DRAW;
  #define CALLBACK_ID 0
  // FILTER_GROUND3_CAM will be defined in the xml file
  cv_add_to_device(&FILTER_GROUND3_CAM, cam_callback, FILTER_GROUND3_FPS, 0);
  PRINT("INIT\n");
}

void filter_ground_periodic(void)
{ 
  // is it really necessary to create a new struct here?
  static struct ground_filter_msg_t local_msg;
  // reading from ground_filter, that is the reason why 
  pthread_mutex_lock(&mutex);
  memcpy(&local_msg, &ground_filter_msg, sizeof(struct ground_filter_msg_t));
  pthread_mutex_unlock(&mutex);
  //PRINT("periodic");
  if (local_msg.updated) {
    // it seems we have to send the plain values
    AbiSendMsgGROUND_FILTER_DETECTION(GROUND_FILTER_DETECTION_ID,
                                local_msg.count_left,
                                local_msg.count_center,
                                local_msg.count_right, 0);

    PRINT("sending msg %d|%d|%d", local_msg.count_left,
                                local_msg.count_center,
                                local_msg.count_right);
    local_msg.updated = false;
  }
}
