#include "modules/computer_vision/cv.h"
#include "optical_flow_group3/optical_flow_group3.h"
#include "stdio.h"
#include "optical_flow_group3/calculate_optical_flow_group3.h"

#ifndef DETECT_OPTICAL_FLOW_FPS
#define DETECT_OPTICAL_FLOW_FPS 0
#endif
#define WIDTH_2_PROCESS 200
#define HEIGHT_2_PROCESS 50


float output_flow[2];

struct image_t *optical_flow_func(struct image_t *img, uint8_t camera_id);
struct image_t *optical_flow_func(struct image_t *img, uint8_t camera_id)
{
    // action act = STANDBY;
    if (img->type == IMAGE_YUV422) {
        farneback((char *) img->buf, output_flow, WIDTH_2_PROCESS, HEIGHT_2_PROCESS, img->w, img->h);
    }
    return img;
}

void calc_action_optical_flow_init(void)
{
    cv_add_to_device(&DETECT_OPTICAL_FLOW_CAMERA, optical_flow_func, DETECT_OPTICAL_FLOW_FPS, 0);
    output_flow[0] = 0.0;
    output_flow[1] = 0.0;
}

void calc_action_optical_flow_periodic(void)
{
    printf("out: %f",output_flow[0]);
}
