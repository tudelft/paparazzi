#include "modules/computer_vision/cv.h"
#include "optical_flow_group3/optical_flow_group3.h"
#include "stdio.h"

#ifndef DETECT_OPTICAL_FLOW_FPS
#define DETECT_OPTICAL_FLOW_FPS 0
#endif
#define WIDTH_2_PROCESS 200
#define HEIGHT_2_PROCESS 50



struct image_t *optical_flow_func(struct image_t *img, uint8_t camera_id);
struct image_t *optical_flow_func(struct image_t *img, uint8_t camera_id)
{
    // action act = STANDBY;
    if (img->type == IMAGE_YUV422) {
        //here we need to put the code
        int a =0;
        // calculate_flow_areas(img, optical_flows_group3);
        // act = FORWARD; //calc_action_optical_flow_periodic(void)(char *) img->buf, HEIGHT_2_PROCESS, WIDTH_2_PROCESS, img->h, img->w);
        //ABI event: sends optical_flows
    }
    return img;
}

void calc_action_optical_flow_init(void)
{
    cv_add_to_device(&DETECT_OPTICAL_FLOW_CAMERA, optical_flow_func, DETECT_OPTICAL_FLOW_FPS, 0);

}

void calc_action_optical_flow_periodic(void)
{
    int a =0;
}
