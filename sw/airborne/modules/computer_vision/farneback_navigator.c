#include "modules/computer_vision/cv.h"
#include "modules/computer_vision/farneback_navigator.h"
#include "modules/computer_vision/farneback_calculator.h"
#include "modules/computer_vision/finite_state_machine.h"


#include "modules/orange_avoider/orange_avoider.h"
#include "firmwares/rotorcraft/navigation.h"
#include "generated/airframe.h"
#include "state.h"
#include "modules/core/abi.h"
#include <time.h>
#include <stdio.h>
#include "math.h"

#define NAV_C // needed to get the nav functions like Inside...
#include "generated/flight_plan.h"

#define FARNEBACK_VERBOSE TRUE

#define PRINT(string,...) fprintf(stderr, "[farneback_navigator->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if FARNEBACK_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

#ifndef FARNEBACK_FPS
#define FARNEBACK_FPS 0       ///< Default FPS (zero means run at camera fps)
#endif

// Height and width of the window that is used to process the dense optical flow
#define WIDTH_2_PROCESS 30
#define HEIGHT_2_PROCESS 140
#define WIDTH_IMG 240
#define HEIGHT_IMG 520

// thresholds for decision making
#define SIDES_OBS_TH 1.22f
#define MIDDLE_OBS_TH_UPPER 1.3f
#define MIDDLE_OBS_TH_LOWER 0.6f
#define LEFTFLOW_TURNING_TH 1.18f
#define RIGHTFLOW_TURNING_TH 0.75f
#define MIDDLE_OBS_TH_UPPER_2_STEP 2.0f
#define MIDDLE_OBS_TH_LOWER_2_STEP 0.4f
// logging function for profiling purposes
#define LOG(x) fprintf(stderr, "LOG: %s:%d %s %lu \n", __FILE__, __LINE__, x, clock());


float output_flow[3] = {0., 0., 0.};


struct image_t *optical_flow_func(struct image_t *img, int camera_id);
struct image_t *optical_flow_func(struct image_t *img, int camera_id)
{
    if (img->type == IMAGE_YUV422) {
        // This function outputs the flow in the left, middle and right area
        farneback((char *) img->buf, output_flow, WIDTH_2_PROCESS, HEIGHT_2_PROCESS, img->w, img->h);
    }
    // Here we run the finite state machine that is responsible for the decision making
    struct OpticalFlow flow = run_fsm(output_flow[0], output_flow[1], output_flow[2]);
    // Some debugging visualization functions:
    image_editing(img, flow);
    image_cover(img);
    return img;
}

void calc_action_optical_flow_init(void)
{
    cv_add_to_device(&FARNEBACK_CAMERA, optical_flow_func, FARNEBACK_FPS, 0);
}

// Here are some additional visualization functions

void image_editing(struct image_t *img, struct OpticalFlow flow) {
  uint8_t *buffer = img->buf;
    auto width_img = img->w;
    auto height_img = img->h;
    auto width = WIDTH_2_PROCESS;
    auto height = HEIGHT_2_PROCESS;

    // Here the areas are defined that need to be covered by stripes for respective obstacle positions
    int row_start = width_img/2 - width/2; int row_end = width_img/2 + width/2;
    int left_start = height_img/2 - height/2; int left_end = height_img/2;
    int middle_start = height_img/2 - height/4; int middle_end = height_img/2 + height/4 ;
    int right_start = height_img/2; int right_end = height_img/2 + height/2;
    int chosen_start; int chosen_end;

    // Here the correct area is selected
    if(flow.right_left_ratio < 1/SIDES_OBS_TH){
        chosen_start = right_start;
        chosen_end = right_end;
    }
    else if(flow.right_left_ratio > SIDES_OBS_TH){
        chosen_start = left_start;
        chosen_end = left_end;

    }
    else if ((flow.middle_divergence > MIDDLE_OBS_TH_UPPER) || (flow.middle_divergence < MIDDLE_OBS_TH_LOWER)){
        chosen_start = middle_start;
        chosen_end = middle_end;
    }
 // L_R = 0 means obstacle left
    for (uint16_t y = chosen_start; y < chosen_end; y++) {
        for (uint16_t x = row_start; x < row_end; x++) {
            uint8_t *yp, *up, *vp;
            // This will cover the requested section with green stripes
            if (x % 4 == 0) {
                // Even x
                up = &buffer[y * 2 * img->w + 2 * x];      // U
                yp = &buffer[y * 2 * img->w + 2 * x + 1];  // Y1
                vp = &buffer[y * 2 * img->w + 2 * x + 2];  // V
                //yp = &buffer[y * 2 * img->w + 2 * x + 3]; // Y2
                *yp = 0;
                *up = 0;
                *vp = 0;
            } else {
                // Uneven x
                up = &buffer[y * 2 * img->w + 2 * x - 2];  // U
                vp = &buffer[y * 2 * img->w + 2 * x];      // V
                yp = &buffer[y * 2 * img->w + 2 * x + 1];  // Y2
            }
        }
    }

}

void image_cover(struct image_t *img){
    // This function covers up the areas that also cannot be seen by our algorithm
    uint8_t *buffer = img->buf;
    float square_1[4] = {0,img->h/2 - HEIGHT_2_PROCESS/2, 0, img->w};
    float square_2[4] = {img->h/2 + HEIGHT_2_PROCESS/2,img->h,0,img->w};
    float square_3[4] = {img->h/2 - HEIGHT_2_PROCESS/2,img->h/2 + HEIGHT_2_PROCESS/2,0,img->w/2 - WIDTH_2_PROCESS/2}; //[160,360,0,95];
    float square_4[4] = {img->h/2 - HEIGHT_2_PROCESS/2,img->h/2 + HEIGHT_2_PROCESS/2,img->w/2 + WIDTH_2_PROCESS/2,img->w}; // [160,360,145,img->w];
    float* squares[4] = {square_1,square_2,square_3,square_4};
    for (int chosen = 0;chosen<4;chosen++){
        float* ptr = squares[chosen];
        for (uint16_t y = ptr[0]; y < ptr[1]; y++) {
            for (uint16_t x = ptr[2]; x < ptr[3]; x++) {
                uint8_t *yp, *up, *vp;
                if (x % 2 == 0) {
                    // Even x
                    up = &buffer[y * 2 * img->w + 2 * x];      // U
                    yp = &buffer[y * 2 * img->w + 2 * x + 1];  // Y1
                    vp = &buffer[y * 2 * img->w + 2 * x + 2];  // V
                    //yp = &buffer[y * 2 * img->w + 2 * x + 3]; // Y2
                    *yp = 255;
                    *up = 255;
                    *vp = 255;
                } else {
                    // Uneven x
                    up = &buffer[y * 2 * img->w + 2 * x - 2];  // U
                    //yp = &buffer[y * 2 * img->w + 2 * x - 1]; // Y1
                    vp = &buffer[y * 2 * img->w + 2 * x];      // V
                    yp = &buffer[y * 2 * img->w + 2 * x + 1];  // Y2
                    *yp = 255;
                    *up = 255;
                    *vp = 255;
                }
            }
        }
    }
}
