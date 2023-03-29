/*
 * Copyright (C) 2014 Hann Woei Ho
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file modules/computer_vision/opticflow_module.c
 * @brief Optical-flow estimation module
 *
 */


#include "opticflow_module.h"

#include <stdio.h>
#include <pthread.h>
#include "state.h"
#include "core/abi.h"

#include "lib/v4l/v4l2.h"
#include "lib/encoding/jpeg.h"
#include "lib/encoding/rtp.h"
#include "errno.h"

#include "cv.h"

/* ABI messages sender ID */
#ifndef OPTICFLOW_AGL_ID
#define OPTICFLOW_AGL_ID ABI_BROADCAST    ///< Default sonar/agl to use in opticflow visual_estimator
#endif
PRINT_CONFIG_VAR(OPTICFLOW_AGL_ID)

#ifndef OPTICFLOW_FPS
#define OPTICFLOW_FPS 0       ///< Default FPS (zero means run at camera fps)
#endif
PRINT_CONFIG_VAR(OPTICFLOW_FPS)

#define PRINT(string, ...) fprintf(stderr, "[mav_exercise->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)

/* The main opticflow variables */
struct opticflow_t opticflow;                     ///< Struct to store opticflow parameters, like two images in
												   /// sequence. Directly taken from the original opticflow_module
                                                   //  given as example
                                                   
float divg;                                         // Divergence

static bool opticflow_got_result;                ///< When we have an optical flow calculation
static pthread_mutex_t opticflow_mutex;            ///< Mutex lock for thread safety

// Not used right now but could be adapted for telemetry of our own message
/* Static functions */
struct image_t *opticflow_calculator_module(struct image_t *img);     ///< The main optical flow calculation thread

//#if PERIODIC_TELEMETRY
//#include "subsystems/datalink/telemetry.h"
///**
// * Send optical flow telemetry information
// * @param[in] *trans The transport structure to send the information over
// * @param[in] *dev The link to send the data over
// */
//static void opticflow_telem_send(struct transport_tx *trans, struct link_device *dev)
//{
//  pthread_mutex_lock(&opticflow_mutex);
//  if (opticflow_result.noise_measurement < 0.8) {
//    pprz_msg_send_OPTIC_FLOW_EST(trans, dev, AC_ID,
//                                 &opticflow_result.fps, &opticflow_result.corner_cnt,
//                                 &opticflow_result.tracked_cnt, &opticflow_result.flow_x,
//                                 &opticflow_result.flow_y, &opticflow_result.flow_der_x,
//                                 &opticflow_result.flow_der_y, &opticflow_result.vel_body.x,
//                                 &opticflow_result.vel_body.y, &opticflow_result.vel_body.z,
//                                 &opticflow_result.div_size, &opticflow_result.surface_roughness,
//                                 &opticflow_result.divergence); // TODO: no noise measurement here...
//  }
//  pthread_mutex_unlock(&opticflow_mutex);
//}
//#endif

/**
 * Initialize the optical flow module
 */
void opticflow_module_init(void)
{
  // Initialize the opticflow calculation
  opticflow_got_result = false;
  opticflow_calc_init(&opticflow);

  cv_add_to_device(&OPTICFLOW_CAMERA, opticflow_calculator_module, OPTICFLOW_FPS, 0);

// Again not used right now but could be implemented with our message
//#if PERIODIC_TELEMETRY
//  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_OPTIC_FLOW_EST, opticflow_telem_send);
//#endif

}

/**
 * Update the optical flow state for the calculation thread
 * and update the stabilization loops with the newest result
 */
void opticflow_module_run(void)
{
  pthread_mutex_lock(&opticflow_mutex);
  // Update the stabilization loops on the current calculation
  if (opticflow_got_result) {
    // __attribute__((used))
    uint32_t now_ts = get_sys_time_usec();

    // Send OF difference and divergence with our message
    AbiSendMsgOPTICAL_FLOW(FLOW_OPTICFLOW_ID,0,0,0,0,0,0.0f, divg);
    opticflow_got_result = false;
  }
  pthread_mutex_unlock(&opticflow_mutex);
}

/**
 * The main optical flow calculation thread
 * This thread passes the images trough the optical flow
 * calculator
 * @param[in] *img The image_t structure of the captured image
 * @return *img The processed image structure
 */
struct image_t *opticflow_calculator_module(struct image_t *img)
{

  // Do the optical flow calculation
        
  float temp_div;           // temp for the divergence
  if(opticflow_calc_module_call(&opticflow, img, &temp_div)){
    // Copy the result if finished
    pthread_mutex_lock(&opticflow_mutex);
    divg = temp_div;
    opticflow_got_result = true;
    pthread_mutex_unlock(&opticflow_mutex);
  }
  return img;
}
