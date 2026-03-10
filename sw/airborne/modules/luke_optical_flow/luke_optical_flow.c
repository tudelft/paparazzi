/*
 * MAV Course 2026 - Luke Optical Flow Module
 *
 * Implements a Paparazzi module that:
 *  1. Registers a VIDEO CALLBACK (Section 4.2.3) that runs in a separate
 *     camera thread to compute optical flow divergence using Lucas-Kanade.
 *  2. Uses a MUTEX (Section 4.2.3) to safely pass the result from the
 *     camera thread to the autopilot (periodic) thread.
 *  3. Sends an ABI VISUAL_DETECTION MESSAGE (Section 4.2.4) so the
 *     orange_avoider (or any other subscriber) can react to the obstacle.
 *  4. Exposes the divergence threshold as a GCS SETTING (Section 4.2.7)
 *     so it can be tuned live without recompiling.
 */

#include "luke_optical_flow.h"

#include <stdio.h>    // printf for debug output (Section 4.3.1)
#include <pthread.h>  // pthread_mutex_t for thread safety

#include "modules/core/abi.h"              // AbiSendMsgVISUAL_DETECTION
#include "modules/computer_vision/cv.h"   // cv_add_to_device
#include "opticflow_module.h"             // opticflow_t, opticflow_result_t,
                                          // opticflow_calc_init, opticflow_calc_frame
#include "modules/pose_history/pose_history.h" // get_rotation_at_timestamp (for derotation)

// ─── Configuration ────────────────────────────────────────────────────────────
// The camera to run optical flow on. Set in the airframe XML with:
//   <define name="LUKE_OF_CAMERA" value="front_camera"/>
// Defaults to front_camera if not defined.
#ifndef LUKE_OF_CAMERA
#define LUKE_OF_CAMERA front_camera
#endif

// Maximum FPS for the video callback. 0 means "run at the camera's native FPS".
#ifndef LUKE_OF_FPS
#define LUKE_OF_FPS 0
#endif

// ABI sender ID for our outgoing VISUAL_DETECTION messages.
// The orange_avoider listens on ABI_BROADCAST by default, so this works.
// You can pin a specific ID in your airframe if needed.
#ifndef LUKE_OF_VISUAL_DETECTION_ID
#define LUKE_OF_VISUAL_DETECTION_ID ABI_BROADCAST
#endif

// ─── GCS-tunable parameter (Section 4.2.7) ────────────────────────────────────
// Divergence above this value → obstacle detected (quality=1).
// Exposed via <dl_setting> in the XML; the GCS writes directly to this variable.
float luke_of_divergence_threshold = 0.3f;

// ─── Thread-shared state ──────────────────────────────────────────────────────
// opticflow_calc_init expects an array, so we declare one element here.
static struct opticflow_t luke_of_opticflow[1];

// This struct holds the latest result from the video thread.
// It is protected by luke_of_mutex — never access it without holding the lock.
static struct opticflow_result_t luke_of_result;

// Flag set by the video thread when a fresh result is ready,
// cleared by the periodic thread after it has consumed the result.
static bool luke_of_got_result = false;

// The mutex that guards luke_of_result and luke_of_got_result.
static pthread_mutex_t luke_of_mutex;

// ─── Video callback (runs in the camera thread) ───────────────────────────────
//
// Paparazzi calls this function from a dedicated camera thread every time a
// new frame arrives. Because this runs IN PARALLEL with the autopilot loop,
// it MUST NOT write to shared variables without locking the mutex first.
//
// The function signature is fixed by Paparazzi's cv.h:
//   struct image_t *callback(struct image_t *img, uint8_t camera_id)
// You must return the img pointer at the end so the next subscriber gets it.
static struct image_t *luke_of_calc(struct image_t *img,
                                    uint8_t camera_id __attribute__((unused)))
{
  // Copy the drone's attitude at this frame's exact timestamp.
  // The opticflow calculator uses the Euler angles to remove the rotational
  // component from the flow (derotation), leaving only translational flow.
  struct pose_t pose = get_rotation_at_timestamp(img->pprz_ts);
  img->eulers = pose.eulers;

  // opticflow_calc_frame runs Lucas-Kanade + divergence estimation.
  // It returns true when a valid result is available (not on the very first
  // frame, since it needs two frames to compute flow).
  static struct opticflow_result_t temp; // static: corners are kept between calls
  if (opticflow_calc_frame(&luke_of_opticflow[0], img, &temp)) {
    // We have a valid result. Copy it to the shared variable behind the mutex.
    // Keep the lock as short as possible — just the copy, nothing else.
    pthread_mutex_lock(&luke_of_mutex);
    luke_of_result = temp;
    luke_of_got_result = true;
    pthread_mutex_unlock(&luke_of_mutex);
  }

  // Return the (possibly annotated) image for the next camera subscriber.
  return img;
}

// ─── Init function (called once at autopilot startup) ─────────────────────────
//
// This is specified in the XML as <init fun="luke_optical_flow_init()"/>.
// Use it to initialise data structures and register the video callback.
// Do NOT do heavy computation here.
void luke_optical_flow_init(void)
{
  // Initialise the mutex before any thread can use it.
  pthread_mutex_init(&luke_of_mutex, NULL);

  // Initialise the opticflow calculator's internal state (sets all parameters
  // like window size, FAST9 thresholds, etc. from the OPTICFLOW_* defines).
  opticflow_calc_init(luke_of_opticflow);

  // Register our video callback with Paparazzi's camera framework.
  // From this point on, luke_of_calc() will be called in the camera thread
  // every time a new frame arrives from LUKE_OF_CAMERA at up to LUKE_OF_FPS.
  cv_add_to_device(&LUKE_OF_CAMERA, luke_of_calc, LUKE_OF_FPS, 0);
}

// ─── Periodic function (called at 4 Hz in the autopilot thread) ───────────────
//
// This is specified in the XML as <periodic fun="..." freq="4"/>.
// It runs in the MAIN autopilot thread, so it must not take too long.
// Here we read the latest divergence, log it, and decide whether an obstacle
// is present by comparing against the tunable threshold.
void luke_optical_flow_periodic(void)
{
  float local_div;
  bool got;

  // Step 1: safely copy the shared state under the lock.
  // We copy to local variables immediately so we release the lock fast,
  // allowing the video thread to write its next result without waiting.
  pthread_mutex_lock(&luke_of_mutex);
  local_div = luke_of_result.div_size; // divergence in 1/seconds
  got = luke_of_got_result;
  luke_of_got_result = false;          // consume the flag
  pthread_mutex_unlock(&luke_of_mutex);

  // Step 2: if no new result since last call, skip (avoids acting on stale data).
  if (!got) { return; }

  // Step 3: print to terminal for debugging (Section 4.3.1).
  // Visible in the Paparazzi Center terminal during simulation.
  printf("[luke_of] divergence = %f  (threshold = %f)\n",
         local_div, luke_of_divergence_threshold);

  // Step 4: threshold check — obstacle detected if divergence is too large.
  // quality=1 signals "obstacle present", quality=0 signals "path clear".
  int32_t quality = (local_div > luke_of_divergence_threshold) ? 1 : 0;

  // Step 5: send the VISUAL_DETECTION ABI message (Section 4.2.4).
  // The orange_avoider subscribes to this message type and turns away when
  // quality (mapped to color_count inside the avoider) is above its own
  // threshold. The pixel_x/y/width/height fields are unused here (set to 0).
  AbiSendMsgVISUAL_DETECTION(LUKE_OF_VISUAL_DETECTION_ID,
                              0,       // pixel_x  (not used)
                              0,       // pixel_y  (not used)
                              0,       // pixel_width
                              0,       // pixel_height
                              quality, // quality → used as obstacle flag
                              0);      // extra
}
