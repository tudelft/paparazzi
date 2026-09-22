#ifndef CATIA_LWIR_CAM_PIPE_H
#define CATIA_LWIR_CAM_PIPE_H

/**
 * @file lwir_cam_pipe.h
 * @brief Parent-side protocol for CATIA's persistent Tiny1-C LWIR capture server.
 * @details CATIA uses this interface to keep the thermal stream warmed between shots.
 * A line protocol carries capture and geolocation requests; the implementation handles
 * bounded server lifecycle, response validation, and capture timing evidence. CATIA's
 * recovery worker owns retry policy.
 */

#include <stddef.h>
#include <signal.h>
#include "capture_timing.h"

/** @brief Start and qualify the persistent LWIR capture server.
 * @param unused Reserved common backend argument.
 * @return 0 after the server reports ready, otherwise -1. */
int lwir_cam_pipe_init(const char *unused);
/** @brief Start the server but abort promptly when CATIA begins shutdown.
 * @param unused Reserved common backend argument.
 * @param keep_running Signal-safe flag that remains nonzero while startup may continue.
 * @return 0 after the server reports ready, otherwise -1 after cleanup. */
int lwir_cam_pipe_init_cancelable(const char *unused,
                                  const volatile sig_atomic_t *keep_running);
/** @brief Override the output directory for an isolated standalone benchmark. */
void lwir_cam_pipe_set_photo_directory(const char *directory);
/** @brief Open, stabilize, and close the sensor without writing an image.
 * @return 0 after a stable warmup frame, otherwise -1. */
int lwir_cam_pipe_warmup(void);
/** @brief Select whether successful captures keep an explicit .raw companion.
 * @param enabled Nonzero to request native raw output. */
void lwir_cam_pipe_set_native_raw(int enabled);
/** @brief Set the optional calibration file used by server-side geolocation.
 * @param path Nonempty calibration YAML path.
 * @return 0 on acceptance or -1 for an invalid path. */
int lwir_cam_pipe_set_calibration(const char *path);
/** @brief Process a mock thermal image for hardware-free local testing.
 * @param filename Input/output image path.
 * @return 0 on success or -1 on processing failure. */
int lwir_cam_pipe_process_mock(char *filename);
/** @brief Add thermal hotspot/geolocation metadata through the running server.
 * @param filename JPEG to update.
 * @return 0 on success or -1 if the operation cannot be completed. */
int lwir_cam_pipe_geolocate(char *filename);
/** @brief Return the delay of the last successful capture request in seconds.
 * @return Nonnegative delay, or a negative value when no timing is available. */
double lwir_cam_pipe_capture_delay(void);
/** @brief Return detailed callback or polling timing evidence for the last capture.
 * @return Timing structure; use capture_timing_valid() before consuming it. */
struct capture_timing lwir_cam_pipe_capture_timing(void);
/** @brief Request one thermal JPEG from an initialized capture server.
 * @param filename Output path buffer, cleared on failure.
 * @param filename_size Output path capacity.
 * @param image_number CATIA shot sequence.
 * @return 0 only for a verified nonempty output image, otherwise -1; CATIA owns retry policy. */
int lwir_cam_pipe_shoot(char *filename, size_t filename_size, int image_number);
/** @brief Stop, reap, and forget the persistent LWIR capture server. */
void lwir_cam_pipe_deinit(void);

#endif