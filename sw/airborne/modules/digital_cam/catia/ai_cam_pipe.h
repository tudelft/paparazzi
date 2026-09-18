#ifndef CATIA_AI_CAM_PIPE_H
#define CATIA_AI_CAM_PIPE_H

/**
 * @file ai_cam_pipe.h
 * @brief CATIA backend interface for Raspberry Pi AI Camera JPEG captures.
 */

#include <stddef.h>

/** @brief Validate configured AICam storage before shots are accepted.
 * @param unused Reserved common backend argument.
 * @return 0 on readiness or -1 on configuration/storage failure. */
int ai_cam_pipe_init(const char *unused);
/** @brief Override the output directory for an isolated standalone benchmark. */
void ai_cam_pipe_set_photo_directory(const char *directory);
/** @brief Capture a JPEG with rpicam-still.
 * @param filename Output path buffer, cleared on failure.
 * @param filename_size Output path capacity.
 * @param image_number CATIA shot number.
 * @return 0 only for a verified output file; otherwise -1. */
int ai_cam_pipe_shoot(char *filename, size_t filename_size, int image_number);
/** @brief Stop the persistent rpicam-still started by ai_cam_pipe_init(), if any. */
void ai_cam_pipe_deinit(void);

#endif