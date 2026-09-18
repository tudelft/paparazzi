#ifndef CATIA_VEHICLE_DETECT_PIPE_H
#define CATIA_VEHICLE_DETECT_PIPE_H

/**
 * @file vehicle_detect_pipe.h
 * @brief Parent-side protocol for CATIA's persistent on-sensor vehicle-detection server.
 * @details Mirrors lwir_cam_pipe.h: a line protocol over the child's stdin/stdout keeps
 * the IMX500 network loaded between shots, since firmware upload takes 17-60s and cannot
 * be paid on every capture. The server always produces an image file per shot; the last
 * detection result (if any) is retrieved separately for EXIF writing.
 */

#include <stddef.h>

/** @brief Start and qualify the persistent vehicle-detection capture server.
 * @param unused Reserved common backend argument.
 * @return 0 after the server reports ready, otherwise -1. */
int vehicle_detect_pipe_init(const char *unused);
/** @brief Override the output directory for an isolated standalone benchmark. */
void vehicle_detect_pipe_set_photo_directory(const char *directory);
/** @brief Request one JPEG, restarting the server once if it dies mid-shot.
 * @param filename Output path buffer, cleared on failure.
 * @param filename_size Output path capacity.
 * @param image_number CATIA shot sequence.
 * @return 0 only for a verified nonempty output image, otherwise -1. */
int vehicle_detect_pipe_shoot(char *filename, size_t filename_size, int image_number);
/** @brief Stop, reap, and forget the persistent vehicle-detection capture server. */
void vehicle_detect_pipe_deinit(void);
/** @brief Format the most recent shot's detection result for EXIF writing.
 * @param buffer Caller-owned destination for a short human-readable summary.
 * @param buffer_size Capacity of @p buffer.
 * @details Reflects the last vehicle_detect_pipe_shoot() call only; safe to call even
 * when that call failed (reports status=analysis_failed) or no server ever ran. */
void vehicle_detect_pipe_last_detection_summary(char *buffer, size_t buffer_size);

#endif
