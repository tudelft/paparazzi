#ifndef DIGITAL_CAM_CATIA_CAMERA_SPEEDTEST_H
#define DIGITAL_CAM_CATIA_CAMERA_SPEEDTEST_H

/**
 * @file camera_speedtest.h
 * @brief Standalone, backend-neutral still-camera throughput benchmark.
 */

#include <signal.h>
#include <stddef.h>

#define CAMERA_SPEEDTEST_SAMPLE_COUNT 10
#define CAMERA_SPEEDTEST_FIRST_IMAGE 900000
#define CAMERA_SPEEDTEST_LAST_IMAGE 999999

/**
 * @brief Immutable dependencies for one selected and initialized camera backend.
 * @details The benchmark deliberately receives only the synchronous capture operation
 * and output naming information. It does not own backend initialization, UART traffic,
 * EXIF processing, or SODA; including them would turn a camera measurement into a
 * system-load measurement.
 */
struct camera_speedtest_config {
  /** Human-readable backend name included in the final report. */
  const char *backend_name;
  /** Directory receiving the private benchmark subdirectory and retained JPEGs. */
  const char *photo_directory;
  /** Backend-specific filename prefix, such as @c a for AIcam or @c l for LWIRcam. */
  char filename_prefix;
  /** Synchronous backend capture operation; returns its actual output path in @p filename. */
  int (*shoot)(char *filename, size_t filename_size, int image_number);
  /** Shared shutdown flag set by signal handling; checked between bounded captures. */
  volatile sig_atomic_t *keep_running;
};

/**
 * @brief Atomically create a private directory for one benchmark run.
 * @param base_directory Configured backend photo root.
 * @param output Receives the new directory path.
 * @param output_size Capacity of @p output.
 * @return 0 on success, otherwise -1 with errno set.
 * @details A private mode-0700 directory prevents benchmark backends, which normally
 * replace same-number files, from racing with or overwriting mission photographs.
 */
int camera_speedtest_create_output_directory(const char *base_directory,
                                             char *output, size_t output_size);

/**
 * @brief Find an unused consecutive image-number block for warm-up and samples.
 * @param config Backend filename information.
 * @param first_image Receives the warm-up image number.
 * @return 0 on success, -1 on invalid input, filesystem error, or exhausted range.
 * @details CATIA camera backends replace an existing same-number image. Preflighting
 * the complete block before camera initialization keeps benchmark evidence separate
 * from mission captures and prevents accidental replacement of an existing photo.
 */
int camera_speedtest_find_image_block(const struct camera_speedtest_config *config,
                                      int *first_image);

/**
 * @brief Capture one untimed warm-up image and ten timed images sequentially.
 * @param config Selected and initialized camera backend.
 * @param first_image Warm-up image number returned by camera_speedtest_find_image_block().
 * @return 0 only when every requested image was captured and validated.
 * @details Only synchronous shoot time is used for camera throughput. Progress records
 * are written through bounded nonblocking I/O by a separate reporter thread, so a slow
 * terminal cannot delay capture requests or inflate measured camera latency. Images
 * are retained for review.
 */
int camera_speedtest_run(const struct camera_speedtest_config *config, int first_image);

#endif /* DIGITAL_CAM_CATIA_CAMERA_SPEEDTEST_H */