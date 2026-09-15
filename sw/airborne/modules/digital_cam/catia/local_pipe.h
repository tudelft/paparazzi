/** @file local_pipe.h @brief Local mock-camera backend API for simulator and desk testing. */
#ifndef CATIA_LOCAL_PIPE_H
#define CATIA_LOCAL_PIPE_H

#include <stddef.h>

/** @brief Initialize a deterministic mock image source for local camera simulation.
 * @param mock_image Readable JPEG copied for each shot.
 * @return 0 on readiness or -1 on missing source/output storage. */
int local_pipe_init(const char *mock_image);
/** @brief Initialize test mode with a supplied JPEG or a random fixture beside CATIA.
 * @param mock_image Optional explicit JPEG; NULL enables testphotos discovery.
 * @return 0 on readiness or -1 on configuration failure. */
int local_pipe_test_init(const char *mock_image);
/** @brief Create a mock capture file for a shot.
 * @param filename Destination path buffer.
 * @param filename_size Destination capacity.
 * @param image_number Shot sequence.
 * @param camera_suffix Real-camera filename prefix (`c`, `a`, `l`, or `e`).
 * @return 0 after a complete copy, otherwise -1. */
int local_pipe_shoot(char *filename, size_t filename_size, int image_number, char camera_suffix);
/** @brief Forget selected local sources; no persistent process is owned. */
void local_pipe_deinit(void);

#endif