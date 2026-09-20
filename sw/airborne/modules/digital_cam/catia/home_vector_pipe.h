#ifndef CATIA_HOME_VECTOR_PIPE_H
#define CATIA_HOME_VECTOR_PIPE_H

/**
 * @file home_vector_pipe.h
 * @brief Parent-side protocol for CATIA's persistent on-sensor visual-homing server.
 * @details Mirrors vehicle_detect_pipe.h's persistent-server pattern: the IMX500 network
 * firmware upload (17-60s) is paid once per session, not per shot, via a line protocol
 * over the child's stdin/stdout. Unlike vehicle detection, every shot produces a fresh
 * predicted home vector (not just an occasional box), which the caller retrieves after
 * each home_vector_pipe_shoot() to forward over the UART link to the flight controller.
 */

#include <stdbool.h>
#include <stddef.h>

/** @brief Start and qualify the persistent visual-homing capture server.
 * @param unused Reserved common backend argument.
 * @return 0 after the server reports ready, otherwise -1. */
int home_vector_pipe_init(const char *unused);
/** @brief Override the output directory for an isolated standalone benchmark. */
void home_vector_pipe_set_photo_directory(const char *directory);
/** @brief Request one JPEG plus a fresh home-vector prediction, restarting the server
 * once if it dies mid-shot.
 * @param filename Output path buffer, cleared on failure.
 * @param filename_size Output path capacity.
 * @param image_number CATIA shot sequence.
 * @return 0 only for a verified nonempty output image, otherwise -1. */
int home_vector_pipe_shoot(char *filename, size_t filename_size, int image_number);
/** @brief Stop, reap, and forget the persistent visual-homing capture server. */
void home_vector_pipe_deinit(void);
/** @brief Retrieve the most recent shot's decoded body-frame prediction.
 * @param dx Body-frame direction-to-home x (forward), unit vector component.
 * @param dy Body-frame direction-to-home y (right), unit vector component.
 * @param dist Predicted distance to home, metres.
 * @return true if the last home_vector_pipe_shoot() produced a valid prediction. */
bool home_vector_pipe_last_result(float *dx, float *dy, float *dist);

#endif
