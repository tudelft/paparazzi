/** @file ear_cam_pipe.h @brief CATIA API for persistent EARcam sampling, fusion, and acoustic image rendering. */
#ifndef CATIA_EAR_CAM_PIPE_H
#define CATIA_EAR_CAM_PIPE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "protocol.h"

struct ear_loudest_spot {
  bool valid;
  double lat_deg;
  double lon_deg;
  double agl_m;
  double alt_m;
  double level_db;
  double confidence;   // 0..1
  uint32_t sample_count;
  uint32_t used_count;
};

/**
 * @brief Start EARcam sampling or its deterministic simulated-source replacement.
 * @param unused Reserved for the common camera-backend initialization signature; ignored.
 * @return 0 on usable initialization; -1 for configuration or allocation failure.
 * @details A missing microphone is deliberately not fatal. The optical pipeline can continue
 * operating while @c ear_cam_pipe_record() periodically retries the server. Configure the
 * simulated source and frequency band before calling this function.
 */
int ear_cam_pipe_init(const char *unused);

/**
 * @brief Stop EARcam activity and release its process, descriptors, and session storage.
 * @details Safe after a partially successful initialization. Call this after the final render
 * or solve because it discards unfinalized session samples.
 */
void ear_cam_pipe_deinit(void);

/**
 * @brief Report whether the live microphone server completed its ready handshake.
 * @return True only when the persistent EARcam server is ready to provide samples.
 * @details Simulated mode has no microphone process; callers should use this as live-hardware
 * readiness evidence rather than as a general indication that simulation is configured.
 */
bool ear_cam_pipe_ready(void);

/**
 * @brief Replace microphone input with a virtual fixed loudspeaker for NPS or desk tests.
 * @param lat_deg Source latitude in degrees.
 * @param lon_deg Source longitude in degrees.
 * @param level_db_at_1m Sound level at one metre in decibels.
 * @details Must be called before @c ear_cam_pipe_init(). Simulation avoids ALSA and the
 * external EARcam process while preserving the same geotagging and fusion interface.
 */
void ear_cam_pipe_set_simulated_source(double lat_deg, double lon_deg, double level_db_at_1m);

/**
 * @brief Set the live EARcam tone-search band.
 * @param low_cut_hz Lower cutoff in hertz, or zero to retain EARcam's default.
 * @param high_cut_hz Upper cutoff in hertz, or zero to retain EARcam's default.
 * @details Must be called before @c ear_cam_pipe_init() because the values become arguments
 * of the persistent external process. Invalid ordering is rejected by the caller's CLI path.
 */
void ear_cam_pipe_set_band(double low_cut_hz, double high_cut_hz);

/**
 * @brief Associate the newest sound window with one flight-controller shot pose.
 * @param shot Flight pose and shot number received from CATIA transport.
 * @return 0 when a usable sample was recorded; -1 for invalid input or unavailable data.
 * @details The newest bounded-age sound window is used rather than waiting for a future one,
 * preserving trigger-time positioning. Calls also provide the server-restart opportunity when
 * live hardware is temporarily unavailable.
 */
int ear_cam_pipe_record(const union dc_shot_union *shot);

/**
 * @brief Fuse the session into its final loudest-spot result and clear stored samples.
 * @param result Destination for the solved or invalid result.
 * @return 0 when processing completed; -1 for invalid arguments or internal failure.
 * @details Finalization consumes the session so the next survey cannot accidentally mix old
 * and new flight legs. Render before this call if an acoustic evidence image is required.
 */
int ear_cam_pipe_finish(struct ear_loudest_spot *result);

/**
 * @brief Solve the accumulated session without clearing it.
 * @param result Destination for the current estimate.
 * @return 0 when processing completed; -1 for invalid arguments or internal failure.
 * @details Use for refinement decisions. Unlike @c ear_cam_pipe_finish(), subsequent records
 * remain part of the same survey session.
 */
int ear_cam_pipe_solve(struct ear_loudest_spot *result);

/**
 * @brief Render the current session as a north-up acoustic evidence JPEG.
 * @param result Solved loudest-spot estimate to mark on the image.
 * @param filename Destination buffer receiving the JPEG path.
 * @param filename_size Capacity of @p filename.
 * @param simulated True when rendering NPS/desk simulated data.
 * @return 0 on success; -1 if inputs, session data, or output creation fail.
 * @details Must be called before @c ear_cam_pipe_finish() clears the session. The JPEG and
 * sidecar preserve the sampled field and final estimate for post-flight review.
 */
int ear_cam_pipe_render(const struct ear_loudest_spot *result, char *filename, size_t filename_size,
                        bool simulated);

/**
 * @brief Get the shot number associated with the newest stored acoustic sample.
 * @return The newest shot number, or 0 when the session is empty.
 */
int32_t ear_cam_pipe_last_shot_nr(void);

/** Pure fusion over an existing session; exposed for tests and tooling. */
struct ear_sample {
  uint64_t timestamp_ms;
  int32_t shot_nr;
  double lat_deg;
  double lon_deg;
  double agl_m;
  double alt_m;
  double level_db;
  double trend_db;
  double contrast_db;
  double frequency_hz;
  bool alarm;
  bool clipped;
};

/**
 * @brief Estimate the loudest ground position from a supplied geotagged sample set.
 * @param samples Immutable acoustic samples ordered by collection time.
 * @param count Number of entries in @p samples.
 * @param result Destination for the valid estimate or invalid-result marker.
 * @return 0 when the calculation completed; -1 for invalid arguments or insufficient data.
 * @details This pure function has no process, filesystem, or global-session dependency, which
 * makes it appropriate for deterministic tests and offline replay as well as live sessions.
 */
int calculated_loudestspot(const struct ear_sample *samples, size_t count,
                           struct ear_loudest_spot *result);

#endif
