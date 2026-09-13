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

/** Start the earcam server. Returns -1 only for configuration errors (missing
 *  executable, no memory); a microphone that is not (yet) present is not fatal,
 *  the server is restarted periodically from ear_cam_pipe_record(). */
int ear_cam_pipe_init(const char *unused);
void ear_cam_pipe_deinit(void);
bool ear_cam_pipe_ready(void);

/** Replace the microphone by a virtual loudspeaker (for NPS/desk simulation).
 *  Must be called before ear_cam_pipe_init(); no earcam process is started. */
void ear_cam_pipe_set_simulated_source(double lat_deg, double lon_deg, double level_db_at_1m);

/** Tone search band passed to earcam (--low-cut/--high-cut); 0 keeps the earcam
 *  defaults. Must be called before ear_cam_pipe_init(). */
void ear_cam_pipe_set_band(double low_cut_hz, double high_cut_hz);

/** Geotag the newest microphone measurement with a shot message. */
int ear_cam_pipe_record(const union dc_shot_union *shot);

/** Finalize the session: fuse all samples into the loudest spot and clear the buffer. */
int ear_cam_pipe_finish(struct ear_loudest_spot *result);

/** Intermediate solve over the samples so far; the session keeps accumulating. */
int ear_cam_pipe_solve(struct ear_loudest_spot *result);

/** Render the current session as an acoustic "photo" photos/e%06d.jpg (north-up
 *  intensity map with the loudest spot marked). Fills filename; returns 0 on success.
 *  Must be called before ear_cam_pipe_finish() clears the session. */
int ear_cam_pipe_render(const struct ear_loudest_spot *result, char *filename, size_t filename_size);

/** Shot number of the newest recorded sample (0 when empty). */
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

int calculated_loudestspot(const struct ear_sample *samples, size_t count,
                           struct ear_loudest_spot *result);

#endif
