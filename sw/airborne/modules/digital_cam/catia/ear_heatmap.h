#ifndef CATIA_EAR_HEATMAP_H
#define CATIA_EAR_HEATMAP_H

#include <stddef.h>

#include "ear_cam_pipe.h"

/** Render the geotagged samples as a north-up acoustic intensity map.
 *  Ground resolution follows the microphone footprint (about AGL/4 per cell);
 *  the fused loudest spot is drawn as a marker. Writes filename and returns 0. */
int ear_heatmap_write(const char *filename, const struct ear_sample *samples, size_t count,
                      const struct ear_loudest_spot *spot);

/** Same field without dots, marker and scale bar, for map overlays (ear_heatmap_overlay.py). */
int ear_heatmap_write_field(const char *filename, const struct ear_sample *samples, size_t count,
                            const struct ear_loudest_spot *spot);

#endif
