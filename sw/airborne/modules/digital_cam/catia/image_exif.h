#ifndef CATIA_IMAGE_EXIF_H
#define CATIA_IMAGE_EXIF_H

#include "protocol.h"
#include "capture_timing.h"

int image_exif_write(const char *filename, const union dc_shot_union *shot);
int image_exif_write_timed(const char *filename, const union dc_shot_union *shot,
						   double capture_delay_s, int compensate);
int image_exif_write_hotspots(const char *filename, const char *information);
int image_exif_write_capture(const char *filename, const union dc_shot_union *shot,
							 double capture_delay_s, int compensate, const struct capture_timing *timing);

#endif