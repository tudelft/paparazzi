#ifndef CATIA_IMAGE_EXIF_H
#define CATIA_IMAGE_EXIF_H

#include "protocol.h"

int image_exif_write(const char *filename, const union dc_shot_union *shot);

#endif