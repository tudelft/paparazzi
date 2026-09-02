#ifndef CATIA_IMAGE_FAKE_TRANSFORM_H
#define CATIA_IMAGE_FAKE_TRANSFORM_H

#include "protocol.h"

#define IMAGE_FAKE_TRANSFORM_APPLIED 0
#define IMAGE_FAKE_TRANSFORM_SKIPPED 1

int image_fake_transform(const char *filename, const union dc_shot_union *shot);

#endif