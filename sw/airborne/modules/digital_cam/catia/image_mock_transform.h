#ifndef CATIA_IMAGE_MOCK_TRANSFORM_H
#define CATIA_IMAGE_MOCK_TRANSFORM_H

#include "protocol.h"

#define IMAGE_MOCK_TRANSFORM_APPLIED 0
#define IMAGE_MOCK_TRANSFORM_SKIPPED 1

int image_mock_transform(const char *filename, const union dc_shot_union *shot);

#endif