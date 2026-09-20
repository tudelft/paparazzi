/** @file image_mock_transform.h @brief Local-simulation JPEG attitude transform API. */
#ifndef CATIA_IMAGE_MOCK_TRANSFORM_H
#define CATIA_IMAGE_MOCK_TRANSFORM_H

#include "protocol.h"

/** @brief Successful result: the JPEG was rewritten with the simulated attitude transform. */
#define IMAGE_MOCK_TRANSFORM_APPLIED 0
/** @brief Successful result: transform was intentionally skipped for a near-horizon attitude. */
#define IMAGE_MOCK_TRANSFORM_SKIPPED 1

/**
 * @brief Rewrite a local test JPEG to simulate the shot attitude.
 * @param filename Writable path to the JPEG to transform in place.
 * @param shot Flight pose containing fixed-point roll, pitch, yaw, and position fields.
 * @return @c IMAGE_MOCK_TRANSFORM_APPLIED, @c IMAGE_MOCK_TRANSFORM_SKIPPED, or -1 on failure.
 * @details The transform uses a perspective projection and an atomic temporary-file rename so
 * consumers never observe a partial JPEG. Near-horizon frames are skipped or synthesized by
 * configured policy because planar image projection becomes numerically untrustworthy there.
 * @warning This is simulation evidence only; it does not model a calibrated physical camera.
 */
int image_mock_transform(const char *filename, const union dc_shot_union *shot);

#endif