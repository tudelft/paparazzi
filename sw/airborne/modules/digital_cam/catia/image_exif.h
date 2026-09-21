/** @file image_exif.h @brief Flight, timing, and thermal-analysis metadata writing for captured JPEGs. */
#ifndef CATIA_IMAGE_EXIF_H
#define CATIA_IMAGE_EXIF_H

#include "protocol.h"
#include "capture_timing.h"

/** @brief Add standard CATIA flight metadata to a JPEG.
 * @param filename Existing JPEG to rewrite atomically.
 * @param shot Flight-controller shot payload.
 * @return 0 on success or -1 on invalid input, JPEG, metadata, or storage failure. */
int image_exif_write(const char *filename, const union dc_shot_union *shot);
/** @brief Add flight metadata with optional capture-delay motion compensation.
 * @param filename Existing JPEG to rewrite atomically.
 * @param shot Flight-controller shot payload.
 * @param capture_delay_s Nonnegative request-to-frame latency, or a negative value when unknown.
 * @param compensate Nonzero to estimate ground displacement from speed/course and latency.
 * @details Original coordinates remain in the textual record so downstream analysis can audit
 * the correction rather than treating the constant-velocity estimate as ground truth. */
int image_exif_write_timed(const char *filename, const union dc_shot_union *shot,
						   double capture_delay_s, int compensate);
/** @brief Append structured thermal hotspot/geolocation evidence to JPEG EXIF.
 * @param filename Existing JPEG.
 * @param information Bounded UTF-8/ASCII analysis record.
 * @return 0 on atomic rewrite success, otherwise -1. */
int image_exif_write_hotspots(const char *filename, const char *information);
/** @brief Append structured vehicle-detection evidence (AICam) to JPEG EXIF.
 * @param filename Existing JPEG.
 * @param information Bounded UTF-8/ASCII analysis record.
 * @return 0 on atomic rewrite success, otherwise -1. */
int image_exif_write_vehicle_detections(const char *filename, const char *information);
/** @brief Add flight metadata using authoritative capture timing when available.
 * @param filename Existing JPEG to rewrite atomically.
 * @param shot Flight-controller shot payload.
 * @param capture_delay_s Fallback latency when @p timing is absent or invalid.
 * @param compensate Nonzero to apply conservative constant-velocity ground correction.
 * @param timing Optional validated server timing evidence; it overrides a supplied delay.
 * @details Timing describes request-to-frame arrival, not sensor exposure time; the emitted
 * metadata labels this limitation explicitly. */
int image_exif_write_capture(const char *filename, const union dc_shot_union *shot,
							 double capture_delay_s, int compensate, const struct capture_timing *timing);

#endif