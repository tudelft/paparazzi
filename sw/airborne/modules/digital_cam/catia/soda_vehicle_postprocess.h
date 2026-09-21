/** @file soda_vehicle_postprocess.h @brief EXIF-driven vehicle-hit post-processing for SODA. */
#ifndef SODA_VEHICLE_POSTPROCESS_H
#define SODA_VEHICLE_POSTPROCESS_H

/**
 * @brief Post-process a captured image for a vehicle-detection hit, if it has one.
 * @param image_path Path to the JPEG SODA was invoked on.
 * @return true when there was nothing to do (no AICAM_VEHICLES_V1 hit in EXIF, or the
 * marker couldn't be read at all -- the common case for every non-aicam-detect shot) or
 * every post-processing step for a real hit succeeded; false only when a hit was found but
 * at least one downstream step (full-frame copy, crop, or CSV row) failed.
 * @details No-op beyond one EXIF read unless @p image_path carries an AICAM_VEHICLES_V1
 * record (written by catia's image_exif_write_vehicle_detections()) reporting count >= 1.
 * On a hit, copies the image into a sibling `vehicles_captured/` directory, saves a padded
 * crop of the detected box into a sibling `tight_crop_vehicles/` directory, and appends one
 * row to `vehicles_captured/detections.csv`. Both directories are derived from
 * @p image_path's own directory -- there is no separately configured base directory.
 */
bool soda_process_vehicle_detection(const char *image_path);

#endif // SODA_VEHICLE_POSTPROCESS_H
