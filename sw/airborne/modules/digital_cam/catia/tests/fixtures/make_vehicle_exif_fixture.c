/*
 * Writes real CATIA flight-metadata EXIF (so EXIF_TAG_IMAGE_DESCRIPTION exists, matching
 * what image_exif_write_vehicle_detections() requires) onto an existing JPEG, then --
 * unless count is 0 -- appends an AICAM_VEHICLES_V1 vehicle-detection record, in exactly
 * the format vehicle_detect_pipe_last_detection_summary() produces for a real hit. Used by
 * tests/soda_vehicle_postprocess_test.sh to build hit/non-hit fixtures without needing the
 * real on-chip detection pipeline.
 *
 * Usage: make_vehicle_exif_fixture IMAGE COUNT CONFIDENCE BOX_X BOX_Y BOX_W BOX_H
 * COUNT=0 skips the vehicle-detection EXIF write entirely (the "not an aicam-detect shot"
 * case), ignoring the remaining arguments.
 */
#include "../../image_exif.h"
#include "../../protocol.h"

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>

int main(int argc, char **argv)
{
  assert(argc == 8);
  const char *image_path = argv[1];
  int count = atoi(argv[2]);

  union dc_shot_union shot = {0};
  shot.data.nr = 6;
  shot.data.lat = 488100000;
  shot.data.lon = 78530000;
  shot.data.alt = 50000;
  assert(image_exif_write_capture(image_path, &shot, -1, 0, NULL) == 0);

  if (count > 0) {
    char summary[256];
    int written = snprintf(summary, sizeof(summary),
                           "status=ok; count=%d; label=vehicle; confidence=%s; box=%s,%s,%s,%s",
                           count, argv[3], argv[4], argv[5], argv[6], argv[7]);
    assert(written > 0 && (size_t)written < sizeof(summary));
    assert(image_exif_write_vehicle_detections(image_path, summary) == 0);
  }

  puts("fixture ready");
  return 0;
}
