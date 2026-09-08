// C11 + POSIX.1-2008.
#define _POSIX_C_SOURCE 200809L

// Re-render an EAR session log (earlogs/ear_*.csv) into the loudest-spot heatmap
// exactly as CATIA does in flight: fuse the samples, write the JPEG and its .geo
// georeference sidecar. Ground-side tool; pair it with ear_heatmap_overlay.py to
// put the result over satellite imagery.
//
//   ear_heatmap_replay earlogs/ear_20260908_103533.csv photos/e000686.jpg
//   -> photos/e000686.jpg, e000686.geo, e000686_field.jpg (undecorated, for the overlay)
#include "ear_cam_pipe.h"
#include "ear_heatmap.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAX_SAMPLES 100000U

static int parse_line(const char *line, struct ear_sample *s)
{
  unsigned long long t_ms;
  int shot, alarm, clip;
  double freq;
  int fields = sscanf(line, "%llu,%d,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%d,%d",
                      &t_ms, &shot, &s->lat_deg, &s->lon_deg, &s->agl_m, &s->alt_m,
                      &s->level_db, &s->trend_db, &s->contrast_db, &freq, &alarm, &clip);
  if (fields != 12) {
    return -1;
  }
  s->timestamp_ms = t_ms;
  s->shot_nr = shot;
  s->frequency_hz = freq;
  s->alarm = alarm != 0;
  s->clipped = clip != 0;
  return 0;
}

int main(int argc, char **argv)
{
  if (argc != 3) {
    fprintf(stderr, "usage: %s SESSION.csv OUT.jpg\n", argv[0]);
    return 2;
  }
  FILE *input = fopen(argv[1], "r");
  if (input == NULL) {
    perror(argv[1]);
    return 1;
  }
  struct ear_sample *samples = calloc(MAX_SAMPLES, sizeof(*samples));
  if (samples == NULL) {
    fclose(input);
    return 1;
  }
  size_t count = 0;
  char line[512];
  while (fgets(line, sizeof(line), input) != NULL && count < MAX_SAMPLES) {
    if (line[0] == '#' || line[0] == 't' || line[0] == '\n') {
      continue;   // comment, header or blank
    }
    if (parse_line(line, &samples[count]) == 0) {
      count++;
    }
  }
  fclose(input);

  struct ear_loudest_spot spot;
  memset(&spot, 0, sizeof(spot));
  if (calculated_loudestspot(samples, count, &spot) != 0 || !spot.valid) {
    fprintf(stderr, "%s: no valid loudest spot from %zu samples\n", argv[1], count);
    free(samples);
    return 1;
  }
  printf("samples=%zu used=%u spot lat=%.7f lon=%.7f level=%.2f dB conf=%.3f\n",
         count, spot.used_count, spot.lat_deg, spot.lon_deg, spot.level_db, spot.confidence);
  int status = ear_heatmap_write(argv[2], samples, count, &spot);
  if (status == 0) {
    // Undecorated field for ear_heatmap_overlay.py: OUT.jpg -> OUT_field.jpg
    char field[512];
    size_t n = strlen(argv[2]);
    size_t stem = (n > 4 && strcmp(argv[2] + n - 4, ".jpg") == 0) ? n - 4 : n;
    if (stem + 11 < sizeof(field)) {
      for (size_t i = 0; i < stem; i++) {
        field[i] = argv[2][i];
      }
      field[stem] = '\0';
      strcat(field, "_field.jpg");
      if (ear_heatmap_write_field(field, samples, count, &spot) == 0) {
        printf("wrote %s\n", field);
      }
    }
  }
  free(samples);
  if (status != 0) {
    fprintf(stderr, "failed to write %s\n", argv[2]);
    return 1;
  }
  printf("wrote %s (+ .geo sidecar)\n", argv[2]);
  return 0;
}
