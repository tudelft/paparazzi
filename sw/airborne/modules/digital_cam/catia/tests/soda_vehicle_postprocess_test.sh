#!/usr/bin/env bash
set -euo pipefail

root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
output=$(mktemp -d /tmp/catia-soda-vehicle-postprocess.XXXXXX)
trap 'rm -rf "$output"' EXIT

# soda catia: soda for the binary under test, catia to also get .build/project/image_exif.o
# (only built as one of catia's own PROJECT_OBJECTS) for the fixture generator below.
make -s -C "$root" soda catia

gcc -std=gnu11 -Wall -Wextra -Werror -Wno-expansion-to-defined \
  -I"$root" -I/home/coco-allie-69/paparazzi/sw/ext -I/home/coco-allie-69/paparazzi/sw/ext/libexif \
  -DNO_VERBOSE_TAG_DATA -DNO_VERBOSE_TAG_STRINGS \
  "$root/tests/fixtures/make_vehicle_exif_fixture.c" \
  "$root"/.build/project/image_exif.o "$root"/.build/libexif/*.o \
  -lm -o "$output/make_fixture"

# --- Unit tests for the internal crop-box/EXIF-parsing helpers ---
g++ -std=c++14 -Wall -Wextra -Werror -Wno-expansion-to-defined \
  -I"$root" -I/home/coco-allie-69/paparazzi/sw/ext -I/home/coco-allie-69/paparazzi/sw/ext/libexif \
  -I/home/coco-allie-69/paparazzi/sw/ext/opencv_bebop/opencv/3rdparty/libjpeg \
  -DNO_VERBOSE_TAG_DATA -DNO_VERBOSE_TAG_STRINGS \
  "$root/tests/soda_vehicle_postprocess_test.cpp" \
  "$root"/.build/libexif/*.o "$root"/.build/libjpeg/*.o \
  -lpthread -lm -o "$output/unit_test"
"$output/unit_test"

# --- End-to-end: a genuine hit produces all three artifacts ---
cp "$root/mock_aicam_01.jpg" "$output/a000006.jpg"
"$output/make_fixture" "$output/a000006.jpg" 2 0.8734 100 150 300 220
cp "$output/a000006.jpg" "$output/a000006-before-soda.jpg"

"$root/soda" --aicam "$output/a000006.jpg" > "$output/hit.log" 2>&1

# The plain source file must stay byte-for-byte untouched by SODA -- the whole point of
# this fix is that the configured photo directory only ever holds clean, unannotated
# captures. (Compared against its own post-EXIF-write state, not the pristine mock image,
# since make_fixture legitimately wrote flight/detection EXIF into it above.)
cmp "$output/a000006-before-soda.jpg" "$output/a000006.jpg"

# The vehicles_captured/ copy is annotated (re-encoded with the box drawn in, so it can
# never be byte-identical to the source) but must be a valid JPEG at the same dimensions.
test -s "$output/vehicles_captured/a000006.jpg"
! cmp -s "$output/a000006.jpg" "$output/vehicles_captured/a000006.jpg"
file "$output/vehicles_captured/a000006.jpg" | grep -q '1200x899'

test -s "$output/tight_crop_vehicles/a000006_vehicle01.jpg"
file "$output/tight_crop_vehicles/a000006_vehicle01.jpg" | grep -q '408x300'

test -s "$output/vehicles_captured/detections.csv"
[[ $(wc -l < "$output/vehicles_captured/detections.csv") -eq 2 ]]
head -1 "$output/vehicles_captured/detections.csv" \
  | grep -qFx 'image,count,confidence,box_x,box_y,box_w,box_h'
tail -1 "$output/vehicles_captured/detections.csv" \
  | grep -qFx 'a000006.jpg,2,0.8734,100,150,300,220'

# --- Negative case: no vehicle-detect EXIF at all -> nothing added for this image, even
# though vehicles_captured/ and tight_crop_vehicles/ already exist from the hit above ---
cp "$root/mock_aicam_01.jpg" "$output/a000007.jpg"
"$output/make_fixture" "$output/a000007.jpg" 0 0 0 0 0 0
"$root/soda" --aicam "$output/a000007.jpg" > /dev/null
[[ ! -f "$output/vehicles_captured/a000007.jpg" ]]
[[ ! -f "$output/tight_crop_vehicles/a000007_vehicle01.jpg" ]]
[[ $(grep -c 'a000007' "$output/vehicles_captured/detections.csv" || true) -eq 0 ]]

# --- Concurrency: two SODA processes appending to the same CSV at once must not corrupt it ---
cp "$root/mock_aicam_01.jpg" "$output/a000008.jpg"
cp "$root/mock_aicam_01.jpg" "$output/a000009.jpg"
"$output/make_fixture" "$output/a000008.jpg" 1 0.5000 10 20 30 40
"$output/make_fixture" "$output/a000009.jpg" 1 0.6000 50 60 70 80
"$root/soda" --aicam "$output/a000008.jpg" > /dev/null &
pid1=$!
"$root/soda" --aicam "$output/a000009.jpg" > /dev/null &
pid2=$!
wait "$pid1" "$pid2"

[[ $(wc -l < "$output/vehicles_captured/detections.csv") -eq 4 ]]
[[ $(grep -c '^image,count,confidence,box_x,box_y,box_w,box_h$' "$output/vehicles_captured/detections.csv") -eq 1 ]]
grep -qFx 'a000008.jpg,1,0.5000,10,20,30,40' "$output/vehicles_captured/detections.csv"
grep -qFx 'a000009.jpg,1,0.6000,50,60,70,80' "$output/vehicles_captured/detections.csv"

printf 'soda vehicle post-processing tests passed\n'
