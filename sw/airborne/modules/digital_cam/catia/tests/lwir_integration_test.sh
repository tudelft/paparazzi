#!/usr/bin/env bash
set -euo pipefail

root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
workspace=$(cd "$root/../../../../.." && pwd)
output=$(mktemp -d /tmp/catia-lwir-test.XXXXXX)
trap 'rm -rf "$output"' EXIT
mkdir "$output/photos"
mkdir "$output/no-external-tools"
make -s -C "$root" -j32 catia soda lwircam
objects=()
for object in "$root"/.build/project/*.o; do
  case "$object" in
    */catia.o|*/local_pipe.o|*/lwir_cam_pipe.o|*/ear_heatmap_replay.o) ;;
    *) objects+=("$object") ;;
  esac
done
gcc -Wall -Wextra -Werror -Wno-expansion-to-defined \
  -I"$workspace/sw/ext" -I"$workspace/sw/ext/libexif" \
  -I"$workspace/sw/ext/opencv_bebop/opencv/3rdparty/libjpeg" \
  -DCATIA_LOCAL_PHOTO_DIR="\"$output/photos\"" \
  -DCATIA_SODA="\"$root/soda\"" \
  -DCATIA_LWIR_CAM_COMMAND="\"$root/lwircam-native\"" \
  "$root/tests/lwir_integration_test.c" "$root/local_pipe.c" "$root/lwir_cam_pipe.c" \
  "${objects[@]}" "$root"/.build/libexif/*.o "$root"/.build/libjpeg/*.o \
  -lpthread -lm -o "$output/test"
PATH="$output/no-external-tools" EXIFTOOL="$output/no-external-tools/exiftool" PERL5LIB= \
  "$output/test" "$root/lwircam/mock_lwir_01.jpg" "$output/photos" \
  "$root/lwircam/tests/mock_camera_256x192.yml" | tee "$output/log"
test -s "$output/photos/mc000037.jpg"
test -s "$output/photos/ma000038.jpg"
test -s "$output/photos/ml000039.jpg"
test ! -e "$output/photos/ml000039.jpg.hotspots.json"
test -s "$output/photos/ma000040.jpg"
test ! -e "$output/photos/ma000040.jpg.hotspots.json"
grep -q 'CATIA-39:.*soda return 0' "$output/log"
grep -q 'CATIA-40:.*soda return 0' "$output/log"
for camera in aicam chdkcam lwircam earcam; do
  grep -Fxq "Now I can do nifty stuff for $camera" "$output/log"
done
g++ -std=c++11 -Wall -Wextra -Werror -I"$root/lwircam" \
  "$root/lwircam/temperature_layer.cpp" "$root/lwircam/tests/compare_temperature_layers.cpp" \
  -o "$output/compare"
"$output/compare" "$root/lwircam/mock_lwir_01.jpg" "$output/photos/ml000039.jpg"