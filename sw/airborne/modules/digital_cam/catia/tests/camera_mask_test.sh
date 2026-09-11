#!/usr/bin/env bash
set -euo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
workspace=$(cd "$root/../../../../.." && pwd)
output=$(mktemp -d /tmp/catia-camera-mask.XXXXXXXX)
trap 'rm -rf "$output"' EXIT
make -s -C "$root" -j32 catia
objects=()
for object in "$root"/.build/project/*.o; do
  case "$object" in
    */catia.o|*/ear_heatmap_replay.o) ;;
    *) objects+=("$object") ;;
  esac
done
gcc -std=gnu11 -Wall -Wextra -Werror -Wno-expansion-to-defined \
  -I"$workspace/sw/ext" -I"$workspace/sw/ext/libexif" \
  -I"$workspace/sw/ext/opencv_bebop/opencv/3rdparty/libjpeg" \
  -DCATIA_CHDK_PHOTO_DIR='"photos"' \
  "$root/tests/camera_mask_test.c" "${objects[@]}" \
  "$root"/.build/libexif/*.o "$root"/.build/libjpeg/*.o \
  -lpthread -lm -o "$output/test"
cd "$output"
./test "$root/mock_image_01.jpg"