#!/usr/bin/env bash
set -euo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
workspace=$(cd "$root/../../../../.." && pwd)
output=$(mktemp -d /tmp/catia-clock-test.XXXXXX)
trap 'rm -rf "$output"' EXIT
gcc -std=c11 -Wall -Wextra -Werror -fsanitize=address,undefined -g \
  "$root/clock_alignment.c" "$root/tests/clock_alignment_test.c" -o "$output/core"
"$output/core"
gcc -std=c11 -Wall -Wextra -Werror -fsanitize=address,undefined -g \
  "$root/tests/serial_tx_test.c" "$root/protocol.c" -Wl,--wrap=write -pthread -o "$output/tx"
"$output/tx"
make -s -C "$root" -j32 catia
objects=()
for object in "$root"/.build/project/*.o; do
  case "$object" in */catia.o|*/ear_heatmap_replay.o) ;; *) objects+=("$object") ;; esac
done
gcc -Wall -Wextra -Werror -Wno-expansion-to-defined \
  -I"$workspace/sw/ext" -I"$workspace/sw/ext/libexif" \
  -I"$workspace/sw/ext/opencv_bebop/opencv/3rdparty/libjpeg" \
  "$root/tests/clock_link_test.c" "${objects[@]}" \
  "$root"/.build/libexif/*.o "$root"/.build/libjpeg/*.o -lpthread -lm -o "$output/link"
"$output/link"