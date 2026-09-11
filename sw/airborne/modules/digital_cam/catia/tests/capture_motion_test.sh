#!/usr/bin/env bash
set -euo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
output=$(mktemp -d /tmp/catia-capture-motion.XXXXXX)
trap 'rm -rf "$output"' EXIT
gcc -std=c11 -Wall -Wextra -Werror "$root/tests/motion_compensation_test.c" -lm -o "$output/motion"
"$output/motion"
gcc -std=gnu11 -Wall -Wextra -Werror "$root/tests/capture_timing_test.c" -lm -o "$output/timing"
"$output/timing"
g++ -std=c++11 -Wall -Wextra -Werror -I"$root/lwircam" \
  "$root/lwircam/tests/frame_stability_test.cpp" -o "$output/stability"
"$output/stability"
printf '%s\n' 'Capture timing, rolling stability and motion compensation tests passed'