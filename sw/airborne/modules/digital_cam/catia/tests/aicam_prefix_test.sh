#!/usr/bin/env bash
set -euo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
output=$(mktemp -d /tmp/catia-aicam-prefix.XXXXXXXX)
trap 'rm -rf "$output"' EXIT
gcc -std=gnu11 -Wall -Wextra -Werror "$root/tests/aicam_prefix_test.c" \
  "$root/ai_cam_pipe.c" -o "$output/test"
cd "$output"
./test