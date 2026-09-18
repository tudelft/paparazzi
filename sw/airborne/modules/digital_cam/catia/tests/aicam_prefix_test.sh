#!/usr/bin/env bash
set -euo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
output=$(mktemp -d /tmp/catia-aicam-prefix.XXXXXXXX)
trap 'rm -rf "$output"' EXIT
gcc -std=gnu11 -Wall -Wextra -Werror -DCATIA_AI_CAM_PERSISTENT=0 "$root/tests/aicam_prefix_test.c" \
  "$root/ai_cam_pipe.c" -o "$output/test"
cd "$output"
./test
gcc -std=gnu11 -Wall -Wextra -Werror -DCATIA_AI_CAM_PERSISTENT=0 -DCATIA_AI_CAM_TIMEOUT_SECONDS=1 \
  "$root/tests/ai_cam_timeout_test.c" "$root/ai_cam_pipe.c" -o "$output/timeout_test"
"$output/timeout_test"
gcc -std=gnu11 -Wall -Wextra -Werror -DCATIA_AI_CAM_STARTUP_MS=100 \
  "$root/tests/aicam_persistent_test.c" "$root/ai_cam_pipe.c" -o "$output/persistent_test"
"$output/persistent_test"
