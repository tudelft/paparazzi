#!/usr/bin/env bash
set -euo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
output=$(mktemp -d /tmp/catia-lwir-timeout.XXXXXXXX)
trap 'rm -rf "$output"' EXIT
chmod +x "$root/tests/fixtures/hanging_lwir_server.sh"
gcc -std=gnu11 -Wall -Wextra -Werror \
  -DCHILD_STOP_GRACE_MS=100 -DLWIR_REQUEST_TIMEOUT_MS=100 \
  -DCATIA_LWIR_CAM_COMMAND="\"$root/tests/fixtures/hanging_lwir_server.sh\"" \
  "$root/tests/lwir_timeout_test.c" -o "$output/test"
"$output/test"