#!/usr/bin/env bash
set -euo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
output=$(mktemp -d /tmp/catia-lwir-startup-cancel.XXXXXXXX)
trap 'rm -rf "$output"' EXIT
chmod +x "$root/tests/fixtures/never_ready_lwir_server.sh"
read -r -a sanitizer_flags <<< "${SANITIZER_FLAGS:-}"
gcc -std=gnu11 -Wall -Wextra -Werror "${sanitizer_flags[@]}" \
  -DCHILD_STOP_GRACE_MS=100 -DLWIR_SERVER_START_TIMEOUT_MS=5000 \
  -DCATIA_LWIR_CAM_COMMAND="\"$root/tests/fixtures/never_ready_lwir_server.sh\"" \
  "$root/tests/lwir_startup_cancel_test.c" -lpthread "${sanitizer_flags[@]}" -o "$output/test"
timeout 10s "$output/test"