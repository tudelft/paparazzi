#!/usr/bin/env bash
set -euo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
output=$(mktemp -d /tmp/catia-lwir-shoot-recovery.XXXXXX)
trap 'rm -rf "$output"' EXIT
chmod +x "$root/tests/fixtures/dying_lwir_server.sh"
gcc -std=gnu11 -Wall -Wextra -Werror \
  -DCATIA_LWIR_CAM_COMMAND="\"$root/tests/fixtures/dying_lwir_server.sh\"" \
  "$root/tests/lwir_shoot_recovery_test.c" -o "$output/test"
"$output/test"
