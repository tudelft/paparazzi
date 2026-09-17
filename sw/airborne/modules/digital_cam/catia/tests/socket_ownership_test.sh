#!/usr/bin/env bash
set -euo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)
output=$(mktemp -d /tmp/catia-socket-ownership.XXXXXXXX)
trap 'rm -rf "$output"' EXIT

gcc -std=gnu11 -Wall -Wextra -Werror \
  "$root/tests/socket_ownership_test.c" "$root/socket.c" \
  -o "$output/socket_ownership_test"
"$output/socket_ownership_test"