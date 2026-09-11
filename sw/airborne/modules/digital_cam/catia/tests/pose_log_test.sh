#!/usr/bin/env bash
set -euo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
output=$(mktemp -d /tmp/catia-pose-test.XXXXXX)
trap 'rm -rf "$output"' EXIT
gcc -std=c11 -Wall -Wextra -Werror -fsanitize=address,undefined -g \
  "$root/tests/pose_log_test.c" -pthread -o "$output/logger"
"$output/logger"