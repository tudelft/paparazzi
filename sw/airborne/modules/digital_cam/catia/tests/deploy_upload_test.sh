#!/usr/bin/env bash
set -euo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
DEPLOY_TEST_LOG=$(mktemp /tmp/catia-upload-test.XXXXXXXX)
export DEPLOY_TEST_LOG
trap 'rm -f "$DEPLOY_TEST_LOG"' EXIT
make() { printf 'make %s\n' "$*" >> "$DEPLOY_TEST_LOG"; }
rsync() { printf 'rsync %s\n' "$*" >> "$DEPLOY_TEST_LOG"; }
ssh() {
  printf 'ssh %s\n' "$*" >> "$DEPLOY_TEST_LOG"
  if [[ "$*" == *mktemp* ]]; then printf '/home/air/digital_cam/.deploy.TEST1234\n'; fi
}
file() { printf 'ELF 64-bit ARM aarch64, stripped\n'; }
aarch64-linux-gnu-gcc() { return 0; }
aarch64-linux-gnu-g++() { return 0; }
aarch64-linux-gnu-readelf() { return 0; }
aarch64-linux-gnu-strip() { printf 'strip %s\n' "$*" >> "$DEPLOY_TEST_LOG"; }
export -f make rsync ssh file aarch64-linux-gnu-gcc aarch64-linux-gnu-g++ aarch64-linux-gnu-readelf aarch64-linux-gnu-strip
bash "$root/deploy_mora.sh" test@mock
grep -q '^make .* arm64 ' "$DEPLOY_TEST_LOG"
! grep -q '^make .* clean' "$DEPLOY_TEST_LOG"
for name in catia soda lwircam earcam; do
  grep -Fq "$root/$name-arm64 test@mock:/home/air/digital_cam/.deploy.TEST1234/$name" "$DEPLOY_TEST_LOG"
done
! grep '^rsync ' "$DEPLOY_TEST_LOG" | grep -q -- '-native'
printf 'ARM64 deployment upload mapping tests passed\n'