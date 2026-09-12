#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)

MORA_SSH_TARGET=${1:-air@theatre}
MORA_INSTALL_DIR=${2:-/home/air/digital_cam}
BUILD_JOBS=${BUILD_JOBS:-$(nproc)}

if [[ "$MORA_INSTALL_DIR" != "/home/air/digital_cam" ]]; then
  echo "deploy_mora.sh: catia.service requires /home/air/digital_cam" >&2
  exit 2
fi

for required_command in aarch64-linux-gnu-gcc aarch64-linux-gnu-g++ \
  aarch64-linux-gnu-readelf aarch64-linux-gnu-strip file make rsync ssh; do
  if ! command -v "$required_command" >/dev/null 2>&1; then
    echo "deploy_mora.sh: required command not found: $required_command" >&2
    exit 1
  fi
done

echo "MORA CATIA: cross-compiling for ARM64"
make -C "$SCRIPT_DIR" -j"$BUILD_JOBS" arm64 \
  OPTFLAGS="-O2 -g0" MORA_INSTALL_DIR="$MORA_INSTALL_DIR"

echo "MORA CATIA: testing earcam natively"
make -C "$SCRIPT_DIR/earcam" -j"$BUILD_JOBS" test

echo "MORA CATIA: stripping release binaries"
aarch64-linux-gnu-strip --strip-all \
  "$SCRIPT_DIR/catia-arm64" \
  "$SCRIPT_DIR/soda-arm64" \
  "$SCRIPT_DIR/lwircam-arm64" \
  "$SCRIPT_DIR/earcam-arm64"

for executable in catia-arm64 soda-arm64 lwircam-arm64 earcam-arm64; do
  executable_path="$SCRIPT_DIR/$executable"
  binary_description=$(file -b "$executable_path")
  if [[ "$binary_description" != *"ARM aarch64"* ]]; then
    echo "deploy_mora.sh: $executable is not an AArch64 binary: $binary_description" >&2
    exit 1
  fi
  if [[ "$binary_description" != *"stripped"* || "$binary_description" == *"not stripped"* ]]; then
    echo "deploy_mora.sh: $executable is not stripped: $binary_description" >&2
    exit 1
  fi
  if aarch64-linux-gnu-readelf --sections "$executable_path" | grep '\.debug' >/dev/null; then
    echo "deploy_mora.sh: $executable still contains debug sections" >&2
    exit 1
  fi
  echo "MORA CATIA: verified $executable: $binary_description"
done

echo "MORA CATIA: staging $MORA_SSH_TARGET:$MORA_INSTALL_DIR"
stage=$(ssh "$MORA_SSH_TARGET" \
  "mkdir -p '$MORA_INSTALL_DIR' && mktemp -d '$MORA_INSTALL_DIR/.deploy.XXXXXXXX'")
if [[ ! "$stage" =~ ^/home/air/digital_cam/\.deploy\.[a-zA-Z0-9]+$ ]]; then
  echo "deploy_mora.sh: invalid remote staging path" >&2
  exit 1
fi

for executable in catia soda lwircam earcam; do
  rsync --archive --human-readable --info=progress2 --chmod=F755 \
    "$SCRIPT_DIR/$executable-arm64" "$MORA_SSH_TARGET:$stage/$executable"
done
rsync --archive --human-readable --info=progress2 --chmod=F644 \
  "$SCRIPT_DIR/mock_image_01.jpg" \
  "$SCRIPT_DIR/lwircam/mock_lwir_01.jpg" \
  "$SCRIPT_DIR/99-tiny1c.rules" "$SCRIPT_DIR/catia.service" \
  "$SCRIPT_DIR/deploy_mora_remote.sh" \
  "$MORA_SSH_TARGET:$stage/"

echo "MORA CATIA: validating and activating staged release"
if ! ssh "$MORA_SSH_TARGET" "if [[ -f /home/air/pass.txt ]]; then sudo -S -v < /home/air/pass.txt 2>/dev/null || true; elif [[ -f \$HOME/pass.txt ]]; then sudo -S -v < \$HOME/pass.txt 2>/dev/null || true; fi; bash '$stage/deploy_mora_remote.sh' '$MORA_INSTALL_DIR' '$stage'"; then
  echo "deploy_mora.sh: activation failed; inspect rollback output and $stage/backup" >&2
  ssh "$MORA_SSH_TARGET" \
    "if [[ -f /home/air/pass.txt ]]; then sudo -S -v < /home/air/pass.txt 2>/dev/null || true; elif [[ -f \$HOME/pass.txt ]]; then sudo -S -v < \$HOME/pass.txt 2>/dev/null || true; fi; \
     sudo -n systemctl --no-pager --full status catia.service || true; \
     sudo -n journalctl --no-pager -u catia.service -n 50 || true"
  exit 1
fi

echo "MORA CATIA: deployment complete"
echo "MORA CATIA: service status: ssh $MORA_SSH_TARGET 'systemctl status catia.service'"
echo "MORA CATIA: live logs: ssh $MORA_SSH_TARGET 'journalctl -fu catia.service'"