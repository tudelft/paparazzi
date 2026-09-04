#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
PAPARAZZI_HOME=$(cd -- "$SCRIPT_DIR/../../../../.." && pwd)

MORA_SSH_TARGET=${1:-air@theatre}
MORA_INSTALL_DIR=${2:-/home/air/digital_cam}
BUILD_JOBS=${BUILD_JOBS:-$(nproc)}
RESTORE_SERVICE_ON_FAILURE=false

restore_service_on_failure()
{
  exit_status=$?
  trap - EXIT
  if [[ $exit_status -ne 0 && "$RESTORE_SERVICE_ON_FAILURE" == true ]]; then
    echo "MORA CATIA: deployment failed; restarting the previous service" >&2
    ssh "$MORA_SSH_TARGET" "sudo -n systemctl start catia.service" || true
  fi
  exit "$exit_status"
}

trap restore_service_on_failure EXIT

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
make -C "$SCRIPT_DIR" clean
make -C "$SCRIPT_DIR" -j"$BUILD_JOBS" \
  CC=aarch64-linux-gnu-gcc \
  CXX=aarch64-linux-gnu-g++ \
  OPTFLAGS="-O2 -g0" \
  CATIA_SERIAL_DEVICE=/dev/serial0 \
  CATIA_MOCK_IMAGE="$MORA_INSTALL_DIR/mock_image_01.jpg" \
  CATIA_LOCAL_SODA="$MORA_INSTALL_DIR/soda_local" \
  CATIA_LOCAL_PHOTO_DIR="$MORA_INSTALL_DIR/photos" \
  CATIA_AI_CAM_PHOTO_DIR="$MORA_INSTALL_DIR/photos" \
  CATIA_LWIR_CAM_PHOTO_DIR="$MORA_INSTALL_DIR/photos" \
  CATIA_LWIR_CAM_COMMAND="$MORA_INSTALL_DIR/sample" \
  CATIA_CHDK_PHOTO_DIR="$MORA_INSTALL_DIR/photos" \
  LWIR_ARCH=aarch64-gnu \
  LWIR_STATIC=1 \
  LWIR_VIDEO_DISPLAY=0

echo "MORA CATIA: stripping release binaries"
aarch64-linux-gnu-strip --strip-all \
  "$SCRIPT_DIR/catia" \
  "$SCRIPT_DIR/soda_local" \
  "$SCRIPT_DIR/lwircam/sample"

for executable in catia soda_local lwircam/sample; do
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

echo "MORA CATIA: preparing $MORA_SSH_TARGET:$MORA_INSTALL_DIR"
ssh "$MORA_SSH_TARGET" \
  "mkdir -p '$MORA_INSTALL_DIR' '$MORA_INSTALL_DIR/photos'"

if ssh "$MORA_SSH_TARGET" "sudo -n systemctl is-enabled --quiet catia.service"; then
  RESTORE_SERVICE_ON_FAILURE=true
fi
echo "MORA CATIA: stopping the managed service for deployment"
ssh "$MORA_SSH_TARGET" \
  "sudo -n systemctl stop catia.service 2>/dev/null || true"
if ssh "$MORA_SSH_TARGET" \
  "pgrep -af '^$MORA_INSTALL_DIR/(catia|sample)( |$)'"; then
  RESTORE_SERVICE_ON_FAILURE=false
  echo "deploy_mora.sh: stop the unmanaged CATIA process shown above and retry" >&2
  exit 1
fi

echo "MORA CATIA: installing Tiny 1-C USB permissions"
rsync --archive --chmod=F644 \
  "$SCRIPT_DIR/99-tiny1c.rules" "$SCRIPT_DIR/catia.service" \
  "$MORA_SSH_TARGET:$MORA_INSTALL_DIR/"
ssh "$MORA_SSH_TARGET" \
  "sudo -n install -m 0644 '$MORA_INSTALL_DIR/99-tiny1c.rules' /etc/udev/rules.d/99-tiny1c.rules && \
   sudo -n udevadm control --reload-rules && \
   sudo -n udevadm trigger --action=add --subsystem-match=usb --attr-match=idVendor=0bda --attr-match=idProduct=5840"

echo "MORA CATIA: transferring executables"
rsync --archive --human-readable --info=progress2 --chmod=F755 \
  "$SCRIPT_DIR/catia" "$SCRIPT_DIR/soda_local" "$SCRIPT_DIR/lwircam/sample" \
  "$MORA_SSH_TARGET:$MORA_INSTALL_DIR/"

echo "MORA CATIA: transferring mock test image"
rsync --archive --human-readable --info=progress2 --chmod=F644 \
  "$SCRIPT_DIR/mock_image_01.jpg" \
  "$SCRIPT_DIR/lwircam/mock_lwir_photos/mock_lwir_01.jpg" \
  "$MORA_SSH_TARGET:$MORA_INSTALL_DIR/"

echo "MORA CATIA: installing and enabling catia.service"
ssh "$MORA_SSH_TARGET" \
  "sudo -n install -m 0644 '$MORA_INSTALL_DIR/catia.service' /etc/systemd/system/catia.service && \
   sudo -n systemd-analyze verify /etc/systemd/system/catia.service && \
   sudo -n systemctl daemon-reload && \
   sudo -n systemctl enable catia.service && \
   sudo -n systemctl restart catia.service"

if ! ssh "$MORA_SSH_TARGET" \
  "sudo -n systemctl is-enabled --quiet catia.service && \
  sudo -n systemctl is-active --quiet catia.service && \
  test \"\$(sudo -n systemctl show catia.service -p SubState --value)\" = running"; then
  echo "deploy_mora.sh: catia.service did not start" >&2
  ssh "$MORA_SSH_TARGET" \
    "sudo -n systemctl --no-pager --full status catia.service || true; \
     sudo -n journalctl --no-pager -u catia.service -n 50 || true"
  exit 1
fi

RESTORE_SERVICE_ON_FAILURE=false
echo "MORA CATIA: deployment complete"
echo "MORA CATIA: service status: ssh $MORA_SSH_TARGET 'systemctl status catia.service'"
echo "MORA CATIA: live logs: ssh $MORA_SSH_TARGET 'journalctl -fu catia.service'"