#!/usr/bin/env bash
set -euo pipefail

INPUT="${1:-}"
DEV="${2:-}"

if [ -z "$INPUT" ] || [ -z "$DEV" ]; then
    echo "Usage: sudo $0 <backup_file.img.xz> <target_device>"
    echo "Example: sudo $0 my_backup.img.xz /dev/sdb"
    exit 1
fi

if [ "$EUID" -ne 0 ]; then
    echo "Error: Must be run as root (use sudo)."
    exit 1
fi

if [ ! -f "$INPUT" ]; then
    echo "Error: Backup file '$INPUT' does not exist."
    exit 1
fi

if [ ! -b "$DEV" ]; then
    echo "Error: Target device '$DEV' does not exist or is not a block device."
    exit 1
fi

echo "NOTE: The whole process can take a LOOONNGGGGG time..."
echo "...just wait and let it do it's thing!"
echo "==> Step 1: Unmounting active partitions on $DEV..."
umount "${DEV}"* 2>/dev/null || true
umount "${DEV}"p* 2>/dev/null || true

echo "==> Step 2: Extracting uncompressed size..."
UNCOMPRESSED_BYTES=""

# Attempt 1: Robot parser
if command -v xz &>/dev/null; then
    UNCOMPRESSED_BYTES=$(xz --robot -l "$INPUT" 2>/dev/null | awk '/^totals/ {print $5}' || true)
fi

# Attempt 2: Standard header parser fallback
if [ -z "$UNCOMPRESSED_BYTES" ] || [ "$UNCOMPRESSED_BYTES" -eq 0 ]; then
    UNCOMPRESSED_BYTES=$(xz -l "$INPUT" 2>/dev/null | awk '/Totals:/ {print $5}' | tr -d ' \t\r\n' || true)
fi

# Force full progress bar layout (-p % bar, -t time, -e ETA, -r rate, -b byte counter)
if [ -n "$UNCOMPRESSED_BYTES" ] && [ "$UNCOMPRESSED_BYTES" -gt 0 ]; then
    HUMAN_SIZE=$(numfmt --to=iec-i --suffix=B "$UNCOMPRESSED_BYTES" 2>/dev/null || echo "$UNCOMPRESSED_BYTES bytes")
    echo "==> Uncompressed payload size: $HUMAN_SIZE ($UNCOMPRESSED_BYTES bytes)"
    PV_ARGS=("-p" "-t" "-e" "-r" "-b" "-s" "$UNCOMPRESSED_BYTES")
else
    echo "==> Size header unavailable; rendering dynamic rate tracker..."
    PV_ARGS=("-p" "-t" "-r" "-b")
fi

echo "==> Step 3: Writing directly to SD card hardware..."
echo "    Even at 100% done it can take a LOOONNGG time for the next step."
echo "    Just let it run, it will finish!"
echo ""

xz -dc -T0 "$INPUT" | pv "${PV_ARGS[@]}" | dd of="$DEV" bs=4M oflag=direct status=none

echo ""
echo "==> Step 4: Refreshing partition table..."
partprobe "$DEV" 2>/dev/null || true
udevadm settle 2>/dev/null || true
sleep 2

echo "==> Step 5: Detecting rootfs partition..."
PART_NUM=$(lsblk -nl -o NAME,TYPE "$DEV" | awk '$2=="part"{print $1}' | tail -n1 | grep -o '[0-9]*$')

if [ -z "$PART_NUM" ]; then
    echo "Error: Could not detect any partitions on $DEV after restore."
    exit 1
fi

if [[ "$DEV" =~ [0-9]$ ]]; then
    PART_DEV="${DEV}p${PART_NUM}"
else
    PART_DEV="${DEV}${PART_NUM}"
fi

echo "==> Target partition: $PART_DEV (Partition #$PART_NUM)"

echo "==> Step 6: Resizing partition boundary to 100% capacity..."
parted -s "$DEV" resizepart "$PART_NUM" 100%

echo "==> Refreshing kernel partition table..."
partprobe "$DEV" 2>/dev/null || true
udevadm settle 2>/dev/null || true
sleep 2

echo "==> Step 7: Checking EXT4 filesystem consistency..."
e2fsck -fy "$PART_DEV" || true

echo "==> Step 8: Expanding EXT4 filesystem..."
resize2fs "$PART_DEV"

echo ""
echo "==> Restore and expansion complete!"
echo "==> Final partition layout:"
lsblk "$DEV"
