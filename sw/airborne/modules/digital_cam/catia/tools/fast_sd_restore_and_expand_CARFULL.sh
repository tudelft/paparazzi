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

echo "==> Step 1: Unmounting all active partitions on $DEV..."
umount "${DEV}"* 2>/dev/null || true
umount "${DEV}"p* 2>/dev/null || true

echo "==> Step 2: Restoring image..."
pv "$INPUT" | xz -dc -T0 | dd of="$DEV" bs=4M status=none conv=fsync

echo "==> Informing kernel of partition changes..."
partprobe "$DEV" 2>/dev/null || true
sleep 2

echo "==> Step 3: Detecting last partition number..."
PART_NUM=$(lsblk -nl -o NAME,TYPE "$DEV" | awk '$2=="part"{print $1}' | tail -n1 | grep -o '[0-9]*$')

if [ -z "$PART_NUM" ]; then
    echo "Error: Could not detect any partitions on $DEV after restore."
    exit 1
fi

# Construct partition path handling mmcblkXpY vs sdXY naming
if [[ "$DEV" =~ [0-9]$ ]]; then
    PART_DEV="${DEV}p${PART_NUM}"
else
    PART_DEV="${DEV}${PART_NUM}"
fi

echo "==> Detected rootfs partition: $PART_DEV (Partition #$PART_NUM)"

echo "==> Step 4: Resizing partition boundary to 100% capacity..."
parted -s "$DEV" resizepart "$PART_NUM" 100%

echo "==> Refreshing kernel partition table..."
partprobe "$DEV" 2>/dev/null || true
sleep 2

echo "==> Step 5: Checking EXT4 filesystem consistency..."
e2fsck -fy "$PART_DEV" || true

echo "==> Step 6: Expanding EXT4 filesystem to fill partition..."
resize2fs "$PART_DEV"

echo ""
echo "==> Restore and expansion complete!"
echo "==> Verified layout:"
lsblk "$DEV"