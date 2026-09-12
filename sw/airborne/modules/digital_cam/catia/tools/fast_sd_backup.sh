#!/usr/bin/env bash
set -euo pipefail

DEV="${1:-}"
OUTPUT="${2:-sdcard_backup_$(date +%Y%m%d_%H%M%S).img.xz}"

if [ -z "$DEV" ]; then
    echo "Usage: sudo $0 /dev/sdX [output_filename.img.xz]"
    exit 1
fi

if [ "$EUID" -ne 0 ]; then
    echo "Error: Must be run as root (use sudo)."
    exit 1
fi

if [ ! -b "$DEV" ]; then
    echo "Error: Device $DEV does not exist or is not a block device."
    exit 1
fi

echo "==> Unmounting active partitions on $DEV..."
umount "${DEV}"* 2>/dev/null || true
umount "${DEV}"p* 2>/dev/null || true

echo "==> Detecting partition boundary..."
SECTOR_SIZE=$(blockdev --getss "$DEV" 2>/dev/null || echo 512)

# Parse sfdisk dump to find the highest end sector among all partitions
END_SECTOR=$(sfdisk -d "$DEV" | awk -F'[,=]' '/start=/ {
    s=0; z=0;
    for(i=1; i<=NF; i++) {
        if($i ~ /start/) s=$(i+1);
        if($i ~ /size/) z=$(i+1);
    }
    end = s + z - 1;
    if(end > max) max = end;
} END { print max }')

if [ -z "$END_SECTOR" ] || [ "$END_SECTOR" -eq 0 ]; then
    echo "Error: Could not detect any valid partitions on $DEV."
    exit 1
fi

BYTES=$(( (END_SECTOR + 1) * SECTOR_SIZE ))
HUMAN_SIZE=$(numfmt --to=iec-i --suffix=B "$BYTES" 2>/dev/null || echo "$BYTES bytes")

echo "==> Last used sector:  $END_SECTOR"
echo "==> Backup target size: $HUMAN_SIZE ($BYTES bytes)"
echo "==> Output destination: $OUTPUT"
echo ""

# Pipeline: dd truncated byte read -> pv with forced status & ETA -> xz multi-threaded compression
dd if="$DEV" bs=4M count="$BYTES" iflag=count_bytes status=none | \
    pv -f -s "$BYTES" | \
    xz -1 -T0 > "$OUTPUT"

echo ""
echo "==> Backup complete: $OUTPUT"