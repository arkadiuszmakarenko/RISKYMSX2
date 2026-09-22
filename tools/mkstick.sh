#!/bin/bash
# mkstick.sh - prepare a USB stick for the RISKYMSX2 Nextor environment
# (docs/NEXTOR_PLAN.md section 6C).
#
# Nextor boots MSX-DOS from the stick's first FAT16/FAT32 primary
# partition; the boot chain needs NEXTOR.SYS in its root, plus
# COMMAND3.COM and the Nextor tools for a usable environment
# (DRVTEST.COM, MAPDRV.COM, ...).
#
# All source files come from the vendored Nextor tools disk image:
#   MSXSoftware/NextorDriver/vendor/nextor.dsk
# (Konamiman/Nextor "tools-disk-image-and-zip-v1.1" release - NEXTOR.SYS,
#  NEXTORJ.SYS, COMMAND3.COM and every tool .COM).
#
# Usage:
#   ./mkstick.sh                        # extract nextor-sys/ (no root)
#   sudo CONFIRM=/dev/sdX ./mkstick.sh --device /dev/sdX
#
# --device mode: creates ONE primary partition spanning the whole device,
# formats it (FAT16 when <=2 GiB - most compatible with DOS tools; FAT32
# above that - Nextor 3 reads it fine), and copies the system files.
# NOTHING is written unless CONFIRM matches the device path, and the
# script echoes the target first. Always double-check the device node -
# this erases it.

set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
DSK="$HERE/../MSXSoftware/NextorDriver/vendor/nextor.dsk"
MNT="$(mktemp -d /tmp/mkstick.XXXXXX)"
trap 'umount "$MNT" 2>/dev/null || true; rmdir "$MNT" 2>/dev/null || true' EXIT

if [[ "${1:-}" == "--device" ]]; then
    DEV="${2:?usage: mkstick.sh --device /dev/sdX}"
    [[ -b "$DEV" ]] || { echo "error: $DEV is not a block device" >&2; exit 1; }
    echo "About to ERASE $DEV and write a Nextor boot partition."
    echo "Run again with CONFIRM=$DEV on the command line to proceed:"
    echo "  sudo CONFIRM=$DEV $0 --device $DEV"
    [[ "${CONFIRM:-}" == "$DEV" ]] || exit 1
    # One primary partition over the whole device.
    size_mb=$(( $(blockdev --getsize64 "$DEV") / 1024 / 1024 ))
    if (( size_mb <= 2048 )); then fstype=fat16; else fstype=fat32; fi
    parted --script "$DEV" mklabel msdos mkpart primary 0% 100%
    partprobe "$DEV"
    PART="${DEV}1"
    if [[ $fstype == fat16 ]]; then
        mkfs.fat -F 16 -n NEXTOR "$PART"
    else
        mkfs.fat -F 32 -n NEXTOR "$PART"
    fi
    mount "$PART" "$MNT"
    mcopy -i "$DSK" -s ::/* "$MNT/" >/dev/null
    umount "$MNT"
    sync
    echo "Stick prepared: partition 1 ($fstype, label NEXTOR) with the"
    echo "Nextor system files + tools. Plug it into the RISKYMSX2 and"
    echo "power on WITHOUT holding GRAPH (or press F1 in the loader)."
    exit 0
fi

# No-args mode (no root needed): extract the system files so they can be
# copied onto any stick manually or mounted by an emulator.
mkdir -p "$HERE/nextor-sys"
mcopy -i "$DSK" -s ::/* "$HERE/nextor-sys/" >/dev/null
echo "Extracted the Nextor system files + tools to $HERE/nextor-sys/:"
ls "$HERE/nextor-sys"
echo
echo "The stick needs: a single primary FAT16/FAT32 partition with"
echo "NEXTOR.SYS in its root. Copy them over with any FAT tool, e.g.:"
echo "  mcopy -i <partition-image> nextor-sys/NEXTOR.SYS ::/"
echo "Or run: sudo CONFIRM=/dev/sdX $0 --device /dev/sdX"
