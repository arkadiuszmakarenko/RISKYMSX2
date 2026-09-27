#!/usr/bin/env python3
"""
Create a proper bootable 720K floppy image from the Nextor tools disk.
The vendor nextor.dsk is a tools disk (partition image), not bootable.
This creates a corrected floppy image with:
- Valid floppy BPB (hidden sectors = 0)
- NEXTOR.SYS + COMMAND3.COM with system+hidden attributes
- Proper boot sector code
"""
import struct
import shutil
from pathlib import Path

VENDOR_DSK = Path("/home/makaron/Repo/RISKYMSX2/MSXSoftware/NextorDriver/vendor/nextor.dsk")
OUTPUT_DSK = Path("/home/makaron/Repo/RISKYMSX2/MSXSoftware/NextorDriver/nextor_boot.dsk")

SECTOR_SIZE = 512
TOTAL_SECTORS = 1440  # 720K
SECTORS_PER_CLUSTER = 2
RESERVED_SECTORS = 1
NUM_FATS = 2
ROOT_ENTRIES = 112
SECTORS_PER_FAT = 3
SECTORS_PER_TRACK = 9
HEADS = 2
HIDDEN_SECTORS = 0      # FIXED: was 270008320
MEDIA_DESC = 0xF9

def read_vendor_files():
    """Extract NEXTOR.SYS and COMMAND3.COM from vendor disk."""
    with open(VENDOR_DSK, 'rb') as f:
        data = f.read()

    # Parse vendor BPB
    bpb_bytes_per_sec = struct.unpack('<H', data[11:13])[0]
    bpb_spc = data[13]
    bpb_reserved = struct.unpack('<H', data[14:16])[0]
    bpb_fats = data[16]
    bpb_root_entries = struct.unpack('<H', data[17:19])[0]
    bpb_spf = struct.unpack('<H', data[22:24])[0]

    fat_start = bpb_reserved
    fat_size = bpb_fats * bpb_spf
    root_start = fat_start + fat_size
    root_sectors = (bpb_root_entries * 32 + bpb_bytes_per_sec - 1) // bpb_bytes_per_sec
    data_start = root_start + root_sectors

    # Read root directory
    root_data = data[root_start * bpb_bytes_per_sec : (root_start + root_sectors) * bpb_bytes_per_sec]

    def parse_fat12(fat_data):
        clusters = []
        for i in range(0, len(fat_data), 3):
            if i + 2 >= len(fat_data):
                break
            b0, b1, b2 = fat_data[i], fat_data[i+1], fat_data[i+2]
            c1 = b0 | ((b1 & 0x0F) << 8)
            c2 = (b1 >> 4) | (b2 << 4)
            clusters.append(c1)
            clusters.append(c2)
        return clusters

    fat1 = data[fat_start * bpb_bytes_per_sec : (fat_start + bpb_spf) * bpb_bytes_per_sec]
    fat_clusters = parse_fat12(fat1)

    def read_file(cluster, size):
        """Read file data from cluster chain."""
        result = bytearray()
        while cluster < 0xFF8 and len(result) < size:
            sector = data_start + (cluster - 2) * bpb_spc
            for s in range(bpb_spc):
                offset = (sector + s) * bpb_bytes_per_sec
                chunk = data[offset : offset + bpb_bytes_per_sec]
                take = min(len(chunk), size - len(result))
                result.extend(chunk[:take])
                if len(result) >= size:
                    break
            cluster = fat_clusters[cluster]
        return bytes(result[:size])

    # Find files in root directory
    file_info = {}  # name -> {cluster, size, attr}
    for i in range(bpb_root_entries):
        entry = root_data[i*32 : (i+1)*32]
        if entry[0] == 0x00:
            break
        if entry[0] == 0xE5:
            continue
        if entry[11] & 0x08:  # volume label
            continue
        name = entry[0:11].decode('ascii', errors='ignore').rstrip()
        attr = entry[11]
        cluster = struct.unpack('<H', entry[26:28])[0]
        size = struct.unpack('<I', entry[28:32])[0]
        if name in ('NEXTOR  SYS', 'COMMAND3COM'):
            file_info[name] = {'cluster': cluster, 'size': size, 'attr': attr}
            print(f"Found {name}: cluster={cluster}, size={size}, attr={hex(attr)}")

    # Read file data
    file_data = {}
    for name, info in file_info.items():
        file_data[name] = read_file(info['cluster'], info['size'])
        print(f"  Read {len(file_data[name])} bytes for {name}")

    return file_info, file_data

def create_boot_sector():
    """Create a proper floppy boot sector (720K, hidden=0)."""
    boot = bytearray(SECTOR_SIZE)

    # Jump instruction (JP to bootstrap code at offset 0x3E)
    boot[0:3] = b'\xEB\x3E\x90'

    # OEM ID
    boot[3:11] = b'NEXTOR3 '

    # BPB
    boot[11:13] = struct.pack('<H', SECTOR_SIZE)        # bytes per sector
    boot[13] = SECTORS_PER_CLUSTER                       # sectors per cluster
    boot[14:16] = struct.pack('<H', RESERVED_SECTORS)    # reserved sectors
    boot[16] = NUM_FATS                                  # number of FATs
    boot[17:19] = struct.pack('<H', ROOT_ENTRIES)        # root entries
    boot[19:21] = struct.pack('<H', TOTAL_SECTORS)       # total sectors (16-bit)
    boot[21] = MEDIA_DESC                                # media descriptor
    boot[22:24] = struct.pack('<H', SECTORS_PER_FAT)     # sectors per FAT
    boot[24:26] = struct.pack('<H', SECTORS_PER_TRACK)   # sectors per track
    boot[26:28] = struct.pack('<H', HEADS)               # heads
    boot[28:32] = struct.pack('<I', HIDDEN_SECTORS)      # hidden sectors (FIXED to 0)
    # NOTE: 32-bit total sectors at offset 0x20 is ONLY present when
    # 16-bit total sectors (offset 0x13) is 0. Ours is 1440 (non-zero),
    # so we MUST NOT write at 0x20-0x23. Extended boot record starts at 0x24.

    # Extended BPB (FAT12/16) starts at offset 0x24 (36)
    boot[36] = 0x00  # physical drive number
    boot[37] = 0x00  # reserved
    boot[38] = 0x29  # extended boot signature
    boot[39:43] = struct.pack('<I', 0x12345678)  # volume serial
    boot[43:54] = b'RISKYMSX2  '  # volume label (11 bytes)
    boot[54:62] = b'FAT12   '     # filesystem type

    # Bootstrap code area (offset 0x3E onwards)
    # We'll copy the vendor's bootstrap code but it should work for floppy
    with open(VENDOR_DSK, 'rb') as f:
        vendor = f.read()
    # Copy from offset 0x3E to 0x1FE (bootstrap code)
    boot[0x3E:0x1FE] = vendor[0x3E:0x1FE]

    # Boot signature
    boot[0x1FE:0x200] = b'\x55\xAA'

    return bytes(boot)

def build_fat12(root_entries, files, file_data):
    """Build FAT12 and directory for the new image."""
    # Calculate layout
    root_dir_sectors = (root_entries * 32 + SECTOR_SIZE - 1) // SECTOR_SIZE
    fat_start = RESERVED_SECTORS
    fat_size = NUM_FATS * SECTORS_PER_FAT
    root_start = fat_start + fat_size
    data_start = root_start + root_dir_sectors

    total_data_sectors = TOTAL_SECTORS - data_start
    total_clusters = total_data_sectors // SECTORS_PER_CLUSTER

    # Allocate clusters for files - FAT must be exactly SECTORS_PER_FAT * SECTOR_SIZE bytes
    fat_size_bytes = SECTORS_PER_FAT * SECTOR_SIZE
    fat = bytearray(fat_size_bytes)
    # Cluster 0: media descriptor in low 12 bits
    # Cluster 1: end of chain (0xFFF)
    set_fat12(fat, 0, (0xFF0 | MEDIA_DESC))  # cluster 0
    set_fat12(fat, 1, 0xFFF)                 # cluster 1

    cluster = 2
    file_info = {}  # name -> (start_cluster, size)

    for name in ['NEXTOR  SYS', 'COMMAND3COM']:
        if name not in files:
            continue
        data = file_data[name]
        size = len(data)
        sectors_needed = (size + SECTOR_SIZE - 1) // SECTOR_SIZE
        clusters_needed = (sectors_needed + SECTORS_PER_CLUSTER - 1) // SECTORS_PER_CLUSTER

        start_cluster = cluster
        file_info[name] = (start_cluster, size)

        for i in range(clusters_needed):
            if i == clusters_needed - 1:
                set_fat12(fat, cluster, 0xFFF)
            else:
                set_fat12(fat, cluster, cluster + 1)
            cluster += 1

    # Build root directory
    root_dir = bytearray(root_dir_sectors * SECTOR_SIZE)
    entry_idx = 0

    for name in ['NEXTOR  SYS', 'COMMAND3COM']:
        if name not in file_info:
            continue
        start_cluster, size = file_info[name]
        entry = bytearray(32)
        entry[0:11] = name.ljust(11).encode('ascii')
        entry[11] = 0x27  # system + hidden + archive
        entry[22:24] = b'\x00\x00'  # time
        entry[24:26] = b'\x00\x00'  # date
        entry[26:28] = struct.pack('<H', start_cluster)
        entry[28:32] = struct.pack('<I', size)
        root_dir[entry_idx*32 : (entry_idx+1)*32] = entry
        entry_idx += 1

    return bytes(fat), bytes(root_dir), data_start, file_info

def set_fat12(fat, cluster, value):
    """Set a FAT12 entry."""
    offset = cluster + (cluster // 2)
    if cluster & 1:
        # High 12 bits
        fat[offset] = (fat[offset] & 0x0F) | ((value & 0x0F) << 4)
        fat[offset + 1] = (value >> 4) & 0xFF
    else:
        # Low 12 bits
        fat[offset] = value & 0xFF
        fat[offset + 1] = (fat[offset + 1] & 0xF0) | ((value >> 8) & 0x0F)

def main():
    print("Reading vendor disk...")
    file_info, file_data = read_vendor_files()

    print("Creating boot sector...")
    boot_sector = create_boot_sector()

    print("Building FAT and directory...")
    fat, root_dir, data_start, file_info_out = build_fat12(ROOT_ENTRIES, file_info, file_data)

    print("Writing output image...")
    with open(OUTPUT_DSK, 'wb') as f:
        # Boot sector
        f.write(boot_sector)
        # FATs
        f.write(fat)
        f.write(fat)  # second FAT
        # Root directory
        f.write(root_dir)
        # Data area - write files at their cluster positions
        data_area_size = (TOTAL_SECTORS - data_start) * SECTOR_SIZE
        data_area = bytearray(data_area_size)
        for name, (start_cluster, size) in file_info_out.items():
            data = file_data[name]
            cluster = start_cluster
            remaining = size
            while remaining > 0 and cluster < 0xFF8:
                cluster_offset = data_start + (cluster - 2) * SECTORS_PER_CLUSTER
                sector_offset = cluster_offset * SECTOR_SIZE
                take = min(remaining, SECTORS_PER_CLUSTER * SECTOR_SIZE)
                rel_offset = sector_offset - data_start * SECTOR_SIZE
                data_area[rel_offset : rel_offset + take] = data[size - remaining : size - remaining + take]
                remaining -= take
                cluster += 1  # sequential clusters

        f.write(data_area)

    print(f"Done! Wrote {OUTPUT_DSK} ({TOTAL_SECTORS * SECTOR_SIZE} bytes)")
    print(f"Copy this to USB stick as NEXTOR.DSK (or RISKYMSX.DSK)")

if __name__ == '__main__':
    main()