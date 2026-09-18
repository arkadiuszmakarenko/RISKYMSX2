#!/usr/bin/env python3
"""extract_payload.py - stage-2 helper for the RomLoader Makefile.

Reads payload.flat.bin (64 KiB makebin output of the loader linked at
0xC000) + payload.map, writes payload.bin = the exact span the stub
must copy: 0xC000 .. (s__DATA + l__DATA), i.e. code + data (the SDCC
z80 port merges zero-initialised globals into _DATA right after
_CODE, so one contiguous copy covers everything).

NOTE: makebin fills gaps with 0xFF (not 0x00), so "trim trailing
zeros" is WRONG - the span must come from the linker map.
"""
data = open('payload.flat.bin', 'rb').read()

# Parse s__DATA / l__DATA from the map (lowercase symbol spelling).
bss_base = bss_len = 0
for line in open('payload.map'):
    parts = line.split()
    # map format: "<hex-value>  <symbol>"  (value first, symbol second)
    if len(parts) >= 2 and parts[1] == 's__DATA':
        bss_base = int(parts[0], 16)
    if len(parts) >= 2 and parts[1] == 'l__DATA':
        bss_len = int(parts[0], 16)
    if bss_base and bss_len:
        break
if bss_base == 0:
    raise SystemExit('extract_payload: s__DATA/l__DATA not found in map')

start = 0xC000
end = bss_base + bss_len          # exclusive
span = end - start
open('payload.bin', 'wb').write(data[start:end])
print('payload: %d bytes (code+data, 0x%04X..0x%04X)'
      % (span, start, end - 1))