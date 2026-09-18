#!/usr/bin/env python3
"""assemble_rom.py - stage-3 helper for the RomLoader Makefile.

Assembles the final 32 KiB selector.bin from:
    stub.flat.bin - 64 KiB flat image containing header + copy stub
                    at 0x4000 (crt0.rel linked at its ROM address)
    payload.bin   - RAM-resident loader bytes (to sit at Z80 0x4040)

ROM layout produced (Z80 address == file offset + 0x4000):
    0x4000  "AB" header, INIT -> stub
    0x4010  payload_len (word, patched here after measuring payload.bin)
    0x4020  copy stub: LDIR payload -> 0xC000, JP 0xC000
    0x4040  payload bytes
    rest    0xFF padding
"""
import sys

if len(sys.argv) > 2:
    print(f"usage: {sys.argv[0]} [output.bin]", file=sys.stderr)
    sys.exit(2)

out_path = sys.argv[1] if len(sys.argv) == 2 else 'selector.bin'

stub = open('stub.flat.bin', 'rb').read()
payload = open('payload.bin', 'rb').read()

rom = bytearray(b'\xFF' * 32768)
# stub region covers 0x4000..0x4040 (header + payload_len slot + code)
rom[0x00:0x40] = stub[0x4000:0x4040]

# patch payload_len at file offset 0x10 (little-endian word)
span = len(payload)
rom[0x10] = span & 0xFF
rom[0x11] = span >> 8

# payload at file offset 0x40 (Z80 0x4040)
assert span <= 32768 - 0x40, 'payload too big for 32 KiB ROM'
rom[0x40:0x40 + span] = payload

assert rom[0] == 0x41 and rom[1] == 0x42, 'missing AB header'
open(out_path, 'wb').write(bytes(rom))
print('%s: 32 KiB (payload %d bytes at 0x4040)' % (out_path, span))