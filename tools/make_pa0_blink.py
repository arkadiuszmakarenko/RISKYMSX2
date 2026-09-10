#!/usr/bin/env python3
"""
make_pa0_blink.py - emit a tiny RV32I binary that toggles PA0 on a CH32V407V.

Used as a test payload for XLOAD_PSRAM/RUN_PSRAM to verify whether code
running from external PSRAM can keep up (toggle frequency tells you the
fetch cost).

Usage:
    python3 make_pa0_blink.py test_blink.bin
    python3 load.py /dev/ttyACM0 test_blink.bin --dest psram --addr 0 --run psram
    python3 load.py /dev/ttyACM0 test_blink.bin --dest sram  --addr 0 --run sram

Scope PA0 (LED pin on the cart). Compare toggle period between the two
runs:
  - SRAM: tight ~100 ns toggle (zero-wait-state)
  - PSRAM: stretched toggle proportional to FSMC wait states, OR hang
"""
import struct
import sys

# CH32V407V GPIOA base = 0x40010800
GPIOA_BASE = 0x40010800
# Register offsets:
#   CFGHR  = +0x04 (pins 8..15; PA0 is in CFGLR though)
#   CFGLR  = +0x00 (pins 0..7)
#   OUTDR  = +0x0C
#   BSRR   = +0x10  (bit-set: write 1 to bit n -> set)
#   BCR    = +0x14  (bit-reset: write 1 to bit n -> clear)
# Use OUTDR via XOR so we don't need to read-modify-write.

# Hand-assembled RV32I:
#   lui  a0, 0x40011         ; a0 = 0x40011000 (GPIOA base)
#   addi a0, a0, -0x800      ; a0 = 0x40010800
#   addi a1, zero, 0x4       ; a1 = 4 (CFGLR offset)
#   lui  a2, 0x44444         ; a2 = 0x44444000
#   addi a2, a2, 0x444       ; a2 = 0x44444444 (PA0 push-pull out)
#   sw   a2, 0(a1 + a0)      ; *(CFGHR-ish)*= use 0x0(a0) for CFGLR
#   addi a3, zero, 0xc       ; a3 = 12 (OUTDR offset)
#   addi a4, zero, 1         ; a4 = 1 (PA0 mask)
# loop:
#   lw   a5, 0(a0 + a3)      ; a5 = OUTDR
#   xor  a5, a5, a4          ; toggle PA0
#   sw   a5, 0(a0 + a3)
#   j loop

# Encode each instruction.
def r(type_, *args):
    return (type_ | args).to_bytes(4, 'little')

LUI    = lambda rd, imm: r(0b0110111 | (rd << 7), (imm >> 12) & 0xFFFFF)
ADDI   = lambda rd, rs1, imm: r(
    0b0010011 | (rd << 7) | (rs1 << 15),
    ((imm >> 5) & 1) << 30, ((imm >> 0) & 0x1F) << 20,
    ((imm >> 12) & 1) << 31, (imm & 0xFFF) << 20,
)
# Actually ADDI uses I-type encoding: imm[11:0] | rs1 | funct3 | rd | opcode
def ADDI2(rd, rs1, imm):
    imm12 = imm & 0xFFF
    return ((imm12 << 20) | (rs1 << 15) | (0b000 << 12) | (rd << 7) | 0b0010011).to_bytes(4, 'little')

def LUI2(rd, imm20):
    return ((imm20 << 12) | (rd << 7) | 0b0110111).to_bytes(4, 'little')

def SW(rs2, rs1, imm):
    imm12 = imm & 0xFFF
    # S-type: imm[11:5] | rs2 | rs1 | funct3 | imm[4:0] | opcode
    hi = (imm12 >> 5) & 0x7F
    lo = imm12 & 0x1F
    return (
        (hi << 25) | (rs2 << 20) | (rs1 << 15) |
        (0b010 << 12) | (lo << 7) | 0b0100011
    ).to_bytes(4, 'little')

def LW(rd, rs1, imm):
    imm12 = imm & 0xFFF
    return (
        (imm12 << 20) | (rs1 << 15) |
        (0b010 << 12) | (rd << 7) | 0b0000011
    ).to_bytes(4, 'little')

def XOR(rd, rs1, rs2):
    return (
        (0 << 25) | (rs2 << 20) | (rs1 << 15) |
        (0b100 << 12) | (rd << 7) | 0b0110011
    ).to_bytes(4, 'little')

# Hand-pick immediates for LUI+ADDI pairs that build 0x40010800 and 0x44444444.
# lui rd, imm20 sets rd = imm20 << 12. So:
#   lui  a0, 0x40011   -> a0 = 0x40011000
#   addi a0, a0, -2048 -> a0 = 0x40011000 - 0x800 = 0x40010800. ADDI imm12 = 0x800 = 2048, but it's a signed 12-bit (-2048).
import struct
prog = b''
prog += LUI2(10, 0x40011)            # a0 = 0x40011000
prog += ADDI2(10, 10, -0x800)        # a0 = 0x40010800
prog += ADDI2(11, 0, 0)              # a1 = 0 (offset 0 = CFGLR)
prog += LUI2(12, 0x44444)            # a2 = 0x44444000
prog += ADDI2(12, 12, 0x444)         # a2 = 0x44444444
prog += SW(12, 10, 0)               # store a2 -> 0(a0)
prog += ADDI2(13, 0, 0x0C)           # a3 = 12 (OUTDR offset)
prog += ADDI2(14, 0, 1)              # a4 = 1 (PA0 mask)
# loop at offset = len(prog)
loop_off = len(prog)
prog += LW(15, 10, 0x0C)             # a5 = OUTDR (offset 12)
prog += XOR(15, 15, 14)              # toggle PA0
prog += SW(15, 10, 0x0C)             # store OUTDR
prog += struct.pack('<I', 0x6F | (loop_off & 0xFFFE) << 12 | (loop_off & 1) << 31 | (((loop_off >> 1) & 0x3FF) << 21) | (0 << 20) | (0 << 15) | (0 << 12) | (0 << 7))
# Actually JAL encoding is easier if we use a forward branch.
# The instruction above is wrong. Recompute:
prog = prog[:-4]
# jal x0, loop_off  -> J-type encoding
# imm[20] | imm[10:1] | imm[11] | imm[19:12] | rd | opcode
def JAL(rd, imm):
    # imm is signed; offset in bytes from current PC
    imm20 = imm & 0x1FFFFF
    b20 = (imm20 >> 20) & 1
    b10_1 = (imm20 >> 1) & 0x3FF
    b11 = (imm20 >> 11) & 1
    b19_12 = (imm20 >> 12) & 0xFF
    return (
        (b20 << 31) | (b10_1 << 21) | (b11 << 20) |
        (b19_12 << 12) | (rd << 7) | 0b1101111
    ).to_bytes(4, 'little')

prog += JAL(0, loop_off)             # j loop

out_path = sys.argv[1] if len(sys.argv) > 1 else 'test_blink.bin'
with open(out_path, 'wb') as f:
    f.write(prog)

print(f"wrote {len(prog)} bytes to {out_path}")