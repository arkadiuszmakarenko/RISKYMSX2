"""
verify_upload.py - upload a small file and verify every byte matches.

Usage:
    python3 verify_upload.py /dev/ttyACM0 /path/to/some.bin
"""
import serial
import sys
import time

if len(sys.argv) != 3:
    print(__doc__)
    sys.exit(1)

PORT, FILE = sys.argv[1], sys.argv[2]
with open(FILE, "rb") as f:
    payload = f.read()

print(f"uploading {len(payload)} bytes from {FILE}")

s = serial.Serial(PORT, 115200, timeout=3)
s.reset_input_buffer()

# XLOAD header
s.write(f"XLOAD 0 {len(payload):x}\r\n".encode())
# Wait for READY (host writes to bridge -> cart -> READY echoed back)
buf = b""
t0 = time.monotonic()
while time.monotonic() - t0 < 2.0:
    if s.in_waiting:
        buf += s.read(s.in_waiting)
        if b"READY" in buf:
            break
if b"READY" not in buf:
    print("no READY:", buf)
    sys.exit(1)

# Drain the bridge echo buffer (it may have queued the echoed XLOAD line).
time.sleep(0.1)
s.reset_input_buffer()

# Stream payload
s.write(payload)
# Wait for OK
buf = b""
t0 = time.monotonic()
while time.monotonic() - t0 < 2.0:
    if s.in_waiting:
        buf += s.read(s.in_waiting)
        if b"OK\r" in buf or b"\nOK" in buf:
            break
        if b"ERR" in buf:
            print("XLOAD failed:", buf)
            sys.exit(1)

# Drain
time.sleep(0.1)
s.reset_input_buffer()

# DUMP and capture
s.write(f"DUMP 0 {min(len(payload), 256):x}\r\n".encode())
time.sleep(0.3)
data = b""
t0 = time.monotonic()
while time.monotonic() - t0 < 1.0:
    if s.in_waiting:
        data += s.read(s.in_waiting)
    else:
        time.sleep(0.02)

# Find the data line.
dump_line = None
for ln in data.split(b"\r\n"):
    if ln.startswith(b"DUMP ") and b":" in ln:
        dump_line = ln
        break

if not dump_line:
    print("no DUMP response:", data)
    sys.exit(1)

# Parse hex bytes after the colon.
hex_part = dump_line.split(b":", 1)[1].strip()
hexes = hex_part.split()
readback = bytes(int(p, 16) for p in hexes)

# Compare
n = min(len(payload), len(readback))
mismatch = 0
for i in range(n):
    if payload[i] != readback[i]:
        mismatch += 1
        if mismatch <= 5:
            print(f"  off {i:5d}: want {payload[i]:02x} got {readback[i]:02x} <<")

if mismatch == 0 and len(payload) == len(readback):
    print(f"OK: all {len(payload)} bytes match exactly")
elif len(payload) != len(readback):
    print(f"length mismatch: sent {len(payload)}, got {len(readback)}")
    sys.exit(1)
else:
    print(f"FAIL: {mismatch} byte(s) differ")
    sys.exit(1)