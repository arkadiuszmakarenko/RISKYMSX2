"""Verify XLOAD byte alignment - debug version with raw capture."""
import serial, time

payload = bytes(range(16))
print(f"payload hex: {payload.hex()}")

s = serial.Serial("/dev/ttyACM0", 115200, timeout=2)
s.reset_input_buffer()
s.write(f"XLOAD 0 {len(payload):x}\r\n".encode())
time.sleep(0.1)
buf = b""
t0 = time.monotonic()
while time.monotonic() - t0 < 1.0:
    if s.in_waiting:
        buf += s.read(s.in_waiting)
        if b"READY" in buf:
            break
print(f"handshake: {buf!r}")
s.write(payload)
time.sleep(0.3)
post = s.read(256)
print(f"post-load: {post!r}")

s.reset_input_buffer()
s.write(b"DUMP 0 16\r\n")
time.sleep(0.5)
data = b""
t0 = time.monotonic()
while time.monotonic() - t0 < 1.0:
    if s.in_waiting:
        data += s.read(s.in_waiting)
    else:
        time.sleep(0.02)
print(f"DUMP raw hex: {data.hex()}")
print(f"DUMP text   : {data.decode('latin1', errors='replace')}")

for ln in data.split(b"\r\n"):
    if ln.startswith(b"DUMP 0 16:"):
        hexes = ln.split(b":", 1)[1].strip().split()
        readback = bytes(int(p, 16) for p in hexes)
        print(f"readback hex: {readback.hex()}")
        if readback == payload:
            print("MATCH: XLOAD writes at exact requested offsets")
        else:
            print("MISMATCH:")
            for i in range(len(payload)):
                a = payload[i]
                b = readback[i] if i < len(readback) else 0
                mark = "  " if a == b else "<<"
                print(f"  off {i:2d}: want {a:02x} got {b:02x} {mark}")
        break
else:
    print("No DUMP data line found")