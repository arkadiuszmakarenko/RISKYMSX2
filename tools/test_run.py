"""Quick test: upload blink program to SRAM and RUN_SRAM."""
import serial, time

with open('/tmp/blink.bin', 'rb') as f:
    data = f.read()

s = serial.Serial('/dev/ttyACM0', 115200, timeout=2)
s.reset_input_buffer()

# 1. Upload to SRAM
header = f"XLOAD_SRAM 0 {len(data):x}\r\n".encode()
s.write(header)
time.sleep(0.1)
buf = b""
t0 = time.monotonic()
while time.monotonic() - t0 < 1.0:
    if s.in_waiting:
        buf += s.read(s.in_waiting)
        if b"READY" in buf: break
print("handshake:", buf.decode("latin1", errors="replace").strip())

s.write(data)
time.sleep(0.5)
post = s.read(256)
print("post-upload:", post.decode("latin1", errors="replace").strip())

# 2. Skip MSX auto-reset by waiting it out, then RUN
time.sleep(2)
s.reset_input_buffer()
s.write(b"RUN_SRAM 0\r\n")
time.sleep(1.0)
out = s.read(256)
print("RUN_SRAM:", repr(out.decode("latin1", errors="replace")))