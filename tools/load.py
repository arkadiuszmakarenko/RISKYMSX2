#!/usr/bin/env python3
"""
load.py - upload a .bin file to RISKYMSX2 over USART.

Supports two destinations and three modes:

Destinations (where the bytes are written):
    --dest sram     -> XLOAD_SRAM  -> internal SRAM cart mirror (32 KiB)
    --dest psram    -> XLOAD_PSRAM -> external PSRAM bus (1 MiB+)

Modes (what happens after the upload):
    (default) upload only
    --run sram      -> also issues RUN_SRAM <addr>  (jump to code in SRAM)
    --run psram     -> also issues RUN_PSRAM <addr> (jump to code in PSRAM)
    --reset-ms N    -> pulse MSX ~RESET for N ms (default 100, 0 disables)

Usage examples:
    # Upload Zaxxon into SRAM cart mirror and reboot MSX
    python3 load.py /dev/ttyACM0 Zaxxon.rom --dest sram

    # Upload code into PSRAM at 0x10000 and jump to it
    python3 load.py /dev/ttyACM0 test.bin --dest psram --addr 0x10000 --run psram

Protocol:
    host ->  "XLOAD_<SRAM|PSRAM> <addr> <len>\r\n"
    cart -> "READY\r\n"
    host ->  <len> raw bytes (no framing)
    cart -> "OK\r\n> "

If --run is given, after the OK the script also sends
    RUN_<SRAM|PSRAM> <addr>\r\n
which makes the cart disable IRQs and jump to <addr>.
"""

import argparse
import glob
import os
import serial
import sys
import time


def auto_port():
    """Pick the first USB-serial / CDC-ACM device.

    On the RISKYMSX2 setup, /dev/ttyACM0 is the WCH-Link debugger's
    UART bridge which forwards the cart's USART1 traffic.
    """
    candidates = sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*"))
    return candidates[0] if candidates else "/dev/ttyUSB0"


def wait_for_prompt(ser, prompt=b"> ", timeout=15.0):
    """Block until we see the prompt bytes or timeout.

    The cart takes a few seconds to boot on a fresh reset. Default 15 s.
    """
    deadline = time.monotonic() + timeout
    buf = bytearray()
    last_print = time.monotonic()
    while time.monotonic() < deadline:
        if ser.in_waiting:
            chunk = ser.read(ser.in_waiting)
            buf += chunk
            sys.stdout.write(chunk.decode("ascii", errors="replace"))
            sys.stdout.flush()
            if prompt in buf:
                return True
            if b"ERR" in buf:
                sys.stderr.write(f"\ndevice error: {buf!r}\n")
                return False
        else:
            time.sleep(0.02)
            if time.monotonic() - last_print > 3.0 and buf:
                sys.stderr.write(" ...still waiting for prompt...\n")
                last_print = time.monotonic()
    sys.stderr.write(f"\ntimeout waiting for {prompt!r}, got: {buf!r}\n")
    return False


def main():
    ap = argparse.ArgumentParser(description="Upload .bin to RISKYMSX2")
    ap.add_argument("port", nargs="?",
                    help="serial port (auto-detected if omitted)")
    ap.add_argument("file", help=".bin file to upload")
    ap.add_argument("--dest", choices=("sram", "psram"), default="sram",
                    help="destination: sram (cart mirror, 32K) or "
                         "psram (external bus). Default sram.")
    ap.add_argument("--addr", type=lambda x: int(x, 0), default=0,
                    help="destination offset (hex or dec), default 0")
    ap.add_argument("--baud", type=int, default=115200,
                    help="serial baud (default 115200)")
    ap.add_argument("--chunk", type=int, default=4096,
                    help="write chunk size (default 4096 bytes)")
    ap.add_argument("--reset-ms", type=int, default=100,
                    help="pulse MSX ~RESET for this many ms after upload "
                         "(default 100). 0 disables reset.")
    ap.add_argument("--no-reset", action="store_true",
                    help="alias for --reset-ms 0 (suppress the post-upload "
                         "MSX reset pulse)")
    ap.add_argument("--run", choices=("sram", "psram", "none"), default="none",
                    help="after upload, jump to <addr> via RUN_SRAM / RUN_PSRAM")
    ap.add_argument("--max-size", type=int, default=0,
                    help="0 = no limit (psram) / 32K (sram). Override to "
                         "truncate the file.")
    args = ap.parse_args()

    if args.port is None:
        args.port = auto_port()
        print(f"auto-selected serial port: {args.port}")

    if args.no_reset:
        args.reset_ms = 0

    # Read file.
    with open(args.file, "rb") as f:
        data = f.read()
    size = len(data)

    # Apply destination-specific caps unless overridden.
    cap = args.max_size or (32768 if args.dest == "sram" else 1024 * 1024)
    if size > cap:
        sys.stderr.write(f"file is {size} bytes; truncating to {cap}\n")
        data = data[:cap]
        size = len(data)

    if args.addr + size > cap:
        sys.stderr.write(
            f"addr 0x{args.addr:x} + size {size} exceeds {args.dest} "
            f"capacity ({cap})\n")
        sys.exit(1)

    cmd = "XLOAD_SRAM" if args.dest == "sram" else "XLOAD_PSRAM"
    print(f"uploading {size} bytes to {args.dest.upper()} @ 0x{args.addr:x} "
          f"via {args.port} @ {args.baud}")

    with serial.Serial(args.port, args.baud, timeout=2.0) as ser:
        ser.reset_input_buffer()

        # Optional: wait for prompt to confirm cart is alive.
        wait_for_prompt(ser, timeout=2.0)
        ser.reset_input_buffer()

        # Send the XLOAD header.
        header = f"{cmd} {args.addr:x} {size:x}\r\n".encode()
        ser.write(header)
        ser.flush()

        # Wait for READY handshake.
        deadline = time.monotonic() + 5.0
        buf = bytearray()
        while time.monotonic() < deadline:
            if ser.in_waiting:
                buf += ser.read(ser.in_waiting)
                if b"READY" in buf:
                    break
                if b"ERR" in buf:
                    sys.stderr.write(f"device rejected XLOAD: {buf!r}\n")
                    sys.exit(1)
        else:
            sys.stderr.write(f"timeout waiting for READY, got: {buf!r}\n")
            sys.exit(1)
        print("cart READY, streaming bytes...")

        # Stream payload.
        sent = 0
        t0 = time.monotonic()
        for off in range(0, size, args.chunk):
            n = ser.write(data[off:off + args.chunk])
            sent += n
            pct = 100 * sent // size
            if (off // args.chunk) % 4 == 0:
                print(f"  {pct:3d}% ({sent}/{size})")
        ser.flush()

        elapsed = time.monotonic() - t0
        rate = sent / elapsed if elapsed > 0 else 0
        print(f"sent {sent} bytes in {elapsed:.2f}s ({rate/1024:.1f} KiB/s)")

        # Wait for OK.
        deadline = time.monotonic() + 5.0
        buf = bytearray()
        while time.monotonic() < deadline:
            if ser.in_waiting:
                buf += ser.read(ser.in_waiting)
                if b"OK\r" in buf or b"\nOK" in buf:
                    break
                if b"ERR" in buf:
                    sys.stderr.write(f"device error after upload: {buf!r}\n")
                    sys.exit(1)
        else:
            sys.stderr.write(f"no OK after upload, got: {buf!r}\n")
            sys.exit(1)
        print("upload OK")

        # Optional: jump to user code.
        if args.run != "none":
            ser.reset_input_buffer()
            run_cmd = "RUN_SRAM" if args.run == "sram" else "RUN_PSRAM"
            ser.write(f"{run_cmd} {args.addr:x}\r\n".encode())
            ser.flush()
            time.sleep(0.3)
            # Drain. Note: if the user code doesn't return, we'll never
            # see "OK returned" - the script just exits.
            ser.reset_input_buffer()
            print(f"jumped to {args.run.upper()} @ 0x{args.addr:x}")
            # When --run is used the cart owns the CPU - we cannot send
            # RST from here. Note this in the log.
            if args.reset_ms > 0:
                print("(MSX reset skipped - cart is running user code)")
            return

        # Optional: pulse MSX ~RESET. Fires by default after every upload
        # (matches the original psram_upload.py behaviour).
        if args.reset_ms > 0:
            ser.reset_input_buffer()
            cmd = f"RST {args.reset_ms}\r\n".encode()
            ser.write(cmd)
            ser.flush()
            time.sleep(0.3)
            ser.reset_input_buffer()
            print(f"MSX reset pulsed for {args.reset_ms} ms")


if __name__ == "__main__":
    main()