#!/usr/bin/env python3
"""
load.py - upload a .bin / .rom file into the RISKYMSX2 PSRAM cart image
window via the CLI XLOAD command, then optionally select a mapper.

The cart image lives entirely in PSRAM (8 MiB at 0x80000000). The mapper
that interprets Z80 reads is selected separately via `MAP <name>`:

  ROM16k       16 KiB image mirrored at 0x4000 and 0x8000
  ROM32k       32 KiB image at 0x4000..0xBFFF  (default)
  ROM48k       48 KiB image at 0x4000..0xFFFF
  KONAMI       Konami mapper, banks switch only at 0x6000/0x8000/0xA000
  KONAMINOSCC  Konami mapper, any write to 0x4000..0xBFFF selects a bank
  KONAMISCC    Konami-with-SCC mapper (KONAMINOSCC banks + SCC sound
               chip: 0x9800..0x98FF is the SCC-I register window)
  ASCII8k      ASCII 8k mapper
  ASCII16k     ASCII 16k mapper
  NEO8         NEO 8 mapper
  NEO16        NEO 16 mapper
  NONE         bus floats (no mapper; Z80 reads 0xFF)

Usage:
    # Upload a 32 KiB ROM and boot with the default mapper (ROM32k):
    python3 load.py /dev/ttyACM0 game.rom

    # Upload a Konami-mapped ROM:
    python3 load.py /dev/ttyACM0 nemesis.rom --map KONAMI

    # Upload at a non-zero offset within the PSRAM window:
    python3 load.py /dev/ttyACM0 manbow.rom --map ASCII16k --addr 0x10000

    # Port auto-detected if omitted:
    python3 load.py nemesis.rom --map KONAMI

Protocol (USART1, 115200 8N1):
    host ->  "MAP <name>\r\n"
    cart -> "OK MAP <name> @ <hex>\r\n> "      (or ERR if PSRAM not ready)
    host ->  "XLOAD <addr> <len>\r\n"
    cart -> "READY\r\n"
    host ->  <len> raw bytes (no framing)
    cart -> "OK\r\n> " (then back to prompt mode)
    host ->  "RST <ms>\r\n"   (only if --reset-ms > 0)

After this command runs, the MSX is in reset for <ms> ms and then boots
the newly-uploaded cart image under the requested mapper.
"""

import argparse
import os
import re
import serial
import sys
import time

# Mapper names accepted by the cart's MAP command. Kept in sync with
# firmware/User/cart.h:Cart_MapperNames[].
MAPPERS = ("NONE", "ROM16k", "ROM32k", "ROM48k", "KONAMI", "KONAMINOSCC",
           "ASCII8k", "ASCII16k", "NEO8", "NEO16", "KONAMISCC")

PROMPT = b"\r\n> "

def check_port_exclusive(path):
    """Refuse to run if another process has the port open.

    The WCH-Link UART bridge is a single-channel device - if `screen`,
    `minicom`, a second copy of this script, or anything else has the
    same /dev/ttyACM* open, BOTH readers fight for every byte and the
    protocol fails in bizarre ways (mid-line truncation, missing
    prompts, phantom ERRs). This function exits with a clear message
    pointing the user at the right remedy.

    We use `lsof` because it works on all Linux distros and reports
    which other process holds the device. On failure we fall back to
    `fuser -v` output before giving up. """
    import subprocess
    try:
        out = subprocess.run(
            ["lsof", "--", path],
            capture_output=True, text=True, timeout=2.0,
        )
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return

    my_pid = os.getpid()
    holders = []
    for line in out.stdout.splitlines()[1:]:
        parts = line.split()
        if len(parts) >= 2:
            try:
                pid = int(parts[1])
            except ValueError:
                continue
            if pid == my_pid:
                continue
            holders.append(parts[0] + " (pid " + str(pid) + ")")

    if holders:
        sys.stderr.write(
            f"\n*** another process has {path} open:\n"
            f"    {', '.join(holders)}\n"
            f"*** this will garble every byte the cart sends. Detach it\n"
            f"*** (Ctrl-A :detach in screen, or 'kill <pid>') and retry.\n\n"
        )
        sys.exit(2)

def _line_starts_with(buf, token):
    """True if `buf` contains a line whose first field is `token`.

    The device emits two kinds of line boundaries in responses:
      1. "\\r\\n" (between response lines and at end of each line)
      2. "> "   (the standing CLI prompt, before the response on the
                 same line - the response lives after the prompt)
    A token must appear at the start of a NEW line, not as a substring
    of an earlier field. The search window is bounded to the last 512
    bytes to keep each poll cheap. """
    data = bytes(buf)
    if len(data) > 512:
        data = data[-512:]
    if data.startswith(token):
        return True
    needle1 = b"\r\n" + token
    needle2 = b"> " + token
    return (needle1 in data) or (needle2 in data)

def read_until(ser, token, timeout=10.0, echo=False):
    """Read until `token` appears at the start of a new line, or until
    ERR appears, or timeout. Callers must ALSO drain through the
    trailing prompt (see wait_prompt_after) before sending the next
    command - the prompt is the device's dispatcher-idle signal.
    """
    deadline = time.monotonic() + timeout
    buf = bytearray()
    while time.monotonic() < deadline:
        if ser.in_waiting:
            chunk = ser.read(ser.in_waiting)
            buf += chunk
            if echo:
                sys.stdout.write(chunk.decode("ascii", errors="replace"))
                sys.stdout.flush()
            if _line_starts_with(buf, token) or _line_starts_with(buf, b"ERR"):
                return buf
        else:
            time.sleep(0.02)
    return buf

def wait_prompt_after(ser, buf, timeout=10.0, echo=True):
    """Ensure the trailing "\\r\\n> " prompt arrived after a response.

    The WCH-Link USB-CDC bridge holds short responses (< USB-FS packet
    boundary) in its TX FIFO and only flushes them when the OUT
    endpoint sees new host->device traffic. If the response is just
    "OK MAP ...\\r\\n> " (a few dozen bytes), the trailing "> " may sit
    in the bridge until we send *something*. Probe with a tiny PING
    after 200 ms of no data - the PING (a) is answered with "OK\\r\\n> "
    which the device buffers behind our still-pending prompt, but its
    outgoing bytes wake the bridge's OUT endpoint and force the flush. """
    if PROMPT in bytes(buf)[-32:]:
        return True
    deadline = time.monotonic() + timeout
    primed = False
    while time.monotonic() < deadline:
        if ser.in_waiting:
            chunk = ser.read(ser.in_waiting)
            buf += chunk
            if echo:
                sys.stdout.write(chunk.decode("ascii", errors="replace"))
                sys.stdout.flush()
            if PROMPT in bytes(buf)[-32:]:
                return True
        else:
            if not primed:
                poll_elapsed = timeout - (deadline - time.monotonic())
                if poll_elapsed > 0.2:
                    ser.write(b"PING\r\n")
                    ser.flush()
                    primed = True
            time.sleep(0.02)
    return False

def send_cmd(ser, cmd, expect, timeout=10.0, echo=True):
    """Send one line command, wait for `expect` at line start, then drain
    through the trailing prompt. Calls sys.exit on ERR/timeout. """
    ser.write(cmd.encode() + b"\r\n")
    ser.flush()
    buf = read_until(ser, expect.encode(), timeout=timeout, echo=echo)
    if not _line_starts_with(buf, expect.encode()):
        sys.stderr.write(f"\ntimeout waiting for {expect!r} after {cmd!r}: {buf!r}\n")
        sys.exit(1)
    if _line_starts_with(buf, b"ERR"):
        sys.stderr.write(f"\ndevice error after {cmd!r}: {buf!r}\n")
        sys.exit(1)
    if not wait_prompt_after(ser, buf, timeout=timeout, echo=echo):
        sys.stderr.write(f"\ntimeout waiting for prompt after {cmd!r}\n")
        sys.exit(1)
    return buf

def verify_upload(ser, data, addr, timeout=10.0):
    """Read back the uploaded image through DUMP and compare it byte-for-byte."""
    dump_re = re.compile(
        rb"(?:^|\r\n|> )DUMP ([0-9a-fA-F]+) ([0-9a-fA-F]+):((?: [0-9a-fA-F]{2})*)"
    )
    chunk_size = 256  # DUMP caps responses at 256 bytes in the firmware.
    for offset in range(0, len(data), chunk_size):
        expected = data[offset:offset + chunk_size]
        command_addr = addr + offset
        buf = send_cmd(
            ser, f"DUMP {command_addr:x} {len(expected):x}", "DUMP",
            timeout=timeout, echo=False,
        )
        match = dump_re.search(bytes(buf))
        if match is None:
            sys.stderr.write(
                f"malformed DUMP response at 0x{command_addr:x}: {buf!r}\n"
            )
            sys.exit(1)
        response_addr = int(match.group(1), 16)
        response_len = int(match.group(2), 16)
        actual = bytes.fromhex(match.group(3).decode("ascii"))
        if response_addr != command_addr or response_len != len(expected):
            sys.stderr.write(
                f"DUMP header mismatch: requested 0x{command_addr:x} "
                f"{len(expected)} bytes, got 0x{response_addr:x} "
                f"{response_len} bytes\n"
            )
            sys.exit(1)
        if actual != expected:
            mismatch = next(i for i, (got, want) in enumerate(zip(actual, expected))
                             if got != want)
            sys.stderr.write(
                f"verification failed at 0x{command_addr + mismatch:x}: "
                f"got 0x{actual[mismatch]:02x}, expected 0x{expected[mismatch]:02x}\n"
            )
            sys.exit(1)
        print(f"verified {offset + len(expected)}/{len(data)} bytes")

    print("upload verification OK")

def wait_for_prompt(ser, timeout=15.0):
    """Block until we see the prompt bytes or timeout.

    The cart takes a few seconds to boot on a fresh reset (banner print
    + CLI banner), and the WCH-Link UART bridge sometimes comes up a
    beat later. Default 15 s.

    The boot banner ends with '> ' (no leading CRLF) so we accept EITHER
    the boot prompt OR a trailing post-response prompt. Once the device
    is sitting idle, every subsequent prompt is '\r\n> '. """
    deadline = time.monotonic() + timeout
    buf = bytearray()
    last_print = time.monotonic()
    while time.monotonic() < deadline:
        if ser.in_waiting:
            chunk = ser.read(ser.in_waiting)
            buf += chunk
            sys.stdout.write(chunk.decode("ascii", errors="replace"))
            sys.stdout.flush()
            tail = bytes(buf)[-32:]
            if b"> " in tail:
                return True
        else:
            time.sleep(0.02)
            if time.monotonic() - last_print > 3.0 and buf:
                sys.stderr.write(" ...still waiting for prompt...\n")
                last_print = time.monotonic()
    sys.stderr.write(f"\ntimeout waiting for prompt, got: {buf!r}\n")
    return False

def auto_port():
    """Pick the first USB-serial / CDC-ACM device."""
    import glob
    candidates = sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*"))
    if candidates:
        return candidates[0]
    return "/dev/ttyUSB0"

def main():
    ap = argparse.ArgumentParser(description="Upload .rom/.bin to RISKYMSX2 PSRAM cart window")
    ap.add_argument("port", nargs="?",
                    help="serial port (auto-detected if omitted)")
    ap.add_argument("file", help=".bin/.rom file to upload")
    ap.add_argument("--map", choices=MAPPERS, default="ROM32k",
                    help="cart mapper to select before upload (default ROM32k). "
                         "KONAMI accepts bank writes at 0x6000/0x8000/0xA000 only; "
                         "KONAMINOSCC accepts bank writes anywhere in 0x4000..0xBFFF; "
                         "KONAMISCC = KONAMINOSCC banks + the SCC sound chip "
                         "(writes to 0x9800..0x98FF go to the SCC emulator). "
                         "Use 'NONE' to upload without activating any mapper.")
    ap.add_argument("--addr", type=lambda x: int(x, 0), default=0,
                    help="window offset (hex or dec), default 0. Konami banks "
                         "start at 0x4000, ASCII8k at 0x4000, ASCII16k at "
                         "0x4000 (page 1) / 0x8000 (page 2), NEO at 0x4000.")
    ap.add_argument("--no-map", action="store_true",
                    help="skip MAP command (upload only, keep current mapper)")
    ap.add_argument("--keep", action="store_true",
                    help="do NOT pulse MSX reset after upload")
    ap.add_argument("--verify", action="store_true",
                    help="read back and verify the uploaded game before reset")
    ap.add_argument("--reset-ms", type=int, default=100,
                    help="pulse MSX ~RESET for this many ms after upload "
                         "(default 100). Ignored if --keep is set.")
    ap.add_argument("--baud", type=int, default=115200,
                    help="serial baud (default 115200)")
    ap.add_argument("--chunk", type=int, default=4096,
                    help="write chunk size (default 4096 bytes)")
    args = ap.parse_args()
    if args.port is None:
        args.port = auto_port()
        print(f"auto-selected serial port: {args.port}")

    with open(args.file, "rb") as f:
        data = f.read()

    size = len(data)
    psram_window = 8 * 1024 * 1024  # 8 MiB
    if args.addr >= psram_window or args.addr + size > psram_window:
        sys.stderr.write(
            f"file range 0x{args.addr:x}..0x{args.addr + size:x} "
            f"exceeds PSRAM window ({psram_window} bytes)\n"
        )
        sys.exit(1)
    print(f"uploading {size} bytes to PSRAM @ 0x{args.addr:x} via {args.port} @ {args.baud}")

    with serial.Serial(args.port, args.baud, timeout=2.0) as ser:
        ser.reset_input_buffer()

        # Refuse to run if another process has the port open.
        check_port_exclusive(args.port)

        # Optional: wait for the prompt.
        wait_for_prompt(ser, timeout=2.0)
        ser.reset_input_buffer()

        # 1) Select the mapper (unless --no-map). The cart's MAP command
        #    swaps the EXTI0 handler in the PFIC VTF slot atomically and
        #    replies "OK MAP <name> @ <hex>". Any non-NONE mapper
        #    requires PSRAM_Init() to have passed at boot; the cart
        #    replies ERR otherwise - send_cmd exits on ERR.
        if not args.no_map:
            map_reply = send_cmd(ser, f"MAP {args.map}", "OK")
            print(f"cart mapper: {map_reply.decode('ascii', 'replace').strip()}")

        # 2) Send the XLOAD header and wait for READY.
        ser.write(f"XLOAD {args.addr:x} {size:x}\r\n".encode())
        ser.flush()
        buf = read_until(ser, b"READY", timeout=5.0, echo=False)
        if not _line_starts_with(buf, b"READY"):
            sys.stderr.write(f"timeout waiting for READY, got: {buf!r}\n")
            sys.exit(1)
        print("cart READY, streaming bytes...")

        # 3) Stream the binary data.
        sent = 0
        t0 = time.monotonic()
        for off in range(0, size, args.chunk):
            n = ser.write(data[off:off + args.chunk])
            sent += n
            if (off // args.chunk) % 4 == 0:
                pct = 100 * sent // size
                print(f"  {pct:3d}% ({sent}/{size})")
        ser.flush()

        elapsed = time.monotonic() - t0
        rate = sent / elapsed if elapsed > 0 else 0
        print(f"sent {sent} bytes in {elapsed:.2f}s ({rate/1024:.1f} KiB/s)")

        # 4) Wait for OK. Send a PING if no bytes arrive within 200 ms -
        #    the WCH-Link USB-CDC bridge holds short responses in its TX
        #    FIFO and only flushes on OUT traffic.
        deadline = time.monotonic() + 10.0
        buf = bytearray()
        primed = False
        while time.monotonic() < deadline:
            if ser.in_waiting:
                buf += ser.read(ser.in_waiting)
                if _line_starts_with(buf, b"OK") and not _line_starts_with(buf, b"ERR"):
                    tail = bytes(buf)[-32:]
                    if (b"> " in tail) or (PROMPT in tail):
                        break
            else:
                if not primed and len(buf) == 0:
                    poll_elapsed = 10.0 - (deadline - time.monotonic())
                    if poll_elapsed > 0.2:
                        ser.write(b"PING\r\n")
                        ser.flush()
                        primed = True
                time.sleep(0.02)
        if not _line_starts_with(buf, b"OK") or _line_starts_with(buf, b"ERR"):
            sys.stderr.write(f"no OK after upload, got: {buf!r}\n")
            sys.exit(1)
        tail = bytes(buf)[-32:]
        if b"> " not in tail and PROMPT not in tail:
            if not wait_prompt_after(ser, buf, timeout=5.0, echo=False):
                sys.stderr.write(
                    f"no prompt after upload OK - last 32 bytes: "
                    f"{bytes(buf)[-32:]!r}\n"
                )
                sys.exit(1)
        print("upload OK")

        # 5) Optionally read back the image before allowing the MSX to boot.
        if args.verify:
            verify_upload(ser, data, args.addr)

        # 6) Optionally pulse MSX reset so it re-reads the cart slot
        #    and boots the newly uploaded ROM.
        if not args.keep:
            send_cmd(ser, f"RST {args.reset_ms}", "OK", echo=False)
            print(f"MSX reset pulsed for {args.reset_ms} ms")

if __name__ == "__main__":
    main()