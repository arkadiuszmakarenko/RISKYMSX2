#!/usr/bin/env python3
"""HP 1660C control for RISKYMSX2 PSRAM bus debugging.

Built on terriblefire's tflab-tools HP1660 driver
(https://github.com/terriblefire/tflab-tools) with a kernel-USBTMC
transport for the XyphroLabs GpibUsb adapter (/dev/usbtmc0) - no vendor
library needed.

Subcommands:
  idn                    identify the instrument
  labels [file.json]     DOWNLOAD the front-panel configuration:
                         machines, pods, labels (name/polarity/bit map),
                         thresholds, trigger - i.e. the wiring view
  setup                  apply the PSRAM-serve capture config (labels,
                         TTL thresholds, trigger on MARK, 10 ns samples,
                         waveform markers)
  arm                    single-shot arm
  wait [--timeout S]     wait for measurement complete
  latency                X/O waveform markers -> serve latency in ns
  capture [file.vcd]     download the acquisition to a GTKWave VCD
  errs                   drain the analyzer's error queue
  cmd '<text>' / q '<text>'   raw access

Signal map used by `setup` (see docs/PSRAM_TIMING_DEBUG.md):
  pod1 bits0-15 = A0..A15, pod2 bits0-7 = D0..D7,
  pod3 bits0-7 = ~SLTSL,~RD,~WR,~MREQ,~M1,~IORQ,~WAIT,~RESET,
  pod3 bit8 = MARK (cart PA0/LEDFLASH test point; pulses HIGH during
  each PSRAM access in the CART_LA_MARKER diagnostic build).
"""

import argparse
import json
import os
import select
import sys
import time

try:
    from tflab.hp1660 import HP1660
except ImportError:
    sys.exit("tflab-tools not importable. Install with:\n"
             "  pip install --user --break-system-packages tflab-tools\n"
             "or clone https://github.com/terriblefire/tflab-tools and put "
             "its src/ on PYTHONPATH.")

DEV = "/dev/usbtmc0"
XYPHRO_VID, XYPHRO_PID = 0x03eb, 0x2065


class KernelUsbtmcTransport:
    """Kernel USBTMC (/dev/usbtmcN) transport.

    NOTE: does not work with the Xyphro firmware in practice - the
    adapter ACKs writes but never delivers bulk-IN reads. Kept only as
    a fallback (--transport kernel)."""

    def __init__(self, dev=DEV):
        try:
            self.fd = os.open(dev, os.O_RDWR)
        except PermissionError:
            sys.exit(f"permission denied on {dev} - run:\n"
                     f"  echo 'KERNEL==\"usbtmc[0-9]*\", MODE=\"0660\", GROUP=\"plugdev\"' | "
                     f"sudo tee /etc/udev/rules.d/99-usbtmc.rules\n"
                     f"  sudo udevadm control --reload && sudo udevadm trigger")
        except FileNotFoundError:
            sys.exit(f"{dev} not found - is the GPIB-USB adapter plugged in?")
        self.timeout = 5.0

    def write(self, data):
        os.write(self.fd, data)

    def read(self, n):
        r, _, _ = select.select([self.fd], [], [], self.timeout)
        if not r:
            raise TimeoutError("usbtmc read timeout")
        return os.read(self.fd, n)

    def settimeout(self, t):
        self.timeout = t

    def indicator_pulse(self):
        raise NotImplementedError("indicator pulse needs the raw transport")


class RawUsbtmcTransport:
    """python-usbtmc (libusb) transport for the Xyphro UsbGpib adapter.

    This is the path every VISA stack uses with this adapter (the
    kernel usbtmc driver does not complete reads against its firmware).
    tflab's HP1660 reads one byte at a time, so responses are buffered
    internally: each buffer refill is ONE full USBTMC message read.
    """

    def __init__(self, vid=XYPHRO_VID, pid=XYPHRO_PID):
        import usb.core
        dev = usb.core.find(idVendor=vid, idProduct=pid)
        if dev is None:
            sys.exit("Xyphro UsbGpib adapter not found on USB. If it is "
                     "plugged in, the GPIB side may be disconnected\n"
                     "(the adapter only enumerates when the instrument "
                     "answers its auto-ID).")
        self.usbdev = dev
        try:
            if dev.is_kernel_driver_active(0):
                dev.detach_kernel_driver(0)
        except NotImplementedError:
            pass  # backend without driver management
        import usbtmc
        try:
            self.inst = usbtmc.Instrument(vid, pid)
        except Exception as e:
            if "Permission" in str(e) or "denied" in str(e) or "Access" in str(e):
                sys.exit("No write access to the adapter's USB device node.\n"
                          "Unplug and replug the adapter's USB cable - the "
                          "login session's ACL is applied at enumeration\n"
                          "(the kernel usbtmc rule alone is not enough for "
                          "the libusb path).\n"
                          "If that does not help, add:\n"
                          "  echo 'SUBSYSTEM==\"usb\", ATTRS{idVendor}==\"03eb\", "
                          "ATTRS{idProduct}==\"2065\", MODE=\"0666\"' | sudo tee "
                          "/etc/udev/rules.d/99-xyphro.rules\n"
                          "  sudo udevadm control --reload && sudo udevadm trigger")
            raise
        self.inst.timeout = 5.0
        self._buf = b""

    def write(self, data):
        if isinstance(data, str):
            data = data.encode()
        self.inst.write_raw(data)

    def read(self, n):
        while len(self._buf) < n:
            chunk = self.inst.read_raw(1 << 16)
            if not chunk:
                break
            self._buf += bytes(chunk)
        out, self._buf = self._buf[:n], self._buf[n:]
        return out

    def settimeout(self, t):
        self.inst.timeout = t

    def indicator_pulse(self):
        """USBTMC INDICATOR_PULSE request - the Xyphro firmware treats it
        as 'interpret the next message as an adapter command' (!term,
        !addr, !ver?, !reset, ...). Must go through the handle that
        claimed the interface (python-usbtmc's), not a second open."""
        dev = getattr(self.inst, "device", None) or self.usbdev
        return dev.ctrl_transfer(0xA1, 64, 0, 0, 1)


# --- capture configuration -------------------------------------------------
# tflab label spec: pod_masks are one 16-bit mask PER POD, in DECREASING
# pod order ([podN, ..., pod1]). clock_bits=0 for timing.

CAPTURE_LABELS = {
    # name: (polarity, clock_bits, [pod4, pod3, pod2, pod1] masks)
    "ADDR": ("POSITIVE", 0, [0,     0,     0, 65535]),  # pod1  A0..A15
    "DATA": ("POSITIVE", 0, [0,     0,   255,     0]),  # pod2  D0..D7
    "CTRL": ("POSITIVE", 0, [0,   255,     0,     0]),  # pod3  bits 0..7
    "MARK": ("POSITIVE", 0, [0,   256,     0,     0]),  # pod3  bit 8 (PA0)
}
CTRL_BITS = {"SLTSL": 0, "RD": 1, "WR": 2, "MREQ": 3,
             "M1": 4, "IORQ": 5, "WAIT": 6, "RESET": 7}

SAMPLE_PERIOD = "1E-8"   # 10 ns; try 2E-9/4E-9 in half-pod timing mode

# Waveform markers for the automated latency measurement:
#   X = ~SLTSL falling (bus cycle start), searched BACKWARD from trigger
#   O = MARK falling (byte driven on the bus), FORWARD from trigger
# XOTIME? then reports the serve latency.
LATENCY_COMMANDS = [
    ":MACH1:TWAVEFORM:MMODE TIME",
    ":MACH1:TWAVEFORM:XPATTERN 'CTRL', '#BXXXXXXXX0'",
    ":MACH1:TWAVEFORM:XCONDITION ENTERING",
    ":MACH1:TWAVEFORM:XSEARCH -1, TRIGGER",
    ":MACH1:TWAVEFORM:OPATTERN 'MARK', '#B0'",
    ":MACH1:TWAVEFORM:OCONDITION ENTERING",
    ":MACH1:TWAVEFORM:OSEARCH +1, TRIGGER",
]


def open_la(dev=DEV, transport="raw"):
    t = RawUsbtmcTransport() if transport == "raw" else KernelUsbtmcTransport(dev)
    return HP1660(t), t


def cmd_adapter(la, t, text):
    """Adapter-internal command (!ver?, !term lf, !addr N, !reset ...)."""
    t.indicator_pulse()
    if "?" in text:
        t.write(text + "\n")
        print(t.read(4096).decode(errors="replace"))
    else:
        t.write(text + "\n")
        print("sent.")


def cmd_idn(la, t):
    try:
        t.indicator_pulse()
        t.write("!ver?\n")
        print("adapter: " + t.read(4096).decode(errors="replace"))
    except Exception as e:
        print(f"adapter query unavailable: {e}")
    print("instrument: " + la.idn())


def cmd_labels(la, outfile=None):
    data = {"idn": la.idn(), "machines": []}
    for m in la.get_machines():
        fmt = la.get_format(m.machine)
        trig = la.get_trigger(m.machine)
        entry = {"machine": m.to_dict(),
                 "format": fmt.to_dict() if fmt else None,
                 "trigger": trig.to_dict() if trig else None}
        data["machines"].append(entry)
        print(f"machine {m.machine}: {m.type} '{m.name}' pods={m.pods}")
        if fmt:
            for l in fmt.labels:
                masks = ", ".join(f"pod{p}:0x{m:04X}"
                                  for p, m in zip(fmt.pods, l.pod_masks)
                                  if m and p in fmt.pods)
                print(f"  label '{l.name}' {l.polarity} width={l.width} "
                      f"[{masks}]")
            print(f"  thresholds: {fmt.thresholds}")
    if outfile:
        with open(outfile, "w") as f:
            json.dump(data, f, indent=2)
        print(f"saved to {outfile}")


def cmd_setup(la):
    # Machine 1: timing analyzer on pods 1-3, TTL thresholds
    la.cmd(":SELECT 1")
    la.cmd(":MACH1:NAME 'Z80'")
    la.cmd(":MACH1:TYPE TIMING")
    la.set_pods([1, 2, 3], machine=1)
    for pod in (1, 2, 3):
        la.set_threshold(pod, "TTL", machine=1)
    # Labels (per-pod masks, decreasing pod order)
    la.remove_all_labels(machine=1)
    for name, (pol, clk, masks) in CAPTURE_LABELS.items():
        la.set_label(name, pol, clk, masks, machine=1)
    # Trigger: sequence level 1 finds term A = MARK high
    # (a PSRAM mapper-RAM access is in flight). Window centered.
    la.cmd(":MACH1:TTRIGGER:SEQUENCE 1")
    la.cmd(":MACH1:TTRIGGER:CLEAR ALL")
    la.cmd(":MACH1:TTRIGGER:ACQUISITION AUTOMATIC")
    la.set_sample_period(SAMPLE_PERIOD, machine=1)
    la.set_term("A", "MARK", "#B1", machine=1)
    la.set_find(1, "A", "OCCURRENCE,1", machine=1)
    la.set_trigger_position("CENTER", machine=1)
    # Waveform display
    la.cmd(":MACH1:TWAVEFORM:REMOVE")
    for lbl in ("MARK", "CTRL", "DATA", "ADDR"):
        la.cmd(f":MACH1:TWAVEFORM:INSERT '{lbl}', ALL")
    la.cmd(":MACH1:TWAVEFORM:RANGE 2E-6")
    la.cmd(":MENU 1,5")
    errs = la.errors()
    if errs:
        print("analyzer reported errors:")
        for code, msg in errs:
            print(f"  {code}: {msg}")
    else:
        print("capture configuration applied cleanly (no errors).")
    cmd_labels(la)


def cmd_arm(la):
    la.cmd(":RMODE SINGLE")
    la.run()
    print("armed (single). Trigger: MARK high = PSRAM access in flight.")
    print("Reset/power-cycle the MSX, or let the mapper traffic run.")


def cmd_wait(la, timeout):
    ok = la.wait(timeout)
    print("measurement complete." if ok else
          f"timeout after {timeout}s (still armed? trigger not seen?)")


def cmd_latency(la):
    for c in LATENCY_COMMANDS:
        la.cmd(c)
        time.sleep(0.05)
    xo = la.query(":MACH1:TWAVEFORM:XOTIME?")
    print(f"X->O time (~SLTSL fall -> byte on bus): {xo}")
    try:
        val = float(str(xo).split()[-1].rstrip("Ss"))
        ns = val * 1e9
        verdict = ("LATE - past a ~600 ns Z80 sampling budget"
                   if ns > 600 else "within a ~600 ns budget")
        print(f"  = {ns:.0f} ns serve latency ({verdict})")
    except ValueError:
        print("  (read the value from the waveform menu if the parse failed)")
    print(f"O marker absolute: {la.query(':MACH1:TWAVEFORM:OTIME?')}")
    print(f"X marker absolute: {la.query(':MACH1:TWAVEFORM:XTIME?')}")


def cmd_capture(la, vcdfile):
    print("downloading acquisition...")
    acq = la.acquire(machine=1)
    acq.to_vcd(vcdfile)
    ps = acq.sample_period_ps
    print(f"saved {vcdfile}: {len(acq.rows)} rows, "
          f"sample period {ps/1000 if ps else '?'} ns, "
          f"trigger row {acq.trigger_row}")
    print("view with: gtkwave " + vcdfile)


def cmd_errs(la):
    errs = la.errors()
    if errs:
        for code, msg in errs:
            print(f"{code}: {msg}")
    else:
        print("error queue empty")


def cmd_learn(la, path):
    la.cmd(":SYSTEM:SETUP?")
    blob = la.read_block()
    with open(path, "wb") as f:
        f.write(blob)
    print(f"saved {len(blob)} bytes of configuration to {path}")


def cmd_load(la, path):
    with open(path, "rb") as f:
        blob = f.read()
    la.cmd(":SYSTEM:SETUP #" + str(len(blob)).zfill(8))
    la._t.write(blob)
    print(f"replayed {len(blob)} bytes of configuration from {path}")


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--dev", default=DEV,
                   help="kernel usbtmc node (only with --transport kernel)")
    p.add_argument("--transport", choices=("raw", "kernel"), default="raw",
                   help="raw = libusb/python-usbtmc (default, works with the "
                        "Xyphro firmware); kernel = /dev/usbtmcN (broken with "
                        "this adapter, kept for reference)")
    sub = p.add_subparsers(dest="cmd", required=True)
    sub.add_parser("idn")
    lb = sub.add_parser("labels"); lb.add_argument("outfile", nargs="?")
    sub.add_parser("setup")
    sub.add_parser("arm")
    w = sub.add_parser("wait"); w.add_argument("--timeout", type=float, default=30.0)
    sub.add_parser("latency")
    cp = sub.add_parser("capture"); cp.add_argument("vcdfile", nargs="?", default="capture.vcd")
    sub.add_parser("errs")
    ad = sub.add_parser("adapter"); ad.add_argument("text", help="!ver?, !term lf, !addr 7, !reset, ...")
    ln = sub.add_parser("learn"); ln.add_argument("file", help="save full analyzer config")
    ld = sub.add_parser("load"); ld.add_argument("file", help="replay saved analyzer config")
    c = sub.add_parser("cmd"); c.add_argument("text")
    q = sub.add_parser("q"); q.add_argument("text")
    a = p.parse_args()

    la, t = open_la(a.dev, a.transport)
    try:
        if a.cmd == "idn":       cmd_idn(la, t)
        elif a.cmd == "labels":  cmd_labels(la, a.outfile)
        elif a.cmd == "setup":   cmd_setup(la)
        elif a.cmd == "arm":     cmd_arm(la)
        elif a.cmd == "wait":    cmd_wait(la, a.timeout)
        elif a.cmd == "latency": cmd_latency(la)
        elif a.cmd == "capture": cmd_capture(la, a.vcdfile)
        elif a.cmd == "errs":    cmd_errs(la)
        elif a.cmd == "adapter": cmd_adapter(la, t, a.text)
        elif a.cmd == "learn":   cmd_learn(la, a.file)
        elif a.cmd == "load":    cmd_load(la, a.file)
        elif a.cmd == "cmd":     la.cmd(a.text)
        elif a.cmd == "q":       print(la.query(a.text))
    finally:
        pass


if __name__ == "__main__":
    main()
