# PSRAM bus timing & crash troubleshooting (HP 1660C)

Symptom: intermittent crashes once the MSX/Nextor kernel uses the
sub-slot-1 memory mapper (PSRAM-backed RAM), suspected cause: the
mapper-RAM byte is driven onto the cart data bus **after** the Z80's
data sampling point, so the Z80 latches a wrong byte. When the kernel
executes or RAM-tests from mapper RAM, one late byte = one wrong
opcode/value = crash.

Two independent measurements answer it:

1. **Firmware statistics** (no LA needed) - every mapper-RAM serve timed
   from bus-cycle entry to byte-on-bus, "late" counter vs a threshold.
2. **Logic analyser** (HP 1660C via XyphroLabs GpibUsb, `tools/hp1660.py`)
   - direct oscilloscope-grade view of a serve inside a Z80 cycle, with
   a PA0 marker pulse around each PSRAM access.

---

## 1. Firmware statistics build

    cd firmware2/RISKYMSX2
    make rebuild QUIET=0 DEBUG=1 CART_TIMING_STATS=1 CART_LA_MARKER=1 flash

Serial console: USART1 TX (PA9) @ 921600 8N1, e.g.
`picocom -b 921600 /dev/ttyACM0` (note: the WCH-Link shares that USB
port with flashing - unplug/replug or use a separate USB-UART).

Boot log prints milestones, then once per ~2 s:

    MAPPER: reads=12345 late=0 cyc=46..63 (230..315 ns); writes=... ...

Interpretation:

| observation | meaning |
|---|---|
| `late=0`, max < ~120 cyc (600 ns) | serves fit the budget; crashes are elsewhere |
| `late>0`, spikes correlate with crashes | confirmed: PSRAM serves late |
| `cyc max` jumps (e.g. 46..300) | interrupt-entry jitter (USB IRQ? cache miss) lengthens serves |

The threshold `CART_LATE_CYCLES` (cart.c, default 120 = 600 ns) is a
rough budget: Z80 @3.58 MHz read cycle from ~SLTSL fall to the data
sampling point. Calibrate it against the LA measurement below.

## 2. Logic analyser capture

### One-time: GPIB adapter permission

    echo 'KERNEL=="usbtmc[0-9]*", MODE="0660", GROUP="plugdev"' | sudo tee /etc/udev/rules.d/99-usbtmc.rules
    sudo udevadm control --reload && sudo udevadm trigger

(quick alternative: `sudo chmod 666 /dev/usbtmc0`)

### Probe wiring (cart edge + one board test point)

| LA pod | bits | signal |
|---|---|---|
| pod 1 | 0..15 | A0..A15 |
| pod 2 | 0..7  | D0..D7 |
| pod 3 | 0     | ~SLTSL |
| pod 3 | 1     | ~RD |
| pod 3 | 2     | ~WR |
| pod 3 | 3     | ~MREQ |
| pod 3 | 4     | ~M1 |
| pod 3 | 5     | ~IORQ |
| pod 3 | 6     | ~WAIT |
| pod 3 | 7     | ~RESET |
| pod 3 | 8     | **MARK = cart PA0 (LEDFLASH test point)** |

Ground the pod clips to the cart GND. Requires the `CART_LA_MARKER=1`
build: PA0 goes HIGH while a PSRAM access is in flight, LOW when the
byte is on the bus.

### Capture procedure

    python3 tools/hp1660.py idn        # sanity: talks to the 1660C
    python3 tools/hp1660.py setup      # apply the capture config
    python3 tools/hp1660.py arm        # single-shot, trigger on MARK high
    #   -> reset the MSX, let Nextor boot and use the mapper RAM
    python3 tools/hp1660.py wait
    python3 tools/hp1660.py latency

`latency` places waveform markers X = ~SLTSL fall (searched backwards
from the trigger) and O = MARK fall (byte driven) and reports the
X->O time. On the waveform menu you can additionally eyeball:

* ~SLTSL fall -> MARK rise: interrupt entry + dispatch latency
* MARK high time: the PSRAM HB-bus read itself
* MARK fall vs ~RD rise / end of cycle: the margin to the Z80's data
  sampling point - if MARK falls after the Z80 samples, that byte is
  lost, every time it happens

Trigger alternatives (front panel or `hp1660.py cmd`):

* trigger on `CTRL #BXXXXXXXX0` (~SLTSL low) to capture *every* cart
  cycle and compare ROM-served vs PSRAM-served cycles (MARK only pulses
  on PSRAM ones)
* time-qualified trigger (TTRigger duration terms, 8 ns..5 s) on
  ~SLTSL low longer than ~700 ns to catch any *stretched* cycle

### Reading the result

* MARK fall consistently before ~RD rise with margin -> serves are fine;
  look elsewhere (PSRAM data integrity: try `PSRAM_BAND_CEILING_MHZ`
  300 in psram.c; or bank-register aliasing).
* MARK fall near/after the sampling point, or firmware `late>0` ->
  serves are late. Fix options, in order of preference:
  1. **Stretch mapper-RAM cycles with ~WAIT**: assert ~WAIT at handler
     entry for sub-slot-1 reads and release after the byte is driven
     (the `Cart_HoldMSXWait_*` machinery, PE3, safe by MSX spec). Costs
     Z80 throughput only on PSRAM accesses.
  2. Shrink interrupt-entry latency for the mapper path (VTF handler in
     `.ramfunc`, or re-check EXTI priorities vs USB).
  3. Loosen PSRAM HB timing (band/LATENCY) if the HB read itself is the
     dominant term.

## Files

* `tools/hp1660.py` - 1660C control (idn/setup/arm/wait/latency/dump/
  learn/load; `--dev` selects the usbtmc device)
* `firmware2/RISKYMSX2/User/cart.c` - `CART_TIMING_STATS`,
  `CART_LA_MARKER` instrumentation
* HP 1660 Programmer's Guide 01660-97024 - command reference
