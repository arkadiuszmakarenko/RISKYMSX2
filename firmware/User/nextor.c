/*
 * nextor.c - Nextor kernel mapper support: the mailbox window.
 *
 * The ASCII16K bank decode and the ROM read path live in
 * cart.c::Cart_EXTI0_Nextor_Handler. This module owns everything
 * BEHIND the mailbox - the six registers at 0x7FF0..0x7FF5 that
 * MSXSoftware/NextorDriver/driver.asm uses to reach the disk.
 *
 *   1. Lifecycle
 *        Nextor_Init      - power-on mailbox state
 *        Nextor_Service   - main-loop hook, answers the request in flight
 *   2. Code entries for Cart_EXTI0_Nextor_Handler
 *        Nextor_ReadByte  - IRQ context, mailbox reads only
 *        Nextor_WriteByte - IRQ context, mailbox writes only
 *
 * The handler calls the two code entries ONLY for addresses inside
 * NEXTOR_MBOX_BASE..NEXTOR_MBOX_END; every other cart read is served
 * from nextor_rom[] by the handler. Register semantics (addresses,
 * status bits, command bytes) are in nextor.h.
 *
 * ---------------------------------------------------------------------------
 * Split of duties: what runs where, and why
 * ---------------------------------------------------------------------------
 * driver.asm's MB_SEND pushes a request as a BURST of cart cycles:
 *
 *     ld (MBOX_CMD),a      ; command byte FIRST ...
 *     ld a,(hl)
 *     ld (MBOX_DATA),a     ; ... then each argument byte
 *     djnz MB_SEND_L
 *
 * Every one of those cycles raises EXTI0, so the capture half
 * (Nextor_WriteByte) runs in IRQ context once per byte. For a WRITE
 * that is 517 interrupts back to back with the Z80 holding the cart bus
 * in between, so these entries do the least possible work: one bounds
 * test, one store, one pointer bump. No printf, no dispatch, no loop,
 * no disk.
 *
 * The other half runs in the main loop (Nextor_Service): decode the
 * request, answer it, and print exactly one debug line. The driver is
 * blocked in MB_POLL spinning on the DONE bit for the whole of this
 * function, which is what makes the multi-millisecond cost of a printf
 * invisible to it.
 *
 * Why the argument count is a table, not a wire field: MB_SEND pushes
 * the command byte before the arguments and never tells the firmware
 * how many are coming. The firmware therefore has to know, per command,
 * both how many argument bytes to expect and when the burst is over.
 * See s_cmd_args[]; it is transcribed from the command table in
 * driver.asm lines 25-33 and must move in lockstep with it.
 *
 * ---------------------------------------------------------------------------
 * Why printing is gated on a state change
 * ---------------------------------------------------------------------------
 * MB_POLL is a spin loop: it re-reads MBOX_STAT thousands of times per
 * request until DONE appears. If anything printed per poll - or per
 * Nextor_Service call - the UART would be flooded and the driver would
 * time out waiting behind the log. Two mechanisms prevent that:
 *
 *   armed   set once when a burst is fully captured, cleared as soon
 *           as Nextor_Service consumes it. One request -> one pass.
 *   seq     incremented once per captured request; Nextor_Service
 *           prints only when seq differs from the last one it printed.
 *
 * seq is the belt-and-braces half: it guarantees "one request is
 * printed once" even if a request were somehow still observable on a
 * later pass. It also detects loss - a jump of more than 1 between
 * prints means Nextor_Service fell behind and a request was overwritten
 * before it was answered, which the log reports as a gap.
 *
 * STATUS is stored precomputed (s_mb.status) rather than assembled on
 * read, because STATUS is by far the hottest register - MB_POLL reads
 * it and nothing else for the whole request - so the poll reduces to a
 * single byte load and the RX_AVAIL update is paid once per result
 * byte instead of once per poll.
 *
 * ---------------------------------------------------------------------------
 * The device table
 * ---------------------------------------------------------------------------
 * Two devices are exported to the kernel, and neither of them is the raw
 * USB stick. Each is one file on the stick, read and written one 512-byte
 * sector at a time through FatFs:
 *
 *   device 1   NEXTOR.DSK - a fixed, READ-ONLY 720 KiB MSX-DOS floppy
 *              image (1440 x 512, FAT12, no partition table, boot sector
 *              at LBA 0). The driver flags it as a floppy disk drive
 *              (device query 2, flag bit 2), which is what makes the
 *              kernel map a drive straight onto sector 0 instead of
 *              hunting for partitions it does not have. Backed by
 *              dsk_image.c.
 *
 *   device 2   NEXTOR.IMG - a READ-WRITE disk image whose length IS its
 *              capacity, containing an MBR partition table and a FAT16
 *              volume. Flagged removable and NOT flagged as a floppy,
 *              so the kernel runs its ordinary partition scan and finds
 *              the MBR. Backed by img_image.c, which can also create
 *              and delete the file (the terminal's "Nextor" menu does).
 *
 * Both are reached through ONE command set and ONE driver entry point.
 * Every device-scoped command carries the device number as its first
 * argument, so the device is a wire field rather than a constant. The
 * firmware side needs exactly one place that knows what a device IS -
 * the s_dev[] table at the bottom of this file - and adding a third
 * device is one row in it plus one `ld b,3` in driver.asm.
 *
 * Reads and writes go straight to the file - f_lseek + f_read /
 * f_write into the result FIFO / out of args[] - with the handle kept
 * open across commands, so a sector costs one seek and one 512-byte
 * transfer and no path parsing. A failed FatFS call drops the handle:
 * the stick may have been pulled mid-session, and the next command then
 * re-probes, which is also how the image comes back after a re-plug.
 *
 * The obvious next optimisation for device 1, and deliberately not done
 * here: cache the whole 737280-byte image in PSRAM at Nextor_Init and
 * serve reads from there. It is not done because the 1-2 s it takes to
 * stream the image off the stick would have to happen while the Z80 is
 * already polling, and reads out of the file are already faster than the
 * Z80 can drain the result FIFO. Device 2 cannot use that trick at all -
 * it is writable, so it has to go to the file anyway.
 *
 * Where the probing happens, and why it is split in two:
 *
 *   request path  CAPACITY and READ call the backend's IsPresent() and
 *                 will report "no media" if it is false, but never mount
 *                 the volume themselves. That can be slow while a stick
 *                 is still enumerating, and slower than the driver's
 *                 ~0.8 s command budget, so it is not done from a
 *                 command that is cheap to answer otherwise.
 *   main loop     img_probe() runs from Nextor_Service while no request
 *                 is in flight, so by the time the kernel asks its first
 *                 question the file is normally already open and the
 *                 answer is immediate.
 *   STATUS never probes at all - it reports what the firmware currently
 *                 knows, because the kernel polls status far more often
 *                 than it asks for real work.
 *
 * The probe covers BOTH devices on one 2 s rate limiter, not one
 * limiter each: the slow part (USB enumeration and f_mount) is shared, so
 * a stick that has just been plugged in does not get enumerated twice,
 * and a device that is merely absent does not hold the other one back.
 * Within the pass, a device that is already present is not re-opened -
 * a mounted volume and an open handle are stable state, and re-opening
 * the handle of a device the kernel is actively reading would only add a
 * path parse and a seek to every retry cycle.
 * ---------------------------------------------------------------------------
 */

#include "nextor.h"
#include "dsk_image.h"
#include "img_image.h"
#include "ch32v4x7_conf.h"
#include "ff.h"
#include <stdio.h>

/* ========================================================================
 * Wire contract
 * ====================================================================== */

/* driver.asm: "512-byte sectors, two devices." The result FIFO is
 * exactly one sector, so 512 is both the driver's sector size and the
 * largest answer a single command can produce. */
#define NEXTOR_SECTOR_SIZE   512U

/* Largest argument burst: WRITE = device(1) + LBA(4) + one sector. */
#define NEXTOR_ARG_MAX       (1U + 4U + NEXTOR_SECTOR_SIZE)    /* 517 */

/* Largest result burst: READ = one sector. */
#define NEXTOR_RES_MAX       NEXTOR_SECTOR_SIZE           /* 512 */

/* Mailbox writes remembered for the invalid-command report. 24 is about
 * one HANDSHAKE plus its poll, which is the shortest span that can hold
 * an unexplained CMD write together with whatever it interrupted. */
#define NEXTOR_TRACE_N       24U

/* Argument bytes each command pushes after its command byte.
 * Transcribed from driver.asm's command table. Indexed by command byte.
 *
 * The value doubles as the bound for the arg sink, which is what makes
 * an over-long burst impossible to overrun args[] with.
 *
 * Version 2 prepends the device byte to every device-scoped command;
 * HANDSHAKE and ABORT are not scoped to a device and stay at zero. A
 * mismatch here and in driver.asm is not a silent corruption: the
 * handshake carries NEXTOR_VERSION and the driver refuses to continue
 * unless the two agree, so a half-updated pair fails at boot rather
 * than answering for the wrong device. */
static const uint16_t s_cmd_args[NEXTOR_CMD_MAX] = {
    0U,                                   /* 0x00 HANDSHAKE */
    1U,                                   /* 0x01 CAPACITY: dev */
    1U,                                   /* 0x02 STATUS:   dev */
    (uint16_t)(1U + 4U),                  /* 0x03 READ:   dev + LBA */
    (uint16_t)(1U + 4U + NEXTOR_SECTOR_SIZE), /* 0x04 WRITE: dev + LBA + data */
    0U,                                   /* 0x05 ABORT     */
    1U,                                   /* 0x06 STAPEEK: dev */
};

static const char *const s_cmd_name[NEXTOR_CMD_MAX] = {
    "HANDSHAKE", "CAPACITY", "STATUS", "READ", "WRITE", "ABORT", "STAPEEK",
};

/* ========================================================================
 * Mailbox state
 *
 * The two bulk buffers are addressed by POINTER rather than index: the
 * IRQ entries then need no bounds arithmetic at all, just
 *
 *     if (p < end) { *p++ = v; if (p == end) { ... } }
 *
 * which is the smallest thing that can capture a byte. An index form
 * (`args[i++] = v`) is a store, an index reload for the completion
 * test, and a second store, and because both fields are volatile the
 * compiler re-reads the index from memory rather than keeping it in a
 * register. The end pointer also makes the overflow bound obvious at a
 * glance: the sink physically cannot pass s_cmd_args[].
 *
 * Fields split by who touches them:
 *   argp / argend / res_r / res_end / armed / seq / status   IRQ + Service
 *   err / last_seq                                           Service only
 * ====================================================================== */

static struct {
    /* --- request capture (IRQ writes, Service reads) --- */
    uint8_t * volatile argp;     /* next argument slot                */
    uint8_t * volatile argend;   /* one past the last slot            */
    volatile uint8_t  cmd;       /* latched command byte              */
    volatile uint8_t  armed;     /* 1 = burst complete, answer pending */
    volatile uint8_t  seq;       /* requests captured, wraps at 256   */

    /* --- result FIFO (Service writes, IRQ pops) --- */
    uint8_t * volatile res_r;    /* next byte the driver will pop     */
    uint8_t * volatile res_end;  /* one past the last produced byte   */

    /* --- STATUS register, precomputed (IRQ reads it every poll) --- */
    volatile uint8_t  status;

    /* --- answer / bookkeeping, Service only --- */
    uint8_t           err;       /* NEXTOR_ERR_*                      */
    uint8_t           last_seq;  /* seq of the last line we printed   */

    /* --- Z80 write trace (IRQ writes, Service prints) ---
     * Ring of the last NEXTOR_TRACE_N mailbox writes, in arrival order.
     * The only thing a write to 0x7FF0 naming no known command can be is
     * a bus fault, a mis-decode or a corrupted stack, and none of those
     * is distinguishable from the outside without seeing what the Z80
     * did either side of it. Three stores per write is cheap next to the
     * handler it lives in. */
    volatile uint8_t  tr_reg [NEXTOR_TRACE_N];
    volatile uint8_t  tr_val [NEXTOR_TRACE_N];
    volatile uint8_t  tr_pos;    /* next slot to write                */
    volatile uint8_t  tr_gen [NEXTOR_TRACE_N]; /* tick at that write   */
    volatile uint8_t  tr_tick;   /* mailbox writes seen, wraps at 256  */
    uint8_t           args[NEXTOR_ARG_MAX];
    uint8_t           res [NEXTOR_RES_MAX];
} s_mb;

/* Command byte that means "nothing latched". Also the count of real
 * commands, so an out-of-range byte parks the collector instead of
 * indexing s_cmd_args[] out of bounds. */
#define NEXTOR_CMD_NONE    NEXTOR_CMD_MAX

/* Short (8.3) names on purpose. The stick in use stores long file names
 * as VFAT entries and f_open() resolves them unreliably here; the
 * loader and terminal hit exactly this and both work off the SFN.
 *
 * NEXTOR.DSK is the stock Nextor tools disk from the Nextor releases: a
 * bare 737280-byte FAT12 image whose sector 0 is a boot sector. That is
 * the raw image, NOT a DSK container - no 0x100-byte per-track offset
 * header - which is why the file has to be exactly 1440 sectors and why
 * the driver flags the device as a floppy (a bare FAT image has no
 * partition table for the kernel to find).
 *
 * NEXTOR.IMG is device 2's image and is created by the terminal's
 * "Nextor" menu (F1 from the file list) rather than shipped, because it
 * has to be as large as the stick can spare. It is a plain file of
 * whole 512-byte sectors whose first sector is an MBR, which is what
 * makes it look like an IDE hard disk to the kernel.
 *
 * A short candidate list rather than one name, because "the file is
 * called something else" and "the image is not there" look identical
 * from the MSX side: no drive, no error, nothing in the log but one
 * f_open failure. Two names cost 18 bytes of flash and remove the most
 * likely reason for that. Order matters - the first hit wins. */
static const char *const s_dsk_names[] = {
    "0:/NEXTOR.DSK",
    "0:/RISKYMSX.DSK",
};

static const char *const s_img_names[] = {
    "0:/NEXTOR.IMG",
    "0:/RISKYMSX.IMG",
};

/* ========================================================================
 * 1b. Request timestamps.
 *
 * Why this exists: a driver query loop that the firmware cannot satisfy
 * never settles - the log fills with an endless CAPACITY/STATUS/STATUS
 * cycle. Whether that is the kernel's intended watchdog (once a second or
 * so) or a tight retry spin (which would peg a CPU and make the machine
 * unusable) is not something the log can show, because a list of lines
 * with no intervals cannot distinguish them. So each line carries the gap
 * since the previous one.
 *
 * The clock is rdcycle, not SysTick: Delay_Ms() borrows SysTick as a
 * one-shot on every call and leaves it stopped, so there is no free-running
 * millisecond tick to read. rdcycle is always running.
 *
 * rdcycle counts at the core clock while SystemCoreClock is HCLK, and this
 * part is configured SYSCLK_400MHz_HCLK_200MHz_HSI - so assuming the two
 * are equal would put every number here out by a factor of two. Rather
 * than guess, calibrate once against Delay_Ms, which is itself derived
 * from SystemCoreClock: that measures exactly the ratio that is in
 * doubt. A 4 ms window is long enough to swamp the call overhead and
 * short enough not to be worth measuring twice.
 * ======================================================================== */

#if defined(__riscv)
static uint32_t rdcycle (void) {
    uint32_t c;
    __asm__ volatile ("rdcycle %0" : "=r" (c));
    return c;
}
#else
/* Host build. The conformance test for this file compiles it natively, and
 * there is no rdcycle off-target, so fall back to the host's monotonic
 * clock. This only affects the UNIT of the log's intervals, never which
 * requests get a line - and the test asserts on the lines, not their
 * spacing, so the substitution is invisible to what it checks. */
#include <time.h>
static uint32_t rdcycle (void) {
    struct timespec ts;
    clock_gettime (CLOCK_MONOTONIC, &ts);
    return (uint32_t)((uint64_t)ts.tv_sec * 1000000000ULL
                      + (uint64_t)ts.tv_nsec);
}
#endif

extern void Delay_Ms (uint32_t n);

/* Cycles per millisecond, 0 until the calibration below has run. */
static uint32_t s_cyc_per_ms;

static void cyc_calibrate (void) {
    const uint32_t t0 = rdcycle();
    Delay_Ms (4U);
    /* Unsigned subtraction, so a wrap between the two reads is harmless. */
    s_cyc_per_ms = (rdcycle() - t0) / 4U;
    if (s_cyc_per_ms == 0U) {
        s_cyc_per_ms = 1U;              /* never divide by zero */
    }
}

/* Milliseconds since the previous call. The first call reports time since
 * the calibration, which is the time since boot of the log itself. */
static uint32_t ms_since_prev (void) {
    static uint32_t prev;
    static uint8_t  primed;
    const uint32_t  now = rdcycle();
    uint32_t        d;

    if (primed == 0U) {
        primed = 1U;
        prev   = now;
        return 0U;
    }
    d = (now - prev) / s_cyc_per_ms;
    prev = now;
    return d;
}

/* Milliseconds on the same scale ms_since_prev() reports in, as an
 * absolute reading rather than a delta. Used to rate-limit probing.
 * The multiplication is done in 64 bits because rdcycle() wraps every
 * ~10 s at 400 MHz and the calibration constant makes the division
 * widen the range rather than narrow it. */
static uint32_t ms_now (void) {
    return (uint32_t)(((uint64_t)rdcycle() * 1000ULL) / (uint64_t)s_cyc_per_ms);
}

/* ms_now() reading taken in Nextor_Init, used by img_probe's boot
 * delay. 0 until then (so a host-side unit test that never calls
 * Nextor_Init is not delayed). */
static uint32_t s_boot_start_ms;

/* ========================================================================
 * 1b2. The device table
 *
 * THE ONE PLACE that knows what a device IS.
 *
 * Everything above this table - the capture buffers, the result FIFO, the
 * status precompute, the command switch - is device-agnostic. The device
 * byte a command carries is used for exactly one thing: indexing
 * s_dev[]. Each row is a bag of function pointers with the same shape as
 * DskImage_* / ImgImage_*, so the command handlers never name a backend
 * and never branch on "is this the floppy or the disk".
 *
 * Why a table rather than `if (dev == 1) DskImage_ReadSector(...) else
 * ImgImage_ReadSector(...)`: the second and third devices then cost a row
 * here and a `ld b,3` in driver.asm, instead of a second copy of the
 * whole command switch. Two devices is already past the point where the
 * if-chain would have been shorter to write - the thunk row is longer
 * than the branch it replaces - but the third device is where the table
 * starts paying.
 *
 * Index 0 is deliberately unused and permanently zero: the wire device
 * numbers are 1-based (they are the same numbers the kernel uses), so
 * `s_dev[dev]` on a device byte of 0 lands on a row with no function
 * pointers in it rather than on device 1. That is a bug-catching
 * property, not an accident - dev_lookup() rejects it.
 *
 * writable is stored rather than derived from "write_sector != NULL"
 * because device 1 HAS a write entry point (returning failure) so that
 * the command handler does not need a second null check: the difference
 * between "no media" and "read-only" is two different error codes and the
 * kernel can only be told apart from the answer.
 * ====================================================================== */

typedef struct {
    uint8_t  writable;             /* 0 = CMD_WRITE is refused outright */
    ImgErr  (*open)(void);
    void    (*invalidate)(void);
    uint8_t (*has_probed)(void);
    uint8_t (*is_present)(void);
    uint32_t (*sector_count)(void);
    uint8_t (*read_sector)(uint32_t lba, uint8_t *dst);
    uint8_t (*write_sector)(uint32_t lba, const uint8_t *src);
    uint8_t (*media_state)(void);
    void    (*media_latch_consume)(void);
} NextorDevice;

/* --- row 1: the read-only floppy image (dsk_image.c) ----------------- */

/* DskImage_Open reports DskErr, not ImgErr. The two enumerations are
 * deliberately kept separate in the headers because each has exactly the
 * codes its own backend can produce, and DskErr's BAD_BPB / TRUNCATED
 * have no meaning for the .img backend (which parses no BPB). The only
 * thing the command path looks at is "did it work", so the mapping
 * through ImgErr is by construction and both sets use 0 for success. */
static ImgErr dev1_open (void) {
    return (ImgErr)DskImage_Open ();
}

/* Device 1 is a floppy image and is never written. This entry exists so
 * the dispatch has no null check: a CMD_WRITE for device 1 has to come
 * back as NEXTOR_ERR_READONLY rather than as a generic failure, and the
 * driver turns those two into different DOS errors. */
static uint8_t dev1_write (uint32_t lba, const uint8_t *src) {
    (void)lba;
    (void)src;
    return 0U;
}

/* --- row 2: the read-write disk image (img_image.c) ------------------- */

static ImgErr dev2_open (void) {
    return ImgImage_Open ();
}

static const NextorDevice s_dev[NEXTOR_DEVICE_COUNT + 1] = {
    /* 0 - unused, see the comment above */
    { 0U, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
    /* 1 - NEXTOR.DSK: 720 KiB read-only floppy */
    { 0U, dev1_open, DskImage_Invalidate, DskImage_HasProbed,
      DskImage_IsPresent, DskImage_SectorCount, DskImage_ReadSector,
      dev1_write, DskImage_MediaState, DskImage_MediaLatchConsume },
    /* 2 - NEXTOR.IMG: read-write disk */
    { 1U, dev2_open, ImgImage_Invalidate, ImgImage_HasProbed,
      ImgImage_IsPresent, ImgImage_SectorCount, ImgImage_ReadSector,
      ImgImage_WriteSector, ImgImage_MediaState, ImgImage_MediaLatchConsume },
};

/* The wire device byte -> a table row, or NULL if it names a device this
 * firmware does not have (0, or anything past NEXTOR_DEVICE_COUNT).
 *
 * The bound is written as an explicit `dev <= COUNT` rather than relying
 * on the row being zero, because a zeroed row has null function pointers
 * and dereferencing one of those is a jump to address 0 in the middle of
 * a command handler. */
static const NextorDevice *dev_lookup (uint8_t dev) {
    if ((dev == 0U) || (dev > NEXTOR_DEVICE_COUNT)) {
        return 0;
    }
    return &s_dev[dev];
}

/* ========================================================================
 * 1b3. Probe policy (kept in nextor.c on purpose; see below).
 *
 * THE PROBE GUARD is the reliability fix for the freeze the old backend
 * shipped with. A probe that has to enumerate the stick runs hundreds
 * of milliseconds of retries (5 x USBH_PreDeal, each with an internal
 * bus-reset + descriptor dance, plus f_mount's own 3 tries with 200 ms
 * delays) - far past the driver's ~0.8 s MB_POLL budget. A probe that
 * starts from inside CAPACITY/READ therefore blows the Z80's command
 * timeout on top of a request the kernel is actively waiting on:
 * CAPACITY returns .DISK, the drive goes away, DOS wedges. The rule
 * enforced everywhere:
 *
 *   - img_probe may only run from the main loop, and only while NO
 *     request is in flight (Nextor_Service's idle path).
 *   - the command handlers NEVER probe. They report what the firmware
 *     knows right now (is_present). A command that arrives while the
 *     image is absent gets the honest fast answer (no media) and the
 *     kernel retries, exactly as its own status machinery expects.
 *   - the first probe is also delayed until the kernel's boot-time
 *     command burst has settled (NEXTOR_PROBE_DELAY_MS from boot), so
 *     the initial CAPACITY/STATUS volley never races the enumeration.
 *
 * ONE rate limiter for both devices, not one each. The expensive part of
 * a probe is shared - USB enumeration and f_mount - so a stick that has
 * just been inserted would otherwise be enumerated twice per window and
 * take twice as long to settle. The limiter therefore gates the whole
 * pass, and a device that is merely absent does not hold the other one
 * back because the pass tries them in order and the first hit is enough
 * to be worth continuing past.
 *
 * The BPB parse, f_open, and sector reads live in the backends; this
 * function only owns the WHEN of probing (Nextor-specific timing policy)
 * and delegates the HOW through the table above.
 * ====================================================================== */
#define NEXTOR_PROBE_DELAY_MS  200U

static uint32_t s_next_probe;       /* ms timestamp of the next probe */

static void img_probe (void) {
    const uint32_t now = ms_now ();
    uint8_t        dev;

    /* Boot delay: skip probes until NEXTOR_PROBE_DELAY_MS after the
     * boot anchor. The first probe lands well inside the kernel's
     * first CAPACITY burst, which is what used to wedge it. */
    if (s_boot_start_ms != 0U
        && (uint32_t)(now - s_boot_start_ms) < NEXTOR_PROBE_DELAY_MS) {
        return;
    }

    for (dev = 1U; dev <= NEXTOR_DEVICE_COUNT; dev++) {
        const NextorDevice *d = &s_dev[dev];

        /* Already open: nothing to do. Re-opening the handle of a device
         * the kernel is actively reading would only add a path parse and
         * a seek to every retry cycle, and would reset its media latch -
         * which would read to the kernel as a media change on every
         * probe, i.e. forever. */
        if (d->is_present ()) {
            continue;
        }

        /* Rate-limit on retries - otherwise a missing stick turns into
         * a continuous enumeration retry that pegs the SCSI bus.
         *
         * has_probed() rather than a per-device timer: a device that has
         * never been tried must not be locked out by the rate limit the
         * other device established, or device 2 could not appear until
         * device 1 had succeeded once. */
        if (d->has_probed ()
            && (uint32_t)(now - s_next_probe) < 2000U) {
            continue;
        }
        s_next_probe = now;
        (void)d->open ();
    }
}

/* ========================================================================
 * 2. Lifecycle
 *
 * (Back to the mailbox itself. Sections 1a-1c above are the state and
 * the helpers the lifecycle and the service path share.)
 * ====================================================================== */

/* Stick removed / volume unmounted: drop BOTH images through their
 * classes. Called from usb_disk.c's disconnect path. The kernel sees a
 * media change to "no media" on each affected drive, which is the honest
 * state - the files it was reading are gone with the stick - and lets
 * DOS re-probe when a fresh stick arrives.
 *
 * Both, not "whichever happens to be open": the check is a few stores
 * and the alternative is a device that keeps answering from a handle
 * FatFs has already invalidated, which reads as corrupt sectors rather
 * than as a missing stick. */
void Nextor_CacheInvalidate (void) {
    DskImage_Invalidate ();
    ImgImage_Invalidate ();
}

/* Power-on mailbox state. Called from main() at boot and again from
 * Cart_SetMapper() on every swap to CART_MAP_NEXTOR.
 *
 * READY is set unconditionally: it means "the firmware is alive and
 * answering", which is the driver's very first question. DONE stays
 * clear until a command actually completes - the driver's MB_POLL loop
 * has to see it change, so pre-setting it would make every command look
 * like it returned stale results. RX_AVAIL and ERR start clear so the
 * first request is answered from a known-empty result FIFO. */
void Nextor_Init (void) {
    s_mb.argp    = s_mb.args;
    s_mb.argend  = s_mb.args;
    s_mb.res_r   = s_mb.res;
    s_mb.res_end = s_mb.res;
    s_mb.cmd     = NEXTOR_CMD_NONE;
    s_mb.armed   = 0U;
    s_mb.seq     = 0U;
    s_mb.status  = NEXTOR_STAT_READY;
    s_mb.err     = NEXTOR_ERR_NONE;
    s_mb.last_seq = 0U;
    /* Calibrate the log's millisecond scale before the first request can
     * be timed. See the "Request timestamps" section. This is the only
     * place that costs a known 4 ms, and it is safe there: the Nextor
     * mapper is installed once per boot (or per soft reset into the cart,
     * terminal.c's soft_reset_into_cart), not per driver query - the Z80
     * reaches its own banks by writing the bank registers, not by asking
     * for a mapper swap. */
    s_cyc_per_ms = 0U;
    cyc_calibrate ();
    s_boot_start_ms = ms_now ();
    s_next_probe    = 0U;
    /* Hand each image class its candidate list. The Open calls run later
     * from the main loop and consume them. Both lists are set up here
     * rather than left as file-static state inside the backends, so that
     * "which file is this device" is answerable from this file alone -
     * which is the question the terminal's Nextor screen also asks. */
    DskImage_Init ();
    DskImage_SetCandidates (s_dsk_names,
                            (uint8_t)(sizeof(s_dsk_names)
                                      / sizeof(s_dsk_names[0])));
    ImgImage_Init ();
    ImgImage_SetCandidates (s_img_names,
                            (uint8_t)(sizeof(s_img_names)
                                      / sizeof(s_img_names[0])));
}

/* Device byte, the first argument of every device-scoped command.
 *
 * Separate from arg_lba() rather than folded into it because the two
 * halves of a READ are read at different times: the device has to be
 * known before the switch can pick a row, and the LBA only matters once
 * it has. */
static uint8_t arg_dev (void) {
    return s_mb.args[0];
}

/* LBA argument, little-endian - MB_SEND pushes the driver's (HL) block
 * verbatim, and driver.asm documents READ/WRITE as "dev, LBA LE(4)".
 *
 * Starts at args[1], past the device byte. */
static uint32_t arg_lba (void) {
    return  (uint32_t)s_mb.args[1]
         | ((uint32_t)s_mb.args[2] << 8)
         | ((uint32_t)s_mb.args[3] << 16)
         | ((uint32_t)s_mb.args[4] << 24);
}

/* Start an answer: empty the result FIFO and clear the error. The
 * driver only issues a new command after draining the previous answer,
 * so there is never a result worth keeping. */
static void res_begin (void) {
    s_mb.res_r   = s_mb.res;
    s_mb.res_end = s_mb.res;
    s_mb.err     = NEXTOR_ERR_NONE;
    s_mb.status &= (uint8_t)~NEXTOR_STAT_RX_AVAIL;
}

/* Publish the result FIFO and release the driver. s_mb.res_end must
 * become visible before DONE, or MB_POLL can return and MB_GETRES can
 * read against a stale end pointer. */
static void res_publish (void) {
    if (s_mb.res_end != s_mb.res_r) {
        s_mb.status |= NEXTOR_STAT_RX_AVAIL;
    }
    if (s_mb.err != NEXTOR_ERR_NONE) {
        s_mb.status |= NEXTOR_STAT_ERR;
    }
    s_mb.status |= NEXTOR_STAT_DONE;
}

/* Record one mailbox write. IRQ context, one caller site, no branches
 * that depend on anything but tr_pos: this runs in the middle of a bus
 * cycle the Z80 is waiting on. tr_tick is the write counter the dumper
 * uses to tell the slots it has actually filled from the ones it has
 * not, which is why the two live in different arrays. */
static void trace_put (uint8_t reg, uint8_t value) {
    const uint8_t pos = s_mb.tr_pos;

    s_mb.tr_reg[pos] = reg;
    s_mb.tr_val[pos] = value;
    s_mb.tr_gen[pos] = s_mb.tr_tick;
    s_mb.tr_tick    = (uint8_t)(s_mb.tr_tick + 1U);
    s_mb.tr_pos     = (uint8_t)((pos + 1U) % NEXTOR_TRACE_N);
}

/* Print the recorded writes newest first, so whatever followed the
 * unexplained one is on top and the command itself is easy to find.
 * The marker goes on the newest CMD write, not on the newest write: an
 * invalid command parks the arg sink but does NOT stop the Z80, so the
 * rest of the burst it was in the middle of lands in the trace after
 * it and the top line would be an argument byte. */
static void trace_dump (void) {
    static const char *const regname[] = {
        "CMD", "DATA", "RXCL", "RXCH", "ERR", "VER"
    };
    const uint8_t tick = s_mb.tr_tick;
    uint8_t       k;
    int           marked = 0;

    for (k = 0U; k < (uint8_t)NEXTOR_TRACE_N; k++) {
        const uint8_t back = (uint8_t)(k + 1U);
        const uint8_t pos  = (uint8_t)((s_mb.tr_pos + NEXTOR_TRACE_N - back)
                                       % NEXTOR_TRACE_N);
        const uint8_t reg  = s_mb.tr_reg[pos];
        const char   *tag;

        /* Every write bumps tr_tick, so a slot is current exactly when
         * its generation is tick - back. Stale slots stop the walk. */
        if ((uint8_t)(tick - s_mb.tr_gen[pos]) != back) {
            break;
        }
        if ((marked == 0) && (reg == NEXTOR_MBOX_CMD)) {
            marked = 1;
            tag    = "   <== the command that was rejected";
        } else {
            tag = "";
        }
        printf ("[nx]      -%-4s <- 0x%02x%s\r\n",
                (reg < 6U) ? regname[reg] : "????",
                (unsigned)s_mb.tr_val[pos], tag);
    }
}

/* One debug line per request, printed only when seq has moved. */
/* Everything log_request() is going to print, captured in one place.
 *
 * The capture has to happen BEFORE res_publish(), not merely at the top
 * of log_request(). res_publish() sets DONE, and the Z80 - which has been
 * blocked in MB_POLL for the whole of Service - can return the instant
 * DONE is visible and start the next burst. The first byte of that burst
 * runs Nextor_WriteByte(), which resets s_mb.argend and rewrites
 * s_mb.args[]. So a field read even a few instructions after the publish
 * describes the NEXT request: `args=` and `lba=` came out as 0, or as a
 * completely unrelated sector, whenever the Z80 won the race.
 *
 * Reading them into locals at the top of log_request() is not enough,
 * because its own first printf blocks: at 921600 baud a 40-character
 * line is ~430 us, while the Z80's MB_POLL spins in ~10 us. Anything
 * read after that first printf is stale. Snapshotting here, while the
 * Z80 cannot possibly have moved on, removes the race instead of making
 * it unlikely.
 *
 * A single field read is a single load on this core, so the individual
 * captures cannot tear against each other either. */
typedef struct {
    uint8_t  cmd;                       /* s_mb.cmd */
    uint8_t  seq;                       /* s_mb.seq */
    uint8_t  dev;                       /* arg_dev(), valid if cmd is scoped */
    uint8_t  err;                       /* s_mb.err */
    uint16_t nres;                      /* result FIFO length */
    uint16_t nargs;                     /* argument count of THIS request */
    uint32_t lba;                       /* arg_lba(), valid for READ/WRITE */
    uint8_t  media;                     /* s_mb.res[0], the STATUS byte */
} log_snap;

/* Is this command scoped to a device? True for every command except
 * HANDSHAKE (which is about the mailbox itself) and ABORT (which is a
 * mailbox-level operation and never reaches a medium). Kept next to
 * s_cmd_args[] because it is the same distinction: those two are the two
 * zero-argument entries in that table. */
static uint8_t cmd_is_scoped (uint8_t cmd) {
    return ((cmd == NEXTOR_CMD_HANDSHAKE) || (cmd == NEXTOR_CMD_ABORT))
           ? 0U : 1U;
}

static void log_request (const log_snap *s) {
    const uint8_t  cmd  = s->cmd;
    const uint8_t  seq  = s->seq;
    const uint8_t  err  = s->err;
    const uint16_t res  = s->nres;
    const uint16_t nargs = s->nargs;
    const uint32_t lba  = s->lba;
    const uint8_t  media = s->media;

    /* One request, one line. If seq moved by more than one, a request
     * was overwritten before Nextor_Service reached it - say so rather
     * than printing a log that looks complete. */
    if ((uint8_t)(seq - s_mb.last_seq) > 1U) {
        printf ("[nx] --- %u request(s) dropped, service fell behind ---\r\n",
                (unsigned)((uint8_t)(seq - s_mb.last_seq) - 1U));
    }
    s_mb.last_seq = seq;

    if (cmd >= NEXTOR_CMD_MAX) {
        printf ("[nx] #%u <invalid cmd 0x%02x> last Z80 writes:\r\n",
                (unsigned)seq, (unsigned)cmd);
        /* Deliberately NOT snapshotted: this is the one case where a
         * burst is still arriving (the sink is parked, so nothing stops
         * the remaining bytes), and the point of the dump is to show
         * what the Z80 wrote. Newer writes landing mid-dump are the
         * interesting ones, not a defect. */
        trace_dump ();
        return;
    }

    printf ("[nx] #%u %s args=%u", (unsigned)seq, s_cmd_name[cmd],
            (unsigned)nargs);
    /* The device byte, for every command that has one. Worth its own
     * field rather than being folded into the command name: with two
     * devices, "CAPACITY args=1" no longer says which capacity, and the
     * interleaving of the kernel's per-device probes is exactly what the
     * log exists to make visible. */
    if (cmd_is_scoped (cmd)) {
        printf (" dev=%u", (unsigned)s->dev);
    }
    if ((cmd == NEXTOR_CMD_READ) || (cmd == NEXTOR_CMD_WRITE)) {
        printf (" lba=0x%08x", (unsigned)lba);
    }
    if (cmd == NEXTOR_CMD_STATUS) {
        /* The media byte is the whole point of this command, and a bare
         * "res=1" in a wall of lines is not readable a hundred lines
         * later. */
        printf (" media=%u%s", (unsigned)media,
                (media == 2U) ? " (changed)" : "");
    }
    if (err != NEXTOR_ERR_NONE) {
        printf (" -> ERR %u", (unsigned)err);
    }
    if (res > 0U) {
        printf (" res=%u", (unsigned)res);
    }
    /* Gap since the previous request. The second line reads how long the
     * Z80 took to get from the driver's banner to its first mailbox
     * request - the 5 s boot stall plus the screen print, so it doubles
     * as a check on the stall. */
    printf ("  +%ums\r\n", (unsigned)ms_since_prev());
}

/* Main-loop service hook. Called from main()'s idle loop; never from
 * IRQ context. This is the only place blocking work and printing may
 * happen - the driver is spinning in MB_POLL on the DONE bit for the
 * whole of this function. */
void Nextor_Service (void) {
    /* Nothing to answer, so use the time to get the image ready. The
     * only slow work that lives outside a command is enumeration /
     * mount / f_open + BPB parse in DskImage_Open (driven from
     * img_probe); READs are served directly from FatFs on the command
     * path. */
    if (s_mb.armed == 0U) {
        (void)img_probe ();
        return;
    }

    /* Take a coherent snapshot of (cmd, args, seq). The Z80 bursts CMD
     * + args as back-to-back cart writes, every one of which raises
     * EXTI0 - the handler runs in IRQ context between those cycles
     * and may complete the burst while THIS code is between the
     * armed read above and the field reads below. Without the
     * snapshot we could answer an old request with another
     * request's LBA - a STATUS immediately followed by a READ (the
     * DOS FCB layer's pattern) would otherwise read STATUS's stale
     * args[] while seq said the READ was the one being answered.
     *
     * The snapshot is taken with interrupts DISABLED, so IRQ cannot
     * land between the field reads. seq is the ticket number - IRQ
     * increments it exactly once per completed burst capture, so a
     * same-seq snapshot is internally consistent by construction.
     *
     * Crucially, this NEVER drops a request. An earlier version
     * returned without answering when seq moved mid-clear; that
     * made the Z80 time out and retry, with 0.5-1.5 s stalls on
     * every STATUS during the kernel's per-sector scan. The window
     * is closed by disabling IRQ around the field reads: the IRQ
     * will land the moment we re-enable, see armed=0, and complete
     * normally on the next pass. */
    uint8_t cmd;
    /* Snapshot fields are kept as separate locals because log_request
     * needs cmd / seq / dev / nargs / lba after the answer path runs and
     * those fields are no longer stable (IRQ has been re-enabled and
     * the Z80 is reading the response FIFO). */
    uint8_t  snap_seq;
    uint8_t  snap_dev;
    uint16_t snap_nargs;
    uint32_t snap_lba;
    /* The device this command is about, or NULL for HANDSHAKE / ABORT
     * and for a device byte that names nothing. Resolved ONCE here, from
     * the snapshot, so the switch below can never pick a row from a
     * half-overwritten args[]. */
    const NextorDevice *dev;
    {
        __disable_irq ();
        s_mb.armed = 0U;
        cmd        = s_mb.cmd;
        snap_seq   = s_mb.seq;
        snap_dev   = arg_dev ();
        snap_nargs = (uint16_t)(s_mb.argend - s_mb.args);
        snap_lba   = arg_lba ();
        __enable_irq ();
    }

    /* dev_lookup() is asked about the device byte only for the commands
     * that have one. HANDSHAKE and ABORT leave args[] holding whatever
     * the PREVIOUS request put there, so looking them up would resolve a
     * stale device and could reject a perfectly good handshake as
     * NEXTOR_ERR_NODEV. */
    dev = cmd_is_scoped (cmd) ? dev_lookup (snap_dev) : 0;

    res_begin ();

    /* A device-scoped command for a device this firmware does not have
     * is one answer for every case below, so it is taken before the
     * switch rather than inside each case. NEXTOR_ERR_NODEV is a
     * firmware/driver version disagreement (the driver built against a
     * different device count), which the handshake is supposed to have
     * caught first - this is the backstop that makes the disagreement
     * visible instead of answering for whichever device happens to be
     * row 1. */
    if (cmd_is_scoped (cmd) && (dev == 0)) {
        s_mb.err = NEXTOR_ERR_NODEV;
    } else {
        switch (cmd) {
        case NEXTOR_CMD_HANDSHAKE: {
            /* "RNX2" + firmware version. Pure firmware state - already a
             * correct answer, and the driver's MB_HANDSHAKE_CHK needs it to
             * conclude the mailbox is alive. The version byte is what makes
             * a driver/firmware mismatch fail here rather than as a stream of
             * wrongly-decoded commands: the driver compares it against
             * MB_EXPECT and skips itself if they differ. */
            static const uint8_t magic[4] = {
                NEXTOR_MAGIC[0], NEXTOR_MAGIC[1],
                NEXTOR_MAGIC[2], NEXTOR_MAGIC[3]
            };
            uint16_t i;
            for (i = 0U; i < 4U; i++) {
                s_mb.res[i] = magic[i];
            }
            s_mb.res[4]   = NEXTOR_VERSION;
            s_mb.res_end  = s_mb.res + 5U;
            break;
        }

        case NEXTOR_CMD_STATUS:
        case NEXTOR_CMD_STAPEEK: {
            /* 0 = no image, 1 = ready, 2 = ready and changed since the last
             * STATUS. The latch is what the "changed once, then ready"
             * tracking in the driver is built on: without it the kernel would
             * never notice that an image appeared after boot.
             *
             * STAPEEK deliberately does not consume it - device query 4 must
             * not disturb query 3's tracking, or a media check in between two
             * status requests would swallow the change.
             *
             * No probing here (see img_probe): STATUS is polled far more
             * often than anything else, including by the kernel's own
             * startup code, and an enumeration retry in the middle of it
             * would blow the driver's command budget. The latch state is kept
             * current by img_probe() from the main loop, so this is a read
             * of state that is at most a couple of seconds old. */
            uint8_t st = dev->media_state ();
            if (cmd == NEXTOR_CMD_STATUS) {
                dev->media_latch_consume ();
            }
            s_mb.res[0]  = st;
            s_mb.res_end = s_mb.res + 1U;
            break;
        }

        case NEXTOR_CMD_ABORT:
            /* Nothing to abort: no request is ever in flight here, because
             * Nextor_Service answers each one in a single pass. */
            break;

        case NEXTOR_CMD_CAPACITY: {
            /* 4-byte block count + 4-byte block size, both little-endian -
             * the wire shape the driver's DO_DEVQ_GET_PARAMS streams straight
             * into the kernel's device parameter block.
             *
             * No image is reported as an error rather than as a zero
             * capacity: the driver turns a failed CAPACITY into "0 sectors,
             * device still exists", which is what makes the kernel keep the
             * drive and retry later instead of dropping the device. */
            if (!dev->is_present ()) {
                s_mb.err = NEXTOR_ERR_NO_MEDIA;
                break;
            }
            {
                uint32_t v = dev->sector_count ();
                uint8_t  i;
                for (i = 0U; i < 4U; i++) {
                    s_mb.res[i]     = (uint8_t)(v >> (8U * i));
                    s_mb.res[4U + i] = (uint8_t)(NEXTOR_SECTOR_SIZE >> (8U * i));
                }
            }
            s_mb.res_end = s_mb.res + 8U;
            break;
        }

        case NEXTOR_CMD_READ: {
            /* One sector straight from the backend into the result FIFO. The
             * buffer is exactly one sector long, so a read can never overrun
             * it, and the driver drains it as soon as DONE appears. */
            if (dev->read_sector (snap_lba, s_mb.res) == 0U) {
                s_mb.err = NEXTOR_ERR_NO_MEDIA;
                break;
            }
            s_mb.res_end = s_mb.res + NEXTOR_SECTOR_SIZE;
            break;
        }

        case NEXTOR_CMD_WRITE: {
            /* A real write, for device 2. The sector is the tail of args[] -
             * args[0] is the device, args[1..4] the LBA, args[5..516] the
             * 512 bytes - and is handed to the backend where it lies, with no
             * bounce copy: NEXTOR_ARG_MAX is sized so the data ends exactly at
             * the end of the array, so &args[5] is a whole sector by
             * construction and a separate 512-byte buffer would be RAM for
             * nothing.
             *
             * Nothing is produced in the answer - a successful write is a
             * 0-byte answer - so res_end stays where res_begin put it and the
             * driver sees DONE with no result to drain.
             *
             * The two failure modes are kept distinct on purpose. A read-only
             * device is a permanent condition the driver already knows about
             * (it refuses locally, before the command is ever sent), so this
             * is only the backstop; a missing medium is the honest answer
             * when there is no image; and a write that FatFs refused is the
             * one case where the medium IS there and the write did not
             * happen, which is NEXTOR_ERR_IO and not a media error. Reporting
             * the last of those as "no media" would make the kernel unmount a
             * disk that is perfectly present. */
            if (dev->writable == 0U) {
                s_mb.err = NEXTOR_ERR_READONLY;
                break;
            }
            if (!dev->is_present ()) {
                s_mb.err = NEXTOR_ERR_NO_MEDIA;
                break;
            }
            if (dev->write_sector (snap_lba, &s_mb.args[5]) == 0U) {
                s_mb.err = NEXTOR_ERR_IO;
                break;
            }
            break;
        }

        default:
            /* Unknown command byte. Nextor_WriteByte() parks the collector
             * for those instead of arming, so reaching here means the byte
             * was valid but this switch has no case for it - a firmware bug,
             * not a driver one. */
            s_mb.err = NEXTOR_ERR_IO;
            break;
        }
    }

    /* Snapshot for the log. The request fields (cmd, seq, dev, nargs,
     * lba) were captured above, IRQ-protected, before res_begin. The
     * response fields (err, nres, media) are written only by US
     * after that snapshot, with IRQ re-enabled - IRQ doesn't touch
     * them in any way that affects the answer (s_mb.err is read only
     * by the Z80; s_mb.res[] is popped by the Z80 too, never by
     * IRQ). The only field IRQ does touch is s_mb.res_r, on pop,
     * and at most one byte may have been popped between res_publish
     * and the log call - that is a cosmetic one-byte count error in
     * the log line, never a correctness issue. */
    {
        const log_snap snap = {
            cmd,
            snap_seq,
            snap_dev,
            s_mb.err,
            (uint16_t)(s_mb.res_end - s_mb.res_r),
            snap_nargs,
            snap_lba,
            s_mb.res[0]
        };

        res_publish ();
        log_request (&snap);
    }
}

/* ========================================================================
 * 3. Code entries - called from Cart_EXTI0_Nextor_Handler.
 *
 * Both are reached only for addresses in NEXTOR_MBOX_BASE..
 * NEXTOR_MBOX_END, so the register index is just the A2..A4 slot
 * number and needs no further range check. Keeping the window test in
 * the handler means every decode miss there is address-local and
 * silent.
 *
 * NEXTOR_MBOX_REG() is (address - BASE). The low address bits do
 * arrive intact - the bank registers in cart.c are compared exactly,
 * including 0x77FF which pins A0..A7 - so the index is a straight
 * subtraction with no masking.
 *
 * IRQ context. No blocking calls, no printf, no long loops. Every store
 * below is load-bearing for the next bus cycle: the Z80 is holding the
 * cart bus and the next request byte arrives on the very next
 * interrupt. trace_put() is the one addition over what the Z80 can see
 * and is four stores with no load-dependent branch; a 516-byte WRITE
 * burst still costs 32 us of trace against 6.7 ms of bus time.
 * ====================================================================== */

void Nextor_WriteByte (uint16_t address, uint8_t value) {
    const uint8_t reg = NEXTOR_MBOX_REG (address);

    trace_put (reg, value);

    if (reg == NEXTOR_MBOX_CMD) {
        uint8_t cmd = value;

        /* DONE must fall here, not in Service: MB_POLL returns the
         * instant DONE is visible, so clearing it late would let the
         * driver read the PREVIOUS answer. */
        s_mb.status &= (uint8_t)~NEXTOR_STAT_DONE;

        if (cmd >= NEXTOR_CMD_MAX) {
            /* Park the sink at the empty range. The burst that follows
             * is of unknown length, and guessing would either overrun
             * args[] or arm on a short burst. */
            s_mb.cmd    = NEXTOR_CMD_NONE;
            s_mb.argend = s_mb.args;
        } else {
            s_mb.cmd    = cmd;
            s_mb.argend = s_mb.args + s_cmd_args[cmd];
        }
        s_mb.argp  = s_mb.args;
        /* Zero-argument commands (all but READ/WRITE) are complete the
         * moment the command byte lands - there is no DATA write to trip
         * the completion test in the branch below. seq is bumped here
         * for exactly that reason: it counts REQUESTS, and a request
         * whose count comes from s_cmd_args[] alone must still be
         * counted or Nextor_Service's print-once gap check goes blind
         * to it. */
        if (s_mb.argp == s_mb.argend) {
            s_mb.armed = 1U;
            s_mb.seq++;
        } else {
            s_mb.armed = 0U;
        }
        return;
    }

    if (reg == NEXTOR_MBOX_DATA) {
        /* Argument push - the throughput-critical path, one interrupt
         * per byte for the length of the burst.
         *
         * argp and argend are each read once into a local. Neither can
         * legally change underneath us: interrupts are already masked
         * in EXTI context and no other code path advances the sink.
         * They are volatile, so written inline as `args[argp++]` each
         * one costs an extra load from memory - hoisting by hand is
         * what turns this into two loads, one store and two compares. */
        uint8_t *p   = s_mb.argp;
        uint8_t *end = s_mb.argend;

        /* The bound is the overflow guard: s_cmd_args[] set argend, and
         * anything past it is dropped rather than allowed to run off
         * args[]. */
        if (p < end) {
            *p = value;
            s_mb.argp = ++p;
            if (p == end) {
                s_mb.armed = 1U;
                s_mb.seq++;
            }
        }
        return;
    }

    /* 0x7FF2..0x7FF5 are read-only; ignore writes rather than let a
     * stray write corrupt the collector. */
}

uint8_t Nextor_ReadByte (uint16_t address) {
    const uint8_t reg = NEXTOR_MBOX_REG (address);

    switch (reg) {
    case NEXTOR_MBOX_CMD:
        /* Same index as NEXTOR_MBOX_STAT - the protocol is
         * write=command / read=status on 0x7FF0. Returned precomputed
         * so MB_POLL's spin is a single load. */
        return s_mb.status;

    case NEXTOR_MBOX_DATA: {
        /* Result pop. The driver pops a known count, so this only has
         * to stay in range; an empty FIFO returns 0xFF (the
         * floating-bus pattern) so a driver that over-pops sees a
         * stable wrong value and bails instead of spinning forever. */
        uint8_t *p   = s_mb.res_r;
        uint8_t *end = s_mb.res_end;
        uint8_t v;
        if (p < end) {
            v = *p;
            s_mb.res_r = ++p;
            if (p == end) {
                s_mb.status &= (uint8_t)~NEXTOR_STAT_RX_AVAIL;
            }
        } else {
            v = 0xFFU;
        }
        return v;
    }

    case NEXTOR_MBOX_RXCNT_LO:
        /* Informational only (driver.asm never reads it) but cheap to
         * keep honest. */
        return (uint8_t)(s_mb.res_end - s_mb.res_r);

    case NEXTOR_MBOX_RXCNT_HI:
        return (uint8_t)((s_mb.res_end - s_mb.res_r) >> 8);

    case NEXTOR_MBOX_ERR:
        return s_mb.err;

    case NEXTOR_MBOX_VER:
        return NEXTOR_VERSION;

    default:
        /* Slots 6 and 7 (0x7FF8, 0x7FFC) are decoded by the handler but
         * unused by the protocol. */
        return 0xFFU;
    }
}
