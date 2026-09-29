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
 * Device 1 = one fixed, read-only 720 KiB .dsk image
 * ---------------------------------------------------------------------------
 * The disk behind device 1 is an MSX-DOS floppy image file on the USB
 * stick: 1440 sectors of 512 bytes, FAT12, no partition table, boot
 * sector at LBA 0. The driver flags the device as a floppy disk drive
 * (device query 2, flag bit 2), which is what makes the kernel map a
 * drive onto sector 0 directly rather than hunting for partitions.
 *
 * Reads are served from the file itself - f_lseek + f_read into the
 * result FIFO - with the handle kept open across commands, so a sector
 * costs one seek and one 512-byte read and no path parsing. A failed
 * FatFS call drops the handle: the stick may have been pulled mid
 * session, and the next command then re-probes, which is also how the
 * image comes back after a re-plug.
 *
 * The obvious next optimisation, and deliberately not done here: cache
 * the whole 737280-byte image in PSRAM at Nextor_Init and serve reads
 * from there. It is not done here because the 1-2 s it takes to stream
 * the image off the stick would have to happen while the Z80 is already
 * polling, and reads out of the file are already faster than the Z80
 * can drain the result FIFO.
 *
 * Where the probing happens, and why it is split in two:
 *
 *   request path  CAPACITY and READ call img_ensure() and will mount
 *                 the volume if it has to. That can be slow while a
 *                 stick is still enumerating, and slower than the
 *                 driver's ~0.8 s command budget, so it is not done
 *                 from a command that is cheap to answer otherwise.
 *   main loop     img_probe() runs from Nextor_Service while no request
 *                 is in flight, so by the time the kernel asks its first
 *                 question the file is normally already open and the
 *                 answer is immediate.
 *   STATUS never probes at all - it reports what the firmware currently
 *                 knows, because the kernel polls status far more often
 *                 than it asks for real work.
 * ---------------------------------------------------------------------------
 */

#include "nextor.h"
#include "usb_disk.h"
#include "ff.h"
#include <stdio.h>

/* ========================================================================
 * Wire contract
 * ====================================================================== */

/* driver.asm: "512-byte sectors, read-only floppy image, 1440 sectors."
 * The result FIFO is exactly one sector, so this is both the driver's
 * sector size and the largest answer a single command can produce. */
#define NEXTOR_SECTOR_SIZE   512U

/* Largest argument burst: WRITE = LBA(4) + one sector. */
#define NEXTOR_ARG_MAX       (4U + NEXTOR_SECTOR_SIZE)    /* 516 */

/* Largest result burst: READ = one sector. */
#define NEXTOR_RES_MAX       NEXTOR_SECTOR_SIZE           /* 512 */

/* Mailbox writes remembered for the invalid-command report. 24 is about
 * one HANDSHAKE plus its poll, which is the shortest span that can hold
 * an unexplained CMD write together with whatever it interrupted. */
#define NEXTOR_TRACE_N       24U

/* Argument bytes each command pushes after its command byte.
 * Transcribed from driver.asm lines 25-33. Indexed by command byte.
 *
 * The value doubles as the bound for the arg sink, which is what makes
 * an over-long burst impossible to overrun args[] with. */
static const uint16_t s_cmd_args[NEXTOR_CMD_MAX] = {
    0U,                                   /* 0x00 HANDSHAKE */
    0U,                                   /* 0x01 CAPACITY  */
    0U,                                   /* 0x02 STATUS    */
    4U,                                   /* 0x03 READ: LBA */
    (uint16_t)(4U + NEXTOR_SECTOR_SIZE),  /* 0x04 WRITE: LBA + data */
    0U,                                   /* 0x05 ABORT     */
    0U,                                   /* 0x06 STAPEEK   */
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

/* ========================================================================
 * 1a. The image behind device 1.
 *
 * A fixed, read-only 720 KiB MSX-DOS disk image held in a file on the
 * USB stick. One file name, one geometry, no writing, no hot-swap
 * handling beyond "re-probe if a read fails": the point is that the
 * kernel gets a plain, always-the-same disk, not a second block device.
 *
 * Why a file and not the raw stick: the image is what MSX-DOS actually
 * wants to boot (FAT12, a boot sector at LBA 0), and it keeps the
 * driver's device model to one read-only floppy-flavored block device
 * with a fixed size. Turning the raw stick into a second, writable
 * device is a follow-up; nothing in this file prevents it, the state
 * here is just per-medium.
 *
 * Everything below is deliberately tiny and stateless-looking because
 * the driver is a ROM driver with no RAM of its own: all the mutable
 * bookkeeping (handle, sector count, change latch) lives here on the
 * firmware side, where it can actually be written to.
 * ====================================================================== */

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
 * A short candidate list rather than one name, because "the file is
 * called something else" and "the image is not there" look identical
 * from the MSX side: no drive, no error, nothing in the log but one
 * f_open failure. Two names cost 18 bytes of flash and remove the most
 * likely reason for that. Order matters - the first hit wins. */
static const char *const s_img_names[] = {
    "0:/NEXTOR.DSK",
    "0:/RISKYMSX.DSK",
};
#define NEXTOR_IMG_NAME_COUNT (sizeof(s_img_names) / sizeof(s_img_names[0]))

/* 720 KiB = 1440 x 512. The driver takes the sector count from
 * CMD_CAPACITY rather than hardcoding it, so these two constants are
 * the single source of truth for the size. */
#define NEXTOR_IMG_SECTORS    1440U
#define NEXTOR_IMG_BYTES      (NEXTOR_IMG_SECTORS * NEXTOR_SECTOR_SIZE)

/* Boot sector + root directory LBA range that is ALWAYS served from
 * Direct from FatFs: LBAs come straight from the .dsk file on the USB
 * stick. The kernel's BPB parsing and every directory entry lookup
 * land in this window. 7 = the first root-directory sector for this
 * BPB (1 boot + 2x3 FAT). */
#define NEXTOR_IMG_ROOT_LBA       7U
#define NEXTOR_IMG_ROOT_SECTORS   7U

/* Image state. Only the service/main-loop side ever touches this.
 *
 *   open     the FIL is open and usable.
 *   sectors  how many 512-byte sectors the file holds, capped at
 *            NEXTOR_IMG_SECTORS. 0 means "no image", which the driver
 *            is told as a zero capacity / no media.
 *   present  what STATUS last reported, kept so the change latch below
 *            only fires on an actual transition.
 *   changed  the media-change latch: set on a transition, consumed by
 *            STATUS, left alone by STAPEEK (device query 4 must not
 *            disturb query 3's tracking).
 *   probed   1 once a probe has run, so "no image" is not reported as
 *            a change on every STATUS.
 *   failed   count of consecutive failed read commands, kept for
 *            diagnostic log output only.
 */
static struct {
    FIL     fp;
    uint32_t sectors;
    uint32_t next_probe;  /* ms timestamp of the next allowed probe   */
    uint8_t  open;
    uint8_t  present;
    uint8_t  changed;
    uint8_t  probed;      /* 1 once a probe has run, so "no image" is
                           * not reported as a change on every STATUS  */
    uint8_t  failed;
    uint8_t  last_fr;     /* FRESULT of the last f_open, for the log   */
} s_img;

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
 * 1c. Image access
 *
 * The whole disk backend is these four functions. Everything is a plain
 * FatFs file read; the only real decisions are (a) how often to probe,
 * and (b) what to tell the driver when there is no image.
 * ====================================================================== */

/* Drop the image and forget that we ever had one. Also resets the
 * fill cursor so a fresh open starts streaming from the file's sector
 * 0 again. */
static void img_reset (void) {
    if (s_img.open != 0U) {
        (void)f_close(&s_img.fp);
    }
    s_img.open       = 0U;
    s_img.sectors    = 0U;
    s_img.present    = 0U;
    s_img.changed    = 0U;
    s_img.probed     = 0U;
    s_img.last_fr    = 0U;
    s_img.next_probe = 0U;
    s_img.failed     = 0U;
}

/* Note a presence transition and arm the media-change latch. The latch
 * is what makes device query 3 report "changed" exactly once after the
 * image appears, which is how the kernel learns a drive has become
 * usable without being told to look. */
static void img_set_present (uint8_t now) {
    if ((s_img.probed != 0U) && (now == s_img.present)) {
        return;                         /* not a transition */
    }
    if (s_img.probed != 0U) {
        s_img.changed = 1U;
    }
    s_img.present = now;
    s_img.probed  = 1U;
}

/* Open the image, once, and restart the streaming fill. Safe to call
 * from the main loop and from the service path; it is the only place
 * that touches USB_TryEnsureMounted and f_open.
 *
 * THE PROBE GUARD is the reliability fix for the freeze the old backend
 * shipped with. A probe that has to enumerate the stick runs hundreds
 * of milliseconds of retries (5 x USBH_PreDeal, each with an internal
 * bus-reset + descriptor dance, plus f_mount's own 3 tries with 200 ms
 * delays) - far past the driver's ~0.8 s MB_POLL budget. A probe that
 * starts from inside CAPACITY/READ (img_ensure -> img_probe) therefore
 * blows the Z80's command timeout on top of a request the kernel is
 * actively waiting on: CAPACITY returns .DISK, the drive goes away,
 * DOS wedges. The rule enforced everywhere below:
 *
 *   - img_probe may only run from the main loop, and only while NO
 *     request is in flight (Nextor_Service's img_probe path).
 *   - img_ensure NEVER probes. It reports what the firmware knows
 *     right now. A command that arrives while the image is absent
 *     gets the honest fast answer (no media) and the kernel retries,
 *     exactly as its own status machinery expects.
 *   - the first probe is also delayed until the kernel's boot-time
 *     command burst has settled (NEXTOR_PROBE_DELAY from boot), so
 *     the initial CAPACITY/STATUS volley never races the enumeration.
 */
#define NEXTOR_PROBE_DELAY_MS  1500U

static void img_probe (void) {
    FRESULT    fr;
    FSIZE_t    sz;
    uint32_t   sectors;
    const char *name = "?";
    const uint32_t now = ms_now();

    if (s_img.open != 0U) {
        return;                         /* already have it */
    }
    /* Boot-delay: the kernel's initial CAPACITY/STATUS volley arrives
     * within the first seconds after the mapper swap. Running the first
     * probe (enumeration, f_mount, f_open - hundreds of ms of USB
     * activity) while that volley is being answered is what used to
     * wedge CAPACITY. The probe waits out a quiet window instead. The
     * delay anchor is the boot timestamp taken in Nextor_Init; probes
     * triggered later (rate-limited retries) pass this check trivially
     * because ms_now() has long since moved past it. */
    if (s_boot_start_ms != 0U
        && (uint32_t)(now - s_boot_start_ms) < NEXTOR_PROBE_DELAY_MS) {
        return;
    }
    if (s_img.probed != 0U) {
        /* Rate limit. Without this the main loop's "is it armed?" check
         * turns into a continuous enumeration retry, which keeps the
         * stick's SCSI bus busy for no reason. */
        if ((uint32_t)(now - s_img.next_probe) < 2000U) {
            return;
        }
    }
    s_img.next_probe = now;

    if (USB_TryEnsureMounted() != DEF_SUCCESS) {
        img_set_present(0U);
        return;
    }

    /* Try each candidate in turn. The last FRESULT is the one reported:
     * FR_NO_FILE for every name is the informative case, and any other
     * code (a name that is a directory, say) is worth seeing too. */
    {
        uint32_t i;

        name = s_img_names[0];
        for (i = 0U; i < NEXTOR_IMG_NAME_COUNT; i++) {
            name = s_img_names[i];
            fr   = f_open(&s_img.fp, name, FA_READ);
            s_img.last_fr = (uint8_t)fr;
            if (fr == FR_OK) {
                break;
            }
        }
    }
    if (fr != FR_OK) {
        img_set_present(0U);
        return;
    }

    /* A 720 KiB image is the contract; anything shorter is served
     * truncated (a partly written file still boots what it has) and
     * anything longer is capped, so the driver never hands the kernel a
     * sector count the file cannot back. */
    sz      = f_size(&s_img.fp);
    sectors = (sz > (FSIZE_t)NEXTOR_IMG_BYTES)
            ? NEXTOR_IMG_SECTORS
            : (uint32_t)(sz / (FSIZE_t)NEXTOR_SECTOR_SIZE);
    if (sectors > NEXTOR_IMG_SECTORS) {
        sectors = NEXTOR_IMG_SECTORS;
    }

    s_img.sectors = sectors;
    s_img.open    = 1U;
    s_img.failed   = 0U;
    img_set_present((uint8_t)((sectors != 0U) ? 1U : 0U));
    printf ("[nx] image '%s' open: %u bytes, %u sectors of %u%s\r\n",
            name, (unsigned)sz, (unsigned)sectors,
            (unsigned)NEXTOR_SECTOR_SIZE,
            (sectors != NEXTOR_IMG_SECTORS) ? " (not a full 720K image)" : "");
}

/* Invalidate the image handle: the stick was removed/re-plugged (or the
 * volume was unmounted by anyone else), so the open file handle may be
 * pointing at an unmounted volume. Called from usb_disk.c's disconnect
 * path. The next img_probe() in the main loop reopens the file from
 * the (new) stick.
 *
 * This does NOT arm the media-change latch: the served content does not
 * change on a refresh (the file is always sector 0..NEXTOR_IMG_SECTORS),
 * so the kernel should not be told its fixed disk changed just because
 * the stick behind it was re-inserted. */
void Nextor_CacheInvalidate (void) {
    if (s_img.open != 0U) {
        (void)f_close(&s_img.fp);
        printf("[nx] image handle closed: stick removed\r\n");
    }
    s_img.open     = 0U;
    s_img.sectors  = 0U;
    s_img.failed   = 0U;
    img_set_present(0U);
}

/* Report image availability WITHOUT probing. The command handlers use
 * this; a command that arrives while the image is absent gets the fast,
 * honest "no media" answer instead of a multi-hundred-ms enumeration
 * that would blow the driver's MB_POLL budget (see img_probe's guard
 * note). The main loop's img_probe() is what turns "absent" into
 * "present" over time. */
static uint8_t img_ensure (void) {
    return (uint8_t)((s_img.open != 0U && s_img.sectors != 0U) ? 1U : 0U);
}

/* Read one sector into dst. Returns 1 on success.
 *
 * SERVED FROM THE PSRAM CACHE ONLY. This runs inside Nextor_Service
 * with the Z80 spinning in MB_POLL - the whole answer has to be far
 * under the ~0.8 s command budget, and a PSRAM word-copy is
 * microseconds. No USB, no FatFs, no printf on this path.
 */
static uint8_t img_read_sector (uint32_t lba, uint8_t *dst) {
    UINT    br = 0U;
    FRESULT fr;

    if (img_ensure() == 0U) {
        return 0U;
    }
    if (lba >= s_img.sectors) {
        return 0U;                       /* past the end of the image */
    }

    /* Direct from FatFs, no PSRAM cache. f_lseek + f_read of one
     * sector on a healthy stick is ~2-10 ms, well under the ~0.8 s
     * MB_POLL budget. A slow stick will time out and the Z80 will
     * retry - that's the same behavior the cache used to mask, but
     * now visible in the trace instead of hidden. */
    fr = f_lseek(&s_img.fp, (FSIZE_t)lba * (FSIZE_t)NEXTOR_SECTOR_SIZE);
    if (fr == FR_OK) {
        fr = f_read(&s_img.fp, dst, NEXTOR_SECTOR_SIZE, &br);
    }
    if ((fr != FR_OK) || (br != NEXTOR_SECTOR_SIZE)) {
        /* Stick error mid-transfer: report honestly, keep the image
         * handle open so the next command can retry. The probe path
         * (img_probe) handles handle-loss recovery. */
        return 0U;
    }

    /* BOOT SECTOR PATCHING REMOVED (was: "hidden sectors = 0" at
     * 0x1C..0x1F). The vendor image's 16-bit hidden field (0x1C..0x1D)
     * is ALREADY 0; bytes 0x1E..0x1F are the JR opcode of the DOS-2.20
     * boot block that lives at BOOTAD+1Eh, and zeroing them destroyed
     * that code on every boot - the sector read back with its loader
     * jump gone. The image now ships with a correct boot sector (see
     * make_boot_dsk.py), so nothing is patched here any more.
     *
     * Kept: the root-directory attribute fix below (NEXTOR.SYS needs
     * a system attribute for TRY_MSX_DOS's FCB open to accept it).
     */

    /* The vendor NEXTOR.DSK is a *tools disk*, not a boot disk.
     * NEXTOR.SYS and NEXTORJ.SYS have attribute 0x20 (archive only),
     * but a bootable system file needs attribute 0x21 (system) or
     * 0x27 (system+hidden). Patch the directory entry attribute on
     * the fly when serving root directory sectors (LBA 7-13). */
    if (lba >= NEXTOR_IMG_ROOT_LBA
        && lba < NEXTOR_IMG_ROOT_LBA + NEXTOR_IMG_ROOT_SECTORS) {
        /* Root directory: 112 entries, 32 bytes each, 7 sectors.
         * Scan for "NEXTOR  SYS" and "NEXTORJ SYS" and set attr=0x27. */
        for (uint16_t off = 0; off < 512; off += 32) {
            if (dst[off] == 0x00) break;           /* end of directory */
            if (dst[off] == 0xE5) continue;        /* deleted */
            if ((dst[off + 11] & 0x08) != 0) continue; /* volume label */

            /* Check name "NEXTOR  SYS" (8+3, space-padded) */
            if (dst[off + 0] == 'N' && dst[off + 1] == 'E' &&
                dst[off + 2] == 'X' && dst[off + 3] == 'T' &&
                dst[off + 4] == 'O' && dst[off + 5] == 'R' &&
                dst[off + 6] == ' ' && dst[off + 7] == ' ' &&
                dst[off + 8] == 'S' && dst[off + 9] == 'Y' &&
                dst[off + 10] == 'S') {
                dst[off + 11] = 0x27;  /* system+hidden+archive */
            }
            /* Check name "NEXTORJ SYS" */
            if (dst[off + 0] == 'N' && dst[off + 1] == 'E' &&
                dst[off + 2] == 'X' && dst[off + 3] == 'T' &&
                dst[off + 4] == 'O' && dst[off + 5] == 'R' &&
                dst[off + 6] == 'J' && dst[off + 7] == ' ' &&
                dst[off + 8] == 'S' && dst[off + 9] == 'Y' &&
                dst[off + 10] == 'S') {
                dst[off + 11] = 0x27;  /* system+hidden+archive */
            }
        }
    }
    return 1U;
}

/* Direct read path. img_read_sector above reads straight from FatFs.
 * No on-demand cache fetch and no boot-window prime - both were
 * scaffolding for the now-removed PSRAM cache. */

/* ========================================================================
 * 2. Lifecycle
 *
 * (Back to the mailbox itself. Sections 1a-1c above are the state and
 * the helpers the lifecycle and the service path share.)
 * ====================================================================== */

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
    img_reset ();
}

/* LBA argument, little-endian - MB_SEND pushes the driver's (HL) block
 * verbatim, and driver.asm documents READ/WRITE as "LBA LE(4)". */
static uint32_t arg_lba (void) {
    return  (uint32_t)s_mb.args[0]
         | ((uint32_t)s_mb.args[1] << 8)
         | ((uint32_t)s_mb.args[2] << 16)
         | ((uint32_t)s_mb.args[3] << 24);
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
    uint8_t  err;                       /* s_mb.err */
    uint16_t nres;                      /* result FIFO length */
    uint16_t nargs;                     /* argument count of THIS request */
    uint32_t lba;                       /* arg_lba(), valid for READ/WRITE */
    uint8_t  media;                     /* s_mb.res[0], the STATUS byte */
} log_snap;

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
     * mount / f_open in img_probe(); READs are served directly from
     * FatFs on the command path. */
    if (s_mb.armed == 0U) {
        img_probe ();
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
     * needs cmd / seq / nargs / lba after the answer path runs and
     * those fields are no longer stable (IRQ has been re-enabled and
     * the Z80 is reading the response FIFO). */
    uint8_t  snap_seq;
    uint16_t snap_nargs;
    uint32_t snap_lba;
    {
        __disable_irq ();
        s_mb.armed = 0U;
        cmd        = s_mb.cmd;
        snap_seq   = s_mb.seq;
        snap_nargs = (uint16_t)(s_mb.argend - s_mb.args);
        snap_lba   = arg_lba ();
        __enable_irq ();
    }

    res_begin ();

    switch (cmd) {
    case NEXTOR_CMD_HANDSHAKE: {
        /* "RNX2" + firmware version. Pure firmware state - already a
         * correct answer, and the driver's MB_HANDSHAKE_CHK needs it to
         * conclude the mailbox is alive. */
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
         * would blow the driver's command budget. s_img is kept current
         * by img_probe() from the main loop, so this is a read of state
         * that is at most a couple of seconds old. */
        uint8_t st;
        if (s_img.sectors == 0U) {
            st = 0U;                       /* no image attached */
        } else if (s_img.changed != 0U) {
            st = 2U;                       /* attached, changed */
        } else {
            st = 1U;                       /* attached, unchanged */
        }
        if (cmd == NEXTOR_CMD_STATUS) {
            s_img.changed = 0U;            /* consumed; STAPEEK leaves it */
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
        if (img_ensure () == 0U) {
            s_mb.err = NEXTOR_ERR_NO_MEDIA;
            break;
        }
        {
            uint32_t v = s_img.sectors;
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
        /* One sector straight out of the PSRAM cache into the result
         * FIFO. The buffer is exactly one sector long, so a read can
         * never overrun it, and the driver drains it as soon as DONE
         * appears. */
        uint32_t lba = arg_lba();
        if (img_read_sector (lba, s_mb.res) == 0U) {
            s_mb.err = NEXTOR_ERR_NO_MEDIA;
            break;
        }
        s_mb.res_end = s_mb.res + NEXTOR_SECTOR_SIZE;
        break;
    }

    case NEXTOR_CMD_WRITE:
        /* Refused, not ignored. The device is read only, and answering
         * DONE with an empty FIFO would leave the driver believing the
         * sector was written. The driver never sends this (it returns
         * .WPROT locally), so this is the backstop for a stray command. */
        s_mb.err = NEXTOR_ERR_READONLY;
        break;

    default:
        /* Unknown command byte. Nextor_WriteByte() parks the collector
         * for those instead of arming, so reaching here means the byte
         * was valid but this switch has no case for it - a firmware bug,
         * not a driver one. */
        s_mb.err = NEXTOR_ERR_IO;
        break;
    }

    /* Snapshot for the log. The request fields (cmd, seq, nargs, lba)
     * were captured above, IRQ-protected, before res_begin. The
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
