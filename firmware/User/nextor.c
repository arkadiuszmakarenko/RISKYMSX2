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
 * What is behind the mailbox now
 * ---------------------------------------------------------------------------
 * One raw SCSI medium: the whole USB stick, in raw_disk.c. There is no
 * filesystem in the data path, so this file's command handlers are a
 * straight translation of "the kernel asked for sector N" into
 * "RawDisk_ReadSectors(N, 1, buf)".
 *
 * That is a very different file from the one that served two files
 * through FatFs, and the difference is not cosmetic:
 *
 *   - A command handler is now allowed to block. A READ is a USB bulk
 *     transfer of 512 bytes plus a CSW, so it costs hundreds of
 *     microseconds to a few milliseconds of genuine I/O. Nothing on the
 *     path takes a filesystem lock, walks a FAT, or waits on another
 *     layer's state machine, so there is nothing left for it to
 *     deadlock against. (With FatFs in the path, that is exactly what
 *     could not be argued: f_read could block inside the volume's
 *     sector cache while a previous command's state was still live.)
 *
 *   - The device exists or it does not, and "does not" is answered as a
 *     sector count of 0 rather than as a failure. There is no
 *     intermediate state in which a file is present but its handle is
 *     not open yet.
 *
 *   - There is no per-device dispatch. With one medium, dev_lookup(),
 *     the device table and the device byte on the wire all went away:
 *     every device-scoped command is about the same drive. See the note
 *     on the wire protocol version in nextor.h.
 *
 * ---------------------------------------------------------------------------
 * Split of duties: what runs where, and why
 * ---------------------------------------------------------------------------
 *   IRQ context    Nextor_ReadByte / Nextor_WriteByte. Latch a command,
 *                  capture its argument bytes, pop result bytes. No
 *                  printf, no USB, no loops with a bound other than the
 *                  FIFO's own bound.
 *   main loop      Nextor_Service. Everything that can take time:
 *                  answering a READ or WRITE, and probing the stick.
 *
 * The IRQ side is bounded and unconditional, which is the property that
 * makes the cart bus safe to leave enabled: the Z80 can burst a whole
 * 517-byte WRITE at the window and the handler must never be the reason
 * a cycle is missed.
 * ---------------------------------------------------------------------------
 *
 * Probing, and why it is not done from a command handler
 * -------------------------------------------------------------
 * Enumerating a USB stick takes hundreds of milliseconds - retries, a
 * bus reset, descriptor fetches - and the driver's command budget is
 * about 0.8 s (MB_POLL in driver.asm spins three passes of a 16-bit
 * counter). Probing from inside a command therefore spends most of the
 * kernel's patience on a request the kernel is actively blocked on, and
 * the failure mode is not a slow answer but a timeout: the drive goes
 * away and DOS wedges.
 *
 * So the rule is:
 *
 *   - raw_probe() may only run from the main loop, and only while NO
 *     request is in flight (Nextor_Service's idle path).
 *   - the command handlers NEVER probe. CAPACITY reports the capacity
 *     the firmware currently knows, which is 0 until the first probe
 *     succeeds - an honest answer that the kernel retries, exactly as
 *     its own status machinery expects.
 *   - STATUS never probes either, and never touches the USB bus at all.
 *     It reports cached state. The kernel polls status far more often
 *     than it asks for real work, and the polled value is at most one
 *     probe interval stale.
 *   - the first probe is also delayed until the kernel's boot-time
 *     command burst has settled (NEXTOR_PROBE_DELAY_MS from boot), so
 *     the initial CAPACITY/STATUS volley never races enumeration.
 *
 * Probing is rate limited to one attempt every NEXTOR_PROBE_RETRY_MS,
 * with the exception of a device that has never been probed: a stick
 * that is simply not plugged in must not turn into a continuous
 * enumeration retry that pegs the USB host controller.
 * ---------------------------------------------------------------------------
 */

#include "nextor.h"
#include "raw_disk.h"
#include "ch32v4x7_conf.h"
#include <stdio.h>
#include <string.h>

/* ---------------------------------------------------------------------------
 * NEXTOR_LOG - the mailbox request log, on or off
 * ---------------------------------------------------------------------------
 * The log is two lines per request and one of them goes out BEFORE the
 * command is answered, which is the only reason a hang inside a handler is
 * visible at all. That makes it a debugging tool with a cost: a UART line
 * is a blocking write, on a path the Z80 is spinning on MB_POLL for. It
 * does not change what the protocol does, but it does change how long
 * answers take, and the thing being chased here is a timing-sensitive
 * overlap - so "does it still fail with logging off?" is a question worth
 * being able to ask without editing thirty call sites.
 *
 * So the switch shadows the printf identifier rather than #ifdef-ing every
 * call. One block, no call-site edits, and the two builds cannot drift
 * apart because there is only one place to change.
 *
 * Note this also compiles out the ARGUMENTS, which is deliberate: several
 * of them call into raw_disk.c (RawDisk_LastFailure and friends), and a
 * logging build that kept making those calls would not be the quiet build
 * it looks like.
 *
 * Override from the command line, e.g.
 *     make CFLAGS_EXTRA=-DNEXTOR_LOG=0
 * or by editing this to 0.
 */
#ifndef NEXTOR_LOG
#define NEXTOR_LOG 1
#endif

#if !NEXTOR_LOG
#undef printf
#define printf(...)    ((void)0)
#endif

/* ========================================================================
 * Wire contract
 * ====================================================================== */

/* driver.asm's sector size. The result FIFO is exactly one sector, so
 * 512 is both the driver's sector size and the largest answer a single
 * command can produce. raw_disk.h rejects a stick whose READ CAPACITY
 * block size is anything else, so the two can never disagree. */
#define NEXTOR_SECTOR_SIZE   512U

/* The 12-byte Nextor device parameter block: see the Device query 2
 * table in the Driver Development Guide, and the block that
 * NEXTOR_CMD_CAPACITY returns verbatim. */
#define NEXTOR_PARAMS_SIZE   12U

/* INQUIRY fields carried by NEXTOR_CMD_IDENT: vendor (8 bytes) followed
 * by product (20 bytes - 16 from INQUIRY, widened so the driver has a
 * fixed-width source for OUTPUT_STRING). Both space-padded by the
 * firmware, which is what lets the driver treat them as fixed-width
 * C strings without having to trim them itself. */
#define NEXTOR_IDENT_VENDOR  8U
#define NEXTOR_IDENT_MODEL   20U
#define NEXTOR_IDENT_SIZE    (NEXTOR_IDENT_VENDOR + NEXTOR_IDENT_MODEL)

/* Largest argument burst: WRITE = LBA(4) + one sector. READ's LBA is the
 * same four bytes. The value doubles as the bound for the arg sink. */
#define NEXTOR_ARG_MAX       (4U + NEXTOR_SECTOR_SIZE)      /* 516 */

/* Largest result burst: READ = one sector. CAPACITY (12) and IDENT (28)
 * are both well inside it. */
#define NEXTOR_RES_MAX       NEXTOR_SECTOR_SIZE            /* 512 */

/* Mailbox writes remembered for the invalid-command report. 24 is about
 * one HANDSHAKE plus its poll, which is the shortest span that can hold
 * an unexplained CMD write together with whatever it interrupted. */
#define NEXTOR_TRACE_N       24U

/* Read-trace geometry. Declared up here rather than beside the dump because
 * the ring is a field of s_mb, and the ring is written from IRQ context.
 *
 * N must stay a power of two - the read path indexes it with a mask, not a
 * modulo, because it is the hottest path in the module (MB_POLL spins on
 * STATUS here tens of thousands of times per slow answer). A handshake is
 * ~8 reads and a 12-byte params fetch ~15, so 32 holds every control
 * exchange whole; a 512-byte sector read does not fit and is not meant to,
 * and the dump says so rather than shortening the order string silently. */
#define NEXTOR_RDTRACE_N            32U
#define NEXTOR_RD_ORDER_N           28U

/* How long after publishing an answer the read trace waits before
 * declaring the exchange over. Long enough that a driver still spinning on
 * MB_POLL is not reported prematurely, short enough that a driver that has
 * given up is reported while the failure is still what is on screen. Only
 * the idle-path caller waits; the log_request caller knows the exchange is
 * already over and asking again is what makes that distinction. */
#define NEXTOR_READ_QUIET_MS        250U

/* Argument bytes each command pushes after its command byte.
 * Transcribed from driver.asm's command table. Indexed by command byte.
 *
 * The value doubles as the bound for the arg sink, which is what makes
 * an over-long burst impossible to overrun args[] with.
 *
 * Version 3 carries NO device byte: see nextor.h. A mismatch between
 * this table and driver.asm is not a silent corruption, because the
 * handshake carries NEXTOR_VERSION and the driver refuses to continue
 * unless the two agree - so a half-updated pair fails at boot rather
 * than answering with arguments in the wrong places. */
static const uint16_t s_cmd_args[NEXTOR_CMD_MAX] = {
    0U,                                       /* 0x00 HANDSHAKE */
    0U,                                       /* 0x01 CAPACITY  */
    0U,                                       /* 0x02 STATUS    */
    4U,                                       /* 0x03 READ:   LBA */
    (uint16_t)(4U + NEXTOR_SECTOR_SIZE),      /* 0x04 WRITE:  LBA + data */
    0U,                                       /* 0x05 ABORT     */
    0U,                                       /* 0x06 STAPEEK   */
    0U,                                       /* 0x07 IDENT     */
};

static const char *const s_cmd_name[NEXTOR_CMD_MAX] = {
    "HANDSHAKE", "CAPACITY", "STATUS", "READ",
    "WRITE", "ABORT", "STAPEEK", "IDENT",
};

/* Breadcrumb names, indexed by (command byte - NEXTOR_CMD_MARK_FIRST).
 *
 * This table is the firmware half of the MB_MK_* list in driver.asm, and
 * the two must stay in step: a marker printed as the wrong step is worse
 * than no marker, because it sends the reader looking in the wrong place.
 * The order below is the order of the equates in driver.asm.
 *
 * The names say WHERE the driver is; the argument says what value it was
 * looking at. Each comment says what the argument is for that step.
 *
 * The names are short on purpose. A boot log with a few hundred of these
 * is still readable; a boot log with a few hundred of a paragraph is not.
 * The interpretation lives in docs/NEXTOR_USER_GUIDE.md. */
/* NEXTOR_MARK_DEFINED counts the entries above; the reserved range is
 * deliberately wider (see NEXTOR_CMD_MARK_COUNT), so the difference has to
 * be bounded rather than assumed away. */
#define NEXTOR_MARK_DEFINED        30U

static const char *const s_mark_name[NEXTOR_CMD_MARK_COUNT] = {
    "RW:enter",              /* MB_MK_RW_ENTER       device number     */
    "RW:frame built",        /* MB_MK_RW_FRAME       sector count      */
    "RW:dir from C bit0",    /* MB_MK_RW_DIR         0=read 1=write    */
    "RW:medium present",     /* MB_MK_RW_MEDIUM      STATUS byte       */
    "RW:sector ok",          /* MB_MK_RW_SECTOR      LBA low byte      */
    "RW:all done",           /* MB_MK_RW_ALLDONE     sectors done      */
    "RW:exit .IDEVN",        /* MB_MK_RW_IDEVN       the device number */
    "RW:exit .NRDY",         /* MB_MK_RW_NRDY        the device number */
    "RW:exit .DISK",         /* MB_MK_RW_DISK        firmware err code */
    "RW:dev 0 -> 1",         /* MB_MK_RW_COERCE      the overridden no. */
    "DEVQ entered",          /* MB_MK_DEVQ           query index       */
    "DEVQ 2 get params",     /* MB_MK_DQ_PARAMS      H of HL           */
    "DEVQ 2 answered local", /* MB_MK_DQ_PARAMS_HL0  -                 */
    "DEVQ 2 via mailbox",    /* MB_MK_DQ_PARAMS_MB   -                 */
    "DEVQ 2 mailbox timeout",/* MB_MK_DQ_PARAMS_TO   -                 */
    "DEVQ 3 get status",     /* MB_MK_DQ_STATUS      device number     */
    "DEVQ 4 availability",   /* MB_MK_DQ_AVAIL       device number     */
    "DRVQ entered",          /* MB_MK_DRVQ           query index       */
    "RW:buf byte 0",        /* MB_MK_RW_BUF0        sector byte 0     */
    "RW:buf byte 511",      /* MB_MK_RW_BUFN        sector byte 511   */
    "RW:buffer addr low",   /* MB_MK_RW_BUFL        buffer address low */
    "RW:buffer addr high",  /* MB_MK_RW_BUFH        buffer address high*/
    "RW:MBR status",        /* MB_MK_RW_STAT        entry +446        */
    "RW:MBR type",          /* MB_MK_RW_TYPE        entry +450        */
    "RW:MBR start low",     /* MB_MK_RW_START       entry +454        */
    "RW:MBR sig low",       /* MB_MK_RW_SIGL        sector +510       */
    "RW:MBR sig high",      /* MB_MK_RW_SIGH        sector +511       */
    "RW:bad buffer low",    /* MB_MK_RW_BADBUFL     invalid buffer low */
    "RW:bad buffer high",   /* MB_MK_RW_BADBUFH     invalid buffer high*/
    "RW:buf high byte",     /* MB_MK_RW_BUFPG       dest page: 81h=DTA */
};

/* A command byte is a breadcrumb rather than a mistake if it lands in the
 * reserved marker range. Checked first, before the "invalid command"
 * branch, so a marker is never reported as a wild write: the two would be
 * indistinguishable in the log and only one of them is expected. */
static uint8_t is_mark (uint8_t cmd) {
    return (uint8_t)((cmd >= NEXTOR_CMD_MARK_FIRST)
                  && (cmd < (NEXTOR_CMD_MARK_FIRST
                             + NEXTOR_CMD_MARK_COUNT)));
}

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
    /* The command byte exactly as the Z80 wrote it, kept separately from
     * `cmd` because `cmd` is normalised: anything the driver sent that is
     * not one of the eight real commands is stored as NEXTOR_CMD_NONE so
     * the argument sink can be parked and the dispatch switch cannot be
     * reached with a value that is not a command. That normalisation throws
     * away precisely the byte a breadcrumb log needs - a marker would
     * arrive as 0x08, indistinguishable from a wild write, and the whole
     * point of the marker is to tell those two apart. One more byte in the
     * capture keeps the original. */
    volatile uint8_t  raw;       /* the byte as written, unnormalised  */
    volatile uint8_t  armed;     /* 1 = burst complete, answer pending */
    volatile uint8_t  seq;       /* requests captured, wraps at 256   */
    /* Command BYTES seen, not requests completed. seq is bumped when a
     * burst finishes, which makes it useless as a "has anything new
     * arrived?" test: for a READ there are five bus cycles between the
     * command byte and the moment seq moves, and for a WRITE there are 512
     * of them. Answering inside that window hands the driver the previous
     * sector under the new request's DONE, and nothing on either side can
     * see it - the driver has no request id to compare and the exchange
     * looks well formed in the log.
     *
     * `cap` is bumped the instant a command byte lands, so it covers the
     * whole capture including the part where armed is still 0. See the
     * publish check in Nextor_Service. */
    volatile uint8_t  cap;       /* command bytes captured, wraps 256  */

    /* --- result FIFO (Service writes, IRQ pops) --- */
    uint8_t * volatile res_r;    /* next byte the driver will pop     */
    uint8_t * volatile res_end;  /* one past the last produced byte   */

    /* --- STATUS register, precomputed (IRQ reads it every poll) --- */
    volatile uint8_t  status;

    /* --- answer / bookkeeping, Service only --- */
    uint8_t           err;       /* NEXTOR_ERR_*                      */
    uint8_t           last_seq;  /* seq of the last line we printed   */
    uint8_t           mbr_dumped;/* sector 0 already described once   */
    uint8_t           boot_dumped;/* partition boot sector once          */
    uint8_t           params_dumped; /* parameter block printed once  */

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

    /* --- Z80 read trace (IRQ reads, idle path prints) ---
     * tr_* above is the half of the exchange the driver SENT. These are
     * the half it got back, and they are the half that decides whether an
     * answer was accepted.
     *
     * The reason this exists: the driver prints its accept/reject verdict
     * to the MSX SCREEN, not the UART, so from the terminal an answer the
     * driver refused looks identical to an answer that never arrived. Both
     * show up as "one HANDSHAKE, then silence". These counters separate
     * them - a driver that popped DATA the right number of times read the
     * answer and rejected it on content; a driver with STAT reads but no
     * DATA pops never got past MB_POLL; no reads at all means it never
     * looked.
     *
     * Reset at publish, not at res_begin: the reads we care about all
     * happen AFTER the answer is released, so the window has to start
     * there. */
    volatile uint16_t rd_stat;   /* STATUS reads since the last publish */
    volatile uint16_t rd_data;   /* DATA pops    "        "             */
    volatile uint16_t rd_rxcnt;  /* RXCNT reads  "        "             */
    volatile uint16_t rd_err;    /* ERR reads    "        "             */
    volatile uint8_t  rd_last;   /* last STATUS value handed back       */
    volatile uint8_t  rd_seq;    /* seq of the request being reported   */
    volatile uint8_t  rd_done;   /* summary already printed for it      */
    volatile uint8_t  rd_active; /* a result has actually been published */
    volatile uint32_t rd_t0;     /* publish time, for the quiet window  */

    /* The ORDERED form of the four counters above, which is what actually
     * settles an ambiguous total. A count of "0 STATUS reads" means two
     * very different things - the driver never polled, or the poll that saw
     * DONE landed in the neighbouring request's window - and they need
     * opposite fixes. The ring shows the DONE-detecting read in sequence,
     * next to the DATA pops it must have preceded.
     *
     * A ring rather than a linear log because it cannot overrun: the 512
     * DATA pops of a sector read would need 512 slots, and the only thing
     * worth keeping from a long exchange is its tail. rr_n saturating at
     * the ring size is what tells the dump when it has lost the head. */
    volatile uint8_t  rr_reg [NEXTOR_RDTRACE_N];
    volatile uint8_t  rr_val [NEXTOR_RDTRACE_N];
    volatile uint8_t  rr_pos;    /* next slot to write                  */
    volatile uint8_t  rr_n;      /* reads seen, saturating at the size  */

    uint8_t           args[NEXTOR_ARG_MAX];
    uint8_t           res [NEXTOR_RES_MAX];
} s_mb;

/* Command byte that means "nothing latched". Also the count of real
 * commands, so an out-of-range byte parks the collector instead of
 * indexing s_cmd_args[] out of bounds. */
#define NEXTOR_CMD_NONE    NEXTOR_CMD_MAX

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
 * short enough to not be worth measuring twice.
 * ======================================================================== */

#if defined(__riscv)
static uint32_t rdcycle (void) {
    uint32_t c;
    __asm__ volatile ("rdcycle %0" : "=r" (c));
    return c;
}
#else
/* Host build (the unit tests). Cycle count is meaningless, so the
 * calibration below degenerates to "1 cycle per millisecond", which
 * only has to be monotonic and non-zero for the log to be readable. */
static uint32_t rdcycle (void) {
    static uint32_t fake;
    return ++fake;
}
#endif

extern void Delay_Ms (uint32_t n);

/* Core cycles per millisecond, measured by cyc_calibrate(). */
static uint32_t s_cyc_per_ms;

/* Cycle count of the most recent command byte, which is the moment the Z80
 * last asked for something. raw_probe() refuses to start a probe until the
 * Z80 has been quiet for NEXTOR_PROBE_IDLE_MS, so this is the gate on every
 * USB transaction the firmware makes on the kernel's behalf.
 *
 * Written from IRQ context, read from the main loop. rdcycle() is a single
 * instruction and a 32-bit aligned store is atomic on this core, so the
 * reader cannot see a torn value - and unlike ms_now()'s accumulators,
 * which are main-loop-only state, this touches nothing shared. */
static volatile uint32_t s_last_cmd_cyc;

static void cyc_calibrate (void) {
    const uint32_t t0 = rdcycle();

    Delay_Ms (4U);
    {
        const uint32_t d = rdcycle() - t0;
        s_cyc_per_ms = (d != 0U) ? (d / 4U) : 1U;
    }
}

/* ms_now() - monotonic milliseconds. Used both for the log's gaps and for
 * the probe rate limit, so it has to be right about two things at once.
 *
 * 1. NO DOUBLE DIVISION. An earlier version computed "now" in milliseconds
 *    and then divided the DELTA by s_cyc_per_ms a second time. With
 *    s_cyc_per_ms around 200000 that turns every gap shorter than 200
 *    seconds into "+0ms", and the only non-zero numbers that could ever
 *    appear were the ones where the (already-millisecond) subtraction went
 *    negative and wrapped. The log was not reporting gaps; it was
 *    reporting "less than 200 s" and "wrapped".
 *
 * 2. NO SAWTOOTH. rdcycle() is a free-running 32-bit counter that wraps
 *    every ~21 s at 200 MHz, so a raw cycle->millisecond conversion
 *    produces a ramp that restarts every 21 s. Anything built on it -
 *    "time since the last request", "how long since boot" - either goes
 *    backwards at the wrap or underflows. This counter accumulates
 *    forward instead, so it only wraps after 49 days. */
static uint32_t ms_now (void) {
    static uint32_t last_cyc;
    static uint32_t acc;
    static uint8_t  primed;
    const uint32_t now_cyc = rdcycle();

    if (primed == 0U) {
        primed   = 1U;
        last_cyc = now_cyc;
        acc      = 0U;
        return 0U;
    }

    {
        uint32_t d = now_cyc - last_cyc;

        /* A real gap longer than the counter's own period cannot be
         * recovered from the subtraction - the subtraction has already
         * wrapped and the result is a small number. Anything this big is
         * certainly a wrap, so charge the clamp (about 10.7 s) instead:
         * the exact value of an unmeasurable gap does not matter, and
         * reporting a wrapped delta would be a lie. */
        if (d >= 0x40000000U) {
            d = 0x40000000U;
        }
        last_cyc = now_cyc;
        acc += (s_cyc_per_ms != 0U) ? (d / s_cyc_per_ms) : 0U;
    }
    return acc;
}

/* Gap since the previous call, in the same units as ms_now(). The first
 * call has no previous reading to subtract from; take the reading and
 * report a zero gap rather than a wild one. */
static uint32_t ms_since_prev (void) {
    static uint32_t prev;
    static uint8_t  primed;
    const uint32_t now = ms_now();
    uint32_t       d;

    if (primed == 0U) {
        primed = 1U;
        prev   = now;
        return 0U;
    }
    d = now - prev;
    prev = now;
    return d;
}

/* ms_now() reading taken in Nextor_Init, used by raw_probe's boot
 * delay. 0 until then (so a host-side unit test that never calls
 * Nextor_Init is not delayed). */
static uint32_t s_boot_start_ms;

/* ========================================================================
 * 1b2. Probe policy (kept in nextor.c on purpose; see below).
 *
 * THE PROBE GUARD is the reliability fix for the freeze the old backend
 * shipped with. A probe that has to enumerate the stick runs hundreds
 * of milliseconds of retries (5 x USBH_PreDeal, each with an internal
 * bus-reset + descriptor dance) - far past the driver's ~0.8 s MB_POLL
 * budget. A probe that starts from inside CAPACITY/READ therefore blows
 * the Z80's command timeout on top of a request the kernel is actively
 * waiting on. The rule enforced everywhere:
 *
 *   - raw_probe may only run from the main loop, and only while NO
 *     request is in flight (Nextor_Service's idle path).
 *   - the command handlers NEVER probe (see the header note).
 *   - the first probe is also delayed until the kernel's boot-time
 *     command burst has settled (NEXTOR_PROBE_DELAY_MS from boot).
 *
 * ONE device means one rate limiter and nothing more. The expensive
 * part of a probe - USB enumeration - is now shared with everything
 * else on the stick rather than being repeated per file, and there is
 * no second device that could be held back by the first one's timing.
 *
 * RawDisk_Probe() is delegated the HOW (it calls USB_TryEnsureEnumerated,
 * READ CAPACITY and INQUIRY); this function owns only the WHEN, which is
 * Nextor-specific timing policy. Keeping the WHEN here means the rule
 * above is enforced in one place instead of being re-argued at each
 * call site.
 * ====================================================================== */
#define NEXTOR_PROBE_DELAY_MS      200U
#define NEXTOR_PROBE_RETRY_MS     2000U
#define NEXTOR_PROBE_IDLE_MS      250U

static uint32_t s_next_probe;       /* ms timestamp of the next probe */

static void raw_probe (void) {
    const uint32_t now = ms_now ();

    /* Never start a probe while the Z80 is talking. Checked FIRST, because
     * it is the strongest of the three rules and it is the one that was
     * missing.
     *
     * "Nextor_Service's idle path" is only idle one command at a time. A
     * probe is a USB enumeration attempt, and one of those runs hundreds
     * of milliseconds of retries - far past the driver's ~0.8 s MB_POLL
     * budget if a request happens to arrive mid-probe. The observed
     * failure is not a slow answer but pile-up: the Z80's retry arrives
     * while the probe still owns the bus, is not serviced either, and the
     * kernel's next request overwrites an unanswered one. That is what
     * "--- N request(s) dropped, service fell behind ---" in the log is,
     * and it makes the whole exchange sequence unreliable rather than
     * merely slow.
     *
     * A quarter-second of Z80 silence is the gate because the kernel's
     * boot-time burst is back-to-back HANDSHAKE/CAPACITY/STAPEEK a few
     * milliseconds apart, so that window never opens during init - which
     * is exactly when a probe does the most damage and the least good. It
     * does open between sessions, so a stick plugged in later is still
     * picked up. */
    if ((uint32_t)(rdcycle() - s_last_cmd_cyc)
        < (s_cyc_per_ms * NEXTOR_PROBE_IDLE_MS)) {
        return;
    }

    /* Boot delay: skip probes until NEXTOR_PROBE_DELAY_MS after the
     * boot anchor. The first probe otherwise lands well inside the
     * kernel's first CAPACITY burst, which is what used to wedge it. */
    if (s_boot_start_ms != 0U
        && (uint32_t)(now - s_boot_start_ms) < NEXTOR_PROBE_DELAY_MS) {
        return;
    }

    /* Already have a medium: nothing to do. Re-probing would reset the
     * media latch, which the kernel would then read as a media change
     * on every pass - i.e. forever - and would spend a USB enumeration
     * on each cycle while the kernel is actively reading. */
    if (RawDisk_IsPresent ()) {
        return;
    }

    /* Rate-limit the retries - otherwise a missing stick turns into a
     * continuous enumeration retry that pegs the USB host controller.
     *
     * has_probed() rather than an unconditional delay: a stick that has
     * never been tried must not be locked out by the rate limit an
     * earlier failed attempt established. */
    if (RawDisk_HasProbed ()
        && (uint32_t)(now - s_next_probe) < NEXTOR_PROBE_RETRY_MS) {
        return;
    }
    s_next_probe = now;
    (void)RawDisk_Probe ();
}

/* ========================================================================
 * 2. Lifecycle
 *
 * (Back to the mailbox itself. Sections 1a-1c above are the state and
 * the helpers the lifecycle and the service path share.)
 * ====================================================================== */

/* Stick removed: drop the cached capacity and the INQUIRY strings, so
 * the next idle pass re-probes whatever is now plugged in and the
 * kernel is told about the media change. Called from usb_disk.c's
 * disconnect path, which runs in the enumeration step that
 * USB_TryEnsureEnumerated() performs - so the caller is main-loop
 * context, not IRQ.
 *
 * The medium is the ONLY thing being cached, so there is no per-file
 * handle to forget and no choice about which of several backends to
 * drop. That is the whole point of the change of approach: the previous
 * version had to invalidate both image backends because a FIL handle
 * left open on a vanished volume answers with corrupt sectors rather
 * than with an error, and getting that wrong reads as data corruption
 * rather than as a missing stick. */
void Nextor_CacheInvalidate (void) {
    RawDisk_Invalidate ();
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
    /* Boot marker. The first thing to establish when Nextor DOS does not
     * come up is which half of the mailbox is missing, and this line is the
     * only proof that the firmware half is alive at all. With it:
     *
     *   this line, then nothing  -> the Z80 never wrote a command byte, so
     *                               the fault is in the cart window decode
     *                               or in the driver's detection
     *   this line, then [nx] #0  -> the mailbox works; whatever is wrong is
     *                               in the answers, and the STALE / dropped
     *                               lines below are the ones to read
     *   no line at all           -> Nextor_Init never ran, so the Nextor
     *                               mapper was never installed
     *
     * Printed after the state is set, so everything it claims is already
     * true. One line per mapper install, so a swap storm cannot flood the
     * terminal - and a flood here would itself stall the boot it is
     * meant to explain. */
    s_mb.argp    = s_mb.args;
    s_mb.argend  = s_mb.args;
    s_mb.res_r   = s_mb.res;
    s_mb.res_end = s_mb.res;
    s_mb.cmd     = NEXTOR_CMD_NONE;
    s_mb.raw     = NEXTOR_CMD_NONE;
    s_mb.armed   = 0U;
    s_mb.seq     = 0U;
    s_mb.cap     = 0U;
    s_mb.status  = NEXTOR_STAT_READY;
    s_mb.err     = NEXTOR_ERR_NONE;
    s_mb.last_seq = 0U;
    /* "Print once" latches, so a mapper swap or a soft reset gets a fresh
     * description rather than staying silent about a medium that has since
     * been re-probed. */
    s_mb.mbr_dumped    = 0U;
    s_mb.params_dumped = 0U;
    s_mb.rd_stat  = 0U;
    s_mb.rd_data  = 0U;
    s_mb.rd_rxcnt = 0U;
    s_mb.rd_err   = 0U;
    s_mb.rd_last  = 0U;
    s_mb.rd_seq   = 0U;
    s_mb.rd_done  = 0U;
    s_mb.rd_active = 0U;
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
    /* Start the probe-idle window CLOSED. The Z80 starts talking the
     * instant the mapper swap completes, and the window has to already be
     * shut when it does - otherwise the window is "250 ms since
     * power-on", which has long since expired, and the first probe is free
     * to land in the middle of the kernel's first HANDSHAKE. */
    s_last_cmd_cyc = rdcycle ();
    s_next_probe    = 0U;
    /* DELIBERATELY NOT RawDisk_Init() here.
     *
     * The comment this replaces said the right thing and the code
     * underneath it did the opposite: "a mapper swap needs no cleanup
     * beyond the mailbox state above - there is nothing to close". But
     * RawDisk_Init() throws away the enumeration, INQUIRY strings and
     * capacity that the previous probe had already paid for, so every
     * mapper swap put the medium back to "unknown".
     *
     * The medium is a property of the USB bus, not of the cart mapper.
     * Nothing about swapping the Z80's view of the cartridge changes
     * whether a stick is plugged in, so a swap has no business
     * invalidating it. It did, and the cost was not subtle: with the
     * cache empty, the first idle pass after the swap had to run a full
     * USB enumeration + READ CAPACITY + INQUIRY, which is hundreds of
     * milliseconds, and it ran it while the Z80 was already asking for
     * its first HANDSHAKE. Every session therefore began with requests
     * being dropped ("--- N request(s) dropped, service fell behind ---")
     * and the first few HANDSHAKE/CAPACITY/STAPEEK answers missing or
     * answering the wrong request - a protocol desync, not just a slow
     * start, and it makes the log unreliable at exactly the point where
     * it is most needed.
     *
     * RawDisk_Init() is a power-on call and now lives in main(). The
     * invalidation path for a stick that really does go away is
     * Nextor_CacheInvalidate(), which the USB disconnect path calls. */

    /* The boot marker, printed last so that everything it asserts is
     * already true - including the 4 ms calibration, which is why this is
     * at the end of the function and not at the top. */
    printf ("[nx] mapper installed: mailbox READY, probe window CLOSED,"
            " t=+%lums\r\n", (unsigned long)s_boot_start_ms);
}

/* LBA argument, little-endian - MB_SEND pushes the driver's (DE) block
 * verbatim, and driver.asm documents READ/WRITE as "LBA LE(4)".
 *
 * args[0..3], with no device byte in front of it (protocol v3). */
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

/* Publish the result FIFO and release the driver - but only if `cap` is
 * still the generation this answer was built for.
 *
 * The check and the DONE store are one transaction on purpose. The only
 * writer that can invalidate an answer is the interrupt handler, which bumps
 * `cap` on a command byte; masking IRQs across the comparison and the store
 * means no command byte can land between them. s_mb.res_end must also become
 * visible before DONE, or MB_POLL can return and MB_GETRES can read against
 * a stale end pointer.
 *
 * Out: 1 = published, 0 = a newer request was captured and this answer is
 * stale; the caller must answer that one instead. Never both, never neither.
 *
 * Why the generation is checked at all, and why a stale answer is not simply
 * thrown away, is spelled out at the call site in Nextor_Service. */
static uint8_t res_publish (uint8_t cap) {
    uint8_t published;

    /* DONE is the acquire point for the Z80: once it sees DONE, the cart
     * interrupt may immediately serve DATA reads. Make the result bytes,
     * res_r/res_end, and ERR visible before publishing that status bit on
     * the memory-mapped mailbox. Without an explicit RISC-V fence the
     * second identical READ can observe DONE while still seeing the old
     * FIFO state, even though the service-side log already shows 512 bytes.
     */
    __asm__ volatile ("fence rw, rw" ::: "memory");

    __disable_irq ();
    if (s_mb.cap != cap) {
        __enable_irq ();
        return 0U;
    }

    /* The status byte is built in a register and stored ONCE. DONE, ERR and
     * RX_AVAIL are all read by the driver out of the same byte, and
     * separate read-modify-writes would open a window in the middle of the
     * update where STATUS reads as something that means nothing. The mask
     * above is what stops the interrupt handler's own write of the same byte
     * racing this one - not the store, which is already single.
     *
     * DONE and ERR are deliberately the only bits touched. Nothing here
     * clears DONE on the way out, and res_begin() owns RX_AVAIL: the request
     * this answer belongs to may already have been superseded (which is what
     * the cap check above is about), and clearing its status bits on the way
     * out would destroy the state of a request this pass is not answering.
     *
     * This is also where the read trace's window opens. Everything the Z80
     * reads from here on belongs to THIS answer, which is the only
     * attribution that makes the counters meaningful - the driver reads
     * STATUS both before and after the publish, and only the latter half
     * tells us whether it saw the DONE just raised. */
    {
        uint8_t status = (uint8_t)(s_mb.status
                                   & (uint8_t)~NEXTOR_STAT_RX_AVAIL);

        if (s_mb.res_end != s_mb.res_r) {
            status |= NEXTOR_STAT_RX_AVAIL;
        }
        if (s_mb.err != NEXTOR_ERR_NONE) {
            status |= NEXTOR_STAT_ERR;
        }
        status |= NEXTOR_STAT_DONE;

        s_mb.rd_stat  = 0U;
        s_mb.rd_data  = 0U;
        s_mb.rd_rxcnt = 0U;
        s_mb.rd_err   = 0U;
        s_mb.rd_last  = 0U;
        s_mb.rr_pos   = 0U;
        s_mb.rr_n     = 0U;
        s_mb.rd_seq   = s_mb.seq;
        s_mb.rd_done  = 0U;
        s_mb.rd_active = 1U;
        s_mb.rd_t0    = ms_now ();

        s_mb.status = status;
    }
    published = 1U;
    __enable_irq ();

    return published;
}

/* How many times one Nextor_Service() pass may rebuild its answer because
 * the driver moved on mid-answer. See the call site. */
#define NEXTOR_ANSWER_REBUILDS      2U


/* Report what the Z80 read back from the last published answer.
 *
 * This is the read half of the exchange, and it is the half that decides
 * whether an answer was ACCEPTED. The driver announces its verdict on the
 * MSX screen, so from the terminal "the driver rejected the answer" and
 * "the answer never arrived" are the same observation: one request logged,
 * then nothing. These counters are what tells them apart:
 *
 *   STAT=0                  the Z80 never polled the mailbox again. It is
 *                           not waiting for anything - it has given up on
 *                           the driver, or the answer was published for a
 *                           request it had already abandoned.
 *   STAT>0, DATA=0          MB_POLL ran and never saw DONE. A timeout, so
 *                           the exchange died inside the wait.
 *   STAT>0, DATA==expected  the answer WAS read, and rejected on content.
 *                           That is a protocol/wrong-value fault, not a
 *                           timing one, and rd_last says what it saw.
 *   DATA>expected           MB_RESOLVE drained residue - a leftover from an
 *                           earlier request, i.e. the mailbox was already
 *                           desynchronised before this one started.
 *
 * Called from two places, which differ in what they can guarantee:
 *
 *   read_summary (0) - from log_request, i.e. when the NEXT command byte
 *       has landed. The previous exchange is definitively over, so no wait
 *       is needed and none is wanted: back-to-back requests are 0 ms apart,
 *       so a quiet window here would never expire and this would silently
 *       never print anything.
 *   read_summary (1) - from the idle path, where there may be no next
 *       request at all. Here the wait is the whole point: reporting the
 *       moment the answer was published would show zero reads for a driver
 *       that is merely still thinking.
 *
 * rd_done makes the two callers idempotent: whichever gets there first
 * reports it, once.
 *
 * One shot per answer (rd_done), so a driver that is still polling produces
 * one line rather than a heartbeat. */
static void read_summary (uint8_t wait_quiet) {
    const uint8_t  seq   = s_mb.rd_seq;
    const uint16_t stat  = s_mb.rd_stat;
    const uint16_t data  = s_mb.rd_data;
    const uint16_t rxcnt = s_mb.rd_rxcnt;
    const uint16_t err   = s_mb.rd_err;
    const uint8_t  left  = (uint8_t)(s_mb.res_end != s_mb.res_r);
    /* READ and WRITE are the only high-frequency commands - a directory
     * scan issues thousands of them - and for those a line of bookkeeping
     * per request would both bury the anomalies and lengthen the answer the
     * Z80 is waiting on. So they report only when something is actually
     * wrong. The control and boot commands are rare and are exactly where
     * "read the answer and refused it" has to be visible. */
    const uint8_t frequent = ((s_mb.raw == NEXTOR_CMD_READ)
                           || (s_mb.raw == NEXTOR_CMD_WRITE)) ? 1U : 0U;

    if (s_mb.rd_active == 0U || s_mb.rd_done != 0U) {
        return;
    }
    if ((wait_quiet != 0U)
        && ((uint32_t)(ms_now () - s_mb.rd_t0) < NEXTOR_READ_QUIET_MS)) {
        return;
    }
    /* Silence and a stranded answer are always worth a line. Otherwise the
     * report is worth one only for a rare command, or when the driver read
     * the answer without consuming it. */
    if ((frequent != 0U)
        && ((stat | data | rxcnt | err) != 0U)
        && (left == 0U)) {
        return;
    }
    s_mb.rd_done = 1U;

    if ((stat | data | rxcnt | err) == 0U) {
        printf ("[nx] reads after #%u: NONE - the Z80 never polled, so it"
                " was not waiting on this answer at all%s\r\n",
                (unsigned)seq, (left != 0U)
                    ? "  *** UNREAD ANSWER LEFT IN THE FIFO ***" : "");
        return;
    }

    printf ("[nx] reads after #%u: STAT=%u (last 0x%02x) DATA=%u RXCNT=%u"
            " ERR=%u%s\r\n",
            (unsigned)seq, (unsigned)stat, (unsigned)s_mb.rd_last,
            (unsigned)data, (unsigned)rxcnt, (unsigned)err,
            (left != 0U) ? "  *** ANSWER NEVER FULLY READ ***" : "");

    /* ...followed by the order, which is the part the counters cannot give.
     *
     * One register letter per read, so the sequence that matters reads
     * directly: `SDDDDDDDDDDDDXX` is a driver that polled once, saw DONE,
     * and drained - the shape every healthy exchange has. What is NOT that
     * shape is the finding: a leading run of S with no D (polled, never
     * drained), a D with no preceding S (drained an answer it never waited
     * for), or a lone D where several were published (popped residue).
     *
     * Truncated to NEXTOR_RD_ORDER_N and the loss is stated rather than
     * hidden: a 512-byte sector read cannot fit, and a silently shortened
     * order string would read as a complete one. */
    if (s_mb.rr_n != 0U) {
        char      ord [NEXTOR_RD_ORDER_N + 1U];
        uint8_t   i;
        uint8_t   n   = s_mb.rr_n;
        uint8_t   k   = 0U;
        uint8_t   cut = 0U;

        if (n > NEXTOR_RDTRACE_N) {
            n = (uint8_t)NEXTOR_RDTRACE_N;
        }
        /* Where the oldest surviving entry is.
         *
         * NOT simply rr_pos. That is only right once the ring has wrapped:
         * with 8 reads in a 32-slot ring rr_pos is 8, and starting there
         * would begin at slot 8 - the next slot to be written, holding
         * whatever the previous exchange left - and run off the end into
         * stale entries. Unwrapped, the oldest entry is slot 0. Wrapped,
         * it is slot rr_pos, because that is the one about to be
         * overwritten. */
        const uint8_t start = (s_mb.rr_n >= NEXTOR_RDTRACE_N)
                              ? s_mb.rr_pos : 0U;

        for (i = 0U; i < n; i++) {
            const uint8_t idx = (uint8_t)((start + i)
                                          & (NEXTOR_RDTRACE_N - 1U));
            const uint8_t r   = s_mb.rr_reg [idx];

            if (k >= NEXTOR_RD_ORDER_N) {
                cut = 1U;
                break;
            }
            ord [k++] = (r == NEXTOR_MBOX_DATA)          ? 'D'
                      : (r == NEXTOR_MBOX_CMD)           ? 'S'
                      : (r == NEXTOR_MBOX_RXCNT_LO)      ? 'X'
                      : (r == NEXTOR_MBOX_RXCNT_HI)      ? 'x'
                      : (r == NEXTOR_MBOX_ERR)           ? 'E'
                      : (r == NEXTOR_MBOX_VER)           ? 'V'
                                                         : '?';
        }
        ord [k] = '\0';
        printf ("[nx] rds #%u n=%u%s: %s%s\r\n",
                (unsigned)seq, (unsigned)s_mb.rr_n,
                (s_mb.rr_n >= NEXTOR_RDTRACE_N) ? "(head lost)" : "",
                ord, cut ? "..." : "");
    }
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

/* ========================================================================
 * MBR decoding
 *
 * One sector, printed once, and it is the highest-value line the firmware
 * can produce when a drive refuses to appear.
 *
 * A Nextor drive that is automapped and boots goes: scan sector 0 for an
 * MBR, walk the partition entries, read the volume boot sector of each
 * plausible one, and mount the first FAT volume it recognises. Three
 * things in that chain can fail while EVERY sector read still succeeds:
 *
 *   - no 55AA signature, because the stick was never partitioned (a
 *     "superfloppy", formatted by a tool that put a FAT volume straight at
 *     sector 0 with no MBR at all). This is the single most common
 *     arrangement for a USB stick and it is invisible in a log that only
 *     counts successful reads.
 *   - a signature with four partition entries that are all zero, i.e. an
 *     MBR-shaped sector 0 that is actually a FAT boot record.
 *   - partitions whose type byte Nextor will not mount (0x07 NTFS, 0x0B
 *     or 0x0C FAT32) or whose active flag is clear, so the kernel walks
 *     past them and finds nothing.
 *
 * Each of those prints one line here instead of costing a bisect on real
 * hardware. The decode is deliberately dumb - raw bytes plus the few
 * fields the kernel's own MBR scan (partit.mac, F_GPART_INNER /
 * CDP_GPART) actually tests for - because the point is to answer "what did
 * the medium return", not to reimplement the partition scanner.
 *
 * Partition types worth naming in the log, from the kernel's own
 * PT_* equates: 01/04 FAT12/FAT16 <32M, 05 extended, 06 FAT16, 0E FAT16
 * LBA, 07 NTFS/exFAT, 0B/0C FAT32, 0F extended LBA. A stick formatted by
 * Windows as exFAT/NTFS comes back 07, and that is a "this will never
 * mount in Nextor" rather than a bug worth chasing.
 * ====================================================================== */
static void dump_mbr (const uint8_t *sec) {
    /* Indexed by (type & 0x0F), the low nibble of the MBR type byte - the
     * high nibble is the "LBA" flag and Nextor looks at the low one. The
     * 16 entries are exhaustive on purpose: a stick formatted as exFAT
     * comes back as 07, and recognising that by name is the difference
     * between "Nextor cannot mount this" and "there is a bug somewhere". */
    static const char *const type_name[16] = {
        "empty",      "FAT12",   "FAT16<32M", "FAT16<32M",
        "FAT16",      "extended", "FAT16",     "NTFS/exFAT",
        "?",          "FAT32",   "FAT32-LBA", "?",
        "?",          "?",       "?",         "FAT16-LBA"
    };
    const uint16_t sig = (uint16_t)(sec[510] | ((uint16_t)sec[511] << 8));
    uint8_t       part;
    uint8_t       usable = 0U;

    printf ("[nx] MBR lba=0: %02x %02x %02x %02x  sig=%04x%s\r\n",
            (unsigned)sec[0], (unsigned)sec[1], (unsigned)sec[2],
            (unsigned)sec[3], (unsigned)sig,
            (sig == 0xAA55U) ? "" : "  *** NOT A VALID MBR ***");

    for (part = 0U; part < 4U; part++) {
        const uint8_t *e   = &sec[446U + (uint16_t)(part * 16U)];
        const uint8_t  typ = e[4];
        const uint32_t lba = (uint32_t)e[8] | ((uint32_t)e[9] << 8)
                           | ((uint32_t)e[10] << 16) | ((uint32_t)e[11] << 24);
        const uint32_t cnt = (uint32_t)e[12] | ((uint32_t)e[13] << 8)
                           | ((uint32_t)e[14] << 16) | ((uint32_t)e[15] << 24);
        const uint8_t  boot_flag = (uint8_t)(e[0] & 0x80U);

        if (typ != 0U) {
            usable++;
        }
        printf ("[nx]   part%u type=%02x %-10s boot=%s lba=%-10lu sectors=%-10lu",
                (unsigned)part, (unsigned)typ,
                type_name[typ & 0x0FU],
                boot_flag ? "yes" : "no ",
                (unsigned long)lba, (unsigned long)cnt);
        if (typ == 0U) {
            printf ("  (empty)\r\n");
        } else if ((typ == 0x01U) || (typ == 0x04U) || (typ == 0x06U)
                   || (typ == 0x0EU)) {
            /* The four types partit.mac's IS_SUITABLE_PART_TYPE accepts.
             * Counted separately because a stick with only these, but
             * with the active flag clear, is the other way the kernel
             * finds nothing to mount. */
            printf ("  *** bootable, but %s ***\r\n",
                    boot_flag ? "mountable" : "NOT ACTIVE");
        } else if ((typ == 0x05U) || (typ == 0x0FU)) {
            printf ("  (extended - needs the inner scan)\r\n");
        } else {
            printf ("  *** not mountable by Nextor ***\r\n");
        }
    }

    if (sig != 0xAA55U) {
        printf ("[nx] MBR lba=0: sector 0 has no 55AA signature. Nextor cannot"
                " automap this stick as-is - it needs a real MBR with a"
                " bootable FAT12/16 partition (the B key in the terminal"
                " menu writes one).\r\n");
    } else if (usable == 0U) {
        printf ("[nx] MBR lba=0: signature present but all four partition"
                " entries are empty.\r\n");
    }
}

/* ========================================================================
 * Request logging
 *
 * TWO LINES PER REQUEST, on purpose.
 *
 * The previous version printed one line, after the command had been
 * answered. That is useless for exactly the failure it was written to
 * diagnose: if answering the request never returns - a hang inside the
 * sector transfer, a wedged USB host controller, a bus fault in the
 * firmware - the single line is never printed at all, and the log ends
 * with nothing after the last command that DID work. The evidence that
 * matters most is the evidence that is missing.
 *
 * So the request line goes out first, before the handler runs:
 *
 *   [nx] #7 READ args=4 lba=0x00000000  +0ms      <- about to answer this
 *   [nx] #7    -> res=512  +4ms                  <- answered, this long
 *
 * A request line with no result line is then an unambiguous statement
 * that the firmware stopped inside that command, and the LBA on it says
 * which sector it stopped on.
 *
 * Both lines come from one snapshot taken with interrupts disabled, at
 * the top of the handler, BEFORE res_publish() - res_publish sets DONE
 * and the Z80 can return the instant DONE is visible and start the next
 * burst, whose first byte runs Nextor_WriteByte() and overwrites
 * args[]. Reading a field even a few instructions after the publish
 * describes the NEXT request.
 * ====================================================================== */
typedef struct {
    uint8_t  cmd;                       /* s_mb.cmd, normalised           */
    uint8_t  raw;                       /* s_mb.raw, the byte as written   */
    uint8_t  rawarg;                    /* args[0]: a breadcrumb's value   */
    uint8_t  seq;                       /* s_mb.seq */
    uint8_t  err;                       /* s_mb.err */
    uint16_t nres;                      /* result FIFO length */
    uint16_t nargs;                     /* argument count of THIS request */
    uint32_t lba;                       /* arg_lba(), valid for READ/WRITE */
    uint8_t  media;                     /* s_mb.res[0], the STATUS byte */
} log_snap;

/* The request half: printed before the command is answered. */
static void log_request (const log_snap *s) {
    /* The previous answer's read side is finished by now - the driver has
     * moved on and sent a new command byte - so this is the second of the
     * two places the read summary can be reported from. The idle path is
     * the better-timed of them (it also catches a driver that went quiet
     * for good and never came back), but the idle path only runs while the
     * mailbox is idle, and a driver in the middle of a bulk read never
     * leaves it idle long enough. rd_done makes the two callers
     * idempotent: whichever arrives first reports it, once.
     *
     /* Deliberately BEFORE the dropped-request check below: that check is
     * about writes and this is about reads, so neither line can mask the
     * other, and the read verdict is the one that explains a driver which
     * saw every answer and still went somewhere else. */
    read_summary (0U);

    (void)s_mark_name; /* marker names are disabled during timing tests */
    const uint8_t  cmd  = s->cmd;
    const uint8_t  raw  = s->raw;
    const uint8_t  seq  = s->seq;
    const uint16_t nargs = s->nargs;
    const uint32_t lba  = s->lba;

    if (is_mark (raw)) {
        /* Markers are diagnostic traffic, not requests. They must not
         * contribute to request-drop accounting. The driver's probe
         * markers (index 18 and up) ARE printed: they carry the values
         * the Z80 actually received, which is the only way to compare
         * the drained bytes against what raw_disk.c read. The old
         * fire-and-forget storm (indices 0-17) stays silent - those
         * call sites are compiled to no-ops in the driver anyway. */
        {
            const uint8_t idx = (uint8_t)(raw - NEXTOR_CMD_MARK_FIRST);
            if (idx >= 18U) {
                printf ("[nx]   . %-22s = %3u (0x%02x)\r\n",
                        (idx < NEXTOR_MARK_DEFINED)
                            ? s_mark_name[idx] : "(unnamed)",
                        (unsigned)s->rawarg, (unsigned)s->rawarg);
            } else if (idx == 1U) {
                printf ("[nx]   . RW:sectors requested = %3u (0x%02x)\r\n",
                        (unsigned)s->rawarg, (unsigned)s->rawarg);
            }
        }
        s_mb.last_seq = seq;
        return;
    }

    /* One request, one pair of lines. If seq moved by more than one, a
     * real request was overwritten before Nextor_Service reached it. */
    if ((uint8_t)(seq - s_mb.last_seq) > 1U) {
        printf ("[nx] --- %u request(s) dropped, service fell behind ---\r\n",
                (unsigned)((uint8_t)(seq - s_mb.last_seq) - 1U));
    }
    s_mb.last_seq = seq;

    if (cmd >= NEXTOR_CMD_MAX) {
        printf ("[nx] #%u <invalid cmd 0x%02x> last Z80 writes:\r\n",
                (unsigned)seq, (unsigned)raw);
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
    /* Gap since the previous request. The first line reads how long the
     * Z80 took to get from the driver's banner to its first mailbox
     * request - the boot stall plus the screen print, so it doubles as a
     * check on the stall. */
    printf ("  +%ums\r\n", (unsigned)ms_since_prev());
}

/* The result half: printed after the answer is complete. */
static void log_result (const log_snap *s) {
    if (is_mark (s->raw)) {
        return;                 /* a breadcrumb has no answer to print */
    }
    if (s->cmd >= NEXTOR_CMD_MAX) {
        return;                 /* trace_dump already said everything */
    }

    printf ("[nx] #%u    ->", (unsigned)s->seq);
    if (s->err != NEXTOR_ERR_NONE) {
        printf (" ERR %u", (unsigned)s->err);
    }
    if (s->nres > 0U) {
        printf (" res=%u", (unsigned)s->nres);
    }
    if (s->cmd == NEXTOR_CMD_STATUS) {
        /* The media byte is the whole point of this command, and a bare
         * "res=1" in a wall of lines is not readable a hundred lines
         * later. */
        printf (" media=%u%s", (unsigned)s->media,
                (s->media == 2U) ? " (changed)" : "");
    }
    /* Gap since the request line: the firmware's own time for this
     * command, which is what a USB-side problem shows up in. */
    printf ("  +%ums\r\n", (unsigned)ms_since_prev());
}

/* Build the 12-byte Nextor device parameter block.
 *
 * This is the ONE place that decides what the kernel thinks the drive
 * is, and it is deliberately not a copy of what raw_disk.c guesses:
 * sector size and sector count are the medium's own numbers, and the
 * flags are a statement about how Nextor should treat it.
 *
 * Flags:
 *   bit 0 removable   set. A USB stick is removable media in Nextor's
 *                     sense, and setting it is what makes the kernel
 *                     poll "get device status" so a hot-unplug is
 *                     noticed instead of becoming read errors.
 *   bit 1 read only   clear. The stick is writable, and a device that
 *                     can be dynamically write-protected must not be
 *                     reported as read-only (Driver Development Guide,
 *                     device query 2).
 *   bit 2 floppy      CLEAR, and this is the load-bearing decision of
 *                     the whole change of approach. A floppy-flagged
 *                     device makes the kernel map a drive straight onto
 *                     sector 0 and skip the partition scan; the raw
 *                     stick has a partition table at sector 0 and must
 *                     go through that scan. Flagging it as a floppy is
 *                     what the previous NEXTOR.DSK device did, and it
 *                     is exactly what stopped the drive from working.
 *   bit 3 no automap  clear. The drive is automappable.
 *
 * No medium is a sector count of 0, NOT an error and NOT an empty
 * answer. Per the Driver Development Guide, "get device parameters"
 * returning a 0 total is a valid report of "nothing readable here",
 * and it is what keeps the device enumerated and retryable: a failed
 * query would make the kernel drop the device, and a drive that only
 * disappears because the stick was plugged in late would never come
 * back. Sector size is still 512 because the guide requires a block
 * device to report one, and the kernel refuses anything else outright. */
static void params_build (uint8_t *out) {
    uint16_t cyl  = 0U;
    uint8_t  head = 0U;
    uint8_t  spt  = 0U;
    uint32_t sectors = RawDisk_IsPresent () ? RawDisk_SectorCount () : 0U;
    uint16_t ssize    = RawDisk_SectorSize ();
    uint8_t  i;

    if (sectors != 0U) {
        RawDisk_Geometry (&cyl, &head, &spt);
    }

    out[0] = 0U;                                   /* device type: block  */
    out[1] = (uint8_t)(ssize & 0xFFU);
    out[2] = (uint8_t)(ssize >> 8);
    for (i = 0U; i < 4U; i++) {
        out[3U + i] = (uint8_t)(sectors >> (8U * i));
    }
    out[7] = 0x01U;                                 /* flags: removable    */
    out[8] = (uint8_t)(cyl & 0xFFU);
    out[9] = (uint8_t)(cyl >> 8);
    out[10] = head;
    out[11] = spt;
}

/* Copy a C string into a fixed-width, space-padded field.
 *
 * INQUIRY gives fixed-width space-padded fields and raw_disk.h already
 * trims them to NUL-terminated C strings; this goes back the other way,
 * because the driver needs a fixed-width source it can hand straight to
 * OUTPUT_STRING with an exact length. Truncation on the right is
 * correct for SCSI strings (space-padded, not NUL-padded) and a string
 * longer than the field cannot be represented anyway. */
static void ident_pad (uint8_t *out, const char *src, uint8_t width) {
    uint8_t n = 0U;

    while ((n < width) && (src[n] != '\0')) {
        out[n] = (uint8_t)src[n];
        n++;
    }
    while (n < width) {
        out[n] = (uint8_t)' ';
        n++;
    }
}

/* Main-loop service hook. Called from main()'s idle loop; never from
 * IRQ context. This is the only place blocking work and printing may
 * happen - the driver is spinning in MB_POLL on the DONE bit for the
 * whole of this function. */
void Nextor_Service (void) {
    /* How many times one pass may rebuild its answer because the driver
     * moved on while the answer was being built. Two is generous: each
     * rebuild means the driver's MB_POLL expired on the previous attempt,
     * which is milliseconds of Z80 spin per attempt. The bound is what makes
     * this terminate - without it a driver that keeps timing out would hold
     * the main loop here forever and starve every other service. */
    unsigned rebuild = NEXTOR_ANSWER_REBUILDS;

next_request:
    /* Nothing to answer, so use the time to get the medium ready. The
     * only slow work outside a command is enumeration + READ CAPACITY +
     * INQUIRY in RawDisk_Probe (driven from raw_probe); READs and
     * WRITEs go straight to SCSI on the command path.
     *
     * read_summary() lives here because this is the only place that runs
     * once the exchange is over. raw_probe() is called first so it cannot
     * report a stalled medium as a dead mailbox - a probe does not touch
     * the mailbox registers, so it would only delay the answer. */
    if (s_mb.armed == 0U) {
        (void)raw_probe ();
        read_summary (1U);
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
    /* Snapshot fields are kept as separate locals because the log needs
     * cmd / seq / nargs / lba after the answer path runs and those
     * fields are no longer stable (IRQ has been re-enabled and the Z80
     * is reading the response FIFO). */
    uint8_t  snap_seq;
    uint8_t  snap_raw;
    uint8_t  snap_arg;
    uint8_t  snap_cap;
    uint16_t snap_nargs;
    uint32_t snap_lba;
    log_snap snap;
    {
        __disable_irq ();
        s_mb.armed = 0U;
        cmd        = s_mb.cmd;
        snap_raw   = s_mb.raw;
        snap_seq   = s_mb.seq;
        snap_cap   = s_mb.cap;
        snap_nargs = (uint16_t)(s_mb.argend - s_mb.args);
        snap_lba   = arg_lba ();
        /* A breadcrumb's payload is args[0]. Read it in the same critical
         * section as everything else: a new command byte resets argp, so
         * reading it afterwards would race the next marker and report its
         * value instead. */
        snap_arg   = (snap_nargs > 0U) ? s_mb.args[0] : 0U;
        __enable_irq ();
    }

    snap.cmd   = cmd;
    snap.raw   = snap_raw;
    snap.rawarg = snap_arg;
    snap.seq   = snap_seq;
    snap.nargs = snap_nargs;
    snap.lba   = snap_lba;
    snap.err   = NEXTOR_ERR_NONE;
    snap.nres  = 0U;
    snap.media = 0U;

    res_begin ();

    /* Request line first, before anything that can take time or fail to
     * return. See the logging section for why. */
    log_request (&snap);

    switch (cmd) {
    case NEXTOR_CMD_HANDSHAKE: {
        /* "RNX3" + firmware version. Pure firmware state - already a
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

    case NEXTOR_CMD_CAPACITY:
        /* The whole Nextor parameter block, verbatim - the driver streams
         * these twelve bytes straight into the kernel's buffer for device
         * query 2, so there is no byte the two sides could assemble
         * differently. An absent medium is a 12-byte answer with a sector
         * count of 0, NOT an error: see params_build(). */
        params_build (s_mb.res);
        s_mb.res_end = s_mb.res + NEXTOR_PARAMS_SIZE;
        /* Print the block once. The kernel makes three separate decisions
         * from it - is the sector size 512, how many sectors are there, and
         * is this device automappable (bit 3 of the flags) - and if any of
         * them is wrong the drive simply never appears, with a log full of
         * perfectly successful CAPACITY exchanges and nothing showing the
         * bytes that were actually handed over. */
        if (s_mb.params_dumped == 0U) {
            s_mb.params_dumped = 1U;
            printf ("[nx] params: %02x %02x %02x %02x %02x %02x "
                    "%02x %02x %02x %02x %02x %02x\r\n",
                    (unsigned)s_mb.res[0],  (unsigned)s_mb.res[1],
                    (unsigned)s_mb.res[2],  (unsigned)s_mb.res[3],
                    (unsigned)s_mb.res[4],  (unsigned)s_mb.res[5],
                    (unsigned)s_mb.res[6],  (unsigned)s_mb.res[7],
                    (unsigned)s_mb.res[8],  (unsigned)s_mb.res[9],
                    (unsigned)s_mb.res[10], (unsigned)s_mb.res[11]);
            printf ("[nx]   type=%u secsize=%u sectors=%lu flags=%02x"
                    "%s%s%s  chs=%u/%u/%u\r\n",
                    (unsigned)s_mb.res[0],
                    (unsigned)(s_mb.res[1] | ((uint16_t)s_mb.res[2] << 8)),
                    (unsigned long)((uint32_t)s_mb.res[3]
                                  | ((uint32_t)s_mb.res[4] << 8)
                                  | ((uint32_t)s_mb.res[5] << 16)
                                  | ((uint32_t)s_mb.res[6] << 24)),
                    (unsigned)s_mb.res[7],
                    (s_mb.res[7] & 0x01U) ? " removable" : "",
                    (s_mb.res[7] & 0x04U) ? " FLOPPY(!)" : "",
                    (s_mb.res[7] & 0x08U) ? " no-automap" : "",
                    (unsigned)(s_mb.res[8] | ((uint16_t)s_mb.res[9] << 8)),
                    (unsigned)s_mb.res[10],
                    (unsigned)s_mb.res[11]);
        }
        break;

    case NEXTOR_CMD_IDENT:
        /* INQUIRY vendor + product, so Nextor's device screen names the
         * stick that is actually plugged in. Purely decorative: a stick
         * that never answered INQUIRY produces spaces, the driver prints
         * them, and nothing else changes. */
        ident_pad (&s_mb.res[0], RawDisk_Manufacturer (), NEXTOR_IDENT_VENDOR);
        ident_pad (&s_mb.res[NEXTOR_IDENT_VENDOR], RawDisk_Model (),
                   NEXTOR_IDENT_MODEL);
        s_mb.res_end = s_mb.res + NEXTOR_IDENT_SIZE;
        break;

    case NEXTOR_CMD_STATUS:
    case NEXTOR_CMD_STAPEEK:
        /* 0 = no medium, 1 = ready, 2 = ready and changed since the last
         * STATUS. The latch is what the "changed once, then ready"
         * tracking in the driver is built on: without it the kernel would
         * never notice that a stick appeared after boot.
         *
         * STAPEEK deliberately does not consume it - device query 4 must
         * not disturb query 3's tracking, or a media check in between two
         * status requests would swallow the change.
         *
         * No USB traffic and no probing here (see the header note):
         * STATUS is polled far more often than anything else, including
         * by the kernel's own startup code, and an enumeration retry in
         * the middle of it would blow the driver's command budget. */
        {
            const uint8_t st = RawDisk_MediaState ();

            if (cmd == NEXTOR_CMD_STATUS) {
                RawDisk_MediaLatchConsume ();
            }
            s_mb.res[0]  = st;
            s_mb.res_end = s_mb.res + 1U;
        }
        break;

    case NEXTOR_CMD_ABORT:
        /* Nothing to abort: no request is ever in flight here, because
         * Nextor_Service answers each one in a single pass. */
        break;

    case NEXTOR_CMD_READ:
        /* One sector straight off the USB medium into the result FIFO.
         * The buffer is exactly one sector long, so a read can never
         * overrun it, and the driver drains it as soon as DONE appears.
         *
         * "No medium" and "the transfer failed" are kept apart because
         * the driver turns them into different DOS errors (.NRDY versus
         * .DISK), and reporting a failed transfer as "no medium" would
         * make the kernel unmap a drive that is perfectly present.
         *
         * Every read failure names itself and says why. The request line
         * above already carries the LBA, so the result line only has to
         * say which of the three reasons it was - a bare "ERR 2" in a log
         * where the same LBA is retried every second is not a diagnosis,
         * it is a number. */
        if (RawDisk_IsPresent () == 0U) {
            s_mb.err = NEXTOR_ERR_NO_MEDIA;
            printf ("[nx]   no medium, LBA rejected\r\n");
            break;
        }
        if (RawDisk_ReadSectors (snap_lba, 1U, s_mb.res) == 0U) {
            s_mb.err = NEXTOR_ERR_IO;
            printf ("[nx]   read failed: %s (present=%u sectors=%lu)\r\n",
                    RawDisk_LastFailure (), (unsigned)RawDisk_IsPresent (),
                    (unsigned long)RawDisk_SectorCount ());
            break;
        }
        /* Keep the USB/backend result separate from the later mailbox
         * drain diagnostics. If this line is correct but RW:buf byte 0 is
         * not, the corruption is between res[] and the cart DATA port. If
         * this line is already wrong, the USB BOT transfer is the fault.
         *
         * Every read dumps its head and tail now, not just LBA 0 and
         * 2048: the driver's per-read probes report the bytes the Z80
         * received, and this line is the byte-for-byte reference to
         * compare them against. */
        {
            uint16_t qi;
            printf ("[nx]   raw lba=%lu:", (unsigned long)snap_lba);
            for (qi = 0U; qi < 4U; qi++) {
                printf (" %02x", (unsigned)s_mb.res[qi]);
            }
            printf ("  tail=%02x %02x\r\n",
                    (unsigned)s_mb.res[510], (unsigned)s_mb.res[511]);
        }
        if ((snap_lba == 2048U) && (s_mb.boot_dumped == 0U)) {
            s_mb.boot_dumped = 1U;
            printf ("[nx]   raw lba2048: %02x %02x %02x %02x "
                    "bps=%02x%02x spc=%02x fats=%02x root=%02x%02x "
                    "spf=%02x%02x sig=%02x%02x\r\n",
                    (unsigned)s_mb.res[0], (unsigned)s_mb.res[1],
                    (unsigned)s_mb.res[2], (unsigned)s_mb.res[3],
                    (unsigned)s_mb.res[11], (unsigned)s_mb.res[12],
                    (unsigned)s_mb.res[13], (unsigned)s_mb.res[16],
                    (unsigned)s_mb.res[17], (unsigned)s_mb.res[18],
                    (unsigned)s_mb.res[22], (unsigned)s_mb.res[23],
                    (unsigned)s_mb.res[510], (unsigned)s_mb.res[511]);
        }
        /* First successful read of sector 0 is the MBR, and the MBR is
         * the one sector whose contents decide whether Nextor can map
         * anything at all: no 55AA signature, or a partition table the
         * kernel rejects, and the drive never appears no matter how many
         * reads succeed. Print it once, so a log that shows thousands of
         * successful reads also shows WHY there is no drive. */
        if ((snap_lba == 0U) && (s_mb.mbr_dumped == 0U)) {
            s_mb.mbr_dumped = 1U;
            dump_mbr (s_mb.res);
        }
        s_mb.res_end = s_mb.res + NEXTOR_SECTOR_SIZE;
        break;

    case NEXTOR_CMD_WRITE:
        /* A real write to the medium. The sector is the tail of args[] -
         * args[0..3] the LBA, args[4..515] the 512 bytes - and is handed
         * to raw_disk.c where it lies, with no bounce copy:
         * NEXTOR_ARG_MAX is sized so the data ends exactly at the end of
         * the array, so &args[4] is a whole sector by construction and a
         * separate 512-byte buffer would be RAM for nothing.
         *
         * Nothing is produced in the answer - a successful write is a
         * 0-byte answer - so res_end stays where res_begin put it and the
         * driver sees DONE with no result to drain.
         *
         * No read-only case to check: the device is writable by design
         * and says so in its parameter block, so the kernel never sends
         * a write it expects to be refused. A refused write is a SCSI
         * error, and the honest answer for that is .DISK, not "no
         * media" - which is what keeps the kernel from unmounting a disk
         * that is present and merely had a write failure. */
        if (RawDisk_IsPresent () == 0U) {
            s_mb.err = NEXTOR_ERR_NO_MEDIA;
            printf ("[nx]   no medium, write rejected\r\n");
            break;
        }
        if (RawDisk_WriteSectors (snap_lba, 1U, &s_mb.args[4]) == 0U) {
            s_mb.err = NEXTOR_ERR_IO;
            printf ("[nx]   write failed: %s\r\n", RawDisk_LastFailure ());
            break;
        }
        break;

    default:
        /* A breadcrumb, or a byte the driver should never have sent.
         *
         * A breadcrumb is not a failure. It wants no answer, and the
         * driver that wrote it is already running the next instruction -
         * nothing is polling, so publishing an error is harmless today
         * and a real trap the moment a marker is put anywhere near a
         * poll. It gets DONE with no error and no result, which the next
         * real command overwrites. Its one argument byte has already been
         * logged by log_request by this point, so the empty result side
         * loses nothing. */
        if (is_mark (snap.raw)) {
            break;
        }
        /* A byte that is neither a command nor a breadcrumb. */
        s_mb.err = NEXTOR_ERR_IO;
        break;
    }

    /* Snapshot the answer, then release the driver, then print.
     *
     * The response fields (err, nres, media) are written only by us, with
     * IRQ re-enabled, and IRQ does not touch them in any way that
     * affects the answer (s_mb.err is read only by the Z80; s_mb.res[] is
     * popped by the Z80 too, never by IRQ). The only field IRQ does touch
     * is s_mb.res_r, on pop, and at most one byte may have been popped
     * between res_publish and the log call - that is a cosmetic one-byte
     * count error in the log line, never a correctness issue.
     *
     * res_publish() comes BEFORE the print deliberately: the driver is
     * spinning on DONE, and making it wait for a UART line to drain would
     * add the transfer time of the whole log line to every single command. */
    snap.err   = s_mb.err;
    snap.nres  = (uint16_t)(s_mb.res_end - s_mb.res_r);
    snap.media = s_mb.res[0];

    /* Publish, but only if this answer is still the answer the driver is
     * waiting for.
     *
     * The answer above was built with IRQ enabled and it contains a USB
     * transfer, so it takes milliseconds - long enough for the driver's
     * MB_POLL to expire and for the driver to abandon this request and send
     * a different one. The new command byte has already cleared DONE, so
     * raising DONE here publishes THIS answer under the NEW request: 512
     * bytes of sector where a one-byte STATUS was expected. The driver
     * reads the sector's first byte as the media state, gets something that
     * is neither 0, 1 nor 2, and the volume is reported as an unrecognised
     * command - with every exchange in the log looking like it worked.
     *
     * `cap` is the test, and not `armed`, because `armed` only goes up when
     * a burst *completes*. A WRITE's argument burst is 512 bus cycles long,
     * so for most of it the new request exists and armed is still 0; testing
     * armed would let exactly the answer that cannot be delivered slip
     * through. `cap` moves the instant a command byte lands, so it covers
     * the whole capture. See s_mb.cap.
     *
     * The check and the DONE store are one transaction with respect to the
     * only writer that can invalidate this answer - the interrupt handler,
     * which bumps `cap` - because res_publish() masks IRQs across its own
     * store. Nothing can slip in between.
     *
     * A stale answer is NOT simply dropped. Dropping it is what an earlier
     * attempt at this fix did, and it made the drive disappear entirely:
     * the request the driver is actually waiting for never got an answer,
     * MB_POLL expired on it too, and the mailbox was left claiming work it
     * had thrown away. Instead the pass restarts and builds the answer for
     * the request that is pending now, so the driver's next poll is served
     * immediately and correctly. Only if the rebuilds run out is the answer
     * deferred - and even then the request stays armed, so the next
     * main-loop pass picks it up. Nothing is ever dropped. */
    if (res_publish (snap_cap) == 0U) {
        printf ("[nx] #%u STALE cap=%u/%u - driver moved on, rebuilding%s\r\n",
                (unsigned)snap_seq, (unsigned)snap_cap, (unsigned)s_mb.cap,
                (rebuild == 0U) ? " (DEFERRED, no rebuilds left)" : "");
        if (rebuild == 0U) {
            return;         /* deferred, not lost: s_mb.armed is still set */
        }
        rebuild--;
        goto next_request;
    }
    log_result (&snap);
}

/* ========================================================================
 * 3. Code entries - called from Cart_EXTI0_Nextor_Handler.
 *
 * Both are reached only for addresses in NEXTOR_MBOX_BASE..
 * NEXTOR_MBOX_END, so the register index is just the A2..A4 slot
 * number and needs no further range check. Keeping the window test in
 * one place (cart.c) is what lets these two stay branch-light: a
 * comparison per register would be paid on every one of the ~520
 * interrupts a WRITE burst costs.
 * ====================================================================== */

void Nextor_WriteByte (uint16_t address, uint8_t value) {
    const uint8_t reg = NEXTOR_MBOX_REG (address);

    trace_put (reg, value);

    if (reg == NEXTOR_MBOX_CMD) {
        uint8_t cmd = value;

        /* DONE and ERR are read by the driver out of the same byte, so they
         * are cleared by one read-modify-write and move together. Clearing
         * them in two steps would open a window in which STATUS shows the new
         * request as already failed while it has not even been answered.
         *
         * ERR is cleared HERE and not in res_begin, because res_begin runs
         * from the main loop - a driver that read STATUS in between would
         * still see the previous request's failure. More to the point, ERR
         * used to be only ever set and never cleared by anything at all,
         * which is not a cosmetic latch: the driver tests MBST_ERR after
         * every READ and turns it into .DISK, so the first failed sector - a
         * transient USB stall, which a bulk FAT scan will eventually hit -
         * made every subsequent sector fail as well, and the drive stayed
         * dead for the rest of the session. A scan through the directory then
         * never completed, which is what this looked like from the outside.
         *
         * DONE must also fall here, not in Service: MB_POLL returns the
         * instant DONE is visible, so clearing it late would let the driver
         * read the PREVIOUS answer.
         *
         * `cap` moves on every command byte and is the staleness test in
         * res_publish(). It is here, at the instant of capture, and not at
         * arm time, so it also covers the whole of a 512-byte argument burst
         * during which armed is still 0. */
        s_mb.status &= (uint8_t)~(NEXTOR_STAT_DONE | NEXTOR_STAT_ERR);
        s_mb.cap++;

        /* First-contact marker, print once per mapper install. The request
         * log only starts at [nx] #0, which requires a completed burst; if
         * the Z80 writes command bytes but no request line ever follows,
         * the arguments are not being captured and the fault is in the
         * argument path, not in the answers. This line is the only way to
         * see that, and it is here because this is the only place that
         * knows a command byte landed at all.
         *
         * Once per install, not per command: this is IRQ context on the
         * hot path, and a per-command print would be both a timing hazard
         * and unreadable. `cap` is 1 exactly on the first one. */
        if (s_mb.cap == 1U) {
            printf ("[nx] first CMD byte from the Z80: 0x%02x%s\r\n",
                    (unsigned)value,
                    (cmd < NEXTOR_CMD_MAX) ? "" : "  *** NOT A VALID COMMAND ***");
        }

        if (cmd >= NEXTOR_CMD_MAX) {
            /* Park the sink at the empty range. The burst that follows
             * is of unknown length, and guessing would either overrun
             * args[] or arm on a short burst. */
            s_mb.cmd    = NEXTOR_CMD_NONE;
            /* EXCEPT for a breadcrumb, whose one payload byte is a known
             * length - see NEXTOR_MARK_ARGC. Without this the payload
             * write would be dropped by the overflow guard below and
             * every marker would log its argument as zero, which looks
             * exactly like "the value really was zero". */
            s_mb.argend = is_mark (cmd) ? (s_mb.args + NEXTOR_MARK_ARGC)
                                        : s_mb.args;
        } else {
            s_mb.cmd    = cmd;
            s_mb.argend = s_mb.args + s_cmd_args[cmd];
        }
        s_mb.raw   = cmd;
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
        /* The Z80 is talking. raw_probe() uses this to keep a USB
         * enumeration off the bus while the kernel is mid-conversation -
         * see the idle guard there for why that matters. */
        s_last_cmd_cyc = rdcycle();
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
    uint8_t v;

    switch (reg) {
    case NEXTOR_MBOX_CMD:
        /* Same index as NEXTOR_MBOX_STAT - the protocol is
         * write=command / read=status on 0x7FF0. Returned precomputed
         * so MB_POLL's spin is a single load.
         *
         * This is by far the most-read register: MB_POLL spins on it for
         * the whole answer, tens of thousands of times. The stores that
         * record it are the entire cost of the read trace and are worth
         * it - seeing the STATUS values in order, next to the DATA pops
         * they gate, is what identifies a driver polling a status that
         * never showed DONE. */
        s_mb.rd_stat++;
        v = s_mb.status;
        break;

    case NEXTOR_MBOX_DATA: {
        /* Result pop. The driver pops a known count, so this only has
         * to stay in range; an empty FIFO returns 0xFF (the
         * floating-bus pattern) so a driver that over-pops sees a
         * stable wrong value and bails instead of spinning forever. */
        uint8_t *p   = s_mb.res_r;
        uint8_t *end = s_mb.res_end;
        s_mb.rd_data++;
        if (p < end) {
            v = *p;
        } else {
            v = 0xFFU;
        }
        if (p < end) {
            s_mb.res_r = ++p;
            if (p == end) {
                s_mb.status &= (uint8_t)~NEXTOR_STAT_RX_AVAIL;
            }
        } else {
            v = 0xFFU;
        }
        break;
    }

    case NEXTOR_MBOX_RXCNT_LO:
        /* Informational only (driver.asm never reads it) but cheap to
         * keep honest. */
        s_mb.rd_rxcnt++;
        v = (uint8_t)(s_mb.res_end - s_mb.res_r);
        break;

    case NEXTOR_MBOX_RXCNT_HI:
        s_mb.rd_rxcnt++;
        v = (uint8_t)((s_mb.res_end - s_mb.res_r) >> 8);
        break;

    case NEXTOR_MBOX_ERR:
        s_mb.rd_err++;
        v = s_mb.err;
        break;

    case NEXTOR_MBOX_VER:
        v = NEXTOR_VERSION;
        break;

    default:
        /* Slots 6 and 7 (0x7FF8, 0x7FFC) are decoded by the handler but
         * unused by the protocol. */
        v = 0xFFU;
        break;
    }

    /* Ordered record, after the value is final - for DATA that means after
     * the FIFO has been popped, so the trace shows what was handed over
     * rather than what was in the slot beforehand.
     *
     * rr_pos is masked rather than taken modulo: the ring is a power of two
     * and this is the hottest path in the module (MB_POLL spins on STATUS
     * here tens of thousands of times per slow answer), so it is worth
     * saving the divide. rr_n saturating is what tells the dump that the
     * head of a long exchange was lost - it must never wrap, or the dump
     * would report a truncated exchange as a complete one. */
    {
        const uint8_t pos = s_mb.rr_pos;

        s_mb.rr_reg [pos] = reg;
        s_mb.rr_val [pos] = v;
        s_mb.rr_pos       = (uint8_t)((pos + 1U) & (NEXTOR_RDTRACE_N - 1U));
        if (s_mb.rr_n < NEXTOR_RDTRACE_N) {
            s_mb.rr_n++;
        }
    }
    return v;
}
