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
 * Not implemented yet
 * ---------------------------------------------------------------------------
 * There is still no disk backend, so the answers below are what a
 * firmware with no medium attached would report. HANDSHAKE, STATUS and
 * STAPEEK are answered from firmware state alone and are therefore
 * already correct; CAPACITY, READ and WRITE report NO_MEDIA, which is
 * the honest answer until the USB/SCSI path is wired in. The bank-
 * switching half of the mapper is complete, so the kernel ROM boots and
 * its driver runs, and the log below is the tool for watching exactly
 * which command the kernel wants next.
 */

#include "nextor.h"
#include <stdio.h>

/* ========================================================================
 * Wire contract
 * ====================================================================== */

/* driver.asm: "512-byte sectors, removable medium." */
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
 * 1. Lifecycle
 * ====================================================================== */

/* ========================================================================
 * 1b. Request timestamps.
 *
 * Why this exists: the Nextor kernel polls a drive it has mapped, and with
 * no disk backend behind CAPACITY the poll never settles - the log fills
 * with an endless CAPACITY/STATUS/STATUS cycle. Whether that is the
 * kernel's intended watchdog (once a second or so) or a tight retry spin
 * (which would peg a CPU and make the machine unusable) is not something
 * the log can show, because a list of lines with no intervals cannot
 * distinguish them. So each line carries the gap since the previous one.
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
static void log_request (void) {
    const uint8_t  cmd = s_mb.cmd;
    const uint8_t  seq = s_mb.seq;
    const uint8_t  err = s_mb.err;
    const uint16_t res = (uint16_t)(s_mb.res_end - s_mb.res_r);

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
        trace_dump ();
        return;
    }

    printf ("[nx] #%u %s args=%u", (unsigned)seq, s_cmd_name[cmd],
            (unsigned)(s_mb.argend - s_mb.args));
    if ((cmd == NEXTOR_CMD_READ) || (cmd == NEXTOR_CMD_WRITE)) {
        printf (" lba=0x%08x", (unsigned)arg_lba());
    }
    if (err != NEXTOR_ERR_NONE) {
        printf (" -> ERR %u", (unsigned)err);
    }
    if (res > 0U) {
        printf (" res=%u", (unsigned)res);
    }
    /* Gap since the previous request. First line reads "+0ms" and the
     * second reads "+<calibration>ms", which is how long the Z80 took to
     * get from the driver's banner to its first mailbox request - the
     * 5 s boot stall plus the screen print, so it doubles as a check on
     * the stall. */
    printf ("  +%ums\r\n", (unsigned)ms_since_prev());
}

/* Main-loop service hook. Called from main()'s idle loop; never from
 * IRQ context. This is the only place blocking work and printing may
 * happen - the driver is spinning in MB_POLL on the DONE bit for the
 * whole of this function. */
void Nextor_Service (void) {
    uint8_t cmd;

    /* State-change gate. armed is set exactly once per captured request
     * and cleared below, so an idle mailbox costs one load and a branch
     * - no printing, and in particular nothing per STATUS poll. */
    if (s_mb.armed == 0U) {
        return;
    }
    s_mb.armed = 0U;

    cmd = s_mb.cmd;
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
    case NEXTOR_CMD_STAPEEK:
        /* 0 = no media. Both report the same thing today because the
         * media-change latch has no producer yet; STAPEEK must not
         * consume the latch, which is why there is no consume-on-read
         * state to clear here. */
        s_mb.res[0]  = 0U;
        s_mb.res_end = s_mb.res + 1U;
        break;

    case NEXTOR_CMD_ABORT:
        /* Nothing to abort: no request is ever in flight here, because
         * Nextor_Service answers each one in a single pass. */
        break;

    case NEXTOR_CMD_CAPACITY:
    case NEXTOR_CMD_READ:
    case NEXTOR_CMD_WRITE:
    default:
        /* No disk backend yet. Reporting NO_MEDIA keeps the driver's
         * error path honest and lets it move on to the next command
         * instead of wedging in MB_POLL. */
        s_mb.err = NEXTOR_ERR_NO_MEDIA;
        break;
    }

    res_publish ();
    log_request ();
}

/* ========================================================================
 * 2. Code entries - called from Cart_EXTI0_Nextor_Handler.
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
