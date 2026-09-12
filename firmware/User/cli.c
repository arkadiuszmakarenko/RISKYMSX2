/*
 * Command-line interface over USART1. See cli.h for the protocol spec.
 *
 * RX path:
 *   USART1 RXNE IRQ -> CLI_USART1_Handler()
 *     - echo the byte
 *     - assemble into s_line[]
 *     - on \r or \n, dispatch via CLI_HandleLine()
 *
 * All work runs inside the USART1 IRQ, which has PFIC priority 0x40
 * (lower than EXTI0 at 0x00, so the cart IRQ always preempts us).
 *
 * Dispatched commands:
 *   PING / HELP / RST [ms] / SRC [SRAM|PSRAM]
 *   LOAD <hexaddr> <hexbytes> / XLOAD <hexaddr> <len> / DUMP <hexaddr> <len>
 *
 * LOAD / XLOAD / DUMP target the PSRAM cart image window (8 MiB at
 * 0x80000000). The active mapper (see Cart_SetMapper) interprets reads
 * from this same window.
 */

#include "cli.h"
#include "ch32v4x7.h"
#include "cart.h"
#include "psram.h"
#include "scc.h"
#include "debug.h"
#include <string.h>

/* Linear command buffer. */
#define CLI_LINE_SZ   96U
static char     s_line[CLI_LINE_SZ];
static uint16_t s_line_len = 0U;

/* Dispatch snapshot. When a line completes, the IRQ copies it here and
 * arms s_line_ready. s_line itself is immediately reusable for the
 * next command, so characters typed while the main loop is still
 * dispatching the previous command can NEVER overwrite a pending line
 * (the pre-snapshot design had exactly that race). At most one
 * undispatched command can be pending - the CRLF suppression keeps a
 * terminator pair from arming twice, and the prompt-synced host tools
 * never send a new command before the prompt anyway. */
static char     s_dispatch_buf[CLI_LINE_SZ];

/* XLOAD state: after the XLOAD header line is dispatched (main loop),
 * the next <len> raw bytes are appended to the ACTIVE image window at
 * <addr>.
 *
 * Note on the old swallow-first logic: when dispatch ran inside the
 * USART IRQ, the header's trailing '\n' could arrive after the pump
 * armed and had to be swallowed. With dispatch deferred to the main
 * loop, the line assembler consumes BOTH terminator bytes ('\r' arms
 * the line, the CRLF-suppression skips the '\n') before the pump ever
 * arms - so every byte the pump sees is real data. No swallow needed. */
static uint8_t  s_xload_active    = 0U;
static uint32_t s_xload_addr      = 0U;
static uint32_t s_xload_remaining = 0U;

/* ------------------------------------------------------------------ */
/* Active cart image window.                                          */
/* ------------------------------------------------------------------ */

/* The window is always PSRAM (8 MiB). LOAD/XLOAD/DUMP operate on the
 * PSRAM image; the cart handlers serve from the same PSRAM window.
 * Cart_SetMapper() picks which mapper interprets the reads. */
static uint8_t *CLI_GetWindow (void) {
    return (uint8_t *)PSRAM_CART_BASE;
}

static uint32_t CLI_GetWindowSize (void) {
    return Cart_GetImageSize();   /* PSRAM_CART_SIZE = 8 MiB */
}

/* ------------------------------------------------------------------ */
/* USART1 GPIO + peripheral setup.                                    */
/* ------------------------------------------------------------------ */

static void CLI_USART1_GPIO_Init(void)
{
    /* PA9 = TX (AF push-pull 50 MHz), PA10 = RX (floating input). */
    RCC_PB2PeriphClockCmd(RCC_PB2Periph_GPIOA, ENABLE);

    GPIOA->CFGHR &= ~((0xFu << 4) | (0xFu << 8));   /* clear PA9, PA10 */
    GPIOA->CFGHR |=  ((0xBu << 4) | (0x4u << 8));
    /* 0xB = 0b1011 = AF-out PP 50 MHz, 0x4 = floating input. */
}

static void CLI_USART1_Periph_Init(uint32_t baud)
{
    USART_InitTypeDef ui;
    RCC_PB2PeriphClockCmd(RCC_PB2Periph_USART1, ENABLE);

    USART_StructInit(&ui);
    ui.USART_BaudRate            = baud;
    ui.USART_WordLength          = USART_WordLength_8b;
    ui.USART_StopBits            = USART_StopBits_1;
    ui.USART_Parity              = USART_Parity_No;
    ui.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
    ui.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART1, &ui);

    USART_Cmd(USART1, ENABLE);

    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

    NVIC_EnableIRQ(USART1_IRQn);
    NVIC_SetPriority(USART1_IRQn, 0x40);   /* below EXTI0 (priority 0) */
}

void CLI_Init(void)
{
    CLI_USART1_GPIO_Init();
    CLI_USART1_Periph_Init(115200U);

    printf ("\r\nCLI ready. Type HELP for commands.\r\n> ");
}

/* ------------------------------------------------------------------ */
/* Helpers.                                                            */
/* ------------------------------------------------------------------ */

/* Hex parser. Accepts 0x prefix or bare hex. Advances *pp on success. */
static int CLI_ParseHex(const char **pp, uint32_t *out, uint32_t max_digits)
{
    const char *p = *pp;
    uint32_t v = 0U;
    uint32_t n = 0U;

    while (*p == ' ' || *p == '\t') p++;

    if (p[0] == '0' && (p[1] == 'x' || p[1] == 'X')) p += 2;

    if (*p == '\0' || *p == '\r' || *p == '\n') return -1;

    for (; n < max_digits; n++, p++) {
        char c = *p;
        uint32_t d;
        if      (c >= '0' && c <= '9') d = (uint32_t)(c - '0');
        else if (c >= 'a' && c <= 'f') d = (uint32_t)(c - 'a' + 10);
        else if (c >= 'A' && c <= 'F') d = (uint32_t)(c - 'A' + 10);
        else break;
        v = (v << 4) | d;
    }
    if (n == 0) return -1;
    *out = v;
    *pp = p;
    return 0;
}

static int CLI_Token(const char *line, const char *tok)
{
    while (*tok != '\0') {
        if (*line != *tok) return 0;
        line++; tok++;
    }
    return (*line == ' ' || *line == '\t' || *line == '\0' ||
            *line == '\r' || *line == '\n');
}

/* ------------------------------------------------------------------ */
/* Command dispatch.                                                  */
/* ------------------------------------------------------------------ */

static void CLI_HandleLine(char *line)
{
    while (*line == ' ' || *line == '\t') line++;
    if (*line == '\0' || *line == '\r' || *line == '\n') return;

    if (CLI_Token(line, "PING")) {
        printf ("OK\r\n");
    }
    else if (CLI_Token(line, "PE4")) {
        /* Diagnostic: read PE4 input state. Returns 0 (driven/floating
         * low) or 1 (high). Used to verify reset wiring. */
        uint32_t v = (GPIOE->INDR >> 4) & 1U;
        printf ("PE4=%u CFGLR=%08lx\r\n",
                (unsigned)v, (unsigned long)GPIOE->CFGLR);
    }
    else if (CLI_Token(line, "HELP")) {
        printf ("Commands:\r\n");
        printf ("  PING                       - liveness check\r\n");
        printf ("  RST [ms]                   - pulse MSX reset (decimal, default 100)\r\n");
        printf ("  PE4                        - read current PE4 state (0=low, 1=high)\r\n");
        printf ("  MAP [name|none]            - show or switch the active cart mapper\r\n");
        printf ("  SCC                        - SCC emulator diagnostics (queue level, mapper)\r\n");
        printf ("  LOAD <hexaddr> <bytes...>  - write hex bytes into PSRAM image window\r\n");
        printf ("  XLOAD <hexaddr> <len>      - raw binary upload (script-driven)\r\n");
        printf ("  DUMP <hexaddr> <len>       - read <len> bytes from PSRAM image window\r\n");
    }
    else if (CLI_Token(line, "MAP")) {
        /* Show or switch the active cart mapper. Mapper selection
         * installs the matching EXTI0 handler in the PFIC VTF slot;
         * the cart image lives entirely in PSRAM (see Cart_GetImageBase).
         * PSRAM must be initialised before any non-NONE mapper is
         * accepted - Cart_SetMapper() returns -1 otherwise. */
        const char *p = line + 3;
        while (*p == ' ' || *p == '\t') p++;
        if (*p == '\0' || *p == '\r' || *p == '\n' || CLI_Token(p, "?")) {
            printf ("MAP %s  (image @ 0x%08x, %u bytes)\r\n",
                    Cart_MapperNames[(unsigned)Cart_GetMapper()],
                    (unsigned)Cart_GetImageBase(),
                    (unsigned)Cart_GetImageSize());
            printf ("  NONE   ROM16k ROM32k ROM48k KONAMI KONAMINOSCC "
                    "ASCII8k ASCII16k NEO8 NEO16 KONAMISCC\r\n");
        } else {
            int rc = -1;
            for (unsigned i = 0; i < (unsigned)CART_MAP_MAX; i++) {
                if (CLI_Token(p, (char *)Cart_MapperNames[i])) {
                    rc = Cart_SetMapper ((Cart_Mapper)i);
                    break;
                }
            }
            if (rc == 0) {
                printf ("OK MAP %s @ 0x%08x\r\n",
                        Cart_MapperNames[(unsigned)Cart_GetMapper()],
                        (unsigned)Cart_GetImageBase());
            } else {
                printf ("ERR MAP: unknown name or PSRAM not initialised\r\n");
            }
        }
    }
    else if (CLI_Token(line, "SCC")) {
        /* SCC emulator diagnostics: queue level + which mapper feeds
         * it. Queue drains at the sample rate, so a non-zero level on
         * a running MSX is normal; a level stuck at 63/64 means the
         * TIM4 pump is not running (or the KONAMISCC mapper is queued
         * writes while SCC_Init never ran). */
        printf ("SCC q=%u/64 mapper=%s\r\n",
                (unsigned)SCC_GetLevel(),
                Cart_MapperNames[(unsigned)Cart_GetMapper()]);
    }
    else if (CLI_Token(line, "RST")) {
        /* RST takes a DECIMAL millisecond count (most users expect this),
         * unlike LOAD/DUMP which take hex addresses and byte values. */
        const char *p = line + 3;
        while (*p == ' ' || *p == '\t') p++;
        uint32_t ms = 100U;
        if (*p >= '0' && *p <= '9') {
            ms = 0U;
            while (*p >= '0' && *p <= '9') {
                ms = ms * 10U + (uint32_t)(*p - '0');
                p++;
            }
        }
        if (ms == 0U) ms = 100U;
        if (ms > 10000U) ms = 10000U;   /* sanity cap */
        Cart_AssertMSXReset(ms);
        printf ("OK %ums\r\n", (unsigned)ms);
    }
    else if (CLI_Token(line, "LOAD")) {
        const char *p = line + 4;
        uint32_t addr = 0U;
        uint32_t i = 0U;
        if (CLI_ParseHex(&p, &addr, 8) != 0) {
            printf ("ERR LOAD: bad address\r\n");
            return;
        }
        uint8_t *base = CLI_GetWindow();
        uint32_t cap = CLI_GetWindowSize();
        while (*p != '\0' && *p != '\r' && *p != '\n') {
            uint32_t byte = 0U;
            if (CLI_ParseHex(&p, &byte, 2) != 0) {
                printf ("ERR LOAD: bad byte at index %u\r\n", (unsigned)i);
                return;
            }
            if ((addr + i) >= cap) break;
            base[addr + i] = (uint8_t)byte;
            i++;
        }
        printf ("OK %u\r\n", (unsigned)i);
    }
    else if (CLI_Token(line, "XLOAD")) {
        /* Binary upload: header "XLOAD <addr> <len>" then <len> raw bytes.
         * Each byte goes straight into the ACTIVE window and is NOT
         * echoed.
         *
         * IMPORTANT: the host sends the header terminated with CRLF. The
         * line assembler fires on the '\r' to enter XLOAD mode, but the
         * trailing '\n' is still in the USART RX FIFO. Without explicit
         * handling, that '\n' would be consumed as the FIRST data byte
         * (writing 0x0a to s_xload_addr), shifting all subsequent bytes
         * by +1. We solve this by setting a flag that tells the IRQ to
         * swallow exactly one byte after entering XLOAD mode - the byte
         * that is the '\n' from the header's CRLF terminator. */
        const char *p = line + 5;
        uint32_t addr = 0U, len = 0U;
        if (CLI_ParseHex(&p, &addr, 8) != 0 ||
            CLI_ParseHex(&p, &len, 8) != 0) {
            printf ("ERR XLOAD: usage XLOAD <addr> <len>\r\n");
            return;
        }
        uint32_t cap = CLI_GetWindowSize();
        if (len > cap || addr >= cap || (addr + len) > cap) {
            printf ("ERR XLOAD: range exceeds window (%u bytes)\r\n",
                    (unsigned)cap);
            return;
        }
        s_xload_active    = 1U;
        s_xload_addr      = addr;
        s_xload_remaining = len;
        printf ("READY\r\n");
        return;
    }
    else if (CLI_Token(line, "DUMP")) {
        const char *p = line + 4;
        uint32_t addr = 0U, len = 0U;
        if (CLI_ParseHex(&p, &addr, 8) != 0 ||
            CLI_ParseHex(&p, &len, 8) != 0) {
            printf ("ERR DUMP: usage DUMP <addr> <len>\r\n");
            return;
        }
        uint32_t cap = CLI_GetWindowSize();
        if (addr >= cap) addr = cap - 1U;
        if (len > 256U) len = 256U;
        if ((addr + len) > cap) len = cap - addr;
        const uint8_t *base = CLI_GetWindow();
        printf ("DUMP %x %x:", (unsigned)addr, (unsigned)len);
        for (uint32_t i = 0U; i < len; i++) {
            printf (" %02x", (unsigned)base[addr + i]);
        }
        printf ("\r\n");
    }
    else {
        printf ("ERR unknown command\r\n");
    }
}

/* ------------------------------------------------------------------ */
/* USART1 IRQ entry. Byte-level RX + line assembly.                   */
/* ------------------------------------------------------------------ */
/* Command dispatch happens in the MAIN LOOP (CLI_Service), not in the
 * IRQ: printf() busy-waits on TXE for every character of a response,
 * which can block the IRQ for ~1.5 ms at 115200. Meanwhile the USART
 * RX register is only 1 deep - any byte the host sends during that
 * window is lost (ORE). The XLOAD host pumps bytes back-to-back, so a
 * command issued while a previous response was still printing would
 * lose its leading characters and dispatch as garbage ("ERR unknown
 * command"). Deferring dispatch to main() means the IRQ never spends
 * longer than one echo byte in the handler, and the dispatcher can
 * only start when the previous response fully drained (prompt printed,
 * flag cleared). */

/* Set by the IRQ when a full command line is ready for dispatch. */
static volatile uint8_t s_line_ready = 0U;

void CLI_USART1_Handler(void)
{
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
        uint8_t b = (uint8_t)USART_ReceiveData(USART1);

        /* XLOAD: each byte goes straight into the ACTIVE window, no
         * echo. The line assembler + CRLF suppression have already
         * consumed the header's '\r' and '\n' before the pump armed
         * (dispatch happens in the main loop), so every byte arriving
         * here is real data - no terminator swallowing. The window
         * base is captured once here so the per-byte path is a plain
         * indexed store. XLOAD stays in the IRQ because it is pure
         * RX->store, no printf, no dispatch, and must keep up with a
         * back-to-back host at full line rate. */
        if (s_xload_active) {
            uint8_t *base = CLI_GetWindow();
            base[s_xload_addr++] = b;
            s_xload_remaining--;
            if (s_xload_remaining == 0U) {
                s_xload_active = 0U;
                /* Print the trailing prompt FIRST, then the OK line.
                 *
                 * The reverse order (OK then prompt) was dropping the
                 * prompt over the WCH-Link's USB-CDC bridge: after
                 * "OK\r\n" finished its 4-byte transmission, the
                 * subsequent printf("> ") was being absorbed by the
                 * next USART RX byte - which on a 32 KiB blob is the
                 * next XLOAD byte the host is still streaming. Putting
                 * the prompt before the line marker makes the
                 * response start with the prompt byte, which the
                 * bridge cannot lose to RX contention. The
                 * line-anchored matcher in load.py accepts either
                 * "OK\r\n> " or "> OK\r\n" as a valid response. */
                printf ("> OK\r\n");
            }
        }
        else {
            /* Normal line mode: echo + assemble.
             *
             * NOTE: the echo busy-waits on TXE for ONE byte (~87 us),
             * which the 1-deep RX shift register tolerates because the
             * host cannot send the next byte's start bit before then
             * unless it is already streaming. Interactive typists are
             * far slower than that, and scripted commands are sent one
             * line at a time (prompt-synchronised by the host tools). */
            if (b == '\r') {
                USART_SendData(USART1, '\r');
                while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET) {}
                USART_SendData(USART1, '\n');
                while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET) {}
            } else if (b != '\n') {
                USART_SendData(USART1, b);
                while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET) {}
            }

            if (b == '\r' || b == '\n') {
                /* Terminate the line and hand it to the main loop.
                 * Always null-terminate (even for empty lines, at
                 * index 0) so CLI_Service never re-dispatches a stale
                 * previous command. Empty lines dispatch as no-ops and
                 * just re-print the prompt.
                 *
                 * CRLF suppression: if this '\n' arrives while a line
                 * is ALREADY pending dispatch (set by its '\r'), it is
                 * the second half of a CRLF - ignore it instead of
                 * arming a second (empty) dispatch, which would print
                 * a duplicate "> " prompt. A bare '\n' from an LF-only
                 * host still arms normally because no '\r' preceded it. */
                if (b == '\n' && s_line_ready) {
                    /* second half of CRLF - already armed, skip */
                } else {
                    s_line[s_line_len] = '\0';
                    for (uint16_t i = 0U; i <= s_line_len; i++) {
                        s_dispatch_buf[i] = s_line[i];
                    }
                    s_line_len = 0U;
                    s_line_ready = 1U;
                }
            } else if (b == 0x7F || b == 0x08) {
                if (s_line_len > 0U) s_line_len--;
            } else if (s_line_len < (CLI_LINE_SZ - 1U)) {
                s_line[s_line_len++] = (char)b;
            }
        }
    }

    /* ORE: clear if latched. A lost byte only matters for line-mode
     * commands (XLOAD re-syncs on READY/OK handshakes), and the host
     * tools always wait for the prompt before the next command. */
    if (USART_GetITStatus(USART1, USART_IT_ORE) != RESET) {
        (void)USART_ReceiveData(USART1);
        USART_ClearITPendingBit(USART1, USART_IT_ORE);
    }
}

/* ------------------------------------------------------------------ */
/* Main-loop service.                                                 */
/* ------------------------------------------------------------------ */

void CLI_Service(void)
{
    if (s_line_ready) {
        s_line_ready = 0U;
        CLI_HandleLine(s_dispatch_buf);
        printf ("> ");
    }
}