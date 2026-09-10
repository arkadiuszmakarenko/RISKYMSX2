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
 *   PING / HELP / RST [ms] / LOAD <hexaddr> <hexbytes> / DUMP <hexaddr> <len>
 */

#include "cli.h"
#include "ch32v4x7.h"
#include "cart.h"
#include "psram.h"
#include "debug.h"
#include <string.h>

/* Linear command buffer. */
#define CLI_LINE_SZ   96U
static char     s_line[CLI_LINE_SZ];
static uint16_t s_line_len = 0U;

/* XLOAD state: after the XLOAD header line is parsed, the next <len>
 * raw bytes are appended to the destination chosen by the command
 * (SRAM cart mirror or external PSRAM).
 *
 * s_xload_swallow_first: set when XLOAD is armed, used by the IRQ to
 * discard exactly one byte (the '\n' from the header's CRLF terminator).
 * Without this, the '\n' would be counted as the first XLOAD data byte
 * and shift all subsequent bytes by +1. */
typedef enum {
    XLOAD_DEST_NONE = 0,
    XLOAD_DEST_SRAM,
    XLOAD_DEST_PSRAM,
} XLoadDest;
static uint8_t  s_xload_active        = 0U;
static uint8_t  s_xload_swallow_first = 0U;
static uint32_t s_xload_addr          = 0U;
static uint32_t s_xload_remaining     = 0U;
static XLoadDest s_xload_dest         = XLOAD_DEST_NONE;

/* PSRAM write accumulator. PSRAM's FSMC interface needs 32-bit word
 * stores to complete each bus cycle cleanly; byte stores in rapid
 * succession from IRQ context can drop bytes. We accumulate up to 4
 * bytes and flush as a single 32-bit store. SRAM doesn't need this. */
static uint8_t  s_xload_byte_buf[4];
static uint8_t  s_xload_byte_buf_len  = 0U;

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
        printf ("  LOAD <hexaddr> <bytes...>  - write hex bytes into SRAM cart mirror\r\n");
        printf ("  XLOAD <hexaddr> <len>      - raw binary upload into SRAM (alias for XLOAD_SRAM)\r\n");
        printf ("  XLOAD_SRAM <hexaddr> <len> - raw binary upload into SRAM cart mirror\r\n");
        printf ("  XLOAD_PSRAM <hexaddr> <len>- raw binary upload into external PSRAM bus\r\n");
        printf ("  DUMP <hexaddr> <len>       - read <len> bytes from SRAM cart mirror\r\n");
        printf ("  DUMP_PSRAM <hexaddr> <len> - read <len> bytes from PSRAM (offset)\r\n");
        printf ("  RUN <hexaddr>              - jump to machine code at SRAM addr (IRQs off)\r\n");
        printf ("  RUN_SRAM <hexaddr>         - jump to machine code at SRAM addr (alias)\r\n");
        printf ("  RUN_PSRAM <hexaddr>        - jump to machine code at PSRAM addr (IRQs off)\r\n");
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
        uint8_t *base = (uint8_t *)SRAM_ROM_BASE;
        while (*p != '\0' && *p != '\r' && *p != '\n') {
            uint32_t byte = 0U;
            if (CLI_ParseHex(&p, &byte, 2) != 0) {
                printf ("ERR LOAD: bad byte at index %u\r\n", (unsigned)i);
                return;
            }
            base[addr + i] = (uint8_t)byte;
            i++;
            if (i >= CART_ROM_SIZE) break;
        }
        printf ("OK %u\r\n", (unsigned)i);
    }
    else if (CLI_Token(line, "XLOAD_SRAM") ||
             CLI_Token(line, "XLOAD_PSRAM") ||
             CLI_Token(line, "XLOAD")) {
        /* Binary upload to one of two destinations:
         *   XLOAD_SRAM  <addr> <len>  -> SRAM cart mirror (default)
         *   XLOAD_PSRAM <addr> <len>  -> external PSRAM bus
         *   XLOAD       <addr> <len>  -> alias for XLOAD_SRAM (compat)
         *
         * Header is terminated by CRLF. The line assembler fires on
         * '\r' to enter XLOAD mode, but the trailing '\n' is still in
         * the RX FIFO. s_xload_swallow_first discards exactly one byte
         * after arming - that's the '\n' from the header's CRLF.
         */
        XLoadDest dest = XLOAD_DEST_SRAM;
        const char *p;
        if (CLI_Token(line, "XLOAD_PSRAM")) {
            dest = XLOAD_DEST_PSRAM;
            p = line + 11;       /* skip "XLOAD_PSRAM" */
        } else if (CLI_Token(line, "XLOAD_SRAM")) {
            dest = XLOAD_DEST_SRAM;
            p = line + 10;       /* skip "XLOAD_SRAM" */
        } else {
            dest = XLOAD_DEST_SRAM;
            p = line + 5;        /* skip "XLOAD" */
        }

        uint32_t addr = 0U, len = 0U;
        if (CLI_ParseHex(&p, &addr, 8) != 0 ||
            CLI_ParseHex(&p, &len, 8) != 0) {
            printf ("ERR XLOAD: usage XLOAD_<SRAM|PSRAM> <addr> <len>\r\n");
            return;
        }

        uint32_t cap = (dest == XLOAD_DEST_PSRAM)
                     ? PSRAM_TEST_SIZE * 1024U  /* 1 MiB - roomier */
                     : CART_ROM_SIZE;
        if (len > cap || (addr + len) > cap) {
            printf ("ERR XLOAD: range exceeds dest (%u bytes)\r\n",
                    (unsigned)cap);
            return;
        }
        s_xload_active        = 1U;
        s_xload_dest          = dest;
        s_xload_addr          = addr;
        s_xload_remaining     = len;
        s_xload_swallow_first = 1U;
        printf ("READY\r\n");
        return;
    }
    else if (CLI_Token(line, "RUN_SRAM") ||
             CLI_Token(line, "RUN_PSRAM") ||
             CLI_Token(line, "RUN")) {
        /* Jump to user code at <addr>. Disables IRQs first so the
         * caller doesn't accidentally re-enter the CLI mid-run, and
         * restores mepc/mstatus so a `mret` inside the user code can
         * return cleanly to the CLI (if it bothers to).
         *
         * The user code at <addr> must be RV32I machine code. It runs
         * with global IRQs disabled; if it wants UART output it should
         * re-enable USART1 and the cart EXTI itself.
         *
         * The argument is an OFFSET relative to the chosen region:
         *   RUN_SRAM  <off>  -> jumps to SRAM_ROM_BASE + off
         *   RUN_PSRAM <off>  -> jumps to PSRAM_BUS_BASE + off
         *   RUN       <off>  -> alias for RUN_SRAM
         */
        const char *p;
        uint32_t addr;
        uint32_t base;
        const char *which;
        if (CLI_Token(line, "RUN_PSRAM")) {
            p      = line + 9;
            base   = PSRAM_BUS_BASE;
            which  = "PSRAM";
        } else if (CLI_Token(line, "RUN_SRAM")) {
            p      = line + 8;
            base   = SRAM_ROM_BASE;
            which  = "SRAM";
        } else {
            p      = line + 3;
            base   = SRAM_ROM_BASE;
            which  = "SRAM";
        }
        if (CLI_ParseHex(&p, &addr, 8) != 0) {
            printf ("ERR RUN_%s: usage RUN_%s <hexoffset>\r\n",
                    which, which);
            return;
        }
        uint32_t target = base + addr;
        printf ("JUMP %s offset=0x%x target=0x%08x (IRQs off)\r\n",
                which, (unsigned)addr, (unsigned)target);

        /* Disable IRQs before the jump so the cart EXTI doesn't fire
         * and corrupt the user's code path. Caller re-enables if it
         * returns via mret. */
        __disable_irq();
        /* The compiler doesn't know about this indirect call; asm
         * volatile guarantees the jump is emitted. */
        __asm__ volatile ("jalr zero, %0, 0" : : "r" (target) : "ra", "memory");
        /* If the user code returns, re-enable IRQs and resume CLI. */
        __enable_irq();
        printf ("OK returned from 0x%08x\r\n", (unsigned)target);
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
        if (len > 256U) len = 256U;
        const uint8_t *base = (const uint8_t *)SRAM_ROM_BASE;
        printf ("DUMP %x %x:", (unsigned)addr, (unsigned)len);
        for (uint32_t i = 0U; i < len; i++) {
            printf (" %02x", (unsigned)base[addr + i]);
        }
        printf ("\r\n");
    }
    else if (CLI_Token(line, "DUMP_PSRAM")) {
        /* Like DUMP but reads from the external PSRAM bus. The address
         * is an offset relative to PSRAM_BUS_BASE (0x80000000). */
        const char *p = line + 10;
        uint32_t addr = 0U, len = 0U;
        if (CLI_ParseHex(&p, &addr, 8) != 0 ||
            CLI_ParseHex(&p, &len, 8) != 0) {
            printf ("ERR DUMP_PSRAM: usage DUMP_PSRAM <addr> <len>\r\n");
            return;
        }
        if (len > 256U) len = 256U;
        const uint8_t *base = (const uint8_t *)PSRAM_BUS_BASE;
        printf ("DUMP_PSRAM %x %x:", (unsigned)addr, (unsigned)len);
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
/* USART1 IRQ entry. Byte-level RX + line assembly + dispatch.        */
/* ------------------------------------------------------------------ */

void CLI_USART1_Handler(void)
{
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
        uint8_t b = (uint8_t)USART_ReceiveData(USART1);

        /* XLOAD: each byte goes straight into the chosen destination
         * (SRAM cart mirror or external PSRAM via the integrated
         * controller), no echo.
         * The first byte after entering XLOAD is the trailing '\n' from
         * the header's CRLF - discard it without writing.
         *
         * For SRAM, byte stores are zero-wait-state so we just write
         * straight through. For the integrated PSRAM controller, the
         * SPI/QSPI write FIFO can drop bytes if we hammer it byte-by-
         * byte from the IRQ, so we instead accumulate into a small byte
         * buffer and flush with 32-bit word stores every time we have
         * 4 bytes queued. The flush runs from IRQ context too, so
         * throughput is unchanged for the host but PSRAM cycles
         * complete cleanly. */
        if (s_xload_active) {
            if (s_xload_swallow_first) {
                s_xload_swallow_first = 0U;
                /* discard this byte, don't write it */
            }
            else {
                uint8_t *base = (s_xload_dest == XLOAD_DEST_PSRAM)
                              ? (uint8_t *)PSRAM_BUS_BASE
                              : (uint8_t *)SRAM_ROM_BASE;
                /* Always byte-store into the byte buffer. The flush
                 * code below converts PSRAM writes to 32-bit word
                 * stores. SRAM goes straight through with byte stores. */
                s_xload_byte_buf[s_xload_byte_buf_len++] = b;
                if (s_xload_dest == XLOAD_DEST_SRAM) {
                    /* SRAM: write through immediately. */
                    base[s_xload_addr++] = b;
                    s_xload_remaining--;
                } else {
                    /* PSRAM: accumulate. Flush when we have 4 bytes or
                     * at end-of-transfer. */
                    if (s_xload_byte_buf_len == 4U ||
                        s_xload_remaining == 1U) {
                        uint32_t target = s_xload_addr & ~3U;
                        /* Read-modify-write so partial words merge
                         * with previous content. */
                        uint32_t cur = *(volatile uint32_t *)(base + target);
                        uint8_t *cur_bytes = (uint8_t *)&cur;
                        uint32_t off = s_xload_addr & 3U;
                        for (uint32_t i = 0U; i < s_xload_byte_buf_len; i++) {
                            cur_bytes[off + i] = s_xload_byte_buf[i];
                        }
                        *(volatile uint32_t *)(base + target) = cur;
                        s_xload_addr += s_xload_byte_buf_len;
                        s_xload_remaining -= s_xload_byte_buf_len;
                        s_xload_byte_buf_len = 0U;
                    }
                }

                if (s_xload_remaining == 0U) {
                    s_xload_active = 0U;
                    s_xload_dest   = XLOAD_DEST_NONE;
                    printf ("OK\r\n");
                    /* Auto-reset the MSX so it re-reads the cart slot
                     * with the freshly-uploaded ROM. Mirrors the
                     * psram_upload.py behaviour where the script sent
                     * RST 100 after every upload. */
                    printf ("> RST 100\r\n");
                    Cart_AssertMSXReset(100U);
                    printf ("OK 100ms\r\n");
                    printf ("> ");
                }
            }
            /* NOTE: we deliberately do not echo, do not line-assemble,
             * and do not check TXE. The host is pumping bytes at us as
             * fast as possible - any echo would bottleneck. */
        }
        else {
            /* Normal line mode: echo + assemble. */
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
                s_line[s_line_len] = '\0';
                CLI_HandleLine(s_line);
                s_line_len = 0U;
                printf ("> ");
            } else if (b == 0x7F || b == 0x08) {
                if (s_line_len > 0U) s_line_len--;
            } else if (s_line_len < (CLI_LINE_SZ - 1U)) {
                s_line[s_line_len++] = (char)b;
            }
        }
    }

    /* ORE: clear if latched. */
    if (USART_GetITStatus(USART1, USART_IT_ORE) != RESET) {
        (void)USART_ReceiveData(USART1);
        USART_ClearITPendingBit(USART1, USART_IT_ORE);
    }
}