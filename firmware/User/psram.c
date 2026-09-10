/********************************** (C) COPYRIGHT *******************************
 * File Name          : psram.c
 * Description        : PSRAM init + self-test + hello_rom mirror for RISKYMSX2.
 *
 *                      The init sequence is a near-byte-for-byte port of the
 *                      WCH reference example at PSRAM/PSRAM/User/main.c
 *                      (PSRAM_300MHz_HSE + PSRAM_INIT). That example is
 *                      known-good on the same CH32V4x7 silicon; matching it
 *                      verbatim avoids the trap of hand-rolling MR0/MR4
 *                      encodings that the device may not match.
 *
 *                      Differences vs the WCH reference:
 *                        - Self-test runs a 1 KiB walking-byte pattern at
 *                          three different PSRAM offsets so multi-row
 *                          addressing gets exercised.
 *                        - After the test, hello_rom[32768] is mirrored to
 *                          PSRAM_BUS_BASE and byte-compared.
 *                        - On any failure, the cart handler keeps using the
 *                          flash image (g_psram_mirror_base stays 0).
 *
 *                      Latency constants are local copies of the values in
 *                      PSRAM/PSRAM/User/PSRAM.h. They are not in the
 *                      in-firmware ch32v4x7_psram.h, hence the local
 *                      definitions.
 *********************************************************************************/

#include "psram.h"
#include "ch32v4x7.h"
#include "ch32v4x7_psram.h"
#include "debug.h"
#include "string.h"

extern const uint8_t hello_rom[PSRAM_ROM_SIZE];

/* ---------- PSRAM device MR encodings (copied from PSRAM/PSRAM/User/PSRAM.h) */
#define PSRAM_MR_ADDR_0          0x00U  /* read latency / operating range */
#define PSRAM_MR_ADDR_4          0x04U  /* write latency / operating range */
#define PSRAM_MR_ADDR_8          0x08U  /* Hfreq_En / variable-latency enable */

/* MR0 read-operating-range codes (3 bits, encoded in MR0[4:2]). */
#define MR0_READ_166M            ((uint32_t)0x3)
#define MR0_READ_200M            ((uint32_t)0x4)
#define MR0_READ_250M            ((uint32_t)0x6)
#define MR0_READ_300M            ((uint32_t)0x7)

/* MR4 write-operating-range codes (3 bits, encoded in MR4[7:5]). */
#define MR4_WRITE_166M           ((uint32_t)0x6)
#define MR4_WRITE_200M           ((uint32_t)0x1)
#define MR4_WRITE_250M           ((uint32_t)0x3)
#define MR4_WRITE_300M           ((uint32_t)0x7)

/* Variable-latency vs fixed-latency selection (MR0[5]). */
#define READ_LATENCY_VARIABLE    ((uint32_t)0x0)
#define READ_LATENCY_FIXED       ((uint32_t)0x1)

/* Latency in clocks (MR0[2:0]). */
#define LATENCY_166M             ((uint32_t)0x6)
#define LATENCY_200M             ((uint32_t)0x7)
#define LATENCY_250M             ((uint32_t)0x9)
#define LATENCY_300M             ((uint32_t)0xb)

/* Hfreq_En - enables the high-frequency path in the device, used at >=333 MHz. */
#define HFREQ_EN                 ((uint32_t)0x01)

/* Latency for the peripheral's PSRAM->LATENCY register (the AHB bridge
 * wait-states). Match the device MR0 latency we're programming. */
#define PERIPH_LATENCY_200M      LATENCY_200M

/* ---------- PSRAM peripheral primitives (mirror WCH reference) */
#define PSRAM_CMD_MR_RESET       ((PSRAM->CMD1_CFG >> 16) & 0xFFU)
#define PSRAM_CMD_MR_WRITE       ((PSRAM->CMD1_CFG >> 8)  & 0xFFU)
#define PSRAM_CMD_MR_READ        ((PSRAM->CMD1_CFG >> 0)  & 0xFFU)
#define PSRAM_CMD_BUSY_BIT       ((uint32_t)0x00000001U)

static inline void psram_wait_busy(void)
{
    while ((PSRAM->ADDR & PSRAM_CMD_BUSY_BIT) != 0U) {
    }
}

/* Issue a software reset to the PSRAM device (per WCH Global_RST_Set). */
static void psram_global_reset(void)
{
    PSRAM_Set_CMD((uint8_t)PSRAM_CMD_MR_RESET);
    PSRAM_Set_Busy(ENABLE);
    Delay_Us(20);     /* PSRAM reset requires >= 2 us of CS high. */
}

/* Write a value into a PSRAM mode register. The encoding matches what
 * the WCH reference uses in SetWrLatency/SetRdLatency. */
static void psram_write_reg(uint32_t addr, uint16_t data)
{
    psram_wait_busy();
    PSRAMSetData(data);
    PSRAM_Set_MR_ADDR((uint8_t)addr);
    PSRAM_Set_CMD((uint8_t)PSRAM_CMD_MR_WRITE);
    PSRAM_Set_Busy(ENABLE);
    PSRAM_Set_MW(ENABLE);
    psram_wait_busy();
}

/* Replicate of WCH SetWrLatency(). Writes MR4 with the write-operating-
 * range code and the peripheral LATENCY register with the latency. */
static void psram_set_wr_latency(uint32_t mr4_freq, uint32_t latency)
{
    psram_write_reg(PSRAM_MR_ADDR_4, (uint16_t)(mr4_freq << 5));
    PSRAMSetWrLatency(latency);
    psram_wait_busy();

    /* For >= 333 MHz parts the WCH reference enables the Hfreq path in MR8.
     * We run at <= 200 MHz so we don't need it, but keeping the structure
     * makes the diff against the reference obvious. */
    if ((mr4_freq == 0x0U) /* MR4_Write_333M */
     || (mr4_freq == 0x4U) /* MR4_Write_400M */) {
        psram_write_reg(PSRAM_MR_ADDR_8, (uint16_t)(HFREQ_EN << 5));
    }
}

/* Replicate of WCH SetRdLatency(). Writes MR0 with ONLY the operating-range
 * code; the device's latency field is left untouched (factory default) so
 * the device's internal state machine can pick the right latency for the
 * configured range.
 *
 * Earlier versions of this function tried to write both the freq code AND
 * the latency field to MR0, but Octal PSRAM devices treat bits[2:0] as
 * read-only or auto-managed. Writing the latency bits at all corrupts the
 * device's intended behavior and produces single-bit read errors at high
 * HCLK. The WCH reference's `SetRdLatency()` only writes (freq<<2); we do
 * the same. */
static void psram_set_rd_latency(uint32_t mr0_freq, uint32_t latency,
                                 uint32_t latency_type)
{
    (void)latency_type;

    psram_write_reg(PSRAM_MR_ADDR_0, (uint16_t)(mr0_freq << 2));
    PSRAMSetRdLatency(latency);
    psram_wait_busy();

    if ((mr0_freq == 0x0U) /* MR0_Read_333M */
     || (mr0_freq == 0x1U) /* MR0_Read_400M */) {
        psram_write_reg(PSRAM_MR_ADDR_8, (uint16_t)(HFREQ_EN << 5));
    }
}

/* Mirror hello_rom[] (32 KiB) into PSRAM using 32-bit word copies then a
 * byte-for-byte verify (covers endian/swap bugs the word loop can't see). */
static uint8_t psram_copy_rom(void)
{
    const uint32_t *src32 = (const uint32_t *)hello_rom;
    volatile uint32_t *dst32 = (volatile uint32_t *)PSRAM_BUS_BASE;
    uint32_t words = PSRAM_ROM_SIZE / 4U;

    for (uint32_t i = 0; i < words; i++) {
        dst32[i] = src32[i];
    }
    const uint8_t *src8 = hello_rom;
    volatile uint8_t *dst8 = (volatile uint8_t *)PSRAM_BUS_BASE;
    if (memcmp((const void *)src8, (const void *)dst8, PSRAM_ROM_SIZE) != 0) {
        return PSRAM_ERR_ROM_MIRROR;
    }
    return PSRAM_OK;
}

/* Self-test: write/read a known pattern at `offset`, return PSRAM_OK on match.
 * On mismatch, prints a one-line diagnostic via printf so the UART log shows
 * which byte offset failed and what was actually read back. */
static uint8_t psram_test_at(uint32_t offset)
{
    static uint8_t pattern[PSRAM_TEST_SIZE];
    volatile uint8_t *ps = (volatile uint8_t *)(PSRAM_BUS_BASE + offset);

    for (uint32_t i = 0; i < PSRAM_TEST_SIZE; i++) {
        pattern[i] = (uint8_t)((i * 31U + (offset & 0xFFU)) ^ 0xA5U);
    }
    for (uint32_t i = 0; i < PSRAM_TEST_SIZE; i++) {
        ps[i] = pattern[i];
    }
    for (uint32_t i = 0; i < PSRAM_TEST_SIZE; i++) {
        if (ps[i] != pattern[i]) {
            printf ("PSRAM diag: off=0x%06x i=%u wrote=0x%02x read=0x%02x\r\n",
                    (unsigned)offset, (unsigned)i,
                    (unsigned)pattern[i], (unsigned)ps[i]);
            return PSRAM_ERR_TEST_PATTERN;
        }
    }
    return PSRAM_OK;
}

static uint32_t g_psram_mirror_base = 0U;

uint32_t PSRAM_GetRomMirrorBase(void)
{
    return g_psram_mirror_base;
}

uint8_t PSRAM_Init(void)
{
    PSRAMInitTypeDef       psram_init_struct = {0};
    PSRAMTimingInitTypeDef psram_timing      = {0};

    /* 0. HCLK ceiling. The PSRAM device clock is derived from HCLK/2.
     *    Most Octal PSRAMs top out at 133-200 MHz device clock. Empirically
     *    the WCH reference design works up to 200 MHz HCLK (=100 MHz device
     *    clock); above that, the device's data-valid window narrows below
     *    what the CH32V4x7 PSRAM controller can reliably sample. Bail out
     *    cleanly so the cart handler falls back to the flash image. */
    if (SystemCoreClock > 200000000U) {
        return PSRAM_ERR_HCLK_TOO_HIGH;
    }

    /* 1. Bring up the peripheral clock and reset the controller. */
    RCC_HBPeriphClockCmd(RCC_HBPeriph_PSRAM, ENABLE);
    PSRAMDeInit();

    /* 2. Peripheral timing + capacity. 64 Mbit (CH32V467VET6 population).
     *    Note: PSRAM_EXIT_LPMD is left at 0 (matches WCH reference); the
     *    mode-register writes below wake the device from low-power mode.
     *
     *    The WCH-recommended tRC/tCPH/tXLPD values (0x14/0x0C/0x07) are
     *    expressed in AHB clock cycles and only meet the PSRAM device's
     *    timing at <=120 MHz HCLK. At 175/200/240 MHz the read-write cycle
     *    time shrinks below the device spec and reads return corrupted data.
     *    Scale tRC and tCPH to keep at least 125 ns of read-write cycle
     *    time and 60 ns of CS-high pulse width regardless of HCLK; tXLPD is
     *    device-internal and stays at the fixed WCH-recommended value.
     *
     *    Formula: cycles = ceil(ns * HCLK_hz / 1e9), with extra headroom for
     *    board-level skew on long-cycle PSRAM parts. */
    {
        uint32_t hclk_hz = SystemCoreClock;     /* already updated in main() */
        /* 150 ns minimum tRC, rounded up to next cycle.
         * NOTE: must use 64-bit math. This target is RV32 (ilp32), so
         * `unsigned long` is only 32 bits; `150UL * 175_000_000UL` would
         * overflow silently and yield garbage. */
        uint64_t hclk64 = (uint64_t)hclk_hz;
        uint32_t trc_cycles  = (uint32_t)(((150ULL * hclk64) + 999999999ULL) / 1000000000ULL);
        uint32_t tcph_cycles = (uint32_t)((( 60ULL * hclk64) + 999999999ULL) / 1000000000ULL);

        /* Floor at WCH defaults so slow boards still work. */
        if (trc_cycles  < 0x14U)  trc_cycles  = 0x14U;
        if (tcph_cycles < 0x0CU)  tcph_cycles = 0x0CU;
        /* Ceiling at 0xFF (8-bit register field). */
        if (trc_cycles  > 0xFFU)  trc_cycles  = 0xFFU;
        if (tcph_cycles > 0xFFU)  tcph_cycles = 0xFFU;

        psram_timing.PSRAM_trc   = (uint8_t)trc_cycles;
        psram_timing.PSRAM_tcph  = (uint8_t)tcph_cycles;
        psram_timing.PSRAM_txlpd = 0x07U;
    }

    psram_init_struct.PSRAM_cfifo       = PSRAM_CFIFO_BTWWRRD;
    psram_init_struct.PSRAM_cap_cfg     = PSRAM_CAP_64M;
    psram_init_struct.PSRAM_exti_lpmd   = 0U;  /* Wake via MR writes instead. */
    psram_init_struct.PSRAMTimingStruct = &psram_timing;

    PSRAMInit(&psram_init_struct);

    /* 3. Software reset the PSRAM device itself, then a settling delay. */
    psram_global_reset();
    Delay_Ms(1);

    /* 4. Program write/read latency. Use the 300 MHz timings from the WCH
     *    reference; the PSRAM device auto-detects the actual incoming clock
     *    and the WCH example successfully reads/writes at 150 MHz HCLK with
     *    exactly this setting. Variable-latency read mode matches the WCH
     *    reference. */
    psram_set_wr_latency(MR4_WRITE_300M, LATENCY_300M);
    psram_set_rd_latency(MR0_READ_300M, LATENCY_300M,
                         READ_LATENCY_VARIABLE);

    printf ("PSRAM diag: HCLK=%u trc=0x%02x tcph=0x%02x CTLR=0x%08x TIMING=0x%08x LATENCY=0x%08x STATUS=0x%08x\r\n",
            (unsigned)SystemCoreClock,
            (unsigned)psram_timing.PSRAM_trc,
            (unsigned)psram_timing.PSRAM_tcph,
            (unsigned)PSRAM->CTLR, (unsigned)PSRAM->TIMING,
            (unsigned)PSRAM->LATENCY, (unsigned)PSRAM->STATUS);

    /* 5. Multi-offset self-test. Three addresses hit three different
     *    internal row groups on the 64 Mbit part. */
    uint8_t err = PSRAM_OK;
    err |= psram_test_at(0x000000U);
    if (err == PSRAM_OK) err |= psram_test_at(0x100000U);
    if (err == PSRAM_OK) err |= psram_test_at(0x400000U);
    if (err != PSRAM_OK) {
        return err;
    }

    /* 6. Mirror hello_rom[] into PSRAM and verify byte-for-byte. */
    err = psram_copy_rom();
    if (err != PSRAM_OK) {
        return err;
    }

    g_psram_mirror_base = PSRAM_BUS_BASE;
    return PSRAM_OK;
}