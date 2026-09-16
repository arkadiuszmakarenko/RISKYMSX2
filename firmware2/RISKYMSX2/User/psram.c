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
#include "cart.h"
#include "ch32v4x7.h"
#include "ch32v4x7_psram.h"
#include "debug.h"
#include <string.h>

/* Cart image from hello_rom.c. Declared with an INCOMPLETE array type
 * and a separate length symbol: the definition is a 1-byte placeholder
 * (no embedded ROM), and taking sizeof() on a 32768-declared extern
 * would always report 32768 - the placeholder branch would be dead
 * code and the full-32K copy would read past the 1-byte object in
 * flash. Use hello_rom_len to decide which path to take. */
extern const uint8_t  hello_rom[];
extern const uint32_t hello_rom_len;

/* ---------- PSRAM device MR encodings (copied from PSRAM/PSRAM/User/PSRAM.h) */
#define PSRAM_MR_ADDR_0          0x00U  /* read latency / operating range */
#define PSRAM_MR_ADDR_4          0x04U  /* write latency / operating range */
#define PSRAM_MR_ADDR_8          0x08U  /* Hfreq_En / variable-latency enable */

/* Variable-latency vs fixed-latency read mode (MR0[5]). */
#define READ_LATENCY_VARIABLE    ((uint32_t)0x0)

/* Hfreq_En - enables the device's high-frequency IO path. REQUIRED for the
 * 333M/400M operating-range codes: those code values alias with the 66M/109M
 * bands, and it is the MR8 Hfreq bit that tells the device which table to
 * decode them from. Writing a 400M code without Hfreq_En leaves the device
 * configured for the 109M band - the IO compensation runs low-frequency
 * mode while 400 MHz DDR arrives, and reads corrupt intermittently. */
#define HFREQ_EN                 ((uint32_t)0x01)

/* Band-lookup ceiling. The CH32V4x7 PSRAM peripheral has NO clock
 * divider (PSRAMCLK is hardwired to 2x HCLK, dual-edge DDR). So if
 * HCLK = 200 MHz the PSRAM device is *electrically* clocked at
 * 400 MHz DDR regardless of what we ask for - there is no way to
 * actually slow it down from firmware.
 *
 * What this macro does is override the MR operating-range band the
 * driver programs into the device. If the device silicon and PCB are
 * healthy at 400 MHz DDR, programming the 400M band is correct and
 * verify passes. If the bus is marginal, programming a lower band
 * (e.g. 300M) tells the device to use looser internal IO/timing
 * compensation - which sometimes lets a marginal bus pass.
 *
 * Defaults to 400 (the band that matches the actual device clock).
 * Override by defining before including this file, or uncomment
 * the #define below. Common diagnostic values: 300, 250.
 *
 * NOTE: this changes MR programming only. The device clock itself
 * still runs at 2x HCLK. If the 400M-band failure is a real signal-
 * integrity issue, no band trick will fix it - this knob is for
 * debugging the band-encoding path, not for fixing PCB problems. */
/* #define PSRAM_BAND_CEILING_MHZ 300U */
#ifndef PSRAM_BAND_CEILING_MHZ
#define PSRAM_BAND_CEILING_MHZ 400U
#endif

/* Operating-range band table. The PSRAM device is clocked at 2x HCLK
 * (dual-edge / DDR: the WCH "PSRAM_300MHz_HSE" reference runs HCLK 150 MHz
 * with the 300M codes), so the band must be selected by the DEVICE clock,
 * not HCLK. Codes and the peripheral-LATENCY pairing are copied verbatim
 * from PSRAM/PSRAM/User/PSRAM.h (MR0 codes <<2, MR4 codes <<5):
 *
 *   device MHz top | MR0 read | MR4 write | periph LATENCY | Hfreq
 *   ---------------+----------+-----------+----------------+------
 *   66 / 109 / 133 / 166 / 200 / 225 / 250 / 300   (Hfreq = 0)
 *   333 / 400                                     (Hfreq = 1)  */
typedef struct {
    uint32_t dev_mhz_max;      /* highest device clock this band covers */
    uint32_t mr0_read;         /* MR0[4:2] read operating-range code     */
    uint32_t mr4_write;        /* MR4[7:5] write operating-range code    */
    uint32_t periph_latency;   /* PSRAM->LATENCY value paired w/ band     */
    uint8_t  hfreq;            /* MR8 Hfreq_En required for this band     */
} Psram_Band;

static const Psram_Band psram_bands[] = {
    {  66U, 0x0U, 0x0U, 0x03U, 0U },  /* 66M           */
    { 109U, 0x1U, 0x4U, 0x04U, 0U },  /* 109M          */
    { 133U, 0x2U, 0x2U, 0x05U, 0U },  /* 133M          */
    { 166U, 0x3U, 0x6U, 0x06U, 0U },  /* 166M          */
    { 200U, 0x4U, 0x1U, 0x07U, 0U },  /* 200M          */
    { 225U, 0x5U, 0x5U, 0x08U, 0U },  /* 225M          */
    { 250U, 0x6U, 0x3U, 0x09U, 0U },  /* 250M          */
    { 300U, 0x7U, 0x7U, 0x0BU, 0U },  /* 300M (WCH ref) */
    { 333U, 0x0U, 0x0U, 0x0CU, 1U },  /* 333M + Hfreq  */
    { 400U, 0x1U, 0x4U, 0x10U, 1U },  /* 400M + Hfreq  */
};

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
 * range code and the peripheral LATENCY register with the band-paired
 * latency. The hfreq flag selects the 333M/400M bands: those code values
 * must be paired with the MR8 Hfreq_En bit, otherwise the device decodes
 * them as the 66M/109M bands (see the HFREQ_EN comment above). */
static void psram_set_wr_latency(uint32_t mr4_freq, uint32_t latency,
                                 uint8_t hfreq)
{
    psram_write_reg(PSRAM_MR_ADDR_4, (uint16_t)(mr4_freq << 5));
    PSRAMSetWrLatency(latency);
    psram_wait_busy();

    if (hfreq) {
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
                                 uint32_t latency_type, uint8_t hfreq)
{
    (void)latency_type;

    psram_write_reg(PSRAM_MR_ADDR_0, (uint16_t)(mr0_freq << 2));
    PSRAMSetRdLatency(latency);
    psram_wait_busy();

    if (hfreq) {
        psram_write_reg(PSRAM_MR_ADDR_8, (uint16_t)(HFREQ_EN << 5));
    }
}

/* Fill the entire cart image window with 0xFF (the MSX "open bus"
 * pattern for an unpopulated slot). NO embedded ROM is mirrored at
 * boot any more: with the mapper left at NONE the MSX drops to BASIC;
 * a cart image is expected via XLOAD. (The hello_rom[] mirror + verify
 * this replaces was the boot-time "initial tests" the diagnostic ROM
 * ran on every reset.) */
static uint8_t psram_copy_rom(void)
{
    volatile uint32_t *dst32 = (volatile uint32_t *)PSRAM_BUS_BASE;
    uint32_t words = PSRAM_CART_SIZE / 4U;
    for (uint32_t i = 0; i < words; i++) dst32[i] = 0xFFFFFFFFU;
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

    /* 0. HCLK ceiling. The PSRAM device is driven by the HB-bus clock, and
     *    HCLK is the core clock on this chip (SystemCoreClockUpdate:
     *    HCLKClock = SystemCoreClock), so core speed and PSRAM speed are
     *    the same knob - there is no independent PSRAM divider. The device
     *    clock is 2x HCLK (dual-edge) and the MR operating-range table tops
     *    out at the 400M band, so above 200 MHz HCLK there is no valid band
     *    to program (at 240 MHz reads return the 0xA0 corruption pattern).
     *    Bail out cleanly so the cart handler falls back to the flash image.
     *
     *    The PSRAM_BAND_CEILING_MHZ override (see top of file) only
     *    influences MR programming; it does NOT change the device clock.
     *    So this HCLK ceiling still applies regardless. */
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
     *    UNITS GOTCHA (200 MHz instability, root cause #2): RM 34.2.1.2 says
     *    TRC/TCPH count PSRAM clock periods, and the PSRAM clock is 2x HCLK
     *    (dual-edge device). The old code computed the cycle counts from
     *    HCLK, halving every enforced interval: at HCLK 200, TCPH=12
     *    delivered only 30 ns of CE#-high time, below the 32 ns minimum
     *    from the RM's own example ("333 MHz -> minimum 12 for 32 ns").
 *    HCLK 175 delivered 34 ns and worked; HCLK 200 delivered 30 ns and
     *    corrupted intermittently. Compute from the DEVICE clock instead.
     *
     *    Targets: tRC 150 ns (RM floor is 60 ns; keep the conservative
     *    margin - it only costs boot-time copies, the Z80 hot path does
     *    one isolated read per ~1 us cycle and is not tRC-bound), and tCPH
     *    40 ns (RM floor 32 ns; matches the WCH default 0x0C at a 300 MHz
     *    device clock = 40 ns). tXLPD is device-internal (HSI units) and
     *    stays at the fixed WCH-recommended value.
     *
     *    NOTE: must use 64-bit math. This target is RV32 (ilp32), so
     *    `unsigned long` is only 32 bits; `150UL * 400_000_000UL` would
     *    overflow silently and yield garbage. */
    {
        uint32_t devclk_hz = SystemCoreClock * 2U;  /* PSRAM clock = 2x HCLK */
        uint64_t dev64     = (uint64_t)devclk_hz;
        uint32_t trc_cycles  = (uint32_t)(((150ULL * dev64) + 999999999ULL) / 1000000000ULL);
        uint32_t tcph_cycles = (uint32_t)((( 40ULL * dev64) + 999999999ULL) / 1000000000ULL);

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

    /* 4. Program the operating-range band that matches the actual DEVICE
     *    clock (2x HCLK). 200 MHz instability, root cause #1: the old code
     *    hardcoded the WCH reference's 300M codes, valid only up to a
     *    300 MHz device clock. At HCLK 200 the device runs 400 MHz - 33%
     *    past the selected band - and the MR8 Hfreq_En bit that the 400M
     *    code requires was never set, so the device's IO/timing compensation
     *    ran in low-frequency mode while 400 MHz DDR arrived (intermittent
     *    corruption). HCLK 175 (350 MHz device) was 17% past the band and
     *    only worked by silicon margin.
     *
     *    The peripheral LATENCY (AHB bridge wait-states) is paired with the
     *    band exactly like the WCH reference (Latency_400M = 0x10 with the
     *    400M band). Variable-latency read mode matches the reference. */
    uint32_t dev_mhz = (SystemCoreClock * 2U) / 1000000U;
    /* Cap the band lookup at PSRAM_BAND_CEILING_MHZ. The PSRAM device is
     * still electrically clocked at 2x HCLK - this only changes the MR
     * operating-range we program, not the actual device clock. Useful for
     * diagnosing whether a verify failure at 400/200 is a band-encoding
     * problem or a real signal-integrity problem. See the long comment
     * on PSRAM_BAND_CEILING_MHZ at the top of this file. */
    uint32_t band_ceiling_mhz = PSRAM_BAND_CEILING_MHZ;
    if (band_ceiling_mhz > 400U) band_ceiling_mhz = 400U;
    if (band_ceiling_mhz <  66U) band_ceiling_mhz =  66U;

    const Psram_Band *band = NULL;
    for (uint32_t i = 0;
         i < (uint32_t)(sizeof(psram_bands) / sizeof(psram_bands[0]));
         i++) {
        if (band_ceiling_mhz <= psram_bands[i].dev_mhz_max) {
            band = &psram_bands[i];
            break;
        }
    }
    if (band == NULL) {
        /* No MR band covers this device clock. Step 0 already rejects
         * HCLK > 200 MHz, so this is defensive only. */
        return PSRAM_ERR_HCLK_TOO_HIGH;
    }

    psram_set_wr_latency (band->mr4_write, band->periph_latency,
                          band->hfreq);
    psram_set_rd_latency (band->mr0_read, band->periph_latency,
                          READ_LATENCY_VARIABLE, band->hfreq);

    printf ("PSRAM diag: HCLK=%u devclk=%uMHz ceiling<=%uM band<=%uM hfreq=%u mr0=0x%x mr4=0x%x lat=0x%02x trc=0x%02x tcph=0x%02x TIMING=0x%08x LATENCY=0x%08x STATUS=0x%08x\r\n",
            (unsigned)SystemCoreClock,
            (unsigned)dev_mhz,
            (unsigned)band_ceiling_mhz,
            (unsigned)band->dev_mhz_max,
            (unsigned)band->hfreq,
            (unsigned)band->mr0_read,
            (unsigned)band->mr4_write,
            (unsigned)band->periph_latency,
            (unsigned)psram_timing.PSRAM_trc,
            (unsigned)psram_timing.PSRAM_tcph,
            (unsigned)PSRAM->TIMING,
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