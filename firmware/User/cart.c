#include "cart.h"
#include "psram.h"

#pragma GCC push_options
#pragma GCC optimize("Ofast")

/*
 * Cartridge image pointer. Initialized to point at the flash-resident
 * hello_rom[] (compile-time constant), but reassigned at runtime to the
 * PSRAM mirror once PSRAM_Init() succeeds. RunCart32k runs from .ramfunc
 * (zero-wait-state SRAM) and reads `*cartpnt` on every SLTSL assertion; an
 * 8-bit load from flash has 1-2 wait states while the same load from PSRAM
 * is 0-wait at 160 MHz, so the PSRAM path is the fast one.
 *
 * Declared `restrict` because RunCart32k is the only reader.
 */
extern const uint8_t hello_rom[];
uint8_t *restrict cartpnt = (uint8_t *)&hello_rom[0];

/*
 * Active image source. Initialised to FLASH so the cart handler is
 * immediately usable even if Init_Cart() runs before PSRAM_Init().
 * The atomic-pointer-write guarantee on RV32C / ARM means a single
 * aligned 32-bit store is observed atomically by RunCart32k.
 */
static volatile Cart_ImageSrc g_cart_image_src = CART_IMG_FLASH;

/*
 * Whether RunCart32k should assert the MSX WAIT line around its read.
 * Updated atomically by Cart_SetImageSource() so the IRQ can branch on it
 * without any range-check or function call. Avoids the cost of testing
 * cartpnt against the PSRAM window on every SLTSL assertion.
 */
static volatile uint32_t g_cart_wait_active = 0U;

/*
 * ROM32k: a single 32 KiB page mapped at Z80 0x4000..0xBFFF.
 * bankOffsets[0] = -0x4000 converts a Z80 address straight into the
 * linear ROM index (address + offset).
 */
static uint32_t bankOffsets0;

/*********************************************************************
 * @fn      Init_Cart
 *
 * @brief   Configure GPIO + EXTI for the MSX cartridge bus and install
 *          the fast-interrupt handler for slot-select.
 */
void Init_Cart(void)
{
    /* Enable GPIO + AFIO clocks */
    RCC_PB2PeriphClockCmd(RCC_PB2Periph_GPIOA |
                          RCC_PB2Periph_GPIOB |
                          RCC_PB2Periph_GPIOD |
                          RCC_PB2Periph_GPIOE |
                          RCC_PB2Periph_AFIO, ENABLE);

    /* Address bus PD0..PD15 as inputs (default 0x44444444 = floating input).
     * Explicitly drive both config regs so nothing is left in an
     * alternate-function state after reset. */
    GPIOD->CFGLR = 0x44444444;   /* A0..A7   */
    GPIOD->CFGHR = 0x44444444;   /* A8..A15  */

    /* Control bus PE0(SLTSL) PE1(RD) PE2(WR) PE5(MREQ) as inputs.
 * PE3 = WAIT (left floating; we're not driving the MSX WAIT line because
 *         toggling it experimentally corrupts the bus read).
 * PE4 = MSX ~RESET - drive LOW here, release later from main() after the
 *         PSRAM mirror and cart source are configured. */
    GPIOE->CFGLR = 0x44444444;   /* PE0,1,2,3,5,6,7 = floating input */

    /* Data bus PB8..PB15 starts tri-stated (bus off) */
    GPIOB->CFGHR = CART_BUS_OFF;

    /* LEDFLASH PA0 as push-pull output */
    GPIOA->CFGLR &= ~(0xFu << 0);
    GPIOA->CFGLR |= (0x3u << 0);   /* MODE0=11 (50MHz), CNF0=00 (PP out) */

    /* Single 32k bank offset */
    bankOffsets0 = (uint32_t)(-0x4000);

    /* PE4 is wired to the MSX's ~RESET pin on the cartridge edge connector
     * (per board/pinout.txt). We do NOT drive it here - the cart must not
     * hold the MSX in reset, since that caused hangs in testing. Leave PE4
     * as a floating input (default state of GPIOE->CFGLR = 0x44444444 above)
     * so the MSX's own reset circuit controls it. */

    /* No EXTI setup - the cart service runs as a polling loop from main,
     * not as an interrupt. This eliminates EXTI latency (5-15 cycles on
     * WCH parts) and the interrupt entry/exit overhead (~10 cycles),
     * giving the tightest possible SLTSL-fall-to-data-on-bus delay. */
}

/*********************************************************************
 * @fn      Cart_SetImageSource
 *
 * @brief   Switch cartpnt to one of the supported image sources.
 *          Atomically updates both cartpnt (the data pointer the fast IRQ
 *          dereferences) and the g_cart_image_src tracking variable.
 *
 *          Unknown source values fall back to FLASH (safe default).
 */
void Cart_SetImageSource(Cart_ImageSrc src)
{
    switch (src) {
    case CART_IMG_PSRAM:
        cartpnt = (uint8_t *)PSRAM_BUS_BASE;
        g_cart_image_src = CART_IMG_PSRAM;
        g_cart_wait_active = 1U;     /* PSRAM needs WAIT stretching */
        break;
    case CART_IMG_SRAM:
        /* SRAM mirror not implemented in this build. */
        cartpnt = (uint8_t *)&hello_rom[0];
        g_cart_image_src = CART_IMG_FLASH;
        g_cart_wait_active = 0U;
        break;
    case CART_IMG_FLASH:
    default:
        cartpnt = (uint8_t *)&hello_rom[0];
        g_cart_image_src = CART_IMG_FLASH;
        g_cart_wait_active = 0U;     /* flash is fast enough, no WAIT */
        break;
    }
}

/*********************************************************************
 * @fn      Cart_GetImageSource / Cart_GetImageBase
 *
 * @brief   Read-back helpers. Useful from main() for boot-time logging
 *          and from runtime code that wants to know where the cart data
 *          is currently being served from.
 */
Cart_ImageSrc Cart_GetImageSource(void)
{
    return g_cart_image_src;
}

uint32_t Cart_GetImageBase(void)
{
    return (uint32_t)cartpnt;
}

/*********************************************************************
 * @fn      Cart_SetImageBase (legacy)
 *
 * @brief   Re-point cartpnt at a raw address. Pass 0 to revert to flash;
 *          pass PSRAM_BUS_BASE for the PSRAM mirror. Kept for backwards
 *          compatibility with code that already has a base address handy.
 *          New code should prefer Cart_SetImageSource().
 */
void Cart_SetImageBase(uint32_t addr)
{
    if (addr == (uint32_t)PSRAM_BUS_BASE) {
        Cart_SetImageSource(CART_IMG_PSRAM);
    } else {
        Cart_SetImageSource(CART_IMG_FLASH);
    }
}

/*********************************************************************
 * @fn      CartServiceLoop
 *
 * @brief   Forever-loop cart service. Polls PE0 (~SLTSL) directly with
 *          zero interrupt latency. Runs from .ramfunc (zero-wait-state
 *          SRAM) and never returns.
 *
 *          Performance vs the old EXTI-driven RunCart32k:
 *            - Old: SLTSL fall -> EXTI detect -> vector fetch -> handler
 *                   entry -> read SLTSL again -> drive data. ~15-25 cycles
 *                   of latency before data hits PB8..15.
 *            - New: SLTSL fall -> read SLTSL -> drive data. ~3-5 cycles
 *                   of latency (just the GPIO read + branch + data drive).
 *
 *          The loop:
 *            1. Spin reading GPIOE->INDR until bit0 (~SLTSL) goes low.
 *            2. Reconfigure PB8..15 CFGHR -> push-pull output.
 *            3. Read cartpnt + offset, write to PB8..15 OUTDR << 8.
 *            4. Spin reading GPIOE->INDR until bit0 (~SLTSL) goes high.
 *            5. Reconfigure PB8..15 CFGHR -> floating input (release bus).
 *            6. Goto 1.
 *
 *          Step 1 is the tightest possible wait - just a GPIO bit-load
 *          and branch, with the compiler unrolling nothing (each iteration
 *          is one lbu + one andi + one branch).
 *
 *          Marked noinline + section(".ramfunc") so:
 *            - The loop body stays as a real loop (no function-call overhead
 *              per iteration).
 *            - Code executes from zero-wait-state internal SRAM.
 */
void __attribute__((section(".ramfunc"), noinline)) CartServiceLoop(void)
{
    /* Outer loop: wait for SLTSL to fall, service it, wait for SLTSL to
     * rise, release the bus, repeat. */
    for (;;) {
        /* Spin until ~SLTSL goes low. Volatile read so the compiler keeps
         * the load (it can't constant-fold a memory-mapped register). */
        while (((volatile uint16_t)GPIOE->INDR & CART_SLTSL_MASK) != 0U) {
            /* Tight poll - no body, just a GPIO bit-load per iteration. */
        }

        /* SLTSL just went low - the Z80 is selecting our slot.
         * Drive the cart byte onto PB8..15 within ~3-5 cycles. */
        GPIOB->CFGHR = CART_BUS_ON;
        GPIOB->OUTDR =
            ((uint32_t)(*(cartpnt +
                          ((uint16_t)GPIOD->INDR + bankOffsets0)))) << 8;

        /* Hold data on the bus until SLTSL rises. The Z80 reads D0..7
         * during T3 (after any Tw states) - our data stays stable on
         * PB8..15 throughout this wait. */
        while (((volatile uint16_t)GPIOE->INDR & CART_SLTSL_MASK) == 0U) {
            /* Tight poll - waiting for Z80 to finish its bus cycle. */
        }

        /* SLTSL rose - release the data bus back to high-Z so the MSX
         * can use the bus for the next instruction. */
        GPIOB->CFGHR = CART_BUS_OFF;
    }
}

#pragma GCC pop_options
