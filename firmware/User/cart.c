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

    /* Route PE0 -> EXTI0 and fire on the falling edge (SLTSL is active-low).
     * Rising edge also fires - the handler releases the data bus on rise.
     * Both triggers must be enabled for the two-phase serve/release cycle. */
    AFIO->EXTICR[0] = (AFIO->EXTICR[0] & ~(0xFU << 0)) |
                       (AFIO_EXTICR1_EXTI0_PE << 0);
    EXTI->INTENR  = (EXTI->INTENR  & ~EXTI_INTENR_MR0) | EXTI_INTENR_MR0;
    EXTI->RTENR   = (EXTI->RTENR   & ~EXTI_RTENR_TR0)  | EXTI_RTENR_TR0;
    EXTI->FTENR   = (EXTI->FTENR   & ~EXTI_FTENR_TR0)  | EXTI_FTENR_TR0;
    EXTI->INTFR   = EXTI_INTENR_MR0;  /* clear any stale pending bit */

    /* Install Cart_EXTI0_Handler via the PFIC VTF (Vector-Table-Free) slot.
     * The PFIC stores our handler's address in VTFADDR[0] and dispatches
     * directly to it when EXTI0 fires - no vector-table fetch, no
     * indirect jump through flash. Faster than the vector path.
     *
     * Three things all need to be true for VTF dispatch to fire:
     *   1. Cart_EXTI0_Handler() is marked
     *      __attribute__((interrupt("WCH-Interrupt-fast"))) so the
     *      prologue/epilogue save mepc/mstatus and return via mret.
     *      Without it the IRQ fires once and then the return PC is garbage.
     *   2. SetVTFIRQ(...,ENABLE) writes the VTF slot (addr in VTFADDR[0],
     *      IRQn in VTFIDR[0]). num=0 selects slot 0.
     *   3. The IRQ is ALSO enabled in NVIC->IENR via NVIC_EnableIRQ().
     *      VTF alone does NOT gate the IRQ - it only sets the dispatch
     *      address. Without NVIC_EnableIRQ the pending bit never latches.
     *   4. Global IRQs are enabled at the PFIC top level via __enable_irq(),
     *      which writes 0x88 to PFIC.SCTLR. The QingKe V4 core has a
     *      separate top-level gate from the per-IRQ enable.
     *
     * Belt-and-suspenders: set both VTF dispatch AND NVIC enable, then
     * open the global gate. */
    SetVTFIRQ((uint32_t)Cart_EXTI0_Handler, EXTI0_IRQn, 0, ENABLE);
    NVIC_EnableIRQ(EXTI0_IRQn);
    NVIC_SetPriority(EXTI0_IRQn, 0x00);  /* highest priority */
    __enable_irq();
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
 * @fn      Cart_EXTI0_Handler
 *
 * @brief   EXTI0 IRQ: serves one MSX cartridge bus read on the falling
 *          edge of ~SLTSL (PE0), and releases the data bus on the rising
 *          edge. Runs from .ramfunc (zero-wait-state SRAM) and is
 *          installed as the EXTI0 IRQ vector (PFIC priority 0).
 *
 *          Falling edge (~SLTSL low -> Z80 selected our slot):
 *            1. Reconfigure PB8..15 CFGHR -> push-pull output.
 *            2. Read cartpnt + offset, write to PB8..15 OUTDR << 8.
 *
 *          Rising edge (~SLTSL high -> Z80 finished its bus cycle):
 *            3. Reconfigure PB8..15 CFGHR -> floating input (release bus).
 *
 *          Marked noinline + section(".ramfunc") so the body stays in
 *          zero-wait-state internal SRAM and is not call-clobbered.
 *
 *          Also marked __attribute__((interrupt("WCH-Interrupt-fast")))
 *          so the prologue saves/restore the right CSR state when the
 *          PFIC dispatches us via the VTF (vector-table-free) slot.
 *          Without this attribute, VTF dispatch returns into garbage
 *          and the IRQ fires once then hangs.
 */
void __attribute__((section(".ramfunc"), noinline,
                    interrupt("WCH-Interrupt-fast")))
Cart_EXTI0_Handler(void)
{
    /* Read SLTSL state at entry. Falling-edge fires when SLTSL goes low,
     * rising-edge fires when SLTSL goes high. Branch on the current level. */
    if ((GPIOE->INDR & CART_SLTSL_MASK) == 0U) {
        /* SLTSL fell - drive the addressed cart byte onto PB8..15. */
        GPIOB->CFGHR = CART_BUS_ON;
        GPIOB->OUTDR =
            ((uint32_t)(*(cartpnt +
                          ((uint16_t)GPIOD->INDR + bankOffsets0)))) << 8;
    } else {
        /* SLTSL rose - release the data bus back to high-Z. */
        GPIOB->CFGHR = CART_BUS_OFF;
    }

    /* Clear the EXTI0 pending bit so the next edge can fire. Writing 1
     * to INTFR bit 0 clears it (WCH PFIC-style edge-triggered EXTI). */
    EXTI->INTFR = EXTI_INTENR_MR0;
}

#pragma GCC pop_options
