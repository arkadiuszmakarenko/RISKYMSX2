#include "cart.h"
#include "psram.h"
#include "ch32v4x7.h"
#include "debug.h"

#pragma GCC push_options
#pragma GCC optimize("Ofast")

/*
 * Cartridge image pointer. Hard-wired to the zero-wait-state internal
 * SRAM mirror of hello_rom[] at SRAM_ROM_BASE. The startup copy loop in
 * startup_ch32v4x7.S already copies hello_rom[] from flash LMA to this
 * SRAM VMA at reset, so the cart handler can read cartpnt directly with
 * no setup needed by main().
 *
 * An 8-bit load from flash has 1-2 wait states while the same load from
 * internal SRAM is 0-wait at 200 MHz, so SRAM is the fastest possible
 * source for the cart handler (PSRAM is also 0-wait but adds bus
 * contention with other peripherals and requires an init step).
 *
 * Declared `restrict` because Cart_EXTI0_Handler is the only reader.
 */
uint8_t *restrict cartpnt = (uint8_t *)SRAM_ROM_BASE;

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
 * @fn      Cart_GetImageBase
 *
 * @brief   Diagnostic accessor. Returns the (constant) base address of
 *          the cart image, which is always SRAM_ROM_BASE. Useful for
 *          boot-time logging from main().
 */
uint32_t Cart_GetImageBase(void)
{
    return (uint32_t)cartpnt;
}

/*********************************************************************
 * @fn      ROM_Clear
 *
 * @brief   Zero the cart ROM mirror in SRAM (32 KiB at SRAM_ROM_BASE).
 *          Called once at boot before the cart IRQ is enabled, so the
 *          MSX sees a deterministic 0xFF byte pattern on every cart
 *          read until an XLOAD upload populates the mirror.
 *
 *          0xFF is the MSX "open bus" pattern for an unpopulated slot,
 *          so a freshly-booted cart behaves like a missing cart to the
 *          MSX's BIOS - the BIOS prints "NO CARTRIDGE" and the user
 *          knows to upload a ROM via the CLI.
 *
 *          Word-sized writes (32 bits at a time) keep this to ~8K
 *          store instructions = ~1 ms at 200 MHz.
 */
void ROM_Clear(void)
{
    volatile uint32_t *p = (volatile uint32_t *)SRAM_ROM_BASE;
    const uint32_t words = CART_ROM_SIZE / 4U;
    for (uint32_t i = 0; i < words; i++) {
        p[i] = 0xFFFFFFFFU;
    }
}

/*********************************************************************
 * @fn      Cart_AssertMSXReset
 *
 * @brief   Pulse the MSX ~RESET line (PE4) low for the requested number
 *          of milliseconds, then release it back to floating input.
 *
 *          ~RESET on the MSX is active-low and must be held low for at
 *          least one full machine cycle (typically a few microseconds)
 *          to be recognised; 100 ms gives a clean cold-boot.
 *
 *          PE4 is normally a floating input (see Init_Cart); we briefly
 *          reconfigure it as push-pull output low, busy-wait, then put
 *          it back to floating.
 *
 *          Cart bus activity continues during the wait - the Z80 inside
 *          the MSX resets, then the BIOS re-reads from our cart.
 */
void Cart_AssertMSXReset(uint32_t ms)
{
    /* Use the standard SDK GPIO helpers so the CFGLR / OUTDR register
     * sequencing is guaranteed correct (no chance of writing OUTDR while
     * the pin is still in analog/floating-input mode and having the
     * write ignored). */

    /* 1. Reconfigure PE4 as push-pull output, 50 MHz, initially high
     *    (inactive). Driving high first avoids any glitch on the line
     *    if it was previously being read as an input. */
    GPIO_InitTypeDef io = {0};
    io.GPIO_Pin   = GPIO_Pin_4;
    io.GPIO_Mode  = GPIO_Mode_Out_PP;
    io.GPIO_Speed = GPIO_Speed_High;   /* ~50 MHz slew */
    GPIO_Init(GPIOE, &io);
    GPIO_SetBits(GPIOE, GPIO_Pin_4);   /* deasserted (high) */

    /* 2. Drive low to assert MSX reset. */
    GPIO_ResetBits(GPIOE, GPIO_Pin_4);

    /* 3. Crude busy-wait using SysTick ticks. We're inside the USART1
     *    IRQ (priority 0x40), so the cart EXTI0 (priority 0) preempts
     *    freely, but SysTick is one of the few timers we can use without
     *    extra config. Delay_Ms uses SysTick. */
    Delay_Ms(ms);

    /* 4. Release the line - drive high then switch back to floating
     *    input so the MSX's own reset circuit takes over. */
    GPIO_SetBits(GPIOE, GPIO_Pin_4);

    /* Switch PE4 back to floating input (the cart is no longer driving
     * the MSX reset line). */
    io.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOE, &io);
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
