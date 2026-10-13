/********************************** (C) COPYRIGHT *******************************
 * File Name          : main.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2025/06/06
 * Description        : Main program body.
 *
 * Migration notes (NextorV3 merge): the V3 version of this file added
 * Cart_HoldMSXWait_Begin/End around the boot path (to coordinate with the
 * startup asm's PE3 assert), Cart_HardReset* and g_cart_* instrumented
 * counters used by the V3 cart.c. Those V3 cart.c internals are NOT
 * migrated (per request), so this file is adapted:
 *
 *   - The Cart_HoldMSXWait_Begin/End calls around the boot path are kept
 *     because the new startup asm asserts ~WAIT during reset and the
 *     primitives exist in the NextorOnly cart.c.
 *   - The V3 g_cart_* report blocks and the CART_MAPPER_RAM_BACKEND / CART_TIMING_STATS
 *     branches are guarded with #if 0 - they are dead code until the
 *     matching V3 cart.c changes land. Flipping the guard back is the
 *     one-line change needed to reactivate them.
 *   - Cart_HardReset_* references were inside CART_TFMSX_HARD_RESET
 *     guards that default to 0 (see cart.h), so they don't compile in.
 *   - The Loader module was deleted in V3; its only references in this
 *     file (Loader_Service in the service loop) are dropped.
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#include "debug.h"
#include "cart.h"
#include "psram.h"
#include "scc.h"
#include "nextor.h"
#include "raw_disk.h"
#include "terminal.h"
#include "usb_disk.h"
#include "usb_tests.h"

int main (void) {

    /* Synchronize the C-side hold flag with the low PE3 output asserted
     * by startup_ch32v4x7.S. The MSX is held while the terminal mapper
     * and the USB backend are brought up. */
    Cart_HoldMSXWait_Begin ();

    /* Heartbeat: toggle the LED in a tight loop so we can confirm
     * main() is reached even if USART is dead. PA0 = LEDFLASH. */
    {
        RCC_PB2PeriphClockCmd(RCC_PB2Periph_GPIOA, ENABLE);
        GPIOA->CFGLR &= ~(0xFu << 0);
        GPIOA->CFGLR |=  (0x3u << 0);
        for (volatile int i = 0; i < 200000; i++) {
            GPIOA->OUTDR ^= (1u << 0);
        }
    }

    SystemCoreClockUpdate();
    Delay_Init();


    USART_Printf_Init (921600);
    PWR_VDD18LevelConfig(PWR_VDD18_Level1);

    /* Init_Cart configures PE4 as a floating input - the firmware
     * never drives MSX ~RESET (read-only on most MSX2+ machines;
     * driving it can damage the mainboard). The MSX is therefore
     * running while we boot; its BIOS will probe 0x4000 as soon as
     * we enable GPIO clocks. Init_Cart -> Cart_SetMapper(TERMINAL)
     * below gets the terminal cart armed before any cart access
     * can land, so the BIOS reads the terminal ROM and runs the
     * MSX-side terminal program (which copies itself to RAM and
     * polls the firmware's output FIFO).
     *
     * NOTE: the firmware owns ~WAIT (asserted by startup_ch32v4x7.S,
     * cleared by Cart_HoldMSXWait_End() below) while the terminal ROM
     * is installed and the mailbox FIFOs are NOT drained. Terminal and
     * Nextor are flash-ROM mappers and cannot serve a cycle from a held
     * bus (the IRQ needs the cycle to start), so the terminal boot relies
     * on the BIOS either re-reading from a held cycle once we let go or
     * skipping its slot probe of the cart. */
    Init_Cart ();
    SCC_Init ();
    USB_Initialization ();

    /* Reset the terminal mailbox before installing the mapper so the
     * EXTI0 handler starts with empty FIFOs. */
    Terminal_Reset ();

    printf ("SystemClk:%d\r\n", SystemCoreClock);
    printf ("ChipID:%08x\r\n", DBGMCU_GetCHIPID());

    /* Bring up PSRAM. It backs game-image loads. The V3 cart.c adds a
     * "set the slotted mapper RAM ready here" hook gated on
     * CART_MAPPER_RAM_BACKEND - that branch is V3-only and is guarded
     * out below because the matching cart.c code is not migrated. */
    PSRAM_Init();

    /* RawDisk_Init() is a power-on call. Cart_SetMapper(NEXTOR) already
     * initialized the Nextor mailbox before the BIOS started executing;
     * do NOT call Nextor_Init() again here, because the kernel may already
     * have queued its first command and a second reset would discard it.
     * A mapper swap later from the terminal menu performs its own single
     * Nextor_Init() at the swap point. */
    RawDisk_Init ();

#if 0 /* V3 cart.c internals not migrated - re-enable when ported */
#if CART_MAPPER_RAM_BACKEND == CART_MAPPER_RAM_SRAM
    Cart_SetSlottedPSRAMReady (1U);
#endif
#endif

    Cart_SetMapper_Safe (CART_MAP_TERMINAL);
    Delay_Ms(500);

    /* USB and the disk backend are now ready for the first Nextor
     * mailbox request. Release WAIT and let the held BIOS cycle finish. */
    Cart_HoldMSXWait_End ();

    printf ("\r\n=== boot complete (terminal mapper active) ===\r\n");

    /* Idle loop: service both mailboxes. */
    /* The Nextor engine is always live in the background: it drives
     * the USB enumeration, INQUIRY, READ CAPACITY and IDENTIFY state
     * machine once after boot, then idles until the kernel needs a
     * READ/WRITE. Calling Nextor_Service() unconditionally is
     * cheap and harmless outside the NEXTOR mapper. */
    for (;;) {
        Nextor_Service ();
        Terminal_Service ();

#if 0 /* V3 cart.c internals not migrated */
        /* Cycle-end spin timeouts: the cart ISR's ~SLTSL release and
         * RD/WR settle spins are bounded and count here instead of
         * wedging the MCU inside the ISR with the UART stopped
         * mid-line. */
        {
            static uint32_t s_cyc_to_seen = 0U;
            uint32_t cyc_to = g_cart_cycle_timeout;
            if (cyc_to != s_cyc_to_seen) {
                s_cyc_to_seen = cyc_to;
                printf ("CART: cycle-end spin timeout x%lu (SLTSL never rose)\r\n",
                        (unsigned long)cyc_to);
            }
        }

        /* Combined mapper-RAM timing report. */
        {
            enum { CART_MAPPER_LOG_INTERVAL = 1000U };
            static uint32_t s_mar_reads_seen  = 0U;
            static uint32_t s_mar_writes_seen = 0U;
            static uint32_t s_mar_late_seen   = 0U;
            static uint32_t s_mar_wlate_seen  = 0U;
            static uint32_t s_mar_max_seen    = 0U;
            static uint32_t s_mar_wmax_seen   = 0U;

            const uint32_t reads  = g_cart_slotted_reads;
            const uint32_t writes = g_cart_slotted_writes;
            const uint32_t late   = g_cart_slotted_late;
            const uint32_t wlate  = g_cart_slotted_wlate;
            const uint32_t rmax   = g_cart_slotted_cyc_max;
            const uint32_t wmax   = g_cart_slotted_wcyc_max;
            const uint32_t rmin = (g_cart_slotted_cyc_min == 0xFFFFFFFFU)
                                ? 0U : g_cart_slotted_cyc_min;
            const uint32_t wmin = (g_cart_slotted_wcyc_min == 0xFFFFFFFFU)
                                ? 0U : g_cart_slotted_wcyc_min;

            const uint8_t count_due =
                ((reads - s_mar_reads_seen) >= CART_MAPPER_LOG_INTERVAL)
                || ((writes - s_mar_writes_seen) >= CART_MAPPER_LOG_INTERVAL);
            const uint8_t timing_changed =
                (late != s_mar_late_seen) || (wlate != s_mar_wlate_seen)
                || (rmax != s_mar_max_seen) || (wmax != s_mar_wmax_seen);

            if (count_due || timing_changed) {
                printf ("CART: mapper-RAM reads=%lu late=%lu "
                        "rcycles=%lu..%lu; writes=%lu wlate=%lu "
                        "wcycles=%lu..%lu\r\n",
                        (unsigned long)reads,
                        (unsigned long)late,
                        (unsigned long)rmin,
                        (unsigned long)rmax,
                        (unsigned long)writes,
                        (unsigned long)wlate,
                        (unsigned long)wmin,
                        (unsigned long)wmax);
                s_mar_reads_seen  = reads;
                s_mar_writes_seen = writes;
                s_mar_late_seen   = late;
                s_mar_wlate_seen  = wlate;
                s_mar_max_seen    = rmax;
                s_mar_wmax_seen   = wmax;
            }
        }
#endif

        /* No WFI - while the Nextor kernel's driver is polling a
         * register in a tight loop, each cart read generates an EXTI0
         * interrupt. WFI wakes on the interrupt but the ISR overhead +
         * WFI re-entry latency can be too slow for the driver's
         * timeout. Spin instead so Nextor_Service runs immediately
         * after the ISR returns. */
        __asm__ volatile ("nop");
    }
}