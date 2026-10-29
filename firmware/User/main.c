/********************************** (C) COPYRIGHT *******************************
 * File Name          : main.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2025/06/06
 * Description        : Main program body.
 *
 * The cartridge boots directly into the Nextor kernel mapper. Startup never
 * asserts the MSX ~WAIT line: the cart handler is installed early and the
 * MSX keeps running while PSRAM and USB initialize.
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#include "debug.h"
#include "cart.h"
#include "nextor.h"
#include "psram.h"
#include "raw_disk.h"
#include "usb_disk.h"

int main (void) {

    SystemCoreClockUpdate();
    Delay_Init();

    /* Keep the unserved-bus window as short as possible: install the
     * flash-served Nextor ROM before UART setup or external-memory init. */
    RawDisk_Init ();
    Cart_SetMapperRamReady (CART_MAPPER_RAM_BACKEND == CART_MAPPER_RAM_SRAM);
    Init_Cart ();
    const int mapper_rc = Cart_SetMapper (CART_MAP_NEXTOR);

    USART_Printf_Init (921600);
    PWR_VDD18LevelConfig(PWR_VDD18_Level1);

    if (mapper_rc != 0) {
        printf ("ERROR: could not install Nextor mapper\r\n");
    }

#if CART_MSX_RESET_PULSE
    /* Optional: cold-start the MSX from a known instant so a logic
     * analyser sees the boot from its first Z80 cycle. Holds ~RESET,
     * prompts for CART_MSX_RESET_ARM_DELAY_MS so the capture can be
     * armed, then releases it. Off by default - only safe on hardware
     * where the cart edge ~RESET is drivable (see cart.h). */
    Cart_MSXReset_PulseForCapture ();
#endif

    printf ("SystemClk:%d\r\n", SystemCoreClock);
    printf ("ChipID:%08x\r\n", DBGMCU_GetCHIPID());

    /* Initialize PSRAM while the Nextor ROM handler remains live. No WAIT
     * is asserted; mapper RAM stays hidden until initialization passes. */
#if CART_MAPPER_RAM_BACKEND == CART_MAPPER_RAM_PSRAM
    {
        const uint8_t psram_rc = PSRAM_Init ();
        if (psram_rc == PSRAM_OK) {
            Cart_SetMapperRamReady (1U);
        } else {
            printf ("PSRAM mapper RAM disabled (init rc=0x%02x)\r\n",
                    (unsigned)psram_rc);
        }
    }
#endif
    Nextor_ClockInit ();
    USB_Initialization ();

    printf ("\r\n=== startup complete (Nextor mapper active; MSX was not held) ===\r\n");

    /* Service Nextor's mailbox and USB lifecycle. The engine drives
     * the USB enumeration, INQUIRY, READ CAPACITY and IDENTIFY state
     * machine once after boot, then idles until the kernel needs a
     * READ/WRITE. */
    for (;;) {
        Nextor_Service ();

        /* Expanded-slot diagnostics: report when activity changes so a
         * hang can be attributed to the SSR value and per-page banks. */
        {
            extern volatile uint32_t g_subslot_ssr_reads;
            extern volatile uint32_t g_subslot_ssr_writes;
            extern volatile uint8_t  g_subslot_last_ssr_wr;
            extern volatile uint32_t g_subslot_ram_reads[4];
            extern volatile uint32_t g_subslot_ram_writes[4];
            extern volatile uint8_t  g_subslot_last_bank[4];
            static uint32_t s_prev_ssr_rd, s_prev_ssr_wr;
            static uint32_t s_prev_rr[4], s_prev_rw[4];
            static uint32_t s_loops = 0U;
            if (++s_loops >= 2000000U) {
                s_loops = 0U;
                const uint8_t changed =
                    (g_subslot_ssr_reads != s_prev_ssr_rd)
                 || (g_subslot_ssr_writes != s_prev_ssr_wr)
                 || (g_subslot_ram_reads[0] != s_prev_rr[0])
                 || (g_subslot_ram_reads[1] != s_prev_rr[1])
                 || (g_subslot_ram_reads[2] != s_prev_rr[2])
                 || (g_subslot_ram_reads[3] != s_prev_rr[3])
                 || (g_subslot_ram_writes[0] != s_prev_rw[0])
                 || (g_subslot_ram_writes[1] != s_prev_rw[1])
                 || (g_subslot_ram_writes[2] != s_prev_rw[2])
                 || (g_subslot_ram_writes[3] != s_prev_rw[3]);
                {
                    extern volatile uint32_t g_iorq_edges;
                    extern volatile uint32_t g_iorq_fc_edges;
                    extern volatile uint8_t  g_iorq_last_port;
                    static uint32_t s_prev_edges = 0xFFFFFFFFU;
                    if (g_iorq_edges != s_prev_edges) {
                        s_prev_edges = g_iorq_edges;
                        if (g_iorq_edges > 0U) {
                            printf ("IORQ: edges=%lu fc..ff=%lu lastport=%02x\r\n",
                                    (unsigned long)g_iorq_edges,
                                    (unsigned long)g_iorq_fc_edges,
                                    (unsigned)g_iorq_last_port);
                        }
                    }
                }
                if (changed) {
                    printf ("SSR: rd=%lu wr=%lu last=%02x "
                            "bank[0..3]=%02x,%02x,%02x,%02x "
                            "rr=%lu,%lu,%lu,%lu rw=%lu,%lu,%lu,%lu\r\n",
                            (unsigned long)g_subslot_ssr_reads,
                            (unsigned long)g_subslot_ssr_writes,
                            (unsigned)g_subslot_last_ssr_wr,
                            (unsigned)g_subslot_last_bank[0],
                            (unsigned)g_subslot_last_bank[1],
                            (unsigned)g_subslot_last_bank[2],
                            (unsigned)g_subslot_last_bank[3],
                            (unsigned long)g_subslot_ram_reads[0],
                            (unsigned long)g_subslot_ram_reads[1],
                            (unsigned long)g_subslot_ram_reads[2],
                            (unsigned long)g_subslot_ram_reads[3],
                            (unsigned long)g_subslot_ram_writes[0],
                            (unsigned long)g_subslot_ram_writes[1],
                            (unsigned long)g_subslot_ram_writes[2],
                            (unsigned long)g_subslot_ram_writes[3]);
                    s_prev_ssr_rd = g_subslot_ssr_reads;
                    s_prev_ssr_wr = g_subslot_ssr_writes;
                    for (int i = 0; i < 4; i++) {
                        s_prev_rr[i] = g_subslot_ram_reads[i];
                        s_prev_rw[i] = g_subslot_ram_writes[i];
                    }
                }
            }
        }

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
