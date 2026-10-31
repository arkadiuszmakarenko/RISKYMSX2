/********************************** (C) COPYRIGHT *******************************
 * File Name          : main.c
 * Description        : RISKYMSX2 minimal boot firmware (firmware2).
 *
 * Boots the cartridge straight into the Nextor kernel mapper and keeps
 * it running. Ported from /firmware main.c with everything non-essential
 * removed: no UART console (silent build), no mapper menu, no SCC, no
 * terminal, no loader. The boot sequence is:
 *
 *   1. RawDisk_Init            - zero the disk module (before anything
 *                                 touches the mailbox)
 *   2. Init_Cart               - GPIO/EXTI wiring; the no-mapper
 *                                 placeholder handler is armed (~SLTSL
 *                                 cycles get a floating bus, 0xFF)
 *   3. Cart_SetMapper(NEXTOR)  - installs the Nextor handler and
 *                                 completes the mapper/mailbox state:
 *                                 the kernel ROM is served from flash
 *                                 and the MSX can fetch from 0x4000
 *   4. PSRAM_Init              - brings up the mapper-RAM backing store;
 *                                 sub-slot 1 stays hidden until it passes
 *   5. Nextor_ClockInit        - mailbox timing calibration
 *   6. USB_Initialization      - USBHS host + MSC/SCSI engine
 *   7. spin on Nextor_Service  - drains mailbox disk requests from the
 *                                 Z80 driver (READ/WRITE/CAPACITY/...)
 *
 * This is the first working firmware2 port's boot order, with the
 * serve-latency diagnostics (CART_TIMING_STATS / CART_LA_MARKER)
 * available as build options - see cart.c.
 *
 * Startup never asserts the MSX ~RESET or ~WAIT lines: the cart handler
 * is installed early and the MSX keeps running while PSRAM and USB
 * initialize.
 *
 * Startup never asserts the MSX ~WAIT line: the cart handler is
 * installed early and the MSX keeps running while PSRAM and USB
 * initialize.
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#include "debug.h"
#include "ch32v4x7.h"
#include "cart.h"
#include "nextor.h"
#include "psram.h"
#include "raw_disk.h"
#include "usb_disk.h"

int main (void) {

    SystemCoreClockUpdate();
    Delay_Init();

#if !RISKY_QUIET
    /* Debug-build UART console (USART1 TX @ 921600). Silent builds
     * (QUIET=1, the default) never touch the UART and printf is
     * compiled out at link time. */
    USART_Printf_Init (921600);
#endif
#if RISKY_DEBUG
    printf ("RISKYMSX2 boot: HCLK=%lu\r\n", (unsigned long)SystemCoreClock);
#endif

    /* Keep the unserved-bus window as short as possible: install the
     * flash-served Nextor ROM before external-memory or USB init. */
    RawDisk_Init ();
    Cart_SetMapperRamReady (0);   /* PSRAM not up yet: sub-slot 1 hidden */
    Init_Cart ();
    const int mapper_rc = Cart_SetMapper (CART_MAP_NEXTOR);
#if RISKY_DEBUG
    printf ("boot: Nextor mapper installed (rc=%d)\r\n", mapper_rc);
#endif

    PWR_VDD18LevelConfig(PWR_VDD18_Level1);

    /* Initialize PSRAM while the Nextor ROM handler remains live. No WAIT
     * is asserted; mapper RAM stays hidden until initialization passes. */
    {
        const uint8_t psram_rc = PSRAM_Init ();
#if RISKY_DEBUG
        printf ("boot: PSRAM init rc=0x%02x\r\n", (unsigned)psram_rc);
#endif
        if (psram_rc == PSRAM_OK) {
            Cart_SetMapperRamReady (1U);
        }
    }
    Nextor_ClockInit ();
    USB_Initialization ();
#if RISKY_DEBUG
    printf ("boot: USB initialized; entering service loop\r\n");
#endif

    /* Service Nextor's mailbox and USB lifecycle. The engine drives the
     * USB enumeration, INQUIRY, READ CAPACITY and IDENTIFY state machine
     * once after boot, then idles until the kernel needs a READ/WRITE. */
#if RISKY_DEBUG
    uint32_t loop = 0U;
#endif
    for (;;) {
        Nextor_Service ();

#if RISKY_DEBUG
        /* Boot heartbeat: one line every ~4M spins so a frozen firmware
         * is distinguishable from a silent one on the serial console. */
        if (++loop == 4000000U) {
            loop = 0U;
            printf ("alive: mapper=%d req=%u\r\n",
                    (int)Cart_GetMapper (),
                    (unsigned)Nextor_RequestPending ());
#if CART_TIMING_STATS
            /* PSRAM serve counts. Latency itself is measured externally
             * with the CART_LA_MARKER pulse on a logic analyser - timing
             * instrumentation on the hot path breaks bus serving. */
            printf ("MAPPER: reads=%lu writes=%lu\r\n",
                    (unsigned long)g_cart_slotted_reads,
                    (unsigned long)g_cart_slotted_writes);
#endif
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
