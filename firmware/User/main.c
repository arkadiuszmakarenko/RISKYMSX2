/********************************** (C) COPYRIGHT *******************************
 * File Name          : main.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2025/06/06
 * Description        : Main program body.
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#include "debug.h"
#include "cart.h"
#include "psram.h"
#include "scc.h"
#include "loader.h"
#include "nextor.h"
#include "terminal.h"
#include "usb_disk.h"
#include "usb_tests.h"

int main (void) {

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

    /* Run clock diagnostics BEFORE SystemCoreClockUpdate so we can see
     * the raw RCC state. USART_Printf_Init uses SystemCoreClock for
     * baud calc, but at this point we're still on HSI 20 MHz so the
     * baud will be correct (SystemCoreClock default = HSI_VALUE). */
    // ClockTree_Diag();

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
     * polls the firmware's output FIFO). */
    Init_Cart ();
    SCC_Init ();
    /* Reset the terminal mailbox before installing the mapper so the
     * EXTI0 handler starts with empty FIFOs. */
    Terminal_Reset ();
    Cart_SetMapper (CART_MAP_TERMINAL);


    printf ("SystemClk:%d\r\n", SystemCoreClock);
    printf ("ChipID:%08x\r\n", DBGMCU_GetCHIPID());

    /* Bring up PSRAM after the flash selector is already armed. It is
     * only needed later when the user loads a cartridge image. */
    PSRAM_Init();

    /* USBHS host init. Powers up the controller so it's ready. */
    USB_Initialization ();

    printf ("\r\n=== boot complete (terminal mapper active) ===\r\n");

    /* Idle loop: service both mailboxes. The current active mapper
     * is g_mapper; the FLASH loader mailbox only fires its IRQ when
     * the FLASH mapper is installed (the 0x7FF0..0x7FFF window is
     * only decoded by Cart_EXTI0_Flash_Handler), so calling
     * Loader_Service unconditionally is harmless - the have_cmd
     * flag stays 0 and the service returns. Same for
     * Terminal_Service when the TERMINAL mapper is not installed:
     * it only touches its own volatile fields, which the next cart
     * swap to TERMINAL simply discards. */
    /* The Nextor engine is additionally gated on its mapper being
     * active, so it stays fully inert in the terminal workflow and
     * only services the DOS driver's mailbox after the N-key launch
     * installs CART_MAP_NEXTOR. */
    for (;;) {
        if (Cart_GetMapper () == CART_MAP_NEXTOR) {
            /* Debug: report driver-binding activity as it happens. The
             * three counters answer three independent questions about a
             * silent boot:
             *
             *   g_nx_cmd_count  -- mailbox CMD writes (driver issueing
             *                      a command). Stays at 0 when the
             *                      driver never reaches the mailbox.
             *   g_nx_bank_writes + g_nx_last_bank
             *                    -- page-1 bank-select writes
             *                      (ASCII16 CHGBNK). Ticks every
             *                      time the kernel pages a new bank
             *                      into 0x4000-0x7FFF. Driver bank
             *                      is bank 7 - a tick with
             *                      g_nx_last_bank==7 means the
             *                      kernel HAS selected the driver
             *                      bank; if g_nx_cmd_count still
             *                      reads 0 after that, the driver
             *                      is sitting idle (no disk request
             *                      is reaching it yet).
             *   g_nx_media_changes
             *                    -- USB stick insertion/removal
             *                      transitions. The kernel reads
             *                      STAPEEK/STATUS at boot to pick
             *                      up the change latch - if this
             *                      tick happens but cmd_count
             *                      stays 0, the kernel didn't poll
             *                      status on this boot (typical at
             *                      the MSX-BASIC prompt).
             */
            static uint32_t s_last_cmd_count = 0U;
            static uint32_t s_last_attempts = 0U;
            static uint32_t s_last_stat_reads = 0U;
            static uint32_t s_last_bank_count = 0U;
            static uint32_t s_last_media_count = 0U;
            static uint32_t s_last_irq = 0U;
            static uint32_t s_last_late = 0U;
            static uint32_t s_last_zero = 0U;
            static uint32_t s_last_reads = 0U;
            static uint32_t s_last_wr = 0U;
            uint32_t cc = g_nx_cmd_count;
            uint32_t aa = g_nx_cmd_attempts;
            uint32_t ss = g_nx_stat_reads;
            uint32_t bc = g_nx_bank_writes;
            uint32_t mc = g_nx_media_changes;
            uint32_t ie = g_nx_irq_entry;
            uint32_t il = g_nx_irq_late;
            uint32_t iz = g_nx_irq_zero;
            uint32_t rr = g_nx_reads;
            uint32_t ww = g_nx_writes;
            if (cc != s_last_cmd_count || aa != s_last_attempts
                || ss != s_last_stat_reads || bc != s_last_bank_count
                || mc != s_last_media_count || ie != s_last_irq
                || il != s_last_late || iz != s_last_zero
                || rr != s_last_reads || ww != s_last_wr) {
                s_last_cmd_count  = cc;
                s_last_attempts   = aa;
                s_last_stat_reads = ss;
                s_last_bank_count = bc;
                s_last_media_count = mc;
                s_last_irq = ie;
                s_last_late = il;
                s_last_zero = iz;
                s_last_reads = rr;
                s_last_wr = ww;
                printf ("NEXTOR: cmds=%u A=%u S=%u "
                        "irq=%u rd=%u wr=%u l=%u z=%u "
                        "addrs[4/5/6/7/B/F]=%u/%u/%u/%u/%u/%u "
                        "last=0x%04x "
                        "banksel=%u (last=%u, b7sel=%u) "
                        "media_ch=%u (now=%u)\r\n",
                        (unsigned)cc, (unsigned)aa, (unsigned)ss,
                        (unsigned)ie, (unsigned)rr, (unsigned)ww,
                        (unsigned)il, (unsigned)iz,
                        (unsigned)g_nx_addr_counts[0x4],
                        (unsigned)g_nx_addr_counts[0x5],
                        (unsigned)g_nx_addr_counts[0x6],
                        (unsigned)g_nx_addr_counts[0x7],
                        (unsigned)g_nx_addr_counts[0xB],
                        (unsigned)g_nx_addr_counts[0xF],
                        (unsigned)g_nx_last_addr,
                        (unsigned)bc,
                        (unsigned)g_nx_last_bank,
                        (unsigned)g_nx_bank7_selects,
                        (unsigned)mc,
                        (unsigned)g_nx_media_now);
            }
            Nextor_Service ();
        }
        Loader_Service ();
        Terminal_Service ();
        __asm__ volatile ("wfi");
    }
}