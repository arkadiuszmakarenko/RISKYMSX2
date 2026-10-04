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
#include "raw_disk.h"
#include "terminal.h"
#include "usb_disk.h"
#include "usb_tests.h"

int main (void) {

    /* Synchronize the C-side hold flag with the low PE3 output asserted
     * by startup_ch32v4x7.S. The MSX is held while the Nextor mapper and
     * the USB backend are brought up. */
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
  //  Terminal_Reset ();
    /* Preserve the pending BIOS probe edge while atomically installing
     * the Nextor handler. The handler serves the held ROM read and
     * returns without waiting for SLTSL. */
    Cart_SetMapper_Safe (CART_MAP_SLOTTED);


    printf ("SystemClk:%d\r\n", SystemCoreClock);
    printf ("ChipID:%08x\r\n", DBGMCU_GetCHIPID());

    /* Bring up PSRAM after the cart mapper is already armed. It backs the
     * expanded-slot mapper RAM (sub-slot 1, 4 MiB at
     * CART_SLOTTED_BANK_BASE_OFFSET) and every PSRAM-backed cart mapper.
     * The result MUST be published with Cart_SetSlottedPSRAMReady():
     * until it is, the EXTI0 handler treats sub-slot 1 as an empty slot
     * and leaves the data bus floating, which makes the BIOS/Nextor report
     * "slot is expanded but nothing inside". */
    {
        const uint8_t psram_status = PSRAM_Init ();
        Cart_SetSlottedPSRAMReady (psram_status == PSRAM_OK);
        if (psram_status != PSRAM_OK) {
            printf ("PSRAM init failed: status 0x%02X - sub-slot 1 stays "
                    "empty\r\n", psram_status);
        } else {
            printf ("PSRAM ready: mapper RAM window at PSRAM+0x%08X "
                    "(%u x %u KiB)\r\n",
                    (unsigned)(PSRAM_CART_BASE
                               + CART_SLOTTED_BANK_BASE_OFFSET),
                    (unsigned)(CART_SLOTTED_BANK_COUNT),
                    (unsigned)(CART_SLOTTED_BANK_SIZE / 1024U));
        }
    }

    /* USBHS host init. Powers up the controller so it's ready. */
    USB_Initialization ();

    /* RawDisk_Init() is a power-on call. Cart_SetMapper(NEXTOR) already
     * initialized the Nextor mailbox before the BIOS started executing;
     * do NOT call Nextor_Init() again here, because the kernel may already
     * have queued its first command and a second reset would discard it.
     * A mapper swap later from the terminal menu performs its own single
     * Nextor_Init() at the swap point. */
    RawDisk_Init ();

    /* USB and the disk backend are now ready for the first Nextor
     * mailbox request. Release WAIT and let the held BIOS cycle finish. */
    Cart_HoldMSXWait_End ();

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
    /* The Nextor engine is always live in the background: it drives
     * the USB enumeration, INQUIRY, READ CAPACITY and IDENTIFY state
     * machine once after boot, then idles until the kernel needs a
     * READ/WRITE. Calling Nextor_Service() unconditionally is
     * cheap and harmless outside the NEXTOR mapper - the lifecycle
     * state machine parks itself in USB_READY, the in-flight
     * READ/WRITE check returns immediately (state == IDLE), and the
     * only real work it does while NEXTOR is not installed is a
     * single USBH_PreDeal() call. Once the user swaps to
     * CART_MAP_NEXTOR the kernel has already-enumerated stick data
     * waiting. */
    for (;;) {
        Nextor_Service ();
        Loader_Service ();
        Terminal_Service ();

        /* No WFI — while the Nextor kernel's driver is polling a
         * register in a tight loop, each cart read generates an EXTI0
         * interrupt. WFI wakes on the interrupt but the ISR overhead +
         * WFI re-entry latency can be too slow for the driver's
         * timeout. Spin instead so Nextor_Service runs immediately
         * after the ISR returns. */
        __asm__ volatile ("nop");
    }
}
