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

    /* Assert the cartridge ~WAIT hold before clock, UART, or peripheral
     * initialization. The MSX may already be leaving reset while the MCU
     * starts; delaying this until after USART setup lets the BIOS get past
     * the first slot-probe cycles. */
    Cart_HoldMSXWait_Begin ();


    /* Run clock diagnostics BEFORE SystemCoreClockUpdate so we can see
     * the raw RCC state. USART_Printf_Init uses SystemCoreClock for
     * baud calc, but at this point we're still on HSI 20 MHz so the
     * baud will be correct (SystemCoreClock default = HSI_VALUE). */
    // ClockTree_Diag();

    SystemCoreClockUpdate();
    Delay_Init();


    USART_Printf_Init (921600);
    PWR_VDD18LevelConfig(PWR_VDD18_Level1);

    /* Hold the Z80 with the cartridge ~WAIT input while the GPIO/mapper
     * path is installed. Release it only after the slotted cart is ready. */

   // printf ("WAIT: asserted CFG=%08lx INDR=%08lx\r\n",
   //         (unsigned long)GPIOE->CFGLR,
    //        (unsigned long)GPIOE->INDR);
    Init_Cart ();
    /* Initialize PSRAM before arming the slotted mapper or releasing
     * ~WAIT. The BIOS performs its RAM write/read probe immediately after
     * startup and must never see an uninitialized PSRAM controller. */
    const uint8_t psram_status = PSRAM_Init ();
    printf ("PSRAM startup: status=%02x mirror=%08lx\r\n",
            (unsigned)psram_status,
            (unsigned long)PSRAM_GetRomMirrorBase());
    Cart_SetSlottedPSRAMReady (psram_status == PSRAM_OK);
   // SCC_Init ();
    /* Reset the terminal mailbox before installing the mapper so the
     * EXTI0 handler starts with empty FIFOs. */
   // Terminal_Reset ();
    /* The MSX may still be running while the MCU is reprogrammed. Close the
     * handoff window so a cart cycle cannot land between g_mapper, the VTF
     * address and the freshly reset secondary-slot state. */
    Cart_SetMapper_Safe (CART_MAP_SLOTTED);
    Delay_Ms (100);
    Cart_HoldMSXWait_End ();
    printf ("WAIT: released CFG=%08lx INDR=%08lx\r\n",
            (unsigned long)GPIOE->CFGLR,
            (unsigned long)GPIOE->INDR);


    printf ("SystemClk:%d\r\n", SystemCoreClock);
    printf ("ChipID:%08x\r\n", DBGMCU_GetCHIPID());

    /* USBHS host init. Powers up the controller so it's ready. */
  //  USB_Initialization ();

    /* Initialise the Nextor mapper state so a swap to CART_MAP_NEXTOR
     * from the terminal menu (N key) starts with a clean bank select +
     * ATA register file + PATA device signature. The actual USB
     * enumeration is driven by Nextor_Service() from the main loop;
     * this only primes the in-RAM state struct. */
    //Nextor_Init ();

     printf ("\r\n=== boot complete (slotted mapper active) ===\r\n");

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


 //       Nextor_Service ();
   //     Loader_Service ();
     //   Terminal_Service ();

        /* No WFI — while the Nextor kernel's driver is polling a
         * register in a tight loop, each cart read generates an EXTI0
         * interrupt. WFI wakes on the interrupt but the ISR overhead +
         * WFI re-entry latency can be too slow for the driver's
         * timeout. Spin instead so Nextor_Service runs immediately
         * after the ISR returns. */
        __asm__ volatile ("nop");
    }
}
