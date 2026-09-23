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
#include "sunrise_ide.h"
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

    /* Initialise the Sunrise IDE state machine so a swap to
     * CART_MAP_SUNRIDE from the terminal menu (N key) starts with
     * a clean ATA register file + PATA device signature. The actual
     * USB enumeration is driven by Sunrise_IDE_Service() from the
     * main loop; this only zeroes the in-RAM state struct. */
    Sunrise_IDE_Init ();

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
    /* The Sunrise IDE engine is always live in the background: it drives
     * the USB enumeration, INQUIRY, READ CAPACITY and IDENTIFY state
     * machine once after boot, then idles until the kernel needs a
     * READ/WRITE. Calling Sunrise_IDE_Service() unconditionally is
     * cheap and harmless outside the SUNRIDE mapper - the lifecycle
     * state machine parks itself in USB_READY, the in-flight
     * READ/WRITE check returns immediately (state == IDLE), and the
     * only real work it does while SUNRIDE is not installed is a
     * single USBH_PreDeal() call. Once the user swaps to
     * CART_MAP_SUNRIDE the kernel has already-enumerated stick data
     * waiting. */
    for (;;) {
        Sunrise_IDE_Service ();
        Loader_Service ();
        Terminal_Service ();
        __asm__ volatile ("wfi");
    }
}