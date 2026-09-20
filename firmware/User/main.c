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
#include "cli.h"
#include "scc.h"
#include "loader.h"
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

    /* Hold the MSX in reset IMMEDIATELY so its BIOS waits while the
     * firmware finishes booting. The cart must be fully armed before
     * the MSX sees its first rising edge on ~RESET, otherwise the
     * BIOS probes 0x4000 with no slot active and falls through to
     * BASIC. Cart_AssertMSXReset_Begin drives PE4 low; we keep it low
     * through the rest of boot and only release at the end. */
   // Cart_AssertMSXReset_Begin ();
    Init_Cart ();
    SCC_Init ();
    Cart_SetMapper (CART_MAP_FLASH);


    printf ("SystemClk:%d\r\n", SystemCoreClock);
    printf ("ChipID:%08x\r\n", DBGMCU_GetCHIPID());

    /* Bring up PSRAM after the flash selector is already armed. It is
     * only needed later when the user loads a cartridge image. */
    PSRAM_Init();

    /* USBHS host init. Powers up the controller so it's ready. */
    USB_Initialization ();

    /* CLI: USART1 command interface.  Commands are dispatched from the
     * main loop (CLI_Service below), never from the IRQ. */
    CLI_Init ();

    Cart_AssertMSXReset_End ();

    printf ("\r\n=== boot complete ===\r\n");

    /* Idle loop: service CLI commands + the ROM-loader mailbox, sleep
     * between interrupts.  Loader_Service drains any command the MSX
     * posted through the cart mailbox (dir listing, file load, mapper
     * switch, reset). */
    for (;;) {
        CLI_Service ();
        Loader_Service ();
        __asm__ volatile ("wfi");
    }
}