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
#include <string.h>

/* Temporary embedded cart ROM image from MSXSoftware/HelloWorld/hello.bin.
 * 32 KiB, flash-resident. */
extern unsigned char hello_rom[];
extern unsigned int hello_rom_len;

/* The embedded ROM is still linked (psram.c references hello_rom for its
 * size check) but is NO LONGER auto-loaded at boot: Boot_LoadEmbeddedRom()
 * is disabled so the MSX drops to BASIC unless a cart image is uploaded
 * via XLOAD. To re-enable booting the embedded ROM, mirror hello_rom[]
 * into PSRAM and install ROM32k here. */
__attribute__ ((unused)) static void Boot_LoadEmbeddedRom (void) {
    if (hello_rom_len >= PSRAM_CART_SIZE) {
        printf ("Cart image: PSRAM @ 0x%08x (placeholder; upload via XLOAD)\r\n",
                (unsigned)PSRAM_CART_BASE);
        return;
    }
    /* Copy hello_rom -> PSRAM at offset 0. PSRAM_Init() already
     * mirrors the image; we only need to do it again if PSRAM_Init
     * failed and we're falling back to PSRAM (which should never
     * happen here since the caller checks the status first). */
    if (PSRAM_GetRomMirrorBase() != 0U) {
        memcpy ((void *)PSRAM_CART_BASE, hello_rom, hello_rom_len);
        printf ("Cart image: PSRAM @ 0x%08x (32 KiB, embedded hello ROM)\r\n",
                (unsigned)PSRAM_CART_BASE);
    } else {
        printf ("Cart image: PSRAM not initialised; embedded ROM NOT mirrored\r\n");
    }
}



int main (void) {

    /* Run clock diagnostics BEFORE SystemCoreClockUpdate so we can see
     * the raw RCC state. USART_Printf_Init uses SystemCoreClock for
     * baud calc, but at this point we're still on HSI 20 MHz so the
     * baud will be correct (SystemCoreClock default = HSI_VALUE). */
    // ClockTree_Diag();

    SystemCoreClockUpdate();
    Delay_Init();

    /* Arm the flash-resident ROM before the slower peripheral
     * bring-up so the MSX can see it on the initial power-on boot. */
    Init_Cart ();
    SCC_Init ();
    int loader_rc = Cart_SetMapper (CART_MAP_ROM32k);

    USART_Printf_Init (921600);
    PWR_VDD18LevelConfig(PWR_VDD18_Level1);
    printf ("SystemClk:%d\r\n", SystemCoreClock);
    printf ("ChipID:%08x\r\n", DBGMCU_GetCHIPID());

    /* Bring up PSRAM after the flash selector is already armed. It is
     * only needed later when the user loads a cartridge image. */
    uint8_t psram_status = PSRAM_Init();
    if (psram_status == PSRAM_OK) {
        printf ("PSRAM: OK, cart image @ 0x%08x (%u bytes)\r\n",
                (unsigned)PSRAM_CART_BASE, (unsigned)PSRAM_CART_SIZE);
    } else {
        printf ("PSRAM: init FAILED status=0x%02x\r\n",
                (unsigned)psram_status);
    }

    /* Boot straight into the ROM selector from flash. The ROM32k path
     * serves selector_rom[] for 0x4000..0xBFFF and also exposes the
     * mailbox window used by the MSX-side loader. */
    if (loader_rc == 0) {
        printf ("Cart: ROM32k mapper installed "
                "(MSX boots into the ROM selector)\r\n");
    } else {
        printf ("Cart: ROM32k install FAILED\r\n");
    }

    /* USBHS host init. Powers up the controller so it's ready. */
    USB_Initialization ();
    printf ("USBHS: host controller ready\r\n");

    /* CLI: USART1 command interface.  Commands are dispatched from the
     * main loop (CLI_Service below), never from the IRQ. */
    CLI_Init ();

    /* Now that the firmware is fully initialized, reset the MSX so the
     * BIOS sees the flash selector on its first boot pass. */
    if (loader_rc == 0) {
        Cart_AssertMSXReset (100);
    }

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