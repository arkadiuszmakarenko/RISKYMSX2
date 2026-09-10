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

/*
 *@Note
 USART Print debugging routine:
 USART1_Tx(PA9).
 This example demonstrates using USART1(PA9) as a print debug port output.

*/

#include "debug.h"
#include "cart.h"
#include "psram.h"
#include "cli.h"

/* Global typedef */

/* Global define */

/* Global Variable */


/*********************************************************************
 * @fn      main
 *
 * @brief   Main program.
 *
 * @return  none
 */
int main (void) {

    SystemCoreClockUpdate();
    Delay_Init();
    USART_Printf_Init (115200);
    printf ("SystemClk:%d\r\n", SystemCoreClock);
    printf ("ChipID:%08x\r\n", DBGMCU_GetCHIPID());

    /* Zero the SRAM cart mirror so the MSX sees an open-bus (0xFF) pattern
     * until a CLI XLOAD/LOAD command supplies the actual ROM image. Must
     * run BEFORE Init_Cart so the EXTI0 IRQ (if it fires before the
     * XLOAD upload arrives) reads back a deterministic pattern. */
    ROM_Clear();

    /* Start MSX cartridge emulation. Init_Cart configures GPIO, EXTI0,
     * and installs Cart_EXTI0_Handler via the PFIC VTF slot. The cart
     * handler reads the SRAM mirror at SRAM_ROM_BASE on every SLTSL
     * assertion - which is now empty until the CLI uploads a ROM. */
    Init_Cart();

    /* CLI: line-oriented command interface over USART1 (PA9/PA10).
     * Must be after USART_Printf_Init so printf() works. */
    CLI_Init();

    printf ("Cart image: SRAM @ 0x%08x (32 KiB, EMPTY - upload via XLOAD)\r\n",
            (unsigned)Cart_GetImageBase());

    /* Init PSRAM so it's writable from XLOAD_PSRAM. Skip the self-test
     * (slow); we just need the FSMC/PSRAM controller in a writable
     * state. If the chip is missing, XLOAD_PSRAM will simply hang or
     * read garbage. */
    PSRAM_Init();
    printf ("PSRAM bus: 0x%08x ready for writes (no test)\r\n",
            (unsigned)PSRAM_BUS_BASE);

    /* MSX reset is no longer controlled by the cart - Init_Cart() leaves PE4
     * floating so the MSX's own reset circuit handles it. The MSX boots
     * on its own and runs the cart handler whenever ~SLTSL falls. */

    /* Cart service runs from EXTI0_IRQHandler (installed in Init_Cart).
     * Main loop just idles - the IRQ fires on every PE0 edge. The
     * RUN_PSRAM command disables IRQs and jumps to user code, returning
     * here on mret. */
    for (;;) {
        __asm__ volatile ("wfi");
    }
}
