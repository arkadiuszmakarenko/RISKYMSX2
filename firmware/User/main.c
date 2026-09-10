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

    /* Start MSX cartridge emulation (ROM32k hello ROM on slot-select). */
    Init_Cart();

    /* Bring up PSRAM and mirror hello_rom[32768] into it at PSRAM_BUS_BASE.
     * Always use the PSRAM mirror - no flash fallback, no comparison. */
    PSRAM_Init();
    Cart_SetImageSource(CART_IMG_PSRAM);
    printf ("PSRAM: mirror=0x%08x, cart reads from PSRAM\r\n",
            (unsigned)PSRAM_GetRomMirrorBase());

    /* MSX reset is no longer controlled by the cart - Init_Cart() leaves PE4
     * floating so the MSX's own reset circuit handles it. The MSX boots
     * on its own and runs the cart handler whenever ~SLTSL falls. */

    CartServiceLoop();
}
