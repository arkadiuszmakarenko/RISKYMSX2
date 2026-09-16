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
#include "usb_disk.h"
#include <string.h>

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
    PWR_VDD18LevelConfig (PWR_VDD18_Level1);
    SystemCoreClockUpdate();

    Delay_Init();
    USART_Printf_Init (921600);
    printf ("SystemClk:%d\r\n", SystemCoreClock);
    printf ("ChipID:%08x\r\n", DBGMCU_GetCHIPID());

    // PSRAM_Init();
    //  Init_Cart();
    //  CLI_Init();
    USB_Initialization();


    for (;;) {
        CLI_Service();
        __asm__ volatile ("wfi");
    }
}
