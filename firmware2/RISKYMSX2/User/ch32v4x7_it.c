/********************************** (C) COPYRIGHT *******************************
* File Name          : ch32v4x7_it.c
* Author             : WCH
* Version            : V1.0.0
* Date               : 2025/12/01
* Description        : Main Interrupt Service Routines.
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#include "ch32v4x7_it.h"
#include "cli.h"

void NMI_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void HardFault_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void USART1_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

/*********************************************************************
 * @fn      NMI_Handler
 *
 * @brief   This function handles NMI exception.
 *
 * @return  none
 */
void NMI_Handler(void)
{
  while (1)
  {
  }
}

/*********************************************************************
 * @fn      HardFault_Handler
 *
 * @brief   This function handles Hard Fault exception.
 *
 * @return  none
 */
void HardFault_Handler(void)
{
    /* Read trap CSRs before doing anything that might clobber them. */
    uint32_t cause, epc, val, ra;
    __asm__ volatile(
        "csrr %0, mcause\n"
        "csrr %1, mepc\n"
        "csrr %2, mtval\n"
        "mv %3, ra\n"
        : "=r"(cause), "=r"(epc), "=r"(val), "=r"(ra)
    );
    printf("\n[HARDFAULT] cause=%08lx epc=%08lx mtval=%08lx ra=%08lx\n",
           cause, epc, val, ra);
    /* Optional: breakpoint / spin so you can attach the debugger. */
    while (1) { __asm__ volatile ("wfi"); }
}

/*********************************************************************
 * USART1 global interrupt (CLI command input over PA10).
 * Delegates byte RX + line assembly + command dispatch to cli.c.
 */
void USART1_IRQHandler(void)
{
  CLI_USART1_Handler();
}


