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
    /* Dump mcause/mepe/mtval before we reset. With PINRSTF present
     * on every boot, this handler isn't currently firing -- but if
     * the next experiment shows SFT in the RST flags, this will tell
     * us exactly which instruction faulted.
     *
     * mcause:
     *   0x00000005 = load access fault
     *   0x00000007 = store access fault
     *   0x00000001 = instruction access fault
     *   0x00000002 = illegal instruction
     */
    uint32_t mcause, mepc, mtval;
    __asm__ volatile("csrr %0, mcause" : "=r"(mcause));
    __asm__ volatile("csrr %0, mepc"   : "=r"(mepc));
    __asm__ volatile("csrr %0, mtval"  : "=r"(mtval));
    /* Use raw register writes instead of printf -- printf itself
     * can fault if the fault was in USART code. Spin a few cycles
     * so the FIFO drains if printf is still usable. */
    volatile uint32_t * const uart = (volatile uint32_t *)0x40013800; /* USART1 base */
    static const char tag[] = "\r\n*** HF mcause=";
    for (int i = 0; tag[i]; i++) {
        while (!(uart[0x0C / 4] & (1U << 7))) { /* wait TXE */ }
        uart[0x04 / 4] = tag[i];                 /* DR */
    }
    /* Hex dump mcause. */
    {
        static const char hex[] = "0123456789abcdef";
        for (int shift = 28; shift >= 0; shift -= 4) {
            while (!(uart[0x0C / 4] & (1U << 7))) { }
            uart[0x04 / 4] = hex[(mcause >> shift) & 0xF];
        }
        for (int i = 0; " mepc="[i]; i++) {
            while (!(uart[0x0C / 4] & (1U << 7))) { }
            uart[0x04 / 4] = " mepc="[i];
        }
        for (int shift = 28; shift >= 0; shift -= 4) {
            while (!(uart[0x0C / 4] & (1U << 7))) { }
            uart[0x04 / 4] = hex[(mepc >> shift) & 0xF];
        }
        for (int i = 0; " mtval="[i]; i++) {
            while (!(uart[0x0C / 4] & (1U << 7))) { }
            uart[0x04 / 4] = " mtval="[i];
        }
        for (int shift = 28; shift >= 0; shift -= 4) {
            while (!(uart[0x0C / 4] & (1U << 7))) { }
            uart[0x04 / 4] = hex[(mtval >> shift) & 0xF];
        }
        for (int i = 0; " ***\r\n"[i]; i++) {
            while (!(uart[0x0C / 4] & (1U << 7))) { }
            uart[0x04 / 4] = " ***\r\n"[i];
        }
    }
    /* Brief delay so the FIFO drains before reset. */
    for (volatile int i = 0; i < 80000; i++) __asm__ volatile("nop");
    NVIC_SystemReset();
    while (1) { }
}

/*********************************************************************
 * USART1 global interrupt (CLI command input over PA10).
 * Delegates byte RX + line assembly + command dispatch to cli.c.
 */
void USART1_IRQHandler(void)
{
  CLI_USART1_Handler();
}


