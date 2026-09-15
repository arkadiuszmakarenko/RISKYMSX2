/********************************** (C) COPYRIGHT *******************************
* File Name          : ch32v4x7_it.c
* Author             : WCH
* Version            : V1.0.0
* Date               : 2025/12/01
* Description        : Main Interrupt Service Routines.
*********************************************************************************
* Copyright (c) 2025 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#include "ch32v4x7_it.h"
#include "core_riscv.h"

void NMI_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void HardFault_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

/* Captured on HardFault; printed in main() before any other init. */
volatile uint32_t g_fault_mepc   = 0;
volatile uint32_t g_fault_mcause = 0;
volatile uint32_t g_fault_mtval  = 0;
volatile uint32_t g_fault_sp     = 0;
volatile uint32_t g_fault_ra     = 0;

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
  /* Capture fault context BEFORE any print / reset so we can diagnose. */
  g_fault_mepc   = __get_MEPC();
  g_fault_mcause = __get_MCAUSE();
  g_fault_mtval  = __get_MTVAL();
  __asm volatile ("mv %0, sp" : "=r" (g_fault_sp));
  __asm volatile ("mv %0, ra" : "=r" (g_fault_ra));

  NVIC_SystemReset();
  while (1)
  {
  }
}


