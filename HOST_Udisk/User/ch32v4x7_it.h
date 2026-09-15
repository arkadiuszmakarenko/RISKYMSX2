/********************************** (C) COPYRIGHT *******************************
* File Name          : ch32v4x7_it.h
* Author             : WCH
* Version            : V1.0.0
* Date               : 2025/12/01
* Description        : This file contains the headers of the interrupt handlers.
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#ifndef __CH32V4x7_IT_H
#define __CH32V4x7_IT_H

#include "debug.h"

/* HardFault context captured in HardFault_Handler(), printed in main()
 * to identify the faulting instruction.  Declared volatile so the compiler
 * doesn't optimize away the writes from the fault handler. */
extern volatile uint32_t g_fault_mepc;
extern volatile uint32_t g_fault_mcause;
extern volatile uint32_t g_fault_mtval;
extern volatile uint32_t g_fault_sp;
extern volatile uint32_t g_fault_ra;

#endif /* __CH32V4x7_IT_H */


