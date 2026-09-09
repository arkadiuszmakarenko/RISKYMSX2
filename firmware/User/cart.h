#ifndef __CART_H
#define __CART_H

#include "ch32v4x7.h"

/*
 * CH32V467VET6 (RISKYMSX2) MSX cartridge bus pinout.
 *
 *   PD0..PD15   A0..A15   (16-bit address bus)  -> GPIOD->INDR
 *   PB8..PB15   D0..D7    (8-bit data bus)      -> GPIOB high byte
 *
 *   PE0  (pin 97)  ~SLTSL   slot select          -> EXTI0 trigger
 *   PE1  (pin 98)  RD
 *   PE2  (pin  1)  WR
 *   PE5  (pin  4)  MREQ
 *
 *   PA0  (pin 23)  LEDFLASH (status LED)
 */

/* Control-bus bit masks within GPIOE->INDR */
#define CART_SLTSL_MASK   0x0001   /* PE0 */
#define CART_RD_MASK      0x0002   /* PE1 */
#define CART_WR_MASK      0x0004   /* PE2 */
#define CART_MREQ_MASK    0x0020   /* PE5 */

/* Data bus drive config for GPIOB CFGHR (pins 8..15).
 * 0x3 nibble = 50MHz push-pull output, 0x4 nibble = floating input. */
#define CART_BUS_ON       0x33333333
#define CART_BUS_OFF      0x44444444

void Init_Cart(void);
void RunCart32k(void) __attribute__((interrupt("WCH-Interrupt-fast")));

#endif
