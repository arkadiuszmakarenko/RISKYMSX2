#include "cart.h"

#pragma GCC push_options
#pragma GCC optimize("Ofast")

/*
 * Hardcoded cartridge image pointer.
 * Points at the 32 KiB hello.bin payload compiled into flash (hello_rom.c).
 */
extern const uint8_t hello_rom[];
static const uint8_t *restrict cartpnt = &hello_rom[0];

/*
 * ROM32k: a single 32 KiB page mapped at Z80 0x4000..0xBFFF.
 * bankOffsets[0] = -0x4000 converts a Z80 address straight into the
 * linear ROM index (address + offset).
 */
static uint32_t bankOffsets0;

/*********************************************************************
 * @fn      Init_Cart
 *
 * @brief   Configure GPIO + EXTI for the MSX cartridge bus and install
 *          the fast-interrupt handler for slot-select.
 */
void Init_Cart(void)
{
    /* Enable GPIO + AFIO clocks */
    RCC_PB2PeriphClockCmd(RCC_PB2Periph_GPIOA |
                          RCC_PB2Periph_GPIOB |
                          RCC_PB2Periph_GPIOD |
                          RCC_PB2Periph_GPIOE |
                          RCC_PB2Periph_AFIO, ENABLE);

    /* Address bus PD0..PD15 as inputs (default 0x44444444 = floating input).
     * Explicitly drive both config regs so nothing is left in an
     * alternate-function state after reset. */
    GPIOD->CFGLR = 0x44444444;   /* A0..A7   */
    GPIOD->CFGHR = 0x44444444;   /* A8..A15  */

    /* Control bus PE0(SLTSL) PE1(RD) PE2(WR) PE5(MREQ) as inputs */
    GPIOE->CFGLR = 0x44444444;   /* PE0..PE7 */

    /* Data bus PB8..PB15 starts tri-stated (bus off) */
    GPIOB->CFGHR = CART_BUS_OFF;

    /* LEDFLASH PA0 as push-pull output */
    GPIOA->CFGLR &= ~(0xFu << 0);
    GPIOA->CFGLR |= (0x3u << 0);   /* MODE0=11 (50MHz), CNF0=00 (PP out) */

    /* Single 32k bank offset */
    bankOffsets0 = (uint32_t)(-0x4000);

    /* ~SLTSL on PE0 -> EXTI line 0, falling edge */
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOE, GPIO_PinSource0);

    EXTI_InitTypeDef EXTI_InitStructure = {0};
    EXTI_InitStructure.EXTI_Line    = EXTI_Line0;
    EXTI_InitStructure.EXTI_Mode    = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    NVIC_EnableIRQ(EXTI0_IRQn);
    /* Vector Table Free interrupt: jump straight to the handler with no
     * software dispatch. The handler lives in .ramfunc (SRAM). */
    SetVTFIRQ((uint32_t)RunCart32k, EXTI0_IRQn, 0, ENABLE);
}

/*********************************************************************
 * @fn      RunCart32k
 *
 * @brief   Fast-interrupt on ~SLTSL falling edge.
 *          Executes from zero-wait-state SRAM (.ramfunc).
 */
void __attribute__((section(".ramfunc"))) RunCart32k(void)
{
    if (((uint16_t)GPIOE->INDR & CART_SLTSL_MASK) == 0) {
        GPIOB->CFGHR = CART_BUS_ON;
        /* Read upper data byte pins (PB8..PB15) -> shift into D0..D7 */
        GPIOB->OUTDR =
            ((uint32_t)(*(cartpnt +
                          ((uint16_t)GPIOD->INDR + bankOffsets0)))) << 8;
        EXTI->INTFR = EXTI_Line0;
        while ((GPIOE->INDR & CART_SLTSL_MASK) == 0) { };
        GPIOB->CFGHR = CART_BUS_OFF;
        return;
    }
    EXTI->INTFR = EXTI_Line0;
}

#pragma GCC pop_options
