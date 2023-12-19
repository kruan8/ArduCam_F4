/*
 * clock_f4.h
 *
 *  Created on: 20. 12. 2023
 *  Author:     Priesol Vladimir
 */

#ifndef CLOCK_F4_H_
#define CLOCK_F4_H_

#include "stm32f4xx.h"
#include <stdbool.h>
#include "stm32f4xx_ll_rcc.h"

typedef enum
{
  CLOCK_SOURCE_HSI = LL_RCC_PLLSOURCE_HSI,
  CLOCK_SOURCE_HSE = LL_RCC_PLLSOURCE_HSE
} Clock_Source_e;


/**
 * @brief  Sets the main PLL settings for STM32F4xx device
 * @note   PLL can only be configured when PLL is not used as system clock.
 *            For that purpose, this function does the following things:
 *              - Enables HSI or HSE as system core clock
 *              - Disables PLL
 *              - Sets PLL parameters passed as parameters in function
 *              - Enables PLL
 *              - Waits will PLL is ready and locked
 *              - Enables PLL as system core clock
 *              - Updates system core clock variable
 * @param  nM: Division factor for the main PLL input clock (divider before PLL). This value can be between 2 and 63.
 *         nN: Multiplication factor for VCO. This value can be between 192 and 432.
 *         nP: Main PLL division factor for main system clock (divider behind PLL). This value can be 2, 4, 6 or 8.
 * @retval None
 */
void Clock_SetPLL(uint32_t nM, uint32_t nN, uint32_t nP, Clock_Source_e eClockSource);
void Clock_SetHSI(void);

#endif /* CLOCK_F4_H_ */
