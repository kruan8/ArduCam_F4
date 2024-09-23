/*
 * clock_f4.c
 *
 *  Created on: 20. 12. 2023
 *  Author:     Priesol Vladimir
 */


#include "clock_f4.h"
#include "stm32f4xx_ll_system.h"
#include "stm32f4xx_ll_pwr.h"
#include "stm32f4xx_ll_bus.h"
#include "string.h"

/**
 * Set PLL as SYSCLK
 *
 * @param nPll_M - Division factor for the main PLL (PLL) input clock
 *                 Caution: The software has to set these bits correctly to ensure that the VCO input frequency
 *                          ranges from 1 to 2 MHz (recomended 1MHz). It is recommended to select a frequency of 2 MHz to limit PLL jitter.
 * @param nPll_N - Main PLL (PLL) multiplication factor for VCO
 *                 Caution: The software has to set these bits correctly to ensure that the VCO output
 *                          frequency is between 100 and 432 MHz.
 * @param nPll_P - Main PLL (PLL) division factor for main system clock
 *                 Caution: The software has to set these bits correctly not to exceed 100 MHz on this domain.
 *                          PLL output clock frequency = VCO frequency / PLLP with PLLP = 2, 4, 6, or 8
 * @param eClockSource - HSI/HSE clock to select
 */
void Clock_SetPLLasSysClk(uint32_t nPll_M, uint32_t nPll_N, uint32_t nPll_P, Clock_Source_e eClockSource)
{
  uint16_t nTimeout;

  LL_RCC_PLL_Disable();
  if (eClockSource == CLOCK_SOURCE_HSI)
  {
    LL_RCC_HSI_Enable();

    /* Wait till HSI is ready */
    nTimeout = 0xFFFF;
    while (!(LL_RCC_HSI_IsReady()) && nTimeout--);

    LL_RCC_PLL_SetMainSource(LL_RCC_PLLSOURCE_HSI);
  }

  if (eClockSource == CLOCK_SOURCE_HSE)
  {
    LL_RCC_HSE_Enable();

    /* Wait till HSI is ready */
    nTimeout = 0xFFFF;
    while (!(LL_RCC_HSE_IsReady()) && nTimeout--);

    LL_RCC_PLL_SetMainSource(LL_RCC_PLLSOURCE_HSE);
  }

  LL_RCC_PLL_ConfigDomain_SYS(eClockSource, nPll_M, nPll_N, (((nPll_P >> 1) - 1) << RCC_PLLCFGR_PLLP_Pos));

  // if core CLK > 84, set scale1
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
  LL_APB1_GRP1_DisableClock(LL_APB1_GRP1_PERIPH_PWR);

  LL_RCC_PLL_Enable();

  /* Wait till PLL is ready */
  nTimeout = 0xFFFF;
  while (!LL_RCC_PLL_IsReady() && nTimeout--);

  // latency for CLK 90-100 MHz and VCC 2.7 - 3.6V
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_3);

  LL_FLASH_EnablePrefetch();
  LL_FLASH_EnableInstCache();
  LL_FLASH_EnableDataCache();

  /* Enable PLL as main clock */
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

  /* Update system core clock variable */
  SystemCoreClockUpdate();
}

/**
 * Configure PLL_I2S
 *
 * @param nPll_M - Division factor for the audio PLL (PLLI2S) input clock
 *                  Caution: The software has to set these bits correctly to ensure that the VCO input frequency
 *                           ranges from 1 to 2 MHz.It is recommended to select a frequency of 2 MHz to limit PLL jitter.
 * @param nPll_N - PLLI2S multiplication factor for VCO
 *                 Caution: The software has to set these bits correctly to ensure that the VCO output
 *                          frequency is between 100 and 432 MHz. With VCO input frequency ranges from 1
 *                          to 2 MHz (refer to Figure 13 and divider factor M of the RCC PLL configuration register (RCC_PLLCFGR))
 * @param nPll_R - PLLI2S division factor for I2S clocks
 *                 Caution: The I2Ss requires a frequency lower than or equal to 192 MHz to work correctly.
 *                          I2S clock frequency = VCO frequency / PLLR with 2 <= PLLR <= 7
 */
void Clock_SetPLL_I2S(uint32_t nPll_M, uint32_t nPll_N, uint32_t nPll_R)
{
  uint16_t nTimeout;

//  LL_RCC_PLLI2S_ConfigDomain_I2S(eClockSource, nPll_M, nPll_N, (((nPll_P >> 1) - 1) << RCC_PLLCFGR_PLLP_Pos));

  LL_RCC_PLLI2S_Enable();

  /* Wait till PLL is ready */
  nTimeout = 0xFFFF;
  while (!LL_RCC_PLLI2S_IsReady() && nTimeout--);
}

/**
 * Set HSI (16 MHz) as SYSCLK
 */
void Clock_SetHSI(void)
{
  uint16_t nTimeout;

  LL_RCC_HSI_Enable();

  /* Wait till HSI is ready */
  nTimeout = 0xFFFF;
  while (!(LL_RCC_HSI_IsReady()) && nTimeout--);

  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSI);

  LL_RCC_PLL_Disable();

  LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);

  LL_FLASH_DisablePrefetch();
  LL_FLASH_DisableInstCache();
  LL_FLASH_DisableDataCache();

  // if core CLK > 84, set scale1
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE3);
  LL_APB1_GRP1_DisableClock(LL_APB1_GRP1_PERIPH_PWR);
}
