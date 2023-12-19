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
 */
void Clock_SetPLL(uint32_t nPll_M, uint32_t nPll_N, uint32_t nPll_P, Clock_Source_e eClockSource)
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
