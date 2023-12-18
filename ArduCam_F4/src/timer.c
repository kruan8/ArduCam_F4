/*
 * timer.c
 *
 *  Created on: 24. 6. 2016
 *      Author: priesolv
 */

#include "timer.h"
#include "common_f4.h"

#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_rcc.h"

#define TIMER_US      TIM9         // basic timer for us interval
#define TIMER_US_CLK  LL_APB2_GRP1_PERIPH_TIM9

#define TIMER_CALLBACKS     5

static PtrSysTickCallback g_pSysTickCallback[TIMER_CALLBACKS] = { 0 };
static uint8_t g_nCallbacks = 0;

static volatile uint32_t g_nTicks = 0;

void Timer_Init(void)
{
  Timer_SystickUpdate();
  TimerUs_init();
}

void Timer_SystickUpdate(void)
{
  SystemCoreClockUpdate();

  // set Systick to 1ms
  if (SysTick_Config(SystemCoreClock / 1000))
  {
    /* Capture error */
    while (1);
  }
}

void Timer_Delay_ms(uint32_t delay_ms)
{
  uint32_t nEndTime = g_nTicks + delay_ms;
  while (g_nTicks < nEndTime);
}

uint32_t Timer_GetTicks_ms()
{
  return g_nTicks;
}

bool Timer_SetSysTickCallback(PtrSysTickCallback pSysTickCallback)
{
  if (g_nCallbacks < TIMER_CALLBACKS)
  {
    g_pSysTickCallback[g_nCallbacks++] = pSysTickCallback;
    return true;
  }

  while (1);
  return false;
}

void SysTick_Handler(void)
{
  g_nTicks++;

  for (uint8_t i = 0; i < TIMER_CALLBACKS; i++)
  {
    if (g_pSysTickCallback[i])
    {
      g_pSysTickCallback[i]();
    }
  }
}

// timer for us counting
void TimerUs_init(void)
{

  // Enable clock for TIM_US
  LL_APB2_GRP1_EnableClock(TIMER_US_CLK);
  Timer_UsStart();
}

void Timer_UsStart(void)
{
  LL_RCC_ClocksTypeDef RCC_Clocks;
  LL_RCC_GetSystemClocksFreq(&RCC_Clocks);

  TIMER_US->PSC = RCC_Clocks.PCLK2_Frequency / 1000000; // 1 MHz
  TIMER_US->CNT = 0;
  TIMER_US->EGR = TIM_EGR_UG;
  TIMER_US->CR1 |= TIM_CR1_CEN;
}

uint16_t Timer_UsGet_microseconds(void)
{
    return TIMER_US->CNT;
}

void Timer_UsDdelay(uint16_t microseconds)
{
  TIMER_US->CNT = 0;
  while (TIMER_US->CNT < microseconds);
}

void Timer_UsClear(void)
{
  TIMER_US->CNT = 0;
}

void Timer_UsStop(void)
{
  TIMER_US->CR1 &= ~TIM_CR1_CEN;
}
