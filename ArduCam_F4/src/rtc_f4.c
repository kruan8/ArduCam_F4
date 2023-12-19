/*
 * rtc.c
 *
 *  Created on: 7. 11. 2016
 *      Author: priesolv
 */

// implementace pro STM32f411

#include "rtc_f4.h"
#include "timer.h"

#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_pwr.h"
#include "stm32f4xx_ll_rcc.h"
#include "stm32f4xx_ll_rtc.h"

#include <string.h>
#include <stdio.h>

#define RTC_CLOCK_SOURCE_LSE

#define RTC_TR_RESERVED_MASK       ((uint32_t)0x007F7F7F)
#define RTC_DR_RESERVED_MASK       ((uint32_t)0x00FFFF3F)

#define RTC_BCD2BIN(x)             ((((x) >> 4) & 0x0F) * 10 + ((x) & 0x0F))
#define RTC_CHAR2NUM(x)            ((x) - '0')
#define RTC_CHARISNUM(x)           ((x) >= '0' && (x) <= '9')

static int8_t                      g_nTimeZone_h;

void _Enter_RTC_InitMode(void);
void _Exit_RTC_InitMode(void);
void _WaitForSynchro_RTC(void);

bool RTCF4_Init(rtc_clock_e eClock, RTC_OnWakeUp pOnWakeUp)
{
  /* Enable the PWR clock */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  /* As the LSE is in the Backup domain and write access is denied to
     this domain after reset, you have to enable write access using
     PWR_BackupAccessCmd(ENABLE) function before to configure the LSE */
  LL_PWR_EnableBkUpAccess();

  /* Enable the LSE OSC */
  LL_RCC_LSE_Enable();

  /* Wait till LSE is ready */
  uint32_t nEndTime = Timer_GetTicks_ms() + 1000;     // wait 1 second for LSE
  while(LL_RCC_LSE_IsReady() == 0)
  {
    if (Timer_GetTicks_ms() > nEndTime)
    {
      return false;
    }
  }

  /* Select the RTC Clock Source */
  LL_RCC_SetRTCClockSource(LL_RCC_RTC_CLKSOURCE_LSE);

  /* Enable the RTC Clock */
  LL_RCC_EnableRTC();

  LL_RTC_DisableWriteProtection(RTC);
  _Enter_RTC_InitMode();

  LL_RTC_TIME_SetFormat(RTC, LL_RTC_TIME_FORMAT_AM_OR_24);
  LL_RTC_SetAsynchPrescaler(RTC, 0x7F);
  if (eClock == rtc_clock_lse)
  {
    // set prescaler, 32768/128 => 256 Hz/256 => 1Hz
    LL_RTC_SetSynchPrescaler(RTC, 0xFF);
  }
  else
  {
    // set prescaler, 38000/128 => 296 Hz/296 => 1Hz
    LL_RTC_SetSynchPrescaler(RTC, 0x127);
  }

  /* Indicator for the RTC configuration */
  LL_RTC_BAK_SetRegister(RTC, LL_RTC_BKP_DR0, 0x32F2);

  LL_RTC_WAKEUP_Enable(RTC);
  _Exit_RTC_InitMode();
//  LL_PWR_DisableBkUpAccess();

//  RTCF4_Test();
  return true;
}

/**
  * @brief  Enter in initialization mode
  * @note In this mode, the calendar counter is stopped and its value can be updated
  * @param  None
  * @retval RTC_ERROR_NONE if no error
  */
void _Enter_RTC_InitMode(void)
{
  /* Set Initialization mode */
  LL_RTC_EnableInitMode(RTC);

  /* Check if the Initialization mode is set */
  while (LL_RTC_IsActiveFlag_INIT(RTC) != 1)
  {
  }

}

/**
  * @brief  Exit Initialization mode
  * @param  None
  * @retval RTC_ERROR_NONE if no error
  */
void _Exit_RTC_InitMode(void)
{
  LL_RTC_DisableInitMode(RTC);

  /* Wait for synchro */
  /* Note: Needed only if Shadow registers is enabled           */
  /*       LL_RTC_IsShadowRegBypassEnabled function can be used */
  _WaitForSynchro_RTC();
}

/**
  * @brief  Wait until the RTC Time and Date registers (RTC_TR and RTC_DR) are
  *         synchronized with RTC APB clock.
  * @param  None
  * @retval RTC_ERROR_NONE if no error (RTC_ERROR_TIMEOUT will occur if RTC is
  *         not synchronized)
  */
void _WaitForSynchro_RTC(void)
{
  /* Clear RSF flag */
  LL_RTC_ClearFlag_RS(RTC);

  /* Wait the registers to be synchronised */
  while(LL_RTC_IsActiveFlag_RS(RTC) != 1)
  {
  }
}

void RTCF4_SetWakeUp(uint16_t nInterval_s)
{
  RTC_WriteAccess(true);
  LL_RTC_WAKEUP_Disable(RTC);
  while(!LL_RTC_IsActiveFlag_WUTW(RTC)) // wait for enabled
  {
    /* add time out here for a robust application */
  }

  LL_RTC_ClearFlag_WUT(RTC);
  LL_RTC_WAKEUP_SetAutoReload(RTC, nInterval_s - 1);  // WUTR je delicka, takže 0 znamena 1 impulz
  LL_RTC_WAKEUP_SetClock(RTC, LL_RTC_WAKEUPCLOCK_CKSPRE);  // clock source
  LL_RTC_EnableIT_WUT(RTC);                   // enable INT
  LL_RTC_WAKEUP_Enable(RTC);

  // Disable write access
  RTC_WriteAccess(false);

  EXTI->IMR |= EXTI_IMR_IM20;       // unmask line 20
  EXTI->RTSR |= EXTI_RTSR_TR20;     // Rising edge for line 20
  NVIC_SetPriority(RTC_WKUP_IRQn, 0);    // Set priority
  NVIC_EnableIRQ(RTC_WKUP_IRQn);         // Enable RTC_IRQn
}

// Enable/disable write access to RTC registers
void RTC_WriteAccess(bool bEnable)
{
  if (bEnable)
  {
    // Enable write in RTC domain control register
    PWR->CR |= PWR_CR_DBP;
    // Enable write access
    LL_RTC_DisableWriteProtection(RTC);
  }
  else
  {
    // Disable write access
    LL_RTC_EnableWriteProtection(RTC);

    PWR->CR &= ~PWR_CR_DBP;
  }
}

void RTCF4_Set(dt_t *dt, bool bDate, bool bTime)
{
  LL_PWR_EnableBkUpAccess();
  LL_RTC_DisableWriteProtection(RTC);

  _Enter_RTC_InitMode();

  if (bDate)
  {
    RTC->DR = RTCF4_ByteToBcd2(dt->year) << 16 | 1 << 13 | RTCF4_ByteToBcd2(dt->month) << 8 | RTCF4_ByteToBcd2(dt->day);
  }

  if (bTime)
  {
    g_nTimeZone_h = dt->offsetHours;
    RTC->TR = RTCF4_ByteToBcd2(dt->hour) << 16 | RTCF4_ByteToBcd2(dt->min) << 8 | RTCF4_ByteToBcd2(dt->sec);
  }

  _Exit_RTC_InitMode();

  LL_RTC_EnableWriteProtection(RTC);

  LL_PWR_DisableBkUpAccess();
}

void RTCF4_Get(dt_t *dt)
{
  uint32_t value = (uint32_t)(RTC->TR & RTC_TR_RESERVED_MASK);

  dt->hour = (uint8_t)(RTC_BCD2BIN((value >> 16) & 0x3F));
  dt->min = (uint8_t)(RTC_BCD2BIN((value >> 8) & 0x7F));
  dt->sec = (uint8_t)(RTC_BCD2BIN(value & 0x7F));

  value = (uint32_t)(RTC->DR & RTC_DR_RESERVED_MASK);
  dt->year = (uint8_t)(RTC_BCD2BIN((value >> 16) & 0xFF));
  dt->month = (uint8_t)(RTC_BCD2BIN((value >> 8) & 0x1F));
  dt->day = (uint8_t)(RTC_BCD2BIN(value & 0x3F));
}

uint32_t RTCF4_GetNowUnixTimeStamp(void)
{
  dt_t dt;

  RTCF4_Get(&dt);
  return DT_GetUnixTime(&dt);
}

int8_t RTCF4_GetTimeZone(void)
{
  return g_nTimeZone_h;
}

uint8_t RTCF4_ByteToBcd2(uint8_t Value)
{
  uint8_t bcdhigh = 0;

  while (Value >= 10)
  {
    bcdhigh++;
    Value -= 10;
  }

  return  ((uint8_t)(bcdhigh << 4) | Value);
}

/**
  * @brief  Convert from 2 digit BCD to Binary.
  * @param  Value: BCD value to be converted.
  * @retval Converted word
  */
uint8_t RTCF4_Bcd2ToByte(uint8_t Value)
{
  uint8_t tmp = 0;
  tmp = ((uint8_t)(Value & (uint8_t)0xF0) >> (uint8_t)0x4) * 10;
  return (tmp + (Value & (uint8_t)0x0F));
}



//void RTCF4_IRQHandler(void)
//{
//  // Check WUT flag
//  if(RTC->ISR & RTC_ISR_WUTF)
//  {
//    RTC->ISR =~ RTC_ISR_WUTF; /* Reset Wake up flag */
//    EXTI->PR = EXTI_PR_PR20; /* clear exti line 20 flag */
//  }
//}

void RTCF4_Test(void)
{
  // ---------- Test RTC ---------------
  dt_t dt;

  dt.day = 15;
  dt.month = 11;
  dt.year = 16;
  dt.hour = 18;
  dt.min = 25;
  dt.sec = 0;
  RTCF4_Set(&dt, true, true);
  RTCF4_Get(&dt);
  uint32_t t = RTCF4_GetUnixTimeStamp(&dt);
  RTCF4_GetDateTimeFromUnix(&dt, t);
}
