/*
 * rtc.h
 *
 *  Created on: 7. 11. 2016
 *      Author: priesolv
 */

#ifndef RTCF4_H_
#define RTCF4_H_

#include "stm32f4xx.h"
#include "dt.h"
#include <stdbool.h>

typedef enum
{
  rtc_clock_lse,
  rtc_clock_lsi
} rtc_clock_e;

typedef struct
{
  uint8_t second: 4;
  uint8_t second10: 4;
  uint8_t minute: 4;
  uint8_t minute10: 4;
  uint8_t hour: 4;
  uint8_t hour10: 4;
} rtc_time_t;

typedef struct
{
  uint8_t day: 4;
  uint8_t day10: 4;
  uint8_t month: 4;
  uint8_t month10: 1;
  uint8_t week_day: 3;
  uint8_t year: 4;
  uint8_t year10: 4;
} rtc_date_t;

//typedef struct
//{
//  uint8_t sec;
//  uint8_t min;
//  uint8_t hour;
//  uint8_t day;
//  uint8_t month;
//  uint8_t year;   // 2 digits (00-99)
//  int8_t  offsetHours;
//} rtc_record_time_t;

typedef void (*RTC_OnWakeUp)(void);

bool RTCF4_Init(rtc_clock_e eClock, RTC_OnWakeUp pOnWakeUp);
void RTCF4_Set(dt_t *dt, bool bDate, bool bTime);
void RTCF4_Get(dt_t *dt);
int8_t RTCF4_GetTimeZone(void);
void RTCF4_SetWakeUp(uint16_t nInterval_s);
void RTC_WriteAccess(bool bEnable);

uint32_t RTCF4_GetNowUnixTimeStamp(void);
int32_t RTCF4_GetUnixTimeStamp(dt_t* data);
void RTCF4_GetDateTimeFromUnix(dt_t* data, uint32_t unix);

uint8_t RTCF4_ByteToBcd2(uint8_t Value);
uint8_t RTCF4_Bcd2ToByte(uint8_t Value);

void RTCF4_Test(void);

#endif /* RTCF4_H_ */
