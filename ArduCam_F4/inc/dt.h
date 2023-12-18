/*
 * dt.h
 *
 *  Created on: 25. 3. 2021
 *  Author:     Priesol Vladimir
 */

#ifndef DT_H_
#define DT_H_

#include "stm32f4xx.h"
#include <stdbool.h>

typedef struct
{
  uint8_t sec;
  uint8_t min;
  uint8_t hour;
  uint8_t day;
  uint8_t month;
  uint8_t year;   // 2 digits (00-99)
  int8_t  offsetHours;
} dt_t;

int32_t DT_GetUnixTime(dt_t* data);
void DT_GetDateTimeFromUnix(dt_t* data, uint32_t unix);

#endif /* DT_H_ */
