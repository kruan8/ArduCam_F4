/*
 * dt.c
 *
 *  Created on: 25. 3. 2021
 *  Author:     Priesol Vladimir
 */

#include "dt.h"


#define DT_LEAP_YEAR(year)             ((((year) % 4 == 0) && ((year) % 100 != 0)) || ((year) % 400 == 0))
#define DT_DAYS_IN_YEAR(x)             DT_LEAP_YEAR(x) ? 366 : 365
#define DT_OFFSET_YEAR                 1970
#define DT_SECONDS_PER_DAY             86400
#define DT_SECONDS_PER_HOUR            3600
#define DT_SECONDS_PER_MINUTE          60
#define DT_BCD2BIN(x)                  ((((x) >> 4) & 0x0F) * 10 + ((x) & 0x0F))
#define DT_CHAR2NUM(x)                 ((x) - '0')
#define DT_CHARISNUM(x)                ((x) >= '0' && (x) <= '9')

/* Days in a month */
static const uint8_t DT_Months[2][12] = {
  {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31}, /* Not leap year */
  {31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31}  /* Leap year */
};

int32_t DT_GetUnixTime(dt_t* data)
{
  uint32_t days = 0, seconds = 0;
  uint16_t i;
  uint16_t year = (uint16_t) (data->year + 2000);
  /* Year is below offset year */
  if (year < DT_OFFSET_YEAR) {
    return 0;
  }
  /* Days in back years */
  for (i = DT_OFFSET_YEAR; i < year; i++) {
    days += DT_DAYS_IN_YEAR(i);
  }
  /* Days in current year */
  for (i = 1; i < data->month; i++) {
    days += DT_Months[DT_LEAP_YEAR(year)][i - 1];
  }
  /* Day starts with 1 */
  days += data->day - 1;
  seconds = days * DT_SECONDS_PER_DAY;
  seconds += data->hour * DT_SECONDS_PER_HOUR;
  seconds += data->min * DT_SECONDS_PER_MINUTE;
  seconds += data->sec;

  /* seconds = days * 86400; */
  return seconds;
}

void DT_GetDateTimeFromUnix(dt_t* data, uint32_t unix)
{
  uint16_t year;

  /* Get seconds from unix */
  data->sec = unix % 60;
  /* Go to minutes */
  unix /= 60;
  /* Get minutes */
  data->min = unix % 60;
  /* Go to hours */
  unix /= 60;
  /* Get hours */
  data->hour = unix % 24;
  /* Go to days */
  unix /= 24;

  /* Get week day */
  /* Monday is day one */
  data->day = (unix + 3) % 7 + 1;

  /* Get year */
  year = 1970;
  while (1)
  {
    if (DT_LEAP_YEAR(year))
    {
      if (unix >= 366)
      {
        unix -= 366;
      }
      else
      {
        break;
      }
    }
    else if (unix >= 365)
    {
      unix -= 365;
    }
    else
    {
      break;
    }

    year++;
  }

  /* Get year in xx format */
  data->year = (uint8_t) (year - 2000);

  /* Get month */
  for (data->month = 0; data->month < 12; data->month++)
  {
    if (DT_LEAP_YEAR(year))
    {
      if (unix >= (uint32_t)DT_Months[1][data->month])
      {
        unix -= DT_Months[1][data->month];
      }
      else
      {
        break;
      }
    }
    else if (unix >= (uint32_t)DT_Months[0][data->month])
    {
      unix -= DT_Months[0][data->month];
    }
    else
    {
      break;
    }
  }
  /* Get month */
  /* Month starts with 1 */
  data->month++;
  /* Get date */
  /* Date starts with 1 */
  data->day = unix + 1;
}
