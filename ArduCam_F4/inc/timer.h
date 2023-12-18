/*
 * timer.h
 *
 *  Created on: 24. 6. 2016
 *      Author: priesolv
 */

#ifndef TIMER_H_
#define TIMER_H_

#include "stm32f4xx.h"
#include <stdbool.h>

typedef void(*PtrSysTickCallback) (void);

void Timer_Init();
void Timer_SystickUpdate(void);
void Timer_Delay_ms(uint32_t delay_ms);
uint32_t Timer_GetTicks_ms();
bool Timer_SetSysTickCallback(PtrSysTickCallback pFunction);

void TimerUs_init(void);
void Timer_UsStart(void);
uint16_t Timer_UsGet_microseconds(void);
void Timer_UsDdelay(uint16_t microseconds);
void Timer_UsClear(void);
void Timer_UsStop(void);

#endif /* TIMER_H_ */
