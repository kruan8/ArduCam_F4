/*
 * hw.h
 *
 *  Created on: 4. 12. 2019
 *  Author:     Priesol Vladimir
 */

#ifndef INC_HW_H_
#define INC_HW_H_

#include "stm32f4xx.h"
#include <stdbool.h>

//typedef enum
//{
//  app_clock_ahbPresc1 = RCC_SYSCLK_Div1,
//  app_clock_ahbPresc16 = RCC_SYSCLK_Div16,
//} hw_clock_e;

bool HW_Init(void);
void HW_SetBoardLed(bool bOn);
void HW_SetCamSupply(bool bOn);
//void HW_ChangeClock(hw_clock_e eAHBPresc);

void HW_AdcInit(void);
uint16_t HW_AdcMeasure(void);
int16_t HW_MeasureTemperature(uint16_t nVdda_mv);
uint16_t HW_MeasureBatVoltage_mV(uint16_t nVdda_mv);
uint32_t HW_MeasureVrefVoltage(void);
uint16_t HW_GetVoltage_mV(void);
int16_t HW_GetTemperature(void);
void HW_ConfigureWD(void);
void HW_ReadCPUID(void);

#endif /* INC_HW_H_ */
