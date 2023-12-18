/*
 * hw.c
 *
 *  Created on: 4. 12. 2019
 *  Author:     Priesol Vladimir
 */

#include "hw.h"
#include <common_f4.h>
#include <timer.h>

#include <stm32f4xx_ll_adc.h>
#include <stm32f4xx_ll_bus.h>

//#include <sys/_stdint.h>

// BLUE LED na kitu
#define BOARD_LED                PC13
#define BOARD_LED_ON             GPIO_RESETPIN(BOARD_LED)
#define BOARD_LED_OFF            GPIO_SETPIN(BOARD_LED)

// CAMERA supply transistor
#define HW_CAM_SUPPLY            PB9
#define HW_CAM_SUPPLY_ON         GPIO_RESETPIN(HW_CAM_SUPPLY)
#define HW_CAM_SUPPLY_OFF        GPIO_SETPIN(HW_CAM_SUPPLY)

#define HW_BAT                   PA0
#define APP_ADC_BAT_INPUT        ADC_CHANNEL_0_NUMBER  // (PA0)
#define APP_ADC_CONV             8       // pocet konverzi pro teplomer

static uint16_t                  g_nVoltage = 0;                 // measured voltage from modem task
static int16_t                   g_nTemp;                        // measured temperature

bool HW_Init(void)
{

 // WDG_Init(WDG_Timeout_32s);

  Timer_Init();

  HW_ReadCPUID();

  // configure board LED
  GPIO_ConfigPin(BOARD_LED, mode_output, outtype_pushpull, pushpull_no, speed_low);
  BOARD_LED_OFF;

  // configure CAM supply pin
  GPIO_ConfigPin(HW_CAM_SUPPLY, mode_analog, outtype_pushpull, pushpull_down, speed_low);

  // configure BAT analog pin
  //GPIO_ConfigPin(HW_BAT, mode_analog, outtype_pushpull, pushpull_down, speed_low);

  HW_AdcInit();

  return true;
}

void HW_SetBoardLed(bool bOn)
{
  bOn ? BOARD_LED_ON : BOARD_LED_OFF;
}

void HW_SetCamSupply(bool bOn)
{
  bOn ? HW_CAM_SUPPLY_ON : HW_CAM_SUPPLY_OFF;
}

void HW_AdcInit(void)
{
  //enable ADC1 clock
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC1);

  // Set ADC clock (conversion clock) common to several ADC instances
  LL_ADC_SetCommonClock(__LL_ADC_COMMON_INSTANCE(), LL_ADC_CLOCK_SYNC_PCLK_DIV2);

  LL_ADC_InitTypeDef ADC_Init;
  ADC_Init.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;
  ADC_Init.Resolution = LL_ADC_RESOLUTION_12B;
  ADC_Init.SequencersScanMode = LL_ADC_SEQ_SCAN_DISABLE;
  LL_ADC_Init(ADC1, &ADC_Init);

  LL_ADC_REG_SetTriggerSource(ADC1, LL_ADC_REG_TRIG_SOFTWARE);

  LL_ADC_REG_SetSequencerLength(ADC1, LL_ADC_REG_SEQ_SCAN_DISABLE);

  LL_APB2_GRP1_DisableClock(LL_APB2_GRP1_PERIPH_ADC1);
}

uint32_t HW_MeasureVrefVoltage(void)
{
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC1);
  LL_ADC_SetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(), LL_ADC_PATH_INTERNAL_VREFINT);

  // sensor startup 10us from datasheet
  volatile uint32_t wait_loop_index = ((LL_ADC_DELAY_VREFINT_STAB_US * (SystemCoreClock / (100000 * 2))) / 10);
  while(wait_loop_index != 0)
  {
    wait_loop_index--;
  }

  LL_ADC_REG_SetSequencerLength(ADC1, LL_ADC_REG_SEQ_SCAN_DISABLE);  // 1 vstup
  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_VREFINT);
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_VREFINT, LL_ADC_SAMPLINGTIME_480CYCLES);

  /*
  VREFINTcal is the only known value. The value of VREFINT, when measured with a known good 3.3V Vdd.
  If you divide data with cal you know how many bits Vdd is off, assuming VREFINTcal is still valid.

  If VREFINTcal is 1000, and data is 1020, this means you've measured VREFINT to be 20 bits higher than it's supposed to be. Indicating a lower Vdd value.
  Hence 1000/1020 = 0.98. A 2% difference. Thus 3.3*0.98 = 3.24 V.

  You are not measuring the change in VREFINT, but the change of the ADC reference.
   */

  uint32_t nAdcValue = HW_AdcMeasure();

  LL_ADC_SetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(), LL_ADC_PATH_INTERNAL_NONE);

  uint16_t nVrefIntCal = *VREFINT_CAL_ADDR;
  uint32_t nVDD = nAdcValue * 10 * VREFINT_CAL_VREF / nVrefIntCal;

  // rounding
  nVDD = (nVDD + 5) / 10;
  return (uint16_t) nVDD;

}

/**
 * Function measures internal sensor temperature
 * @return temperature, resolution 0.1C
 */
int16_t HW_MeasureTemperature(uint16_t nVdda_mv)
{
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC1);
  LL_ADC_SetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(), LL_ADC_PATH_INTERNAL_TEMPSENSOR);

  // sensor startup 10us from datasheet
  volatile uint32_t wait_loop_index = ((LL_ADC_DELAY_TEMPSENSOR_STAB_US * (SystemCoreClock / (100000 * 2))) / 10);
  while(wait_loop_index != 0)
  {
    wait_loop_index--;
  }

  // Set ADC group regular sequence: channel on the selected sequence rank
  LL_ADC_REG_SetSequencerLength(ADC1, LL_ADC_REG_SEQ_SCAN_DISABLE);  // 1 vstup
  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_TEMPSENSOR);
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_TEMPSENSOR, LL_ADC_SAMPLINGTIME_480CYCLES);

  int16_t nTemp = __LL_ADC_CALC_TEMPERATURE(nVdda_mv,  HW_AdcMeasure(), LL_ADC_RESOLUTION_12B);

  LL_ADC_SetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(), LL_ADC_PATH_INTERNAL_NONE);

  g_nTemp = nTemp;
  return nTemp;
}

/**
 * Function measures battery voltage
 * @return voltage, resolution mV
 */
uint16_t HW_MeasureBatVoltage_mV(uint16_t nVdda_mv)
{
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC1);

  LL_ADC_SetChannelSamplingTime(ADC1, APP_ADC_BAT_INPUT, LL_ADC_SAMPLINGTIME_144CYCLES);

  // Set ADC group regular sequence: channel on the selected sequence rank
  LL_ADC_REG_SetSequencerLength(ADC1, LL_ADC_REG_SEQ_SCAN_DISABLE);  // 1 vstup
  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_0);

  uint32_t nValue = HW_AdcMeasure();
  nValue = nValue * nVdda_mv / 4095;

  // resistor divider divides 2
  nValue *= 2;
  g_nVoltage = nValue;
  return (uint16_t)nValue;
}

uint16_t HW_AdcMeasure(void)
{
  LL_ADC_REG_SetFlagEndOfConversion(ADC1, LL_ADC_REG_FLAG_EOC_UNITARY_CONV);
  LL_ADC_Enable(ADC1);

  uint32_t nValue = 0;
  for (uint8_t i = 0; i < APP_ADC_CONV; i++)
  {
    LL_ADC_REG_StartConversionSWStart(ADC1);

    //wait for conversion complete
    while(!LL_ADC_IsActiveFlag_EOCS(ADC1));

    //read ADC value
    nValue += LL_ADC_REG_ReadConversionData12(ADC1);
  }

  LL_ADC_Disable(ADC1);
  LL_APB2_GRP1_DisableClock(LL_APB2_GRP1_PERIPH_ADC1);

  nValue /= APP_ADC_CONV;
  return (uint16_t)nValue;
}

uint16_t HW_GetVoltage_mV(void)
{
  return g_nVoltage;
}

int16_t HW_GetTemperature(void)
{
  return g_nTemp;
}

//void HW_ChangeClock(hw_clock_e eAHBPresc)
//{
//  RCC_HCLKConfig(eAHBPresc);
//  Timer_SystickUpdate();
//}

void HW_ReadCPUID(void)
{
//  uint32_t w0 = LL_GetUID_Word0();
//  uint32_t w1 = LL_GetUID_Word1();
//  uint32_t w2 = LL_GetUID_Word2();
}
