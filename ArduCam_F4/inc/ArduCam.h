/*
 * ArduCam.h
 *
 *  Created on: 15. 3. 2022
 *  Author:     Priesol Vladimir
 */

#ifndef ARDUCAM_H_
#define ARDUCAM_H_

#include "stm32f4xx.h"
#include <stdbool.h>

#include "spi_f4.h"
#include "i2c_int.h"

typedef struct {
  uint16_t reg;
  uint16_t val;
} ac_sensor_reg_t;

typedef enum
{
 size_160x120 =   0, //160x120
 size_176x144, //176x144
 size_320x240, //320x240
 size_352x288, //352x288
 size_640x480, //640x480
 size_800x600, //800x600
 size_1024x768, //1024x768
 size_1280x1024, //1280x1024
 size_1600x1200, //1600x1200
} ac_jpeg_size_e;

typedef enum
{
  format_bmp = 0,
  format_jpeg,
  format_raw
} ac_format_e;

typedef enum
{
  ligh_mode_auto = 0,
  ligh_mode_sunny,
  ligh_mode_cloudy,
  ligh_mode_office,
  ligh_mode_home,
} ac_light_mode_e;

typedef enum
{
  saturation2 = 0,
  saturation1,
  saturation0,
  saturation_1,
  saturation_2,
  saturation_sizeof,
} ac_saturation_e;

typedef enum
{
  brightness2 = 0,
  brightness1,
  brightness0,
  brightness_1,
  brightness_2,
  brightness_sizeof,
} ac_brightness_e;

typedef enum
{
  contrast2 = 0,
  contrast1,
  contrast0,
  contrast_1,
  contrast_2,
  contrast_sizeof,
} ac_contrast_e;

void ArduCam_Init(spi_drv_t* pSpi, gpio_pins_e eCS, spi_br_e ePrescaler, i2cdrv_t* pI2c);
bool ArduCam_Test(void);
void ArduCam_FlushFifo(void);
void ArduCam_StartCapture(void);
void ArduCam_ClearFifoFlag(void);
uint32_t ArduCam_ReadFifoLength(void);
uint8_t ArduCam_ReadFifoSingle(void);
bool ArduCam_IsCaptureBit(void);
void ArduCam_ReadFifoBlock(uint8_t* pBuffer, uint32_t nLen);
uint8_t ArduCam_ReadSpiReg(uint8_t nAddr);
void ArduCam_WriteSpiReg(uint8_t nAddr, uint8_t nData);
void ArduCam_SetMode(uint8_t nMode);
void ArduCam_SetJpegSize(ac_jpeg_size_e eJpegSize);
void ArduCam_SetFormat(ac_format_e eFormat);
void ArduCam_SetLighMode(ac_light_mode_e eLighMode);
void ArduCam_SetBrightness(ac_brightness_e eBrightness);
void ArduCam_SetContrast(ac_contrast_e eContrast);
void ArduCam_SetColorSaturation(ac_saturation_e eSaturation);
uint8_t ArduCam_GetVersion(void);

#endif /* ARDUCAM_H_ */
