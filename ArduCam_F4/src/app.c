/*
 * app.c
 *
 *  Created on: 18. 3. 2022
 *  Author:     Priesol Vladimir
 */

#include "app.h"
#include "hw.h"
#include "usart.h"
#include "si4463.h"
#include "ips6404.h"
#include "i2c_int.h"
#include "ArduCam.h"
#include "protocol.h"

#include "clock_f4.h"

#include "stm32f4xx_ll_rcc.h"

#define APP_ISP6404_CS                PA4
#define APP_ARDUCAM_CS                PA9

#define APP_BURST_SIZE                1000

typedef enum
{
  app_cmd_size_160x120     = 0,
  app_cmd_size_176x144,
  app_cmd_size_320x240,
  app_cmd_size_352x288,
  app_cmd_size_640x480,
  app_cmd_size_800x600,
  app_cmd_size_1024x768,
  app_cmd_size_1280x1024,
  app_cmd_size_1600x1200,
  app_cmd_cap_JPEG         = 0x10,
  app_cmd_cap_BMP          = 0x30,
  app_cmd_set_JPEG         = 0x11,
  app_cmd_set_BMP          = 0x31,
  app_cmd_none             = 0xFF
} app_serial_cmd_e;

static const uint32_t JPEG_SOI_MARKER = 0xFFD8FF;  // written in little-endian for esp32
static const uint16_t JPEG_EOI_MARKER = 0xD9FF;    // written in little-endian for esp32

static spi_drv_t*                g_pSpi1 = spi1;
static spi_drv_t*                g_pSpi2 = spi2;
static i2cdrv_t*                 g_pI2c1 = i2c1;
static usart_drv_t*              g_pUsart2 = usart2;

static uint32_t                  g_nImageSize = 0;    // capture image size

static app_serial_cmd_e          g_eCmd;

int32_t _FindJpegStart(uint8_t* pBuffer, uint32_t nLen);
int32_t _FindJpegEnd(uint8_t* pBuffer, uint32_t nLen);
bool _ProcessCmd(void);
bool _CaptureImage(void);
void _SendImageSerial(void);
bool _SendImageRadio(void);
void _UsartRcvCbk(uint8_t nValue);

void App_Init(void)
{
  bool bResult = HW_Init();

  LL_RCC_ClocksTypeDef RCC_Clocks;
  LL_RCC_GetSystemClocksFreq(&RCC_Clocks); // Get system clocks

  HW_SetCamSupply(true);

  spi_Init(g_pSpi1, PA5, PA7, PA6);
  i2c_init(g_pI2c1, PB9, PB8);

  if (!IPS6404_Init(g_pSpi1, APP_ISP6404_CS, 84000000))
  {
    bResult = false;
  }


//  Clock_SetHSI();
//  Clock_SetPLL(8, 100, 2, CLOCK_SOURCE_HSI);
//  bool bRes = IPS6404_Test();

  //  ePrescaler = spi_CalculatePrescaler(RCC_Clocks.PCLK1_Frequency, 10000000);
  //  if (!SI4463_Init(g_pSpi2))
  //  {
  //    bResult = false;
  //  }

  usart_Init(g_pUsart2, _UsartRcvCbk, PA2, PA3, 115200);

  spi_br_e ePrescaler = spi_CalculatePrescaler(RCC_Clocks.PCLK2_Frequency, 2000000);
  ArduCam_Init(g_pSpi1, APP_ARDUCAM_CS, ePrescaler, g_pI2c1);

  usart_PrintLn(g_pUsart2, "ACK CMD OV2640 detected. END");
}

void App_Exec(void)
{
  if (g_eCmd != app_cmd_none)
  {
    _ProcessCmd();
  }

  if (g_eCmd == app_cmd_cap_JPEG)
  {
    _CaptureImage();
    _SendImageSerial();
    g_eCmd = app_cmd_none;
  }

}

bool _CaptureImage(void)
{
  ArduCam_FlushFifo();
  ArduCam_ClearFifoFlag();
  ArduCam_StartCapture();
  while (!ArduCam_IsCaptureBit());
  uint32_t nImageSize = ArduCam_ReadFifoLength();
  bool bHeader = false;
  bool bImageEnd = false;

  g_nImageSize = 0;
  while (nImageSize > 0)
  {
    uint8_t arrImage[APP_BURST_SIZE];
    uint32_t nLen = APP_BURST_SIZE;
    if (nImageSize < APP_BURST_SIZE)
    {
      nLen = nImageSize;
    }

    ArduCam_ReadFifoBlock(arrImage, nLen);

    uint8_t* pBuffer = NULL;
    uint32_t nBufferLen;
    if (!bHeader)
    {
      int32_t nOffset = _FindJpegStart(arrImage, nLen);
      if (nOffset != -1)
      {
        bHeader = true;
        pBuffer = arrImage + nOffset;
        nBufferLen = nLen - nOffset;
      }
    }
    else
    {
      int32_t nOffset = _FindJpegEnd(arrImage, nLen);
      pBuffer = arrImage;
      nBufferLen = nLen;
      if (nOffset != -1)
      {
        nBufferLen = nOffset;
        bImageEnd = true;
      }
    }

    if (pBuffer != NULL)
    {
      IPS6404_WriteBuffer(g_nImageSize, pBuffer, nBufferLen);
      g_nImageSize += nBufferLen;
    }

    if (bImageEnd)
    {
      usart_PrintLn(g_pUsart2, "ACK CMD Image captured. END");
      break;
    }

    nImageSize -= nLen;
  }

  return bImageEnd;
}

void _SendImageSerial(void)
{
  const uint16_t nBufferSize = 1000;
  uint8_t arrData[nBufferSize];
  uint32_t nImagePos = 0;

  while (nImagePos < g_nImageSize)
  {
   uint32_t nLen = g_nImageSize - nImagePos;
   if (nLen > nBufferSize)
   {
     nLen = nBufferSize;
   }

    IPS6404_ReadBuffer(nImagePos, arrData, nLen);
    usart_Send(g_pUsart2, arrData, nLen);
    usart_WaitForTransmitComplete(g_pUsart2);

    nImagePos += nLen;
  }
}

bool _SendImageRadio(void)
{
  bool bResult = false;
  if (Prot_InstMediaInfo(true, g_nImageSize))
  {
    uint8_t arrData[PROT_DATA_PACKET_SIZE];
    uint32_t nImagePos = 0;

    while (nImagePos < g_nImageSize)
    {
      uint32_t nLen = g_nImageSize - nImagePos;
      if (nLen > PROT_DATA_PACKET_SIZE)
      {
        nLen = PROT_DATA_PACKET_SIZE;
      }

      IPS6404_ReadBuffer(nImagePos, arrData, sizeof (arrData));
      Prot_InstMediaData(arrData, nLen);
      nImagePos += nLen;
    }

    if (Prot_InstMediaInfo(false, 0))
    {
      bResult = true;
    }
  }

  return bResult;
}

int32_t _FindJpegStart(uint8_t* pBuffer, uint32_t nLen)
{
  uint32_t sig = *((uint32_t *)pBuffer) & 0xFFFFFF;
  if (sig != JPEG_SOI_MARKER)
  {
    for (uint32_t i = 0; i < nLen; i++)
    {
      sig = *((uint32_t *)(&pBuffer[i])) & 0xFFFFFF;
      if (sig == JPEG_SOI_MARKER)
      {
        return i;
      }
    }

    return -1;
  }

  return 0;
}

int32_t _FindJpegEnd(uint8_t* pBuffer, uint32_t nLen)
{
  int nOffset = -1;
  uint8_t *dptr = (uint8_t *)pBuffer + nLen - 2;
  while (dptr > pBuffer)
  {
    uint16_t sig = *((uint16_t *)dptr);
    if (sig == JPEG_EOI_MARKER)
    {
      nOffset = dptr - pBuffer;
      return nOffset + 2;
    }

    dptr--;
  }

  return -1;
}

bool _ProcessCmd(void)
{
  bool bResult = true;

  switch (g_eCmd)
  {
  case app_cmd_size_160x120:
    ArduCam_SetJpegSize(size_160x120);
    break;
  case app_cmd_size_176x144:
    ArduCam_SetJpegSize(size_176x144);
    break;
  case app_cmd_size_320x240:
    ArduCam_SetJpegSize(size_320x240);
    break;
  case app_cmd_size_352x288:
    ArduCam_SetJpegSize(size_352x288);
    break;
  case app_cmd_size_640x480:
    ArduCam_SetJpegSize(size_640x480);
    break;
  case app_cmd_size_800x600:
    ArduCam_SetJpegSize(size_800x600);
    break;
  case app_cmd_size_1024x768:
    ArduCam_SetJpegSize(size_1024x768);
    break;
  case app_cmd_size_1280x1024:
    ArduCam_SetJpegSize(size_1280x1024);
    break;
  case app_cmd_size_1600x1200:
    ArduCam_SetJpegSize(size_1600x1200);
    break;
  default:
    bResult = false;
    break;
  }

  if (bResult)
  {
    usart_PrintLn(g_pUsart2, "ACK CMD Size changed. END");
    g_eCmd = app_cmd_none;
  }

  return bResult;
}

void _UsartRcvCbk(uint8_t nValue)
{
  g_eCmd = nValue;
}
