/*
 * ips6404.c
 *
 *  Created on: 19. 02. 2020
 *      Author: PRIESOL VLADIMIR
 */


#include "ips6404.h"
#include "common_f4.h"

#include <string.h>


#define IPS6404_READ_DATA           0x03   // max 33MHz
//#define IPS6404_READ_DATA           0x0B     // max 84MHz

#define IPS6404_WRITE_DATA          0x02
#define IPS6404_RESET_ENABLE        0x66
#define IPS6404_RESET               0x99
#define IPS6404_READ_ID             0x9F

#define IPS6404_ID                  0x0D5D46

static spi_drv_t*          g_pSpi;
static spi_br_e            g_eSpiPrescaler; // prescaler for SPI interface (communication speed)
static gpio_pins_e         g_eCS;

void _Send24bit(uint32_t nValue);

bool IPS6404_Init(spi_drv_t* pSpi, gpio_pins_e eCS, spi_br_e ePrescaler)
{
  g_pSpi = pSpi;
  g_eCS = eCS;
  g_eSpiPrescaler = ePrescaler;

  // set CS for output
  GPIO_ClockEnable(eCS);
  GPIO_SETPIN(eCS);
  GPIO_ConfigPin(eCS, mode_output, outtype_pushpull, pushpull_no, speed_veryhigh);

  IPS6404_Reset();

  if (IPS6404_GetID() != IPS6404_ID)
  {
    return false;
  }

  return true;
}

void IPS6404_Reset(void)
{
  spi_TransactionBegin(g_pSpi, g_eCS, g_eSpiPrescaler);

  spi_SendData8(g_pSpi, IPS6404_RESET_ENABLE);
  spi_SendData8(g_pSpi, IPS6404_RESET);

  spi_TransactionEnd(g_pSpi, g_eCS);
}

void IPS6404_ReadData(uint32_t nAddr, uint8_t* pBuffer, uint32_t length)
{
  spi_TransactionBegin(g_pSpi, g_eCS, g_eSpiPrescaler);
  spi_SendData8(g_pSpi, IPS6404_READ_DATA);
  _Send24bit(nAddr);

#if IPS6404_READ_DATA == 0x0B
  spi_SendData8(g_pSpi, DUMMY_BYTE);
#endif

  while (length--)
  {
    *pBuffer++ = spi_SendData8(g_pSpi, SPI_DUMMY_BYTE);
  }

  spi_TransactionEnd(g_pSpi, g_eCS);

  spi_SetPrescaler(g_pSpi, spi_br_2);
}

void IPS6404_WriteBuffer(uint32_t nAddr, uint8_t* pBuffer, uint32_t length)
{
  spi_TransactionBegin(g_pSpi, g_eCS, g_eSpiPrescaler);
  spi_SendData8(g_pSpi, IPS6404_WRITE_DATA);
  _Send24bit(nAddr);
  while (length--)
  {
    spi_SendData8(g_pSpi, *pBuffer++);
  }

  spi_WaitForNoBusy(g_pSpi);
  spi_TransactionEnd(g_pSpi, g_eCS);
}

void IPS6404_Write_Open(uint32_t nAddr)
{
  spi_TransactionBegin(g_pSpi, g_eCS, g_eSpiPrescaler);
  spi_SendData8(g_pSpi, IPS6404_WRITE_DATA);
  _Send24bit(nAddr);
  spi_WaitForNoBusy(g_pSpi);
}

void IPS6404_Write_Write(uint8_t nData)
{
  spi_SendData8(g_pSpi, nData);
}

void IPS6404_Write_Close(void)
{
  spi_WaitForNoBusy(g_pSpi);
  spi_TransactionEnd(g_pSpi, g_eCS);
}

void IPS6404_Write_24bitValue(uint32_t nAddr, uint32_t nValue)
{
  IPS6404_WriteBuffer(nAddr, (uint8_t*)&nValue, 3);
}

uint32_t IPS6404_GetID()
{
  spi_TransactionBegin(g_pSpi, g_eCS, g_eSpiPrescaler);

  uint32_t nID = 0;
  spi_SendData8(g_pSpi, IPS6404_READ_ID);
  _Send24bit(0);
  nID = spi_SendData8(g_pSpi, SPI_DUMMY_BYTE) << 16;
  nID |= spi_SendData8(g_pSpi, SPI_DUMMY_BYTE) << 8;
  nID |= spi_SendData8(g_pSpi, SPI_DUMMY_BYTE);

  spi_TransactionEnd(g_pSpi, g_eCS);

  return nID;
}

spi_drv_t* IPS6404_GetDrv(void)
{
  return g_pSpi;
}

void _Send24bit(uint32_t nValue)
{
  spi_SendData8(g_pSpi, nValue >> 16);
  spi_SendData8(g_pSpi, nValue >> 8);
  spi_SendData8(g_pSpi, nValue);
}

bool IPS6404_Test(void)
{
  uint8_t arrDataW[] = { 10, 11, 12, 13, 14, 15 };
  uint8_t arrDataR[sizeof (arrDataW)];

  IPS6404_WriteBuffer(0, arrDataW, sizeof (arrDataW));
  IPS6404_ReadData(0, arrDataR, sizeof (arrDataR));

  if (memcmp(arrDataW, arrDataR, sizeof (arrDataW)) != 0)
  {
    return false;
  }

  return true;
}
