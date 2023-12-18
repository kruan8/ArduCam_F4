/*
 * si4463.c
 *
 *  Created on: 28 oct 2017
 *  Author: Rekawekp
 */

#include "common_f4.h"
#include "si4463.h"
#include "radio_config_Si4463.h"
#include "spi_f4.h"
#include "timer.h"

#include <string.h>

// Chip select pin
#define SI4463_CS             PA4
#define SI4463_CS_ENABLE      GPIO_RESETPIN(SI4463_CS)
#define SI4463_CS_DISABLE     GPIO_SETPIN(SI4463_CS)

// Shoutdown pin
#define SI4463_SDN            PA3
#define SI4463_SDN_ENABLE     GPIO_SETPIN(SI4463_SDN)
#define SI4463_SDN_DISABLE    GPIO_RESETPIN(SI4463_SDN)

static spi_drv_t *g_pDrv;   // SPI driver
static uint8_t g_nChannel;  // Rx/Tx channel

static const uint8_t Radio_Conf_Array[] = RADIO_CONFIGURATION_DATA_ARRAY;

//------------------------------- SI4463_Init ------------------------------
bool SI4463_Init(void)
{
  // config SDN
  GPIO_ConfigPin(SI4463_SDN, mode_output, outtype_pushpull, pushpull_no, speed_low);
  SI4463_SDN_ENABLE;

  // config CS
  GPIO_ConfigPin(SI4463_CS, mode_output, outtype_pushpull, pushpull_no, speed_high);
  SI4463_CS_DISABLE;

  // initialize SPI
  g_pDrv = spi1;
  spi_Init(g_pDrv, PA5, PA7, PA6);
  spi_SetPrescaler(g_pDrv, spi_br_2);

  // exit from shoutdown
  SI4463_SDN_DISABLE;

  // Wait for max. delay of PowerOnReset
  Timer_Delay_ms(10);

  uint16_t nId = SI4463_GetId();
  if (nId != 0x4463)
  {
    return false;
  }

  uint16_t place = 0;
  while(*(Radio_Conf_Array + place) != 0x00)
  {
    SI4463_SetProperty((uint8_t*)(Radio_Conf_Array + place + 1), *(Radio_Conf_Array + place));

    place += *(Radio_Conf_Array + place) + 1;
  }

  g_nChannel = 0;
  return true;
}

si4463_state_e SI4463_GetCurrentState(void)
{
  uint8_t comm_buff[] = { 0x33 };
  uint8_t answer[2];

  SI4463_GetProperty(comm_buff, 1);
  SI4463_ReadBuffer(answer, sizeof(answer));

  return (si4463_state_e)answer[0];
}

//------------------------------- SI4463_Read_Buffer ------------------------------
void SI4463_ReadBuffer(uint8_t *rd_buff, uint8_t len)
{
	uint8_t cts_val = 0;

	while( cts_val != 0xFF )
	{
	  SI4463_CS_ENABLE;

	  spi_SendData8(g_pDrv, 0x44);
		cts_val = spi_SendData8(g_pDrv, SPI_DUMMY_BYTE);
		if( cts_val != 0xFF )
		{
		  SI4463_CS_DISABLE;
		}
	}

	for (uint8_t i = 0 ; i < len ; i++)
	{
		*( rd_buff + i ) = spi_SendData8(g_pDrv, SPI_DUMMY_BYTE);
	}

	SI4463_CS_DISABLE;
}

//------------------------------- SI4463_Wait_CTS ------------------------------
void SI4463_Wait_CTS(void)
{
	uint8_t cts_val = 0;

	while( cts_val != 0xFF )
	{
	  SI4463_CS_ENABLE;

	  spi_SendData8(g_pDrv, 0x44);
		cts_val = spi_SendData8(g_pDrv, SPI_DUMMY_BYTE);

		SI4463_CS_DISABLE;
	}
}

//------------------------------- SI4463_Get_Property ------------------------------
void SI4463_GetProperty(uint8_t *get_buff, uint8_t len)
{
  SI4463_CS_ENABLE;

	for (uint8_t i = 0 ; i < len ; i++)
	{
	  spi_SendData8(g_pDrv, *(get_buff + i));
	}

	SI4463_CS_DISABLE;

//	SI4463_Wait_CTS();
}

//------------------------------- SI4463_Set_Property ------------------------------
void SI4463_SetProperty(uint8_t *set_buff, uint8_t len)
{
  SI4463_CS_ENABLE;

	for (uint8_t i = 0 ; i < len ; i++)
	{
	  spi_SendData8(g_pDrv, *(set_buff + i));
	}

	SI4463_CS_DISABLE;

	SI4463_Wait_CTS();
}

//------------------------------- SI4463_TX_Start ------------------------------
void SI4463_TX_Start(uint8_t size)
{
	uint8_t command[] = { 0x31, g_nChannel, readyState, 0x00, size };
	SI4463_SetProperty(command, 5);
}

//------------------------------- SI4463_RX_Start ------------------------------
void SI4463_RX_Start(uint8_t size)
{
	uint8_t command[] = { 0x32, g_nChannel , 0x00, 0x00, size, 0x00, readyState, readyState }; // readyState
	SI4463_SetProperty(command, 8);
}

//------------------------------- SI4463_TX_FIFO ------------------------------
void SI4463_TX_FIFO(uint8_t *pBuffTx, uint8_t nLen)
{
	uint8_t buff[nLen + 1];

	buff[0] = 0x66;
	memcpy(buff + 1, pBuffTx, nLen);

	SI4463_CS_ENABLE;

	for (uint8_t i = 0; i < sizeof(buff); i++)
	{
	  spi_SendData8(g_pDrv, buff[i]);
	}

	SI4463_CS_DISABLE;
}

//------------------------------- SI4463_RX_FIFO ------------------------------
void SI4463_RX_FIFO(uint8_t *buff_rx, uint8_t len)
{
  SI4463_CS_ENABLE;

  spi_SendData8(g_pDrv, 0x77);

	for (uint8_t i = 0 ; i < len ; i++)
	{
		*(buff_rx+i) = spi_SendData8(g_pDrv, SPI_DUMMY_BYTE);
	}

	SI4463_CS_DISABLE;
}

//------------------------------- SI4463_Clear_TX_FIFO ------------------------------
void SI4463_Clear_TX_FIFO(void)
{
	uint8_t command[] = { 0x15, 0x01 };
	SI4463_SetProperty(command, 2);
}

//------------------------------- SI4463_Clear_RX_FIFO ------------------------------
void SI4463_Clear_RX_FIFO(void)
{
	uint8_t buff_2[] = { 0x15, 0x02 };
	SI4463_SetProperty(buff_2, 2);
}

uint8_t SI4463_GetRxFifoInfo(void)
{
  uint8_t readBuff[2];
  uint8_t command[] = { 0x15, 0x00 };

  SI4463_GetProperty(command, sizeof(command));
  SI4463_ReadBuffer(readBuff, sizeof(readBuff));

  return readBuff[0];
}

//------------------------------- SI4463_GetId ------------------------------
uint16_t SI4463_GetId(void)
{
	uint8_t readBuff[8];
	uint8_t command = 0x01;
	SI4463_GetProperty(&command, sizeof(command));
	SI4463_ReadBuffer(readBuff, 8);

	return (readBuff[1] << 8) + readBuff[2];
	// Si4463-B1B part(4463) rom_id(3) revision(B1B) top_marking(44631B Bxxxxx)
}

//------------------------------- SI4463_Send_Data ------------------------------
void SI4463_SendData(uint8_t* pData, uint8_t size)
{
	SI4463_TX_FIFO (pData, size);
	SI4463_ClearInterrupts();

  SI4463_TX_Start(size);
  while (!SI4463_IsInterrupt(int_TX_ready))
  {

  }
}

//------------------------------- SI4463_Read_Data ------------------------------
uint8_t SI4463_ReadData(uint8_t* pData, uint8_t size, uint32_t nTimeout_ms)
{
  SI4463_ChangeState(readyState);
  SI4463_Clear_RX_FIFO();
  SI4463_ClearInterrupts();
  SI4463_RX_Start(size);

  uint32_t nEndTime = Timer_GetTicks_ms() + nTimeout_ms;
  while (!SI4463_IsInterrupt(int_RX_ready))
  {
    if (Timer_GetTicks_ms() > nEndTime)
    {
      SI4463_ChangeState(readyState);
      return 0;
    }
  }

  uint8_t nSize = SI4463_GetRxFifoInfo();
  SI4463_RX_FIFO(pData, nSize);
  return nSize;
}

bool SI4463_IsInterrupt(si4463_int_e eInt)
{
  uint8_t readBuff[8];
  uint8_t command[] = { 0x20, 0xFF, 0xFF, 0xFF };

  SI4463_GetProperty(command, sizeof(command));
  SI4463_ReadBuffer(readBuff, sizeof(readBuff));

  uint16_t nMask = (uint16_t)eInt;
  uint8_t nPos = (nMask >> 8) & 0xFF;
  bool bRes = readBuff[nPos] & (nMask & 0XFF);
  return bRes;
}

void SI4463_ClearInterrupts(void)
{
  uint8_t read_buff[8];
  uint8_t command[] = { 0x20, 0x00, 0x00, 0x00 };
  SI4463_GetProperty(command, sizeof(command));
  SI4463_ReadBuffer(read_buff, sizeof(read_buff));
}

void SI4463_SetPowerLevel(si4463_power_lvl_t eLevel)
{
  uint8_t command[] = { 0x11, 0x22, 0x01, 0x01, (uint8_t)eLevel};
  SI4463_SetProperty(command, sizeof(command));
}

void SI4463_ChangeState(si4463_state_e eState)
{
  uint8_t command[] = { 0x34, eState };
  SI4463_SetProperty(command, sizeof(command));
}

void SI4463_SetChannel(uint8_t nChannel)
{
  g_nChannel = nChannel;
}

//------------------------------- SI4463_Patch ------------------------------
//void SI4463_Patch( void )
//{
//	uint8_t Radio_Patch_Array[][8] = { SI446X_PATCH_CMDS };
//	uint8_t buff_patch[8];
//	uint16_t size = sizeof(Radio_Patch_Array) / 8 * sizeof(uint8_t);
//
//	for (uint16_t row = 0 ; row < size; row++)
//	{
//		for( uint8_t col = 0 ; col < 8 ; col++)
//		{
//			buff_patch[col] = Radio_Patch_Array[row][col];
//		}
//
//		SI4463_Set_Property( buff_patch, 8 );
//	}
//}

