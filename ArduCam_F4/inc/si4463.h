/*
 * si4463.h
 *
 *  Created on: 29 oct 2017
 *  Author: Rekawekp
 */


#ifndef SI4463_H_
#define SI4463_H_

#include "stm32f4xx.h"
#include <stdbool.h>
#include <stddef.h>

typedef enum
{
  noState,
  sleepState,
  spiActiveState,
  readyState,
  ready2State,
  txTuneState,
  rxTuneState,
  txState,
  rxState
} si4463_state_e;

typedef enum
{
  int_RX_ready = 0x0210,
  int_TX_ready = 0x0220,
  int_CRC_error = 0x0208,
  int_rssi = 0x0408,
  int_preamble_detect = 0x0402,
  int_invalid_preamble = 0x0404,
} si4463_int_e;

typedef enum
{
  pwr_min = 0,
  pwr_mid = 0x40,
  pwr_max = 0x7F,
} si4463_power_lvl_t;

void SI4463_TX_FIFO (uint8_t *buff_tx, uint8_t len);
void SI4463_RX_FIFO (uint8_t *buff_rx, uint8_t len);

void SI4463_RX_Start(uint8_t size);
void SI4463_TX_Start(uint8_t size);

void SI4463_ReadBuffer(uint8_t *rd_buff, uint8_t len);
bool SI4463_Init(void);
si4463_state_e SI4463_GetCurrentState(void);

void SI4463_SetProperty( uint8_t *set_buff, uint8_t len);
void SI4463_GetProperty( uint8_t *get_buff, uint8_t len);

void SI4463_Wait_CTS(void);
bool SI4463_IsInterrupt(si4463_int_e eInt);
void SI4463_ClearInterrupts(void);

void SI4463_Clear_TX_FIFO(void);
void SI4463_Clear_RX_FIFO(void);
uint8_t SI4463_GetRxFifoInfo(void);

void SI4463_SendData(uint8_t* pData, uint8_t size);
uint8_t SI4463_ReadData(uint8_t* pData, uint8_t size, uint32_t nTimeout_ms);

void SI4463_Patch(void);

uint16_t SI4463_GetId(void);

void SI4463_SetPowerLevel(si4463_power_lvl_t eLevel);
void SI4463_ChangeState(si4463_state_e eState);
void SI4463_SetChannel(uint8_t nChannel);

#endif /* SI4463_H_ */
