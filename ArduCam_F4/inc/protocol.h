/*
 * protocol.h
 *
 *  Created on: 11. 12. 2020
 *  Author:     Priesol Vladimir
 */

#ifndef PROTOCOL_H_
#define PROTOCOL_H_


#include "stm32f4xx.h"
#include "stdbool.h"

#define PROT_DATA_PACKET_SIZE           50

typedef enum __attribute__((packed))
{
  check = 1,
  data,
  relation_data,
  wakeup,
  media_info,
  media_data,
  inst_reply = 0x80
} inst_e;

typedef enum
{
  type_temp = 0,
  type_batt,
} data_type_e;

bool Prot_IsChannelClear(void);
void Prot_SetReplyTimeout(uint32_t nTimeout_ms);
void Prot_SetMyAddr(uint8_t nAddr);
void Prot_SetServerAddr(uint8_t nAddr);
uint8_t Prot_GetServerAddr(void);

bool Prot_InstCheck();
bool Prot_InstData(data_type_e eType, int32_t nValue);
uint16_t Prot_InstRelationData();
void Prot_InstWakeUp();

bool Prot_InstMediaInfo(bool bStart, uint32_t nMediaLen);
void Prot_InstMediaData(uint8_t* pData, uint32_t nLen);


#endif /* PROTOCOL_H_ */
