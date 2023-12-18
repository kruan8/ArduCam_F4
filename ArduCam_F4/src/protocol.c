/*
 * protocol.c
 *
 *  Created on: 11. 12. 2020
 *  Author:     Priesol Vladimir
 */

#include "protocol.h"
#include "si4463.h"
#include "timer.h"

#include <string.h>

#define  PROT_RECV_ATTEMPTS             3
#define  PROT_DEFAULT_TIMEOUT_MS        800
#define  PROT_WAKEUP_DURATION_MS        1000

#define PROT_PACKET_SIZE           		  64


typedef struct __attribute__((packed))
{
  uint8_t  nAddrDest;
  uint8_t  nAddrSource;
  inst_e   eInst;
  uint8_t  nLength;
} inst_header_t;


static uint8_t       g_arrBuffer[64];    // RX buffer
static uint8_t*      g_pData = g_arrBuffer + sizeof(inst_header_t);  // pointer to reply instruction payload
static uint8_t       g_nPayloadSize;            // RX size

static inst_e        g_eInst;            // transmited instruction

static uint8_t       g_nMyAddr = 0;
static uint8_t       g_nServerAddr = 0;
static uint16_t      g_nBlockCounter;            // block counter for media transfer

static uint32_t      g_nTimeout_ms = PROT_DEFAULT_TIMEOUT_MS;

bool _SendPacket(inst_e eInst, uint8_t* pBuffer, uint8_t nLen);
bool _WaitForReply(void);

bool _SendPacket(inst_e eInst, uint8_t* pBuffer, uint8_t nLen)
{
  inst_header_t* pHeader = (inst_header_t*)g_arrBuffer;

  g_eInst = eInst;

  pHeader->eInst = eInst;
  pHeader->nAddrDest = g_nServerAddr;
  pHeader->nAddrSource = g_nMyAddr;
  pHeader->nLength = nLen;

  if (pBuffer != NULL && nLen > 0)
	{
	  memcpy(g_arrBuffer + sizeof(inst_header_t), pBuffer, nLen);
	}

  uint8_t nLoop = PROT_RECV_ATTEMPTS;
  bool bResult;
  while (nLoop--)
  {
	//Serial.println(nLen);
	SI4463_SendData(g_arrBuffer, PROT_PACKET_SIZE);
	bResult = _WaitForReply();
	if (bResult)
	{
		break;
	}
  }

  return bResult;
}

bool _WaitForReply(void)
{
  if (g_nTimeout_ms == 0)   // wakeup instruction has not reply
  {
    return true;
  }

  if ((g_nPayloadSize = SI4463_ReadData(g_arrBuffer, PROT_PACKET_SIZE, g_nTimeout_ms)) == 0)
  {
    return false;
  }

  //Serial.println(g_nPayloadSize);

  inst_header_t* pHeader = (inst_header_t*)g_arrBuffer;
  // check instruction code
  if (pHeader->eInst != (g_eInst | inst_reply))
  {
    return false;
  }

  // check my address
  if (pHeader->nAddrDest != g_nMyAddr)
  {
    return false;
  }

  g_nPayloadSize = pHeader->nLength;

  return true;
}

bool Prot_IsChannelClear(void)
{
  if ((SI4463_ReadData(g_arrBuffer, sizeof(g_arrBuffer), 500)) == 0)
  {
    if (!SI4463_IsInterrupt(int_preamble_detect))
    {
      return true;
    }
  }

  return false;
}

void Prot_SetReplyTimeout(uint32_t nTimeout_ms)
{
  g_nTimeout_ms = nTimeout_ms;
}

void Prot_SetMyAddr(uint8_t nAddr)
{
  g_nMyAddr = nAddr;
}

void Prot_SetServerAddr(uint8_t nAddr)
{
  g_nServerAddr = nAddr;
}

uint8_t Prot_GetServerAddr(void)
{
  return g_nServerAddr;
}

// --- Instruction implementation ----
bool Prot_InstCheck()
{
	g_nTimeout_ms = PROT_DEFAULT_TIMEOUT_MS;
  return _SendPacket(check, NULL, 0);
}

bool Prot_InstData(data_type_e eType, int32_t nValue)
{
  uint8_t nBufferSize = 1 + sizeof(uint32_t);
  uint8_t arrBuffer[nBufferSize];

  arrBuffer[0] = eType;
  uint32_t nValueU = 32768 + nValue;
  memcpy(&arrBuffer[1], &nValueU, sizeof(uint32_t));

  if (!_SendPacket(data, arrBuffer, nBufferSize))
  {
    return false;
  }

  return true;
}

// return interval, 0=no respond
uint16_t Prot_InstRelationData()
{
  if (!_SendPacket(relation_data, NULL, 0))
  {
    return false;
  }

  if (g_nPayloadSize != 4)
  {
    return 0;
  }

  uint16_t nInterval_s = *((uint16_t*)&g_pData[0]);
  uint16_t nRssi = *((uint16_t*)&g_pData[2]);
  if (nInterval_s > 0)
  {
    //AppData_SetInterval(nInterval_s);
  }

  return nRssi;
}

bool Prot_InstMediaInfo(bool bStart, uint32_t nMediaLen)
{
	uint8_t buffer[10];
	buffer[0] = (uint8_t)bStart;
	g_nBlockCounter = 0;

	if (bStart)
	{
		uint16_t nBlocks = nMediaLen / PROT_DATA_PACKET_SIZE;
		if (nMediaLen % PROT_DATA_PACKET_SIZE > 0)
		{
			nBlocks++;
		}

		buffer[1] =  nBlocks & 0xFF;
		buffer[2] =  nBlocks >> 8;
		uint32_t* pnLength = (uint32_t*)&(buffer[3]);
		*pnLength = nMediaLen;
	}
	else if (!bStart)
	{

	}

	bool bRes = _SendPacket(media_info, buffer, sizeof(buffer));
	if (bRes)
	{
		if (!bStart && g_pData[0] != 1)
		{
			return false;
		}
		else
		{

		}
	}

	return true;
}

void Prot_InstMediaData(uint8_t* pData, uint32_t nLen)
{
	uint8_t buffer[PROT_DATA_PACKET_SIZE + 2];

	while (nLen)
	{
		buffer[0] = g_nBlockCounter & 0xFF;
		buffer[1] = g_nBlockCounter >> 8;

		uint8_t nDataLen = PROT_DATA_PACKET_SIZE;
		if (nLen < PROT_DATA_PACKET_SIZE)
		{
			nDataLen = nLen;
		}

		memcpy(&buffer[2], pData, nDataLen);

		g_nTimeout_ms = 0;
		_SendPacket(media_data, buffer, nDataLen + 2);

		nLen -= nDataLen;
		pData += nDataLen;
		g_nBlockCounter++;

		Timer_Delay_ms(5);
	}

	g_nTimeout_ms = PROT_DEFAULT_TIMEOUT_MS;

}

void Prot_InstWakeUp()
{
  uint32_t nEndTime = Timer_GetTicks_ms() + PROT_WAKEUP_DURATION_MS;

  while (Timer_GetTicks_ms() < nEndTime)
  {
	  g_nTimeout_ms = 0;
    _SendPacket(wakeup, NULL, 0);
  }

  g_nTimeout_ms = PROT_DEFAULT_TIMEOUT_MS;
}
