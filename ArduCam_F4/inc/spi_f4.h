/*
 * spi.h
 *
 *  Created on: 13. 3. 2019
 *  Author:     Priesol Vladimir
 */

#ifndef SPI_H_
#define SPI_H_

#include "stm32f4xx.h"
#include "stdbool.h"
#include "stddef.h"
#include "common_f4.h"

#define SPI_DUMMY_BYTE    0xFF

typedef struct
{
  SPI_TypeDef* reg;
  uint8_t  nGpioAF;
  IRQn_Type irq;
} spi_hw_t;

typedef enum
{
  spi_br_2 =   (0b000 << 3),   // fPCLK/2
  spi_br_4 =   (0b001 << 3),   // fPCLK/4
  spi_br_8 =   (0b010 << 3),   // fPCLK/8
  spi_br_16 =  (0b011 << 3),   // fPCLK/16
  spi_br_32 =  (0b100 << 3),   // fPCLK/32
  spi_br_64 =  (0b101 << 3),   // fPCLK/64
  spi_br_128 = (0b110 << 3),   // fPCLK/128
  spi_br_256 = (0b111 << 3),   // fPCLK/256
} spi_br_e;

typedef enum
{
  spi_mode_0 = 0b00,  // CPOL=0; CPHA=0 (clk idle L, pha 1. edge)
  spi_mode_1 = 0b01,  // CPOL=0; CPHA=1 (clk idle L, pha 2. edge)
  spi_mode_2 = 0b10,  // CPOL=1; CPHA=0 (clk idle H, pha 1. edge)
  spi_mode_3 = 0b11,  // CPOL=1; CPHA=1 (clk idle H, pha 2. edge)
} spi_mode_e;

typedef enum
{
  spi_dir_rx = 0xBFFF,
  spi_dir_tx = 0x4000,
} spi_direction_e;

typedef enum
{
  spi_dir_mode_2Lines_FullDuplex = 0x0000,
  spi_dir_mode_2Lines_RxOnly     = 0x0400,
  spi_dir_mode_1Line_Rx          = 0x8000,
  spi_dir_mode_1Line_Tx          = 0xC000,
} spi_dir_mode_e;

typedef struct
{
  spi_hw_t const *   pHW;
  uint32_t           nBusFrequencyHz;
  spi_mode_e         eMode;
  spi_dir_mode_e     eDirMode;
//  volatile uint16_t  nWriteLen;
//  volatile uint16_t  nReadLen;
//  volatile uint8_t*  pWrite;
//  volatile uint8_t*  pRead;

  volatile bool bError;

  bool bLock;      /* bus lock */
  bool bComplete;  /* completion from ISR */

} spi_drv_t;


void spi_Init(spi_drv_t* pDrv, gpio_pins_e eClkPin, gpio_pins_e eMosiPin, gpio_pins_e eMisoPin);
void spi_TransactionBegin(spi_drv_t* pDrv, gpio_pins_e eChipSelect, spi_br_e ePrescaler);
void spi_TransactionEnd(spi_drv_t* pDrv, gpio_pins_e eChipSelect);
void spi_SendData16(spi_drv_t* pDrv, uint16_t nValue);
uint8_t spi_SendData8(spi_drv_t* pDrv, uint8_t nValue);

void spi_WriteBidirectionalByte(spi_drv_t* pDrv, uint16_t nValue);
uint8_t spi_ReadBidirectionalByte(spi_drv_t* pDrv);

void spi_SetPrescaler(spi_drv_t* pDrv, spi_br_e ePrescaler);
spi_br_e spi_CalculatePrescaler(uint32_t nBusClock_Hz, uint32_t nMaxFreq_Hz);
uint32_t spi_GetPrescalerDivider(spi_br_e ePrescaler);
void spi_SetMode(spi_drv_t* pDrv, spi_mode_e eMode);
void spi_WaitForNoBusy(spi_drv_t* pDrv);
void spi_SetDirection(spi_drv_t* pDrv, spi_direction_e bDirection);

extern spi_drv_t _spi1_drv;
#define spi1 (&_spi1_drv)

extern spi_drv_t _spi2_drv;
#define spi2 (&_spi2_drv)

extern spi_drv_t _spi3_drv;
#define spi3 (&_spi3_drv)

#endif /* SPI_H_ */

