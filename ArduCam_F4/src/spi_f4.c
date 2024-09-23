/*
 * SPI1.c
 *
 *  Created on: 13. 3. 2019
 *      Author: priesolv
 *
 *      implementace pro STM32F407
 */

#include "spi_f4.h"
#include "stm32f4xx_ll_spi.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_rcc.h"

const spi_hw_t spi1_hw =
{
  .reg = SPI1,
  .nGpioAF = LL_GPIO_AF_5,
  .irq = SPI1_IRQn,
};

const spi_hw_t spi2_hw =
{
  .reg = SPI2,
  .nGpioAF = LL_GPIO_AF_5,
  .irq = SPI2_IRQn,
};

const spi_hw_t spi3_hw =
{
  .reg = SPI3,
  .nGpioAF = LL_GPIO_AF_6,
  .irq = SPI3_IRQn,
};

spi_drv_t _spi1_drv = { &spi1_hw, 0, spi_mode_0, spi_dir_mode_2Lines_FullDuplex, false, false, false };
spi_drv_t _spi2_drv = { &spi2_hw, 0, spi_mode_0, spi_dir_mode_2Lines_FullDuplex, false, false, false };
spi_drv_t _spi3_drv = { &spi3_hw, 0, spi_mode_0, spi_dir_mode_2Lines_FullDuplex, false, false, false };


void spi_Init(spi_drv_t* pDrv, gpio_pins_e eClkPin, gpio_pins_e eMosiPin, gpio_pins_e eMisoPin)
{
  GPIO_ConfigPin(eClkPin, mode_alternate, outtype_pushpull, pushpull_no, speed_high);
  GPIO_SetAFpin(eClkPin, pDrv->pHW->nGpioAF);

  GPIO_ConfigPin(eMosiPin, mode_alternate, outtype_pushpull, pushpull_no, speed_high);
  GPIO_SetAFpin(eMosiPin, pDrv->pHW->nGpioAF);

  if (eMisoPin != P_UNUSED)
  {
    GPIO_ConfigPin(eMisoPin, mode_alternate, outtype_pushpull, pushpull_no, speed_high);
    GPIO_SetAFpin(eMisoPin, pDrv->pHW->nGpioAF);
  }

  LL_RCC_ClocksTypeDef RCC_Clocks;
  LL_RCC_GetSystemClocksFreq(&RCC_Clocks); // Get system clocks

  if (pDrv->pHW->reg == SPI1)
  {
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI1);
    pDrv->nBusFrequencyHz = RCC_Clocks.PCLK2_Frequency;
  }

  if (pDrv->pHW->reg == SPI2)
  {
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_SPI2);
    pDrv->nBusFrequencyHz = RCC_Clocks.PCLK1_Frequency;
  }
  else if (pDrv->pHW->reg == SPI3)
  {
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_SPI3);
    pDrv->nBusFrequencyHz = RCC_Clocks.PCLK1_Frequency;
  }

  /* SPI configuration -------------------------------------------------------*/
  LL_SPI_DeInit(pDrv->pHW->reg);

  LL_SPI_InitTypeDef  SPI_InitStructure;
  SPI_InitStructure.TransferDirection = pDrv->eDirMode;
  SPI_InitStructure.Mode = LL_SPI_MODE_MASTER;
  SPI_InitStructure.DataWidth = LL_SPI_DATAWIDTH_8BIT;
  SPI_InitStructure.ClockPolarity = LL_SPI_POLARITY_LOW;
  SPI_InitStructure.ClockPhase = LL_SPI_PHASE_1EDGE;
  SPI_InitStructure.NSS = LL_SPI_NSS_SOFT;
  SPI_InitStructure.BaudRate = spi_br_256;  // SPI speed
  SPI_InitStructure.BitOrder = LL_SPI_MSB_FIRST;
  SPI_InitStructure.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
  LL_SPI_Init(pDrv->pHW->reg, &SPI_InitStructure);

  spi_SetMode(pDrv, pDrv->eMode);
  LL_SPI_Enable(pDrv->pHW->reg);
}

void spi_TransactionBegin(spi_drv_t* pDrv, gpio_pins_e eChipSelect, spi_br_e ePrescaler)
{
  spi_SetPrescaler(pDrv, ePrescaler);
  GPIO_RESETPIN(eChipSelect);
}

void spi_TransactionEnd(spi_drv_t* pDrv, gpio_pins_e eChipSelect)
{
  while (LL_SPI_IsActiveFlag_BSY(pDrv->pHW->reg));
  GPIO_SETPIN(eChipSelect);
}

void spi_SendData16(spi_drv_t* pDrv, uint16_t nValue)
{
  while (!(pDrv->pHW->reg->SR & LL_SPI_SR_TXE));
  pDrv->pHW->reg->DR = nValue >> 8;
  while (!(pDrv->pHW->reg->SR & LL_SPI_SR_TXE));
  pDrv->pHW->reg->DR = nValue & 0xFF;
}

uint8_t spi_SendData8(spi_drv_t* pDrv, uint8_t nValue)
{
  while (!LL_SPI_IsActiveFlag_TXE(pDrv->pHW->reg));
  LL_SPI_TransmitData8(pDrv->pHW->reg, nValue);

//  while (LL_SPI_IsActiveFlag_BSY(pDrv->pHW->reg));
//  return LL_SPI_ReceiveData8(pDrv->pHW->reg);

  while ((pDrv->pHW->reg->SR & LL_SPI_SR_BSY));
  return pDrv->pHW->reg->DR;
}

void spi_WriteBidirectionalByte(spi_drv_t* pDrv, uint16_t nValue)
{
  while (!(pDrv->pHW->reg->SR & LL_SPI_SR_TXE));
  pDrv->pHW->reg->DR = nValue;
  while (!(pDrv->pHW->reg->SR & LL_SPI_SR_TXE));
  while (pDrv->pHW->reg->SR & LL_SPI_SR_BSY);
}

uint8_t spi_ReadBidirectionalByte(spi_drv_t* pDrv)
{
  spi_SetDirection(pDrv, spi_dir_rx);
  while (!(pDrv->pHW->reg->SR & LL_SPI_SR_RXNE));
  uint8_t nValue = pDrv->pHW->reg->DR;
  spi_SetDirection(pDrv, spi_dir_tx);
  return nValue;
}

void spi_SetPrescaler(spi_drv_t* pDrv, spi_br_e ePrescaler)
{
  LL_SPI_Disable(pDrv->pHW->reg);

  pDrv->pHW->reg->CR1 = (pDrv->pHW->reg->CR1 & ~(SPI_CR1_BR)) | ePrescaler;

  LL_SPI_Enable(pDrv->pHW->reg);
}

spi_br_e spi_CalculatePrescaler(uint32_t nBusClock_Hz, uint32_t nMaxFreq_Hz)
{
  const spi_br_e arrPrescalers[] = { spi_br_2, spi_br_4, spi_br_8, spi_br_16, spi_br_32, spi_br_64, spi_br_128, spi_br_256 };
  spi_br_e ePrescaler = spi_br_256;
  uint16_t nSize = sizeof(arrPrescalers) / sizeof(spi_br_e);
  for (uint8_t i = 0; i < nSize; ++i)
  {
    uint32_t nDivider = 1 << (i + 1);
    if (nBusClock_Hz / nDivider <= nMaxFreq_Hz)
    {
      ePrescaler = arrPrescalers[i];
      break;
    }
  }

  return ePrescaler;
}

uint32_t spi_GetPrescalerDivider(spi_br_e ePrescaler)
{
  const uint8_t DividerTable[] = { 2, 4, 8, 16, 32, 64, 128 };

  return DividerTable[ePrescaler];
}

void spi_SetMode(spi_drv_t* pDrv, spi_mode_e eMode)
{
  LL_SPI_Disable(pDrv->pHW->reg);

  pDrv->pHW->reg->CR1 = (pDrv->pHW->reg->CR1 & ~(SPI_CR1_CPHA | SPI_CR1_CPOL)) | eMode;

  LL_SPI_Enable(pDrv->pHW->reg);
}

void spi_WaitForNoBusy(spi_drv_t* pDrv)
{
  while (pDrv->pHW->reg->SR & LL_SPI_SR_BSY);
}

void spi_SetDirection(spi_drv_t* pDrv, spi_direction_e bDirection)
{
  LL_SPI_SetTransferDirection(pDrv->pHW->reg, bDirection);
}
