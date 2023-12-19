/*
 * ips6404.h
 *
 *  Created on: 19. 02. 2020
 *      Author: PRIESOL VLADIMIR
 */

#ifndef IPS6404_H_
#define IPS6404_H_

#include "stm32f4xx.h"
#include <stdbool.h>

#include "spi_f4.h"

bool IPS6404_Init(spi_drv_t* pSpi, gpio_pins_e eCS, uint32_t nMaxFreqMhz);
void IPS6404_Reset(void);
uint32_t IPS6404_GetID();

void IPS6404_ReadBuffer(uint32_t nAddr, uint8_t* pBuffer, uint32_t length);
void IPS6404_WriteBuffer(uint32_t nAddr, uint8_t* pBuffer, uint32_t length);

void IPS6404_Write_24bitValue(uint32_t nAddr, uint32_t nValue);

spi_drv_t* IPS6404_GetDrv(void);

bool IPS6404_Test(void);

#endif /* IPS6404_H_ */
