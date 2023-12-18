/*
 * usart.h
 *
 *  Created on: 25. 3. 2019
 *  Author:     Priesol Vladimir
 */

#ifndef USART_H_
#define USART_H_

#include "stm32f4xx.h"
#include <stdbool.h>
#include <stddef.h>
#include "common_f4.h"

typedef struct
{
  USART_TypeDef* reg;
  uint32_t nAPBclock;
  uint8_t  nGpioAF;
  IRQn_Type irq;
} usart_hw_t;

typedef struct
{
  usart_hw_t const * pHW;
  volatile uint16_t  nWriteLen;
  volatile uint16_t  nReadLen;
  volatile uint8_t*  pWrite;
  volatile uint8_t*  pRead;

  bool bLock;      /* bus lock */
  bool bComplete;  /* completion from ISR */
} usart_drv_t;


void usart_Init(usart_drv_t* pDrv, void(*rxCb)(uint8_t), gpio_pins_e eTxPin, gpio_pins_e eRxPin, uint32_t nBaudrate);
void usart_Send(usart_drv_t* pDrv, uint8_t *buf, uint32_t len);
void usart_WaitForTransmitComplete(usart_drv_t* pDrv);
void usart_PrintLn(usart_drv_t* pDrv, char* pText);

extern usart_drv_t _usart1_drv;
#define usart1 (&_usart1_drv)

extern usart_drv_t _usart2_drv;
#define usart2 (&_usart2_drv)

extern usart_drv_t _usart3_drv;
#define usart3 (&_usart3_drv)

#endif /* USART_H_ */
