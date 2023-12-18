/*
 * usart.c
 *
 *  Created on: 25. 3. 2019
 *  Author:     Priesol Vladimir
 */

#include "usart.h"

#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_usart.h"

const usart_hw_t usart1_hw =
{
  .reg = USART1,
  .nAPBclock = LL_APB2_GRP1_PERIPH_USART1,
  .nGpioAF = LL_GPIO_AF_7,
  .irq = USART1_IRQn,
};

const usart_hw_t usart2_hw =
{
  .reg = USART2,
  .nAPBclock = LL_APB1_GRP1_PERIPH_USART2,
  .nGpioAF = LL_GPIO_AF_7,
  .irq = USART2_IRQn,
};

static void _usart_irq_handler(usart_drv_t *pDrv);

usart_drv_t _usart1_drv = { &usart1_hw, 0, 0, NULL, NULL, false, true };
usart_drv_t _usart2_drv = { &usart2_hw, 0, 0, NULL, NULL, false, true };

static void    (*g_rxCallback)(uint8_t);   ///< Callback function for receiving data

static uint8_t*  g_pBuffer;
static uint32_t  g_nLen;

void usart_Init(usart_drv_t* pDrv, void(*rxCb)(uint8_t), gpio_pins_e eTxPin, gpio_pins_e eRxPin, uint32_t nBaudrate)
{
  g_rxCallback = rxCb;

  if (pDrv->pHW->reg == USART1)
  {
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);
  }

  if (pDrv->pHW->reg == USART2)
  {
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);
  }

  /* USART TX pin configuration */
  GPIO_ConfigPin(eTxPin, mode_alternate, outtype_pushpull, pushpull_no, speed_high);
  GPIO_SetAFpin(eTxPin, pDrv->pHW->nGpioAF);

  /* USART RX pin configuration */
  GPIO_ConfigPin(eRxPin, mode_alternate, outtype_pushpull, pushpull_no, speed_high);
  GPIO_SetAFpin(eRxPin, pDrv->pHW->nGpioAF);

  LL_USART_InitTypeDef usartInit;
  LL_USART_StructInit(&usartInit);

  usartInit.BaudRate = nBaudrate;
  LL_USART_Init(pDrv->pHW->reg, &usartInit);

  LL_USART_Enable(pDrv->pHW->reg);

  /* Enable RXNE interrupt */
  LL_USART_EnableIT_RXNE(pDrv->pHW->reg);

  // Disable TXE interrupt - we enable it only when there is
  // data to send
  LL_USART_DisableIT_TXE(pDrv->pHW->reg);

  // Enable USART2 global interrupt
  NVIC_EnableIRQ(pDrv->pHW->irq);

}

void usart_Send(usart_drv_t* pDrv, uint8_t *buf, uint32_t len)
{
  pDrv->bComplete = false;
  pDrv->bLock = true;
  g_pBuffer = buf;
  g_nLen = len;
  LL_USART_EnableIT_TXE(pDrv->pHW->reg);
}

//void usart_Receive(usart_drv_t* pDrv, uint8_t *buf, uint16_t len)
//{
//  g_pBuffer = buf;
//  g_nLen = len;
//  USART_ITConfig(pDrv->pHW->reg, USART_IT_RXNE, ENABLE);
//}

void usart_WaitForTransmitComplete(usart_drv_t* pDrv)
{
  while (pDrv->bComplete == false);
}

void usart_PrintLn(usart_drv_t* pDrv, char* pText)
{
  usart_WaitForTransmitComplete(pDrv);
  while (!LL_USART_IsActiveFlag_TXE(pDrv->pHW->reg));
  while (*pText)
  {
    while (!LL_USART_IsActiveFlag_TXE(pDrv->pHW->reg));
    LL_USART_TransmitData8(pDrv->pHW->reg, *pText++);
  }

  while (!LL_USART_IsActiveFlag_TXE(pDrv->pHW->reg));
  LL_USART_TransmitData8(pDrv->pHW->reg, 13);

  while (!LL_USART_IsActiveFlag_TXE(pDrv->pHW->reg));
  LL_USART_TransmitData8(pDrv->pHW->reg, 10);

  while (!LL_USART_IsActiveFlag_TXE(pDrv->pHW->reg));
}

static void _usart_irq_handler(usart_drv_t *pDrv)
{
  if(LL_USART_IsActiveFlag_TXE(pDrv->pHW->reg))
  {
    if (g_nLen)
    {
      LL_USART_TransmitData8(pDrv->pHW->reg, *g_pBuffer); // Send data
      g_pBuffer++;
      g_nLen--;
    }
    else
    {
      // if no more data to send disable the transmitter
      LL_USART_DisableIT_TXE(pDrv->pHW->reg);
      pDrv->bComplete = true;
      pDrv->bLock = false;
    }
  }

  // If RX buffer not empty interrupt
  if(LL_USART_IsActiveFlag_RXNE(pDrv->pHW->reg))
  {
//    *g_pBuffer = USART_ReceiveData(pDrv->pHW->reg); // Get data from UART
//    g_nLen--;
//    if (g_nLen == 0)
//    {
//      USART_ITConfig(pDrv->pHW->reg, USART_IT_RXNE, DISABLE);
//    }
    uint8_t c = LL_USART_ReceiveData8(pDrv->pHW->reg);
    if (g_rxCallback)
    {
      g_rxCallback(c); // send received data to higher layer
    }
  }
}

void USART1_IRQHandler(void)
{
  _usart_irq_handler(usart1);
}

void USART2_IRQHandler(void)
{
  _usart_irq_handler(usart2);
}

void USART3_IRQHandler(void)
{
  _usart_irq_handler(usart3);
}
