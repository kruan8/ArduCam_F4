/*
 * i2c_int.c
 *
 *  Created on: 15. 3. 2022
 *  Author:     Priesol Vladimir
 *
 *          STM32F411
 *
 *  according to
 *  https://github.com/elliottt/stm32f4/blob/master/stm32f4/src/i2c.c
 */


#include "i2c_int.h"
#include "string.h"

#include "stm32f4xx_ll_i2c.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_bus.h"

const i2chw_t i2c1_hw =
{
  .reg = I2C1,
  .nAPBclock = LL_APB1_GRP1_PERIPH_I2C1,
  .nGpioAF = LL_GPIO_AF_4,
  .irq_ev = I2C1_EV_IRQn,
  .irq_er = I2C1_ER_IRQn
};

const i2chw_t i2c2_hw =
{
  .reg = I2C2,
  .nAPBclock = LL_APB1_GRP1_PERIPH_I2C2,
  .nGpioAF = LL_GPIO_AF_4,
  .irq_ev = I2C2_EV_IRQn,
  .irq_er = I2C2_ER_IRQn
};

const i2chw_t i2c3_hw =
{
  .reg = I2C3,
  .nAPBclock = LL_APB1_GRP1_PERIPH_I2C3,
  .nGpioAF = LL_GPIO_AF_4,
  .irq_ev = I2C3_EV_IRQn,
  .irq_er = I2C3_ER_IRQn
};


i2cdrv_t _i2c1_drv = { &i2c1_hw, 0, 0, 0, NULL, NULL, false, false, false };
i2cdrv_t _i2c2_drv = { &i2c2_hw, 0, 0, 0, NULL, NULL, false, false, false };
i2cdrv_t _i2c3_drv = { &i2c3_hw, 0, 0, 0, NULL, NULL, false, false, false };


/**
 * Generate an I2C START condition.  Per the reference manual, we wait
 * for the hardware to clear the start bit after setting it before
 * allowing any further writes to CR1.  This prevents random lockups.
 */
static inline void __i2c_set_start(i2cdrv_t *i2c)
{
  i2c->pHW->reg->CR1 |= I2C_CR1_START;
  while (i2c->pHW->reg->CR1 & I2C_CR1_START);
}

/**
 * Generate an I2C STOP condition.  Per the reference manual, we wait
 * for the hardware to clear the stop bit after setting it before
 * allowing any further writes to CR1.  This prevents random lockups.
 */
static inline void __i2c_set_stop(i2cdrv_t *i2c)
{
  i2c->pHW->reg->CR1 |= I2C_CR1_STOP;
  while (i2c->pHW->reg->CR1 & I2C_CR1_STOP);
}

/**
 * Set the peripheral frequency.
 */
static inline void __i2c_set_freq(i2cdrv_t *i2c, uint32_t nBusClock)
{
//  i2c->pHW->reg->CR2 &= ~I2C_CR2_FREQ;
//  __i2c_update_cr2(i2c, freq->pclk1 / 1000000);
}

void i2c_init(i2cdrv_t *pDrv, gpio_pins_e eSdaPin, gpio_pins_e eSclPin)
{
  GPIO_ConfigPin(eSdaPin, mode_input, outtype_od, pushpull_no, speed_high);

  // busy SDA line check and release
  if ((GPIO_GetPort(eSdaPin)->IDR & GPIO_GetPin(eSdaPin)) == 0)
  {
    GPIO_ConfigPin(eSclPin, mode_output, outtype_od, pushpull_no, speed_high);

    LL_GPIO_ResetOutputPin(GPIO_GetPort(eSclPin), GPIO_GetPin(eSclPin));
    while ((GPIO_GetPort(eSdaPin)->IDR & GPIO_GetPin(eSdaPin)) == 0)
    {
      LL_GPIO_SetOutputPin(GPIO_GetPort(eSclPin), GPIO_GetPin(eSclPin));

      // delay min 10us
      for (uint32_t i = 0; i < 1500; ++i)
      {
        asm("nop");
      }

      LL_GPIO_ResetOutputPin(GPIO_GetPort(eSclPin), GPIO_GetPin(eSclPin));
      // delay min 10us
      for (uint32_t i = 0; i < 1500; ++i)
      {
        asm("nop");
      }
    }
  }

  LL_APB1_GRP1_EnableClock(pDrv->pHW->nAPBclock);

  // Configure I2C pins: SCL and SDA
  GPIO_ConfigPin(eSclPin, mode_alternate, outtype_od, pushpull_no, speed_high);
  GPIO_SetAFpin(eSclPin, pDrv->pHW->nGpioAF);

  GPIO_ConfigPin(eSdaPin, mode_alternate, outtype_od, pushpull_no, speed_high);
  GPIO_SetAFpin(eSdaPin, pDrv->pHW->nGpioAF);

  /* Reset and clear peripheral. */
  pDrv->pHW->reg->CR1 = I2C_CR1_SWRST;
  pDrv->pHW->reg->CR1 = 0;

  //  struct rcc_clocks_freq freq;
  //  rcc_get_clocks_freq(&freq);

  //  // fast speed init
  //  uint16_t result = (uint16_t)(freq.pclk1 / (400000 * 25));
  //  if (result < 1)
  //    result = 1;
  //
  //  reg->CCR |= I2C_CCR_DUTY | I2C_CCR_FS | result;
  //  reg->TRISE = (uint16_t)((((freq.pclk1 / 1000000) * 300) / 1000) + 1);

    // standard speed init
#if 0
  uint16_t result = (uint16_t)(freq.pclk1 / (100000 << 1));
  if(result < 0x4) {
      result = 0x04;
  }
  reg->CCR |= result;
  reg->TRISE = ((freq.pclk1 / 1000000 + 1) & I2C_TRISE_TRISE);
#endif

  /* I2C configuration */
  LL_I2C_InitTypeDef I2C_InitStructure;
  I2C_InitStructure.PeripheralMode = LL_I2C_MODE_I2C;
  I2C_InitStructure.DutyCycle = LL_I2C_DUTYCYCLE_2;
  I2C_InitStructure.OwnAddress1 = 0x00;
  I2C_InitStructure.TypeAcknowledge = LL_I2C_ACK;
  I2C_InitStructure.OwnAddrSize = LL_I2C_OWNADDRESS1_7BIT;
  I2C_InitStructure.ClockSpeed = 100000; // PCLK1 must be a multiple of 10MHz to reach the 400 kHz maximum I2C Fm mode clock.
  I2C_InitStructure.AnalogFilter = LL_I2C_ANALOGFILTER_ENABLE;
  I2C_InitStructure.DigitalFilter = 0x00;
  LL_I2C_Init(pDrv->pHW->reg, &I2C_InitStructure);

  // enable interrupts
  pDrv->pHW->reg->CR2 |= I2C_CR2_ITERREN  /* Error interrupt enable */
      | I2C_CR2_ITEVTEN  /* Event interrupt enable */
      | I2C_CR2_ITBUFEN  /* Buffer interrupt enable */
      ;

  /* Enable interrupts globally */
  NVIC_SetPriority(pDrv->pHW->irq_ev, 2);
  NVIC_SetPriority(pDrv->pHW->irq_er, 2);
  NVIC_EnableIRQ(pDrv->pHW->irq_ev);
  NVIC_EnableIRQ(pDrv->pHW->irq_er);

  /* Enable the I2C peripheral */
  pDrv->pHW->reg->CR1 = I2C_CR1_PE;
}

bool i2c_transfer(i2cdrv_t *drv, uint8_t addr,
                  const uint8_t *tx_buf, size_t tx_len,
                  uint8_t *rx_buf, size_t rx_len)
{
  bool result = true;

  if (drv->bLock)
  {
    return false;
  }

  drv->nAddr     = addr;
  drv->nWriteLen = tx_len;
  drv->pWrite    = (uint8_t *)tx_buf;
  drv->nReadLen  = rx_len;
  drv->pRead     = rx_buf;

  if (drv->bError)
  {
    drv->bError = false;
    result = false;
  }

  drv->bLock = true;
  drv->bComplete = false;
  __i2c_set_start(drv);

  while (!drv->bComplete);
  drv->bLock = false;
  return result;
}

bool i2c_write(i2cdrv_t *drv, uint8_t addr, uint8_t *buf, size_t len)
{
  return i2c_transfer(drv, addr, buf, len, NULL, 0);
}

bool i2c_read(i2cdrv_t *pDrv, uint8_t addr, uint8_t *buf, uint8_t len)
{
  return i2c_transfer(pDrv, addr, NULL, 0, buf, len);
}

bool i2c_write_reg(i2cdrv_t *drv, uint8_t addr, uint8_t reg, uint8_t data)
{
  uint8_t buf[2] = { reg, data };
  return i2c_write(drv, addr, buf, sizeof(buf));
}

bool i2c_write_regs(i2cdrv_t *drv, uint8_t addr, uint8_t reg, uint8_t *data, uint8_t len)
{
  uint8_t buf[len + 1];    // requires C99, careful of stack overflow!
  buf[0] = reg;
  memcpy(&buf[1], data, len);
  return i2c_transfer(drv, addr, buf, len + 1, NULL, 0);
}

bool i2c_read_reg(i2cdrv_t *drv, uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
  return i2c_transfer(drv, addr, &reg, 1, buf, len);
}

static void __i2c_event_irq_handler(i2cdrv_t *drv)
{
  I2C_TypeDef* i2cReg = drv->pHW->reg;

  /* Read both status registers*/
  uint16_t sr1 = i2cReg->SR1;
  i2cReg->SR2;

  /* Start bit sent. */
  if (sr1 & I2C_SR1_SB)
  {
    i2cReg->DR = (drv->nAddr << 1) | (drv->nWriteLen ? 0 : 1);
  }

  /* Address sent. */
  if (sr1 & I2C_SR1_ADDR)
  {
    if (drv->nWriteLen)
    {
      /* Send a byte off the write buffer. */
      i2cReg->DR = *(drv->pWrite);
      drv->pWrite++;
      drv->nWriteLen--;
    }
    else
    {
      if (drv->nReadLen > 1)
      {
        /* Send ack on next read byte */
        i2cReg->CR1 |= I2C_CR1_ACK;
      }
      else
      {
        /* One byte left to read, send stop afterwards. */
        __i2c_set_stop(drv);
      }
    }
  }

  /* RX Not empty (got new byte) */
  if (sr1 & I2C_SR1_RXNE)
  {
    /* Read into read buffer. */
    *(drv->pRead) = i2cReg->DR;
    drv->pRead++;
    drv->nReadLen--;

    if (drv->nReadLen == 1)
    {
      /* Unset Ack, set Stop */
      i2cReg->CR1 &= ~I2C_CR1_ACK;
      __i2c_set_stop(drv);
    }

    if (drv->nReadLen == 0)
    {
      drv->bComplete = true;
    }
  }

  /* TXE set, BTF clear: tx buffer empty, still writing. */
  if (sr1 & I2C_SR1_TXE && !(sr1 & I2C_SR1_BTF))
  {
    if (drv->nWriteLen)
    {
      /* send next byte from write buffer. */
      i2cReg->DR = *(drv->pWrite);
      drv->pWrite++;
      drv->nWriteLen--;
    }
    else
    {
      if (drv->nReadLen)
      {
        /* done writing, now reading: send repeated stat */
        __i2c_set_start(drv);
      }
      else
      {
        /* done reading: send stop */
        __i2c_set_stop(drv);
        drv->bComplete = true;
      }
    }
  }

}

static void __i2c_error_irq_handler(i2cdrv_t *drv)
{
  /* Read SRs to clear them */
  drv->pHW->reg->SR1;
  drv->pHW->reg->SR2;

  /* Write 0 to SR1 ?? XXX why  */
  drv->pHW->reg->SR1 = 0;

  /* Send stop */
//  __i2c_set_stop(drv);   //  zamrzava
  drv->bError = 1;
}

/* Static per-hw event handlers */
void I2C1_EV_IRQHandler (void)
{
  __i2c_event_irq_handler(i2c1);
}

void I2C2_EV_IRQHandler (void)
{
  __i2c_event_irq_handler(i2c2);
}

void I2C3_EV_IRQHandler (void)
{
  __i2c_event_irq_handler(i2c3);
}

/* Static per-hw error handlers */
void I2C1_ER_IRQHandler (void)
{
  __i2c_error_irq_handler(i2c1);
}

void I2C2_ER_IRQHandler (void)
{
  __i2c_error_irq_handler(i2c2);
}

void I2C3_ER_IRQHandler (void)
{
  __i2c_error_irq_handler(i2c3);
}
