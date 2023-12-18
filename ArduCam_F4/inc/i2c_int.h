/*
 * i2c_int.h
 *
 *  Created on: 7. 3. 2019
 *  Author:     Priesol Vladimir
 */

#ifndef I2C_INT_H_
#define I2C_INT_H_

#include "stm32f4xx.h"
#include "stdbool.h"
#include "stddef.h"
#include "common_f4.h"

typedef struct
{
    I2C_TypeDef* reg;
    uint32_t nAPBclock;
    uint8_t  nGpioAF;
    IRQn_Type irq_ev;
    IRQn_Type irq_er;
} i2chw_t;

typedef struct
{
    i2chw_t const * pHW;
    volatile uint8_t   nAddr;
    volatile uint16_t  nWriteLen;
    volatile uint16_t  nReadLen;
    volatile uint8_t*  pWrite;
    volatile uint8_t*  pRead;

    volatile bool bError;

    bool bLock;      /* bus lock */
    bool bComplete;  /* completion from ISR */
} i2cdrv_t;

void i2c_init(i2cdrv_t *pDrv, gpio_pins_e eSdaPin, gpio_pins_e eSclPin);

/**
 * Perform a generic I2C write/read transfer.
 *
 * First, "tx_len" bytes are written from "tx_buf".  Then "rx_len"
 * bytes are read into "rx_buf".
 *
 */
bool i2c_transfer(i2cdrv_t *drv, uint8_t addr, const uint8_t *tx_buf, size_t tx_len, uint8_t *rx_buf, size_t rx_len);

bool i2c_read_reg(i2cdrv_t *drv, uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf);

bool i2c_write(i2cdrv_t *drv, uint8_t addr, uint8_t *buf, size_t len);
bool i2c_read(i2cdrv_t *pDrv, uint8_t addr, uint8_t *buf, uint8_t len);

bool i2c_write_reg(i2cdrv_t *drv, uint8_t addr, uint8_t reg, uint8_t data);

/** Write multiple registers, assuming auto-incrementing. */
bool i2c_write_regs(i2cdrv_t *drv, uint8_t addr, uint8_t reg, uint8_t *data, uint8_t len);

extern i2cdrv_t _i2c1_drv;
#define i2c1 (&_i2c1_drv)

extern i2cdrv_t _i2c2_drv;
#define i2c2 (&_i2c2_drv)

extern i2cdrv_t _i2c3_drv;
#define i2c3 (&_i2c3_drv)

#endif /* I2C_INT_H_ */
