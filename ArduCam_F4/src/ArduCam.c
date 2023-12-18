/*
 * ArduCam.c
 *
 *  Created on: 15. 3. 2022
 *  Author:     Priesol Vladimir
 */

// https://www.arducam.com/docs/spi-cameras-for-arduino/detailed-data-timing-on-arducam-spi-bus/

#include "ArduCam.h"
#include "ov2640_regs.h"
#include "timer.h"

// jpeg_quality = 10;                     // 0-63 lower number means higher quality (reg 0x44, bank 0)

#define ARDUCAM_ADDR                    0x30

#define ARDUCAM_RWBIT                   0x80

#define ARDUCAM_TEST_REG                0x00
#define ARDUCAM_TEST_VALUE              0xA5

#define ARDUCAM_CAP_CTRL_REG            0x01

#define ARDUCAM_FIFO_CTRL_REG           0x04
#define ARDUCAM_FIFO_CLEAR_BIT          0x01
#define ARDUCAM_FIFO_START_BIT          0x02
#define ARDUCAM_FIFO_RDPTR_RST_BIT      0x10
#define ARDUCAM_FIFO_WRPTR_RST_BIT      0x20

#define ARDUCAM_BURST_FIFO_READ         0x3C  //Burst FIFO read operation
#define ARDUCAM_SINGLE_FIFO_READ        0x3D  //Single FIFO read operation

#define ARDUCAM_ARDUCHIP_VERSION        0x40

#define ARDUCAM_TRIG                    0x41  //Trigger register
#define ARDUCAM_VSYNC_BIT               0x01
#define ARDUCAM_SHUTTER_BIT             0x02
#define ARDUCAM_CAP_DONE_BIT            0x08

#define ARDUCAM_FIFO_SIZE1              0x42  //Camera write FIFO size[7:0] for burst to read
#define ARDUCAM_FIFO_SIZE2              0x43  //Camera write FIFO size[15:8]
#define ARDUCAM_FIFO_SIZE3              0x44  //Camera write FIFO size[18:16]

static spi_drv_t*               g_pSpi;
static spi_br_e                 g_eSpiPrescaler;
static i2cdrv_t*                g_pI2c;
static gpio_pins_e              g_eCS;            // camera SPI chip select

static ac_format_e  g_eFormat =         format_jpeg;

bool _WriteSensorReg8_8(uint8_t nRegID, uint8_t nData);
bool _ReadSensorReg8_8(uint8_t nRegID, uint8_t* nData);
bool _WriteSensorRegs8_8(const ac_sensor_reg_t reglist[]);


void ArduCam_Init(spi_drv_t* pSpi, gpio_pins_e eCS, spi_br_e ePrescaler, i2cdrv_t* pI2c)
{
  // configure SPI_CS
  g_pSpi = pSpi;
  g_eCS = eCS;
  g_eSpiPrescaler = ePrescaler;
  g_pI2c = pI2c;

  // set CS for output
  GPIO_SETPIN(g_eCS);
  GPIO_ConfigPin(g_eCS, mode_output, outtype_pushpull, pushpull_no, speed_veryhigh);

  _WriteSensorReg8_8(0xff, 0x01);
  _WriteSensorReg8_8(0x12, 0x80);
  Timer_Delay_ms(100);

  //Check if the camera module type is OV2640
  uint8_t nVid;
  uint8_t nPid;
  _WriteSensorReg8_8(0xff, 0x01);
  _ReadSensorReg8_8(OV2640_CHIPID_HIGH, &nVid);
  _ReadSensorReg8_8(OV2640_CHIPID_LOW, &nPid);
  if ((nVid != 0x26 ) && ((nPid != 0x41 ) || (nPid != 0x42 )))
  {
    while (1);
  }

  // configure regs
  if (g_eFormat == format_jpeg)
  {
    _WriteSensorRegs8_8(OV2640_JPEG_INIT);
    _WriteSensorRegs8_8(OV2640_YUV422);
    _WriteSensorRegs8_8(OV2640_JPEG);
    _WriteSensorReg8_8(0xff, 0x01);
    _WriteSensorReg8_8(0x15, 0x00);
    _WriteSensorRegs8_8(OV2640_320x240_JPEG);
    //wrSensorReg8_8(0xff, 0x00);
    //wrSensorReg8_8(0x44, 0x32);
  }
  else
  {
    _WriteSensorRegs8_8(OV2640_QVGA);
  }

  Timer_Delay_ms(1000);

  if (!ArduCam_Test())
  {
    while (1);
  }

}

bool ArduCam_Test(void)
{

  ArduCam_WriteSpiReg(ARDUCAM_TEST_REG, ARDUCAM_TEST_VALUE);
  if (ArduCam_ReadSpiReg(ARDUCAM_TEST_REG) != ARDUCAM_TEST_VALUE)
  {
    return false;
  }

  return true;
}

/**
 * flash fifo function is used to reset the fifo read pointer to ZERO
 */
void ArduCam_FlushFifo(void)
{
  ArduCam_WriteSpiReg(ARDUCAM_FIFO_CTRL_REG,  ARDUCAM_FIFO_RDPTR_RST_BIT);
  ArduCam_WriteSpiReg(ARDUCAM_FIFO_CTRL_REG,  ARDUCAM_FIFO_WRPTR_RST_BIT);
}

/**
Once a frame image is buffed to onboard memory, the capture completion flag is assert
ed
automatically. The clear_fifo_flag function is used to clear this flag before issuing next capture
command.
*/
void ArduCam_ClearFifoFlag(void)
{
  ArduCam_WriteSpiReg(ARDUCAM_FIFO_CTRL_REG, ARDUCAM_FIFO_CLEAR_BIT);
}

/**
 * start_capture function is
used to issue a capture command. After this command the ArduCAM
hardware will wait for a start of a new frame then store the entire frame data to onboard frame
buffer.
 */
void ArduCam_StartCapture(void)
{
  ArduCam_WriteSpiReg(ARDUCAM_FIFO_CTRL_REG, ARDUCAM_FIFO_START_BIT);
}

/**
function is used to determine the length of current captured image. No te the
Rev.C shield doesn't support this feature
 * @return 32 bit length of captured image
 */
uint32_t ArduCam_ReadFifoLength(void)
{
  uint32_t nSize1, nSize2, nSize3;

  nSize1 = ArduCam_ReadSpiReg(ARDUCAM_FIFO_SIZE1);
  nSize2 = ArduCam_ReadSpiReg(ARDUCAM_FIFO_SIZE2);
  nSize3 = ArduCam_ReadSpiReg(ARDUCAM_FIFO_SIZE3);

  return ((nSize3 << 16) | (nSize2 << 8) | nSize1) & 0x07fffff;
}

uint8_t ArduCam_ReadFifoSingle(void)
{
  return ArduCam_ReadSpiReg(ARDUCAM_SINGLE_FIFO_READ);
}

bool ArduCam_IsCaptureBit(void)
{
  uint8_t nValue = ArduCam_ReadSpiReg(ARDUCAM_TRIG);
  return nValue & ARDUCAM_CAP_DONE_BIT;
}

/**
 * Function reads data block from memory using burst mode
 * @param pBuffer
 * @param nLen
 */
void ArduCam_ReadFifoBlock(uint8_t* pBuffer, uint32_t nLen)
{
  spi_TransactionBegin(g_pSpi, g_eCS, g_eSpiPrescaler);
  spi_SendData8(g_pSpi, ARDUCAM_BURST_FIFO_READ);
  while (nLen--)
  {
    *pBuffer++ = spi_SendData8(g_pSpi, 0x00);
  }

  spi_TransactionEnd(g_pSpi, g_eCS);
}

uint8_t ArduCam_GetVersion(void)
{
  return ArduCam_ReadSpiReg(ARDUCAM_ARDUCHIP_VERSION);
}

uint8_t ArduCam_ReadSpiReg(uint8_t nAddr)
{
  spi_TransactionBegin(g_pSpi, g_eCS, g_eSpiPrescaler);
  spi_SendData8(g_pSpi, nAddr & 0x7F);
  uint8_t nValue = spi_SendData8(g_pSpi, 0x00);
  spi_TransactionEnd(g_pSpi, g_eCS);
  return nValue;
}

void ArduCam_WriteSpiReg(uint8_t nAddr, uint8_t nData)
{
  spi_TransactionBegin(g_pSpi, g_eCS, g_eSpiPrescaler);
  spi_SendData8(g_pSpi, nAddr | ARDUCAM_RWBIT);
  spi_SendData8(g_pSpi, nData);
  spi_TransactionEnd(g_pSpi, g_eCS);
}

void ArduCam_SetMode(uint8_t nMode)
{

}

void ArduCam_SetJpegSize(ac_jpeg_size_e eJpegSize)
{
  switch(eJpegSize)
    {
    case size_160x120:
      _WriteSensorRegs8_8(OV2640_160x120_JPEG);
      break;
    case size_176x144:
      _WriteSensorRegs8_8(OV2640_176x144_JPEG);
      break;
    case size_320x240:
      _WriteSensorRegs8_8(OV2640_320x240_JPEG);
      break;
    case size_352x288:
      _WriteSensorRegs8_8(OV2640_352x288_JPEG);
      break;
    case size_640x480:
      _WriteSensorRegs8_8(OV2640_640x480_JPEG);
      break;
    case size_800x600:
      _WriteSensorRegs8_8(OV2640_800x600_JPEG);
      break;
    case size_1024x768:
      _WriteSensorRegs8_8(OV2640_1024x768_JPEG);
      break;
    case size_1280x1024:
      _WriteSensorRegs8_8(OV2640_1280x1024_JPEG);
      break;
    case size_1600x1200:
      _WriteSensorRegs8_8(OV2640_1600x1200_JPEG);
      break;
    default:
      _WriteSensorRegs8_8(OV2640_320x240_JPEG);
      break;
  }
}

void ArduCam_SetFormat(ac_format_e eFormat)
{
  g_eFormat = eFormat;
}

void ArduCam_SetLighMode(ac_light_mode_e eLighMode)
{
  switch (eLighMode)
  {
  case ligh_mode_auto:
    _WriteSensorReg8_8(0xff, 0x00);
    _WriteSensorReg8_8(0xc7, 0x00); //AWB on
    break;
  case ligh_mode_sunny:
    _WriteSensorReg8_8(0xff, 0x00);
    _WriteSensorReg8_8(0xc7, 0x40); //AWB off
    _WriteSensorReg8_8(0xcc, 0x5e);
    _WriteSensorReg8_8(0xcd, 0x41);
    _WriteSensorReg8_8(0xce, 0x54);
    break;
  case ligh_mode_cloudy:
    _WriteSensorReg8_8(0xff, 0x00);
    _WriteSensorReg8_8(0xc7, 0x40); //AWB off
    _WriteSensorReg8_8(0xcc, 0x65);
    _WriteSensorReg8_8(0xcd, 0x41);
    _WriteSensorReg8_8(0xce, 0x4f);
    break;
  case ligh_mode_office:
    _WriteSensorReg8_8(0xff, 0x00);
    _WriteSensorReg8_8(0xc7, 0x40); //AWB off
    _WriteSensorReg8_8(0xcc, 0x52);
    _WriteSensorReg8_8(0xcd, 0x41);
    _WriteSensorReg8_8(0xce, 0x66);
    break;
  case ligh_mode_home:
    _WriteSensorReg8_8(0xff, 0x00);
    _WriteSensorReg8_8(0xc7, 0x40); //AWB off
    _WriteSensorReg8_8(0xcc, 0x42);
    _WriteSensorReg8_8(0xcd, 0x3f);
    _WriteSensorReg8_8(0xce, 0x71);
    break;
  default:
    _WriteSensorReg8_8(0xff, 0x00);
    _WriteSensorReg8_8(0xc7, 0x00); //AWB on
    break;
  }
}

void ArduCam_SetBrightness(ac_brightness_e eBrightness)
{
  const uint8_t brightnessLUT[] = { 0x40, 0x30, 0x20, 0x10, 0x00};

  if (eBrightness < brightness_sizeof)
  {
    _WriteSensorReg8_8(0xff, 0x00);
    _WriteSensorReg8_8(0x7c, 0x00);
    _WriteSensorReg8_8(0x7d, 0x04);
    _WriteSensorReg8_8(0x7c, 0x09);
    _WriteSensorReg8_8(0x7d, brightnessLUT[eBrightness]);
    _WriteSensorReg8_8(0x7d, 0x00);
  }
}

void ArduCam_SetContrast(ac_contrast_e eContrast)
{
//  const uint8_t contrastLUT[] = { 0x40, 0x30, 0x20, 0x10, 0x00};
//
//  if (eContrast < contrast_sizeof)
//  {
//    _WriteSensorReg8_8(0xff, 0x00);
//    _WriteSensorReg8_8(0x7c, 0x00);
//    _WriteSensorReg8_8(0x7d, 0x04);
//    _WriteSensorReg8_8(0x7c, 0x09);
//    _WriteSensorReg8_8(0x7d, contrastLUT[eContrast]);
//    _WriteSensorReg8_8(0x7d, 0x00);
//  }
}

void ArduCam_SetColorSaturation(ac_saturation_e eSaturation)
{
  const uint8_t saturationLUT[] = { 0x68, 0x58, 0x48, 0x38, 0x28};

  if (eSaturation < saturation_sizeof)
  {
    _WriteSensorReg8_8(0xff, 0x00);
    _WriteSensorReg8_8(0x7c, 0x00);
    _WriteSensorReg8_8(0x7d, 0x02);
    _WriteSensorReg8_8(0x7c, 0x03);
    _WriteSensorReg8_8(0x7d, saturationLUT[eSaturation]);
    _WriteSensorReg8_8(0x7d, saturationLUT[eSaturation]);
  }
}

bool _WriteSensorReg8_8(uint8_t nRegID, uint8_t nData)
{
  uint8_t buf[2];
  buf[0] = nRegID;
  buf[1] = nData;

  return i2c_transfer(g_pI2c, ARDUCAM_ADDR, buf, 2, NULL, 0);
}

bool _ReadSensorReg8_8(uint8_t nRegID, uint8_t* nData)
{
  return i2c_transfer(g_pI2c, ARDUCAM_ADDR, &nRegID, 1, nData, 1);
}

bool _WriteSensorRegs8_8(const ac_sensor_reg_t reglist[])
{
  while ((reglist->reg != 0xff) | (reglist->val != 0xff))
  {
    if (!_WriteSensorReg8_8(reglist->reg, reglist->val))
    {
      return false;
    }

    reglist++;
  }

  return true;
}
