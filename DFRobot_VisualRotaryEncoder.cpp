/*!
 * @file  DFRobot_VisualRotaryEncoder.cpp
 * @brief  DFRobot_VisualRotaryEncoder.cpp Initialize the I2C,
 * @n      get encoder basic information, button trigger condition and the current value, adjust encoder values and rotation gain
 * @copyright  Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license  The MIT License (MIT)
 * @author  [qsjhyy](yihuan.huang@dfrobot.com)
 * @version  V1.0
 * @date  2021-09-15
 * @url  https://github.com/DFRobot/DFRobot_VisualRotaryEncoder
 */
#include "DFRobot_VisualRotaryEncoder.h"

DFRobot_VisualRotaryEncoder::DFRobot_VisualRotaryEncoder()
{
}


int DFRobot_VisualRotaryEncoder::begin(void)
{
  uint8_t idBuf[2];
  if(0 == readReg(VISUAL_ROTARY_ENCODER_PID_MSB_REG, idBuf, sizeof(idBuf))){   // Judge whether the data bus is successful
    DBG("ERR_DATA_BUS");
    return ERR_DATA_BUS;
  }

  DBG("real sensor id="); DBG( (uint16_t)idBuf[0] << 8 | (uint16_t)idBuf[1], HEX );
  if( VISUAL_ROTARY_ENCODER_PID != ( (uint16_t)idBuf[0] << 8 | (uint16_t)idBuf[1] ) ){   // Judge whether the chip version matches
    DBG("ERR_IC_VERSION");
    return ERR_IC_VERSION;
  }

  usleep(20000);
  DBG("begin ok!");
  return NO_ERR;
}

void DFRobot_VisualRotaryEncoder::refreshBasicInfo(void)
{
  uint8_t tempBuf[8] = {0};
  readReg(VISUAL_ROTARY_ENCODER_PID_MSB_REG, tempBuf, sizeof(tempBuf));
  basicInfo.PID = (uint16_t)tempBuf[0] << 8 | (uint16_t)tempBuf[1];   // PID
  basicInfo.VID = (uint16_t)tempBuf[2] << 8 | (uint16_t)tempBuf[3];   // VID
  basicInfo.version = (uint16_t)tempBuf[4] << 8 | (uint16_t)tempBuf[5];   // version
  basicInfo.i2cAddr = (uint16_t)tempBuf[7];   // I2C addr
}

uint16_t DFRobot_VisualRotaryEncoder::getEncoderValue(void)
{
  uint8_t CountValue[2] = {0};
  readReg(VISUAL_ROTARY_ENCODER_COUNT_MSB_REG, CountValue, sizeof(CountValue));

  return (uint16_t)CountValue[0] << 8 | (uint16_t)CountValue[1];
}

void DFRobot_VisualRotaryEncoder::setEncoderValue(uint16_t value)
{
  if(0x3FF >= value){
    uint8_t tempBuf[2] = {0};
    tempBuf[0] = (value & 0xFF00) >> 8;
    tempBuf[1] = value & 0x00FF;
    writeReg(VISUAL_ROTARY_ENCODER_COUNT_MSB_REG, tempBuf, sizeof(tempBuf));
  }
}

uint8_t DFRobot_VisualRotaryEncoder::getGainCoefficient(void) {
  uint8_t RotateGain = 0;
  readReg(VISUAL_ROTARY_ENCODER_GAIN_REG, &RotateGain, sizeof(RotateGain));

  return RotateGain;
}

void DFRobot_VisualRotaryEncoder::setGainCoefficient(uint8_t gainValue) {
  if((0x01 <= gainValue) && (0x33 >= gainValue)){
    writeReg(VISUAL_ROTARY_ENCODER_GAIN_REG, &gainValue, sizeof(gainValue));
  }
}

bool DFRobot_VisualRotaryEncoder::detectButtonDown(void) {
  bool ret = false;
  uint8_t buttonStatus, clearStatus=0x00;

  readReg(VISUAL_ROTARY_ENCODER_KEY_STATUS_REG, &buttonStatus, sizeof(buttonStatus));

  if( 0 != (buttonStatus & 0x01) ){
    writeReg(VISUAL_ROTARY_ENCODER_KEY_STATUS_REG, &clearStatus, sizeof(clearStatus));
    ret = true;
  }

  return ret;
}

/************ Initialization of I2C interfaces reading and writing ***********/

DFRobot_VisualRotaryEncoder_I2C::DFRobot_VisualRotaryEncoder_I2C(uint8_t i2cAddr, int bus)
{
  if(initI2C_RW(bus, i2cAddr, -1))
	  fprintf(stderr, "Error initialising sensor on bus %d, addr %#x\n", i2cAddr, bus);
}

int DFRobot_VisualRotaryEncoder_I2C::begin(void)
{
  return DFRobot_VisualRotaryEncoder::begin();   // Use the initialization function of the parent class
}

#include <string.h>

void DFRobot_VisualRotaryEncoder_I2C::writeReg(uint8_t reg, const void* pBuf, size_t size)
{
  if(NULL == pBuf){
    DBG("pBuf ERROR!! : null pointer");
  }
  char buf[1 + size];
  buf[0] = reg;
  memcpy(buf + 1, pBuf, size);
  if(write(i2C_file, buf, sizeof(buf)) != sizeof(buf))
    fprintf(stderr, "Failed to write register %d on I2c sensor\n", reg);
}

size_t DFRobot_VisualRotaryEncoder_I2C::readReg(uint8_t reg, void* pBuf, size_t size)
{
  if(NULL == pBuf){
    DBG("pBuf ERROR!! : null pointer");
  }
  i2c_char_t outbuf = reg;
  struct i2c_rdwr_ioctl_data packets;
  struct i2c_msg messages[2];

  /* Reading a register involves a repeated start condition which needs ioctl() */
  messages[0].addr  = i2C_address;
  messages[0].flags = 0;
  messages[0].len   = sizeof(outbuf);
  messages[0].buf   = &outbuf;

  /* The data will get returned in this structure */
  messages[1].addr  = i2C_address;
  messages[1].flags = I2C_M_RD/* | I2C_M_NOSTART*/;
  messages[1].len   = size;
  messages[1].buf   = (uint8_t*)pBuf;

  /* Send the request to the kernel and get the result back */
  packets.msgs      = messages;
  packets.nmsgs     = 2;
  if(ioctl(i2C_file, I2C_RDWR, &packets) < 0) {
      fprintf(stderr, "Failed to read register %d on I2c sensor\n", reg);
      return -1;
  }
  return size;
}
