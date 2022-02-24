/***************************************************************************
  This is a library for the LPS22HB Absolute Digital Barometer
  Designed to work with all kinds of LPS22HB Breakout Boards
  These sensors use I2C, 2 pins are required to interface, as this :
  VDD to 3.3V DC
  SCL to A5
  SDA to A4
  GND to common groud 
  Written by Adrien Chapelet for IoThings
 ***************************************************************************/

#include <Wire.h>

#include "LPS22HB.h"

LPS22HB::LPS22HB() { }

#ifdef ARDUINO_ARDUINO_NANO33BLE
void LPS22HB::begin(uint8_t address) {
  _address = address;
  Wire1.begin();
  write(LPS22HB_RES_CONF, 0x0); // resolution: temp=32, pressure=128
  write(LPS22HB_CTRL_REG1, 0x00); // one-shot mode
}

#else
void LPS22HB::begin(uint8_t address) {
  _address = address;
  Wire.begin();
  write(LPS22HB_RES_CONF, 0x0); // resolution: temp=32, pressure=128
  write(LPS22HB_CTRL_REG1, 0x00); // one-shot mode
}
#endif

#ifdef ARDUINO_ARDUINO_NANO33BLE
byte LPS22HB::whoAmI() {
  Wire1.beginTransmission(_address);
  Wire1.write(LPS22HB_WHO_AM_I);
  Wire1.endTransmission();
  Wire1.requestFrom(_address, 1);
  return Wire1.read();
}

#else
byte LPS22HB::whoAmI() {
  Wire.beginTransmission(_address);
  Wire.write(LPS22HB_WHO_AM_I);
  Wire.endTransmission();
  Wire.requestFrom(_address, 1);
  return Wire.read();
}
#endif

float LPS22HB::readPressure() {
  write(LPS22HB_CTRL_REG2, 0x1);

  if (status(0x1) < 0)
    return 1.23;
  //delay(50);
  uint8_t pressOutH = read(LPS22HB_PRES_OUT_H);
  uint8_t pressOutL = read(LPS22HB_PRES_OUT_L);
  uint8_t pressOutXL = read(LPS22HB_PRES_OUT_XL);

  long val = ( ((long)pressOutH << 16) | ((long)pressOutL << 8) | (long)pressOutXL );
  //if (val == 1.00) readPressure();
  return val/4096.0f;
}

uint32_t LPS22HB::readPressureRAW() {
  write(LPS22HB_CTRL_REG2, 0x1);

  if (status(0x1) < 0)
    return 123;
  //delay(50);
  uint8_t pressOutH = read(LPS22HB_PRES_OUT_H);
  uint8_t pressOutL = read(LPS22HB_PRES_OUT_L);
  uint8_t pressOutXL = read(LPS22HB_PRES_OUT_XL);

  int32_t val = ( (pressOutH << 16) | (pressOutL << 8) | pressOutXL );
  val=val+0x400000;
  //if (val == 1.00) readPressure();
  return (uint32_t)val;
}

uint32_t LPS22HB::readPressureUI() {
  write(LPS22HB_CTRL_REG2, 0x1);

  if (status(0x1) < 0)
    return 1.23;
  //delay(50);
  uint8_t pressOutH = read(LPS22HB_PRES_OUT_H);
  uint8_t pressOutL = read(LPS22HB_PRES_OUT_L);
  uint8_t pressOutXL = read(LPS22HB_PRES_OUT_XL);

  uint32_t val = ((pressOutH << 16) | (pressOutL << 8) | pressOutXL );
  //if (val == 1.00) readPressure();
  return val/4096;
}

float LPS22HB::readTemperature() {
  write(LPS22HB_CTRL_REG2, 0x1);
  if (status(0x2) < 0)
    return 4.56;

  uint8_t tempOutH = read(LPS22HB_TEMP_OUT_H);
  uint8_t tempOutL = read(LPS22HB_TEMP_OUT_L);

  int16_t val = (tempOutH << 8) | (tempOutL & 0xff);
  return ((float)val)/100.0f;
}


uint8_t LPS22HB::status(uint8_t status) {
  int count = 1000;
  uint8_t data = 0xff;
  do {
    data = read(LPS22HB_STATUS_REG);
    --count;
    if (count < 0)
      break;
  } while ((data & status) == 0);

  if (count < 0)
    return -1;
  else
    return 0;
}
#ifdef ARDUINO_ARDUINO_NANO33BLE
uint8_t LPS22HB::read(uint8_t reg) {
  Wire1.beginTransmission(_address);
  Wire1.write(reg);
  Wire1.endTransmission();
  Wire1.requestFrom(_address, 1);
  return Wire1.read();
}

#else
uint8_t LPS22HB::read(uint8_t reg) {
  Wire.beginTransmission(_address);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(_address, 1);
  return Wire.read();
}
#endif

#ifdef ARDUINO_ARDUINO_NANO33BLE
void LPS22HB::write(uint8_t reg, uint8_t data) {
  Wire1.beginTransmission(_address);
  Wire1.write(reg);
  Wire1.write(data);
  Wire1.endTransmission();
}

#else
void LPS22HB::write(uint8_t reg, uint8_t data) {
  Wire.beginTransmission(_address);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
}
#endif