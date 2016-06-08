/**************************************************************************/
/*!
    @file     Adafruit_Max30100.cpp
    @author   Limor Fried (Adafruit Industries)
    @license  BSD (see license.txt)

    This is a library for the Adafruit Max30100 breakout board
    ----> https://www.adafruit.com/products/xxxx

    Adafruit invests time and resources providing this open source code,
    please support Adafruit and open-source hardware by purchasing
    products from Adafruit!

    @section  HISTORY

    v1.0  - First release
*/
/**************************************************************************/

#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#include <Wire.h>
#include <Adafruit_Max30100.h>


/**************************************************************************/

Adafruit_MAX30100::Adafruit_MAX30100(void) {
  _i2caddr = MAX30100_DEFAULT_ADDRESS;
}

bool Adafruit_MAX30100::begin(void) {
  Wire.begin();
  
  if (readRegister8(MAX30100_PARTID_REG) != MAX30100_PARTID_VAL)
    return false;

  // reset!
  writeRegister8(MAX30100_MODECFG_REG, MAX30100_MODECFG_RESET);
  while (readRegister8(MAX30100_MODECFG_REG) &  MAX30100_MODECFG_RESET) {
    delay(1);
  }
  
  readRegister8(MAX30100_IRQSTAT_REG); // clear pwr ready int!

  writeRegister8(MAX30100_IRQENABLE_REG, MAX30100_IRQENABLE_HRREADY | MAX30100_IRQENABLE_SPO2READY);
  return true;
}

float Adafruit_MAX30100::readTemperature(void) {
  uint8_t v;
  v = readRegister8(MAX30100_MODECFG_REG);

  writeRegister8(MAX30100_MODECFG_REG, v | MAX30100_MODECFG_TEMPEN);

  while (readRegister8(MAX30100_MODECFG_REG) & MAX30100_MODECFG_TEMPEN) {
    delay(1);
  }

  uint16_t tempreading = readRegister16(MAX30100_TEMPDATAINT_REG);

  uint8_t tfrac = tempreading & 0xF;
  int8_t tint = tempreading >> 8;

  float temp = tint;
  temp += tfrac * 0.0625;
  return temp;

}


void Adafruit_MAX30100::setSpO2SampleRate(max30100_spo2_samplerate_t rate) {
  uint8_t v;
  v = readRegister8(MAX30100_SPO2CFG_REG);

  v &= 0b11100011; // clear out old settings
  v |= rate << 2;
  writeRegister8(MAX30100_SPO2CFG_REG, v);
}

max30100_spo2_samplerate_t Adafruit_MAX30100::getSpO2SampleRate(void) {
  uint8_t v;
  v = readRegister8(MAX30100_SPO2CFG_REG);
  v >>= 2;
  v &= 0x7; // mask off only the 3 bits

  return (max30100_spo2_samplerate_t)v;
}

void Adafruit_MAX30100::setLEDpulseWidth(max30100_led_pulsewidth_t pw) {
  uint8_t v;
  v = readRegister8(MAX30100_SPO2CFG_REG);
  v &= 0b11111100; // clear out old settings
  v |= pw;
  writeRegister8(MAX30100_SPO2CFG_REG, v);
}

max30100_led_pulsewidth_t Adafruit_MAX30100::getLEDpulseWidth(void) {
  uint8_t v;
  v = readRegister8(MAX30100_SPO2CFG_REG);
  v &= 0x03;
  return (max30100_led_pulsewidth_t)v;
}

void Adafruit_MAX30100::setRedLEDcurrent(max30100_led_current_t i) {
  uint8_t v;
  v = readRegister8(MAX30100_LEDCFG_REG);
  v &= 0x0F; // clear out old settings
  v |= i << 4;
  writeRegister8(MAX30100_LEDCFG_REG, v);
}

max30100_led_current_t Adafruit_MAX30100::getRedLEDcurrent() {
  uint8_t v;
  v = readRegister8(MAX30100_LEDCFG_REG);
  return (max30100_led_current_t)(v >> 4);
}

void Adafruit_MAX30100::setIRLEDcurrent(max30100_led_current_t i) {
  uint8_t v;
  v = readRegister8(MAX30100_LEDCFG_REG);
  v &= 0xF0; // clear out old settings
  v |= i;
  writeRegister8(MAX30100_LEDCFG_REG, v);
}

max30100_led_current_t Adafruit_MAX30100::getIRLEDcurrent() {
  uint8_t v;
  v = readRegister8(MAX30100_LEDCFG_REG);
  return (max30100_led_current_t)(v & 0xF);
}

void Adafruit_MAX30100::startRead(void) {
  writeRegister8(MAX30100_MODECFG_REG, 0x0); // disable
  delay(10);

  // set to spo2 + HR mode
  writeRegister8(MAX30100_MODECFG_REG, MAX30100_MODECFG_HRSP02);
  writeRegister8( MAX30100_FIFO_RDPTR_REG, 0);
  writeRegister8( MAX30100_FIFO_WRPTR_REG, 0);
}

uint8_t Adafruit_MAX30100::readFIFO(uint32_t *fifoptr, uint8_t maxentries) {
  uint32_t t;
  int8_t numsamples = 0;

  do {
    int8_t wrptr = readRegister8(MAX30100_FIFO_WRPTR_REG);
    int8_t rdptr = readRegister8(MAX30100_FIFO_RDPTR_REG);
    
    numsamples = wrptr - rdptr;
    if (numsamples < 0) numsamples +=16;
  
    numsamples = min(maxentries, numsamples);
    //Serial.print(numsamples); Serial.println(" samples avail");
  } while (numsamples < maxentries);

  Wire.beginTransmission(_i2caddr);
  Wire.write((uint8_t)MAX30100_FIFO_DATA_REG);
  Wire.endTransmission(false);

  Wire.requestFrom(_i2caddr, numsamples * 4);
  //Serial.println(Wire.available());
  for (uint8_t i=0; i<8; i++) {
    t = Wire.read();
    t <<= 8;
    t |= Wire.read();
    t <<= 8;
    t |= Wire.read();
    t <<= 8;
    t |= Wire.read();

    fifoptr[i] = t;
  }
  return numsamples;
}

/*******************************************************************/

void Adafruit_MAX30100::writeRegister8(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(_i2caddr);
  Wire.write((uint8_t)reg);
  Wire.write((uint8_t)value);
  Wire.endTransmission();

  //Serial.print("Wrote $"); Serial.print(reg, HEX); Serial.print(": 0x"); Serial.println(value, HEX);
}

uint8_t Adafruit_MAX30100::readRegister8(uint8_t reg) {
  uint8_t value;
  Wire.beginTransmission(_i2caddr);
  Wire.write((uint8_t)reg);
  Wire.endTransmission(false);

  Wire.requestFrom(_i2caddr, 1);
  value = Wire.read();

  //Serial.print("Read $"); Serial.print(reg, HEX); Serial.print(": 0x"); Serial.println(value, HEX);
  return value;
}

uint16_t Adafruit_MAX30100::readRegister16(uint8_t reg) {
  uint16_t value;
  Wire.beginTransmission(_i2caddr);
  Wire.write((uint8_t)reg);
  Wire.endTransmission();

  Wire.requestFrom(_i2caddr, 2);
  value = Wire.read();
  value <<= 8;
  value |= Wire.read();

  //Serial.print("Read $"); Serial.print(reg, HEX); Serial.print(": 0x"); Serial.println(value, HEX);
  return value;
}

uint32_t Adafruit_MAX30100::readRegister32(uint8_t reg) {
  uint32_t value;
  Wire.beginTransmission(_i2caddr);
  Wire.write((uint8_t)reg);
  Wire.endTransmission();

  Wire.requestFrom(_i2caddr, 4);
  value = Wire.read();
  value <<= 8;
  value |= Wire.read();
  value <<= 8;
  value |= Wire.read();
  value <<= 8;
  value |= Wire.read();

  //Serial.print("Read $"); Serial.print(reg, HEX); Serial.print(": 0x"); Serial.println(value, HEX);
  return value;
}



