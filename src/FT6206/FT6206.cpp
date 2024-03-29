/*
* This file started out as an adafruit driver. I have mutated it.
*                                                  ---J. Ian Lindsay  2021.09.09
*/

/*!
 * @file FT6206.cpp
 *
 * @mainpage Adafruit FT2606 Library
 *
 * @section intro_sec Introduction
 *
 * This is a library for the Adafruit Capacitive Touch Screens
 *
 * ----> http://www.adafruit.com/products/1947
 *
 * Check out the links above for our tutorials and wiring diagrams
 * This chipset uses I2C to communicate
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * @section author Author
 *
 * Written by Limor Fried/Ladyada for Adafruit Industries.
 *
 * @section license License

 * MIT license, all text above must be included in any redistribution
 */

#include <AbstractPlatform.h>
#include <FlagContainer.h>
#include "BusQueue/I2CAdapter.h"


/**************************************************************************/
/*!
    @brief  Setups the I2C interface and hardware, identifies if chip is found
    @param  thresh Optional threshhold-for-touch value, default is
   FT6206_DEFAULT_THRESSHOLD but you can try changing it if your screen is
   too/not sensitive.
    @returns True if an FT6206 is found, false on any failure
*/
/**************************************************************************/
int8_t FT6206::init(uint8_t thresh) {
  int8_t ret = -1;
#ifdef FT6206_DEBUG
  Serial.print("Vend ID: 0x");
  Serial.println(readRegister8(FT62XX_REG_VENDID), HEX);
  Serial.print("Chip ID: 0x");
  Serial.println(readRegister8(FT62XX_REG_CHIPID), HEX);
  Serial.print("Firm V: ");
  Serial.println(readRegister8(FT62XX_REG_FIRMVERS));
  Serial.print("Point Rate Hz: ");
  Serial.println(readRegister8(FT62XX_REG_POINTRATE));
  Serial.print("Thresh: ");
  Serial.println(readRegister8(FT62XX_REG_THRESHHOLD));

  // dump all registers
  for (int16_t i = 0; i < 0x10; i++) {
    Serial.print("I2C $");
    Serial.print(i, HEX);
    Serial.print(" = 0x");
    Serial.println(readRegister8(i), HEX);
  }
#endif

  // change threshhold to be higher/lower
  writeRegister8(FT62XX_REG_THRESHHOLD, thresh);

  if (readRegister8(FT62XX_REG_VENDID) != FT62XX_VENDID) {
    return false;
  }
  uint8_t id = readRegister8(FT62XX_REG_CHIPID);
  if ((id != FT6206_CHIPID) && (id != FT6236_CHIPID) &&
      (id != FT6236U_CHIPID)) {
    return false;
  }

  return 0;
}

/**************************************************************************/
/*!
    @brief  Determines if there are any touches detected
    @returns Number of touches detected, can be 0, 1 or 2
*/
/**************************************************************************/
uint8_t FT6206::touched() {
  uint8_t n = readRegister8(FT62XX_REG_NUMTOUCHES);
  if (n > 2) {
    n = 0;
  }
  return n;
}

/**************************************************************************/
/*!
    @brief  Queries the chip and retrieves a point data
    @param  n The # index (0 or 1) to the points we can detect. In theory we can
   detect 2 points but we've found that you should only use this for
   single-touch since the two points cant share the same half of the screen.
    @returns {@link TS_Point} object that has the x and y coordinets set. If the
   z coordinate is 0 it means the point is not touched. If z is 1, it is
   currently touched.
*/
/**************************************************************************/
Vector3u16* FT6206::getPoint(uint8_t n) {
  readData();
  if ((touches == 0) || (n > FT62XX_TOUCH_BACKLOG_LENGTH)) {
    return &touches[0];
  }
  else {
    return &touches[n];
  }
}

/************ lower level i/o **************/

/**************************************************************************/
/*!
    @brief  Reads the bulk of data from captouch chip. Fill in {@link touches},
   {@link touchX}, {@link touchY} and {@link touchID} with results
*/
/**************************************************************************/
void FT6206::readData() {

  uint8_t i2cdat[16];
  Wire.beginTransmission(FT62XX_ADDR);
  Wire.write((byte)0);
  Wire.endTransmission();

  Wire.requestFrom((byte)FT62XX_ADDR, (byte)16);
  for (uint8_t i = 0; i < 16; i++)
    i2cdat[i] = Wire.read();

#ifdef FT6206_DEBUG
  for (int16_t i = 0; i < 16; i++) {
    Serial.print("I2C $");
    Serial.print(i, HEX);
    Serial.print(" = 0x");
    Serial.println(i2cdat[i], HEX);
  }
#endif

  touches = i2cdat[0x02];
  if ((touches > 2) || (touches == 0)) {
    touches = 0;
  }

#ifdef FT6206_DEBUG
  Serial.print("# Touches: ");
  Serial.println(touches);

  for (uint8_t i = 0; i < 16; i++) {
    Serial.print("0x");
    Serial.print(i2cdat[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
  if (i2cdat[0x01] != 0x00) {
    Serial.print("Gesture #");
    Serial.println(i2cdat[0x01]);
  }
#endif

  for (uint8_t i = 0; i < 2; i++) {
    uint16_t temp_x = ((i2cdat[0x03 + i * 6] & 0x0F) << 8) | (i2cdat[0x04 + i * 6]);
    uint16_t temp_y = ((i2cdat[0x05 + i * 6] & 0x0F) << 8) | (i2cdat[0x06 + i * 6]);
    _advance_touch_list(temp_x, temp_y, 0, (i2cdat[0x05 + i * 6] >> 4));
  }

#ifdef FT6206_DEBUG
  Serial.println();
  for (uint8_t i = 0; i < touches; i++) {
    Serial.print("ID #");
    Serial.print(touchID[i]);
    Serial.print("\t(");
    Serial.print(touchX[i]);
    Serial.print(", ");
    Serial.print(touchY[i]);
    Serial.print(") ");
  }
  Serial.println();
#endif
}

uint8_t FT6206::readRegister8(uint8_t reg) {
  uint8_t x;
  // use i2c
  Wire.beginTransmission(FT62XX_ADDR);
  Wire.write((byte)reg);
  Wire.endTransmission();

  Wire.requestFrom((byte)FT62XX_ADDR, (byte)1);
  x = Wire.read();

#ifdef I2C_DEBUG
  Serial.print("$");
  Serial.print(reg, HEX);
  Serial.print(": 0x");
  Serial.println(x, HEX);
#endif

  return x;
}

void FT6206::writeRegister8(uint8_t reg, uint8_t val) {
  // use i2c
  Wire.beginTransmission(FT62XX_ADDR);
  Wire.write((byte)reg);
  Wire.write((byte)val);
  Wire.endTransmission();
}

/*

// DONT DO THIS - REALLY - IT DOESNT WORK
void FT6206::autoCalibrate(void) {
 writeRegister8(FT06_REG_MODE, FT6206_REG_FACTORYMODE);
 delay(100);
 //Serial.println("Calibrating...");
 writeRegister8(FT6206_REG_CALIBRATE, 4);
 delay(300);
 for (uint8_t i = 0; i < 100; i++) {
   uint8_t temp;
   temp = readRegister8(FT6206_REG_MODE);
   Serial.println(temp, HEX);
   //return to normal mode, calibration finish
   if (0x0 == ((temp & 0x70) >> 4))
     break;
 }
 delay(200);
 //Serial.println("Calibrated");
 delay(300);
 writeRegister8(FT6206_REG_MODE, FT6206_REG_FACTORYMODE);
 delay(100);
 writeRegister8(FT6206_REG_CALIBRATE, 5);
 delay(300);
 writeRegister8(FT6206_REG_MODE, FT6206_REG_WORKMODE);
 delay(300);
}
*/


void FT6206::_advance_touch_list(int16_t x, int16_t y, int16_t z, uint8_t id) {
  Vector3u16 temp0(x, y, z);
  Vector3u16 temp1;
  uint8_t temp_id0 = id;
  uint8_t temp_id1 = 0;
  for (uint8_t i = 0; i < FT62XX_TOUCH_BACKLOG_LENGTH; i++) {
    temp1.set(&touches[i]);
    temp_id1 = touchID[i];
    touches[i].set(&temp0);
    touchID[i] = temp_id0;
    temp0.set(&temp1);
    temp_id0 = temp_id1;
  }
}


void FT6206::printDebug(StringBuilder* output) {
  output->concat("-- FT6206\n");
  for (uint8_t i = 0; i < FT62XX_TOUCH_BACKLOG_LENGTH; i++) {
  }
}



/*******************************************************************************
* ___     _       _                      These members are mandatory overrides
*  |   / / \ o   | \  _     o  _  _      for implementing I/O callbacks. They
* _|_ /  \_/ o   |_/ (/_ \/ | (_ (/_     are also implemented by Adapters.
*******************************************************************************/

/**
* Called prior to the given bus operation beginning.
* Returning 0 will allow the operation to continue.
* Returning anything else will fail the operation with IO_RECALL.
*   Operations failed this way will have their callbacks invoked as normal.
*
* @param  _op  The bus operation that was completed.
* @return 0 to run the op, or non-zero to cancel it.
*/
int8_t FT6206::io_op_callahead(BusOp* _op) {
  return 0;   // Permit the transfer, always.
}

/**
* When a bus operation completes, it is passed back to its issuing class.
*
* @param  _op  The bus operation that was completed.
* @return BUSOP_CALLBACK_NOMINAL on success, or appropriate error code.
*/
int8_t FT6206::io_op_callback(BusOp* _op) {
  int8_t ret = BUSOP_CALLBACK_NOMINAL;
  I2CBusOp* op = (I2CBusOp*) _op;

  if (!op->hasFault()) {
    uint8_t* buf = op->buffer();
    uint8_t  len = op->bufferLen();
    switch (op->get_opcode()) {
      case BusOpcode::TX:
        break;

      case BusOpcode::RX:  // Only RX in driver is a full refresh.
        break;
      default:
        break;
    }
  }
  else {
    ret = BUSOP_CALLBACK_ERROR;
  }
  return ret;
}
