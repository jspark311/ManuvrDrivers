#include "AS7265X.h"


AS7265X::AS7265X(const uint8_t P, const uint8_t ADDR) _INT_PIN(P) {
}


void AS7265X::init(uint8_t gain, uint8_t mode, uint8_t intTime) {
  i2cm_AS72xx_write(AS72651_DEV_SEL, 0);
  i2cm_AS72xx_write(AS72651_LED_CONFIG, 0x00 );  // turn off led indication on device 1

  // enable interrupt (bit 6), set gain and mode
  i2cm_AS72xx_write(AS72651_CONTROL_SETUP, 0x40 | gain << 4 | mode << 2 );

  // set integration time
  i2cm_AS72xx_write(AS72651_INT_TIME, intTime );
}


uint8_t AS7265X::getStatus() {
  uint8_t c = i2cm_AS72xx_read(AS72651_CONTROL_SETUP);
  return c;
}


void AS7265X::readRawData(int16_t * destination) {
  uint8_t rawData[2];

  // collect R,S,T,U,V, W data
  i2cm_AS72xx_write(AS72651_DEV_SEL, 0);
  for (int i = 0; i < 6; i++) {
    for(int j = 0; j < 2; j++) {
      rawData[j] = i2cm_AS72xx_read(AS72651_RAW_VALUE_0_H + 2*i + j);
    }
    destination[i] = (int16_t) ( ((int16_t) rawData[0] << 8) | rawData[1]);
  }

  // collect J,I,G,H,K, L data
  i2cm_AS72xx_write(AS72651_DEV_SEL, 1);
  for (int i = 0; i < 6; i++) {
    for(int j = 0; j < 2; j++) {
      rawData[j] = i2cm_AS72xx_read(AS72651_RAW_VALUE_0_H + 2*i + j);
    }

    destination[i + 6] = (int16_t) ( ((int16_t) rawData[0] << 8) | rawData[1]);
  }

  //collect D,C,A,B,E, F data
  i2cm_AS72xx_write(AS72651_DEV_SEL, 2);
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 4; j++) {
      rawData[j] = i2cm_AS72xx_read(AS72651_RAW_VALUE_0_H + 2*i + j);
    }
    destination[i + 12] = (int16_t) ( ((int16_t) rawData[0] << 8) | rawData[1]);
  }
}


void AS7265X::readCalData(float * destination) {
  uint8_t rawData[4];

  // collect R,S,T,U,V, W data
  i2cm_AS72xx_write(AS72651_DEV_SEL, 0);
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 4; j++) {
      rawData[j] = i2cm_AS72xx_read(AS72651_CAL_CHAN0_0 + 4*i + j);
    }

    uint32_t x = ((uint32_t) rawData[0] << 24) | ((uint32_t) rawData[1] << 16) | ((uint32_t) rawData[2] << 8) | rawData[3];
    destination[i] = *(float*)&x;
  }

  // collect J,I,G,H,K, L data
  i2cm_AS72xx_write(AS72651_DEV_SEL, 1);
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 4; j++) {
      rawData[j] = i2cm_AS72xx_read(AS72651_CAL_CHAN0_0 + 4*i + j);
    }
    uint32_t x = ((uint32_t) rawData[0] << 24) | ((uint32_t) rawData[1] << 16) | ((uint32_t) rawData[2] << 8) | rawData[3];
    destination[i + 6] = *(float*)&x;
  }

  //collect D,C,A,B,E, F data
  i2cm_AS72xx_write(AS72651_DEV_SEL, 2);
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 4; j++) {
      rawData[j] = i2cm_AS72xx_read(AS72651_CAL_CHAN0_0 + 4*i + j);
    }
    uint32_t x = ((uint32_t) rawData[0] << 24) | ((uint32_t) rawData[1] << 16) | ((uint32_t) rawData[2] << 8) | rawData[3];
    destination[i + 12] = *(float*)&x;
  }
}


uint8_t AS7265X::getDevType() {
  uint8_t c = i2cm_AS72xx_read(AS72651_DEVICE_TYPE);
  return c;
}

uint8_t AS7265X::getHWVersion() {
  uint8_t c = i2cm_AS72xx_read(AS72651_HW_VERSION);
  return c;
}


uint16_t AS7265X::getFWMajorVersion() {
  uint8_t rawData[2] = {0, 0};
  i2cm_AS72xx_write(AS72651_FW_VERSION_H, 0x01);
  rawData[1] = i2cm_AS72xx_read(AS72651_FW_VERSION_H);
  i2cm_AS72xx_write(AS72651_FW_VERSION_L, 0x01);
  rawData[0] = i2cm_AS72xx_read(AS72651_FW_VERSION_L);
  uint16_t c = (uint16_t)( ( (uint16_t) rawData[1] << 8) | rawData[0]);
  return c;
}


uint16_t AS7265X::getFWPatchVersion() {
  uint8_t rawData[2] = {0, 0};
  i2cm_AS72xx_write(AS72651_FW_VERSION_H, 0x02);
  rawData[1] = i2cm_AS72xx_read(AS72651_FW_VERSION_H);
  i2cm_AS72xx_write(AS72651_FW_VERSION_L, 0x02);
  rawData[0] = i2cm_AS72xx_read(AS72651_FW_VERSION_L);
  uint16_t c = (uint16_t)( ( (uint16_t) rawData[1] << 8) | rawData[0]);
  return c;
}


uint16_t AS7265X::getFWBuildVersion() {
  uint8_t rawData[2] = {0, 0};
  i2cm_AS72xx_write(AS72651_FW_VERSION_H, 0x03);
  rawData[1] = i2cm_AS72xx_read(AS72651_FW_VERSION_H);
  i2cm_AS72xx_write(AS72651_FW_VERSION_L, 0x03);
  rawData[0] = i2cm_AS72xx_read(AS72651_FW_VERSION_L);
  uint16_t c = (uint16_t)( ( (uint16_t) rawData[1] << 8) | rawData[0]);
  return c;
}


float AS7265X::getTemperature(uint8_t devNum) {
  i2cm_AS72xx_write(AS72651_DEV_SEL, devNum);
  float c = i2cm_AS72xx_read(AS72651_DEV_TEMP);
  return c;
}


void AS7265X::configureLed(uint8_t ledIndCurrent, uint8_t ledDrvCurrent, uint8_t devNum) {
  i2cm_AS72xx_write(AS72651_DEV_SEL, devNum);
  i2cm_AS72xx_write(AS72651_LED_CONFIG, ledDrvCurrent << 4 | ledIndCurrent < 1);
}


void AS7265X::enableIndLed(uint8_t devNum) {
  i2cm_AS72xx_write(AS72651_DEV_SEL, devNum);
  uint8_t c = i2cm_AS72xx_read(AS72651_LED_CONFIG);
  i2cm_AS72xx_write(AS72651_LED_CONFIG, c | 0x01 );
}


void AS7265X::disableIndLed(uint8_t devNum) {
  i2cm_AS72xx_write(AS72651_DEV_SEL, devNum);
  uint8_t c = i2cm_AS72xx_read(AS72651_LED_CONFIG);
  i2cm_AS72xx_write(AS72651_LED_CONFIG, c &  ~(0x01) );
}


void AS7265X::enableDrvLed(uint8_t devNum) {
  i2cm_AS72xx_write(AS72651_DEV_SEL, devNum);
  uint8_t c = i2cm_AS72xx_read(AS72651_LED_CONFIG);
  i2cm_AS72xx_write(AS72651_LED_CONFIG, c | 0x08 );
}


void AS7265X::disableDrvLed(uint8_t devNum) {
  i2cm_AS72xx_write(AS72651_DEV_SEL, devNum);
  uint8_t c = i2cm_AS72xx_read(AS72651_LED_CONFIG);
  i2cm_AS72xx_write(AS72651_LED_CONFIG, c & ~(0x08) );
}


void AS7265X::i2cm_AS72xx_write(uint8_t virtualReg, uint8_t d) {
  volatile uint8_t status;
  while (1) {
    // Read slave I2C status to see if we can write the reg address.
    status = readByte(AS72651_ADDRESS, I2C_AS72XX_SLAVE_STATUS_REG);
    if ((status & I2C_AS72XX_SLAVE_TX_VALID) == 0)
    // No inbound TX pending at slave. Okay to write now.
    break ;
  }

  // Send the virtual register address
  // (setting bit 7 to indicate a pending write).
  writeByte(AS72651_ADDRESS, I2C_AS72XX_SLAVE_WRITE_REG, (virtualReg | 0x80)) ;
  while (1) {
    // Read the slave I2C status to see if we can write the data byte.
    status = readByte(AS72651_ADDRESS, I2C_AS72XX_SLAVE_STATUS_REG) ;
    if ((status & I2C_AS72XX_SLAVE_TX_VALID) == 0)
    // No inbound TX pending at slave. Okay to write data now.
    break ;
  }

  // Send the data to complete the operation.
  writeByte(AS72651_ADDRESS, I2C_AS72XX_SLAVE_WRITE_REG, d) ;
}


uint8_t AS7265X::i2cm_AS72xx_read(uint8_t virtualReg) {
  volatile uint8_t status, d ;
  while (1) {
    // Read slave I2C status to see if we can write the reg address.
    status = readByte(AS72651_ADDRESS, I2C_AS72XX_SLAVE_STATUS_REG) ;
    if ((status & I2C_AS72XX_SLAVE_TX_VALID) == 0)
    // No inbound TX pending at slave. Okay to write now.
    break;
  }

  // Send the virtual register address
  // (setting bit 7 to indicate a pending write).
  writeByte(AS72651_ADDRESS, I2C_AS72XX_SLAVE_WRITE_REG, virtualReg) ;
  while (1) {
    // Read the slave I2C status to see if our read data is available.
    status = readByte(AS72651_ADDRESS, I2C_AS72XX_SLAVE_STATUS_REG) ;
    if ((status & I2C_AS72XX_SLAVE_RX_VALID) != 0)
    // Read data is ready for us.
    break;
  }
  // Read the data to complete the operation.
  d = readByte(AS72651_ADDRESS, I2C_AS72XX_SLAVE_READ_REG) ;
  return d;
}



// I2C read/write functions for the AS72651 sensor
void AS7265X::writeByte(uint8_t address, uint8_t subAddress, uint8_t data) {
  Wire.beginTransmission(address);  // Initialize the Tx buffer
  Wire.write(subAddress);           // Put slave register address in Tx buffer
  Wire.write(data);                 // Put data in Tx buffer
  Wire.endTransmission();           // Send the Tx buffer
}

uint8_t AS7265X::readByte(uint8_t address, uint8_t subAddress) {
  uint8_t data = 0;                        // `data` will store the register data
  Wire.beginTransmission(address);         // Initialize the Tx buffer
  Wire.write(subAddress);                  // Put slave register address in Tx buffer
  Wire.endTransmission(false);             // Send the Tx buffer, but send a restart to keep connection alive
  Wire.requestFrom(address, 1);            // Read one byte from slave register address
  data = Wire.read();                      // Fill Rx buffer with result
  return data;                             // Return data read from slave register
}


void AS7265X::printDebug(StringBuilder* output) {
  output->concat("-- AS7265X\n");
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
int8_t AS7265X::io_op_callahead(BusOp* _op) {
  return 0;   // Permit the transfer, always.
}

/**
* When a bus operation completes, it is passed back to its issuing class.
*
* @param  _op  The bus operation that was completed.
* @return BUSOP_CALLBACK_NOMINAL on success, or appropriate error code.
*/
int8_t AS7265X::io_op_callback(BusOp* _op) {
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
