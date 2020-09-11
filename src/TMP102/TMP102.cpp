/*
* This file started out as a SparkFun driver. I have mutated it.
*   ---J. Ian Lindsay
*/

/******************************************************************************
SparkFunTMP102.cpp
SparkFunTMP102 Library Source File
Alex Wende @ SparkFun Electronics
Original Creation Date: April 29, 2016
https://github.com/sparkfun/Digital_Temperature_Sensor_Breakout_-_TMP102

This code is beerware; if you see me (or any other SparkFun employee) at the
local, and you've found our code helpful, please buy us a round!

Distributed as-is; no warranty is given.
******************************************************************************/
#include "TMP102.h"
#include <Wire.h>

#define TEMPERATURE_REGISTER 0x00
#define CONFIG_REGISTER 0x01
#define T_LOW_REGISTER 0x02
#define T_HIGH_REGISTER 0x03



TMP102::TMP102(uint8_t addr, uint8_t alrt_pin) : _ADDR(addr), _ALRT_PIN(alrt_pin) {}

TMP102::~TMP102() {}


int8_t TMP102::init(TwoWire* b) {
  int8_t ret = _ll_pin_init();
  if (nullptr != b) {
    _bus = b;
    if (0 == ret) {
      // Here (and only here) we replicate the code for _open_ptr_register and
      //   write a formed config byte. Since this part doesn't have an identity
      //   register, we cope by basing our idea of its presence on success here.
      _bus->beginTransmission(_ADDR);  // Connect to TMP102
      _bus->write(CONFIG_REGISTER);    // Open config register
      ret = _bus->endTransmission();   // Did someone answer?
      if (0 == ret) {
        _tmp_set_flag(TMP102_FLAG_DEVICE_PRESENT);
        uint8_t registerByte[2] = {0, 0};   // Default (bit-cleared) state is enabled.
        registerByte[1] |= ((uint8_t) TMP102DataRate::RATE_4_HZ) << 6;  // Conversion rate
        registerByte[1] |= 1 << 4;  // Exentended mode
        _bus->beginTransmission(_ADDR);  // Set configuration registers
        _bus->write(CONFIG_REGISTER);    // Point to configuration register
        _bus->write(registerByte[0]);    // Write first byte
        _bus->write(registerByte[1]);    // Write second byte
        ret = _bus->endTransmission();   // Close communication with TMP102
        if (0 == ret) {
          _tmp_set_flag(TMP102_FLAG_EXTENDED_MODE | TMP102_FLAG_ENABLED | TMP102_FLAG_INITIALIZED);
        }
      }
      else {
        _tmp_clear_flag(TMP102_FLAG_DEVICE_PRESENT | TMP102_FLAG_EXTENDED_MODE | TMP102_FLAG_ENABLED | TMP102_FLAG_INITIALIZED);
      }
    }
  }
  return ret;
}


/*
* Poll the class for updates.
* Returns...
*   -3 if not initialized and enabled.
*   -1 if data needed to be read, but doing so failed.
*   0  if nothing needs doing.
*   1  if data was read and is fresh.
*   2  An alert is pending.
*/
int8_t TMP102::poll() {
  int8_t ret = -3;
  if (initialized() && enabled()) {
    ret = 0;
    if (dataReady()) {
      if (0 == _read_temp()) {
        ret = 1;
      }
    }

    if (255 != _ALRT_PIN) {
      // TODO: Read pin.
    }
    else {
      // TODO: Read status register.
    }
  }
  return ret;
}


int8_t TMP102::_open_ptr_register(uint8_t pointerReg) {
  int8_t ret = -1;
  if (devFound()) {
    _bus->beginTransmission(_ADDR); // Connect to TMP102
    _bus->write(pointerReg); // Open specified register
    ret = _bus->endTransmission(); // Close communication with TMP102
  }
  return ret;
}


uint8_t TMP102::_read_register(bool registerNumber){
  uint8_t registerByte[2] = {0, 0};
  // Read current configuration register value
  _bus->requestFrom((uint8_t) _ADDR, (uint8_t) 2);   // Read two bytes from TMP102
  if (_bus->available()) {
    registerByte[0] = (_bus->read());  // Read first byte
    registerByte[1] = (_bus->read());  // Read second byte
  }
  return registerByte[registerNumber];
}


int8_t TMP102::_read_temp() {
  uint8_t registerByte[2] = {0, 0};
  int8_t ret = -1;
  int16_t digitalTemp = 0;    // Temperature stored in TMP102 register
  // Read Temperature
  // Change pointer address to temperature register (0)
  if (0 == _open_ptr_register(TEMPERATURE_REGISTER)) {
    // Read from temperature register
    registerByte[0] = _read_register(0);
    registerByte[1] = _read_register(1);
    _last_read = millis();

    // Bit 0 of second byte will always be 0 in 12-bit readings and 1 in 13-bit
    if(registerByte[1]&0x01) {  // 13 bit mode
      // Combine bytes to create a signed int
      digitalTemp = ((registerByte[0]) << 5) | (registerByte[1] >> 3);
      // Temperature data can be + or -, if it should be negative,
      // convert 13 bit to 16 bit and use the 2s compliment.
      if(digitalTemp > 0xFFF) {
        digitalTemp |= 0xE000;
      }
    }
    else {  // 12 bit mode
      // Combine bytes to create a signed int
      digitalTemp = ((registerByte[0]) << 4) | (registerByte[1] >> 4);
      // Temperature data can be + or -, if it should be negative,
      // convert 12 bit to 16 bit and use the 2s compliment.
      if(digitalTemp > 0x7FF) {
        digitalTemp |= 0xF000;
      }
    }
    ret = 0;
    // Convert digital reading to analog temperature (1-bit is equal to 0.0625 C)
    _temp = digitalTemp * 0.0625;
  }
  return ret;
}


uint16_t TMP102::_data_period_ms() {
  const uint8_t SCALARS[4] = {1, 4, 16, 32};
  return (((uint16_t) SCALARS[(_flags >> 6) & 3]) * 250);
}


bool TMP102::dataReady() {
  return (millis() >= (_last_read + _data_period_ms()));
}


int8_t TMP102::conversionRate(TMP102DataRate r) {
  int8_t ret = -1;
  uint8_t rate = (uint8_t) r; // Make sure rate is not set higher than 3.
  uint8_t registerByte[2]; // Store the data from the register here
  if (rate < 4) {
    // Change pointer address to configuration register (0x01)
    if (0 == _open_ptr_register(CONFIG_REGISTER)) {
      // Read current configuration register value
      registerByte[0] = _read_register(0);
      registerByte[1] = _read_register(1);

      // Load new conversion rate
      registerByte[1] &= 0x3F;  // Clear CR0/1 (bit 6 and 7 of second byte)
      registerByte[1] |= rate << 6;  // Shift in new conversion rate

      // Set configuration registers
      _bus->beginTransmission(_ADDR);
      _bus->write(CONFIG_REGISTER);   // Point to configuration register
      _bus->write(registerByte[0]);  // Write first byte
      _bus->write(registerByte[1]);  // Write second byte
      ret = _bus->endTransmission();      // Close communication with TMP102
      if (0 == ret) {
        _tmp_clear_flag(TMP102_FLAG_DATA_RATE_MASK);
        _tmp_set_flag((uint16_t)(rate << 6));
      }
    }
  }
  return ret;
}


int8_t TMP102::extendedMode(bool mode) {
  int8_t ret = -1;
  uint8_t registerByte[2]; // Store the data from the register here

  // Change pointer address to configuration register (0x01)
  if (0 == _open_ptr_register(CONFIG_REGISTER)) {
    // Read current configuration register value
    registerByte[0] = _read_register(0);
    registerByte[1] = _read_register(1);

    // Load new value for extention mode
    registerByte[1] &= 0xEF;    // Clear EM (bit 4 of second byte)
    registerByte[1] |= mode<<4;  // Shift in new exentended mode bit

    // Set configuration registers
    _bus->beginTransmission(_ADDR);
    _bus->write(CONFIG_REGISTER);    // Point to configuration register
    _bus->write(registerByte[0]);    // Write first byte
    _bus->write(registerByte[1]);    // Write second byte
    ret = _bus->endTransmission();   // Close communication with TMP102
    if (0 == ret) {
      _tmp_set_flag(TMP102_FLAG_EXTENDED_MODE, mode);
    }
  }
  return ret;
}


int8_t TMP102::enabled(bool x) {
  int8_t ret = -1;
  if (devFound()) {
    // Change pointer address to configuration register (0x01)
    if (0 == _open_ptr_register(CONFIG_REGISTER)) {
      // Read current configuration register value and clear or set SD (bit 0 of first byte)
      uint8_t registerByte = _read_register(0);
      registerByte = x ? (registerByte &= 0xFE) : (registerByte |= 0x01);

      _bus->beginTransmission(_ADDR);  // ...and re-write it.
      _bus->write(CONFIG_REGISTER);    // Point to configuration register.
      _bus->write(registerByte);       // Write first byte.
      ret = _bus->endTransmission();   // Close communication with TMP102.
      if (0 == ret) {       // Only change the flag if the write worked.
        _tmp_set_flag(TMP102_FLAG_ENABLED, x);
      }
    }
  }
  return ret;
}



int8_t TMP102::alertPolarity(bool polarity) {
  int8_t ret = -1;
  uint8_t registerByte; // Store the data from the register here

  // Change pointer address to configuration register (1)
  if (0 == _open_ptr_register(CONFIG_REGISTER)) {
    // Read current configuration register value
    registerByte = _read_register(0);

    // Load new value for polarity
    registerByte &= 0xFB; // Clear POL (bit 2 of registerByte)
    registerByte |= polarity<<2;  // Shift in new POL bit

    // Set configuration register
    _bus->beginTransmission(_ADDR);
    _bus->write(CONFIG_REGISTER);  // Point to configuration register
    _bus->write(registerByte);      // Write first byte
    ret = _bus->endTransmission();       // Close communication with TMP102
    if (0 == ret) {
      _tmp_set_flag(TMP102_FLAG_ALRT_ACTIVE_HIGH, polarity);
    }
  }
  return ret;
}


bool TMP102::alert() {
  // TODO: Read the alert pin if possible.
  uint8_t registerByte = 0; // Store the data from the register here
  // Change pointer address to configuration register (1)
  if (0 == _open_ptr_register(CONFIG_REGISTER)) {
    registerByte = _read_register(1);   // Read current configuration register value
    registerByte &= 0x20;  // Clear everything but the alert bit (bit 5)
  }
  return registerByte>>5;
}


int8_t TMP102::setLowTemp(float degrees) {
  int8_t ret = -1;
  if (devFound()) {
    float temperature = _normalize_units_accepted(degrees);
    uint8_t registerByte[2];  // Store the data from the register here
    // Convert analog temperature to digital value
    temperature = temperature / 0.0625;
    // Split temperature into separate bytes
    if (extendedMode()) {  // 13-bit mode
      registerByte[0] = int(temperature)>>5;
      registerByte[1] = (int(temperature)<<3);
    }
    else {  // 12-bit mode
      registerByte[0] = int(temperature)>>4;
      registerByte[1] = int(temperature)<<4;
    }

    // Write to T_LOW Register
    _bus->beginTransmission(_ADDR);
    _bus->write(T_LOW_REGISTER);   // Point to T_LOW
    _bus->write(registerByte[0]);  // Write first byte
    _bus->write(registerByte[1]);  // Write second byte
    ret = _bus->endTransmission();      // Close communication with TMP102
  }
  return ret;
}


int8_t TMP102::setHighTemp(float degrees) {
  int8_t ret = -1;
  if (devFound()) {
    uint8_t registerByte[2];  // Store the data from the register here
    float temperature = _normalize_units_accepted(degrees);

    temperature = temperature / 0.0625;
    // Split temperature into separate bytes
    if(extendedMode()) {  // 13-bit mode
      registerByte[0] = int(temperature)>>5;
      registerByte[1] = (int(temperature)<<3);
    }
    else {  // 12-bit mode
      registerByte[0] = int(temperature)>>4;
      registerByte[1] = int(temperature)<<4;
    }

    // Write to T_HIGH Register
    _bus->beginTransmission(_ADDR);
    _bus->write(T_HIGH_REGISTER);      // Point to T_HIGH register
    _bus->write(registerByte[0]);      // Write first byte
    _bus->write(registerByte[1]);      // Write second byte
    ret = _bus->endTransmission();    // Close communication with TMP102
  }
  return ret;
}


float TMP102::readLowTemp() {
  int16_t digitalTemp = 0;    // Store the digital temperature value here

  if (0 == _open_ptr_register(T_LOW_REGISTER)) {
    uint8_t registerByte[2];  // Store the data from the register here
    registerByte[0] = _read_register(0);
    registerByte[1] = _read_register(1);

    if (extendedMode()) {  // 13 bit mode
      // Combine bytes to create a signed int
      digitalTemp = ((registerByte[0]) << 5) | (registerByte[1] >> 3);
      // Temperature data can be + or -, if it should be negative,
      // convert 13 bit to 16 bit and use the 2s compliment.
      if(digitalTemp > 0xFFF) {
        digitalTemp |= 0xE000;
      }
    }
    else { // 12 bit mode
      // Combine bytes to create a signed int
      digitalTemp = ((registerByte[0]) << 4) | (registerByte[1] >> 4);
      // Temperature data can be + or -, if it should be negative,
      // convert 12 bit to 16 bit and use the 2s compliment.
      if(digitalTemp > 0x7FF) {
        digitalTemp |= 0xF000;
      }
    }
  }
  // Convert digital reading to analog temperature (1-bit is equal to 0.0625 C)
  return _normalize_units_returned(digitalTemp * 0.0625);
}


float TMP102::readHighTemp() {
  int16_t digitalTemp;    // Store the digital temperature value here

  if (0 == _open_ptr_register(T_HIGH_REGISTER)) {
    uint8_t registerByte[2];  // Store the data from the register here
    registerByte[0] = _read_register(0);
    registerByte[1] = _read_register(1);

    if (extendedMode()) { // 13 bit mode
      // Combine bytes to create a signed int
      digitalTemp = ((registerByte[0]) << 5) | (registerByte[1] >> 3);
      // Temperature data can be + or -, if it should be negative,
      // convert 13 bit to 16 bit and use the 2s compliment.
      if (digitalTemp > 0xFFF) {
        digitalTemp |= 0xE000;
      }
    }
    else {  // 12 bit mode
      // Combine bytes to create a signed int
      digitalTemp = ((registerByte[0]) << 4) | (registerByte[1] >> 4);
      // Temperature data can be + or -, if it should be negative,
      // convert 12 bit to 16 bit and use the 2s compliment.
      if(digitalTemp > 0x7FF) {
        digitalTemp |= 0xF000;
      }
    }
  }
  // Convert digital reading to analog temperature (1-bit is equal to 0.0625 C)
  return _normalize_units_returned(digitalTemp * 0.0625);
}


int8_t TMP102::setFault(uint8_t faultSetting) {
  int8_t ret = -1;
  uint8_t registerByte; // Store the data from the register here
  faultSetting = faultSetting&3; // Make sure rate is not set higher than 3.
  // Change pointer address to configuration register (0x01)
  if (0 == _open_ptr_register(CONFIG_REGISTER)) {
    // Read current configuration register value
    registerByte = _read_register(0);

    // Load new conversion rate
    registerByte &= 0xE7;  // Clear F0/1 (bit 3 and 4 of first byte)
    registerByte |= faultSetting<<3;  // Shift new fault setting

    // Set configuration registers
    _bus->beginTransmission(_ADDR);
    _bus->write(CONFIG_REGISTER);   // Point to configuration register
    _bus->write(registerByte);     // Write byte to register
    ret = _bus->endTransmission();       // Close communication with TMP102
  }
  return ret;
}


int8_t TMP102::setAlertMode(bool mode) {
  int8_t ret = -1;
  // Change pointer address to configuration register (1)
  if (0 == _open_ptr_register(CONFIG_REGISTER)) {
    uint8_t registerByte; // Store the data from the register here
    // Read current configuration register value
    registerByte = _read_register(0);

    // Load new conversion rate
    registerByte &= 0xFD;  // Clear old TM bit (bit 1 of first byte)
    registerByte |= mode<<1;  // Shift in new TM bit

    // Set configuration registers
    _bus->beginTransmission(_ADDR);
    _bus->write(CONFIG_REGISTER);   // Point to configuration register
    _bus->write(registerByte);     // Write byte to register
    ret = _bus->endTransmission();       // Close communication with TMP102
  }
  return ret;
}


/**
* Used to automatically convert from Fahrenheit if that is how the class is
*   configured to operate.
*/
float TMP102::_normalize_units_accepted(float temperature) {
  if (unitsFahrenheit()) {
    temperature = (temperature - 32) / 1.8;
  }
  // Prevent temperature from exceeding hardware bounds.
  if(temperature > 150.0f) {    temperature = 150.0f;   }
  if(temperature < -55.0) {     temperature = -55.0f;   }
  return temperature;
}


/**
* Used to automatically convert to Fahrenheit if that is how the class is
*   configured to operate.
*/
float TMP102::_normalize_units_returned(float temperature) {
  if (unitsFahrenheit()) {
    temperature = temperature * 1.8 + 32;
  }
  return temperature;
}


/*
* Idempotently setup the low-level pin details.
*/
int8_t TMP102::_ll_pin_init() {
  int8_t ret = 0;
  if (!_tmp_flag(TMP102_FLAG_PINS_CONFIGURED)) {
    if (255 != _ALRT_PIN) {
      pinMode(_ALRT_PIN, INPUT);
      //attachInterrupt(digitalPinToInterrupt(_ALRT_PIN), tmp102_isr_fxn, FALLING);
    }
    _tmp_set_flag(TMP102_FLAG_PINS_CONFIGURED);
  }
  return ret;
}
