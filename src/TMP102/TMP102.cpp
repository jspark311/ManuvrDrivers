/*
* This file started out as a SparkFun driver. I have mutated it.
*   ---J. Ian Lindsay
*
* BusOp conversion.
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

#define TEMPERATURE_REGISTER        0x00
#define CONFIG_REGISTER             0x01
#define T_LOW_REGISTER              0x02
#define T_HIGH_REGISTER             0x03

#define TMP102_DEG_C_PER_BIT     0.0625f


/*******************************************************************************
*      _______.___________.    ___   .___________. __    ______     _______.
*     /       |           |   /   \  |           ||  |  /      |   /       |
*    |   (----`---|  |----`  /  ^  \ `---|  |----`|  | |  ,----'  |   (----`
*     \   \       |  |      /  /_\  \    |  |     |  | |  |        \   \
* .----)   |      |  |     /  _____  \   |  |     |  | |  `----.----)   |
* |_______/       |__|    /__/     \__\  |__|     |__|  \______|_______/
*
* Static members and initializers should be located here.
*******************************************************************************/

/**
* Static function to convert enum to string.
*/
const char* TMP102::odrStr(const TMP102DataRate e) {
  switch (e) {
    case TMP102DataRate::RATE_0_25_HZ:  return "RATE_0_25_HZ";
    case TMP102DataRate::RATE_1_HZ:     return "RATE_1_HZ";
    case TMP102DataRate::RATE_4_HZ:     return "RATE_4_HZ";
    case TMP102DataRate::RATE_8_HZ:     return "RATE_8_HZ";
    default:                            return "INVALID";
  }
}


/*******************************************************************************
*   ___ _              ___      _ _              _      _
*  / __| |__ _ ______ | _ ) ___(_) |___ _ _ _ __| |__ _| |_ ___
* | (__| / _` (_-<_-< | _ \/ _ \ | / -_) '_| '_ \ / _` |  _/ -_)
*  \___|_\__,_/__/__/ |___/\___/_|_\___|_| | .__/_\__,_|\__\___|
*                                          |_|
* Constructors/destructors, class initialization functions and so-forth...
*******************************************************************************/

TMP102::TMP102(uint8_t addr, uint8_t alrt_pin) : I2CDevice(addr), _ALRT_PIN(alrt_pin) {}

TMP102::~TMP102() {}


int8_t TMP102::init(I2CAdapter* b) {
  int8_t ret = _ll_pin_init();
  _tmp_clear_flag(TMP102_FLAG_DEVICE_PRESENT | TMP102_FLAG_INITIALIZED | TMP102_FLAG_PTR_VALID | TMP102_FLAG_IO_IN_FLIGHT);
  if (nullptr != b) {
    _bus = b;
  }
  if (nullptr != _bus) {
    if (0 == ret) {
      ret = ping_device();
      //if (0 == ret) {
      //  uint16_t conf_value = ((uint8_t) TMP102DataRate::RATE_4_HZ) << 6;  // Conversion rate
      //  conf_value |= (3 << 13);  // R[1..0] = 0b11 mode
      //  //conf_value |= (1 << 4);   // Extended mode
      //  ret = _write_register(CONFIG_REGISTER, conf_value);
      //}
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


/**
* Sends a Bus TX to set the device's internal register pointer. This is the
*   first step of a read operation, since writes can send (and reset) this value
*   in the same operation as the data transfer.
*
* @param reg is the register address
* @return 0 on success
*        -1 if the device isn't yet found
*        -2 on BusOp allocation failure
*        -3 I/O rejection
*/
int8_t TMP102::_open_ptr_register(uint8_t reg) {
  int8_t ret = -1;
  if (devFound()) {
    ret--;
    I2CBusOp* op = _bus->new_op(BusOpcode::TX, this);
    if (nullptr != op) {
      ret--;
      _tmp_clear_flag(TMP102_FLAG_PTR_VALID);
      _ptr_val = reg;
      op->dev_addr = _dev_addr;
      op->sub_addr = -1;
      op->setBuffer(&_ptr_val, 1);
      if (0 == queue_io_job(op)) {
        _tmp_set_flag(TMP102_FLAG_IO_IN_FLIGHT);
        ret = 0;
      }
    }
  }
  return ret;
}


/**
* Get the register's shadow value.
*
* @param reg is the register address
* @return the endian-native representation of the given register.
*/
uint16_t TMP102::_get_shadow_value(uint8_t reg) {
  uint8_t msb = *(((uint8_t*) &_shadows[reg & 0x03]) + 0);
  uint8_t lsb = *(((uint8_t*) &_shadows[reg & 0x03]) + 1);
  return (((uint16_t) msb << 8) | lsb);
}


/**
* Reads the given register from the hardware.
*
* @param reg is the register address
* @return 0 on success
*        -1 if the device isn't yet found
*        -2 if there is already I/O in flight
*        -3 I/O failure
*/
int8_t TMP102::_read_register(uint8_t reg) {
  int8_t ret = -1;
  if (devFound()) {
    ret--;
    if (!_tmp_flag(TMP102_FLAG_IO_IN_FLIGHT)) {
      ret--;
      if ((_ptr_val != reg) | !_tmp_flag(TMP102_FLAG_PTR_VALID)) {
        if (0 == _open_ptr_register(reg)) {
          // The read operation will be picked back up on the callback.
          ret = 0;
        }
      }
      else {
        // The PTR reg is already set as needed.
        I2CBusOp* op = _bus->new_op(BusOpcode::RX, this);
        if (nullptr != op) {
          op->dev_addr = _dev_addr;
          op->sub_addr = -1;
          op->setBuffer((uint8_t*) &_shadows[reg], 2);
          if (0 == queue_io_job(op)) {
            _tmp_set_flag(TMP102_FLAG_IO_IN_FLIGHT);
            ret = 0;
          }
        }
      }
    }
  }
  return ret;
}


/**
* Writes the given register with the given value to the hardware.
*
* @param reg is the register address
* @param val is the 16-bit value to write (in native endianness)
* @return 0 on success
*        -1 if the device isn't yet found
*        -2 if there is already I/O in flight
*        -3 on BusOp allocation failure
*        -4 I/O rejection
*/
int8_t TMP102::_write_register(uint8_t reg, uint16_t val) {
  int8_t ret = -1;
  if (devFound()) {
    ret--;
    if (!_tmp_flag(TMP102_FLAG_IO_IN_FLIGHT)) {
      ret--;
      I2CBusOp* op = _bus->new_op(BusOpcode::TX, this);
      if (nullptr != op) {
        ret--;
        uint16_t _be_value = (val >> 8) | ((val & 0x00FF) << 8);
        _shadows[reg] = _be_value;
        _ptr_val = reg;
        _tmp_clear_flag(TMP102_FLAG_PTR_VALID);
        op->dev_addr = _dev_addr;
        op->sub_addr = reg;
        op->setBuffer((uint8_t*) &_shadows[reg], 2);
        if (0 == queue_io_job(op)) {
          _tmp_set_flag(TMP102_FLAG_IO_IN_FLIGHT);
          ret = 0;
        }
      }
    }
  }
  return ret;
}



int8_t TMP102::_read_temp() {
  // Read from temperature register
  int8_t ret = _read_register(0);
  if (0 == ret) {
    _last_read = millis();
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
      //_bus->beginTransmission(_ADDR);
      //_bus->write(CONFIG_REGISTER);   // Point to configuration register
      //_bus->write(registerByte[0]);  // Write first byte
      //_bus->write(registerByte[1]);  // Write second byte
      //ret = _bus->endTransmission();      // Close communication with TMP102
    }
  }
  return ret;
}


TMP102DataRate TMP102::conversionRate() {
  TMP102DataRate ret = TMP102DataRate::RATE_INVALID;
  if (initialized()) {
    const uint16_t MASK       = 0x00C0;
    const uint16_t MASKED_VAL = _get_shadow_value(CONFIG_REGISTER) & MASK;
    ret = (TMP102DataRate) (MASKED_VAL >> 6);
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
    //_bus->beginTransmission(_ADDR);
    //_bus->write(CONFIG_REGISTER);    // Point to configuration register
    //_bus->write(registerByte[0]);    // Write first byte
    //_bus->write(registerByte[1]);    // Write second byte
    //ret = _bus->endTransmission();   // Close communication with TMP102
  }
  return ret;
}


int8_t TMP102::enabled(bool x) {
  int8_t ret = -1;
  if (devFound()) {
    // Change pointer address to configuration register (0x01)
    if (0 == _open_ptr_register(CONFIG_REGISTER)) {
      // Read current configuration register value and clear or set SD (bit 0 of first byte)
      const uint16_t MASK       = 0x0100;
      const uint16_t CUR_VAL    = _shadows[1];
      const uint16_t MASKED_VAL = CUR_VAL & ~MASK;
      ret = _write_register(1, (x ? MASKED_VAL : (MASKED_VAL | MASK)));

      //uint8_t registerByte = _read_register(0);
      //registerByte = x ? (registerByte &= 0xFE) : (registerByte |= 0x01);
      //_bus->beginTransmission(_ADDR);  // ...and re-write it.
      //_bus->write(CONFIG_REGISTER);    // Point to configuration register.
      //_bus->write(registerByte);       // Write first byte.
      //ret = _bus->endTransmission();   // Close communication with TMP102.
    }
  }
  return ret;
}


bool TMP102::enabled() {
  bool ret = false;
  if (initialized()) {
    const uint16_t MASK       = 0x0100;
    const uint16_t MASKED_VAL = _get_shadow_value(CONFIG_REGISTER) & MASK;
    if (MASKED_VAL != MASK) {   // Bit unset implies enablement.
      ret = true;
    }
  }
  return ret;
}


bool TMP102::extendedMode() {
  bool ret = false;
  if (initialized()) {
    const uint16_t MASK       = 0x0010;
    const uint16_t MASKED_VAL = _get_shadow_value(CONFIG_REGISTER) & MASK;
    if (MASKED_VAL == MASK) {
      ret = true;
    }
  }
  return ret;
}


int8_t TMP102::alertActiveLow(bool active_low) {
  int8_t ret = -1;
  uint8_t registerByte; // Store the data from the register here

  // Change pointer address to configuration register (1)
  if (0 == _open_ptr_register(CONFIG_REGISTER)) {
    // Read current configuration register value
    registerByte = _read_register(0);

    // Load new value for polarity
    registerByte &= 0xFB; // Clear POL (bit 2 of registerByte)
    registerByte |= ((!active_low) << 2);  // Shift in new POL bit

    // Set configuration register
    //_bus->beginTransmission(_ADDR);
    //_bus->write(CONFIG_REGISTER);  // Point to configuration register
    //_bus->write(registerByte);      // Write first byte
    //ret = _bus->endTransmission();       // Close communication with TMP102
  }
  return ret;
}


bool TMP102::alertActiveLow() {
  bool ret = false;
  if (initialized()) {
    const uint16_t MASK       = 0x0400;
    const uint16_t MASKED_VAL = _get_shadow_value(CONFIG_REGISTER) & MASK;
    if (MASKED_VAL != MASK) {   // Bit unset implies active low.
      ret = true;
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
    temperature = temperature / TMP102_DEG_C_PER_BIT;
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
    //_bus->beginTransmission(_ADDR);
    //_bus->write(T_LOW_REGISTER);   // Point to T_LOW
    //_bus->write(registerByte[0]);  // Write first byte
    //_bus->write(registerByte[1]);  // Write second byte
    //ret = _bus->endTransmission();      // Close communication with TMP102
  }
  return ret;
}


int8_t TMP102::setHighTemp(float degrees) {
  int8_t ret = -1;
  if (devFound()) {
    uint8_t registerByte[2];  // Store the data from the register here
    float temperature = _normalize_units_accepted(degrees);

    temperature = temperature / TMP102_DEG_C_PER_BIT;
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
    //_bus->beginTransmission(_ADDR);
    //_bus->write(T_HIGH_REGISTER);      // Point to T_HIGH register
    //_bus->write(registerByte[0]);      // Write first byte
    //_bus->write(registerByte[1]);      // Write second byte
    //ret = _bus->endTransmission();    // Close communication with TMP102
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
  return (digitalTemp * TMP102_DEG_C_PER_BIT);   // Convert value to Celcius.
}


float TMP102::readHighTemp() {
  int16_t digitalTemp = 0;    // Store the digital temperature value here

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
  return (digitalTemp * TMP102_DEG_C_PER_BIT);   // Convert value to Celcius.
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
    //_bus->beginTransmission(_ADDR);
    //_bus->write(CONFIG_REGISTER);   // Point to configuration register
    //_bus->write(registerByte);     // Write byte to register
    //ret = _bus->endTransmission();       // Close communication with TMP102
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
    //_bus->beginTransmission(_ADDR);
    //_bus->write(CONFIG_REGISTER);   // Point to configuration register
    //_bus->write(registerByte);     // Write byte to register
    //ret = _bus->endTransmission();       // Close communication with TMP102
  }
  return ret;
}


/**
* Used to automatically prevent temperature from exceeding hardware bounds.
*/
float TMP102::_normalize_units_accepted(float temperature) {
  if(temperature > 150.0f) {    temperature = 150.0f;   }
  if(temperature < -55.0) {     temperature = -55.0f;   }
  return temperature;
}


/*
* Idempotently setup the low-level pin details.
*/
int8_t TMP102::_ll_pin_init() {
  int8_t ret = 0;
  if (!_tmp_flag(TMP102_FLAG_PINS_CONFIGURED)) {
    if (255 != _ALRT_PIN) {
      pinMode(_ALRT_PIN, GPIOMode::INPUT);
      // TODO: I'm sick of having to dance around ISR limitations that push drivers
      //   toward a singleton pattern. Migrate this abstraction to AbstrctPlatform.
      //attachInterrupt(digitalPinToInterrupt(_ALRT_PIN), tmp102_isr_fxn, FALLING);
    }
    _tmp_set_flag(TMP102_FLAG_PINS_CONFIGURED);
  }
  return ret;
}



/*******************************************************************************
* ___     _       _                      These members are mandatory overrides
*  |   / / \ o   | \  _     o  _  _      for implementing I/O callbacks. They
* _|_ /  \_/ o   |_/ (/_ \/ | (_ (/_     are also implemented by Adapters.
*******************************************************************************/

/* Transfers always permitted. */
int8_t TMP102::io_op_callahead(BusOp* _op) {   return 0;   }


/*
* Register I/O calls back to this function for BOTH devices (MAG/IMU). So we
*   split the function up into two halves in private scope in the superclass.
* Bus operations that call back with errors are ignored.
*/
int8_t TMP102::io_op_callback(BusOp* _op) {
  I2CBusOp* op  = (I2CBusOp*) _op;
  int8_t    ret = BUSOP_CALLBACK_NOMINAL;

  if (!op->hasFault()) {
    uint     len = op->bufferLen();
    switch (op->get_opcode()) {
      case BusOpcode::TX:
        // The part requires that the first byte of any write is the pointer.
        _tmp_set_flag(TMP102_FLAG_PTR_VALID);
        _tmp_clear_flag(TMP102_FLAG_IO_IN_FLIGHT);
        if (len > 1) {
          switch (_ptr_val) {
            case CONFIG_REGISTER:
              _tmp_set_flag(TMP102_FLAG_INITIALIZED);
              break;
            case T_LOW_REGISTER:
            case T_HIGH_REGISTER:
            default:
              break;
          }
        }
        else if (1 == len) {
          // A prelude to a value read from hardware is the only reason to have
          //   a single byte transaction in this driver. Make the _read_register()
          //   call again.
          _read_register(_ptr_val);
        }
        break;

      case BusOpcode::RX:
        _tmp_clear_flag(TMP102_FLAG_IO_IN_FLIGHT);
        if (_tmp_flag(TMP102_FLAG_PTR_VALID)) {
          switch (_ptr_val) {
            case TEMPERATURE_REGISTER:
              if (_tmp_flag(TMP102_FLAG_INITIALIZED)) {
                int16_t digitalTemp = 0;    // Temperature stored in TMP102 register
                uint8_t* buf = op->buffer();

                // Bit 0 of second byte will always be 0 in 12-bit readings and 1 in 13-bit
                if (*(buf + 1) & 0x01) {  // 13 bit mode
                  // Combine bytes to create a signed int
                  digitalTemp = (*(buf + 0) << 5) | (*(buf + 1) >> 3);
                  // Temperature data can be + or -, if it should be negative,
                  // convert 13 bit to 16 bit and use the 2s compliment.
                  if (digitalTemp > 0xFFF) {
                    digitalTemp |= 0xE000;
                  }
                }
                else {  // 12 bit mode
                  // Combine bytes to create a signed int
                  digitalTemp = (*(buf + 0) << 4) | (*(buf + 0) >> 4);
                  // Temperature data can be + or -, if it should be negative,
                  // convert 12 bit to 16 bit and use the 2s compliment.
                  if (digitalTemp > 0x7FF) {
                    digitalTemp |= 0xF000;
                  }
                }
                _temp = digitalTemp * TMP102_DEG_C_PER_BIT;
                ret = 0;
              }
              break;
            case CONFIG_REGISTER:
              _tmp_set_flag(TMP102_FLAG_INITIALIZED);
              break;
            case T_LOW_REGISTER:
            case T_HIGH_REGISTER:
            default:
              break;
          }
        }
        break;

      case BusOpcode::TX_CMD:
        if (!_tmp_flag(TMP102_FLAG_DEVICE_PRESENT)) {
          // With no ID register to check, we construe no failure on bus ping
          //   as the condition "device found".
          _tmp_set_flag(TMP102_FLAG_DEVICE_PRESENT);
          _read_register(CONFIG_REGISTER);
        }
        break;

      default: break;
    }
  }
  return ret;
}


/*
*
*/
void TMP102::printDebug(StringBuilder* output) {
  StringBuilder::styleHeader1(output, "TMP102");
  output->concatf("\tAlert Pin:   %u\n", _ALRT_PIN);
  output->concatf("\tPins setup:  %c\n", _tmp_flag(TMP102_FLAG_PINS_CONFIGURED) ? 'y' : 'n');
  output->concatf("\tDev found:   %c\n", devFound() ? 'y' : 'n');
  if (devFound()) {
    output->concatf("\tInitialized: %c\n", initialized() ? 'y' : 'n');
    output->concatf("\t_reg_ptr:    %u (%svalid)\n", _ptr_val, _tmp_flag(TMP102_FLAG_PTR_VALID) ? "": "in");
    output->concatf("\tIO inflight: %c\n", _tmp_flag(TMP102_FLAG_IO_IN_FLIGHT) ? 'y' : 'n');
    if (initialized()) {
      output->concatf("\tSample rate: %s\n", odrStr(conversionRate()));
      output->concatf("\tEnabled:     %c\n", enabled() ? 'y' : 'n');
      output->concatf("\tExtnd mode:  %c\n", extendedMode() ? 'y' : 'n');
      output->concatf("\tLast read:   %u\n", _last_read);
      output->concatf("\tTemperature: %.2f\n", _temp);
    }
  }
  output->concat("\t");
  for (int i = 0; i < 8; i++) {
    output->concatf("0x%02x  ", *(((uint8_t*) &_shadows[0]) + i));
  }
  output->concat("\n");
}
