#include "LSM6DSOX.h"

// Register bit masks
#define ACC_SAMPLE_RANGE_MASK     0x0C
#define GYRO_SAMPLE_RANGE_MASK    0x0E
#define GYRO_FS_125_SCALE_MASK    0x02

#define ACC_NEW_DATA_MASK         0x01
#define GYRO_NEW_DATA_MASK        0x02
#define TEMP_NEW_DATA_MASK        0x04

//Other Definitions
#define ACC_BITS_PER_SAMPLE       16
#define GYRO_BITS_PER_SAMPLE      16
#define TEMP_BITS_PER_SAMPLE      16


/*******************************************************************************
* Statics and externs
*******************************************************************************/

/* The offsets for each register. */
static const uint8_t LSM6DSOX_REGISTER_ADDRESSES[78] = {
  0x01, 0x02, 0x04, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E,
  0x0F, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1A,
  0x1B, 0x1C, 0x1D, 0x1E, 0x20, 0x22, 0x24, 0x26, 0x28, 0x2A, 0x2C, 0x35,
  0x36, 0x37, 0x38, 0x39, 0x3A, 0x3B, 0x40, 0x41, 0x42, 0x43, 0x49, 0x4A,
  0x4C, 0x4E, 0x50, 0x52, 0x54, 0x56, 0x57, 0x58, 0x59, 0x5A, 0x5B, 0x5C,
  0x5D, 0x5E, 0x5F, 0x60, 0x61, 0x62, 0x63, 0x6F, 0x70, 0x71, 0x72, 0x73,
  0x74, 0x75, 0x78, 0x79, 0x7B, 0x7D
};

/* The offsets for each register shadow. */
static const uint8_t LSM6DSOX_SHADOW_OFFSETS[78] = {
  0,  1,  2,  4,  5,  6,  7,  8,  9,  10, 11, 12,
  13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24,
  25, 26, 27, 28, 29, 31, 33, 35, 37, 39, 41, 43,
  44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55,
  57, 59, 61, 63, 65, 67, 68, 69, 70, 71, 72, 73,
  74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85,
  86, 87, 88, 89, 91, 93
};

/* The widths of the registers named in the enum class LSM6DSOXRegister. */
static const uint8_t LSM6DSOX_REG_WIDTH_MAP[78] = {
  1, 1, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
  1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2,
  2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2,
  2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
  1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2
};

/* Is a given LSM6DSOXRegister writable? */
static const bool LSM6DSOX_REG_WRITABLE_MAP[78] = {
  true,  true,  true,  true,  true,  true,  true,  true,  true,  true,
  true,  true,  false, true,  true,  true,  true,  true,  true,  true,
  true,  true,  true,  false, false, false, false, false, false, false,
  false, false, false, false, false, false, false, false, false, false,
  false, false, false, false, false, false, false, false, false, false,
  false, false, false, true,  true,  true,  true,  true,  true,  true,
  true,  true,  true,  true,  true,  true, false,  true,  true,  true,
  true,  true,  true,  true,  false, false, false, false
};


/*
* Given an indexed register definition, return a real register address.
*/
static const uint8_t _get_reg_addr(LSM6DSOXRegister r) {
  return (LSM6DSOX_REGISTER_ADDRESSES[(const uint8_t) r]);
}

/*
* Given an indexed register definition, return offset for location in
*   shadow register
*/
static const uint8_t _get_shadow_offset(const LSM6DSOXRegister id) {
  return LSM6DSOX_SHADOW_OFFSETS[(const uint8_t) id];
}

/*
* Given an indexed register definition, return register width (bytes)
*/
static const uint8_t _reg_width(const LSM6DSOXRegister id) {
  return LSM6DSOX_REG_WIDTH_MAP[(const uint8_t) id];
}

/*
* Return whether a register is writable or not
*/
static const bool _reg_writable(const LSM6DSOXRegister id) {
  return LSM6DSOX_REG_WRITABLE_MAP[(const uint8_t) id];
}

/*
* Given a real register address, return an indexed register definition.
*/
static LSM6DSOXRegister _get_reg_enum(uint8_t a) {
  const uint8_t TEST = a & 0x7F; // Cull the top bit, which has other meaning.
  for (uint8_t i = 0; i < sizeof(LSM6DSOX_REGISTER_ADDRESSES); i++) {
    if (LSM6DSOX_REGISTER_ADDRESSES[i] == TEST) {
      return (LSM6DSOXRegister) i;
    }
  }
  return LSM6DSOXRegister::INVALID;
}

/*
* Given an indexed register definition and a length, return true if the
*   corresponding address range is continuous.
*/
static bool _reg_range_continuous(LSM6DSOXRegister r, uint8_t len) {
  const uint8_t r_int = (uint8_t) r;
  if ((r_int + len) < sizeof(LSM6DSOX_REGISTER_ADDRESSES)) {  // Valid length?
    if (len > 1) {
      uint8_t r_last = LSM6DSOX_REGISTER_ADDRESSES[r_int];
      int i_offset = LSM6DSOX_REG_WIDTH_MAP[r_int] - 1;
      for (uint8_t i = 1; (i + i_offset) < len; i++) {
        uint8_t r_this = r_int + i;
        uint8_t reg_width = LSM6DSOX_REG_WIDTH_MAP[r_this];
        if (reg_width == 1) {
          if (LSM6DSOX_REGISTER_ADDRESSES[r_this] != (r_last + 1)) {  // Check for discontinuity.
            return false;
          }
        }
        i_offset += (reg_width - 1);
        r_last = LSM6DSOX_REGISTER_ADDRESSES[r_this] + (reg_width - 1);
      }
    }
  }
  return (len != 0);
}

/*
* Given an indexed register definition and a length, return true if the
*   corresponding address range is writable. Also checks continuity.
*/
static bool _reg_range_writable(LSM6DSOXRegister r, uint8_t len) {
  if (_reg_range_continuous(r, len)) {  // Valid length, and continuous addresses?
    for (uint8_t i = (uint8_t) r; i < ((uint8_t) r) + len; i++) {
      if (!LSM6DSOX_REG_WRITABLE_MAP[i]) {  // Check for readonly.
        return false;
      }
    }
    return true;
  }
  return false;
}


/*******************************************************************************
*   ___ _              ___      _ _              _      _
*  / __| |__ _ ______ | _ ) ___(_) |___ _ _ _ __| |__ _| |_ ___
* | (__| / _` (_-<_-< | _ \/ _ \ | / -_) '_| '_ \ / _` |  _/ -_)
*  \___|_\__,_/__/__/ |___/\___/_|_\___|_| | .__/_\__,_|\__\___|
*                                          |_|
* Constructors/destructors, class initialization functions and so-forth...
*******************************************************************************/

/*
* Constructor.
*/
LSM6DSOX::LSM6DSOX(const uint8_t cs_pin, const uint8_t int1_pin, const uint8_t int2_pin)
  : _CS_PIN(cs_pin), _INT1_PIN(int1_pin), _INT2_PIN(int2_pin),
    _imu_data_refresh(BusOpcode::RX, this, cs_pin, false),
    _fifo_lev_refresh(BusOpcode::RX, this, cs_pin, false) {}


/*
* Destructor
*/
LSM6DSOX::~LSM6DSOX() {}


/*
* Debug
*/
void LSM6DSOX::printDebug(StringBuilder* output) {
  uint32_t now = millis();
  StringBuilder::styleHeader1(output, "LSM6DSOX General");
  output->concatf("\tPins conf'd:      %c\n", _class_flag(LSM6DSOX_FLAG_PINS_CONFIGURED) ? 'y':'n');
  output->concatf("\tFound:            %c\n", devFound() ? 'y':'n');
  output->concatf("\tInitialized:      %c\n", initialized() ? 'y':'n');
  if (initialized()) {
    output->concatf("\tEnabled:          %c\n", enabled() ? 'y':'n');
    output->concatf("\tHigh Performance: %c\n", accLowPowerMode() ? 'y':'n');
    output->concatf("\tCalibrated:       %c\n", calibrated() ? 'y':'n');
    output->concatf("\tFIFO level:       %u\n", _fifo_remaining);
    output->concat("\tTemperature:      ");
    if (temperatureEnabled()) {
      output->concatf("%.2fC (age: %ums)\n", _temperature, wrap_accounted_delta(now, _last_read_temp));
    }
    else {
      output->concat("Disabled\n");
    }
    output->concatf("\tAccel: (%.4f, %.4f, %.4f)\n", _acc.x, _acc.y, _acc.z);
    output->concatf("\t         +/-%d g\t  %.4f g/bit\n", _acc_get_sample_range(), _data_scale_acc);
    output->concatf("\tGyro:  (%.4f, %.4f, %.4f)\n", _gyro.x, _gyro.y, _gyro.z);
    output->concatf("\t         +/-%d deg/s\t  %.4f deg/s/bit\n", _gyro_get_sample_range(), _data_scale_gyro);
  }
  output->concat("\n");
}


void LSM6DSOX::printRegisters(StringBuilder* output) {
  StringBuilder::styleHeader1(output, "LSM6DSOX Registers");
  const uint8_t LOOP_COUNT = 26;
  for (uint8_t i = 0; i < LOOP_COUNT; i++) {
    output->concatf(
      "\t0x%02x:  0x%02x\t0x%02x:  0x%02x\t0x%02x:  0x%02x\n",
      _get_reg_addr((LSM6DSOXRegister) i), _get_shadow_value((LSM6DSOXRegister) i),
      _get_reg_addr((LSM6DSOXRegister)(i+LOOP_COUNT)), _get_shadow_value((LSM6DSOXRegister) (i+LOOP_COUNT)),
      _get_reg_addr((LSM6DSOXRegister)(i+(LOOP_COUNT << 1))), _get_shadow_value((LSM6DSOXRegister) (i+(LOOP_COUNT << 1)))
    );
  }
  output->concat("\n");
}


void LSM6DSOX::printBusOps(StringBuilder* output) {
  StringBuilder::styleHeader1(output, "LSM6DSOX BusOps");
  StringBuilder::styleHeader2(output, "_imu_data_refresh");
  _imu_data_refresh.printDebug(output);
  StringBuilder::styleHeader2(output, "_fifo_lev_refresh");
  _fifo_lev_refresh.printDebug(output);
}


/*
* Initialization function. Sets up pins, takes bus adapter reference.
* The real init work is handled on callback with a successful WHO_AM_I match.
*/
int8_t LSM6DSOX::init(SPIAdapter* b) {
  int8_t ret = -1;
  _flags &= LSM6DSOX_FLAG_RESET_MASK;
  for (uint8_t i = 0; i < sizeof(reg_shadows); i++) {
    reg_shadows[i] = 0;
  }
  if (nullptr != b) {
    _BUS = b;
  }
  int8_t pin_ret = _ll_pin_init();
  if (nullptr != _BUS) {
    ret--;
    _imu_data_refresh.setAdapter(_BUS);
    _imu_data_refresh.shouldReap(false);
    _imu_data_refresh.shouldFreeBuffer(false);
    _imu_data_refresh.setBuffer(&reg_shadows[_get_shadow_offset(LSM6DSOXRegister::FIFO_DATA_OUT_TAG)], 7);
    _imu_data_refresh.csActiveHigh(false);
    _imu_data_refresh.bitsPerFrame(SPIFrameSize::BITS_8);
    _imu_data_refresh.maxFreq(10000000);
    _imu_data_refresh.cpol(true);  // This part needs SPI mode-3.
    _imu_data_refresh.cpha(true);
    _imu_data_refresh.setParams(_get_reg_addr(LSM6DSOXRegister::FIFO_DATA_OUT_TAG) | 0x80);

    c3p_log(LOG_LEV_INFO, "IMU", "LSM6DSOX::init(): FIFO shadows: %d", _get_shadow_offset(LSM6DSOXRegister::FIFO_DATA_OUT_TAG));
    c3p_log(LOG_LEV_INFO, "IMU", "LSM6DSOX::init(): LB shadows:   %p", &reg_shadows[sizeof(reg_shadows)-1]);

    _fifo_lev_refresh.setAdapter(_BUS);
    _fifo_lev_refresh.shouldReap(false);
    _fifo_lev_refresh.shouldFreeBuffer(false);
    _fifo_lev_refresh.setBuffer(&reg_shadows[_get_shadow_offset(LSM6DSOXRegister::FIFO_STATUS1)], 2);
    _fifo_lev_refresh.csActiveHigh(false);
    _fifo_lev_refresh.bitsPerFrame(SPIFrameSize::BITS_8);
    _fifo_lev_refresh.maxFreq(10000000);
    _fifo_lev_refresh.cpol(true);  // This part needs SPI mode-3.
    _fifo_lev_refresh.cpha(true);
    _fifo_lev_refresh.setParams(_get_reg_addr(LSM6DSOXRegister::FIFO_STATUS1) | 0x80);

    if (0 == pin_ret) {  // Pins are set up.
      ret--;
      if (0 == _read_register(LSM6DSOXRegister::WHO_AM_I)) {
        ret = 0;
      }
    }
  }
  return ret;
}


/*
* Poll LSM6DSOX for updated peripheral data
*/
int8_t LSM6DSOX::poll() {
  int8_t ret = -1;
  if (initialized() && enabled()) {
    const uint32_t POLLING_DELAY = (1000 / accODR());
    uint32_t now = millis();
    if (false) {
      // Interrupt pin changed. Find out why.
    }
    else {
      // No IRQ pin. Resort to polling.
      if (wrap_accounted_delta(_last_read_axes, now) >= POLLING_DELAY) {
        if (0 == _fifo_remaining) {
          if (wrap_accounted_delta(_last_read_fifo, now) >= (16*POLLING_DELAY)) {
            ret = _read_fifo_level();
          }
        }
      }
    }
    if (wrap_accounted_delta(_last_read_temp, now) >= POLLING_DELAY) {
      if (temperatureEnabled()) {
        _read_temperature();
      }
    }
  }
  return ret;
}


/*
* Alters all class flags and members back to their default values if the bus op
*   succeeds.
*/
int8_t LSM6DSOX::reset() {
  int8_t ret = -1;
  int8_t SW_RESET_mask = 0x01;

  _flags = _flags & LSM6DSOX_FLAG_RESET_MASK;
  // Soft reset.
  for (uint8_t i = 0; i < sizeof(reg_shadows); i++) {
    reg_shadows[i] = 0;
  }
  ret = _write_register(LSM6DSOXRegister::CTRL3_C, SW_RESET_mask);
  return ret;
}


/*
* Refresh implemented register shadows from hardware.
*/
int8_t LSM6DSOX::refresh() {
  int8_t ret = -1;
  if (0 == _read_register(LSM6DSOXRegister::FUNC_CONFIG_ACCESS)) {
    ret--;
    if (0 == _read_register(LSM6DSOXRegister::S4S_TPH)) { // Double check this register wasn't changed
      ret--;
      if (0 == _read_registers(LSM6DSOXRegister::FIFO_CTRL1, 6)) {
        ret--;
        if (0 == _read_registers(LSM6DSOXRegister::INT1_CTRL, 18)) {
          ret--;
          if (0 == _read_registers(LSM6DSOXRegister::OUT_TEMP, 13)) {
            ret--;
            if (0 == _read_register(LSM6DSOXRegister::STATUS_REG)) {
              ret = 0;
            }
          }
        }
      }
    }
  }
  return ret;
}


/*
* Returns the current data. Resets NEW_DATA flag.
*/
Vector3<float>* LSM6DSOX::getAcc() {
  _class_clear_flag(LSM6DSOX_FLAG_NEW_DATA_ACC);
  return &_acc;
}

/*
* Returns the current data. Resets NEW_DATA flag.
*/
Vector3<float>* LSM6DSOX::getGyro() {
  _class_clear_flag(LSM6DSOX_FLAG_NEW_DATA_GYRO);
  return &_gyro;
}

/*
* Reads temperature data (Celcius)
*/
double LSM6DSOX::getTemperature() {
  _class_clear_flag(LSM6DSOX_FLAG_NEW_DATA_TEMP);
  return _temperature;
}

/*
* Reads temperature data registers.
*/
int8_t LSM6DSOX::_read_temperature() {
  return _read_registers(LSM6DSOXRegister::OUT_TEMP, 2);
}


/*
* Returns ODR value of Accelerometer (Hz)
* Values rounded to the nearest whole number.
*/
uint16_t LSM6DSOX::accODR() {
  uint16_t ret = 0;
  switch ((0xF0 & _get_shadow_value(LSM6DSOXRegister::CTRL1_XL)) >> 4) {
    case 1:  ret = 13;    break;
    case 2:  ret = 26;    break;
    case 3:  ret = 52;    break;
    case 4:  ret = 104;   break;
    case 5:  ret = 208;   break;
    case 6:  ret = 416;   break;
    case 7:  ret = 833;   break;
    case 8:  ret = 1660;  break;
    case 9:  ret = 3330;  break;
    case 10: ret = 6660;  break;
    case 11: return (accLowPowerMode() ? 2 : 13);
  }
  return ret;
}

/*
* Returns ODR value of Gyroscope (Hz)
* Values rounded to the nearest whole number.
*/
uint16_t LSM6DSOX::gyroODR() {
  uint16_t ret = 0;
  switch ((0xF0 & _get_shadow_value(LSM6DSOXRegister::CTRL2_G)) >> 4) {
    case 1:  ret = 13;    break;
    case 2:  ret = 26;    break;
    case 3:  ret = 52;    break;
    case 4:  ret = 104;   break;
    case 5:  ret = 208;   break;
    case 6:  ret = 416;   break;
    case 7:  ret = 833;   break;
    case 8:  ret = 1660;  break;
    case 9:  ret = 3330;  break;
    case 10: ret = 6660;  break;
  }
  return ret;
}

/*
* Recalculates accelermoter data scale from register contents.
* Returns range value
*/
int8_t LSM6DSOX::_acc_get_sample_range() {
  uint8_t scale = ((_get_shadow_value(LSM6DSOXRegister::CTRL1_XL) & ACC_SAMPLE_RANGE_MASK) >> 4);
  switch(scale) {
    case 0x00:
      return 2;
    case 0x01:
      return (accFSMode() ? 16 : 2);
    case 0x02:
      return 4;
    case 0x03:
      return 8;
  }
  return 1;
}

/*
* Recalculates gyroscope data scale from register contents.
* Returns range value
*/
int16_t LSM6DSOX::_gyro_get_sample_range() {
  uint16_t scale = ((_get_shadow_value(LSM6DSOXRegister::CTRL2_G) & GYRO_SAMPLE_RANGE_MASK) >> 2);
  if (_get_shadow_value(LSM6DSOXRegister::CTRL2_G) & GYRO_FS_125_SCALE_MASK) {
    return 125;
  }
  else{
    switch(scale) {
      case 0x00:  return 250;
      case 0x01:  return 500;
      case 0x02:  return 1000;
      case 0x03:  return 2000;
    }
  }
  return 1;
}

/*
*/
int8_t LSM6DSOX::_read_axes() {
  int8_t ret = -3;
  if (_imu_data_refresh.isIdle()) {
    ret = _BUS->queue_io_job(&_imu_data_refresh);
  }
  return ret;
}

/*
*/
int8_t LSM6DSOX::_read_fifo_level() {
  int8_t ret = -3;
  if (_fifo_lev_refresh.isIdle()) {
    ret = _BUS->queue_io_job(&_fifo_lev_refresh);
  }
  return ret;
}

/*
*/
bool LSM6DSOX::enable(bool enable, LSM6DSOX_ODR LSM6DSOX_acc_ODR, LSM6DSOX_ODR LSM6DSOX_gyro_ODR) {
  uint8_t ODR_filter = 0xF0;

  /* Enable Accelerometer
     Activate ODR and leave Full-Scale Selection at default (0b00)
  */
  uint8_t ctrl1 = ODR_filter & _get_shadow_value(LSM6DSOXRegister::CTRL1_XL);
  ctrl1 |= (enable ? (uint8_t) LSM6DSOX_acc_ODR << 4 : 0x00);
  bool acc_success = !_write_register(LSM6DSOXRegister::CTRL1_XL, ctrl1);

  /* Enable Gyroscope
     Activate ODR and leave Full-Scale Selection at default (0b00)
  */
  uint8_t ctrl2 = ODR_filter & _get_shadow_value(LSM6DSOXRegister::CTRL2_G);
  ctrl2 |= (enable ? (uint8_t) LSM6DSOX_gyro_ODR << 4 : 0x00);
  bool gyro_success = !_write_register(LSM6DSOXRegister::CTRL2_G, ctrl2);
  _write_register(LSM6DSOXRegister::INT1_CTRL, 0x03);

  return (acc_success == true && gyro_success == true);
}


/*
* Returns true if LSM6DSOX is enabled
*/
bool LSM6DSOX::enabled() {
  bool acc_enabled = (0xF0 & _get_shadow_value(LSM6DSOXRegister::CTRL1_XL)) != 0x00;
  bool gyro_enabled = (0xF0 & _get_shadow_value(LSM6DSOXRegister::CTRL2_G)) != 0x00;
  return (acc_enabled && gyro_enabled);
};


/*
* Enables temperature sensor
*/
int8_t LSM6DSOX::enableTemp(LSM6DSOX_ODR odr_temp) {
  uint8_t ODR_filter = 0x30;
  int8_t ret = -1;
  // Clear ODR bits
  uint8_t ctrl_reg = ~ODR_filter & _get_shadow_value(LSM6DSOXRegister::FIFO_CTRL4);
  uint8_t shift_value = 0;
  switch (odr_temp) {
    case LSM6DSOX_ODR::ODR_52:     shift_value++;
    case LSM6DSOX_ODR::ODR_12_5:   shift_value++;
    case LSM6DSOX_ODR::ODR_1_6:    shift_value++;
    case LSM6DSOX_ODR::ODR_0:
      ctrl_reg |= (shift_value << 4);
      ret = (0 == _write_register(LSM6DSOXRegister::FIFO_CTRL4, ctrl_reg)) ? 0 : (ret-1);
      break;
    default:
      break;
  }
  return ret;
}


/*
* Returns true if various settings are enabled
*/
bool LSM6DSOX::temperatureEnabled() {
  return (0x30 & _get_shadow_value(LSM6DSOXRegister::FIFO_CTRL4));
};
bool LSM6DSOX::accLowPowerMode() {
  return (0x10 & _get_shadow_value(LSM6DSOXRegister::CTRL6_C));
};
bool LSM6DSOX::accFSMode() {
  return (0x02 & _get_shadow_value(LSM6DSOXRegister::CTRL6_C));
};
bool LSM6DSOX::gyroLowPowerMode() {
  return (0x80 & _get_shadow_value(LSM6DSOXRegister::CTRL7_G));
};

uint8_t LSM6DSOX::readStatus() {
  uint8_t status = _read_register(LSM6DSOXRegister::STATUS_REG);
  if(status & ACC_NEW_DATA_MASK) {
    _class_set_flag(LSM6DSOX_FLAG_NEW_DATA_ACC);
  }
  if(status & GYRO_NEW_DATA_MASK) {
    _class_set_flag(LSM6DSOX_FLAG_NEW_DATA_GYRO);
  }
  if(status & TEMP_NEW_DATA_MASK) {
    _class_set_flag(LSM6DSOX_FLAG_NEW_DATA_TEMP);
  }
  return status;
};

/*
* Convenience fxn. Returns 0 if register index is out of bounds.
*/
unsigned int LSM6DSOX::_get_shadow_value(LSM6DSOXRegister idx) {
  if (((uint8_t) idx) < sizeof(reg_shadows)) {
    const uint8_t* ptr = &reg_shadows[_get_shadow_offset(idx)];
    switch (_reg_width(idx)) {
      // These are the only two widths in the sensor. Sensor is always little-endian.
      case 1: return *(ptr);
      case 2: return (((uint16_t) *(ptr+1)) << 8) + *(ptr);
    }
  }
  return 0;
}


/*
* Convenience fxn. Returns 0 if register index is out of bounds.
*/
int8_t LSM6DSOX::_set_shadow_value(LSM6DSOXRegister reg, unsigned int val) {
  if (((uint8_t) reg) < sizeof(reg_shadows)) {
    uint8_t* ptr = &reg_shadows[_get_shadow_offset(reg)];
    switch (_reg_width(reg)) {
      // These are the only two widths in the sensor.
      case 1:
        *(ptr) = 0xFF & val;
        return 0;
      case 2:
        *(ptr + 0) = 0xFF & val;
        *(ptr + 1) = 0xFF & (val >> 8);
        return 0;
      default:
        break;
    }
  }
  return -1;
}


/*
*
*/
int8_t LSM6DSOX::_write_register(LSM6DSOXRegister r, uint8_t val) {
  int8_t ret = -2;
  if (_reg_range_writable(r, 1)) {
    ret++;
    if (nullptr != _BUS) {
      SPIBusOp* op = (SPIBusOp*) _BUS->new_op(BusOpcode::TX, (BusOpCallback*) this);
      reg_shadows[_get_shadow_offset(r)] = val;
      op->setParams(_get_reg_addr(r));   // Write without autoincrement.
      op->setBuffer(&reg_shadows[_get_shadow_offset(r)], 1);
      ret = queue_io_job(op);
    }
  }
  return ret;
}


/*
*
*/
int8_t LSM6DSOX::_write_registers(LSM6DSOXRegister r, uint8_t len) {
  int8_t ret = -2;
  if (_reg_range_writable(r, len)) {
    ret++;
    if (nullptr != _BUS) {
      SPIBusOp* op = (SPIBusOp*) _BUS->new_op(BusOpcode::TX, (BusOpCallback*) this);
      op->setParams(_get_reg_addr(r));
      op->setBuffer(&reg_shadows[_get_shadow_offset(r)], len);
      ret = queue_io_job(op);
    }
  }
  return ret;
}


/*
*
*/
int8_t LSM6DSOX::_read_register(LSM6DSOXRegister r) {
  int8_t ret = -1;
  if (nullptr != _BUS) {
    SPIBusOp* op = (SPIBusOp*) _BUS->new_op(BusOpcode::RX, (BusOpCallback*) this);
    op->setParams(_get_reg_addr(r) | 0x80);   // Read without autoincrement.
    op->setBuffer(&reg_shadows[_get_shadow_offset(r)], 1);
    ret = queue_io_job(op);
  }
  return ret;
}


/*
*
*/
int8_t LSM6DSOX::_read_registers(LSM6DSOXRegister r, uint8_t len) {
  int8_t ret = -2;
  if (_reg_range_continuous(r, len)) {
    ret++;
    if (nullptr != _BUS) {
      SPIBusOp* op = (SPIBusOp*) _BUS->new_op(BusOpcode::RX, (BusOpCallback*) this);
      op->setParams(_get_reg_addr(r) | 0x80);   // Read with autoincrement.
      op->setBuffer(&reg_shadows[_get_shadow_offset(r)], len);
      ret = queue_io_job(op);
    }
  }
  return ret;
}


/**
* Idempotent low-level pin intialization.
*
* @return 0 if the pins are setup. Nonzero otherwise.
*/
int8_t LSM6DSOX::_ll_pin_init() {
  int8_t ret = 0;
  if (!_class_flag(LSM6DSOX_FLAG_PINS_CONFIGURED)) {
    ret = -1;
    if (255 != _INT1_PIN) {  // Optional pin
      if (0 > pinMode(_INT1_PIN, GPIOMode::INPUT)) {
        return -2;
      }
    }
    if (255 != _INT2_PIN) {  // Optional pin
      if (0 > pinMode(_INT2_PIN, GPIOMode::INPUT)) {
        return -2;
      }
    }

    if (255 != _CS_PIN) {
      ret = -2;
      if (0 <= pinMode(_CS_PIN, GPIOMode::OUTPUT)) {
       setPin(_CS_PIN, true);
       _class_set_flag(LSM6DSOX_FLAG_PINS_CONFIGURED);
       ret = 0;
      }
    }
  }
  if (ret != 0) {
    c3p_log(LOG_LEV_INFO, "IMU", "LSM6DSOX::_ll_pin_init() returns %d\n", ret);
  }
  return ret;
}


/**
* Following discovery, calling this function will impart the user's desired
*   configuration onto the hardware.
*
* @return 0 on success
*/
int8_t LSM6DSOX::_impart_initial_config() {
  int8_t ret = -1;
  _set_shadow_value(LSM6DSOXRegister::FIFO_CTRL1, 0x80);    // The FIFO watermark threshold is 128 samples.
  _set_shadow_value(LSM6DSOXRegister::FIFO_CTRL2, 0x00);    // No compression, FIFO watermark doesn't stop sensors, no ODR-change.
  _set_shadow_value(LSM6DSOXRegister::FIFO_CTRL3, 0x33);    // 52 Hz batching.
  _set_shadow_value(LSM6DSOXRegister::FIFO_CTRL4, 0x36);    // Temperature data (No timestamp) in FIFO, continuous FIFO mode.
  if (0 == _write_registers(LSM6DSOXRegister::FIFO_CTRL1, 4)) {
    ret--;
    _set_shadow_value(LSM6DSOXRegister::CTRL1_XL, 0x30);     // +/-2g, 52 Hz ODR.
    _set_shadow_value(LSM6DSOXRegister::CTRL2_G,  0x32);     // +/-500 deg/sec, 52 Hz ODR.
    _set_shadow_value(LSM6DSOXRegister::CTRL3_C,  0x64);     // INT pins are active-low and push-pull, 4-wire SPI, sync'd register update,
    _set_shadow_value(LSM6DSOXRegister::CTRL4_C,  0x3C);     // Gyro not sleeping, INT2 not mapped to INT1, filter not settled inhibits DRDY, no i2c, LPF1 enabled.
    _set_shadow_value(LSM6DSOXRegister::CTRL5_C,  0x80);     // No ACC low-power mode, no rounding, no self-tests.
    _set_shadow_value(LSM6DSOXRegister::CTRL6_C,  0x10);     // No DEN triggering, no ACC high-performance, ACC offset weight is 2^-10 g/LSB, FTYPE = 000
    _set_shadow_value(LSM6DSOXRegister::CTRL7_G,  0x84);     // No GYR high-performance or high-pass filtering, no ACC offset, OIS is disabled and controlled from main bus.
    if (0 == _write_registers(LSM6DSOXRegister::CTRL1_XL, 7)) {
      ret = 0;
    }
  }

  return ret;
}



int8_t LSM6DSOX::_bus_read_callback(uint8_t reg_addr, uint8_t* buf, uint16_t len) {
  int8_t ret = BUSOP_CALLBACK_NOMINAL;
  while (0 < len) {
    const LSM6DSOXRegister reg = _get_reg_enum(reg_addr);
    if (_verbosity >= LOG_LEV_DEBUG) c3p_log(LOG_LEV_DEBUG, "IMU", "\tIMU read reg(0x%02x): 0x%04x", reg_addr, _get_shadow_value(reg));
    switch (reg) {   // Get the register byte.
      case LSM6DSOXRegister::FUNC_CONFIG_ACCESS:        // 1 byte      true
      case LSM6DSOXRegister::PIN_CTRL:                  // 1 byte      true
      case LSM6DSOXRegister::S4S_TPH:                   // 2 byte      true
      case LSM6DSOXRegister::S4S_RR:                    // 1 byte      true
      case LSM6DSOXRegister::FIFO_CTRL1:                // 1 byte      true
      case LSM6DSOXRegister::FIFO_CTRL2:                // 1 byte      true
      case LSM6DSOXRegister::FIFO_CTRL3:                // 1 byte      true
      case LSM6DSOXRegister::FIFO_CTRL4:                // 1 byte      true
      case LSM6DSOXRegister::COUNTER_BDR_REG1:          // 1 byte      true
      case LSM6DSOXRegister::COUNTER_BDR_REG2:          // 1 byte      true
      case LSM6DSOXRegister::INT1_CTRL:                 // 1 byte      true
      case LSM6DSOXRegister::INT2_CTRL:                 // 1 byte      true
        break;

      case LSM6DSOXRegister::WHO_AM_I:
        _class_set_flag(LSM6DSOX_FLAG_DEV_FOUND, (0x6C == *buf));
        if (devFound() && !initialized()) {
          _impart_initial_config();
        }
        break;

      case LSM6DSOXRegister::CTRL1_XL:                  // 1 byte      true
      case LSM6DSOXRegister::CTRL2_G:                   // 1 byte      true
      case LSM6DSOXRegister::CTRL3_C:                   // 1 byte      true
      case LSM6DSOXRegister::CTRL4_C:                   // 1 byte      true
      case LSM6DSOXRegister::CTRL5_C:                   // 1 byte      true
      case LSM6DSOXRegister::CTRL6_C:                   // 1 byte      true
      case LSM6DSOXRegister::CTRL7_G:                   // 1 byte      true
      case LSM6DSOXRegister::CTRL8_XL:                  // 1 byte      true
      case LSM6DSOXRegister::CTRL9_XL:                  // 1 byte      true
      case LSM6DSOXRegister::CTRL10_C:                  // 1 byte      true
      case LSM6DSOXRegister::ALL_INT_SRC:               // 1 byte     false
      case LSM6DSOXRegister::WAKE_UP_SRC:               // 1 byte     false
      case LSM6DSOXRegister::TAP_SRC:                   // 1 byte     false
      case LSM6DSOXRegister::D6D_SRC:                   // 1 byte     false
      case LSM6DSOXRegister::STATUS_REG:                // 1 byte     false
        break;

      case LSM6DSOXRegister::OUT_TEMP:                  // 2 byte     false
        if (2 <= len) {
          int16_t temp_raw = (int16_t) (((uint16_t) *(buf + 1) << 8) | *buf);
          _temperature = ((double) temp_raw / 256.0) + 25.0;
          _last_read_temp = millis();
        }
        break;

      case LSM6DSOXRegister::OUTX_G:                    // 2 byte     false
      case LSM6DSOXRegister::OUTY_G:                    // 2 byte     false
      case LSM6DSOXRegister::OUTZ_G:                    // 2 byte     false
      case LSM6DSOXRegister::OUTX_A:                    // 2 byte     false
      case LSM6DSOXRegister::OUTY_A:                    // 2 byte     false
      case LSM6DSOXRegister::OUTZ_A:                    // 2 byte     false
      case LSM6DSOXRegister::EMB_FUNC_STATUS_MAINPAGE:  // 1 byte     false
      case LSM6DSOXRegister::FSM_STATUS_A_MAINPAGE:     // 1 byte     false
      case LSM6DSOXRegister::FSM_STATUS_B_MAINPAGE:     // 1 byte     false
      case LSM6DSOXRegister::MLC_STATUS_MAINPAGE:       // 1 byte     false
      case LSM6DSOXRegister::STATUS_MASTER_MAINPAGE:    // 1 byte     false
        break;

      case LSM6DSOXRegister::FIFO_STATUS1:              // 1 byte     false
      case LSM6DSOXRegister::FIFO_STATUS2:              // 1 byte     false
        _last_read_fifo = millis();
        _fifo_remaining = (((uint16_t) *(buf + 1) << 8) | *buf) & 0x03FF;
        if (_fifo_remaining > 0) {
          _read_axes();
        }
        break;

      case LSM6DSOXRegister::TIMESTAMP0:                // 1 byte     false
      case LSM6DSOXRegister::TIMESTAMP1:                // 1 byte     false
      case LSM6DSOXRegister::TIMESTAMP2:                // 1 byte     false
      case LSM6DSOXRegister::TIMESTAMP3:                // 1 byte     false
      case LSM6DSOXRegister::UI_STATUS_REG_OIS:         // 1 byte     false
      case LSM6DSOXRegister::UI_OUTX_G_OIS:             // 2 byte     false
      case LSM6DSOXRegister::UI_OUTY_G_OIS:             // 2 byte     false
      case LSM6DSOXRegister::UI_OUTZ_G_OIS:             // 2 byte     false
      case LSM6DSOXRegister::UI_OUTX_A_OIS:             // 2 byte     false
      case LSM6DSOXRegister::UI_OUTY_A_OIS:             // 2 byte     false
      case LSM6DSOXRegister::UI_OUTZ_A_OIS:             // 2 byte     false
      case LSM6DSOXRegister::TAP_CFG0:                  // 1 byte      true
      case LSM6DSOXRegister::TAP_CFG1:                  // 1 byte      true
      case LSM6DSOXRegister::TAP_CFG2:                  // 1 byte      true
      case LSM6DSOXRegister::TAP_THS_6D:                // 1 byte      true
      case LSM6DSOXRegister::INT_DUR2:                  // 1 byte      true
      case LSM6DSOXRegister::WAKE_UP_THS:               // 1 byte      true
      case LSM6DSOXRegister::WAKE_UP_DUR:               // 1 byte      true
      case LSM6DSOXRegister::FREE_FALL:                 // 1 byte      true
      case LSM6DSOXRegister::MD1_CFG:                   // 1 byte      true
      case LSM6DSOXRegister::MD2_CFG:                   // 1 byte      true
      case LSM6DSOXRegister::S4S_ST_CMD_CODE:           // 1 byte      true
      case LSM6DSOXRegister::S4S_DT_REG:                // 1 byte      true
      case LSM6DSOXRegister::I3C_BUS_AVB:               // 1 byte      true
      case LSM6DSOXRegister::INTERNAL_FREQ_FINE:        // 1 byte     false
      case LSM6DSOXRegister::UI_INT_OIS:                // 1 byte      true   // R/W is contingent
      case LSM6DSOXRegister::UI_CTRL1_OIS:              // 1 byte      true   // R/W is contingent
      case LSM6DSOXRegister::UI_CTRL2_OIS:              // 1 byte      true   // R/W is contingent
      case LSM6DSOXRegister::UI_CTRL3_OIS:              // 1 byte      true   // R/W is contingent
      case LSM6DSOXRegister::X_OFS_USR:                 // 1 byte      true
      case LSM6DSOXRegister::Y_OFS_USR:                 // 1 byte      true
      case LSM6DSOXRegister::Z_OFS_USR:                 // 1 byte      true
        break;

      case LSM6DSOXRegister::FIFO_DATA_OUT_TAG:
        // This register is the start of a FIFO read, and IDs the type of data.
        if (7 <= len) {
          int16_t new_x = (int16_t) (((uint16_t) *(buf + 2) << 8) | *(buf + 1));
          int16_t new_y = (int16_t) (((uint16_t) *(buf + 4) << 8) | *(buf + 3));
          int16_t new_z = (int16_t) (((uint16_t) *(buf + 6) << 8) | *(buf + 5));
          switch ((*buf >> 3) & 0x1F) {
            case 0x01:  // Gyro data
              {
                _gyro.set(new_x * _data_scale_gyro, new_y * _data_scale_gyro, new_z * _data_scale_gyro);
                _gyro = _gyro - _offset_gyro;
                _last_read_gyro = millis();
                if ((nullptr != _pipeline) && calibrated()) {
                  _pipeline->pushVector(SpatialSense::GYR, &_gyro); // TODO: No error bars on this data.
                }
              }
              break;
            case 0x02:  // Accelerometer data
              {
                _acc.set(new_x * _data_scale_acc, new_y * _data_scale_acc, new_z * _data_scale_acc);
                _acc = _acc - _offset_acc;
                _last_read_axes = millis();
                if ((nullptr != _pipeline) && calibrated()) {
                  _pipeline->pushVector(SpatialSense::ACC, &_acc); // TODO: No error bars on this data.
                }
              }
              break;
            case 0x03:  // Temperature data
              {
                // TODO: Temperature data not in FIFO.
                int16_t temp_raw = (int16_t) (((uint16_t) *(buf + 2) << 8) | *(buf + 1));
                _temperature = ((double) temp_raw / 256.0) + 25.0;
                _last_read_temp = millis();
              }
              break;
            case 0x04:  // Timestamp
              break;
            default:    // Any other data tags are ignored.
              break;
          }
          len -= 6;
          reg_addr += 3;
          buf += 6;
        }
        break;

      case LSM6DSOXRegister::FIFO_DATA_OUT_X:           // 2 byte     false
      case LSM6DSOXRegister::FIFO_DATA_OUT_Y:           // 2 byte     false
      case LSM6DSOXRegister::FIFO_DATA_OUT_Z:           // 2 byte     false
      default:   // Anything else is invalid.
        break;
    }
    len--;
    reg_addr++;
    buf++;
  }
  return ret;
}



int8_t LSM6DSOX::_bus_write_callback(uint8_t reg_addr, uint8_t* buf, uint16_t len) {
  int8_t ret = BUSOP_CALLBACK_NOMINAL;
  while (0 < len) {
    const LSM6DSOXRegister reg = _get_reg_enum(reg_addr);
    if (_verbosity >= LOG_LEV_DEBUG) c3p_log(LOG_LEV_DEBUG, "IMU", "\tIMU wrote reg(0x%02x): 0x%04x", reg_addr, _get_shadow_value(reg));
    switch (reg) {   // Get the register byte.
      case LSM6DSOXRegister::FUNC_CONFIG_ACCESS:
      case LSM6DSOXRegister::PIN_CTRL:
      case LSM6DSOXRegister::S4S_TPH:                   // 2 bytes
      case LSM6DSOXRegister::S4S_RR:
      case LSM6DSOXRegister::FIFO_CTRL1:
      case LSM6DSOXRegister::FIFO_CTRL2:
      case LSM6DSOXRegister::FIFO_CTRL3:
      case LSM6DSOXRegister::FIFO_CTRL4:
      case LSM6DSOXRegister::COUNTER_BDR_REG1:
      case LSM6DSOXRegister::COUNTER_BDR_REG2:
      case LSM6DSOXRegister::INT1_CTRL:
      case LSM6DSOXRegister::INT2_CTRL:
        break;

      case LSM6DSOXRegister::CTRL1_XL:
        switch ((*buf >> 2) & 0x03) {
          case 0:   _data_scale_acc = 0.000061037018952;  break;   // +/-2g
          case 1:   _data_scale_acc = 0.000122074037904;  break;   // +/-4g
          case 2:   _data_scale_acc = 0.000244148075808;  break;   // +/-8g
          case 3:   _data_scale_acc = 0.000488296151616;  break;   // +/-16g
        }
        break;

      case LSM6DSOXRegister::CTRL2_G:
        if (*buf & GYRO_FS_125_SCALE_MASK) {
          _data_scale_gyro = 0.000066581059144;  // +/-125 deg/sec
        }
        else {
          switch ((*buf >> 2) & 0x03) {
            case 0:   _data_scale_gyro = 0.000133162118289;  break;   // +/-2g
            case 1:   _data_scale_gyro = 0.000266324236579;  break;   // +/-4g
            case 2:   _data_scale_gyro = 0.000532648473157;  break;   // +/-8g
            case 3:   _data_scale_gyro = 0.001065296946314;  break;   // +/-16g
          }
        }
        break;

      case LSM6DSOXRegister::CTRL3_C:
      case LSM6DSOXRegister::CTRL4_C:
      case LSM6DSOXRegister::CTRL5_C:
      case LSM6DSOXRegister::CTRL6_C:
        break;
      case LSM6DSOXRegister::CTRL7_G:
        if (!initialized()) {
          _class_set_flag(LSM6DSOX_FLAG_INITIALIZED);
        }
        break;
      case LSM6DSOXRegister::CTRL8_XL:
      case LSM6DSOXRegister::CTRL9_XL:
      case LSM6DSOXRegister::CTRL10_C:
      case LSM6DSOXRegister::TAP_CFG0:
      case LSM6DSOXRegister::TAP_CFG1:
      case LSM6DSOXRegister::TAP_CFG2:
      case LSM6DSOXRegister::TAP_THS_6D:
      case LSM6DSOXRegister::INT_DUR2:
      case LSM6DSOXRegister::WAKE_UP_THS:
      case LSM6DSOXRegister::WAKE_UP_DUR:
      case LSM6DSOXRegister::FREE_FALL:
      case LSM6DSOXRegister::MD1_CFG:
      case LSM6DSOXRegister::MD2_CFG:
      case LSM6DSOXRegister::S4S_ST_CMD_CODE:
      case LSM6DSOXRegister::S4S_DT_REG:
      case LSM6DSOXRegister::I3C_BUS_AVB:
      case LSM6DSOXRegister::UI_INT_OIS:                // R/W is contingent
      case LSM6DSOXRegister::UI_CTRL1_OIS:              // R/W is contingent
      case LSM6DSOXRegister::UI_CTRL2_OIS:              // R/W is contingent
      case LSM6DSOXRegister::UI_CTRL3_OIS:              // R/W is contingent
      case LSM6DSOXRegister::X_OFS_USR:
      case LSM6DSOXRegister::Y_OFS_USR:
      case LSM6DSOXRegister::Z_OFS_USR:
      default:   // Anything else is invalid.
        break;
    }
    len--;
    reg_addr++;
    buf++;
  }
  return ret;
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
int8_t LSM6DSOX::io_op_callahead(BusOp* _op) {
  return 0;   // Permit the transfer, always.
}

/**
* When a bus operation completes, it is passed back to its issuing class.
*
* @param  _op  The bus operation that was completed.
* @return BUSOP_CALLBACK_NOMINAL on success, or appropriate error code.
*/
int8_t LSM6DSOX::io_op_callback(BusOp* _op) {
  int8_t ret = BUSOP_CALLBACK_NOMINAL;
  SPIBusOp* op = (SPIBusOp*) _op;

  if (op->hasFault()) {
    //c3p_log(LOG_LEV_INFO, "IMU", "io_op_callback() rejected a callback because the bus op failed.\n");
    return BUSOP_CALLBACK_ERROR;
  }

  uint8_t* buf = op->buffer();
  uint16_t len = op->bufferLen();
  uint8_t reg_addr = op->getTransferParam(0) & 0x7F;
  switch (op->get_opcode()) {
    case BusOpcode::TX:  ret = _bus_write_callback(reg_addr, buf, len);   break;
    case BusOpcode::RX:  ret = _bus_read_callback(reg_addr, buf, len);    break;
    default:  break;
  }

  if ((op == &_imu_data_refresh) && (_fifo_remaining > 0)) {
    _fifo_remaining--;
    if (_fifo_remaining > 0) {
      ret = BUSOP_CALLBACK_RECYCLE;
    }
    else {
      _read_fifo_level();
    }
  }
  return ret;
}


int8_t LSM6DSOX::queue_io_job(BusOp* _op) {
  // This is the choke-point whereby any parameters to the operation that are
  //   uniform for this driver can be set.
  SPIBusOp* op = (SPIBusOp*) _op;
  op->setCSPin(_CS_PIN);
  op->csActiveHigh(false);
  op->bitsPerFrame(SPIFrameSize::BITS_8);
  op->maxFreq(10000000);
  op->cpol(true);  // This part needs SPI mode-3.
  op->cpha(true);
  return _BUS->queue_io_job(op);
}


/**
* @page console-handlers
* @section lsm6dsox-tools LSM6DSOX tools
*
* This is the console handler for debugging the LSM6DSOX 6-DoF IMU. Called without
*   arguments, this command will print the driver overview.
*
* @subsection cmd-actions Actions
*
* Action    | Description | Additional arguments
* --------- | ----------- | --------------------
* 'init'    | Manually invoke the driver's `init()` function. | None
* `reset`   | Manually invoke the driver's `reset()` function. | None
* `poll`    | Manually invoke the driver's `poll()` function. | None
* 'refresh' | Refresh the register shadows from the hardware. | None
* 'enable'  | Enable or disable the hardware. | [Enable]
*/
int LSM6DSOX::console_handler(StringBuilder* text_return, StringBuilder* args) {
  int8_t ret = 0;
  char*  cmd = args->position_trimmed(0);
  if (0 == StringBuilder::strcasecmp(cmd, "init")) {
    text_return->concatf("IMU.init() returns %d.\n", init());
  }
  else if (0 == StringBuilder::strcasecmp(cmd, "busops")) {
    printBusOps(text_return);
  }
  else if (0 == StringBuilder::strcasecmp(cmd, "regs")) {
    printRegisters(text_return);
  }
  else if (0 == StringBuilder::strcasecmp(cmd, "poll")) {
    text_return->concatf("IMU.poll() returns %d.\n", poll());
  }
  else if (0 == StringBuilder::strcasecmp(cmd, "axes")) {
    text_return->concatf("IMU._read_axes() returns %d.\n", _read_axes());
  }
  else if (0 == StringBuilder::strcasecmp(cmd, "fifo")) {
    text_return->concatf("IMU._read_fifo_level() returns %d.\n", _read_fifo_level());
  }
  else if (0 == StringBuilder::strcasecmp(cmd, "refresh")) {
    text_return->concatf("IMU.refresh() returns %d.\n", refresh());
  }
  else if (0 == StringBuilder::strcasecmp(cmd, "reset")) {
    text_return->concatf("IMU.reset() returns %d.\n", reset());
  }
  else if (0 == StringBuilder::strcasecmp(cmd, "enable")) {
    if (1 < args->count()) {
      bool en = (0 != args->position_as_int(1));
      enableTemp(en ? LSM6DSOX_ODR::ODR_1_6 : LSM6DSOX_ODR::ODR_0);
      text_return->concatf("IMU.enable() returns %d.\n", enable(en));
    }
    else text_return->concatf("IMU is %sabled.\n", enabled()?"en":"dis");
  }
  else if (0 == StringBuilder::strcasecmp(cmd, "verbosity")) {
    if (1 < args->count()) {
      _verbosity = strict_min((uint8_t) 7, (uint8_t) args->position_as_int(1));
    }
    text_return->concatf("IMU verbosity: %u\n", _verbosity);
  }
  else {
    printDebug(text_return);
  }
  return ret;
}
