#include "LSM6DSOX.h"


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
  0x00, 0x01, 0x02, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C,
  0x0D, 0x0E, 0x0F, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18,
  0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1F, 0x21, 0x23, 0x25, 0x27, 0x29, 0x2B,
  0x2C, 0x2D, 0x2E, 0x2F, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37,
  0x39, 0x3B, 0x3D, 0x3F, 0x41, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49,
  0x4A, 0x4B, 0x4C, 0x4D, 0x4E, 0x4F, 0x50, 0x51, 0x52, 0x53, 0x54, 0x55,
  0x56, 0x57, 0x58, 0x59, 0x5B, 0x5D
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

static const uint8_t _get_shadow_offset(const LSM6DSOXRegister id) {
  return LSM6DSOX_SHADOW_OFFSETS[(const uint8_t) id];
}

static const uint8_t _reg_width(const LSM6DSOXRegister id) {
  return LSM6DSOX_REG_WIDTH_MAP[(const uint8_t) id];
}

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
      for (uint8_t i = 1; i < len; i++) {
        uint8_t r_this = r_int + i;
        if (LSM6DSOX_REGISTER_ADDRESSES[r_this] != (r_last + 1)) {  // Check for discontinuity.
          return false;
        }
        r_last = LSM6DSOX_REGISTER_ADDRESSES[r_this];
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
    _fifo_lev_refresh(BusOpcode::RX, this, cs_pin, false)
   {}


/*
* Destructor
*/
LSM6DSOX::~LSM6DSOX() {}


/*
* Debug
*/
void LSM6DSOX::printDebug(StringBuilder* output) {
  output->concat("-- LSM6DSOX\n");
  output->concatf("--\tPins conf'd:   %c\n", _class_flag(LSM6DSOX_FLAG_PINS_CONFIGURED) ? 'y':'n');
  output->concatf("--\tFound:         %c\n", devFound() ? 'y':'n');
  output->concatf("--\tInitialized:   %c\n", initialized() ? 'y':'n');
  output->concatf("--\tCalibrated:   %c\n", calibrated() ? 'y':'n');
  output->concatf("--\tEnabled:       %c\n", enabled() ? 'y':'n');
  output->concatf("--\tTemp enabled:  %c\n", temperatureEnabled() ? 'y':'n');
  if (temperatureEnabled()) {
    output->concatf("--\tTemperature:   %.2fC\n", _temperature);
  }
  output->concatf("--\tlast_temp:     %u\n", _last_read_temp);
  output->concatf("--\tlast_axes:     %u\n", _last_read_axes);
  output->concatf("--\tg/bit:         %.4f\n", _data_scale_acc);
  output->concatf("--\tValue:         (%.4f, %.4f, %.4f)\n", _acc.x, _acc.y, _acc.z);
  output->concatf("--\trad/s/bit:     %.4f\n", _data_scale_gyr);
  output->concatf("--\tValue:         (%.4f, %.4f, %.4f)\n", _gyr.x, _gyr.y, _gyr.z);

  //uint8_t scale = 0x04 << ((reg_shadows[(uint8_t) LSM6DSOXRegister::CTRL_4] >> 4) & 0x03);
  uint8_t scale = 1;
  output->concatf("--\tScale:         +/-%dg\n", scale >> 1);
  output->concatf("--\tSample size:   %u-bit\n", _bits_per_sample());

  output->concatf("--\tfifo depth:    %u\n", _fifo_remaining);
  const uint8_t LOOP_COUNT = 26;
  for (uint8_t i = 0; i < LOOP_COUNT; i++) {
    output->concatf(
      "\t0x%02x:  0x%02x\t0x%02x:  0x%02x\t0x%02x:  0x%02x\n",
      _get_reg_addr((LSM6DSOXRegister) i), _get_shadow_value((LSM6DSOXRegister) i),
      _get_reg_addr((LSM6DSOXRegister)(i+LOOP_COUNT)), _get_shadow_value((LSM6DSOXRegister) (i+LOOP_COUNT)),
      _get_reg_addr((LSM6DSOXRegister)(i+(LOOP_COUNT << 1))), _get_shadow_value((LSM6DSOXRegister) (i+(LOOP_COUNT << 1)))
    );
  }
}


/*
* TODO: Unconfuse return values.
* Initialization function. Sets up pins, takes bus adapter reference.
* The real init work is handled on callback with a successful WHO_AM_I match.
*/
int8_t LSM6DSOX::init(SPIAdapter* b) {
  _flags &= LSM6DSOX_FLAG_RESET_MASK;
  int8_t ret = _ll_pin_init();
  _BUS = b;
  for (uint8_t i = 0; i < sizeof(reg_shadows); i++) {
    reg_shadows[i] = 0;
  }

  _imu_data_refresh.setAdapter(_BUS);
  _imu_data_refresh.shouldReap(false);
  _imu_data_refresh.setBuffer(&reg_shadows[_get_shadow_offset(LSM6DSOXRegister::FIFO_DATA_OUT_TAG)], 7);
  _imu_data_refresh.csActiveHigh(false);
  _imu_data_refresh.bitsPerFrame(SPIFrameSize::BITS_8);
  _imu_data_refresh.maxFreq(10000000);
  _imu_data_refresh.cpol(true);  // This part needs SPI mode-3.
  _imu_data_refresh.cpha(true);
  _imu_data_refresh.setParams(_get_reg_addr(LSM6DSOXRegister::OUT_TEMP));

  _fifo_lev_refresh.setAdapter(_BUS);
  _fifo_lev_refresh.shouldReap(false);
  _fifo_lev_refresh.setBuffer(&reg_shadows[_get_shadow_offset(LSM6DSOXRegister::FIFO_STATUS1)], 2);
  _fifo_lev_refresh.csActiveHigh(false);
  _fifo_lev_refresh.bitsPerFrame(SPIFrameSize::BITS_8);
  _fifo_lev_refresh.maxFreq(10000000);
  _fifo_lev_refresh.cpol(true);  // This part needs SPI mode-3.
  _fifo_lev_refresh.cpha(true);
  _fifo_lev_refresh.setParams(_get_reg_addr(LSM6DSOXRegister::FIFO_STATUS1) | 0x80);

  if (0 == ret) {  // Pins are set up.
    if (nullptr != _BUS) {
      ret = _read_register(LSM6DSOXRegister::WHO_AM_I);
    }
    else {
      ret = -2;
    }
  }
  return ret;
}


int8_t LSM6DSOX::poll() {
  int8_t ret = -1;
  if (initialized() && enabled()) {
    if (millis() >= _last_read_temp + (1000 / getODR())) {
      if (0 == _fifo_remaining) {
        ret = _read_fifo_level();
      }
      if (temperatureEnabled()) {
        _read_temperature();
      }
    }
  }
  return ret;
}


/*
* TODO: this
* Alters all class flags and members back to their default values if the bus op
*   succeeds.
*/
int8_t LSM6DSOX::reset() {
  int8_t ret = -1;
  _flags = _flags & LSM6DSOX_FLAG_RESET_MASK;
  // Soft reset.
  for (uint8_t i = 0; i < sizeof(reg_shadows); i++) {
    reg_shadows[i] = 0;
  }
  //ret = _write_register(LSM6DSOXRegister::CTRL_5, 0x80);
  return ret;
}


/*
* Refresh all register shadows from hardware.
*/
int8_t LSM6DSOX::refresh() {
  int8_t ret = -1;
  if (0 == _read_registers(LSM6DSOXRegister::FUNC_CONFIG_ACCESS, 2)) {
    ret--;
    if (0 == _read_registers(LSM6DSOXRegister::S4S_TPH, 27)) {
      ret--;
      if (0 == _read_registers(LSM6DSOXRegister::FIFO_STATUS1, 2)) {
        ret--;
        if (0 == _read_registers(LSM6DSOXRegister::TAP_CFG0, 12)) {
          /*
          0x20:  0x00
          0x21:  0x00
          0x22:  0x00
          0x23:  0x00
          0x24:  0x00
          0x25:  0x00
          0x26:  0x00
          0x27:  0x00
          0x28:  0x00
          0x29:  0x00
          0x2a:  0x00
          0x2b:  0x00
          0x2c:  0x00
          0x2d:  0x00
          0x2e:  0x00
          0x2f:  0x00
          0x30:  0x00
          0x31:  0x00
          0x32:  0x00
          0x33:  0x00
          0x34:  0x00
          0x35:  0x00
          0x36:  0x00
          0x37:  0x00
          0x38:  0x00
          0x39:  0x00
          0x3a:  0x00
          0x3b:  0x00
          0x3c:  0x00
          0x3d:  0x00
          0x3e:  0x00
          0x3f:  0x00
          0x40:  0x00
          0x41:  0x00
          0x42:  0x00

          0x49:  0x00
          0x4a:  0x00
          0x4b:  0x00
          0x4c:  0x00
          0x4d:  0x00
          0x4e:  0x00
          0x4f:  0x00
          0x50:  0x00
          0x51:  0x00
          0x52:  0x00
          0x53:  0x00

          0x58:  0x00
          0x59:  0x00
          0x5a:  0x00
          0x5b:  0x00
          0x5c:  0x00
          0x5d:  0x00
          0x5e:  0x00
          0x5f:  0x00

          0x66:  0x00
          0x67:  0x00
          0x68:  0x00
          0x69:  0x00
          0x6a:  0x00
          0x6b:  0x00
          */
          ret = 0;
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
  _class_clear_flag(LSM6DSOX_FLAG_NEW_DATA);
  return &_acc;
}


uint16_t LSM6DSOX::getODR() {
  uint16_t ret = 0;
  //switch ((0xF0 & reg_shadows[(uint8_t) LSM6DSOXRegister::CTRL_1]) >> 4) {
  //  case 1:  ret = 1;     break;
  //  case 2:  ret = 10;    break;
  //  case 3:  ret = 25;    break;
  //  case 4:  ret = 50;    break;
  //  case 5:  ret = 100;   break;
  //  case 6:  ret = 200;   break;
  //  case 7:  ret = 400;   break;
  //  case 8:  ret = lowPowerMode() ? 1620 : 0;      break;
  //  case 9:  ret = lowPowerMode() ? 5376 : 1344;   break;
  //}
  return ret;
}




uint8_t LSM6DSOX::_bits_per_sample() {
  uint8_t ret = 8;
  //if (0 == ((reg_shadows[(uint8_t) LSM6DSOXRegister::CTRL_1] >> 3) & 0x01)) {
  //  ret = ((reg_shadows[(uint8_t) LSM6DSOXRegister::CTRL_4] >> 3) & 0x01) ? 12 : 10;
  //}
  return ret;
}


/*
* Recalculates _data_scale from register contents.
* Returns 0 on success, or -1 on failure.
*/
int8_t LSM6DSOX::_find_sample_range() {
  int8_t ret = 0;
  //uint8_t scale = 0x04 << ((reg_shadows[(uint8_t) LSM6DSOXRegister::CTRL_4] >> 4) & 0x03);
  //_data_scale = (float) scale / (float) (1 << _bits_per_sample());
  return ret;
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



int8_t LSM6DSOX::enable(bool x) {
  //uint8_t ctrl1 = 0x08 & reg_shadows[(uint8_t) LSM6DSOXRegister::CTRL_1];
  //ctrl1 |= (x ? 0x37 : 0x00);
  //return _write_register(LSM6DSOXRegister::CTRL_1, ctrl1);
  return -1;
}


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
      reg_shadows[(uint8_t) r] = val;
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
int8_t LSM6DSOX::_write_registers(LSM6DSOXRegister r, uint8_t* buf, uint8_t len) {
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
    if (255 != _INT1_PIN) {  // Optional pin
      if (0 != pinMode(_INT1_PIN, GPIOMode::INPUT)) {
        return -2;
      }
    }
    if (255 != _INT2_PIN) {  // Optional pin
      if (0 != pinMode(_INT2_PIN, GPIOMode::INPUT)) {
        return -2;
      }
    }

    if (255 != _CS_PIN) {
      pinMode(_CS_PIN, GPIOMode::OUTPUT);
      //if (0 != pinMode(_CS_PIN, GPIOMode::OUTPUT)) {
      //  return -2;
      //}
      setPin(_CS_PIN, true);
      _class_set_flag(LSM6DSOX_FLAG_PINS_CONFIGURED);
    }
    else {
      ret = -1;
    }
  }
  return ret;
}


/*
* Following discovery, calling this function will impart the user's desired
*   configuration onto the hardware.
*
* @return 0 on success
*/
int8_t LSM6DSOX::_impart_initial_config() {
  int8_t ret = -1;
  // The FIFO watermark threshold is 16 samples.
  _write_register(LSM6DSOXRegister::FIFO_CTRL1, 0x10);
  // No compression, FIFO watermark doesn't stop sensors, no ODR-change.
  _write_register(LSM6DSOXRegister::FIFO_CTRL2, 0x00);
  // No batching.
  _write_register(LSM6DSOXRegister::FIFO_CTRL3, 0x00);
  // No timestamp and no temperature data in FIFO, continuous FIFO mode.
  _write_register(LSM6DSOXRegister::FIFO_CTRL4, 0x06);

  // INT pins are active-low and push-pull, 4-wire SPI, sync'd register update,
  _write_register(LSM6DSOXRegister::CTRL3_C, 0x64);

  // Gyro not sleeping, INT2 not mapped to INT1, filter not settled inhibits DRDY, no i2c, LPF1 enabled.
  _write_register(LSM6DSOXRegister::CTRL4_C, 0xE0);

  // No ACC low-power mode, no rounding, no self-tests.
  _write_register(LSM6DSOXRegister::CTRL5_C, 0xE0);

  // No DEN triggering, no ACC high-performance, ACC offset weight is 2^-10 g/LSB, FTYPE = 000
  _write_register(LSM6DSOXRegister::CTRL6_C, 0x10);

  // No GYR high-performance or high-pass filtering, no ACC offset, OIS is disabled and controlled from main bus.
  _write_register(LSM6DSOXRegister::CTRL7_G, 0x04);
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

  // There is zero chance this object will be a null pointer unless it was done on purpose.
  if (op->hasFault()) {
    //if (getVerbosity() > 3) local_log.concat("io_op_callback() rejected a callback because the bus op failed.\n");
    return BUSOP_CALLBACK_ERROR;
  }
  uint8_t* buf = op->buffer();
  uint16_t len = op->bufferLen();
  uint8_t reg_idx = op->getTransferParam(0) & 0x7F;

  // No need to test for R/W on RO or WO registers. It is controlled for upstream.
  switch (op->get_opcode()) {

    case BusOpcode::TX:
      while (0 < len) {
        switch (_get_reg_enum(reg_idx)) {   // Get the register byte.
          default:
            break;
        }
        len--;
        reg_idx++;
        buf++;
      }
      break;

    case BusOpcode::RX:
      while (0 < len) {
        switch (_get_reg_enum(reg_idx)) {   // Get the register byte.
          case LSM6DSOXRegister::WHO_AM_I:
            _class_set_flag(LSM6DSOX_FLAG_DEV_FOUND, (0x6C == *buf));
            if (devFound() && !initialized()) {
              _impart_initial_config();
            }
            break;

          // This register is the start of a FIFO read, and IDs the type of data.
          case LSM6DSOXRegister::FIFO_DATA_OUT_TAG:
            if (7 <= len) {
              switch (*buf & 0x1F) {
                case 0x01:  // Gyro data
                  {
                    int16_t new_x_gyr = (int16_t) (((uint16_t) *(buf + 2) << 8) | *(buf + 1));
                    int16_t new_y_gyr = (int16_t) (((uint16_t) *(buf + 4) << 8) | *(buf + 3));
                    int16_t new_z_gyr = (int16_t) (((uint16_t) *(buf + 6) << 8) | *(buf + 5));
                    _gyr.set(new_x_gyr * _data_scale_gyr, new_y_gyr * _data_scale_gyr, new_z_gyr * _data_scale_gyr);
                    _gyr = _gyr - _offset_gyr;
                    if ((nullptr != _pipeline) && calibrated()) {
                      _pipeline->pushVector(SpatialSense::GYR, &_gyr);   // TODO: No error bars on this data.
                    }
                  }
                  break;
                case 0x02:  // Accelerometer data
                  {
                    int16_t new_x_acc = (int16_t) (((uint16_t) *(buf + 2) << 8) | *(buf + 1));
                    int16_t new_y_acc = (int16_t) (((uint16_t) *(buf + 4) << 8) | *(buf + 3));
                    int16_t new_z_acc = (int16_t) (((uint16_t) *(buf + 6) << 8) | *(buf + 5));
                    _acc.set(new_x_acc * _data_scale_acc, new_y_acc * _data_scale_acc, new_z_acc * _data_scale_acc);
                    _acc = _acc - _offset_acc;
                    if ((nullptr != _pipeline) && calibrated()) {
                      _pipeline->pushVector(SpatialSense::ACC, &_acc);   // TODO: No error bars on this data.
                    }
                  }
                  break;
                case 0x03:  // Temperature data
                  _temperature = (((uint16_t) *(buf + 2) << 8) | *(buf + 1)) * 0.001;  // TODO: wrong
                  break;
                case 0x04:  // Timestamp
                  break;
                default:    // Any other data tags are ignored.
                  break;
              }

              len -= 6;
              reg_idx += 3;
              buf += 6;
              if ((op == &_imu_data_refresh) && (_fifo_remaining > 0)) {
                _fifo_remaining--;
                if (_fifo_remaining > 0) {
                  ret = BUSOP_CALLBACK_RECYCLE;
                }
                else {
                  _read_fifo_level();
                }
              }
            }
            break;

          default:
            break;
        }
        len--;
        reg_idx++;
        buf++;
      }
      break;
    default:
      break;
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
