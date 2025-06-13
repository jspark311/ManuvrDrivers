/*
File:   OPT4060.cpp
Author: J. Ian Lindsay
Date:   2025.04.28

*/

#include "OPT4060.h"


// Readability defines...
#define OPT4060_REG_COUNT  ((uint8_t) OPT4060Register::INVALID)


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

static const uint8_t effective_mantissa_bits(const OPT4060DataRate E) {
  switch (E) {
    case OPT4060DataRate::HZ_1667:     return 9;
    case OPT4060DataRate::HZ_1000:     return 10;
    case OPT4060DataRate::HZ_556:      return 11;
    case OPT4060DataRate::HZ_294:      return 12;
    case OPT4060DataRate::HZ_154:      return 13;
    case OPT4060DataRate::HZ_79:       return 14;
    case OPT4060DataRate::HZ_40:       return 15;
    case OPT4060DataRate::HZ_20:       return 16;
    case OPT4060DataRate::HZ_10:       return 17;
    case OPT4060DataRate::HZ_5:        return 18;
    case OPT4060DataRate::HZ_2P5:      return 19;
    case OPT4060DataRate::HZ_1P25:     return 20;
    default:  break;
  }
  return 20;
}



/** Static function to convert enum to string. */
const char* const OPT4060::odrStr(const OPT4060DataRate e) {
  switch (e) {
    case OPT4060DataRate::HZ_1667:   return "HZ_1667";
    case OPT4060DataRate::HZ_1000:   return "HZ_1000";
    case OPT4060DataRate::HZ_556:    return "HZ_556";
    case OPT4060DataRate::HZ_294:    return "HZ_294";
    case OPT4060DataRate::HZ_154:    return "HZ_154";
    case OPT4060DataRate::HZ_79:     return "HZ_79";
    case OPT4060DataRate::HZ_40:     return "HZ_40";
    case OPT4060DataRate::HZ_20:     return "HZ_20";
    case OPT4060DataRate::HZ_10:     return "HZ_10";
    case OPT4060DataRate::HZ_5:      return "HZ_5";
    case OPT4060DataRate::HZ_2P5:    return "HZ_2P5";
    case OPT4060DataRate::HZ_1P25:   return "HZ_1P25";
    default:                         return "INVALID";
  }
}


/** Static function to convert enum to string. */
const char* const OPT4060::chanStr(const OPT4060Channel e) {
  switch (e) {
    case OPT4060Channel::RED:    return "RED";
    case OPT4060Channel::GREEN:  return "GREEN";
    case OPT4060Channel::BLUE:   return "BLUE";
    case OPT4060Channel::WHITE:  return "WHITE";
    default:                     return "INVALID";
  }
}

/** Static function to convert enum to string. */
const char* const OPT4060::modeStr(const OPT4060Mode e) {
  switch (e) {
    case OPT4060Mode::POWER_DOWN:   return "POWER_DOWN";
    case OPT4060Mode::AR_ONESHOT:   return "AR_ONESHOT";
    case OPT4060Mode::ONESHOT:      return "ONESHOT";
    case OPT4060Mode::CONTINUOUS:   return "CONTINUOUS";
    default:                        return "INVALID";
  }
}

/** Static function to convert enum to string. */
const char* const OPT4060::pinModeStr(const OPT4060IntPin e) {
  switch (e) {
    case OPT4060IntPin::ALL_CONV:       return "ALL_CONV";
    case OPT4060IntPin::ALL_CONV_INV:   return "ALL_CONV_INV";
    case OPT4060IntPin::CHAN_CONV:      return "CHAN_CONV";
    case OPT4060IntPin::CHAN_CONV_INV:  return "CHAN_CONV_INV";
    case OPT4060IntPin::TRIGGER:        return "TRIGGER";
    default:                            return "NONE";
  }
}

/** Static function to convert enum to string. */
const char* const OPT4060::regStr(const OPT4060Register e) {
  switch (e) {
    case OPT4060Register::CHAN0_MSW:  return "CHAN0_MSW";
    case OPT4060Register::CHAN0_LSW:  return "CHAN0_LSW";
    case OPT4060Register::CHAN1_MSW:  return "CHAN1_MSW";
    case OPT4060Register::CHAN1_LSW:  return "CHAN1_LSW";
    case OPT4060Register::CHAN2_MSW:  return "CHAN2_MSW";
    case OPT4060Register::CHAN2_LSW:  return "CHAN2_LSW";
    case OPT4060Register::CHAN3_MSW:  return "CHAN3_MSW";
    case OPT4060Register::CHAN3_LSW:  return "CHAN3_LSW";
    case OPT4060Register::THRESH_0:   return "THRESH_0";
    case OPT4060Register::THRESH_1:   return "THRESH_1";
    case OPT4060Register::CONFIG_0:   return "CONFIG_0";
    case OPT4060Register::CONFIG_1:   return "CONFIG_1";
    case OPT4060Register::CONFIG_2:   return "CONFIG_2";
    case OPT4060Register::DEV_ID:     return "DEV_ID";
    default:                          return "INVALID";
  }
}

/** Static function to convert enum to string. */
const char* const OPT4060::rangeStr(const OPT4060DynRange e) {
  switch (e) {
    case OPT4060DynRange::LUX_2200:    return "LUX_2200";
    case OPT4060DynRange::LUX_4500:    return "LUX_4500";
    case OPT4060DynRange::LUX_9000:    return "LUX_9000";
    case OPT4060DynRange::LUX_18000:   return "LUX_18000";
    case OPT4060DynRange::LUX_36000:   return "LUX_36000";
    case OPT4060DynRange::LUX_72000:   return "LUX_72000";
    case OPT4060DynRange::LUX_144000:  return "LUX_144000";
    case OPT4060DynRange::AUTORANGE:   return "AUTORANGE";
    default:                           return "INVALID";
  }
}


const uint32_t OPT4060::data_period_us(const OPT4060DataRate e) {
  switch (e) {
    case OPT4060DataRate::HZ_1667:   return 600;
    case OPT4060DataRate::HZ_1000:   return 1000;
    case OPT4060DataRate::HZ_556:    return 1800;
    case OPT4060DataRate::HZ_294:    return 3400;
    case OPT4060DataRate::HZ_154:    return 6500;
    case OPT4060DataRate::HZ_79:     return 12700;
    case OPT4060DataRate::HZ_40:     return 25000;
    case OPT4060DataRate::HZ_20:     return 50000;
    case OPT4060DataRate::HZ_10:     return 100000;
    case OPT4060DataRate::HZ_5:      return 200000;
    case OPT4060DataRate::HZ_2P5:    return 400000;
    case OPT4060DataRate::HZ_1P25:   return 800000;
    default:                         return 0;
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

OPT4060::OPT4060(const OPT4060Opts o, I2CAdapter* bus) : I2CDevice(o.ADDR, bus), _opts(o),
  _busop_trig_irq(BusOpcode::RX, this),
  _busop_chan_refresh(BusOpcode::RX, this) {}

OPT4060::~OPT4060() {}


/**
*
* @param b is an optional reference to the bus adapter that will handle this driver.
* @return 0 on success.
*/
int8_t OPT4060::init(I2CAdapter* b) {
  _opt4060_clear_flag(OPT4060_FLAG_INIT_MASK | OPT4060_FLAG_IO_IN_FLIGHT);
  int8_t ret = -1;
  if (nullptr != b) {
    _bus = b;
  }
  if (nullptr != _bus) {
    // Reading channels happens often enough that we just have our own local op
    //   for doing it. We also use this for concurrency control.
    // Reads directly to the shadow space.
    _busop_chan_refresh.dev_addr = _dev_addr;
    _busop_chan_refresh.sub_addr = (uint16_t) OPT4060Register::CHAN0_MSW;
    _busop_chan_refresh.setAdapter(_bus);
    _busop_chan_refresh.shouldReap(false);
    _busop_chan_refresh.setBuffer(&_shadows[(uint8_t) OPT4060Register::CHAN0_MSW], 16);

    // This busop is used to service the INT pin feature. If the INT pin is
    //   present and set as a device output, it will read the status register
    //   when the pin state change happens. If INT is NOT a device input, this
    //   busop will be setup as a TX write to the register responsible for
    //   soft-triggering in ONESHOT modes.
    _busop_trig_irq.dev_addr = _dev_addr;
    _busop_trig_irq.sub_addr = (uint8_t) OPT4060Register::CONFIG_2;
    _busop_trig_irq.setAdapter(_bus);
    _busop_trig_irq.shouldReap(false);
    _busop_trig_irq.setBuffer(&_shadows[(uint8_t) OPT4060Register::CONFIG_2], 2);

    ret = _read_registers(OPT4060Register::DEV_ID, 1);
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
int8_t OPT4060::poll() {
  int8_t ret = -3;
  if (initialized()) {
    ret = 0;
    if (_need_to_read()) {
      ret = -1;
      if (0 == _read_channels()) {
        ret = 1;
      }
    }

    if (255 != _opts.ALRT_PIN) {
      // TODO: Read pin.
    }
    else {
      // TODO: Read status register.
    }
  }
  return ret;
}


float OPT4060::channelValue(OPT4060Channel chan) {
  float ret = 0.0f;
  switch (chan) {
    case OPT4060Channel::RED:
    case OPT4060Channel::GREEN:
    case OPT4060Channel::BLUE:
    case OPT4060Channel::WHITE:
      {
        uint16_t msw_val = _get_shadow_value((OPT4060Register) ((uint8_t) OPT4060Register::CHAN0_MSW + ((uint8_t) chan << 1)));
        uint16_t lsw_val = _get_shadow_value((OPT4060Register) ((uint8_t) OPT4060Register::CHAN0_LSW + ((uint8_t) chan << 1)));
        uint16_t exponent  = (msw_val >> 12);
        uint32_t mantissa  = ((uint32_t) (msw_val & 0x0FFF) << 8) | (uint32_t)(lsw_val >> 8);
        uint32_t adc_count = mantissa << exponent;
        ret = adc_count * 0.00215f;
      }
      _opt4060_clear_flag(OPT4060_FLAG_FRESH_VALUE);
      break;
    default:
      break;
  }
  return ret;
}


int8_t OPT4060::conversionRate(OPT4060DataRate r, bool dispatch_io) {
  int8_t ret = _set_conversion_rate(r);
  if (dispatch_io & (0 == ret)) {
    ret = _write_registers(OPT4060Register::CONFIG_0, 1);
  }
  return ret;
}

int8_t OPT4060::mode(OPT4060Mode val, bool dispatch_io) {
  int8_t ret = _set_mode(val);
  if (dispatch_io & (0 == ret)) {
    ret = _write_registers(OPT4060Register::CONFIG_0, 1);
  }
  return ret;
}
int8_t OPT4060::dynamicRange(OPT4060DynRange val, bool dispatch_io) {
  int8_t ret = _set_dynamic_range(val);
  if (dispatch_io & (0 == ret)) {
    ret = _write_registers(OPT4060Register::CONFIG_0, 1);
  }
  return ret;
}


OPT4060DataRate OPT4060::conversionRate() {
  OPT4060DataRate ret = OPT4060DataRate::INVALID;
  if (_pin_regs_known()) {
    const uint16_t MASKED_VAL = _get_shadow_value(OPT4060Register::CONFIG_0) & 0x03C0;
    ret = (OPT4060DataRate) (MASKED_VAL >> 6);
  }
  return ret;
}


OPT4060Mode OPT4060::mode() {
  OPT4060Mode ret = OPT4060Mode::INVALID;
  if (_pin_regs_known()) {
    const uint16_t MASKED_VAL = _get_shadow_value(OPT4060Register::CONFIG_0) & 0x0030;
    ret = (OPT4060Mode) (MASKED_VAL >> 4);
  }
  return ret;
}

OPT4060DynRange OPT4060::dynamicRange() {
  OPT4060DynRange ret = OPT4060DynRange::INVALID;
  if (_pin_regs_known()) {
    const uint16_t MASKED_VAL = _get_shadow_value(OPT4060Register::CONFIG_0) & 0x3C00;
    OPT4060DynRange val = (OPT4060DynRange) (MASKED_VAL >> 10);
    switch (val) {
      case OPT4060DynRange::LUX_2200:
      case OPT4060DynRange::LUX_4500:
      case OPT4060DynRange::LUX_9000:
      case OPT4060DynRange::LUX_18000:
      case OPT4060DynRange::LUX_36000:
      case OPT4060DynRange::LUX_72000:
      case OPT4060DynRange::LUX_144000:
      case OPT4060DynRange::AUTORANGE:
        ret = val;
        break;
      default:
        break;
    }
  }
  return ret;
}



OPT4060IntPin OPT4060::optPinMode() {
  OPT4060IntPin ret = OPT4060IntPin::NONE;
  if (_pin_regs_known()) {
    const uint16_t MASKED_VAL0 = _get_shadow_value(OPT4060Register::CONFIG_0) & 0x0004;
    const uint16_t MASKED_VAL1 = _get_shadow_value(OPT4060Register::CONFIG_1) & 0x0038;
    OPT4060IntPin val = (OPT4060IntPin) ((MASKED_VAL1 >> 2) + (MASKED_VAL0 << 1));
    switch (val) {
      case OPT4060IntPin::NONE:
      case OPT4060IntPin::ALL_CONV:
      case OPT4060IntPin::ALL_CONV_INV:
      case OPT4060IntPin::CHAN_CONV:
      case OPT4060IntPin::CHAN_CONV_INV:
      case OPT4060IntPin::TRIGGER:
        ret = val;
        break;
      default:
        break;
    }
  }
  return ret;
}


uint32_t OPT4060::colorValue(const ImgBufferFormat FMT) {
  uint32_t ret = 0;
  switch (FMT) {
    case ImgBufferFormat::R8_G8_B8:
      ret += ((uint32_t) (0x00FF0000 * _normalized[0]) & 0x00FF0000);
      ret += ((uint32_t) (0x0000FF00 * _normalized[1]) & 0x0000FF00);
      ret += ((uint32_t) (0x000000FF * _normalized[2]) & 0x000000FF);
      break;
  }
  _opt4060_clear_flag(OPT4060_FLAG_FRESH_VALUE);
  return ret;
}


int8_t OPT4060::colorValue(Vector3<float>* rgb_vect) {
  if (dataReady()) {
    rgb_vect->set(_normalized[0], _normalized[1], _normalized[2]);
    _opt4060_clear_flag(OPT4060_FLAG_FRESH_VALUE);
    return 0;
  }
  return -1;
}


int8_t OPT4060::colorValue(Vector3<uint8_t>* rgb_vect) {
  if (dataReady()) {
    rgb_vect->set(
      (_normalized[0] * 0xFF),
      (_normalized[1] * 0xFF),
      (_normalized[2] * 0xFF)
    );
    _opt4060_clear_flag(OPT4060_FLAG_FRESH_VALUE);
    return 0;
  }
  return -1;
}




/*******************************************************************************
* Register access and abstraction functions
* The part has 16-bit registers that are big-endian.
*******************************************************************************/

/**
* Reads the given register from the hardware.
*
* @param reg is the register address
* @param len is the number of registers (not bytes) to read.
* @return 0 on success
*        -1 if the device isn't yet found
*        -2 if there is already I/O in flight
*        -3 I/O failure
*/
int8_t OPT4060::_read_registers(OPT4060Register reg, uint8_t len) {
  int8_t ret = -1;
  I2CBusOp* op = _bus->new_op(BusOpcode::RX, this);
  if (nullptr != op) {
    ret--;
    op->dev_addr = _dev_addr;
    op->sub_addr = (int16_t) reg;
    op->setBuffer(&_shadows[(uint8_t) reg << 1], (len << 1));
    if (0 == queue_io_job(op)) {
      //_opt4060_set_flag(OPT4060_FLAG_IO_IN_FLIGHT);
      ret = 0;
    }
  }
  return ret;
}


/**
* Writes the given register with the given value to the hardware.
*
* @param reg is the register address
* @param len is the number of registers (not bytes) to write.
* @return 0 on success
*        -1 if the device isn't yet found
*        -2 if there is already I/O in flight
*        -3 on BusOp allocation failure
*        -4 I/O rejection
*/
int8_t OPT4060::_write_registers(OPT4060Register reg, uint8_t len) {
  int8_t ret = -1;
  if (devFound()) {
    ret--;
    //if (!_opt4060_flag(OPT4060_FLAG_IO_IN_FLIGHT)) {
      ret--;
      I2CBusOp* op = _bus->new_op(BusOpcode::TX, this);
      if (nullptr != op) {
        ret--;
        op->dev_addr = _dev_addr;
        op->sub_addr = (int16_t) reg;
        op->setBuffer(&_shadows[(uint8_t) reg << 1], (len << 1));
        if (0 == queue_io_job(op)) {
          //_opt4060_set_flag(OPT4060_FLAG_IO_IN_FLIGHT);
          ret = 0;
        }
      }
    //}
  }
  return ret;
}


/**
* Get the register's shadow value.
* This function is and endianness control choke-point.
*
* @param reg is the register address
* @return the endian-native representation of the given register.
*/
uint16_t OPT4060::_get_shadow_value(OPT4060Register reg) {
  const uint8_t REG_BASE = (((uint8_t) reg) << 1);
  const uint8_t msb = _shadows[REG_BASE + 0];
  const uint8_t lsb = _shadows[REG_BASE + 1];
  return (((uint16_t) msb << 8) | lsb);
}

/**
* Set the register's shadow value.
* This function is and endianness control choke-point.
*
* @param reg is the register address
* @param val is the desired value in program-native endian order.
*/
void OPT4060::_set_shadow_value(OPT4060Register reg, uint16_t val) {
  const uint8_t REG_BASE = (((uint8_t) reg) << 1);
  // This code should work on both big and little-endian CPUs. The register
  //   arrangement is big.
  _shadows[REG_BASE + 0] = (val >> 8);
  _shadows[REG_BASE + 1] = (val & 0x00FF);
}


/**
* Resets the register shadows to their PoR values.
*/
void OPT4060::_reset_register_values() {
  for (uint8_t i = 0; i < (OPT4060_REG_COUNT << 1); i++) {
    _shadows[i] = 0;
  }
}


int8_t OPT4060::_set_conversion_rate(OPT4060DataRate r) {
  int8_t ret = -1;
  uint8_t rate = (uint8_t) r; // Make sure rate is not set higher than 3.
  if (rate < 12) {
    const uint16_t MASKED_VAL = _get_shadow_value(OPT4060Register::CONFIG_0) & ~0x03C0;
    // Read current configuration register value
    uint16_t new_val = (MASKED_VAL | (rate << 6));  // Shift in new conversion rate
    _set_shadow_value(OPT4060Register::CONFIG_0, new_val);
    ret = 0;
  }
  return ret;
}


int8_t OPT4060::_set_mode(OPT4060Mode val) {
  int8_t ret = -1;
  switch (val) {
    case OPT4060Mode::POWER_DOWN:
    case OPT4060Mode::AR_ONESHOT:
    case OPT4060Mode::ONESHOT:
    case OPT4060Mode::CONTINUOUS:
      {
        const uint16_t MASKED_VAL = _get_shadow_value(OPT4060Register::CONFIG_0) & ~0x0030;
        // Read current configuration register value
        uint16_t new_val = (MASKED_VAL | ((uint16_t) val << 4));  // Shift in new conversion range
        _set_shadow_value(OPT4060Register::CONFIG_0, new_val);
        ret = 0;
      }
      break;
    default:
      break;
  }
  return ret;
}


int8_t OPT4060::_set_dynamic_range(OPT4060DynRange r) {
  int8_t ret = -1;
  switch (r) {
    case OPT4060DynRange::LUX_2200:
    case OPT4060DynRange::LUX_4500:
    case OPT4060DynRange::LUX_9000:
    case OPT4060DynRange::LUX_18000:
    case OPT4060DynRange::LUX_36000:
    case OPT4060DynRange::LUX_72000:
    case OPT4060DynRange::LUX_144000:
    case OPT4060DynRange::AUTORANGE:
      {
        const uint16_t MASKED_VAL = _get_shadow_value(OPT4060Register::CONFIG_0) & ~0x3C00;
        // Read current configuration register value
        uint16_t new_val = (MASKED_VAL | ((uint16_t) r << 10));  // Shift in new conversion range
        _set_shadow_value(OPT4060Register::CONFIG_0, new_val);
        ret = 0;
      }
      break;
    default:
      break;
  }
  return ret;
}


// This function assumes that the channels are worth reading, and only enforces
//   concurrency limitations on reading.
int8_t OPT4060::_read_channels() {
  // Triggering a conversion if in one-shot or power-down mode.
  // Triggering via INT pin.
  // Trigger via register write.
  int8_t ret = -1;
  if (_busop_chan_refresh.isIdle()) {
    _last_read_us = micros();
    ret = _bus->queue_io_job(&_busop_chan_refresh);
    _opt4060_clear_flag(OPT4060_FLAG_FRESH_VALUE);
  }
  return ret;
}


/**
* Internal state check function to decide if the sensor has data that is worth
*   reading.
*/
bool OPT4060::_need_to_read() {
  bool ret = false;
  if (initialized()) {
    switch (mode()) {
      case OPT4060Mode::POWER_DOWN:
        // TODO: Issue the bus command to power up and proceed as if everything is
        //   running. As long as the bus traffic for this driver stays in proper
        //   order at dispatch, it will remain in that order on the wire.
        return false;

      case OPT4060Mode::AR_ONESHOT:
      case OPT4060Mode::ONESHOT:
        // The sensor is already in one-shot mode, so we need to decide how to
        //   trigger a conversion, if there isn't already one happening.
        ret = (0 == _trigger_conversion());
        break;

      case OPT4060Mode::CONTINUOUS:
        if (_opts.haveAlertPin()) {
          // No action is required here. We just wait for the next polling cycle to
          //   notice that the conversion (presumed running) is finished, and read
          //   the registers in case it is.
        }
        else {
          // Otherwise, we'll need to check how long its been since a read happened.
          ret = _chan_read_timer.expired();
        }
        break;
      default:
        break;
    }
  }
  return ret;
}


// This function assumes that the chip mode is appropriate, and only enforces
//   timing and implements the actual trigger.
int8_t OPT4060::_trigger_conversion() {
  int8_t ret = -1;
  if (_opts.haveTriggerPin()) {
    _last_trig_us = micros();
    setPin(_opts.ALRT_PIN, true);
    setPin(_opts.ALRT_PIN, false);
    ret = 0;
  }
  else if (!_io_in_flight()) {
    if (_have_trig_busop()) {
      if (_busop_trig_irq.isIdle()) {
        ret = _bus->queue_io_job(&_busop_trig_irq);
      }
    }
  }

  if (ret) {
    // NOTE: This would be a good place to profile for performance.
    _chan_read_timer.reset();
  }

  return ret;
}


/**
* The sensor's response is reported logarithmically. This function applies
*   corrective adjustments to the shadow register values, and converts the
*   results into uint32_t. These are then stored in the private _lux
*   member for retreival.
*/
int8_t OPT4060::_normalize_data() {
  static const float ADC_SCALARS[4] = {2.4f, 1.0f, 1.3f, 1.0f};
  static const float RES_TABLE[] = {
    2.15f,    4.30f,    8.6f,  17.2f,  34.4f,  68.8f,
    137.6f,  275.2f,  550.4f,   1.1f,   2.2f,   4.4f,
      8.8f,   17.6f,   35.2f, 70.45f, 140.9f, 281.8f
  };
  const uint8_t  EFCTV_BITS = effective_mantissa_bits(conversionRate());
  const uint32_t EFCTV_MASK = ((0xFFFFF << (20-EFCTV_BITS)) & 0xFFFFF);
  uint32_t adc_codes[4] = {0};
  float    rgb[4] = {0};
  for (uint8_t i = 0; i < 4; i++) {
    const OPT4060Register REG_MSB_ID = (OPT4060Register) (i << 1);
    const OPT4060Register REG_LSB_ID = (OPT4060Register) ((i << 1)+1);
    const uint16_t MSW_VAL = _get_shadow_value(REG_MSB_ID);
    const uint16_t LSW_VAL = _get_shadow_value(REG_LSB_ID);
    const uint8_t  EXPONENT = (MSW_VAL >> 12);
    const uint32_t MANTISSA = ((uint32_t) (MSW_VAL & 0x0FFF) << 8) | (LSW_VAL >> 8);
    adc_codes[i] = ((MANTISSA & EFCTV_MASK) << EXPONENT);
    _effective_res[i] = RES_TABLE[(20-EFCTV_BITS)+(EXPONENT % 6)];
    rgb[i] = adc_codes[i] * ADC_SCALARS[i];
  }

  // Find color
  const float COLOR_SUM = (rgb[0] + rgb[1] + rgb[2]);
  for (uint8_t i = 0; i < 4; i++) {
    _millilux[i] = adc_codes[i] * 2150;   // Find per-channel lux.
    _normalized[i] = ((float) rgb[i] / COLOR_SUM);
  }
  return 0;
}


/*
* Idempotently setup the low-level pin details. Because there is a
*   bi-directional I/O pin involved, the device must be configured prior to
*   calling this function if there is to be any chance of success.
*/
int8_t OPT4060::_ll_pin_init() {
  int8_t ret = 0;
  if (!_opt4060_flag(OPT4060_FLAG_PINS_CONFIGURED)) {
    if (_pin_regs_known()) {
      if (_opts.pinDefined()) {
        switch (optPinMode()) {
          case OPT4060IntPin::ALL_CONV_INV:
          case OPT4060IntPin::CHAN_CONV_INV:
            // TODO: SetFxn();
          case OPT4060IntPin::ALL_CONV:
          case OPT4060IntPin::CHAN_CONV:
            pinMode(_opts.ALRT_PIN, GPIOMode::INPUT_PULLUP);  // TODO: Should be an option.
            break;

          case OPT4060IntPin::TRIGGER:
            //setPin(_opts.ALRT_PIN, true);
            //pinMode(_opts.ALRT_PIN, GPIOMode::OUTPUT);  // TODO: Need to write config first.
            break;

          case OPT4060IntPin::NONE:
          default:
            break;
        }
      }
      _opt4060_set_flag(OPT4060_FLAG_PINS_CONFIGURED);
    }
  }
  return ret;
}



/*
* Write any configuration to the hardware.
*/
int8_t OPT4060::_impart_config() {
  int8_t ret = -1;
  bool need_config_write = false;

  if (_opts.rate != conversionRate()) {
    if (0 == _set_conversion_rate(_opts.rate)) {
      need_config_write = true;
    }
    else {
      c3p_log(LOG_LEV_ERROR, "OPT4060", "conversionRate(%s) failed.", odrStr(conversionRate()));
    }
  }
  if (_opts.range != dynamicRange()) {
    if (0 == _set_dynamic_range(_opts.range)) {
      need_config_write = true;
    }
    else {
      c3p_log(LOG_LEV_ERROR, "OPT4060", "dynamicRange(%s) failed.", rangeStr(dynamicRange()));
    }
  }
  if (_opts.mode != mode()) {
    if (0 == _set_mode(_opts.mode)) {
      need_config_write = true;
    }
    else {
      c3p_log(LOG_LEV_ERROR, "OPT4060", "mode(%s) failed.", modeStr(mode()));
    }
  }

  if (_opts.pinDefined()) {
    const OPT4060IntPin EXISTING_PIN_MODE = optPinMode();
    if (_opts.PIN_MODE != EXISTING_PIN_MODE) {
      const uint16_t PMODE = (uint16_t) _opts.PIN_MODE;
      const uint16_t MASKED_VAL0 = _get_shadow_value(OPT4060Register::CONFIG_0) & ~0x0004;
      const uint16_t MASKED_VAL1 = _get_shadow_value(OPT4060Register::CONFIG_1) & ~0x0038;
      need_config_write = true;
      //_set_shadow_value(OPT4060Register::CONFIG_0, (MASKED_VAL0 | ((PMODE & 0x0008) >> 1)));
      //_set_shadow_value(OPT4060Register::CONFIG_1, (MASKED_VAL1 | ((PMODE & 0x0007) << 2)));
    }
  }

  if (need_config_write) {
    c3p_log(LOG_LEV_INFO, "OPT4060", "Writing config...");
    _write_registers(OPT4060Register::CONFIG_0, 1);
    _write_registers(OPT4060Register::CONFIG_1, 1);
    ret = _write_registers(OPT4060Register::CONFIG_2, 1);
  }
  else {
    ret = _ll_pin_init();
  }
  return ret;
}



/*******************************************************************************
* ___     _       _                      These members are mandatory overrides
*  |   / / \ o   | \  _     o  _  _      for implementing I/O callbacks. They
* _|_ /  \_/ o   |_/ (/_ \/ | (_ (/_     are also implemented by Adapters.
*******************************************************************************/

/* Transfers always permitted. */
int8_t OPT4060::io_op_callahead(BusOp* _op) {   return 0;   }


/*
* Register I/O calls back to this function for BOTH devices (MAG/IMU). So we
*   split the function up into two halves in private scope in the superclass.
* Bus operations that call back with errors are ignored.
*/
int8_t OPT4060::io_op_callback(BusOp* _op) {
  I2CBusOp* op  = (I2CBusOp*) _op;
  int8_t    ret = BUSOP_CALLBACK_NOMINAL;

  if (!op->hasFault()) {
    if (&_busop_chan_refresh == op) {
      _chan_read_timer.reset();
      _opt4060_set_flag(OPT4060_FLAG_FRESH_VALUE);
      if (initialized()) {
        _normalize_data();
      }
      return ret;
    }
    uint32_t len = (op->bufferLen() >> 1);
    const uint8_t SUB_ADDR = (uint8_t) op->sub_addr;
    if ((SUB_ADDR + len) <= (uint32_t) OPT4060Register::INVALID) {
      const BusOpcode BUS_OPCODE = op->get_opcode();
      for (uint8_t reg_idx = SUB_ADDR; reg_idx < (SUB_ADDR+len); reg_idx++) {
        uint16_t reg_val = _get_shadow_value((OPT4060Register) reg_idx);
        switch (BUS_OPCODE) {

          case BusOpcode::TX:
            switch ((OPT4060Register) reg_idx) {
              case OPT4060Register::THRESH_0:     //
              case OPT4060Register::THRESH_1:     //
                break;
              case OPT4060Register::CONFIG_0:     //
                _opt4060_set_flag(OPT4060_FLAG_CONFIG_0_KNOWN);
                _chan_read_timer.reset(data_period_us(conversionRate()));
                break;
              case OPT4060Register::CONFIG_1:     //
                _opt4060_set_flag(OPT4060_FLAG_CONFIG_1_KNOWN);
                break;
              case OPT4060Register::CONFIG_2:     //
                _opt4060_set_flag(OPT4060_FLAG_CONFIG_2_KNOWN);
                break;
              default:
                break;
            }
            break;

          case BusOpcode::RX:
            switch ((OPT4060Register) reg_idx) {
              case OPT4060Register::CONFIG_0:     //
                _opt4060_set_flag(OPT4060_FLAG_CONFIG_0_KNOWN);
                _chan_read_timer.reset(data_period_us(conversionRate()));
                break;
              case OPT4060Register::CONFIG_1:     //
                _opt4060_set_flag(OPT4060_FLAG_CONFIG_1_KNOWN);
                break;
              case OPT4060Register::CONFIG_2:     //
                _opt4060_set_flag(OPT4060_FLAG_CONFIG_2_KNOWN);
                if (_pin_regs_known()) {
                  _impart_config();
                }
                break;

              case OPT4060Register::DEV_ID:       // Read-only
                if (0x0821 == reg_val) {
                  _read_registers(OPT4060Register::THRESH_0, 2);
                  _read_registers(OPT4060Register::CONFIG_0, 3);
                }
                break;

              default:
                break;
            }
            break;

          default: break;
        }
      }
    }
  }
  return ret;
}


/*******************************************************************************
* Console callback
* These are built-in handlers for using this instance via a console.
*******************************************************************************/

/*
*
*/
void OPT4060::printDebug(StringBuilder* output) {
  StringBuilder::styleHeader1(output, "OPT4060");
  output->concatf("\tAlert Pin:   %u\n", _opts.ALRT_PIN);
  output->concatf("\tPins setup:  %c\n", _opt4060_flag(OPT4060_FLAG_PINS_CONFIGURED) ? 'y' : 'n');
  output->concatf("\tDev found:   %c\n", devFound() ? 'y' : 'n');
  if (devFound()) {
    output->concatf("\tInitialized: %c\n", initialized() ? 'y' : 'n');
    output->concatf("\tIO inflight: %c\n", _opt4060_flag(OPT4060_FLAG_IO_IN_FLIGHT) ? 'y' : 'n');
    output->concatf("\tOp mode:       %s\n", modeStr(mode()));
    output->concatf("\tSample rate:   %s\n", odrStr(conversionRate()));
    output->concatf("\tDynamic range: %s\n", rangeStr(dynamicRange()));
    output->concatf("\tPin mode:      %s\n", pinModeStr(optPinMode()));
    if (initialized()) {
      output->concatf("\tLast trig:   %uus ago.\n", micros_since(_last_trig_us));
      output->concatf("\tLast read:   %uus ago.\n", micros_since(_last_read_us));
      output->concatf("\tRead period: %u us %s\n", _chan_read_timer.period(), (_chan_read_timer.expired() ? "(expired)" : ""));
    }
    else {
      output->concatf("\tConf regs known:  %c\n", _all_regs_known() ? 'y' : 'n');
      output->concatf("\tPin regs known:   %c\n", _pin_regs_known() ? 'y' : 'n');
    }
  }
  output->concat("\n");
}



void OPT4060::printRegs(StringBuilder* output) {
  for (uint8_t i = 0; i < OPT4060_REG_COUNT; i++) {
    const OPT4060Register REG_ID = (OPT4060Register) i;
    const uint16_t REG_VAL = _get_shadow_value(REG_ID);
    output->concatf(
      "\t[%2u]  0x%04x\n",
      i,
      REG_VAL
    );
  }
}


void OPT4060::printChannelValues(StringBuilder* output, int8_t chan) {
  const int8_t CHAN_START = ((0 <= chan) & ((int8_t) OPT4060Channel::INVALID > chan)) ? chan : 0;
  const int8_t CHAN_STOP  = ((0 <= chan) & ((int8_t) OPT4060Channel::INVALID > chan)) ? (chan+1) : (int8_t) OPT4060Channel::INVALID;

  //output->concat("\tColor:  \033[38;2;");
  //output->concatf(
  //  "%u;%u;%um",
  //  (_normalized[0] * 0xFF),
  //  (_normalized[1] * 0xFF),
  //  (_normalized[2] * 0xFF)
  //);
  output->concatf("\tColor:  #%06x\n\tChan    lux       Normal \n", colorValue(ImgBufferFormat::R8_G8_B8));
  for (uint8_t i = CHAN_START; i < CHAN_STOP; i++) {
    output->concatf("\t%2u:    %.3f (+/-%.4f)     %.3f\n",
      i,
      (_millilux[i] / 1000.0f),
      _effective_res[i],
      _normalized[i]
    );
  }
}


/**
* @page console-handlers
* @section opt4060-tools OPT4060 tools
*
* This is the console handler for using the OPT4060 light sensor driver. If invoked without
*   arguments, it will print channel values, as are being observed by hardware.
*
* @subsection cmd-actions Actions
*
* Action    | Description | Additional arguments
* --------- | ----------- | --------------------
* `init`    | Manually invoke the driver's `init()` function. | None
* `reset`   | Manually invoke the driver's `reset()` function. | None
*/
int8_t OPT4060::console_handler(StringBuilder* text_return, StringBuilder* args) {
  int   ret  = 0;
  char* cmd  = args->position_trimmed(0);
  int   arg0 = (1 < args->count()) ? args->position_as_int(1) : -1;

  if (0 == StringBuilder::strcasecmp(cmd, "info")) {
    printDebug(text_return);
  }
  else if (0 == StringBuilder::strcasecmp(cmd, "regs")) {
    printRegs(text_return);
  }
  else if (0 == StringBuilder::strcasecmp(cmd, "config")) {
    text_return->concatf("OPT4060 _impart_config() returns %d.\n", _impart_config());
  }
  else if (0 == StringBuilder::strcasecmp(cmd, "pins")) {
    text_return->concatf("OPT4060 _ll_pin_init() returns %d.\n", _ll_pin_init());
  }
  else if (0 == StringBuilder::strcasecmp(cmd, "read")) {
    text_return->concatf("OPT4060 _read_channels() returns %d.\n", _read_channels());
  }

  else if (0 == StringBuilder::strcasecmp(cmd, "init")) {
    text_return->concatf("OPT4060 init() returns %d.\n", init(_bus));
  }
  else if (0 == StringBuilder::strcasecmp(cmd, "refresh")) {
    _read_registers(OPT4060Register::THRESH_0, 5);
  }


  else if (0 == StringBuilder::strcasecmp(cmd, "range")) {
    switch ((OPT4060DynRange) arg0) {
      case OPT4060DynRange::LUX_2200:
      case OPT4060DynRange::LUX_4500:
      case OPT4060DynRange::LUX_9000:
      case OPT4060DynRange::LUX_18000:
      case OPT4060DynRange::LUX_36000:
      case OPT4060DynRange::LUX_72000:
      case OPT4060DynRange::LUX_144000:
      case OPT4060DynRange::AUTORANGE:
        text_return->concatf("OPT4060 _set_dynamic_range() returns %d.\n", _set_dynamic_range((OPT4060DynRange) arg0));
        _write_registers(OPT4060Register::CONFIG_0, 1);
        break;
      default:
        text_return->concatf("Valid values for dynamic range are [0, 6] or 12.\n");
        break;
    }
  }
  else if (0 == StringBuilder::strcasecmp(cmd, "rate")) {
    switch ((OPT4060DataRate) arg0) {
      case OPT4060DataRate::HZ_1667:
      case OPT4060DataRate::HZ_1000:
      case OPT4060DataRate::HZ_556:
      case OPT4060DataRate::HZ_294:
      case OPT4060DataRate::HZ_154:
      case OPT4060DataRate::HZ_79:
      case OPT4060DataRate::HZ_40:
      case OPT4060DataRate::HZ_20:
      case OPT4060DataRate::HZ_10:
      case OPT4060DataRate::HZ_5:
      case OPT4060DataRate::HZ_2P5:
      case OPT4060DataRate::HZ_1P25:
        text_return->concatf("OPT4060 _set_conversion_rate() returns %d.\n", _set_conversion_rate((OPT4060DataRate) arg0));
        _write_registers(OPT4060Register::CONFIG_0, 1);
        break;
      default:
        text_return->concatf("Valid values for data rate are [0, 11].\n");
        break;
    }
  }


  else if (0 == StringBuilder::strcasecmp(cmd, "poll")) {
    text_return->concatf("OPT4060 poll() returns %d.\n", poll());
  }
  else {
    printChannelValues(text_return, arg0);
  }

  return ret;
}
