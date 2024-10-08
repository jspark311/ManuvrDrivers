/*
File:   MCP356x_Regs.cpp
Author: J. Ian Lindsay

This source file is meant to contain the low-level I/O and register access.
*/

#include "MCP356x.h"

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
/* Register widths in bytes. Index corresponds directly to register address. */
static const uint8_t MCP356x_reg_width[16] = {
  1, 1, 1, 1, 1, 1, 1, 3, 3, 3, 3, 3, 1, 1, 2, 2
};

const uint16_t MCP356x::OSR1_VALUES[16] = {
  1, 1, 1, 1, 1, 2, 4, 8, 16, 32, 40, 48, 80, 96, 160, 192
};

const uint16_t MCP356x::OSR3_VALUES[16] = {
  32, 64, 128, 256, 512, 512, 512, 512, 512, 512, 512, 512, 512, 512, 512, 512
};

static const float ADC_GAIN_VALUES[8] = {
  0.33, 1.0, 2.0, 4.0, 8.0, 16.0, 32.0, 64.0
};

const EnumDef<MCP356xState> _STATE_LIST[] = {
  { MCP356xState::UNINIT,      "UNINIT"},      // init() has never been called.
  { MCP356xState::PRE_INIT,    "PRE_INIT"},    // Pin control is being established.
  { MCP356xState::RESETTING,   "RESETTING"},   // Driver is resetting the ADC.
  { MCP356xState::DISCOVERY,   "DISCOVERY"},   // Driver is probing for the ADC.
  { MCP356xState::POST_INIT,   "POST_INIT"},   // The initial ADC configuration is being written.
  { MCP356xState::CLK_MEASURE, "CLK_MEASURE"}, // Driver is measuring the clock.
  { MCP356xState::CALIBRATION, "CALIBRATION"}, // The ADC is self-calibrating.
  { MCP356xState::USR_CONF,    "USR_CONF"},    // User config is being written.
  { MCP356xState::IDLE,        "IDLE"},        // Powered up and calibrated, but not reading.
  { MCP356xState::READING,     "READING"},     // Everything running, data collection proceeding.
  { MCP356xState::FAULT,       "FAULT"},       // State machine encountered something it couldn't cope with.
  { MCP356xState::INVALID,     "INVALID", (ENUM_WRAPPER_FLAG_CATCHALL)}  // FSM hygiene.
};
const EnumDefList<MCP356xState> MCP356x::_FSM_STATES(&_STATE_LIST[0], (sizeof(_STATE_LIST) / sizeof(_STATE_LIST[0])), "MCP356xState");


/*******************************************************************************
* High-level semantic breakouts for registers
* TODO: Some of these are yet to be buried in a private scope.
*******************************************************************************/
/**
* Causes a full refresh. Update our shadows with the state of the hardware
*   registers.
*
* @return
*    -2 on no bus adapter.
*    -1 on failure to read register.
*    0  on success.
*/
int8_t MCP356x::refresh() {
  int8_t  ret = 0;
  _flags.set(MCP356X_FLAG_REFRESH_CYCLE);
  ret = _read_register(MCP356xRegister::ADCDATA);
  return ret;
}


/**
* Runs the current temperature value through the temperature transfer function
*   to arrive at the die temperature.
*
* @return The calculated temperature in degrees C.
*/
float MCP356x::getTemperature() {
  int32_t t_lsb = value(MCP356xChannel::TEMP);
  float ret = 0.0;
  if (_flags.value(MCP356X_FLAG_3RD_ORDER_TEMP)) {
    const double k1 = 0.0000000000000271 * (t_lsb * t_lsb * t_lsb);
    const double k2 = -0.000000018 * (t_lsb * t_lsb);
    const double k3 = 0.0055 * t_lsb;
    const double k4 = -604.22;
    ret = k1 + k2 + k3 + k4;
  }
  else {
    ret = (0.001581D * t_lsb) - 324.27D;
  }
  return ret;
}


/**
* Sets the offset calibration for the ADC.
* Enables the offset feature if the value is non-zero.
*
* @param offset The desired offset for the data.
* @return
*    -2 on no bus adapter.
*    -1 if there was a problem writing one of the config registers.
*    0 if config was set successfully.
*/
int8_t MCP356x::setOffsetCalibration(int32_t offset) {
  uint32_t c3_val_cur = _get_shadow_value(MCP356xRegister::CONFIG3);
  uint32_t c3_val_new = (0 != offset) ? (c3_val_cur | 0x00000002) : (c3_val_cur & 0xFFFFFFFD);
  int8_t ret = 0;
  if (offset != (int32_t) _get_shadow_value(MCP356xRegister::OFFSETCAL)) {
    if (c3_val_new != c3_val_cur) {
      ret = _write_register(MCP356xRegister::CONFIG3, c3_val_new);
    }
    if (0 == ret) {
      ret = _write_register(MCP356xRegister::OFFSETCAL, (uint32_t) offset);
    }
  }
  return ret;
}


/**
* Sets the scale calibration for the ADC.
* Enables the scale feature if the value is non-zero.
*
* @param multiplier The desired scaler for the analog front-end.
* @return
*    -2 on no bus adapter.
*    -1 if there was a problem writing one of the config registers.
*    0 if config was set successfully.
*/
int8_t MCP356x::setGainCalibration(int32_t multiplier) {
  int8_t ret = _write_register(MCP356xRegister::GAINCAL, (uint32_t) multiplier);
  if (0 == ret) {
    uint32_t c_val = _get_shadow_value(MCP356xRegister::CONFIG3);
    c_val = (0 != multiplier) ? (c_val | 0x00000001) : (c_val & 0xFFFFFFFE);
    ret = _write_register(MCP356xRegister::CONFIG3, c_val);
  }
  return ret;
}


/**
* Application-facing accessor for the current gain.
*
* @return the enum for the current gain setting.
*/
MCP356xGain MCP356x::getGain() {
  return (MCP356xGain) ((_get_shadow_value(MCP356xRegister::CONFIG2) >> 3) & 0x07);
}

/**
* Application-facing accessor for the current source setting.
* This is basically used for burnout detection in external hardware.
*
* @return the enum for the present current source setting.
*/
MCP356xBiasCurrent MCP356x::getBiasCurrent() {
  return (MCP356xBiasCurrent) ((_get_shadow_value(MCP356xRegister::CONFIG0) & 0x0000000C) >> 2);
}

/**
* Application-facing accessor for the AMCLK prescaler.
*
* @return the enum for the present setting of the AMCLK prescaler.
*/
MCP356xAMCLKPrescaler MCP356x::getAMCLKPrescaler() {
  return (MCP356xAMCLKPrescaler) ((_get_shadow_value(MCP356xRegister::CONFIG1) & 0x000000C0) >> 6);
}

/**
* Application-facing accessor for the current oversampling ratio.
*
* @return the enum for the current oversampling ratio.
*/
MCP356xOversamplingRatio MCP356x::getOversamplingRatio() {
  return (MCP356xOversamplingRatio) ((_get_shadow_value(MCP356xRegister::CONFIG1) & 0x0000003C) >> 2);
}

/**
* Application-facing accessor for the current effective read mode.
*
* @return the enum for the present read mode setting.
*/
MCP356xMode MCP356x::readMode() {
  return (MCP356xMode)(_get_shadow_value(MCP356xRegister::CONFIG0) & 0x00000003);
}



/**
* Application-facing accessor for VREF selection, if available.
*
* @return true if Vref is using the internally generated value.
*/
bool MCP356x::usingInternalVref() {
  bool ret = false;
  if (_flags.value(MCP356X_FLAG_HAS_INTRNL_VREF)) {
    ret = (0 != (_get_shadow_value(MCP356xRegister::CONFIG0) & 0x00000040));
  }
  return ret;
}


/**
* Application-facing accessor for VREF selection, if available.
*
* @param x true will enable the internal Vref, if available.
* @return 0 on success, -1 on "not supported", or -2 on I/O failure.
*/
int8_t MCP356x::useInternalVref(bool x) {
  int8_t ret = -1;
  if (_flags.value(MCP356X_FLAG_HAS_INTRNL_VREF)) {
    uint32_t c0_val = _get_shadow_value(MCP356xRegister::CONFIG0) & 0x00FFFFBF;
    if (x) {
      c0_val |= 0x00000040;
      _vref_plus  = 2.4;
      _vref_minus = 0;
    }
    ret = (0 == _write_register(MCP356xRegister::CONFIG0, c0_val)) ? 0 : -2;
  }
  return ret;
}



/*******************************************************************************
* Mid-level register API.
*******************************************************************************/

/**
* Changes the conversion mode in CONFIG0.
*
* @param e The desired conversion mode enum.
* @return
*    -2 on no bus adapter.
*    -1 if there was a problem writing the config register.
*    0 if config was set successfully.
*/
int8_t MCP356x::_set_read_mode(MCP356xMode e) {
  int8_t ret = -1;
  uint32_t c0_val = _get_shadow_value(MCP356xRegister::CONFIG0);
  const uint8_t DESIRED_MODE = ((uint8_t) e) & 0x03;
  //if (DESIRED_MODE != (c0_val & 0x03)) {
    const uint8_t NEW_C0 = (DESIRED_MODE | (0xFC & c0_val));
    ret--;
    if (0 == _write_register(MCP356xRegister::CONFIG0, NEW_C0)) {
      ret = 0;
    }
  //}
  return ret;
}

/**
* Changes the current source setting in CONFIG0.
* This is basically used for burnout detection in external hardware.
*
* @param e The desired bias current enum.
* @return
*    -2 on no bus adapter.
*    -1 if there was a problem writing the config register.
*    0 if config was set successfully.
*/
int8_t MCP356x::_set_bias_current(MCP356xBiasCurrent e) {
  uint8_t c0_val = _get_shadow_value(MCP356xRegister::CONFIG0) & 0xF3;
  return _write_register(MCP356xRegister::CONFIG0, (c0_val | ((uint8_t) e << 2)));
}

/**
* Changes the gain setting.
*
* @param g The desired gain enum.
* @return
*    -2 on no bus adapter.
*    -1 if there was a problem writing the config register.
*    0 if config was set successfully.
*/
int8_t MCP356x::_set_gain(MCP356xGain g) {
  uint32_t c_val = _get_shadow_value(MCP356xRegister::CONFIG2);
  uint32_t gain_val = (c_val & 0xFFFFFFC7) | ((uint32_t) g << 3);
  return _write_register(MCP356xRegister::CONFIG2, gain_val);
}

/**
* Changes the AMCLK prescaler.
*
* @param d
* @return
*   -2 if MCLK frequency is unknown, but prescalar setting succeeded.
*   -1 if reconfiguration of prescalar failed.
*   0  on success.
*/
int8_t MCP356x::_set_amlclk_prescaler(MCP356xAMCLKPrescaler d) {
  uint32_t c1_val = _get_shadow_value(MCP356xRegister::CONFIG1) & 0x00FFFF3F;
  c1_val |= ((((uint8_t) d) & 0x03) << 6);
  int8_t ret = _write_register(MCP356xRegister::CONFIG1, c1_val);
  if (0 == ret) {
    ret = _recalculate_clk_tree();
  }
  return ret;
}

/**
* Changes the oversampling ratio.
*
* @param d The desired oversampling enum.
* @return
*    -2 on no bus adapter.
*    -1 if there was a problem writing the config register.
*    0 if config was set successfully.
*/
int8_t MCP356x::_set_oversampling_ratio(MCP356xOversamplingRatio d) {
  uint32_t c1_val = _get_shadow_value(MCP356xRegister::CONFIG1) & 0x00FFFFC3;
  c1_val |= ((((uint8_t) d) & 0x0F) << 2);
  int8_t ret = _write_register(MCP356xRegister::CONFIG1, c1_val);
  if (0 == ret) {
    ret = _recalculate_settling_time();
  }
  return ret;
}


/**
* Returns the number of channels this part supports. Should be (2, 4, 8). Any
*   other value is invalid and indicates a need for a register sync (if 0) or
*   the wrong device entirely.
*
* @return The known channel count of the device, based on its report.
*/
uint8_t MCP356x::_channel_count() {
  switch ((uint16_t) _get_shadow_value(MCP356xRegister::RESERVED2)) {
    case 0x000C:   return 2;  // MCP3561
    case 0x000D:   return 4;  // MCP3562
    case 0x000F:   return 8;  // MCP3564
  }
  return 0;
}


/**
* Depending on config, the width of the ADCDATA register might vary.
*
* @return the number of bytes to be read from the data register.
*/
uint8_t MCP356x::_output_coding_bytes() {
  return (0 == _get_shadow_value(MCP356xRegister::CONFIG3)) ? 3 : 4;
}


/**
* Output coding correction and observation.
* This function also observes the calibration flags and marks the driver as
*   calibrated if all of the parameters have been read.
* Resulting voltage value is stored in the appropriate slot in channel_vals[].
* NOTE: We rely on (assume) the output coding having chan-id and SGN.
*
* @return 0 always.
*/
int8_t MCP356x::_normalize_data_register() {
  uint32_t rval = _get_shadow_value(MCP356xRegister::ADCDATA);
  MCP356xChannel chan = (MCP356xChannel) ((rval >> 28) & 0x0F);

  // Sign extend, if needed.
  int32_t nval = (int32_t) (rval & 0x01000000) ? (rval | 0xFE000000) : (rval & 0x01FFFFFF);

  // Update the over-range marker...
  channel_vals[(uint8_t) chan]  = nval;   // Store the decoded ADC reading.
  _channel_set_ovr_flag(chan, ((nval > 8388609) | (nval < -8388609)));

  switch (chan) {
    // Different channels are interpreted differently...
    case MCP356xChannel::SE_0:   // Single-ended channels.
    case MCP356xChannel::SE_1:
    case MCP356xChannel::SE_2:
    case MCP356xChannel::SE_3:
    case MCP356xChannel::SE_4:
    case MCP356xChannel::SE_5:
    case MCP356xChannel::SE_6:
    case MCP356xChannel::SE_7:
    case MCP356xChannel::DIFF_A:   // Differential channels.
    case MCP356xChannel::DIFF_B:
    case MCP356xChannel::DIFF_C:
    case MCP356xChannel::DIFF_D:
      break;
    case MCP356xChannel::TEMP:
      break;
    case MCP356xChannel::AVDD:
      if (!_flags.value(MCP356X_FLAG_SAMPLED_AVDD)) {
        _flags.set(MCP356X_FLAG_SAMPLED_AVDD);
        if (!_flags.value(MCP356X_FLAG_VREF_DECLARED)) {
          // If we are scanning the AVDD channel, we use that instead of the
          //   assumed 3.3v.
          _vref_plus = nval / (8388608.0 * 0.33);
        }
      }
      break;
    case MCP356xChannel::VCM:
      // Nothing done here yet. Value should always be near 1.2v.
      if (!_flags.value(MCP356X_FLAG_SAMPLED_VCM)) {
        _flags.set(MCP356X_FLAG_SAMPLED_VCM);
      }
      break;
    case MCP356xChannel::OFFSET:
      if (!_flags.value(MCP356X_FLAG_SAMPLED_OFFSET)) {
        _flags.set(MCP356X_FLAG_SAMPLED_OFFSET);
      }
      if (MCP356xState::CALIBRATION == currentState()) {
        setOffsetCalibration(nval);
      }
      break;
  }

  // With a callback, we deliver the value directly.
  // Without anyone specific to notify, mark the channel as updated.
  if ((nullptr != _value_callback) & (MCP356xState::READING == currentState())) {
    _value_callback(chan, valueAsVoltage(chan));
  }
  else {
    _channel_set_new_flag(chan);
  }
  return 0;
}


/**
* Gain enum to value conversion function.
*
* @return the gain of the ADC as a float.
*/
float MCP356x::_gain_value() {
  return ADC_GAIN_VALUES[((uint8_t) getGain()) & 0x07];
}


/**
* Saves the current channel settings and sets them to the new value given.
*
* @param rval The new value for the SCAN register.
* @return
*   -1 on failure
*   0 on success
*/
int8_t MCP356x::_set_scan_channels(uint32_t rval) {
  return (0 == _write_register(MCP356xRegister::SCAN, (uint32_t) rval)) ? 0 : -1;
}



/*******************************************************************************
* Hardware discovery functions
*
* The valid ADC input clock range is between 0.1 and 20 MHz.
* Technically, there is no lower bound on MCLK given in the datasheet. But our
*   temporal resolution in the calibration phase of the FSM takes too long for
*   a decent reading without some lower-bound.
*
* Some designs drive the ADC from an on-board high-Q oscillator. But there is
*   no direct firmware means to discover the setting.
*
* The timing parameters of the ADC must be known to arrive at a linear model of
*   the interrupt rate with respect to input clock. Then, we use the model to
*   determine clock rate by watching the IRQ rate.
*******************************************************************************/

/**
* After the ADC has been running for awhile, we can calculate the true rate of
*   the input clock if we don't know it already.
*
* @param elapsed_us The number of microseconds that were spent taking readings.
* @return The calculated MCLK frequency
*/
double MCP356x::_calculate_input_clock(unsigned long elapsed_us) {
  uint32_t c_val = _get_shadow_value(MCP356xRegister::CONFIG1);
  uint32_t osr_idx = (c_val & 0x0000003C) >> 2;
  uint16_t osr1 = OSR1_VALUES[osr_idx];
  uint16_t osr3 = OSR3_VALUES[osr_idx];
  uint32_t pre_val = (c_val & 0x000000C0) >> 6;
  double  _drclk = ((double) _irqs_noted) / ((double) elapsed_us) * 1000000.0;
  return (4 * (osr3 * osr1) * (1 << pre_val) * _drclk);
}


/**
* Given MCLK, calculate DRCLK and store it locally.
*
* @return
*   -2 if MCLK frequency is out-of-bounds.
*   0  if the calculation completed.
*/
int8_t MCP356x::_recalculate_clk_tree() {
  if (_mclk_in_bounds()) {
    uint32_t pre_val = (_get_shadow_value(MCP356xRegister::CONFIG1) & 0x000000C0) >> 6;
    _dmclk_freq = _mclk_freq / (4 * (1 << pre_val));
    return _recalculate_settling_time();
  }
  return -2;
}


/**
* Given our existing parameters, get the theoretical maximum settling time for
*   the data. This does not account for the circuit, obviously. It is only the
*   analytical guess based on the datasheet. It represents the minimum time
*   that the program should delay for an accurate reading.
*
* @return 0 always
*/
int8_t MCP356x::_recalculate_settling_time() {
  uint32_t osr_idx = (_get_shadow_value(MCP356xRegister::CONFIG1) & 0x0000003C) >> 2;
  uint16_t osr1 = OSR1_VALUES[osr_idx];
  uint16_t osr3 = OSR3_VALUES[osr_idx];
  uint32_t dmclks = (3 * osr3) + ((osr1 - 1) * osr3);
  _settling_ms = ((uint32_t) (1000 / _dmclk_freq)) * dmclks;
  return 0;
}


/*******************************************************************************
* Basal register and SPI I/O functions.
*******************************************************************************/
/**
* Applies the device address and properly shifts the register address into
*   a control byte. Always sets up for incremental read/write.
* This never fails and always returns a byte to be used as the first byte
*   in an SPI transaction with the ADC.
*
* @param r The register we wish to transact with.
* @return the first byte of an SPI register transaction.
*/
uint8_t MCP356x::_get_reg_addr(MCP356xRegister r) {
  return (((_DEV_ADDR & 0x03) << 6) | (((uint8_t) r) << 2) | 0x02);
}


/**
* Sends a fast command that is not register access.
*
* @param cmd The command byte to send
* @return
*    -2 on no bus adapter.
*    -1 on failure to write register.
*    0  on success.
*/
int8_t MCP356x::_send_fast_command(uint8_t cmd) {
  int8_t ret = -2;
  if (nullptr != _BUS) {
    ret++;
    SPIBusOp* op = (SPIBusOp*) _BUS->new_op(BusOpcode::TX, (BusOpCallback*) this);
    if (nullptr != op) {
      op->setParams((uint8_t) ((_DEV_ADDR & 0x03) << 6) | cmd);
      ret = queue_io_job(op);
    }
  }
  return ret;
}


/**
* Resets the internal register shadows to their default values.
* Also resets internal second-order data to avoid corrupted results.
*
* @return 0 always
*/
void MCP356x::_clear_registers() {
  uint32_t flg_mask = MCP356X_FLAG_RESET_MASK;
  if (!(_flags.value(MCP356X_FLAG_USE_INTRNL_CLK))) {
    // The only way the clock isn't running is if it is running internally.
    flg_mask |= MCP356X_FLAG_MCLK_RUNNING;
  }
  _flags = (_flags.raw & flg_mask);  // Reset the flags.
  for (uint8_t i = 0; i < 16; i++) {
    // We decline to revert RESERVED2, since we need it for device identity.
    // RESERVED2 (0x000F for 3564, 0xD for 3562, 0xC for 3561)
    _reg_shadows[i]  = (14 != i) ? 0 : _reg_shadows[i];
    channel_vals[i]  = 0;
  }
  _channel_flags        = 0;
  _discard_window.period(0);
  _profiler_result_read.reset();
  _settling_ms          = 0;
  read_accumulator      = 0;
  micros_last_read      = 0;
  micros_last_window    = 0;
}


/*
* Function stores register values as the ADC's native endianess (Big).
*/
int8_t MCP356x::_set_shadow_value(MCP356xRegister r, uint32_t val) {
  uint8_t final_reg_val[4] = {0, 0, 0, 0};   // TODO: Strict aliasing....
  int8_t ret = -1;
  uint8_t register_size = (MCP356xRegister::ADCDATA == r) ? _output_coding_bytes() : MCP356x_reg_width[(uint8_t) r];
  uint8_t i = 0;
  switch (register_size) {
    case 4:    final_reg_val[i++] = ((uint8_t) (val >> 24) & 0xFF);   // MSB-first
    case 3:    final_reg_val[i++] = ((uint8_t) (val >> 16) & 0xFF);
    case 2:    final_reg_val[i++] = ((uint8_t) (val >> 8)  & 0xFF);
    case 1:    final_reg_val[i++] = ((uint8_t) val & 0xFF);
      _reg_shadows[(uint8_t) r] = *((uint32_t*) final_reg_val);  // Should always be MSB ordered.
      ret = 0;
      break;
    default:
      ret = -2;   // Error on unexpected width.
      break;
  }
  return ret;
}


uint32_t MCP356x::_get_shadow_value(MCP356xRegister r) {
  uint32_t ret = 0;
  uint8_t final_reg_val[4] = {0, 0, 0, 0};
  uint8_t register_size = (MCP356xRegister::ADCDATA == r) ? _output_coding_bytes() : MCP356x_reg_width[(uint8_t) r];
  uint8_t* buf_base = (uint8_t*) &_reg_shadows[(uint8_t) r];
  switch (register_size) {
    case 4:    final_reg_val[--register_size] = *(buf_base++);   // MSB-first
    case 3:    final_reg_val[--register_size] = *(buf_base++);
    case 2:    final_reg_val[--register_size] = *(buf_base++);
    case 1:    final_reg_val[--register_size] = *(buf_base++);
      ret = *((uint32_t*) final_reg_val);   // Now in native MCU endianness.
      break;
    default:
      break;
  }
  return ret;
}


/**
* Internal function for writing registers.
* Does safety checks on the given value/register combo, updates the shadow, and
*   writes to the part.
*
* @param r The register to be written.
* @param val The value to be written to the register, in host endian representation.
* @return
*   0  on success
*   -1 on I/O failure
*   -2 if no bus adapter
*   -3 if illegal register
*/
int8_t MCP356x::_write_register(MCP356xRegister r, uint32_t val) {
  uint32_t safe_val = 0;
  int8_t ret = -2;
  if (nullptr != _BUS) {
    switch (r) {
      // Filter out the unimplemented bits.
      case MCP356xRegister::CONFIG1:
        safe_val = val & 0xFFFFFFFC;
        break;
      case MCP356xRegister::CONFIG2:
        safe_val = val | (_flags.value(MCP356X_FLAG_HAS_INTRNL_VREF) ? 0x00000001 : 0x00000003);
        break;
      case MCP356xRegister::SCAN:
        safe_val = val & 0xFFE0FFFF;
        break;
      case MCP356xRegister::RESERVED0:
        safe_val = 0x00900000;
        break;
      case MCP356xRegister::RESERVED1:
        safe_val = (_flags.value(MCP356X_FLAG_HAS_INTRNL_VREF) ? 0x00000030 : 0x00000050);
        break;
      case MCP356xRegister::RESERVED2:
        safe_val = val & 0x0000000F;
        break;

      // No safety required.
      case MCP356xRegister::CONFIG0:
      case MCP356xRegister::CONFIG3:
      case MCP356xRegister::IRQ:
      case MCP356xRegister::MUX:
      case MCP356xRegister::TIMER:
      case MCP356xRegister::OFFSETCAL:
      case MCP356xRegister::GAINCAL:
      case MCP356xRegister::LOCK:
        safe_val = val;
        break;
      // Not writable.
      case MCP356xRegister::ADCDATA:
      case MCP356xRegister::CRCCFG:
        return -3;
    }
  }

  if (nullptr != _BUS) {
    SPIBusOp* op = (SPIBusOp*) _BUS->new_op(BusOpcode::TX, (BusOpCallback*) this);
    ret++;
    if (nullptr != op) {
      _set_shadow_value(r, safe_val);
      if (_log_verbosity >= LOG_LEV_DEBUG) c3p_log(LOG_LEV_DEBUG, MCP356X_LOG_TAG, "_write_register(%u) --> 0x%08x", (uint8_t) r, safe_val);
      op->setParams((uint8_t) _get_reg_addr(r));
      op->setBuffer((uint8_t*) &_reg_shadows[(uint8_t) r], MCP356x_reg_width[(uint8_t) r]);
      if (0 == queue_io_job(op)) {
        ret = 0;
      }
    }
  }
  return ret;
}


/**
* Reads the given register.
*
* @param r The register to be read.
* @return
*   -2 on no bus
*   -1 on I/O failure
*   0 on success.
*/
int8_t MCP356x::_read_register(MCP356xRegister r) {
  int8_t ret = -2;
  if (nullptr != _BUS) {
    uint8_t bytes_to_read = MCP356x_reg_width[(uint8_t) r];
    if (MCP356xRegister::ADCDATA == r) {
      bytes_to_read = _output_coding_bytes();
    }
    ret++;
    SPIBusOp* op = (SPIBusOp*) _BUS->new_op(BusOpcode::RX, (BusOpCallback*) this);
    if (nullptr != op) {
      op->setParams((uint8_t) _get_reg_addr(r) | 0x01);
      op->setBuffer((uint8_t*) &_reg_shadows[(uint8_t) r], bytes_to_read);
      if (0 == queue_io_job(op)) {
        ret = 0;
      }
    }
  }
  return ret;
}


/**
* Respond to the contents of the IRQ register.
*
* @return
*   0 if there is nothing to do after this.
*   1 if a read operation was dispatched.
*/
int8_t MCP356x::_proc_irq_register() {
  int8_t ret = 0;
  uint8_t irq_reg_data = (uint8_t) _get_shadow_value(MCP356xRegister::IRQ);
  _flags.set(MCP356X_FLAG_CRC_ERROR, (0 == (0x20 & irq_reg_data)));
  if (0 == (0x40 & irq_reg_data)) {   // Conversion is finished.
    //c3p_log(LOG_LEV_DEBUG, MCP356X_LOG_TAG, "_proc_irq_register() conversion finsihed");
    if (_busop_dat_read.hasFault()) {
      // If there was a bus fault, the BusOp might be left in an unqueuable state.
      // Try to reset the BusOp to satisfy the caller.
      _busop_dat_read.markForRequeue();
    }
    if (_busop_dat_read.isIdle()) {
      if (0 == _BUS->queue_io_job(&_busop_dat_read, _bus_priority)) {
        _profiler_result_read.markStart();
        _io_dispatched++;
        ret = 1;
        if (!_flags.value(MCP356X_FLAG_MCLK_RUNNING)) {
          _flags.set(MCP356X_FLAG_MCLK_RUNNING);  // This must be reality.
        }
      }
    }
  }
  if (0 == (0x10 & irq_reg_data)) { // Power-on-Reset has happened.
    // Not sure why this happened, but reset the class.
    setPin(_CS_PIN, 0);
    setPin(_CS_PIN, 1);
    setPin(_CS_PIN, 0);
    setPin(_CS_PIN, 1);
    if (_log_verbosity >= LOG_LEV_NOTICE) c3p_log(LOG_LEV_NOTICE, MCP356X_LOG_TAG, "_proc_irq_register() found a PoR event");
  }
  if (0 == (0x20 & irq_reg_data)) { // CRC config error.
    // Something is sideways in the configuration.
    // Send start/restart conversion. Might also write 0xA5 to the LOCK register.
    //_write_register(MCP356xRegister::LOCK, 0x000000A5);
    //_send_fast_command(0x28);
  }
  //if (0x01 & irq_reg_data) {   // Conversion started
    // We don't configure the class this way, and don't observe the IRQ.
  //}
  // Check the state of the IRQ pin, JiC we took too long.
  _isr_fired = !readPin(_IRQ_PIN);
  //c3p_log(LOG_LEV_DEBUG, MCP356X_LOG_TAG, "_proc_irq_register(%u) returns %d", irq_reg_data, ret);
  _irqs_serviced++;
  return ret;
}


/**
* Processes the consequences of a register write.
*
* @param r The register that was freshly written.
* @return BUSOP_CALLBACK_NOMINAL always
*/
int8_t MCP356x::_proc_reg_write(MCP356xRegister r) {
  uint32_t reg_val = _get_shadow_value(r);
  int8_t ret = BUSOP_CALLBACK_NOMINAL;
  if (_log_verbosity >= LOG_LEV_DEBUG) c3p_log(LOG_LEV_DEBUG, MCP356X_LOG_TAG, "_proc_reg_write(%s)  %u --> 0x%06x", stateStr(currentState()), (uint8_t) r, reg_val);

  switch (r) {
    case MCP356xRegister::CONFIG0:
      if (_flags.value(MCP356X_FLAG_USE_INTRNL_CLK)) {
        _flags.set(MCP356X_FLAG_MCLK_RUNNING);
      }
      _current_conf.bias = getBiasCurrent();
      break;
    case MCP356xRegister::CONFIG1:
      _current_conf.prescaler = getAMCLKPrescaler();
      _current_conf.over = getOversamplingRatio();
      break;
    case MCP356xRegister::CONFIG2:
      _current_conf.gain = getGain();
      break;
    case MCP356xRegister::CONFIG3:
      if (MCP356xState::POST_INIT == currentState()) {
        // We construe the first write to CONFIG3 as being the end of POST_INIT.
        _flags.clear(MCP356X_FLAG_STATE_HOLD);
      }
      break;
    case MCP356xRegister::SCAN:
      _current_conf.scan = _get_shadow_value(MCP356xRegister::SCAN);
      _channel_flags = 0;  // Zero the channel flags.
      break;

    case MCP356xRegister::MUX:
      // When the MUX register changes, we reset the read count.
      resetReadCount();
      break;

    case MCP356xRegister::IRQ:
      // A write to this register implies we are ready to service IRQs.
      _servicing_irqs(true);
      _isr_fired = !readPin(_IRQ_PIN);
      break;
    case MCP356xRegister::TIMER:
      break;
    case MCP356xRegister::OFFSETCAL:
      if (MCP356xState::CALIBRATION == currentState()) {
        _flags.set(MCP356X_FLAG_WROTE_OFFSET_CAL);
      }
      break;
    case MCP356xRegister::GAINCAL:
    case MCP356xRegister::RESERVED0:
    case MCP356xRegister::RESERVED1:
    case MCP356xRegister::LOCK:
    case MCP356xRegister::RESERVED2:
      break;
    default:   // Anything else is an illegal target for write.
      break;
  }
  return ret;
}


/**
* Processes the consequences of a register read.
*
* @param r The register that was freshly read.
* @return
*   BUSOP_CALLBACK_NOMINAL
*   BUSOP_CALLBACK_RECYCLE
*/
int8_t MCP356x::_proc_reg_read(MCP356xRegister r) {
  int8_t ret = BUSOP_CALLBACK_NOMINAL;
  if ((MCP356xRegister::ADCDATA != r) && (_log_verbosity >= LOG_LEV_DEBUG)) {
    // Don't even try to log the data register reads. They will flood the log
    //   under most circumstances.
    c3p_log(LOG_LEV_DEBUG, MCP356X_LOG_TAG, "MCP356x::_proc_reg_read(%s)  %u --> 0x%02x", stateStr(currentState()), (uint8_t) r, _get_shadow_value(r));
  }

  switch (r) {
    case MCP356xRegister::CONFIG0:
    case MCP356xRegister::CONFIG1:
    case MCP356xRegister::CONFIG2:
    case MCP356xRegister::CONFIG3:
    case MCP356xRegister::MUX:
    case MCP356xRegister::SCAN:
    case MCP356xRegister::TIMER:
    case MCP356xRegister::OFFSETCAL:
    case MCP356xRegister::GAINCAL:
    case MCP356xRegister::LOCK:
      break;
    case MCP356xRegister::RESERVED2:
      if (0x00900000 == _get_shadow_value(MCP356xRegister::RESERVED0)) {
        uint8_t res1_val = (uint8_t) _get_shadow_value(MCP356xRegister::RESERVED1);
        switch (res1_val) {
          case 0x30:
          case 0x50:
            // If the chip has an internal Vref, it will start up running and
            //   connected.
            _flags.set(MCP356X_FLAG_HAS_INTRNL_VREF, (res1_val == 0x30));
            switch ((uint8_t) _get_shadow_value(MCP356xRegister::RESERVED2)) {
              case 0x0C:
              case 0x0D:
              case 0x0F:
                _flags.set(MCP356X_FLAG_DEVICE_PRESENT);
                ret = 0;
                break;
              default:
                if (_log_verbosity >= LOG_LEV_ERROR) c3p_log(LOG_LEV_ERROR, MCP356X_LOG_TAG, "bad RESERVED2 value");
                break;
            }
            break;
          default:
            if (_log_verbosity >= LOG_LEV_ERROR) c3p_log(LOG_LEV_ERROR, MCP356X_LOG_TAG, "bad RESERVED1 value");
            break;
        }
      }
      else if (_log_verbosity >= LOG_LEV_ERROR) c3p_log(LOG_LEV_ERROR, MCP356X_LOG_TAG, "bad RESERVED0 value");
      break;
    case MCP356xRegister::CRCCFG:
      break;
    case MCP356xRegister::ADCDATA:    // Handled by a dedicated BusOp object.
    case MCP356xRegister::IRQ:        // Handled by a dedicated BusOp object.
    case MCP356xRegister::RESERVED0:  // Handled in RESERVED2 case.
    case MCP356xRegister::RESERVED1:  // Handled in RESERVED2 case.
    default:
      break;
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
* @param  _op  The bus operation that is about to run.
* @return 0 to run the op, or non-zero to cancel it.
*/
int8_t MCP356x::io_op_callahead(BusOp* _op) {
  return 0;
}

/**
* When a bus operation completes, it is passed back to its issuing class.
* This driver never reads back from the device. Assume all ops are WRITEs.
* The initialization chain is carried forward in this function, so that we don't
*   swamp the bus queue. Display will enable at the end of the init chain.
*
* @param  _op  The bus operation that was completed.
* @return BUSOP_CALLBACK_NOMINAL on success, or appropriate error code.
*/
int8_t MCP356x::io_op_callback(BusOp* _op) {
  SPIBusOp* op = (SPIBusOp*) _op;
  int8_t ret = BUSOP_CALLBACK_NOMINAL;
  uint8_t first_param = 0x3F & op->getTransferParam(0);   // There will always be a transfer param.

  // There is zero chance this object will be a null pointer unless it was done on purpose.
  if (op->hasFault()) {
    _io_called_back++;
    return BUSOP_CALLBACK_ERROR;
  }

  if (op == &_busop_irq_read) {
    // IRQ register read.
    _profiler_irq_timing.markStop();
    _proc_irq_register();
    if (_isr_fired) {   // If IRQ is still not disasserted, re-read the register.
      ret = BUSOP_CALLBACK_RECYCLE;
      _profiler_irq_timing.markStart();
    }
  }
  else if (op == &_busop_dat_read) {
    // DATA register read.
    _profiler_result_read.markStop();
    micros_last_read = micros();
    read_accumulator++;
    switch (currentState()) {
      case MCP356xState::CALIBRATION:
      case MCP356xState::READING:
        if (_discard_window.expired()) {
          _discard_window.reset(0);
          // If we aren't in the settling period, we observe the data that was read.
          _normalize_data_register();
        }
        break;
      default:
        break;
    }
  }
  else {
    switch (first_param) {
      case 0x28:   // Fast command for start/restart conversion.
        break;
      case 0x38:   // Fast command for reset.
        break;
      default:
        {   // This was register access.
          uint8_t reg_idx = first_param >> 2;
          if (BusOpcode::TX == op->get_opcode()) {
            ret = _proc_reg_write((MCP356xRegister) reg_idx);
          }
          else {
            if (_flags.value(MCP356X_FLAG_REFRESH_CYCLE)) {
              if (15 > reg_idx) {
                reg_idx++;
                op->setParams((uint8_t) _get_reg_addr((MCP356xRegister) reg_idx) | 0x01);
                op->setBuffer((uint8_t*) &_reg_shadows[reg_idx], MCP356x_reg_width[reg_idx]);
                ret = BUSOP_CALLBACK_RECYCLE;
              }
              else {   // This is the end of the refresh cycle.
                _flags.clear(MCP356X_FLAG_REFRESH_CYCLE);
                ret = _proc_reg_read(MCP356xRegister::RESERVED2);
              }
            }
            else {
              ret = _proc_reg_read((MCP356xRegister) reg_idx);
            }
          }
        }
        break;
    }
  }

  if (BUSOP_CALLBACK_RECYCLE != ret) {
    _io_called_back++;
  }
  return ret;
}


/**
* This is what is called when the class wants to conduct a transaction on the bus.
* Note that this is different from other class implementations, in that it checks for
*   callback population before clobbering it. This is because this class is also the
*   SPI driver. This might end up being reworked later.
*
* @param  _op  The bus operation to execute.
* @return 0 on success, or appropriate error code.
*/
int8_t MCP356x::queue_io_job(BusOp* _op) {
  // This is the choke-point whereby any parameters to the operation that are
  //   uniform for this driver can be set.
  SPIBusOp* op = (SPIBusOp*) _op;
  op->setCSPin(_CS_PIN);
  op->csActiveHigh(false);
  op->bitsPerFrame(SPIFrameSize::BITS_8);
  op->maxFreq(19000000);
  op->cpol(false);
  op->cpha(false);

  int8_t ret = _BUS->queue_io_job(op, _bus_priority);
  if (0 == ret) {
    _io_dispatched++;
  }
  return ret;
}
