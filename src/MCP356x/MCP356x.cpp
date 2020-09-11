/*
File:   MCP356x.cpp
Author: J. Ian Lindsay
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
// We can have up to four of these in a given system.
#define MCP356X_MAX_INSTANCES    2
volatile static MCP356x* INSTANCES[MCP356X_MAX_INSTANCES] = {0, };

/* Register widths in bytes. Index corresponds directly to register address. */
static const uint8_t MCP356x_reg_width[16] = {
  1, 1, 1, 1, 1, 1, 1, 3, 3, 3, 3, 3, 1, 1, 2, 2
};

static const uint16_t OSR1_VALUES[16] = {
  1, 1, 1, 1, 1, 2, 4, 8, 16, 32, 40, 48, 80, 96, 160, 192
};

static const uint16_t OSR3_VALUES[16] = {
  32, 64, 128, 256, 512, 512, 512, 512, 512, 512, 512, 512, 512, 512, 512, 512
};

static const float ADC_GAIN_VALUES[8] = {
  0.33, 1.0, 2.0, 4.0, 8.0, 16.0, 32.0, 64.0
};

static const char* CHAN_NAMES[16] = {
  "SE_0", "SE_1", "SE_2", "SE_3", "SE_4", "SE_5", "SE_6", "SE_7",
  "DIFF_A", "DIFF_B", "DIFF_C", "DIFF_D", "TEMP", "AVDD", "VCM", "OFFSET"
};

/**
* This is an ISR.
*/
void mcp356x_isr0() {
  ((MCP356x*) INSTANCES[0])->isr_fired = true;
}

/**
* This is an ISR.
*/
void mcp356x_isr1() {
  ((MCP356x*) INSTANCES[1])->isr_fired = true;
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
* Constructor. Delegated.
*/
MCP356x::MCP356x(uint8_t irq_pin, uint8_t cs_pin, uint8_t mclk_pin) :
  MCP356x(irq_pin, cs_pin, mclk_pin, 0x01) {}

/*
* Constructor specifying device address.
*/
MCP356x::MCP356x(uint8_t irq_pin, uint8_t cs_pin, uint8_t mclk_pin, uint8_t addr) :
  _IRQ_PIN(irq_pin), _CS_PIN(cs_pin), _MCLK_PIN(mclk_pin), _DEV_ADDR(addr),
  _busop_irq_read(BusOpcode::RX, this, cs_pin, false)
{
  bool unslotted = true;
  for (uint8_t i = 0; i < MCP356X_MAX_INSTANCES; i++) {
    if (unslotted) {
      if (nullptr == INSTANCES[i]) {
        _slot_number = i;
        INSTANCES[_slot_number] = this;
        unslotted = false;
      }
    }
  }
}


/**
* Destructor
*/
MCP356x::~MCP356x() {
  // TODO: This will almost certainly NEVER be called. Might be smart to wall
  //   it off with a preprocessor directive for pedantic destructors.
  if (255 != _IRQ_PIN) {
    unsetPinFxn(_IRQ_PIN);
  }
  INSTANCES[_slot_number] = nullptr;
}


/**
* Send the fastcommand to bounce the ADC.
*
* @return
*    -2 on no bus adapter.
*    -1 if there was a problem writing the reset command.
*    0 if reset command was sent successfully.
*/
int8_t MCP356x::reset() {
  return _send_fast_command(0x38);
}


/*
* Resets the chip and writes our default configuration.
*
* @return
*   -4 if pin setup failed.
*   -3 if bad bus reference.
*   -2 if reset failed.
*   0  on success.
*/
int8_t MCP356x::init(SPIAdapter* b) {
  int8_t pin_setup_ret = _ll_pin_init();  // Configure the pins if they are not already.
  int8_t ret = -4;
  if (pin_setup_ret >= 0) {
    ret = -3;
    if (nullptr != b) {
      ret = -2;
      _BUS = b;
      _busop_irq_read.setAdapter(_BUS);
      _busop_irq_read.shouldReap(false);
      _busop_irq_read.setParams((uint8_t) _get_reg_addr(MCP356xRegister::IRQ) | 0x01);
      _busop_irq_read.setBuffer((uint8_t*) &_reg_shadows[(uint8_t) MCP356xRegister::IRQ], 1);

      if (0 == reset()) {
        ret = 0;
        //if (0 == _post_reset_fxn()) {
        //  if (!_mclk_in_bounds()) {   // Need to detect MCLK...
        //    ret = -6;
        //    int8_t det_ret = _detect_adc_clock();
        //    switch (det_ret) {
        //      case -3:    break;  // -3 if the class isn't ready for this measurement.
        //      case -2:    break;  // -2 if measurement timed out.
        //      case -1:    break;  // -1 if there was some mechanical problem communicating with the ADC
        //      case 0:     break;  // 0  if a clock signal within expected range was determined
        //      case 1:     break;  // 1  if we got a clock rate that was out of bounds or appears nonsensical
        //      default:    break;
        //    }
        //    //Serial.print("_detect_adc_clock returned ");
        //    //Serial.println(det_ret);
        //  }
        //  if (_mclk_in_bounds()) {
        //    switch (_calibrate_offset()) {
        //      case 0:   ret = 0;    break;
        //      default:  ret = -7;   break;
        //    }
        //  }
        //}
      }
    }
  }
  return ret;
}


/**
* Sets some of the more specialized features of the chip.
* Pass as an argument a select combination of flags defined in the
*   header file.
* NOTE: This fxn must be called ahead of first init(), since our
*   pin configuration might be in conflict with ADC configuration
*   otherwise.
*
* @param flgs The flag for the option to set.
* @return
*   -1 if called too late for a provided option.
*   0  on success.
*/
int8_t MCP356x::setOption(uint32_t flgs) {
  int8_t ret = 0;
  if (flgs & MCP356X_FLAG_USE_INTERNAL_CLK) {
    if (!(_mcp356x_flag(MCP356X_FLAG_PINS_CONFIGURED))) {
      // Only allow this change if the pins are not yet configured.
      _mcp356x_set_flag(MCP356X_FLAG_USE_INTERNAL_CLK);
      _mcp356x_clear_flag(MCP356X_FLAG_GENERATE_MCLK);
    }
    else {
      ret = -1;
    }
  }
  if (flgs & MCP356X_FLAG_GENERATE_MCLK) {
    if (!(_mcp356x_flag(MCP356X_FLAG_PINS_CONFIGURED))) {
      // Only allow this change if the pins are not yet configured.
      _mcp356x_set_flag(MCP356X_FLAG_GENERATE_MCLK);
      _mcp356x_clear_flag(MCP356X_FLAG_USE_INTERNAL_CLK);
    }
    else {
      ret = -1;
    }
  }
  if (flgs & MCP356X_FLAG_3RD_ORDER_TEMP) {
    _mcp356x_set_flag(MCP356X_FLAG_3RD_ORDER_TEMP);
  }
  return ret;
}


/**
* Handles our configuration after reset.
*
* @return
*   -1 on failure to write a register.
*   0  on success.
*/
int8_t MCP356x::_post_reset_fxn() {
  int8_t ret = -1;
  uint32_t c0_val = 0x000000C3;

  // Enable fast command, disable IRQ on conversion start, IRQ pin is open-drain.
  ret = _write_register(MCP356xRegister::IRQ, 0x00000002);
  if (0 == ret) {
    if (_mcp356x_flag(MCP356X_FLAG_USE_INTERNAL_CLK)) {
      c0_val &= 0xFFFFFFCF;   // Set CLK_SEL to use internal clock with no pin output.
      c0_val |= 0x00000020;
    }
    ret = _write_register(MCP356xRegister::CONFIG0, c0_val);
    if (0 == ret) {
      // For simplicity, we select a 32-bit sign-extended data representation with
      //   channel identifiers.
      ret = _write_register(MCP356xRegister::CONFIG3, 0x000000F0);
    }
  }
  return ret;
}


/*
* Reads the part's output register and performs the needed manipulation on the
*   data, according to the current configuration.
* NOTE: If the application recently called discardUnsettledSamples(), the register
*   shadow will not be updated unless the timeout has expired. But no matter what,
*   the true value of the register will be returned from this function. So if
*   the application wants to know if _valid_ data was read, it should call
*   newValue() or scanComplete() ahead of using it.
* Returns...
*   -1 on error reading from registers.
*   0  on successful read, but nothing to do.
*   1  if data was ready and read, but discarded due to being within the
*        declared settling time.
*   2  if the read resulted in a full set of data for all channels being scanned.
*/
int8_t MCP356x::read() {
  int8_t ret = 0;
  if (isr_fired) {
    if (_busop_irq_read.isIdle()) {
      ret = _BUS->queue_io_job(&_busop_irq_read);
      isr_fired = false;
    }
  }
  if (scanComplete()) {
    ret = 2;
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
  _mcp356x_set_flag(MCP356X_FLAG_CRC_ERROR, (0 == (0x20 & irq_reg_data)));
  if (0 == (0x40 & irq_reg_data)) {   // Conversion is finished.
    if (_busop_irq_read.isIdle()) {
      if (0 == _BUS->queue_io_job(&_busop_irq_read)) {
        ret = 1;
        if (_mcp356x_flag(MCP356X_FLAG_MEASURING_MCLK)) {
          _mcp356x_set_flag(MCP356X_FLAG_MCLK_RUNNING);  // This must be reality.
        }
      }
    }
  }
  if (0 == (0x08 & irq_reg_data)) { // Power-on-Reset has happened.
    // Not sure why this happened, but reset the class.
    //_post_reset_fxn();
    setPin(_CS_PIN, 0);
    setPin(_CS_PIN, 1);
    setPin(_CS_PIN, 0);
    setPin(_CS_PIN, 1);
  }
  if (0 == (0x20 & irq_reg_data)) { // CRC config error.
    // Something is sideways in the configuration.
    // Send start/restart conversion. Might also write 0xA5 to the LOCK register.
    _write_register(MCP356xRegister::LOCK, 0x000000A5);
    _send_fast_command(0x28);
  }
  if (0x01 & irq_reg_data) {   // Conversion started
    // We don't configure the class this way, and don't observe the IRQ.
  }
  // Check the state of the IRQ pin, JiC we took too long.
  isr_fired = !readPin(_IRQ_PIN);
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
* Given the ADC channel, walks the value backward through the ADC transfer
*   function to arrive at the voltage on that channel.
* If the application didn't set a reference voltage, we assume it is equal to
*   AVdd. If that channel has never been read, the value will default to 3.3v.
*
* @param chan The ADC channel in question.
* @return The channel's voltage.
*/
double MCP356x::valueAsVoltage(MCP356xChannel chan) {
  float  vrp  = _vref_plus;
  float  vrm  = _vref_minus;
  double result = 0.0;
  switch (chan) {
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
    case MCP356xChannel::OFFSET:
      result = (value(chan) * (vrp - vrm)) / (8388608.0 * _gain_value());
      break;
    case MCP356xChannel::TEMP:
      // TODO: voltage transfer fxn for temperature diode.
      break;
    case MCP356xChannel::AVDD:
      result = value(chan) / (0.33 * 8388608.0);   // Gain on this chan is always 0.33.
      break;
    case MCP356xChannel::VCM:
      result = (value(chan) * (vrp - vrm)) / 8388608.0;
      break;
  }
  return result;
}


/**
* Given a channel, return the last value.
*
* @param chan The ADC channel in question.
* @return The channel's raw value.
*/
int32_t MCP356x::value(MCP356xChannel chan) {
  _channel_clear_new_flag(chan);
  return channel_vals[(uint8_t) chan & 0x0F];
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
  int8_t ret = _write_register(MCP356xRegister::OFFSETCAL, (uint32_t) offset);
  if (0 == ret) {
    uint32_t c_val = _get_shadow_value(MCP356xRegister::CONFIG3);
    c_val = (0 != offset) ? (c_val | 0x00000002) : (c_val & 0xFFFFFFFD);
    ret = _write_register(MCP356xRegister::CONFIG3, c_val);
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
* Changes the gain setting.
*
* @param g The desired gain enum.
* @return
*    -2 on no bus adapter.
*    -1 if there was a problem writing the config register.
*    0 if config was set successfully.
*/
int8_t MCP356x::setGain(MCP356xGain g) {
  uint32_t c_val = _get_shadow_value(MCP356xRegister::CONFIG2);
  uint32_t gain_val = (c_val & 0xFFFFFFC7) | ((uint32_t) g << 3);
  return _write_register(MCP356xRegister::CONFIG2, gain_val);
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
* Changes the BOOST setting.
*
* @param e The desired bias current enum.
* @return
*    -2 on no bus adapter.
*    -1 if there was a problem writing the config register.
*    0 if config was set successfully.
*/
int8_t MCP356x::setBiasCurrent(MCP356xBiasCurrent e) {
  uint32_t c0_val = _get_shadow_value(MCP356xRegister::CONFIG0) & 0x00F3FFFF;
  c0_val += ((((uint8_t) e) & 0x03) << 18);
  return _write_register(MCP356xRegister::CONFIG0, c0_val);
}


/*
* Changes the AMCLK prescaler.
* Returns...
*   -2 if MCLK frequency is unknown, but prescalar setting succeeded.
*   -1 if reconfiguration of prescalar failed.
*   0  on success.
*/
int8_t MCP356x::setAMCLKPrescaler(MCP356xAMCLKPrescaler d) {
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
int8_t MCP356x::setOversamplingRatio(MCP356xOversamplingRatio d) {
  uint32_t c1_val = _get_shadow_value(MCP356xRegister::CONFIG1) & 0x00FFFFC3;
  c1_val |= ((((uint8_t) d) & 0x0F) << 2);
  int8_t ret = _write_register(MCP356xRegister::CONFIG1, c1_val);
  if (0 == ret) {
    ret = _recalculate_settling_time();
  }
  return ret;
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
* Setup the low-level pin details. Execution is idempotent.
*
* @return
*   -1 if the pin setup is wrong. Class must halt.
*   0  if the pin setup is complete.
*   1  if the pin setup is complete, and the clock needs measurement.
*/
int8_t MCP356x::_ll_pin_init() {
  int8_t ret = -1;
  if (_mcp356x_flag(MCP356X_FLAG_PINS_CONFIGURED)) {
    ret = 0;
  }
  else if (255 != _CS_PIN) {
    ret = 1;
    pinMode(_CS_PIN, GPIOMode::OUTPUT);
    setPin(_CS_PIN, 1);
    if (255 != _IRQ_PIN) {
      pinMode(_IRQ_PIN, GPIOMode::INPUT_PULLUP);
      switch (_slot_number) {  // TODO: This is terrible. You know better.
        case 0:
          setPinFxn(_IRQ_PIN, IRQCondition::FALLING, mcp356x_isr0);
          break;
        case 1:
          setPinFxn(_IRQ_PIN, IRQCondition::FALLING, mcp356x_isr1);
          break;
      }
    }
    if (255 != _MCLK_PIN) {
      // If we have MCLK, we need to generate a squarewave on that pin.
      // Otherwise, we hope that the board has an XTAL attached.
      if (_mcp356x_flag(MCP356X_FLAG_USE_INTERNAL_CLK)) {
        // TODO: We presently do nothing with this signal. But we might tap it
        //   for frequency measurement of the internal OSC.
        pinMode(_MCLK_PIN, GPIOMode::INPUT);
      }
      else {
        pinMode(_MCLK_PIN, GPIOMode::OUTPUT);
        if (_mcp356x_flag(MCP356X_FLAG_GENERATE_MCLK)) {
          // NOTE: Not all pin support this. Works for some pins on some MCUs.
          //analogWriteFrequency(_MCLK_PIN, 4915200);
          //analogWrite(_MCLK_PIN, 128);
          //_mclk_freq = 4915200.0;
        }
        else {
          // There is a hardware oscillator whose enable pin we control with
          //   the MCLK pin. Set the pin high (enabled) and measure the clock.
          setPin(_MCLK_PIN, 1);
        }
        _mcp356x_set_flag(MCP356X_FLAG_MCLK_RUNNING);
        ret = _recalculate_clk_tree();
      }
    }
    _mcp356x_set_flag(MCP356X_FLAG_PINS_CONFIGURED);
  }
  return ret;
}


/**
* Resets the internal register shadows to their default values.
* Also resets internal second-order data to avoid corrupted results.
*
* @return 0 always
*/
int8_t MCP356x::_clear_registers() {
  uint32_t flg_mask = MCP356X_FLAG_RESET_MASK;
  if (!(_mcp356x_flag(MCP356X_FLAG_USE_INTERNAL_CLK))) {
    // The only way the clock isn't running is if it is running internally.
    flg_mask |= MCP356X_FLAG_MCLK_RUNNING;
  }
  _flags = _flags & flg_mask;  // Reset the flags.
  for (uint8_t i = 0; i < 16; i++) {
    // We decline to revert RESERVED2, since we need it for device identity.
    // RESERVED2 (0x000F for 3564, 0xD for 3562, 0xC for 3561)
    _reg_shadows[i]  = (14 != i) ? 0 : _reg_shadows[i];
    channel_vals[i]  = 0;
  }
  _channel_flags        = 0;
  _discard_until_millis = 0;
  _settling_ms          = 0;
  read_count            = 0;
  read_accumulator      = 0;
  reads_per_second      = 0;
  millis_last_read      = 0;
  millis_last_window    = 0;
  return 0;
}


int8_t MCP356x::_set_shadow_value(MCP356xRegister r, uint32_t val) {
  int8_t ret = -1;
  uint8_t register_size = (MCP356xRegister::ADCDATA == r) ? _output_coding_bytes() : MCP356x_reg_width[(uint8_t) r];
  uint8_t final_reg_val[4] = {0, 0, 0, 0};
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
  uint8_t register_size = (MCP356xRegister::ADCDATA == r) ? _output_coding_bytes() : MCP356x_reg_width[(uint8_t) r];
  uint8_t final_reg_val[4] = {0, 0, 0, 0};
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
      case MCP356xRegister::CONFIG1:   safe_val = val & 0xFFFFFFFC;    break;
      case MCP356xRegister::CONFIG2:   safe_val = val | 0x00000003;    break;
      case MCP356xRegister::SCAN:      safe_val = val & 0xFFE0FFFF;    break;
      case MCP356xRegister::RESERVED0: safe_val = 0x00900000;          break;
      case MCP356xRegister::RESERVED1: safe_val = 0x00000050;          break;
      case MCP356xRegister::RESERVED2: safe_val = val & 0x0000000F;    break;
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
* TODO: We rely on (assume) the output coding having chan-id and SGN.
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
  _channel_set_new_flag(chan);            // Mark the channel as updated.

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
      _mcp356x_set_flag(MCP356X_FLAG_SAMPLED_AVDD);
      if (!_mcp356x_flag(MCP356X_FLAG_VREF_DECLARED)) {
        // If we are scanning the AVDD channel, we use that instead of the
        //   assumed 3.3v.
        //_vref_plus = nval / (8388608.0 * 0.33);
      }
      if (MCP356X_FLAG_ALL_CAL_MASK == (_mcp356x_flags() & MCP356X_FLAG_ALL_CAL_MASK)) {
        _mark_calibrated();
      }
      break;
    case MCP356xChannel::VCM:
      // Nothing done here yet. Value should always be near 1.2v.
      _mcp356x_set_flag(MCP356X_FLAG_SAMPLED_VCM);
      if (MCP356X_FLAG_ALL_CAL_MASK == (_mcp356x_flags() & MCP356X_FLAG_ALL_CAL_MASK)) {
        _mark_calibrated();
      }
      break;
    case MCP356xChannel::OFFSET:
      _mcp356x_set_flag(MCP356X_FLAG_SAMPLED_OFFSET);
      setOffsetCalibration(nval);
      break;
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
* Causes a full refresh. Update our shadows with the state of the hardware
*   registers. Class state will be updated on async I/O completion.
*
* @return
*    -2 on no bus adapter.
*    -1 on failure to read register.
*    0  on success.
*/
int8_t MCP356x::refresh() {
  uint8_t i   = 0;
  int8_t  ret = 0;
  while ((0 == ret) & (i < 16)) {
    _mcp356x_set_flag(MCP356X_FLAG_REFRESH_CYCLE);
    ret = _read_register((MCP356xRegister) i++);
  }
  return ret;
}


/**
* Causes the ADC to throw away samples after reading them. This should be
*   invoked when it is known that the analog value is changing, and we only want
*   reports of fully-settled values.
*/
void MCP356x::discardUnsettledSamples() {
  _discard_until_millis = getSettlingTime() + _circuit_settle_ms + millis();
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
* Sets the list of channels to read in SCAN mode.
*
* @param count The number of channels supplied to the variadic.
* @param variadic parameter containing the desired channels.
* @return
*   -4 if a channel was requested that the driver doesn't support.
*   -3 if a channel was requested that the hardware doesn't support.
*   -2 if no channels were given.
*   -1 on failure to write the SCAN register.
*   0  on success.
*/
int8_t MCP356x::setScanChannels(int count, ...) {
  int8_t ret = (count > 0) ? 0 : -2;
  uint8_t chan_count = _channel_count();
  uint32_t existing_scan = _get_shadow_value(MCP356xRegister::SCAN);
  uint32_t chans = 0;
  va_list args;
  va_start(args, count);
  for (int i = 0; i < count; i++) {
    MCP356xChannel chan = va_arg(args, MCP356xChannel);
    switch (chan) {
      case MCP356xChannel::SE_0:
      case MCP356xChannel::SE_1:
      case MCP356xChannel::DIFF_A:
      case MCP356xChannel::TEMP:
      case MCP356xChannel::AVDD:
      case MCP356xChannel::VCM:
      case MCP356xChannel::OFFSET:
        if (2 > chan_count) {    ret = -3;    }
        break;
      case MCP356xChannel::SE_2:
      case MCP356xChannel::SE_3:
      case MCP356xChannel::DIFF_B:
        if (4 > chan_count) {    ret = -3;    }
        break;
      case MCP356xChannel::SE_4:
      case MCP356xChannel::SE_5:
      case MCP356xChannel::SE_6:
      case MCP356xChannel::SE_7:
      case MCP356xChannel::DIFF_C:
      case MCP356xChannel::DIFF_D:
        if (8 != chan_count) {   ret = -3;    }
        break;
      default:
        ret = -4;
        break;
    }
    if (0 == ret) {
      chans = chans | (1 << ((uint8_t) chan));
    }
  }
  va_end(args);
  if (0 == ret) {   // If there were no foul ups, we can write the registers.
    chans = chans | (existing_scan & 0xFFFF0000);
    _channel_backup = chans;
    if (_mcp356x_flag(MCP356X_FLAG_CALIBRATED)) {
      ret = _set_scan_channels(chans);
    }
    else {
      _channel_backup = chans;
      ret = 0;
    }
  }
  return ret;
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
  if (!_mcp356x_flag(MCP356X_FLAG_CALIBRATED)) {
    _channel_backup = _get_shadow_value(MCP356xRegister::SCAN);
  }
  return (0 == _write_register(MCP356xRegister::SCAN, (uint32_t) rval)) ? 0 : -1;
}

/**
* Do we have data for all the channels we asked for?
*
* @return true if so.
*/
bool MCP356x::scanComplete() {
  uint32_t scan_chans = _get_shadow_value(MCP356xRegister::SCAN) & 0x0000FFFF;
  return (scan_chans == (_channel_flags & scan_chans));
};

/**
* Some hardware arrangements don't use a rail-to-rail Vref. So this function
*   can be called from the application to manually define it.
*
* @param plus The high-side value of the reference voltage
* @param minus The low-side value of the reference voltage
* @return 0 always
*/
int8_t MCP356x::setReferenceRange(float plus, float minus) {
  _vref_plus  = plus;
  _vref_minus = minus;
  _mcp356x_set_flag(MCP356X_FLAG_VREF_DECLARED);
  return 0;
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
  if (_mcp356x_flag(MCP356X_FLAG_3RD_ORDER_TEMP)) {
    const double k1 = 0.0000000000000271 * (t_lsb * t_lsb * t_lsb);
    const double k2 = -0.000000018 * (t_lsb * t_lsb);
    const double k3 = 0.0055 * t_lsb;
    const double k4 = -604.22;
    ret = k1 + k2 + k3 + k4;
  }
  else {
    ret = 0.001581 * t_lsb - 324.27;
  }
  return ret;
}


int8_t MCP356x::calibrate() {
  int8_t ret = -1;

  return ret;
}



/*******************************************************************************
* Hardware discovery functions
*******************************************************************************/

/**
* The maximum ADC input clock is between 1MHz and 20MHz.
*
* @return true if MCLK (as measured) if within operational boundaries.
*/
bool MCP356x::_mclk_in_bounds() {
  return ((_mclk_freq > 1000000.0) && (_mclk_freq < 20000000.0));
}

/*
* Some designs drive the ADC from an on-board high-Q oscillator. But there is
*   no direct firmware means to discover the setting.
* This function discovers the frequency by timing ADC reads with known clocking parameters
*   and reports the result; storing the answer in the class variable _mclk_freq.
* Returns...
*   -3 if the class isn't ready for this measurement.
*   -2 if measurement timed out.
*   -1 if there was some mechanical problem communicating with the ADC
*   0  if a clock signal within expected range was determined
*   1  if we got a clock rate that was out of bounds or appears nonsensical
*/
int8_t MCP356x::_detect_adc_clock() {
  const uint32_t SAMPLE_TIME_MAX = 200000;
  const uint32_t SAMPLE_TIME_MIN = 50000;
  int8_t ret = -3;
  if (_mcp356x_flag(MCP356X_FLAG_PINS_CONFIGURED)) {
    // The timing parameters of the ADC must be known to arrive at a linear model of
    //   the interrupt rate with respect to input clock. Then, we use the model to determine
    //   clock rate by watching the IRQ rate.
    ret = -1;
    if (0 == _write_register(MCP356xRegister::SCAN, 0)) {
      if (0 == _write_register(MCP356xRegister::MUX, 0xDE)) {
        unsigned long micros_passed     = 0;
        unsigned long micros_adc_time_0 = micros();
        uint16_t rcount = 0;
        while (((1000 > rcount) | (micros_passed < SAMPLE_TIME_MIN)) && (micros_passed < SAMPLE_TIME_MAX)) {
          // We sample for at least 50ms, but no more than 200ms.
          if (isr_fired) {
            // If data is ready...
            if (0 < read()) {
              if (0 == rcount) {
                // The first time through, we reset the read count so that we don't
                //   bake the ADC startup time into our clock calculation.
                resetReadCount();
              }
              rcount++;
            }
          }
          micros_passed = micros() - micros_adc_time_0;
        }
        ret = -2;
        if (micros_passed < SAMPLE_TIME_MAX) {
          ret = 1;
          _mcp356x_set_flag(MCP356X_FLAG_MCLK_RUNNING);  // This must be reality.
          //StringBuilder temp_str;
          //temp_str.concatf("Took %u samples in %luus.\n", rcount, micros_passed);
          //Serial.print((char*) temp_str.string());
          _mclk_freq = _calculate_input_clock(micros_passed);
          if (_mclk_in_bounds()) {
            _recalculate_clk_tree();
            ret = 0;
          }
        }
      }
    }
  }
  return ret;
}


/**
* After the ADC has been running for awhile, we can calculate the true rate of
*   the input clock if we don't know it already.
* NOTE: since the sample count doesn't reset when timing parameters are altered,
*   this only gives accurate results if the settings are unchanged from init, or
*   the caller has pinged resetReadCount() before taking the measurement. See
*   the example file for usage pattern.
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
  double  _drclk = ((double) read_count) / ((double) elapsed_us) * 1000000.0;
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


/**
* Reads the ADC channels that assist us with calibration.
* Saves the existing channel settings, before changing them.
*
* @return
*   -1 if switching to the calibration channels failed
*   0 on success.
*/
int8_t MCP356x::_calibrate_offset() {
  int8_t ret = _set_scan_channels(0x0000E000);
  if (0 == ret) {
    _mcp356x_clear_flag(MCP356X_FLAG_CALIBRATED);
  }
  return ret;
}


/**
* When all of the calibration parameters have been read and written back to the
*   hardware, this will be called to mark the driver as calibrated, and restore
*   the saved channel settings.
*
* @return
*   -1 if the restoration of the previous scan channels failed.
*   0 on success.
*/
int8_t MCP356x::_mark_calibrated() {
  int8_t ret = -1;
  if (0 == _set_scan_channels(_channel_backup)) {
    ret++;
    _mcp356x_set_flag(MCP356X_FLAG_CALIBRATED);
  }
  return ret;
}


void MCP356x::printRegs(StringBuilder* output) {
  output->concatf("reg_shadows[0] (ADCDATA)     = 0x%08x\n", _get_shadow_value(MCP356xRegister::ADCDATA));
  output->concatf("reg_shadows[1] (CONFIG0)     = 0x%02x\n", _get_shadow_value(MCP356xRegister::CONFIG0));
  output->concatf("reg_shadows[2] (CONFIG1)     = 0x%02x\n", _get_shadow_value(MCP356xRegister::CONFIG1));
  output->concatf("reg_shadows[3] (CONFIG2)     = 0x%02x\n", _get_shadow_value(MCP356xRegister::CONFIG2));
  output->concatf("reg_shadows[4] (CONFIG3)     = 0x%02x\n", _get_shadow_value(MCP356xRegister::CONFIG3));
  output->concatf("reg_shadows[5] (IRQ)         = 0x%02x\n", _get_shadow_value(MCP356xRegister::IRQ));
  output->concatf("reg_shadows[6] (MUX)         = 0x%02x\n", _get_shadow_value(MCP356xRegister::MUX));
  output->concatf("reg_shadows[7] (SCAN)        = 0x%06x\n", _get_shadow_value(MCP356xRegister::SCAN));
  output->concatf("reg_shadows[8] (TIMER)       = 0x%06x\n", _get_shadow_value(MCP356xRegister::TIMER));
  output->concatf("reg_shadows[9] (OFFSETCAL)   = 0x%06x\n", _get_shadow_value(MCP356xRegister::OFFSETCAL));
  output->concatf("reg_shadows[10] (GAINCAL)    = 0x%06x\n", _get_shadow_value(MCP356xRegister::GAINCAL));
  output->concatf("reg_shadows[11] (RESERVED0)  = 0x%06x\n", _get_shadow_value(MCP356xRegister::RESERVED0));
  output->concatf("reg_shadows[12] (RESERVED1)  = 0x%02x\n", _get_shadow_value(MCP356xRegister::RESERVED1));
  output->concatf("reg_shadows[13] (LOCK)       = 0x%02x\n", _get_shadow_value(MCP356xRegister::LOCK));
  output->concatf("reg_shadows[14] (RESERVED2)  = 0x%04x\n", _get_shadow_value(MCP356xRegister::RESERVED2));
  output->concatf("reg_shadows[15] (CRCCFG)     = 0x%04x\n", _get_shadow_value(MCP356xRegister::CRCCFG));
}


void MCP356x::printPins(StringBuilder* output) {
  output->concatf("IRQ:   %u\n", _IRQ_PIN);
  output->concatf("CS:    %u\n", _CS_PIN);
  output->concatf("MCLK:  %u\n", _MCLK_PIN);
}


void MCP356x::printTimings(StringBuilder* output) {
  output->concatf("\tMeasuring MCLK: %c\n", (_mcp356x_flag(MCP356X_FLAG_MEASURING_MCLK) ? 'y' : 'n'));
  output->concatf("\tMCLK                = %.4f MHz\n", _mclk_freq / 1000000.0);
  output->concatf("\tDMCLK               = %.4f MHz\n", _dmclk_freq / 1000000.0);
  output->concatf("\tData rate           = %.4f KHz\n", _drclk_freq / 1000.0);
  output->concatf("\tReal sample rate    = %u\n", reads_per_second);
  output->concatf("\tADC settling time   = %u\n", getSettlingTime());
  output->concatf("\tTotal settling time = %u\n", _circuit_settle_ms);
  output->concatf("\tLast read (millis)  = %u\n", millis_last_read);
}


void MCP356x::printData(StringBuilder* output) {
  output->concat("    ADC\n    --------------------------------------------------\n");
  output->concatf("\tFound:          %c\n", (adcFound() ? 'y' : 'n'));
  output->concatf("\tChannels:       %u\n", _channel_count());
  output->concatf("\tClock running:  %c\n", (_mcp356x_flag(MCP356X_FLAG_MCLK_RUNNING) ? 'y' : 'n'));
  output->concatf("\tInitialized:    %c\n", (adcConfigured() ? 'y' : 'n'));
  output->concatf("\tCalibrated:     %c\n", (adcCalibrated() ? 'y' : 'n'));
  output->concatf("\tCRC Error:      %c\n", (_mcp356x_flag(MCP356X_FLAG_CRC_ERROR) ? 'y' : 'n'));
  output->concatf("\tisr_fired:      %c\n", (isr_fired ? 'y' : 'n'));
  output->concatf("\tRead count:     %u\n", read_count);
  output->concatf("\tGain:           x%.2f\n", _gain_value());
  uint8_t _osr_idx = (uint8_t) getOversamplingRatio();
  output->concatf("\tOversampling:   x%u\n", OSR1_VALUES[_osr_idx] * OSR3_VALUES[_osr_idx]);
  output->concatf("\tVref declared:  %c\n", (_vref_declared() ? 'y' : 'n'));
  output->concatf("\tVref range:     %.3f / %.3f\n", _vref_minus, _vref_plus);
  output->concatf("\tClock SRC:      %sternal\n", (_mcp356x_flag(MCP356X_FLAG_USE_INTERNAL_CLK) ? "In" : "Ex"));
  if (_scan_covers_channel(MCP356xChannel::TEMP)) {
    output->concatf("\tTemperature:    %.2fC\n", getTemperature());
    output->concatf("\tThermo fitting: %s\n", (_mcp356x_flag(MCP356X_FLAG_3RD_ORDER_TEMP) ? "3rd-order" : "Linear"));
  }
  if (adcCalibrated()) {
    output->concat("\t");
    printChannel(MCP356xChannel::OFFSET, output);
    output->concat("\t");
    printChannel(MCP356xChannel::VCM, output);
    output->concat("\t");
    printChannel(MCP356xChannel::AVDD, output);
  }
}


/**
* Prints a single channel.
*/
void MCP356x::printChannel(MCP356xChannel chan, StringBuilder* output) {
  output->concatf(
    "%s:\t%.6fv\t%s\n",
    CHAN_NAMES[((uint8_t) chan) & 0x0F],
    valueAsVoltage(chan),
    _channel_over_range(chan) ? "OvR" : " "
  );
}


/**
* Prints the values of all enabled channels.
*/
void MCP356x::printChannelValues(StringBuilder* output) {
  for (uint8_t i = 0; i < 16; i++) {
    MCP356xChannel chan = (MCP356xChannel) i;
    if (_scan_covers_channel(chan)) {
      switch (chan) {
        case MCP356xChannel::TEMP:
          output->concatf("Die temperature     = %.2fC\n", getTemperature());
          break;
        default:
          printChannel(MCP356xChannel::VCM, output);
          break;
      }
    }
  }
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

  switch (r) {
    case MCP356xRegister::CONFIG0:
      if (_mcp356x_flag(MCP356X_FLAG_USE_INTERNAL_CLK)) {
        _mcp356x_set_flag(MCP356X_FLAG_MCLK_RUNNING);
      }
      break;
    case MCP356xRegister::CONFIG1:
    case MCP356xRegister::CONFIG2:
      break;
    case MCP356xRegister::CONFIG3:
      if (!_mcp356x_flag(MCP356X_FLAG_INITIALIZED)) {
        // We mark the driver initialized on our first successful write to CONFIG3.
        _mcp356x_set_flag(MCP356X_FLAG_INITIALIZED);
        if (!_mclk_in_bounds()) {
          // The timing parameters of the ADC must be known to arrive at a linear model of
          //   the interrupt rate with respect to input clock. Then, we use the model to determine
          //   clock rate by watching the IRQ rate.
          if (0 == _write_register(MCP356xRegister::SCAN, 0)) {
            if (0 == _write_register(MCP356xRegister::MUX, 0xDE)) {
              _mcp356x_set_flag(MCP356X_FLAG_MEASURING_MCLK);
              // The first time through, we reset the read count so that we don't
              //   bake the ADC startup time into our clock calculation.
              resetReadCount();
            }
          }
        }
      }
      break;
    case MCP356xRegister::IRQ:
    case MCP356xRegister::MUX:
    case MCP356xRegister::SCAN:
    case MCP356xRegister::TIMER:
      break;
    case MCP356xRegister::OFFSETCAL:
      if (MCP356X_FLAG_ALL_CAL_MASK == (_mcp356x_flags() & MCP356X_FLAG_ALL_CAL_MASK)) {
        _mark_calibrated();
      }
      break;

    case MCP356xRegister::GAINCAL:
    case MCP356xRegister::RESERVED0:
    case MCP356xRegister::RESERVED1:
      break;
    case MCP356xRegister::LOCK:
      break;
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
  uint32_t reg_val = _get_shadow_value(r);
  int8_t ret = BUSOP_CALLBACK_NOMINAL;

  switch (r) {
    case MCP356xRegister::ADCDATA:
      millis_last_read = millis();
      if (!_mcp356x_flag(MCP356X_FLAG_MEASURING_MCLK)) {
        if (_discard_until_millis <= millis_last_read) {
          _normalize_data_register();
        }
      }
      read_count++;
      read_accumulator++;
      if (millis_last_read - millis_last_window >= 1000) {
        millis_last_window = millis_last_read;
        reads_per_second = read_accumulator;
        read_accumulator = 0;
        if (_mcp356x_flag(MCP356X_FLAG_MEASURING_MCLK)) {
          _mcp356x_clear_flag(MCP356X_FLAG_MEASURING_MCLK);
          _mclk_freq = _calculate_input_clock(1000000);
          _recalculate_clk_tree();
          _calibrate_offset();
        }
      }
      break;
    case MCP356xRegister::CONFIG0:
    case MCP356xRegister::CONFIG1:
    case MCP356xRegister::CONFIG2:
    case MCP356xRegister::CONFIG3:
      break;
    case MCP356xRegister::IRQ:
      _proc_irq_register();
      if (isr_fired) {   // If IRQ is still not disasserted, re-read the register.
        ret = BUSOP_CALLBACK_RECYCLE;
      }
      break;
    case MCP356xRegister::MUX:
    case MCP356xRegister::SCAN:
    case MCP356xRegister::TIMER:
    case MCP356xRegister::OFFSETCAL:
    case MCP356xRegister::GAINCAL:
      break;
    case MCP356xRegister::LOCK:
      break;
    case MCP356xRegister::RESERVED2:
      if (0x00900000 == _get_shadow_value(MCP356xRegister::RESERVED0)) {
        if (0x50 == (uint8_t) _get_shadow_value(MCP356xRegister::RESERVED1)) {
          switch ((uint8_t) _get_shadow_value(MCP356xRegister::RESERVED2)) {
            case 0x0C:
            case 0x0D:
            case 0x0F:
              _mcp356x_set_flag(MCP356X_FLAG_DEVICE_PRESENT);
              ret = 0;
              break;
            default:
              //Serial.println("bad RESERVED2 value.");
              break;
          }
        }
        //else Serial.println("bad RESERVED1 value.");
      }
      //else Serial.println("bad RESERVED0 value.");
      break;
    case MCP356xRegister::CRCCFG:
      break;
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
  SPIBusOp* op = (SPIBusOp*) _op;
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
    return BUSOP_CALLBACK_ERROR;
  }

  switch (first_param) {
    case 0x28:   // Fast command for start/restart conversion.
      break;
    case 0x38:   // Fast command for reset.
      sleep_ms(75);   // <--- Arbitrary delay value
      setPin(_CS_PIN, 0);
      setPin(_CS_PIN, 1);
      setPin(_CS_PIN, 0);
      setPin(_CS_PIN, 1);  // Twiddle the CS line to ensure SPI reset.
      if (0 == _clear_registers()) {
        sleep_ms(75);   // <--- Arbitrary delay value
        //Serial.print("MCP356x::io_op_callback() calling refresh.\n");
        refresh();
      }
      break;
    default:
      {   // This was register access.
        uint8_t reg_idx = first_param >> 2;
        if (BusOpcode::TX == op->get_opcode()) {
          ret = _proc_reg_write((MCP356xRegister) reg_idx);
        }
        else {
          if (_mcp356x_flag(MCP356X_FLAG_REFRESH_CYCLE)) {
            if (15 > reg_idx) {
              reg_idx++;
              op->setParams((uint8_t) _get_reg_addr((MCP356xRegister) reg_idx) | 0x01);
              op->setBuffer((uint8_t*) &_reg_shadows[reg_idx], MCP356x_reg_width[reg_idx]);
              ret = BUSOP_CALLBACK_RECYCLE;
            }
            else {   // This is the end of the refresh cycle.
              _mcp356x_clear_flag(MCP356X_FLAG_REFRESH_CYCLE);
              ret = _proc_reg_read(MCP356xRegister::RESERVED2);
              if (!adcConfigured()) {
                //Serial.print("MCP356x::io_op_callback() calling refresh.\n");
                _post_reset_fxn();
              }
            }
          }
          else {
            ret = _proc_reg_read((MCP356xRegister) reg_idx);
          }
        }
      }
      break;
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
  return _BUS->queue_io_job(op);
}
