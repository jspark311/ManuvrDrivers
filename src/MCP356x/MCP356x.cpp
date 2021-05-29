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
* Constructor specifying every setting.
*/
MCP356x::MCP356x(uint8_t irq_pin, uint8_t cs_pin, uint8_t mclk_pin, uint8_t addr, MCP356xConfig* CONF) :
  _IRQ_PIN(irq_pin), _CS_PIN(cs_pin), _MCLK_PIN(mclk_pin), _DEV_ADDR(addr),
  _desired_conf(CONF),
  _busop_irq_read(BusOpcode::RX, this, cs_pin, false),
  _busop_dat_read(BusOpcode::RX, this, cs_pin, false)
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




/*******************************************************************************
* High-level API functions
*******************************************************************************/

/**
* Send the fastcommand to bounce the ADC.
*
* @return
*    -1 if there was a problem writing the reset command.
*    0 if reset command was sent successfully.
*/
int8_t MCP356x::reset() {
  _clear_registers();
  int8_t ret = _send_fast_command(0x38);
  if (0 == ret) {
    _set_state(MCP356xState::RESETTING);
  }
  else {
    _set_fault("Failed to reset");
  }
  return ((0 == ret) ? 0 : -1);
}


/**
* Clears the class, assigns the SPIAdapter, and allows the state machine
*   to march forward.
*
* @return -1 on failure, 0 on success.
*/
int8_t MCP356x::init(SPIAdapter* b) {
  int8_t ret = -1;
  if (nullptr != b) {
    _BUS = b;
    _clear_registers();
    _busop_irq_read.setAdapter(_BUS);
    _busop_irq_read.shouldReap(false);
    _busop_irq_read.setParams((uint8_t) _get_reg_addr(MCP356xRegister::IRQ) | 0x01);
    _busop_irq_read.setBuffer((uint8_t*) &_reg_shadows[(uint8_t) MCP356xRegister::IRQ], 1);
    _busop_irq_read.maxFreq(19000000);
    _busop_irq_read.cpol(false);
    _busop_irq_read.cpha(false);

    _busop_dat_read.setAdapter(_BUS);
    _busop_dat_read.shouldReap(false);
    _busop_dat_read.setParams((uint8_t) _get_reg_addr(MCP356xRegister::ADCDATA) | 0x01);
    _busop_dat_read.setBuffer((uint8_t*) &_reg_shadows[(uint8_t) MCP356xRegister::ADCDATA], 4);
    _busop_dat_read.maxFreq(19000000);
    _busop_dat_read.cpol(false);
    _busop_dat_read.cpha(false);

    setOption(_desired_conf.flags);

    if (MCP356xState::UNINIT == _desired_state) {
      _desired_state = MCP356xState::READING;
    }
    _current_state = MCP356xState::PREINIT;
    _step_state_machine();
    ret = 0;
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
* TODO: Presently sets IRQ pin to be push-pull. So multiple instances of this
*   driver will require independant IRQ pins.
*
* @return
*   -1 on failure to write a register.
*   0  on success.
*/
int8_t MCP356x::_post_reset_fxn() {
  int8_t ret = -1;
  uint32_t c0_val = 0x000000C3;

  // Enable register write.
  ret = _write_register(MCP356xRegister::LOCK, 0x000000A5);
  if (0 == ret) {
    // Enable fast command, disable IRQ on conversion start, IRQ pin is push-pull.
    ret = _write_register(MCP356xRegister::IRQ, 0x00000006);
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
  }
  return ret;
}


/**
* Reads the part's output register and performs the needed manipulation on the
*   data, according to the current configuration.
* NOTE: If the application recently called discardUnsettledSamples(), the register
*   shadow will not be updated unless the timeout has expired. But no matter what,
*   the true value of the register will be returned from this function. So if
*   the application wants to know if _valid_ data was read, it should call
*   newValue() or scanComplete() ahead of using it.
*
* @return
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
    else {
      // We assume that MCLK frequency has been declared by fiat.
      // TODO: There is no need for this if we can measure the data rate.
      ret = _recalculate_clk_tree();
      _mcp356x_set_flag(MCP356X_FLAG_MCLK_RUNNING);
    }
    _mcp356x_set_flag(MCP356X_FLAG_PINS_CONFIGURED);
  }
  if (-1 == ret) {
    _set_fault("_ll_pin_init() failed");
  }
  return ret;
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
  int8_t  ret = 0;
  _mcp356x_set_flag(MCP356X_FLAG_REFRESH_CYCLE);
  ret = _read_register(MCP356xRegister::ADCDATA);

  if (0 != ret) {
    _set_fault("Failed to refresh()");
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
    _desired_conf.scan = chans;
    if (adcCalibrated()) {
      ret = _set_scan_channels(chans);
    }
    else {
      ret = 0;
    }
  }
  return ret;
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


/**
* Reads the ADC channels that assist us with calibration.
* Saves the existing channel settings, before changing them.
*
* @return
*   -1 if switching to the calibration channels failed
*   0 on success.
*/
int8_t MCP356x::calibrate() {
  int8_t ret = _set_scan_channels(0x0000E000);
  _mcp356x_clear_flag(MCP356X_FLAG_CALIBRATED | MCP356X_FLAG_ALL_CAL_MASK);
  if (0 == ret) {
    _set_state(MCP356xState::CALIBRATION);
  }
  else {
    _set_fault("Failed to start calibration");
  }
  return ret;
}



/*******************************************************************************
* Hardware discovery functions
*******************************************************************************/

/**
* The valid ADC input clock is between 0.1 and 20 MHz.
* Technically, there is no lower bound on MCLK given in the datasheet. But our
*   temporal resolution in the calibration phase of the FSM takes too long for
*   a decent reading without some lower-bound.
*
* @return true if MCLK (as measured) if within operational boundaries.
*/
bool MCP356x::_mclk_in_bounds() {
  return ((_mclk_freq >= 100000.0) && (_mclk_freq <= 20000000.0));
}


/**
* Some designs drive the ADC from an on-board high-Q oscillator. But there is
*   no direct firmware means to discover the setting.
* This function discovers the frequency by timing ADC reads with known clocking parameters
*   and reports the result; storing the answer in the class variable _mclk_freq.
*
* @return
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
          _local_log.concatf("Took %u samples in %luus.\n", rcount, micros_passed);
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

  if (0 == setGain(_desired_conf.gain)) {
    if (0 == setBiasCurrent(_desired_conf.bias)) {
      if (0 == setAMCLKPrescaler(_desired_conf.prescaler)) {
        if (0 == setOversamplingRatio(_desired_conf.over)) {
          if (0 == _set_scan_channels(_desired_conf.scan)) {
            ret++;
            _mcp356x_set_flag(MCP356X_FLAG_CALIBRATED);
          }
        }
      }
    }
  }
  return ret;
}



/*******************************************************************************
* State machine parts
*******************************************************************************/

/**
* This is NOT a polling loop. It doesn't check for valid conditions for
*   advancing to a given state. It only chooses and imparts the next state.
*
* @return
*   -1 on error
*   0 on nominal polling with stable state achieved
*   1 on unstable state with no advancement on this call
*   2 on advancement of current state toward desired state
*/
int8_t MCP356x::_step_state_machine() {
  int8_t ret = 0;
  if (!stateStable()) {
    // Check for globally-accessible desired states early so we don't repeat
    //   ourselves in several case blocks later on.
    bool continue_looping = true;
    switch (_desired_state) {
      case MCP356xState::RESETTING:
        ret = (0 == reset()) ? 2 : -1;
        break;
      default:
        break;
    }

    ret++;
    while (continue_looping) {
      continue_looping = false;   // Abort the loop by default.

      switch (_current_state) {
        case MCP356xState::PREINIT:
          // Regardless of where we are going, we only have one way out of here.
          // Check for memory allocation, pin control
          if (0 == _ll_pin_init()) {  // Configure the pins if they are not already.
            ret = (0 == reset()) ? 2 : -1;
          }
          else {
            ret = -1;
          }
          break;

        case MCP356xState::RESETTING:
          // Regardless of where we are going, we only have one way out of here.
          // If we were given one, check that the IRQ pin pulsed.
          if (255 != _IRQ_PIN) {
            sleep_ms(150);   // TODO: Wrong
            if (0 == refresh()) {
              _set_state(MCP356xState::DISCOVERY);
              ret = 2;
            }
            else {
              ret = -1;
            }
          }
          else {
            // Otherwise, observe a delay.
            sleep_ms(75);   // <--- Arbitrary delay value
            if (0 == refresh()) {
              _set_state(MCP356xState::DISCOVERY);
              ret = 2;
            }
            else {
              ret = -1;
            }
          }
          break;

        case MCP356xState::DISCOVERY:
          if (adcFound()) {
            if (0 == _post_reset_fxn()) {
              _set_state(MCP356xState::REGINIT);
              ret = 2;
            }
            else {
              _set_fault("_post_reset_fxn() failed");
              ret = -1;
            }
          }
          else {
            _set_fault("Failed to find MCP356x");
            ret = -1;
          }
          break;

        case MCP356xState::REGINIT:
          if (!_mclk_in_bounds()) {
            // If register init completed, and we don't think we have a
            //   valid clock, try to measure it.
            // The timing parameters of the ADC must be known to arrive at a linear model of
            //   the interrupt rate with respect to input clock. Then, we use the model to determine
            //   clock rate by watching the IRQ rate.
            if (0 == _write_register(MCP356xRegister::SCAN, 0)) {
              if (0 == _write_register(MCP356xRegister::MUX, 0xDE)) {
                _set_state(MCP356xState::CLK_MEASURE);
                _local_log.concat("MCP356x::_proc_reg_write() MEASURING_MCLK...\n");
                // The first time through, we reset the read count so that we don't
                //   bake the ADC startup time into our clock calculation.
                resetReadCount();
                ret = 2;
              }
            }

            if (2 != ret) {
              _set_fault("Failed to start clock measurement");
              ret = -1;
            }
          }
          else if (!adcCalibrated()) {
            // If the clock is good, but the driver isn't calibrated, do that.
            ret = (0 == calibrate()) ? 2 : -1;
          }
          else {
            // If a re-init cycle happened after the clock and cal steps, jump
            //   right to reading.
            _mcp356x_set_flag(MCP356X_FLAG_INITIALIZED);
            switch (_desired_state) {
              case MCP356xState::IDLE:     _set_state(MCP356xState::IDLE);     break;
              case MCP356xState::READING:  _set_state(MCP356xState::READING);  break;
              default:                     _set_state(MCP356xState::IDLE);     break;
            }
            ret = 2;
          }
          break;

        case MCP356xState::CLK_MEASURE:
          if (_mclk_in_bounds()) {
            if (!adcCalibrated()) {
              ret = (0 == calibrate()) ? 2 : -1;
            }
            else {
              _mcp356x_set_flag(MCP356X_FLAG_INITIALIZED);
              switch (_desired_state) {
                case MCP356xState::IDLE:     _set_state(MCP356xState::IDLE);     break;
                case MCP356xState::READING:  _set_state(MCP356xState::READING);  break;
                default:                     _set_state(MCP356xState::IDLE);     break;
              }
            }
            ret = 2;
          }
          else {
            _set_fault("Failed to measure MCLK");
            ret = -1;
          }
          break;

        case MCP356xState::CALIBRATION:
          if (adcCalibrated()) {
            _mcp356x_set_flag(MCP356X_FLAG_INITIALIZED);
            switch (_desired_state) {
              case MCP356xState::IDLE:     _set_state(MCP356xState::IDLE);     break;
              case MCP356xState::READING:  _set_state(MCP356xState::READING);  break;
              default:                     _set_state(MCP356xState::IDLE);     break;
            }
            ret = 2;
          }
          //else {
          //  _set_fault("Failed to calibrate");
          //  ret = -1;
          //}
          break;

        case MCP356xState::IDLE:
          switch (_desired_state) {
            case MCP356xState::CALIBRATION:
              break;
            case MCP356xState::IDLE:
              break;
            case MCP356xState::READING:
              //if () {  // If the ADC is in one-shot mode, initiate a conversion cycle.
              //}
              //else {
              //}
              break;
            default:
              _set_fault("Illegal _desired_state");
              ret = -1;
              break;
          }
          break;
        case MCP356xState::READING:
          switch (_desired_state) {
            case MCP356xState::REGINIT:
              break;
            case MCP356xState::CALIBRATION:
              break;
            case MCP356xState::IDLE:
              break;
            case MCP356xState::READING:
              break;
            case MCP356xState::FAULT:
              _set_fault("Fault entered by outside caller.");
              ret = 2;
              break;
            default:
              _set_fault("Illegal _desired_state");
              ret = -1;
              break;
          }
          break;

        case MCP356xState::UNINIT:  // We can't step our way into this mess. We need to call init().
        case MCP356xState::FAULT:   // We can't step our way out of this mess. We need to be reset().
          break;
        default:
          _set_fault("Illegal _current_state");
          ret = -1;
          break;
      }
    }
  }
  return ret;
}


/**
* Only two cases should not set _current_state by calling this function.
*   1) set_fault(msg);
*   2) Exit from FAULT;
*
* @param e The state that should be stored in _current_state.
* @return 0 always
*/
void MCP356x::_set_state(MCP356xState e) {
  _local_log.concatf("MCP356x::_set_state()  %s --> %s\n", stateStr(_current_state), stateStr(e));
  switch (e) {
    case MCP356xState::PREINIT:
    case MCP356xState::RESETTING:
    case MCP356xState::DISCOVERY:
    case MCP356xState::REGINIT:
    case MCP356xState::CLK_MEASURE:
    case MCP356xState::CALIBRATION:
    case MCP356xState::IDLE:
    case MCP356xState::READING:
      _prior_state = _current_state;
      _current_state = e;
      break;
    case MCP356xState::FAULT:
      _set_fault("Fault entry by outside caller.");
      break;
    case MCP356xState::UNINIT:
    default:
      _set_fault("_set_state(): Illegal state");
      break;
  }
}


/**
* Put the driver into a FAULT state.
*
* @param msg is a debug string to be added to the log.
*/
void MCP356x::_set_fault(const char* msg) {
  _local_log.concatf("MCP356x fault: %s\n", msg);
  _prior_state = _current_state;
  _current_state = MCP356xState::FAULT;
}
