/*
File:   MCP356x.cpp
Author: J. Ian Lindsay
*/

#include "MCP356x.h"

#define MCP356X_FSM_WAYPOINT_DEPTH  12

const char* const LOCAL_LOG_TAG = "MCP356x";

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


// We can have up to two of these in a given system.
// TODO: AbstractPlatform needs a way to call specific objects from setPinFxn()
//   to avoid these sorts of pseudo-singleton patterns. ISR function scope.
#define MCP356X_MAX_INSTANCES    2
volatile static MCP356x* INSTANCES[MCP356X_MAX_INSTANCES] = {0};

/* This is an ISR. */
void mcp356x_isr0() {  ((MCP356x*) INSTANCES[0])->isr_fxn();  }

/* This is an ISR. */
void mcp356x_isr1() {  ((MCP356x*) INSTANCES[1])->isr_fxn();  }


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
MCP356x::MCP356x(const uint8_t irq_pin, const uint8_t cs_pin, const uint8_t mclk_pin, const uint8_t addr, MCP356xConfig* CONF) :
  StateMachine<MCP356xState>("MCP356x-FSM", &MCP356x::_FSM_STATES, MCP356xState::UNINIT, MCP356X_FSM_WAYPOINT_DEPTH),
  _IRQ_PIN(irq_pin), _CS_PIN(cs_pin), _MCLK_PIN(mclk_pin), _DEV_ADDR(addr),
  _desired_conf((MCP356xConfig*) CONF),
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
  // This will almost certainly NEVER be called. It is can be discarded with
  //   build options.
  #if !defined(CONFIG_C3P_OMIT_DRIVER_DESTRUCTORS)
    if (255 != _IRQ_PIN) {
      unsetPinFxn(_IRQ_PIN);
    }
    INSTANCES[_slot_number] = nullptr;
  #endif  // CONFIG_C3P_OMIT_DRIVER_DESTRUCTORS
}



/*******************************************************************************
* High-level API functions
*******************************************************************************/

/**
* Plan the state route to reset the ADC.
*
* @return 0 on success
*        -1 on insufficient driver state
*        -2 on state not stable
*        -3 on state planning failure
*/
int8_t MCP356x::reset() {
  int8_t ret = -1;
  bool allow_reset = false;
  switch (currentState()) {
    case MCP356xState::FAULT:   // We allow this if pin setup previously finished.
      allow_reset = _flags.value(MCP356X_FLAG_PINS_CONFIGURED);
      break;
    case MCP356xState::UNINIT:  // No pin setup has happened.
      break;
    default:
      allow_reset = true;
      break;
  }
  if (allow_reset) {
    ret--;
    if (_fsm_is_stable()) {
      ret--;
      // TODO: Consider CAL state, etc...
      if (0 == _fsm_set_route(2, MCP356xState::RESETTING, MCP356xState::IDLE)) {
        ret = 0;
      }
    }
  }
  return ret;
}


/**
* Clears the class, (possibly) assigns the SPIAdapter, and allows the state
*   machine to march forward.
*
* @return -1 on failure, 0 on success.
*/
int8_t MCP356x::init(SPIAdapter* b) {
  int8_t ret = -1;
  if (nullptr != b) {  _BUS = b;  }

  _clear_registers();
  setOption(MCP356X_FLAG_DRIVER_OPTS_MASK & _desired_conf->flags);

  if (0 == _fsm_set_route(7, MCP356xState::PRE_INIT, MCP356xState::RESETTING, MCP356xState::DISCOVERY,  MCP356xState::POST_INIT, MCP356xState::CLK_MEASURE, MCP356xState::CALIBRATION, MCP356xState::USR_CONF)) {
    if (_flags.value(MCP356X_FLAG_GENERATE_MCLK)) {
      // TODO: Isolate CLK measurement selection.
    }
    MCP356xState next_state = (MCP356xMode::STANDBY == _desired_conf->mode) ? MCP356xState::IDLE : MCP356xState::READING;
    _fsm_append_state(next_state);
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
  if (flgs & MCP356X_FLAG_USE_INTRNL_CLK) {
    if (!(_flags.value(MCP356X_FLAG_PINS_CONFIGURED))) {
      // Only allow this change if the pins are not yet configured.
      _flags.set(MCP356X_FLAG_USE_INTRNL_CLK);
      _flags.clear(MCP356X_FLAG_GENERATE_MCLK);
    }
    else {
      ret = -1;
    }
  }
  if (flgs & MCP356X_FLAG_GENERATE_MCLK) {
    if (!(_flags.value(MCP356X_FLAG_PINS_CONFIGURED))) {
      // Only allow this change if the pins are not yet configured.
      _flags.set(MCP356X_FLAG_GENERATE_MCLK);
      _flags.clear(MCP356X_FLAG_USE_INTRNL_CLK);
    }
    else {
      ret = -1;
    }
  }
  if (flgs & MCP356X_FLAG_USE_INTRNL_VREF) {
    _flags.set(MCP356X_FLAG_USE_INTRNL_VREF);
  }
  if (flgs & MCP356X_FLAG_3RD_ORDER_TEMP) {
    _flags.set(MCP356X_FLAG_3RD_ORDER_TEMP);
  }
  return ret;
}


/**
* Given the ADC channel, walks the value backward through the ADC transfer
*   function to arrive at the voltage on that channel.
* If the application didn't set a reference voltage, we assume it is equal to
*   AVdd. If that channel has never been read, the value will default to 3.3v.
* NOTE: If the application wants to know if _valid_ data was read, it should
*   call newValue() or scanComplete() ahead of using it.
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
* NOTE: If the application wants to know if _valid_ data was read, it should
*   call newValue() or scanComplete() ahead of using it.
*
* @param chan The ADC channel in question.
* @return The channel's raw value.
*/
int32_t MCP356x::value(MCP356xChannel chan) {
  _channel_clear_new_flag(chan);
  return channel_vals[(uint8_t) chan & 0x0F];
}


/**
* Causes the ADC to throw away samples after reading them. This should be
*   invoked when it is known that the analog value is changing, and we only want
*   reports of fully-settled values.
*/
void MCP356x::discardUnsettledSamples() {
  _discard_window.reset((_circuit_settle_ms + getSettlingTime()) * 1000);
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
    _desired_conf->scan = chans;
    ret = 0;
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
  _flags.set(MCP356X_FLAG_VREF_DECLARED);
  return 0;
}


uint16_t MCP356x::getSampleRate() {
  if (0 < _profiler_result_read.executions()) {
    return (uint16_t) (1000000UL / _profiler_result_read.meanTime());
  }
  return 0;
}


/**
* Public-facing API to cause the driver to undergo a calibration cycle.
* NOTE: This is intended for manual re-calibration, and can only be called on a
*   driver that has already initialized. The calibration cycle is already
*   handled during the init flow.
*
* @return 0 on success
*        -1 if the FSM is not stable
*        -2 if the existing FSM state is not amiable to a re-calibration.
*/
int8_t MCP356x::calibrate() {
  int8_t ret = -1;
  if (_fsm_is_stable()) {
    switch (currentState()) {
      case MCP356xState::IDLE:
      case MCP356xState::READING:
        // This is a request to dip into a recalibration tangent in the FSM. Value
        //   collection will stall, but should pick up where it left off.
        _flags.clear(MCP356X_FLAG_ALL_CAL_MASK);
        ret = 0;
        break;
      default:
        ret--;
        break;
    }
  }
  return ret;
}


int8_t MCP356x::readSamples(int32_t scan_count) {
  int8_t ret = -1;
  if (_fsm_is_stable()) {
    ret--;
    switch (currentState()) {
      case MCP356xState::IDLE:
        _fsm_append_state(MCP356xState::READING);
        break;
      case MCP356xState::READING:
        _fsm_append_state(MCP356xState::READING);
        ret = 0;
        break;
      default:
        ret--;
        break;
    }
  }
  if (0 == ret) {
    _scans_req = scan_count;
  }
  return ret;
}


void MCP356x::isr_fxn() {
  _irqs_noted++;
  _profiler_irq_timing.markStart();
  _isr_fired = true;
}



/*******************************************************************************
* Configuration handling
*******************************************************************************/

int8_t MCP356x::setGain(MCP356xGain x) {
  switch (x) {
    case MCP356xGain::GAIN_ONETHIRD:
    case MCP356xGain::GAIN_1:
    case MCP356xGain::GAIN_2:
    case MCP356xGain::GAIN_4:
    case MCP356xGain::GAIN_8:
    case MCP356xGain::GAIN_16:
    case MCP356xGain::GAIN_32:
    case MCP356xGain::GAIN_64:
      _desired_conf->gain = x;
      return 0;
    default:  break;
  }
  return -1;
}

int8_t MCP356x::setBiasCurrent(MCP356xBiasCurrent x) {
  switch (x) {
    case MCP356xBiasCurrent::NONE:
    case MCP356xBiasCurrent::NANOAMPS_900:
    case MCP356xBiasCurrent::NANOAMPS_3700:
    case MCP356xBiasCurrent::NANOAMPS_15000:
      _desired_conf->bias = x;
      return 0;
    default:  break;
  }
  return -1;
}

int8_t MCP356x::setOversamplingRatio(MCP356xOversamplingRatio x) {
  switch (x) {
    case MCP356xOversamplingRatio::OSR_32:
    case MCP356xOversamplingRatio::OSR_64:
    case MCP356xOversamplingRatio::OSR_128:
    case MCP356xOversamplingRatio::OSR_256:
    case MCP356xOversamplingRatio::OSR_512:
    case MCP356xOversamplingRatio::OSR_1024:
    case MCP356xOversamplingRatio::OSR_2048:
    case MCP356xOversamplingRatio::OSR_4096:
    case MCP356xOversamplingRatio::OSR_8192:
    case MCP356xOversamplingRatio::OSR_16384:
    case MCP356xOversamplingRatio::OSR_20480:
    case MCP356xOversamplingRatio::OSR_24576:
    case MCP356xOversamplingRatio::OSR_40960:
    case MCP356xOversamplingRatio::OSR_49152:
    case MCP356xOversamplingRatio::OSR_81920:
    case MCP356xOversamplingRatio::OSR_98304:
      _desired_conf->over = x;
      return 0;
    default:  break;
  }
  return -1;
}


int8_t MCP356x::setAMCLKPrescaler(MCP356xAMCLKPrescaler x) {
  switch (x) {
    case MCP356xAMCLKPrescaler::OVER_1:
    case MCP356xAMCLKPrescaler::OVER_2:
    case MCP356xAMCLKPrescaler::OVER_4:
    case MCP356xAMCLKPrescaler::OVER_8:
      _desired_conf->prescaler = x;
      return 0;
    default:  break;
  }
  return -1;
}


int8_t MCP356x::readMode(MCP356xMode x) {
  switch (x) {
    case MCP356xMode::ONESHOT_SHUTDOWN:
    case MCP356xMode::STANDBY:
    case MCP356xMode::ONESHOT_STANDBY:
    case MCP356xMode::CONTINUOUS:
      _desired_conf->mode = x;
      return 0;
    default:  break;
  }
  return -1;
}


/**
* Calling this function will cause the user's desired configuration to be
*   written to the registers. It may be necessary to call this several times
*   to achieve complete configuration. So call it until it returns 0.
*
* @return 1 on success with pending I/O
*         0 on success with no changes
*        -1 on failure.
*/
int8_t MCP356x::_apply_usr_config() {
  _current_conf = MCP356xConfig(_desired_conf);
  int8_t ret = -1;
  if (0 == _set_gain(_current_conf.gain)) {
    if (0 == _set_bias_current(_current_conf.bias)) {
      if (0 == _set_amlclk_prescaler(_current_conf.prescaler)) {
        if (0 == _set_oversampling_ratio(_current_conf.over)) {
          if (0 == _set_scan_channels(_current_conf.scan)) {
            ret = 0;
          }
        }
      }
    }
  }
  return ret;
}


/*
* NOTE: This function ignores the mode value on purpose.
*/
bool MCP356x::_config_is_written() {
  bool ret = true;
  ret &= (getGain() == _current_conf.gain);
  ret &= (getBiasCurrent() == _current_conf.bias);
  ret &= (getAMCLKPrescaler() == _current_conf.prescaler);
  ret &= (getOversamplingRatio() == _current_conf.over);
  ret &= (_get_shadow_value(MCP356xRegister::SCAN) == _current_conf.scan);
  return ret;
}


/*
* NOTE: This function ignores the mode value on purpose.
*/
bool MCP356x::_config_is_desired() {
  bool ret = true;
  ret &= (_desired_conf->gain == _current_conf.gain);
  ret &= (_desired_conf->bias == _current_conf.bias);
  ret &= (_desired_conf->prescaler == _current_conf.prescaler);
  ret &= (_desired_conf->over == _current_conf.over);
  ret &= (_get_shadow_value(MCP356xRegister::SCAN) == _desired_conf->scan);
  return ret;
}


/*******************************************************************************
* State machine parts
*******************************************************************************/

/*
* Only returns true if in "true idle".
* State is IDLE, No I/O in-flight, and no further states planned.
*/
FAST_FUNC bool MCP356x::isIdle() {
  switch (currentState()) {
    case MCP356xState::IDLE:  return (!io_in_flight() & _fsm_is_stable());
    default:                  return false;
  }
}

/**
*
*
* @return
*   -1 on error
*   0 on nominal polling with stable state achieved
*   1 on unstable state with no advancement on this call
*   2 on advancement of current state toward desired state
*/
FAST_FUNC PollResult MCP356x::poll() {
  if (io_in_flight()) {  return PollResult::NO_ACTION;  }  // Bailout clause

  // The driver handles IRQs first. And that is contingent on mode.
  if (_isr_fired && _servicing_irqs()) {
    // If the driver knows the hardware is present, and the IRQ pin demands
    //   service, read the status registers.
    if (_busop_irq_read.hasFault()) {
      // If there was a bus fault, the BusOp might be left in an unqueuable state.
      // Try to reset the BusOp to satisfy the caller.
      _busop_irq_read.markForRequeue();
    }
    if (_busop_irq_read.isIdle()) {
      if (0 == _BUS->queue_io_job(&_busop_irq_read, _bus_priority)) {
        _io_dispatched++;
      }
    }
  }

  // Always poll the FSM. Considering we only got this far because there no open
  //   I/O, if we have it now, it implies action. If the FSM advanced state, but
  //   there is no I/O pending, it might mean we can advance again with an
  //   immediate repoll.
  int8_t ret = _fsm_poll();
  if (0 == ret) {      return (io_in_flight() ? PollResult::ACTION : PollResult::NO_ACTION);  }
  else if (ret > 0) {  return (io_in_flight() ? PollResult::ACTION : PollResult::REPOLL);     }
  else {               return PollResult::ERROR;  }
}


/**
* Called in idle time by the firmware to prod the driver's state machine forward.
* Considers the current driver state, and decides whether or not to advance the
*   driver's state machine.
* NOTE: This function does not plan state machine routes, and should thus not
*   call _fsm_set_position() directly. Only _advance_state_machine().
*
* @return  1 on state shift
*          0 on no action
*         -1 on error
*/
FAST_FUNC int8_t MCP356x::_fsm_poll() {
  int8_t ret = 0;
  bool fsm_advance = false;
  switch (currentState()) {
    // Exit conditions: The next state is PRE_INIT.
    case MCP356xState::UNINIT:
      fsm_advance = _fsm_is_next_pos(MCP356xState::PRE_INIT);
      break;

    // Exit conditions: Unconditional. If we're here, we are clear to proceed.
    case MCP356xState::PRE_INIT:
      fsm_advance = true;
      break;

    // Exit conditions: The IRQ pin indicates that the chip has completed its
    //   reset cycle (if we were given one). Otherwise, we blindly advance.
    case MCP356xState::RESETTING:
      fsm_advance = ((255 == _IRQ_PIN) | (0 < _irqs_noted));
      //fsm_advance = true;  // TODO: For some reason, we miss the IRQ...
      break;

    // Exit conditions: A compatible device was found by register refresh.
    case MCP356xState::DISCOVERY:
      fsm_advance = adcFound();
      // TODO: Need fault timeout logic to makew this work again. It was
      //   synchronous via sleep before the FSM toss-up.
      //_set_fault("Failed to find MCP356x");
      //ret = -1;
      break;

    // Exit conditions: Configuration for clock calibration has been imparted.
    case MCP356xState::POST_INIT:
      fsm_advance = (!_flags.value(MCP356X_FLAG_STATE_HOLD));
      break;

    // Exit conditions: Input clock has been measured, and timing calibrated.
    case MCP356xState::CLK_MEASURE:
      if (1000 < _irqs_noted) {
        const uint32_t MEASURE_TIME_MICROS = (uint32_t) micros_since(micros_last_window);
        _mclk_freq = _calculate_input_clock(MEASURE_TIME_MICROS);
        fsm_advance = (0 == _recalculate_clk_tree());
      }
      break;

    // Exit conditions: The driver and the hardware are both calibrated.
    case MCP356xState::CALIBRATION:
      fsm_advance = adcCalibrated();
      break;

    // Exit conditions: The intended operating configuration has been imparted.
    case MCP356xState::USR_CONF:
      fsm_advance = _config_is_written();
      if (fsm_advance) {
        if (_fsm_is_stable()) {
          // The next state is undefined. The FSM won't abide dwelling here, so
          //   we choose what state to advance into based on the config that was
          //   written.
          MCP356xState next_state = (MCP356xMode::STANDBY == _desired_conf->mode) ? MCP356xState::IDLE : MCP356xState::READING;
          fsm_advance = (0 == _fsm_append_state(next_state));
          if (!fsm_advance) {
            c3p_log(LOG_LEV_WARN, LOCAL_LOG_TAG, "FAILED TO do");
          }
        }
      }
      break;

    // Exit conditions: These states are canonically stable. So we advance when
    //   the state is not stable (the driver has somewhere else it wants to be).
    case MCP356xState::IDLE:
    case MCP356xState::READING:
      fsm_advance = !_fsm_is_stable();
      if (!fsm_advance) {
        // No new place to be, right away.
        // Check for a calibration request. We will know because one or more
        //   of the cal flags have been cleared.
        if (!_flags.all_set(MCP356X_FLAG_ALL_CAL_MASK)) {
          fsm_advance = (0 == _fsm_prepend_state(MCP356xState::CALIBRATION));
        }
      }
      if (!fsm_advance) {
        // Still no change.
        // Check to see if the user changed the desired configuration.
        if (!_config_is_written()) {
          fsm_advance = (0 == _fsm_prepend_state(MCP356xState::USR_CONF));
        }
      }
      break;

    // We can't step our way out of this mess. We need to be init().
    case MCP356xState::FAULT:       // If the driver is in a FAULT, do nothing.
      break;
    default:   // Can't exit from an unknown state.
      ret = -1;
      break;
  }

  // If the current state's exit criteria is met, we advance the FSM.
  if (fsm_advance & (-1 != ret)) {
    ret = (0 == _fsm_advance()) ? 1 : 0;
  }
  return ret;
}


/**
* Takes actions appropriate for entry into the given state, and sets the current
*   FSM position if successful. Records the existing state as having been the
*   prior state.
* NOTE: Except in edge-cases (reset, init, etc), this function should ONLY be
*   called by _advance_state_machine().
*
* @param The FSM code to test.
* @return 0 on success, -1 otherwise.
*/
FAST_FUNC int8_t MCP356x::_fsm_set_position(MCP356xState new_state) {
  int8_t ret = -1;
  if (_fsm_is_waiting()) return ret;
  bool state_entry_success = false;   // Succeed by default.
  switch (new_state) {
    // We can't step our way into this mess. We need to call init().
    case MCP356xState::UNINIT:
      _set_fault("Tried to _fsm_set_position(UNINIT)");
      break;

    // Entry into PRE_INIT means that the GPIO pins are initiallized, the
    //   SPIAdapter is defined, and any private BusOps are ready for use.
    case MCP356xState::PRE_INIT:
      if (nullptr != _BUS) {
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
        state_entry_success = (0 == _ll_pin_init());
        _io_dispatched  = 0;
        _io_called_back = 0;
        if (!state_entry_success) {
          _set_fault("_ll_pin_init() failed");
        }
      }
      break;

    // Entry into RESET means we didn't fail to reset the part.
    case MCP356xState::RESETTING:
      state_entry_success = (0 == _reset_fxn());
      if (state_entry_success) {
        if (255 == _IRQ_PIN) {     // If we were not given an IRQ pin,
          _fsm_lockout(75);        //   observe an arbitrary delay value.
        }
      }
      else {
        _set_fault("_reset_fxn() failed");
        ret = -1;
      }
      break;

    // Entry into DISCOVERY means we dispatched I/O that will mirror the hardware
    //   register states.
    case MCP356xState::DISCOVERY:
      state_entry_success = (0 == refresh());
      if (!state_entry_success) {
        _set_fault("refresh() failed");
        ret = -1;
      }
      break;

    // Entry into POST_INIT means we didn't fail to reset the part.
    case MCP356xState::POST_INIT:
      state_entry_success = (0 == _post_reset_fxn());
      if (!state_entry_success) {
        _set_fault("_post_reset_fxn() failed");
        ret = -1;
      }
      break;

    case MCP356xState::CLK_MEASURE:
      // If we don't think we have a valid clock, try to measure it.
      // The timing parameters of the ADC must be known to arrive at a linear
      //   model of the interrupt rate with respect to input clock. Then, we
      //   use the model to determine clock rate by watching the IRQ rate.
      // Since a non-zero value in the SCAN register adds padding to
      //   timing, we disable this ability, and use the MUX register to
      //   dwell on the temperature diode.
      if (0 == _write_register(MCP356xRegister::SCAN, 0)) {
        if (0 == _write_register(MCP356xRegister::MUX, 0xDE)) {
          if (0 == _set_oversampling_ratio(_desired_conf->over)) {
            uint32_t c0_val = _get_shadow_value(MCP356xRegister::CONFIG0);
            if (0 == _write_register(MCP356xRegister::CONFIG0, (0x03 | c0_val))) {
              _mclk_freq = 0.0d;
              _irqs_noted = 0;
              micros_last_window = micros();
              state_entry_success = true;
            }
          }
        }
      }
      if (!state_entry_success) {
        _set_fault("Failed to start clock measurement");
        ret = -1;
      }
      break;

    case MCP356xState::CALIBRATION:
      state_entry_success = (0 == _calibrate());
      if (!state_entry_success) {
        _set_fault("_calibrate() failed");
        ret = -1;
      }
      break;

    case MCP356xState::USR_CONF:
      state_entry_success = (0 == _apply_usr_config());
      if (!state_entry_success) {
        _set_fault("_apply_usr_config() failed");
        ret = -1;
      }
      break;

    // Entry to IDLE always succeeds.
    case MCP356xState::IDLE:
      state_entry_success = true;
      break;

    // Entry to READING succeeds if the config bits indite it is so, or a
    //   register write to make it so succeeds,
    case MCP356xState::READING:
    state_entry_success = true;
      state_entry_success = (0 == _set_read_mode(_desired_conf->mode));
      if (!state_entry_success) {
        _set_fault("_set_read_mode(READING) failed");
        ret = -1;
      }
      break;

    // We allow fault entry to be done this way.
    case MCP356xState::FAULT:
      state_entry_success = true;
      _fsm_mark_current_state(MCP356xState::FAULT);
      break;

    default:
      c3p_log(LOG_LEV_ALERT, LOCAL_LOG_TAG, "_fsm_set_position(%s) is unhandled.", _FSM_STATES.enumStr(new_state));
      _set_fault("Unhandled MCP356xState");
      break;
  }

  if (state_entry_success) {
    c3p_log(LOG_LEV_NOTICE, LOCAL_LOG_TAG, "MCP356x State %s ---> %s", _FSM_STATES.enumStr(currentState()), _FSM_STATES.enumStr(new_state));
    ret = 0;
  }
  return ret;
}


/**
* Setup the low-level pin details. Execution is idempotent.
*
* @return
*   -1 if the pin setup is wrong. Class must halt.
*   0  if the pin setup is complete.
*/
int8_t MCP356x::_ll_pin_init() {
  int8_t ret = -1;
  if (_flags.value(MCP356X_FLAG_PINS_CONFIGURED)) {
    ret = 0;
  }
  else if ((255 != _CS_PIN) & (255 != _IRQ_PIN)) {
    ret = 0;
    pinMode(_CS_PIN, GPIOMode::OUTPUT);
    setPin(_CS_PIN, 1);
    pinMode(_IRQ_PIN, GPIOMode::INPUT);
    switch (_slot_number) {  // TODO: This is terrible. You know better.
      case 0:  setPinFxn(_IRQ_PIN, IRQCondition::FALLING, mcp356x_isr0);  break;
      case 1:  setPinFxn(_IRQ_PIN, IRQCondition::FALLING, mcp356x_isr1);  break;
    }
    if (255 != _MCLK_PIN) {
      // If we have MCLK, we need to generate a squarewave on that pin.
      // Otherwise, we hope that the board has an XTAL attached.
      if (_flags.value(MCP356X_FLAG_USE_INTRNL_CLK)) {
        // TODO: We presently do nothing with this signal. But we might tap it
        //   for frequency measurement of the internal OSC.
        pinMode(_MCLK_PIN, GPIOMode::INPUT);
      }
      else {
        if (_flags.value(MCP356X_FLAG_GENERATE_MCLK)) {
          // NOTE: Not all pin support this. Works for some pins on some MCUs.
          //pinMode(_MCLK_PIN, GPIOMode::ANALOG_OUT);
          //analogWriteFrequency(_MCLK_PIN, 4915200);
          //analogWrite(_MCLK_PIN, 128);
          //_mclk_freq = 4915200.0;
          _flags.set(MCP356X_FLAG_MCLK_RUNNING);
          _recalculate_clk_tree();
        }
        else {
          // There is a hardware oscillator whose enable pin we control with
          //   the MCLK pin. Set the pin high (enabled) and measure the clock.
          pinMode(_MCLK_PIN, GPIOMode::OUTPUT);
          _flags.set(MCP356X_FLAG_MCLK_RUNNING);
          setPin(_MCLK_PIN, 1);
        }
      }
    }
    _flags.set(MCP356X_FLAG_PINS_CONFIGURED);
  }
  if (-1 == ret) {
    _set_fault("_ll_pin_init() failed");
  }
  return ret;
}


/**
* Send the fastcommand to bounce the ADC.
*
* @return
*    -1 if there was a problem writing the reset command.
*    0 if reset command was sent successfully.
*/
int8_t MCP356x::_reset_fxn() {
  _clear_registers();
  _isr_fired = false;
  _irqs_noted = 0;
  _irqs_serviced = 0;
  int8_t ret = _send_fast_command(0x38);
  return ((0 == ret) ? 0 : -1);
}


/**
* Handles our configuration after reset.
* Unlocks the registers ahead of any other operation.
* NOTE: Presently sets IRQ pin to be push-pull. So multiple instances of this
*   driver will require independant IRQ pins.
*
* @return
*   -1 on failure to write a register.
*   0  on success.
*/
int8_t MCP356x::_post_reset_fxn() {
  int8_t ret = -1;
  uint32_t c0_val = 0x00000080;

  // Enable register write.
  ret = _write_register(MCP356xRegister::LOCK, 0x000000A5);
  if (0 == ret) {
    // Enable fast command, disable IRQ on conversion start, IRQ pin is push-pull.
    ret = _write_register(MCP356xRegister::IRQ, 0x00000006);
    if (0 == ret) {
      if (_flags.value(MCP356X_FLAG_USE_INTRNL_CLK)) {
        c0_val &= 0xFFFFFFCF;   // Set CLK_SEL to use internal clock with no pin output.
        c0_val |= 0x00000020;
      }
      if (_flags.value(MCP356X_FLAG_USE_INTRNL_VREF)) {
        if (!_flags.value(MCP356X_FLAG_HAS_INTRNL_VREF)) {
          _set_fault("Failed to use internal Vref (unsupported)");
        }
        else {
          c0_val &= 0xFFFFFFBF;   // Set VREF_SEL to use internal VREF with buffered pin output.
          c0_val |= 0x00000040;
          _vref_plus  = 2.4;
          _vref_minus = 0;
        }
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
* Sets up the driver to read the ADC channels that assist us with calibration.
* Clears the existing calibration-related flags.
*
* @return
*   -1 if switching to the calibration channels failed
*   0 on success.
*/
int8_t MCP356x::_calibrate() {
  int8_t ret = -1;
  _flags.clear(MCP356X_FLAG_ALL_CAL_MASK);
  if (0 == _set_scan_channels(0x0000E000)) {
    if (0 == _set_read_mode(MCP356xMode::CONTINUOUS)) {
      // Give the circuit time to settle, JiC the supply isn't yet stable.
      _discard_window.reset(_circuit_settle_ms);
      ret = 0;
    }
  }
  return ret;
}


/**
* Put the driver into a FAULT state.
*
* @param msg is a debug string to be added to the log.
*/
void MCP356x::_set_fault(const char* msg) {
  c3p_log(LOG_LEV_WARN, LOCAL_LOG_TAG, "MCP356x fault: %s", msg);
  _fsm_mark_current_state(MCP356xState::FAULT);
}
