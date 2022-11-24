/*******************************************************************************
*
*******************************************************************************/

#include "SX127x.h"


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
/* Real register addresses */
static const uint8_t SX127X_REG_ADDR[27] = {
  0x00, 0x01, 0x06, 0x07, 0x08, 0x09, 0x0c, 0x0d,
  0x0e, 0x0f, 0x10, 0x12, 0x13, 0x1a, 0x1b, 0x1d,
  0x1e, 0x20, 0x21, 0x22, 0x26, 0x2c, 0x31, 0x37,
  0x39, 0x40, 0x42
};


/**
* Applies the device address and properly shifts the register address into
*   a control byte. Always sets up for incremental read/write.
* This never fails and always returns a byte to be used as the first byte
*   in an SPI transaction with the ADC.
*
* @param r The register we wish to transact with.
* @return the first byte of an SPI register transaction.
*/
uint8_t _get_reg_addr(const SX127xRegister r) {
  uint8_t reg_idx = (uint8_t) r;
  if (27 > reg_idx) {
    return SX127X_REG_ADDR[reg_idx];
  }
  return 0;
}


SX127xRegister _reg_id_from_addr(const uint8_t addr) {
  switch (0x7F & addr) {
    case 0x00:   return SX127xRegister::FIFO;
    case 0x01:   return SX127xRegister::OP_MODE;
    case 0x06:   return SX127xRegister::FRF_MSB;
    case 0x07:   return SX127xRegister::FRF_MID;
    case 0x08:   return SX127xRegister::FRF_LSB;
    case 0x09:   return SX127xRegister::PA_CONFIG;
    case 0x0c:   return SX127xRegister::LNA;
    case 0x0d:   return SX127xRegister::FIFO_ADDR_PTR;
    case 0x0e:   return SX127xRegister::FIFO_TX_BASE_ADDR;
    case 0x0f:   return SX127xRegister::FIFO_RX_BASE_ADDR;
    case 0x10:   return SX127xRegister::FIFO_RX_CURRENT_ADDR;
    case 0x12:   return SX127xRegister::IRQ_FLAGS;
    case 0x13:   return SX127xRegister::RX_NB_BYTES;
    case 0x1a:   return SX127xRegister::PKT_RSSI_VALUE;
    case 0x1b:   return SX127xRegister::PKT_SNR_VALUE;
    case 0x1d:   return SX127xRegister::MODEM_CONFIG_1;
    case 0x1e:   return SX127xRegister::MODEM_CONFIG_2;
    case 0x20:   return SX127xRegister::PREAMBLE_MSB;
    case 0x21:   return SX127xRegister::PREAMBLE_LSB;
    case 0x22:   return SX127xRegister::PAYLOAD_LENGTH;
    case 0x26:   return SX127xRegister::MODEM_CONFIG_3;
    case 0x2c:   return SX127xRegister::RSSI_WIDEBAND;
    case 0x31:   return SX127xRegister::DETECTION_OPTIMIZE;
    case 0x37:   return SX127xRegister::DETECTION_THRESHOLD;
    case 0x39:   return SX127xRegister::SYNC_WORD;
    case 0x40:   return SX127xRegister::DIO_MAPPING_1;
    case 0x42:   return SX127xRegister::VERSION;
  }
  return SX127xRegister::INVALID;
}


/**
* Static function to convert enum to string.
*/
const char* _fsm_state_str(const SX127xState e) {
  switch (e) {
    case SX127xState::UNINIT:        return "UNINIT";
    case SX127xState::PREINIT:       return "PREINIT";
    case SX127xState::RESETTING:     return "RESETTING";
    case SX127xState::DISCOVERY:     return "DISCOVERY";
    case SX127xState::REGINIT:       return "REGINIT";
    case SX127xState::USR_CONF:      return "USR_CONF";
    case SX127xState::READY:         return "READY";
    case SX127xState::LOW_PWR:       return "LOW_PWR";
    case SX127xState::FAULT:         return "FAULT";
    default:   break;
  }
  return "INVALID";
}


/**
* Static function to convert enum to string.
*/
static const bool _fsm_code_valid(const SX127xState e) {
  switch (e) {
    case SX127xState::UNINIT:
    case SX127xState::PREINIT:
    case SX127xState::RESETTING:
    case SX127xState::DISCOVERY:
    case SX127xState::REGINIT:
    case SX127xState::USR_CONF:
    case SX127xState::READY:
    case SX127xState::LOW_PWR:
    case SX127xState::FAULT:
      return true;
    default:
      break;
  }
  return false;
}


/*
* This is an ISR.
*/
void sx127x_d0_isr() {
}

/*
* This is an ISR.
*/
void sx127x_d1_isr() {
}

/*
* This is an ISR.
*/
void sx127x_d2_isr() {
}



/*******************************************************************************
*   ___ _              ___      _ _              _      _
*  / __| |__ _ ______ | _ ) ___(_) |___ _ _ _ __| |__ _| |_ ___
* | (__| / _` (_-<_-< | _ \/ _ \ | / -_) '_| '_ \ / _` |  _/ -_)
*  \___|_\__,_/__/__/ |___/\___/_|_\___|_| | .__/_\__,_|\__\___|
*                                          |_|
* Constructors/destructors, class initialization functions and so-forth...
*******************************************************************************/

/* Constructor */
SX127x::SX127x(const SX127xOpts* o) : _opts{o} {
}


/* Destructor */
SX127x::~SX127x() {
}


int8_t SX127x::init(SPIAdapter* b) {
  int8_t pin_setup_ret = _ll_pin_init();  // Configure the pins if they are not already.
  int8_t ret = -1;
  if (pin_setup_ret >= 0) {
    ret--;
    if (nullptr != b) {
      ret = 0;
      _BUS = b;
      _tx_busop.setAdapter(_BUS);
      _tx_busop.shouldReap(false);
      //_tx_busop.setParams((uint8_t) _get_reg_addr(SX127xRegister::OP_MODE) | 0x01);
      //_tx_busop.setBuffer((uint8_t*) &_shadows[(uint8_t) SX127xRegister::IRQ], 1);

      _rx_busop.setAdapter(_BUS);
      _rx_busop.shouldReap(false);
      _rx_busop.setParams((uint8_t) _get_reg_addr(SX127xRegister::OP_MODE) | 0x01);
      _rx_busop.setBuffer((uint8_t*) &_shadows[(uint8_t) SX127xRegister::OP_MODE], (sizeof(_shadows)-1));
    }
  }
  return ret;
}


int8_t SX127x::reset() {
  int8_t ret = -1;
  return ret;
}


int8_t SX127x::refresh() {
  int8_t ret = -1;
  return ret;
}


/**
* Debug support method. This fxn is only present in debug builds.
*
* @param   StringBuilder* The buffer into which this fxn should write its output.
*/
void SX127x::printDebug(StringBuilder* output) {
  output->concat("\n");
  output->concatf("\tReset: %u\tCS:    %u\n", _opts.reset_pin, _opts.cs_pin);
  output->concatf("\tD0:    %u\tD1:    %u\tD2:    %u\n", _opts.d0_pin, _opts.d1_pin, _opts.d2_pin);
  _print_fsm(output);
}


/**
* Debug support method. Dump the register shadows.
*
* @param   StringBuilder* The buffer into which this fxn should write its output.
*/
void SX127x::printRegs(StringBuilder* output) {
  for (uint8_t i = 0; i < sizeof(_shadows); i++) {
    output->concatf("\t0x%02x:\t0x%02x", SX127X_REG_ADDR[i], _shadows[i]);
  }
}


void SX127x::_print_fsm(StringBuilder* output) {
  bool keep_looping = true;
  int i = 0;
  StringBuilder::styleHeader1(output, "SX127x FSM");
  output->concatf("\tPrior state:   %s\n", _fsm_state_str(_fsm_pos_prior));
  output->concatf("\tCurrent state: %s%s\n\tNext states:   ", _fsm_state_str(_fsm_pos), _fsm_is_waiting() ? " (LOCKED)":" ");
  while (keep_looping & (i < SX127X_FSM_WAYPOINT_DEPTH)) {
    if (SX127xState::UNINIT == _fsm_waypoints[i]) {
      output->concat("<STABLE>");
      keep_looping = false;
    }
    else {
      output->concatf("%s, ", _fsm_state_str(_fsm_waypoints[i]));
    }
    i++;
  }
  if (_fsm_is_waiting()) {
    output->concatf("\tFSM locked for another %ums\n", _fsm_lockout_ms - millis());
  }
  output->concat('\n');
}


/**
* Setup the low-level pin details. Execution is idempotent.
*
* @return
*   -1 if the pin setup is wrong. Class must halt.
*   0  if the pin setup is complete.
*/
int8_t SX127x::_ll_pin_init() {
  int8_t ret = -1;
  if (_flags.value(SX127X_FLAG_PINS_CONFIGURED)) {
    ret = 0;
  }
  else if (255 != _opts.cs_pin) {   // This pin is required.
    ret = pinMode(_opts.cs_pin, GPIOMode::OUTPUT);
    if (0 == ret) {
      setPin(_opts.cs_pin, true);   // Unselect SPI.
      if (255 != _opts.reset_pin) {
        ret = pinMode(_opts.reset_pin, GPIOMode::OUTPUT);
        if (0 == ret) {
          // Start the part in a reset state.
          setPin(_opts.reset_pin, false);
        }
      }
      if (255 != _opts.d0_pin) {
        pinMode(_opts.d0_pin, GPIOMode::INPUT);
      }
      if (255 != _opts.d1_pin) {
        pinMode(_opts.d1_pin, GPIOMode::INPUT);
      }
      if (255 != _opts.d2_pin) {
        pinMode(_opts.d2_pin, GPIOMode::INPUT);
      }
    }
  }
  return ret;
}


/*******************************************************************************
* ___     _       _                      These members are mandatory overrides
*  |   / / \ o   | \  _     o  _  _      for implementing I/O callbacks. They
* _|_ /  \_/ o   |_/ (/_ \/ | (_ (/_     are also implemented by Adapters.
*******************************************************************************/

/**
* This is what we call when this class wants to conduct a transaction on
*   the bus. We simply forward to the bus we are bound to.
*
* @param  _op  The bus operation that was completed.
* @return 0 to run the op, or non-zero to cancel it.
*/
int8_t SX127x::queue_io_job(BusOp* _op) {
  SPIBusOp* op = (SPIBusOp*) _op;
  op->callback = this;
  op->setCSPin(_opts.cs_pin);
  op->csActiveHigh(false);
  op->maxFreq(10000000);
  op->cpol(false);
  op->cpha(false);
  return _BUS->queue_io_job(op);
}


int8_t SX127x::io_op_callahead(BusOp* _op) {
  return 0;
}


int8_t SX127x::io_op_callback(BusOp* _op) {
  SPIBusOp* op = (SPIBusOp*) _op;
  int8_t    ret = BUSOP_CALLBACK_NOMINAL;

  if (!op->hasFault()) {
    uint8_t* buf   = op->buffer();
    uint     len   = op->bufferLen();
    uint8_t  ridx  = (uint8_t) _reg_id_from_addr(op->getTransferParam(0));
    switch (op->get_opcode()) {
      case BusOpcode::TX:
        for (uint i = 0; i < len; i++) {
          uint8_t value = *(buf + i);
          switch ((SX127xRegister) (i + ridx)) {
            case SX127xRegister::FIFO:
            case SX127xRegister::OP_MODE:
            case SX127xRegister::FRF_MSB:
            case SX127xRegister::FRF_MID:
            case SX127xRegister::FRF_LSB:
            case SX127xRegister::PA_CONFIG:
            case SX127xRegister::LNA:
            case SX127xRegister::FIFO_ADDR_PTR:
            case SX127xRegister::FIFO_TX_BASE_ADDR:
            case SX127xRegister::FIFO_RX_BASE_ADDR:
            case SX127xRegister::FIFO_RX_CURRENT_ADDR:
            case SX127xRegister::IRQ_FLAGS:
            case SX127xRegister::RX_NB_BYTES:
            case SX127xRegister::PKT_RSSI_VALUE:
            case SX127xRegister::PKT_SNR_VALUE:
            case SX127xRegister::MODEM_CONFIG_1:
            case SX127xRegister::MODEM_CONFIG_2:
            case SX127xRegister::PREAMBLE_MSB:
            case SX127xRegister::PREAMBLE_LSB:
            case SX127xRegister::PAYLOAD_LENGTH:
            case SX127xRegister::MODEM_CONFIG_3:
            case SX127xRegister::RSSI_WIDEBAND:
            case SX127xRegister::DETECTION_OPTIMIZE:
            case SX127xRegister::DETECTION_THRESHOLD:
            case SX127xRegister::SYNC_WORD:
            case SX127xRegister::DIO_MAPPING_1:
            case SX127xRegister::VERSION:
              break;
            default:   // Illegal write target.
              break;
          }
        }
        break;
      case BusOpcode::RX:
        for (uint i = 0; i < len; i++) {
          uint8_t value = *(buf + i);
          switch ((SX127xRegister) (i + ridx)) {
            case SX127xRegister::FIFO:
            case SX127xRegister::OP_MODE:
            case SX127xRegister::FRF_MSB:
            case SX127xRegister::FRF_MID:
            case SX127xRegister::FRF_LSB:
            case SX127xRegister::PA_CONFIG:
            case SX127xRegister::LNA:
            case SX127xRegister::FIFO_ADDR_PTR:
            case SX127xRegister::FIFO_TX_BASE_ADDR:
            case SX127xRegister::FIFO_RX_BASE_ADDR:
            case SX127xRegister::FIFO_RX_CURRENT_ADDR:
            case SX127xRegister::IRQ_FLAGS:
            case SX127xRegister::RX_NB_BYTES:
            case SX127xRegister::PKT_RSSI_VALUE:
            case SX127xRegister::PKT_SNR_VALUE:
            case SX127xRegister::MODEM_CONFIG_1:
            case SX127xRegister::MODEM_CONFIG_2:
            case SX127xRegister::PREAMBLE_MSB:
            case SX127xRegister::PREAMBLE_LSB:
            case SX127xRegister::PAYLOAD_LENGTH:
            case SX127xRegister::MODEM_CONFIG_3:
            case SX127xRegister::RSSI_WIDEBAND:
            case SX127xRegister::DETECTION_OPTIMIZE:
            case SX127xRegister::DETECTION_THRESHOLD:
            case SX127xRegister::SYNC_WORD:
            case SX127xRegister::DIO_MAPPING_1:
            case SX127xRegister::VERSION:
              break;
            default:   // Illegal read target.
              break;
          }
        }
        break;
      default:  // Illegal operation.
        break;
    }
  }
  return ret;
}


/*******************************************************************************
* FSM functions
*******************************************************************************/

/**
* Considers the current FSM state, and decides whether or not to advance the
*   state machine.
* NOTE: This function does not plan state machine routes, and should thus not
*   call _set_fsm_position() directly. Only _advance_state_machine().
*
* @return  1 on state shift
*          0 on no action
*         -1 on error
*/
int8_t SX127x::_poll_fsm() {
  int8_t ret = 0;
  bool fsm_advance = false;
  switch (_fsm_pos) {
    case SX127xState::UNINIT:
    case SX127xState::PREINIT:
    case SX127xState::RESETTING:
    case SX127xState::DISCOVERY:
    case SX127xState::REGINIT:
    case SX127xState::USR_CONF:
    case SX127xState::READY:
    case SX127xState::LOW_PWR:
    case SX127xState::FAULT:
      break;
    default:   // Can't exit from an unknown state.
      ret = -1;
      break;
  }

  // If the current state's exit criteria is met, we advance the FSM.
  if (fsm_advance & (-1 != ret)) {
    ret = (0 == _advance_state_machine()) ? 1 : 0;
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
int8_t SX127x::_set_fsm_position(SX127xState new_state) {
  int8_t fxn_ret = -1;
  int8_t ret = -1;
  bool state_entry_success = false;   // Fail by default.
  if (!_fsm_is_waiting()) {
    switch (new_state) {
      case SX127xState::UNINIT:
      case SX127xState::PREINIT:
      case SX127xState::RESETTING:
      case SX127xState::DISCOVERY:
      case SX127xState::REGINIT:
      case SX127xState::USR_CONF:
      case SX127xState::READY:
      case SX127xState::LOW_PWR:
      case SX127xState::FAULT:
        break;
      default:
        break;
    }

    if (state_entry_success) {
      if (LOG_LEV_NOTICE <= _verbosity) c3p_log(LOG_LEV_NOTICE, __PRETTY_FUNCTION__, "SX127x FSM moved %s ---> %s", _fsm_state_str(_fsm_pos), _fsm_state_str(new_state));
      _fsm_pos_prior = _fsm_pos;
      _fsm_pos       = new_state;
      fxn_ret = 0;
    }
  }
  return fxn_ret;
}



/**
* Internal function responsible for advancing the state machine.
* NOTE: This function does no checks for IF the FSM should move forward. It only
*   performs the actions required to do it.
* NOTE: This function should only be called by _poll_fsm().
*
* @return 0 on state change, -1 otherwise.
*/
int8_t SX127x::_advance_state_machine() {
  int8_t ret = -1;
  if (SX127xState::UNINIT != _fsm_waypoints[0]) {
    if (0 == _set_fsm_position(_fsm_waypoints[0])) {
      ret = 0;
      for (int i = 0; i < (SX127X_FSM_WAYPOINT_DEPTH-1); i++) {
        _fsm_waypoints[i] = _fsm_waypoints[i+1];
      }
      _fsm_waypoints[SX127X_FSM_WAYPOINT_DEPTH-1] = SX127xState::UNINIT;
    }
  }
  return ret;
}


/**
* This function checks each state code for validity, but does not error-check
*   the validity of the FSM traversal route specified in the arguments. It just
*   adds them to the list if they all correspond to valid state codes.
* This function will accept a maximum of sizeof(_fsm_waypoints) arguments, and
*   will clobber the contents of that member if the call succeeds. Arguments
*   provided in excess of the limit will be truncated with no error.
*
* @return 0 on success, -1 on no params, -2 on invalid FSM code.
*/
int8_t SX127x::_set_fsm_route(int arg_count, ...) {
  int8_t ret = -1;
  const int PARAM_COUNT = strict_min((int8_t) arg_count, (int8_t) SX127X_FSM_WAYPOINT_DEPTH);
  if (PARAM_COUNT > 0) {
    va_list args;
    va_start(args, arg_count);
    SX127xState test_values[PARAM_COUNT] = {SX127xState::UNINIT, };
    ret = 0;
    for (int i = 0; i < PARAM_COUNT; i++) {
      test_values[i] = (SX127xState) va_arg(args, int);
    }
    va_end(args);   // Close out the va_args, and error-check each value.
    for (int i = 0; i < PARAM_COUNT; i++) {
      if (!_fsm_code_valid(test_values[i])) {
        ret = -2;
      }
    }
    if (0 == ret) {
      // If everything looks good, add items to the state traversal list, and
      //   zero the remainder.
      for (int i = 0; i < SX127X_FSM_WAYPOINT_DEPTH; i++) {
        _fsm_waypoints[i] = (i < PARAM_COUNT) ? test_values[i] : SX127xState::UNINIT;
      }
    }
  }
  return ret;
}


/**
* This function checks each state code for validity, but does not error-check
*   the validity of the FSM traversal route specified in the arguments. It just
*   adds them to the list if they all correspond to valid state codes.
* This function will accept a maximum of sizeof(_fsm_waypoints) arguments, and
*   will append to the contents of that member if the call succeeds. Arguments
*   provided in excess of the limit will be truncated with no error.
*
* @return 0 on success, -1 on no params, -2 on invalid FSM code.
*/
int8_t SX127x::_append_fsm_route(int arg_count, ...) {
  int8_t ret = -1;
  const int PARAM_COUNT = strict_min((int8_t) arg_count, (int8_t) SX127X_FSM_WAYPOINT_DEPTH);
  if (PARAM_COUNT > 0) {
    va_list args;
    va_start(args, arg_count);
    SX127xState test_values[PARAM_COUNT] = {SX127xState::UNINIT, };
    ret = 0;
    for (int i = 0; i < PARAM_COUNT; i++) {
      test_values[i] = (SX127xState) va_arg(args, int);
    }
    va_end(args);   // Close out the va_args, and error-check each value.
    for (int i = 0; i < PARAM_COUNT; i++) {
      if (!_fsm_code_valid(test_values[i])) {
        ret = -2;
      }
    }
    if (0 == ret) {
      // If everything looks good, seek to the end of the state traversal list,
      //   and append.
      uint8_t fidx = 0;
      while ((fidx < SX127X_FSM_WAYPOINT_DEPTH) && (SX127xState::UNINIT != _fsm_waypoints[fidx])) {
        fidx++;
      }
      const uint8_t PARAMS_TO_COPY = strict_min((uint8_t)(SX127X_FSM_WAYPOINT_DEPTH - fidx), (uint8_t) PARAM_COUNT);
      for (int i = 0; i < PARAMS_TO_COPY; i++) {
        _fsm_waypoints[i + fidx] = test_values[i];
      }
    }
  }
  return ret;
}


int8_t SX127x::_prepend_fsm_state(SX127xState nxt) {
  int8_t ret = -1;
  if (_fsm_code_valid(nxt)) {
    ret--;
    // If everything looks good, seek to the end of the state traversal list,
    //   and append.
    uint8_t fidx = 0;
    while ((fidx < SX127X_FSM_WAYPOINT_DEPTH) && (SX127xState::UNINIT != _fsm_waypoints[fidx])) {
      SX127xState last = _fsm_waypoints[fidx];
      _fsm_waypoints[fidx] = nxt;
      nxt = last;
      fidx++;
    }
    if (fidx < SX127X_FSM_WAYPOINT_DEPTH) {
      _fsm_waypoints[fidx] = nxt;
      ret = 0;
    }
  }
  return ret;
}



bool SX127x::_fsm_is_waiting() {
  bool ret = false;
  if (0 != _fsm_lockout_ms) {
    if (millis() >= _fsm_lockout_ms) {
      _fsm_lockout_ms = 0;
    }
    else {
      ret = true;
    }
  }
  return ret;
}



/*******************************************************************************
* Console callback
* These are built-in handlers for using this instance via a console.
*******************************************************************************/

/**
* @page console-handlers
* @section sx127x-tools SX127x tools
*
* This is the console handler for using the MCP356x ADC. If invoked without
*   arguments, it will print channel values, as are being observed by hardware.
*
* @subsection cmd-actions Actions
*
* Action    | Description | Additional arguments
* --------- | ----------- | --------------------
* `init`    | Manually invoke the driver's `init()` function. | None
* `reset`   | Manually invoke the driver's `reset()` function. | None
* `refresh` | Refresh the register shadows from the hardware. | None
*/
int8_t SX127x::console_handler(StringBuilder* text_return, StringBuilder* args) {
  int8_t ret = 0;
  bool print_uarts = true;
  if (0 < args->count()) {
    char* cmd = args->position_trimmed(1);
    if (0 == StringBuilder::strcasecmp(cmd, "init")) {
      text_return->concatf("lora.init() returns %d.\n", init());
    }
    else if (0 == StringBuilder::strcasecmp(cmd, "regs")) {
      printRegs(text_return);
    }
    else if (0 == StringBuilder::strcasecmp(cmd, "reset")) {
      text_return->concatf("lora.reset() returns %d.\n", reset());
    }
    else if (0 == StringBuilder::strcasecmp(cmd, "refresh")) {
      text_return->concatf("lora.refresh() returns %d.\n", refresh());
    }
    else {
      ret = -1;
    }
  }
  else {
    printDebug(text_return);
  }
  return ret;
}
