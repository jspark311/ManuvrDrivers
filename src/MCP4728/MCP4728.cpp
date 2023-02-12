#include "MCP4728.h"


/*******************************************************************************
* Statics
*******************************************************************************/

const char* MCP4728::pwrStateStr(MCP4728PwrState e) {
  switch (e) {
    case MCP4728PwrState::NORMAL:        return "NORMAL";
    case MCP4728PwrState::GND_VIA_1K:    return "GND_VIA_1K";
    case MCP4728PwrState::GND_VIA_100K:  return "GND_VIA_100K";
    case MCP4728PwrState::GND_VIA_500K:  return "GND_VIA_500K";
  }
  return "UNKNOWN";
}

const char* MCP4728::vrefStr(MCP4728Vref e) {
  switch (e) {
    case MCP4728Vref::VDD:       return "VDD";
    case MCP4728Vref::INTERNAL:  return "INTERNAL";
  }
  return "UNKNOWN";
}



/*******************************************************************************
* Class boilerplate
*******************************************************************************/

/*
* Constructor.
*/
MCP4728::MCP4728(const float ext_vref, const uint8_t l_pin, const uint8_t b_pin, uint8_t address)
  : I2CDevice(address), _EXT_VREF(ext_vref), _LDAC_PIN(l_pin), _BUSY_PIN(b_pin) {}


/*
* Destructor
*/
MCP4728::~MCP4728() {}


/*
* Debug
*/
void MCP4728::printDebug(StringBuilder* output) {
  output->concat("-- MCP4728\n");
  output->concatf("\tExternal Vref: %.4fv\n", _EXT_VREF);
  output->concatf("\tPins conf'd:   %c\n", _mcp4728_flag(MCP4728_FLAG_PINS_CONFIGURED) ? 'y':'n');
  output->concatf("\tFound:         %c\n", devFound() ? 'y':'n');
  output->concatf("\tInitialized:   %c\n", initialized() ? 'y':'n');

  for (uint8_t i = 0; i < 4; i++) {
    output->concatf("--\tCHAN %u:  0x%03x  (%.4fv)  %s\n", i, chanValue(i), _DAC_VOLTS[i], pwrStateStr(chanPowerState(i)));
    output->concatf("\t  Vref:     %10s\tGain: %ux\n", vrefStr(chanVref(i)), chanGain(i));
    output->concatf("\t  Volatile: 0x%02x  0x%02x  0x%02x\n", _DAC_VALUES[(6 * i) + 0], _DAC_VALUES[(6 * i) + 1], _DAC_VALUES[(6 * i) + 2]);
    output->concatf("\t  EEPROM:   0x%02x  0x%02x  0x%02x\n", _DAC_VALUES[(6 * i) + 3], _DAC_VALUES[(6 * i) + 4], _DAC_VALUES[(6 * i) + 5]);
  }
}


/**
* Initialization function. Sets up pins, takes bus adapter reference.
* The real init work is handled on callback with a successful WHO_AM_I match.
*
* @return 0 on success or no action. Nonzero on failure.
*/
int8_t MCP4728::init(I2CAdapter* b) {
  _flags &= MCP4728_FLAG_RESET_MASK;
  int8_t ret = _ll_pin_init();
  if (nullptr != b) {
    assignBusInstance(b);
  }
  for (uint8_t i = 0; i < 4; i++) {    _DAC_VOLTS[i]  = 0;   }
  for (uint8_t i = 0; i < 24; i++) {   _DAC_VALUES[i] = 0;   }

  if (0 == ret) {  // Pins are set up.
    if (nullptr != _bus) {
      ret = refresh();
    }
    else {
      ret = -2;
    }
  }
  return ret;
}


/**
* Refresh the local variables with the state of the hardware.
*
* @return 0 on success or no action. Nonzero on failure.
*/
int8_t MCP4728::refresh() {
  int8_t ret = -2;
  I2CBusOp* op = _bus->new_op(BusOpcode::RX, this);
  if (nullptr != op) {
    op->dev_addr = _dev_addr;
    op->setBuffer(_DAC_VALUES, 24);
    if (0 == _bus->queue_io_job(op)) {
      ret = 0;
    }
  }
  return ret;
}


/**
* Set the output state for the given channel.
* NOTE: This setting is volatile.
*
* @param chan is the channel to set.
* @param pwr_state is the desired output state.
* @return 0 on success or no action. Nonzero on failure.
*/
int8_t MCP4728::chanPowerState(uint8_t chan, MCP4728PwrState pwr_state) {
  int8_t ret = -1;
  if (4 > chan) {
    const uint8_t SHAD_IDX = 6 * chan;
    uint8_t old_pwr_state = _DAC_VALUES[SHAD_IDX+1] & 0x60;
    uint8_t new_pwr_state = ((uint8_t) pwr_state) << 5;
    ret--;
    if (old_pwr_state != new_pwr_state) {
      uint8_t* buf = (uint8_t*) malloc(4);
      ret--;
      if (nullptr != buf) {
        bool free_buf = false;
        ret--;
        *(buf + 0)  = 0xA0;
        *(buf + 1)  = 0x00;
        *(buf + 0) |= (0 == chan) ? (new_pwr_state >> 3) : ((_DAC_VALUES[1]  & 0x60) >> 3);
        *(buf + 0) |= (1 == chan) ? (new_pwr_state >> 5) : ((_DAC_VALUES[7]  & 0x60) >> 5);
        *(buf + 1) |= (2 == chan) ? (new_pwr_state << 1) : ((_DAC_VALUES[13] & 0x60) << 1);
        *(buf + 1) |= (3 == chan) ? (new_pwr_state >> 1) : ((_DAC_VALUES[19] & 0x60) >> 1);
        I2CBusOp* op = _bus->new_op(BusOpcode::TX, this);
        if (nullptr != op) {
          ret--;
          op->dev_addr = _dev_addr;
          op->setBuffer(buf, 2);
          op->shouldFreeBuffer(true);
          if (0 == _bus->queue_io_job(op)) {
            ret = 0;
          }
          else {
            // Clean up our allocation if we must bail out.
            op->setBuffer(nullptr, 0);
            free_buf = true;
          }
        }
        else {
          free_buf = true;
        }
        if (free_buf) {
          free(buf);
        }
      }
    }
    else {  ret = 0;  }
  }
  return ret;
}


/**
* Set the gain for the given channel.
* NOTE: This setting is volatile.
*
* @param chan is the channel to set.
* @param gain needs to be either 1 or 2. All other values will set gain to 1.
* @return 0 on success or no action. Nonzero on failure.
*/
int8_t MCP4728::chanGain(uint8_t chan, uint8_t gain) {
  int8_t ret = -1;
  if (4 > chan) {
    const uint8_t SHAD_IDX = 6 * chan;
    uint8_t old_state = _DAC_VALUES[SHAD_IDX+1] & 0x10;
    uint8_t new_state = (2 == gain) ? 0x10 : 0;
    ret--;
    if (old_state != new_state) {
      uint8_t* buf = (uint8_t*) malloc(4);
      ret--;
      if (nullptr != buf) {
        bool free_buf = false;
        ret--;
        *(buf + 0)  = 0xC0;
        *(buf + 0) |= (0 == chan) ? (new_state >> 1) : ((_DAC_VALUES[1]  & 0x10) >> 1);
        *(buf + 0) |= (1 == chan) ? (new_state >> 2) : ((_DAC_VALUES[7]  & 0x10) >> 2);
        *(buf + 0) |= (2 == chan) ? (new_state >> 3) : ((_DAC_VALUES[13] & 0x10) >> 3);
        *(buf + 0) |= (3 == chan) ? (new_state >> 4) : ((_DAC_VALUES[19] & 0x10) >> 4);

        I2CBusOp* op = _bus->new_op(BusOpcode::TX, this);
        if (nullptr != op) {
          ret--;
          op->dev_addr = _dev_addr;
          op->setBuffer(buf, 1);
          op->shouldFreeBuffer(true);
          if (0 == _bus->queue_io_job(op)) {
            ret = 0;
          }
          else {
            // Clean up our allocation if we must bail out.
            op->setBuffer(nullptr, 0);
            free_buf = true;
          }
        }
        else {
          free_buf = true;
        }
        if (free_buf) {
          free(buf);
        }
      }
    }
    else {  ret = 0;  }
  }
  return ret;
}


/**
* Set the voltage referrence for the given channel.
* NOTE: This setting is volatile.
*
* @param chan is the channel to set.
* @param vr is the desired reference.
* @return 0 on success or no action. Nonzero on failure.
*/
int8_t MCP4728::chanVref(uint8_t chan, MCP4728Vref vr) {
  int8_t ret = -1;
  if (4 > chan) {
    const uint8_t SHAD_IDX = 6 * chan;
    uint8_t old_state = _DAC_VALUES[SHAD_IDX+1] & 0x80;
    uint8_t new_state = ((uint8_t) vr) << 7;
    ret--;
    if (old_state != new_state) {
      uint8_t* buf = (uint8_t*) malloc(4);
      ret--;
      if (nullptr != buf) {
        bool free_buf = false;
        ret--;
        *(buf + 0)  = 0x80;
        *(buf + 0) |= (0 == chan) ? (new_state >> 4) : ((_DAC_VALUES[1]  & 0x80) >> 4);
        *(buf + 0) |= (1 == chan) ? (new_state >> 5) : ((_DAC_VALUES[7]  & 0x80) >> 5);
        *(buf + 0) |= (2 == chan) ? (new_state >> 6) : ((_DAC_VALUES[13] & 0x80) >> 6);
        *(buf + 0) |= (3 == chan) ? (new_state >> 7) : ((_DAC_VALUES[19] & 0x80) >> 7);
        I2CBusOp* op = _bus->new_op(BusOpcode::TX, this);
        if (nullptr != op) {
          ret--;
          op->dev_addr = _dev_addr;
          op->setBuffer(buf, 1);
          op->shouldFreeBuffer(true);
          if (0 == _bus->queue_io_job(op)) {
            ret = 0;
          }
          else {
            // Clean up our allocation if we must bail out.
            op->setBuffer(nullptr, 0);
            free_buf = true;
          }
        }
        else {
          free_buf = true;
        }
        if (free_buf) {
          free(buf);
        }
      }
    }
    else {  ret = 0;  }
  }
  return ret;
}


/**
* Set the DAC count for the given channel.
* NOTE: This setting is volatile.
*
* @param chan is the channel to set.
* @param value is the DAC count.
* @return 0 on success or no action. Nonzero on failure.
*/
int8_t MCP4728::chanValue(uint8_t chan, uint16_t value) {
  int8_t ret = -1;
  if (4 > chan) {
    const uint8_t SHAD_IDX = 6 * chan;
    uint16_t current_val = _DAC_VALUES[SHAD_IDX + 2] | (((uint16_t) (_DAC_VALUES[SHAD_IDX + 1] & 0x0F)) << 8);
    ret--;
    if (value != current_val) {
      uint8_t* buf = (uint8_t*) malloc(4);
      ret--;
      if (nullptr != buf) {
        bool free_buf = false;
        ret--;
        *(buf + 0)  = 0x40 | (chan << 1);
        *(buf + 1)  = (_DAC_VALUES[SHAD_IDX + 1] & 0xF0) | ((uint8_t) (value >> 8) & 0x0F);
        *(buf + 2)  = (uint8_t) (value & 0x00FF);
        I2CBusOp* op = _bus->new_op(BusOpcode::TX, this);
        if (nullptr != op) {
          ret--;
          op->dev_addr = _dev_addr;
          op->setBuffer(buf, 3);
          op->shouldFreeBuffer(true);
          _mcp4728_set_flag(MCP4728_FLAG_CHAN_A_DIRTY << (chan & 0x03));
          if (0 == _bus->queue_io_job(op)) {
            ret = 0;
          }
          else {
            // Clean up our allocation if we must bail out.
            op->setBuffer(nullptr, 0);
            free_buf = true;
          }
        }
        else {
          free_buf = true;
        }
        if (free_buf) {
          free(buf);
        }
      }
    }
    else {  ret = 0;  }
  }
  return ret;
}


/**
* Stores the present volatile state of the given channel to the DAC's EEPROM.
*
* @param chan is the channel to set.
* @return 0 on success, nonzero otherwise.
*/
int8_t MCP4728::chanStore(uint8_t chan) {
  int8_t ret = -1;
  if (4 > chan) {
    const uint8_t SHAD_IDX = 6 * chan;
    uint8_t* buf = (uint8_t*) malloc(4);
    ret--;
    if (nullptr != buf) {
      bool free_buf = false;
      ret--;
      *(buf + 0)  = 0x58 | (chan << 1);
      *(buf + 1)  = _DAC_VALUES[SHAD_IDX + 1];
      *(buf + 2)  = _DAC_VALUES[SHAD_IDX + 2];
      I2CBusOp* op = _bus->new_op(BusOpcode::TX, this);
      if (nullptr != op) {
        ret--;
        op->dev_addr = _dev_addr;
        op->setBuffer(buf, 3);
        op->shouldFreeBuffer(true);
        if (0 == _bus->queue_io_job(op)) {
          ret = 0;
        }
        else {
          // Clean up our allocation if we must bail out.
          op->setBuffer(nullptr, 0);
          free_buf = true;
        }
      }
      else {
        free_buf = true;
      }
      if (free_buf) {
        free(buf);
      }
    }
  }
  return ret;
}


MCP4728PwrState MCP4728::chanPowerState(uint8_t chan) {
  const uint8_t SHAD_IDX = 6 * chan;
  return (MCP4728PwrState) ((_DAC_VALUES[SHAD_IDX+1] & 0x60) >> 5);
}


/*
* Returns the true gain being applied to the the current channel, regardless of
*   channel gain setting. Will consider Vref first.
*/
uint8_t MCP4728::chanGain(uint8_t chan) {
  const uint8_t  SHAD_IDX = 6 * chan;
  const uint8_t  CONF_BYTE  = _DAC_VALUES[SHAD_IDX + 1];
  if (0 == (CONF_BYTE & 0x80)) {
    return 1;  // Using an external Vref forces gain of 1X.
  }
  else {
    return (CONF_BYTE & 0x10) ? 2 : 1;
  }
}


uint16_t MCP4728::chanValue(uint8_t chan) {
  const uint8_t SHAD_IDX = 6 * chan;
  return _DAC_VALUES[SHAD_IDX + 2] | (((uint16_t) (_DAC_VALUES[SHAD_IDX + 1] & 0x0F)) << 8);
}

MCP4728Vref MCP4728::chanVref(uint8_t chan) {
  return (MCP4728Vref) ((_DAC_VALUES[(chan * 6) + 1] >> 7) & 0x01);
}


/**
* Store the current volatile settings to the DAC's EEPROM.
*
* @return 0 on success, nonzero otherwise.
*/
int8_t MCP4728::storeToNVM() {
  int8_t ret = -1;
  uint8_t* buf = (uint8_t*) malloc(9);
  if (nullptr != buf) {
    bool free_buf = false;
    ret--;
    *(buf + 0) = 0x50;  // Sequential write command.
    for (uint8_t i = 0; i < 4; i++) {
      const uint8_t SHAD_IDX = 6 * i;
      const uint8_t BUF_IDX  = 2 * i;
      *(buf + BUF_IDX + 1) = _DAC_VALUES[SHAD_IDX + 1];
      *(buf + BUF_IDX + 2) = _DAC_VALUES[SHAD_IDX + 2];
    }
    I2CBusOp* op = _bus->new_op(BusOpcode::TX, this);
    if (nullptr != op) {
      ret--;
      op->dev_addr = _dev_addr;
      op->setBuffer(buf, 9);
      op->shouldFreeBuffer(true);
      if (0 == _bus->queue_io_job(op)) {
        ret = 0;
      }
      else {
        // Clean up our allocation if we must bail out.
        op->setBuffer(nullptr, 0);
        free_buf = true;
      }
    }
    else {
      free_buf = true;
    }
    if (free_buf) {
      free(buf);
    }
  }
  return ret;
}


/**
* Set the DAC count for the given channel.
* NOTE: This setting is volatile.
*
* @param chan is the channel to set.
* @param value is the DAC count.
* @return 0 on success or no action. Nonzero on failure.
*/
int8_t MCP4728::chanVoltage(uint8_t chan, float new_voltage) {
  int8_t ret = -1;
  if (4 > chan) {
    const uint8_t  IDX_BASE   = chan * 6;
    const uint8_t  CONF_BYTE  = _DAC_VALUES[IDX_BASE + 1];
    const uint16_t CHAN_COUNT = _DAC_VALUES[IDX_BASE + 2] | (((uint16_t) (CONF_BYTE & 0x0F)) << 8);
    uint16_t new_value = 0;
    ret--;

    if (0 == (CONF_BYTE & 0x80)) {
      new_value = (uint16_t) ((new_voltage / _EXT_VREF) * 4096.0);
    }
    else {
      new_value = (uint16_t) ((new_voltage / ((CONF_BYTE & 0x10) ? 4.096 : 2.048)) * 4096.0);
    }

    if (new_value != CHAN_COUNT) {
      ret = chanValue(chan, new_value);
    }
    else {
      ret = 0;
    }
  }
  return ret;
}



/*
* NOTE: This function takes account of gain settings.
*/
int8_t MCP4728::_recalculate_chan_voltage(uint8_t chan) {
  const uint8_t  IDX_BASE   = chan * 6;
  const uint8_t  CONF_BYTE  = _DAC_VALUES[IDX_BASE + 1];
  const uint16_t CHAN_COUNT = _DAC_VALUES[IDX_BASE + 2] | (((uint16_t) (CONF_BYTE & 0x0F)) << 8);
  const float    RATIO      = CHAN_COUNT / 4096.0;
  float chan_volts = 0.0;
  int8_t ret   = -2;
  if (0 == (CONF_BYTE & 0x60)) {  // If the channel is in normal mode.
    ret = 0;
    if (0 == (CONF_BYTE & 0x80)) {
      chan_volts = RATIO * _EXT_VREF;
    }
    else {
      chan_volts = RATIO * ((CONF_BYTE & 0x10) ? 4.096 : 2.048);
    }
  }
  _mcp4728_clear_flag(MCP4728_FLAG_CHAN_A_DIRTY << chan);
  _DAC_VOLTS[chan] = chan_volts;
  return ret;
}


/**
* Low-level pin intialization.
*
* @return 0 on success. Nonzero otherwise.
*/
int8_t MCP4728::_ll_pin_init() {
  if (!_mcp4728_flag(MCP4728_FLAG_PINS_CONFIGURED)) {
    if (255 != _BUSY_PIN) {  // Optional pin
      if (0 != pinMode(_BUSY_PIN, GPIOMode::INPUT_PULLUP)) {
        return -2;
      }
    }
    if (255 != _LDAC_PIN) {
      if (0 != pinMode(_LDAC_PIN, GPIOMode::OUTPUT)) {
        return -2;
      }
      setPin(_LDAC_PIN, false);
    }
    _mcp4728_set_flag(MCP4728_FLAG_PINS_CONFIGURED);
  }
  return 0;
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
int8_t MCP4728::io_op_callahead(BusOp* _op) {
  return 0;   // Permit the transfer, always.
}

/**
* When a bus operation completes, it is passed back to its issuing class.
*
* @param  _op  The bus operation that was completed.
* @return BUSOP_CALLBACK_NOMINAL on success, or appropriate error code.
*/
int8_t MCP4728::io_op_callback(BusOp* _op) {
  int8_t ret = BUSOP_CALLBACK_NOMINAL;
  I2CBusOp* op = (I2CBusOp*) _op;

  if (0 == op->dev_addr) {
    // This was a general-call operation.
  }
  else {
    if (!op->hasFault()) {
      uint8_t* buf = op->buffer();
      uint8_t  len = op->bufferLen();
      const uint8_t CMD_BYTE = *buf;
      uint8_t chan = (CMD_BYTE & 0x06) >> 1;  // Might not apply.
      if (!devFound()) {
        _mcp4728_set_flag(MCP4728_FLAG_DEV_FOUND);
      }
      switch (op->get_opcode()) {
        case BusOpcode::TX:
          switch (CMD_BYTE & 0xE0) {  // Look at the command byte.
            case 0x00:    // Wrote volatile channel value.
            case 0x10:    // Wrote volatile channel value.
            case 0x20:    // Wrote volatile channel value.
            case 0x30:    // Wrote volatile channel value.
              // TODO: Update shadows for both register and EEPROM.
              // TODO: Recalculate channel voltages.
              break;
            case 0x40:
              switch (CMD_BYTE & 0x18) {
                case 0x00:    // Multi Write command without EEPROM update.
                case 0x18:    // Single Write command. Echoes to EEPROM.
                  while (len >= 3) {
                    const uint8_t SHAD_IDX = 6 * chan;
                    _DAC_VALUES[SHAD_IDX + 1] = *(buf + 1);
                    _DAC_VALUES[SHAD_IDX + 2] = *(buf + 2);
                    if (0x18 == (CMD_BYTE & 0x18)) {
                      _DAC_VALUES[SHAD_IDX + 4] = _DAC_VALUES[SHAD_IDX + 1];
                      _DAC_VALUES[SHAD_IDX + 5] = _DAC_VALUES[SHAD_IDX + 2];
                    }
                    _recalculate_chan_voltage(chan);
                    len -= 3;
                    buf += 3;
                    chan++;
                  }
                  break;
                case 0x10:    // Multi Write command with EEPROM update.
                  if (0 == ((len-1) & 0x01)) {  // Even number of bytes for channels?
                    for (uint8_t i = 1; i < (len-1); i+=2) {
                      // Copy the buffer back into (possibly) both sets of
                      //   registers to reflect that the values are current, and
                      //   optionally written to EEPROM.
                      const uint8_t SHAD_IDX = 6 * chan;
                      _DAC_VALUES[SHAD_IDX + 1] = *(buf + i + 0);
                      _DAC_VALUES[SHAD_IDX + 2] = *(buf + i + 1);
                      _DAC_VALUES[SHAD_IDX + 4] = _DAC_VALUES[SHAD_IDX + 1];
                      _DAC_VALUES[SHAD_IDX + 5] = _DAC_VALUES[SHAD_IDX + 2];
                      _recalculate_chan_voltage(chan);
                      chan++;
                    }
                  }
                  break;
              }
              break;

            case 0x60:    // Wrote new i2c address.
              // TODO: Anything?
              break;

            case 0x80:    // Wrote voltage reference bits for all channels.
            case 0xC0:    // Wrote gain bits for all channels.
              {
                const uint8_t bits = *(buf + 0) & 0x0F;
                uint8_t shift_size = (0x80 == (*buf & 0xE0)) ? 7:4;
                for (uint8_t i = 0; i < 4; i++) {
                  const uint8_t SHAD_IDX = 6 * i;
                  uint8_t scpd_bits = (bits >> (3 - i)) & 0x01;
                  _DAC_VALUES[SHAD_IDX + 1] = (_DAC_VALUES[SHAD_IDX + 1] & ~(1 << shift_size)) | (scpd_bits << shift_size);
                  _recalculate_chan_voltage(i);
                }
              }
              break;

            case 0xA0:    // Wrote power-down bits for all channels.
              {
                const uint8_t bits = (*(buf + 0) << 4) | (*(buf + 1) >> 4);
                for (uint8_t i = 0; i < 4; i++) {
                  const uint8_t SHAD_IDX = 6 * i;
                  uint8_t scpd_bits = (bits >> (6 - (i << 1))) & 0x03;
                  _DAC_VALUES[SHAD_IDX + 1] = (_DAC_VALUES[SHAD_IDX + 1] & 0x9F) | (scpd_bits << 5);
                  _recalculate_chan_voltage(i);
                }
              }
              break;
          }
          break;

        case BusOpcode::RX:  // Only RX in driver is a full refresh.
          if (24 == op->bufferLen()) {
            for (uint8_t chan = 0; chan < 4; chan++) {
              _recalculate_chan_voltage(chan);
            }
            _mcp4728_set_flag(MCP4728_FLAG_INITIALIZED);
          }
          break;

        default:
          break;
      }
    }
    else {
      ret = BUSOP_CALLBACK_ERROR;
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
* @section mcp4728-tools DAC tools (MCP4728)
*
* This is the console handler for using the MCP4728 DAC.
*
* @subsection arguments Arguments
* Argument | Purpose | Required
* -------- | ------- | --------
* 1        | UartID  | No (lists UARTs if not provided)
* 2        | Action  | No (prints debugging information for specified UART if not provided)
* 3        | Action-Specific | No
*
* @subsection cmd-actions Actions
* Action    | Description | Additional arguments
* --------- | ----------- | --------------------
* `init`    | Initialize the driver and prints the result. | None
* `refresh` | Refresh the register shadows from the hardware. | None
* `store`   | Store the current channel values to the chip's non-volatile memory. | [channel]
* `val`     | Renders channel details to the console, or set the value. | <channel> [new-value]
* `enable`  | Enable or disable a DAC channel. | <channel> <0, 1>
* `gain`    | Set the gain for a channel. | <channel> <new-gain>
* `ref`     | Set the voltage reference for a channel. | <channel> <new-ref-id>
*
*/
int8_t MCP4728::console_handler(StringBuilder* text_return, StringBuilder* args) {
  int8_t ret = 0;
  char*  cmd = args->position_trimmed(0);
  int   chan = args->position_as_int(1);
  switch (args->count()) {
    case 0:
      printDebug(text_return);
      break;
    case 1:
      if (0 == StringBuilder::strcasecmp(cmd, "init")) {
        text_return->concatf("MCP4728 init() returns %d.\n", init());
      }
      else if (0 == StringBuilder::strcasecmp(cmd, "refresh")) {
        text_return->concatf("MCP4728 refresh() returns %d.\n", refresh());
      }
      else if (0 == StringBuilder::strcasecmp(cmd, "store")) {
        text_return->concatf("MCP4728 storeToNVM() returns %d.\n", storeToNVM());
      }
      else { ret = -1; }
      break;
    case 2:
      if (chan > 3) {
        ret = -1;
      }
      else if (0 == StringBuilder::strcasecmp(cmd, "val")) {
        text_return->concatf("DAC channel %u value: %u  (%.4f volts)\n", chan, chanValue(chan), chanVoltage(chan));
        text_return->concatf("\tOutput mode: %s\n", MCP4728::pwrStateStr(chanPowerState(chan)));
      }
      else if (0 == StringBuilder::strcasecmp(cmd, "gain")) {
        text_return->concatf("DAC channel %u gain: %u \n", chan, chanGain(chan));
      }
      else if (0 == StringBuilder::strcasecmp(cmd, "store")) {
        text_return->concatf("MCP4728 storeToNVM() returns %d.\n", chanStore(chan));
      }
      else { ret = -1; }
      break;
    case 3:
      if (chan > 3) {
        ret = -1;
      }
      else if (0 == StringBuilder::strcasecmp(cmd, "val")) {
        int val = args->position_as_int(2);
        text_return->concatf("Set DAC channel %u to %u returns %u.\n", chan, val, chanValue(chan, val));
      }
      else if (0 == StringBuilder::strcasecmp(cmd, "enable")) {
        MCP4728PwrState mode = (0 != args->position_as_int(2)) ? MCP4728PwrState::NORMAL : MCP4728PwrState::GND_VIA_1K;
        text_return->concatf("Set DAC channel %u to mode %s returns %u.\n", chan, MCP4728::pwrStateStr(mode), chanPowerState(chan, mode));
      }
      else if (0 == StringBuilder::strcasecmp(cmd, "gain")) {
        int gain = args->position_as_int(2);
        text_return->concatf("Set DAC channel %u to gain %d returns %u.\n", chan, gain, chanGain(chan, gain));
      }
      else if (0 == StringBuilder::strcasecmp(cmd, "ref")) {
        MCP4728Vref vref = (0 != args->position_as_int(2)) ? MCP4728Vref::INTERNAL : MCP4728Vref::VDD;
        text_return->concatf("Set DAC channel %u to ref %s returns %u.\n", chan, MCP4728::vrefStr(vref), chanVref(chan, vref));
      }
      else { ret = -1; }
      break;
    default:
      ret = -1;
      break;
  }
  return ret;
}
