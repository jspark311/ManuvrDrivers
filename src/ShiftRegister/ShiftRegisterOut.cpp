#include "ShiftRegister.h"


ShiftRegisterOut::ShiftRegisterOut(const uint8_t devs, const uint8_t rclk_pin, const uint8_t oe_pin, const uint8_t srclr_pin) :
  DEVS_IN_CHAIN(devs), RCLK_PIN(rclk_pin), OE_PIN(oe_pin), SRCLR_PIN(srclr_pin) {};


/**
* Resets the class and makes the hardware state match.
*
* @return 0 on success, or -1 on allocation failure, or -2 on I/O error.
*/
int8_t ShiftRegisterOut::reset() {
  int8_t ret = -1;
  _flags = _flags & SHIFTREG_FLAG_RESET_MASK;   // Wipe the flags.

  // Zero the buffer.
  if (_allocated()) {
    for (uint8_t i = 0; i < (DEVS_IN_CHAIN << 1); i++) {
      *(_shadows + i) = 0;
    }
    ret--;
    // Write the changes, if possible.
    if (_class_flag(SHIFTREG_FLAG_PINS_CONFIGURED)) {
      if (255 != SRCLR_PIN) {
        setPin(SRCLR_PIN, false);
        setPin(RCLK_PIN, false);
        setPin(RCLK_PIN, true);
        setPin(SRCLR_PIN, true);
        ret = 0;
      }
      else if (0 == _write_chain()){
        ret = 0;
      }
    }
  }
  return ret;
}


/**
* Resets the class and makes the hardware state match.
*
* @return 0 on success, -1 on bad pin config, or -2 on allocation failure.
*/
int8_t ShiftRegisterOut::init() {
  int8_t ret = -1;
  if (0 == _ll_pin_init()) {
    ret--;
    if (_allocated()) {
      _class_set_flag(SHIFTREG_FLAG_INITIALIZED);
      ret = 0;
    }
  }
  return ret;
}


/**
* Set the state of the output of the shift-register if the pin is available.
*
* @return 0 on success, or -1 if no pin was given for that function.
*/
int8_t ShiftRegisterOut::outputEnabled(bool en) {
  int8_t ret = -1;
  if (255 != OE_PIN) {
    _class_set_flag(SHIFTREG_FLAG_ENABLED, en);
    setPin(OE_PIN, !en);  // Pin is inverted.
    ret = 0;
  }
  return ret;
}


/**
* Sets the value of a single pin in the chain. Invokes I/O immediately if possible.
*
* @return 0 on success. -1 on invalid pin. -2 on memory failure. -3 on I/O rejection.
*/
int8_t ShiftRegisterOut::digitalWrite(uint8_t pin, bool val) {
  int8_t ret = -1;
  if (_pin_valid(pin)) {
    ret--;
    if (_allocated()) {
      const uint8_t DEV_IDX  = (pin >> 3);
      const uint8_t BYTE_IDX = (DEVS_IN_CHAIN - 1) - DEV_IDX;
      const uint8_t BIT_MASK = 1 << (pin & 0x07);
      ret = digitalWrite8(DEV_IDX, (_shadows[BYTE_IDX] & ~BIT_MASK) | (val ? BIT_MASK : 0));
    }
  }
  return ret;
}


/**
* Returns the shadow value of a single pin in the chain.
* Never invokes I/O. Uses shadows exclusively.
*
* @return 1 on bit set, 0 on bit unset, -1 on invalid pin. -2 on memory failure.
*/
int8_t ShiftRegisterOut::digitalRead(uint8_t pin) {
  int8_t ret = -1;
  if (_pin_valid(pin)) {
    ret--;
    if (_allocated()) {
      const uint8_t BYTE_IDX = ((DEVS_IN_CHAIN - 1) - (pin >> 3)) + DEVS_IN_CHAIN;
      const uint8_t BIT_MASK = 1 << (pin & 0x07);
      const uint8_t ORIGINAL = _shadows[BYTE_IDX];
      ret = (0 == (ORIGINAL & BIT_MASK) ? 0 : 1);
    }
  }
  return ret;
}


/**
* Sets the value of a single device in the chain. Invokes I/O immediately if possible.
*
* @return 0 on success. -1 on invalid dev. -2 on memory failure. -3 on I/O rejection.
*/
int8_t ShiftRegisterOut::digitalWrite8(uint8_t dev, uint8_t val) {
  int8_t ret = -1;
  if (DEVS_IN_CHAIN > dev) {
    ret--;
    if (_allocated()) {
      const uint8_t BYTE_IDX = (DEVS_IN_CHAIN - 1) - dev;
      if (val != _shadows[BYTE_IDX]) {
        _shadows[BYTE_IDX] = val;
        if (_class_flag(SHIFTREG_FLAG_PINS_CONFIGURED)) {
          // We are in a position to send I/O now.
          ret--;
          if (0 == _write_chain()) {
            ret = 0;
          }
        }
      }
      else {
        ret = 0;
      }
    }
  }
  return ret;
}


/**
* Returns the shadow value of all pins on the given device in the chain.
* Never invokes I/O. Uses shadows exclusively.
*
* @return the byte representing the given device, or zero on failure.
*/
uint8_t ShiftRegisterOut::getPinValues(uint8_t dev) {
  uint8_t ret = 0;
  if (DEVS_IN_CHAIN > dev) {
    if (_allocated()) {
      ret = _shadows[((DEVS_IN_CHAIN - 1) - dev) + DEVS_IN_CHAIN];
    }
  }
  return ret;
}


/*
*
*/
void ShiftRegisterOut::printDebug(StringBuilder* output) {
  //StringBuilder::styleHeader1(output, "ShiftRegisterOut");
  output->concatf("\tChain len:     %u\n", DEVS_IN_CHAIN);
  output->concatf("\tRCLK Pin:      %u\n", RCLK_PIN);
  output->concatf("\tOE Pin:        %u\n", OE_PIN);
  output->concatf("\tSRCLR Pin:     %u\n", SRCLR_PIN);
  output->concatf("\toutputEnabled: %c\n", outputEnabled() ? 'y' : 'n');
  output->concatf("\tpendingIO:     %c\n", pendingIO() ? 'y' : 'n');
  output->concatf("\t_pins_initd:   %c\n", _class_flag(SHIFTREG_FLAG_PINS_CONFIGURED) ? 'y' : 'n');
  output->concatf("\tinitialized:   %c\n", initialized() ? 'y' : 'n');
  output->concatf("\tallocated:     %c\n", allocated() ? 'y' : 'n');
  //StringBuilder::printBuffer(output, _shadows, (DEVS_IN_CHAIN << 1));
}


/**
* OE_PIN and SRCLR_PIN are optional. RCLK_PIN is obligatory.
*
* @return 0 on success. -1 otherwise.
*/
int8_t ShiftRegisterOut::_ll_pin_init() {
  if (!_class_flag(SHIFTREG_FLAG_PINS_CONFIGURED)) {
    if (255 != RCLK_PIN) {
      pinMode(RCLK_PIN, GPIOMode::OUTPUT);
      setPin(RCLK_PIN, true);
      if (255 != SRCLR_PIN) {
        pinMode(SRCLR_PIN, GPIOMode::OUTPUT);
        // Store the zero state in the output registers.
        setPin(SRCLR_PIN, false);
        setPin(RCLK_PIN, false);
        setPin(RCLK_PIN, true);
        setPin(SRCLR_PIN, true);
      }
      // The driver will initialize the hardware with
      //   outputs disabled if it has the choice.
      _class_set_flag(SHIFTREG_FLAG_ENABLED, (255 == OE_PIN));
      if (255 != OE_PIN) {
        pinMode(OE_PIN, GPIOMode::OUTPUT);
        setPin(OE_PIN, true);
      }
      _class_set_flag(SHIFTREG_FLAG_PINS_CONFIGURED);
    }
  }
  return (_class_flag(SHIFTREG_FLAG_PINS_CONFIGURED) ? 0 : -1);
}


/**
* Checks for allocation of memory-resident shadow, attempting allocating if
*   necessary.
*
* @return true if the memory is/was allocated.
*/
bool ShiftRegisterOut::_allocated() {
  if (nullptr == _shadows) {
    _shadows = (uint8_t*) malloc(DEVS_IN_CHAIN << 1);
    if (nullptr != _shadows) {
      for (uint8_t i = 0; i < (DEVS_IN_CHAIN << 1); i++) {
        *(_shadows + i) = 0;
      }
    }
  }
  return (nullptr != _shadows);
};


/**
* Dispatches the write operation to reflect the shadow changes in hardware.
*
* #return 0 on success, nonzero otherwise.
*/
int8_t ShiftRegisterOut::_write_chain() {
  int8_t ret = -1;
  if (nullptr != _BUS) {
    ret--;
    SPIBusOp* op = (SPIBusOp*) _BUS->new_op(BusOpcode::TX, (BusOpCallback*) this);
    if (nullptr != op) {
      ret--;
      op->setBuffer(_shadows, DEVS_IN_CHAIN);
      if (0 == queue_io_job(op)) {
        _class_set_flag(SHIFTREG_FLAG_PENDING_IO);
        ret = 0;
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
* Called prior to the given bus operation beginning.
* Returning 0 will allow the operation to continue.
* Returning anything else will fail the operation with IO_RECALL.
*   Operations failed this way will have their callbacks invoked as normal.
*
* @param  _op  The bus operation that is about to run.
* @return 0 to run the op, or non-zero to cancel it.
*/
int8_t ShiftRegisterOut::io_op_callahead(BusOp* _op) {
  return 0;
}

/**
* When a bus operation completes, it is passed back to its issuing class.
* This driver never reads back from the device. Assume all ops are WRITEs.
*
* @param  _op  The bus operation that was completed.
* @return BUSOP_CALLBACK_NOMINAL on success, or appropriate error code.
*/
int8_t ShiftRegisterOut::io_op_callback(BusOp* _op) {
  int8_t ret = BUSOP_CALLBACK_NOMINAL;
  SPIBusOp* op = (SPIBusOp*) _op;
  if (op->hasFault()) {
    ret = BUSOP_CALLBACK_ERROR;
  }
  else {
    uint8_t* buf = op->buffer();
    const uint32_t BUF_LEN = op->bufferLen();
    uint8_t diff_array[BUF_LEN] = {0, };
    for (uint32_t idx = 0; idx < BUF_LEN; idx++) {
      const uint8_t BYTE_IDX = idx + DEVS_IN_CHAIN;
      diff_array[idx] = *(buf + idx) ^ *(_shadows + BYTE_IDX);
      *(_shadows + BYTE_IDX) = *(buf + idx);
    }
    if (_callback) {
      for (uint32_t dev_idx = 0; dev_idx < DEVS_IN_CHAIN; dev_idx++) {
        const uint8_t BYTE_IDX = ((DEVS_IN_CHAIN - 1) - dev_idx);
        if (0 != diff_array[BYTE_IDX]) {
          const uint8_t ORIGINAL = _shadows[BYTE_IDX + DEVS_IN_CHAIN];
          for (uint32_t bit_idx = 0; bit_idx < 8; bit_idx++) {
            const uint8_t BIT_MASK = 1 << bit_idx;
            if (diff_array[BYTE_IDX] & BIT_MASK) {
              const uint8_t PIN_NUMBER = (dev_idx << 3) + bit_idx;
              const uint8_t LEVEL      = (0 == (ORIGINAL & BIT_MASK) ? 0 : 1);
              _callback(PIN_NUMBER, LEVEL);
            }
          }
        }
      }
    }
  }
  _class_clear_flag(SHIFTREG_FLAG_PENDING_IO);
  return ret;
}


/**
* This is what is called when the class wants to conduct a transaction on the bus.
* Note that this is different from other class implementations, in that it checks for
*   callback population before clobbering it. This is because this class is also the
*   SPI driver. This might end up being reworked later.
*
* @param  _op  The bus operation to execute.
* @return Zero on success, or appropriate error code.
*/
int8_t ShiftRegisterOut::queue_io_job(BusOp* _op) {
  // This is the choke-point whereby any parameters to the operation that are
  //   uniform for this driver can be set.
  SPIBusOp* op = (SPIBusOp*) _op;
  op->setCSPin(RCLK_PIN);
  op->csActiveHigh(false);
  op->bitsPerFrame(SPIFrameSize::BITS_8);
  op->maxFreq(4000000);
  op->cpol(false);
  op->cpha(false);
  return _BUS->queue_io_job(op);
}
