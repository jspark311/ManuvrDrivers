#include "TCA9548.h"


/*******************************************************************************
*   ___ _              ___      _ _              _      _
*  / __| |__ _ ______ | _ ) ___(_) |___ _ _ _ __| |__ _| |_ ___
* | (__| / _` (_-<_-< | _ \/ _ \ | / -_) '_| '_ \ / _` |  _/ -_)
*  \___|_\__,_/__/__/ |___/\___/_|_\___|_| | .__/_\__,_|\__\___|
*                                          |_|
* Constructors/destructors, class initialization functions and so-forth...
*******************************************************************************/

/**
* Constructor
*/
TCA9548::TCA9548(I2CAdapter* bus, const uint8_t I2C_ADDR, const uint8_t RESET_PIN) :
  I2CDevice(I2C_ADDR, bus),
  _RESET_PIN(RESET_PIN),
  _busop_set_mux(BusOpcode::TX, this) {}


/*
*
*/
int8_t TCA9548::init(I2CAdapter* b) {
  int8_t ret = -1;
  _shadow_known = false;
  if (nullptr != b) {
    _bus = b;
  }
  if (nullptr != _bus) {
    ret--;
    if (0 == _ll_pin_init()) {
      ret--;
      _busop_set_mux.shouldReap(false);
      _busop_set_mux.dev_addr = _dev_addr;
      _busop_set_mux.sub_addr = -1;
      _busop_set_mux.setBuffer(&_shadow_w, 1);
      if (0 == _read_register()) {
        ret = 0;
      }
    }
  }
  return ret;
}


int8_t TCA9548::reset() {
  int8_t ret = -1;
  if (255 != _RESET_PIN) {
    setPin(_RESET_PIN, 0);
    sleep_us(5);
    setPin(_RESET_PIN, 1);
    ret = 0;
  }
  else {
    // TODO: wipe out hardware values by register write.
  }
  _shadow_known = false;
  return ret;
}


int8_t TCA9548::_ll_pin_init() {
  int8_t ret = 0;
  if (255 != _RESET_PIN) {
    ret--;
    if (0 == pinMode(_RESET_PIN, GPIOMode::OUTPUT)) {
      setPin(_RESET_PIN, 0);
      sleep_us(5);
      setPin(_RESET_PIN, 1);
      ret = 0;
    }
  }
  _pins_confd = (0 == ret);
  return ret;
}


int8_t TCA9548::_read_register() {
  int8_t ret = -1;
  I2CBusOp* nu = _bus->new_op(BusOpcode::RX, this);
  if (nullptr != nu) {
    ret--;
    nu->dev_addr = _dev_addr;
    nu->sub_addr = -1;
    nu->setBuffer(&_shadow_r, 1);
    if (0 == _bus->queue_io_job(nu)) {
      ret = 0;
    }
  }
  return ret;
}


int8_t TCA9548::setMux(const uint8_t BUS_MASK) {
  int8_t ret = -1;
  _shadow_w = BUS_MASK;
  if (_busop_set_mux.isIdle()) {
    _bus->queue_io_job(&_busop_set_mux);
    ret = 0;
  }
  return ret;
}



int8_t TCA9548::chanActive(const uint8_t C, bool active) {
  int8_t ret = -1;
  if (C == (C & 0x07)) {
    ret--;
    if (initialized()) {
      ret--;
      const uint8_t MASK    = (1 << (C & 0x07));
      const uint8_t OLD_VAL = (_shadow_r & ~MASK);
      const uint8_t NEW_VAL = (MASK | OLD_VAL);
      if (NEW_VAL != OLD_VAL) {
        ret--;
        if (0 == setMux(NEW_VAL)) {
          ret = 0;
        }
      }
      else {
        ret = 1;
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

/* Transfers always permitted. */
int8_t TCA9548::io_op_callahead(BusOp* _op) {   return 0;   }


/*
* Register I/O calls back to this function for BOTH devices (MAG/IMU). So we
*   split the function up into two halves in private scope in the superclass.
* Bus operations that call back with errors are ignored.
*/
int8_t TCA9548::io_op_callback(BusOp* _op) {
  I2CBusOp* op  = (I2CBusOp*) _op;
  int8_t    ret = BUSOP_CALLBACK_NOMINAL;

  if (!_dev_found) {
    _dev_found = !op->hasFault();
  }

  if (!op->hasFault()) {
    if (&_busop_set_mux == op) {
      // Because we are constraining ourselves to a single outstanding write at
      //   a time, a write is as good as a read for the sake of our knowledge.
      _shadow_r = _shadow_w;
    }
    else {
      // A read.
    }
    _shadow_known = true;
  }
  return ret;
}



/*******************************************************************************
* Console callback
*******************************************************************************/
/*
*
*/
void TCA9548::printDebug(StringBuilder* output) {
  StringBuilder tmp;
  StringBuilder::styleHeader1(&tmp, "TCA9548");
  tmp.concatf("\tPins setup:  %c\n", _pins_confd ? 'y' : 'n');
  tmp.concatf("\tDev found:   %c\n", _dev_found ? 'y' : 'n');
  if (_dev_found) {
    tmp.concatf("\tInitialized: %c\n", initialized() ? 'y' : 'n');
    if (initialized()) {
      tmp.concat( "\t        0  1  2  3  4  5  6  7\n");
      tmp.concat( "\t------------------------------\n");
      tmp.concatf("\tActive: %c   %c   %c   %c   %c   %c   %c   %c\n",
        (chanActive(0) ? 'y' : ' '),
        (chanActive(1) ? 'y' : ' '),
        (chanActive(2) ? 'y' : ' '),
        (chanActive(3) ? 'y' : ' '),
        (chanActive(4) ? 'y' : ' '),
        (chanActive(5) ? 'y' : ' '),
        (chanActive(6) ? 'y' : ' '),
        (chanActive(7) ? 'y' : ' ')
      );
      tmp.concatf("\t%u: \n", initialized() ? 'y' : 'n');
    }
    else {
    }
  }
  output->concatHandoff(&tmp);
}


/**
* @page console-handlers
* @section tca9548-tools TCA9548 tools
*
* This is the console handler for using the TCA9548 i2c bus multiplexer driver.
*
* @subsection cmd-actions Actions
*
* Action    | Description | Additional arguments
* --------- | ----------- | --------------------
* `init`    | Manually invoke the driver's `init()` function. | None
* `reset`   | Manually invoke the driver's `reset()` function. | None
*/
int8_t TCA9548::console_handler(StringBuilder* text_return, StringBuilder* args) {
  int    ret    = 0;
  char*  cmd    = args->position_trimmed(0);
  int8_t chan   = (1 < args->count()) ? args->position_as_int(1) : -1;
  int8_t active = (2 < args->count()) ? args->position_as_int(2) : -1;

  if (0 == StringBuilder::strcasecmp(cmd, "init")) {
    text_return->concatf("TCA9548.init() returns %d.\n", init(_bus));
  }
  else if (0 == StringBuilder::strcasecmp(cmd, "chan")) {
    bool print_usage   = true;
    if ((0 <= chan) & (8 > chan)) {
      bool value_coerced = true;
      switch (active) {
        case 0:
        case 'n':
        case 'N':
          value_coerced = false;
          // NOTE: No break;
        case 1:
        case 'y':
        case 'Y':
          print_usage = false;
          text_return->concatf(
            "TCA9548.chanActive(%u, %u) returns %d.\n",
            chan,
            (value_coerced ? 1:0),
            chanActive(chan, value_coerced)
          );
          break;
        default:
          break;
      }
    }
    if (print_usage) {
      text_return->concatf("Usage: %s <index> [value]\n  ...where index is in range [0, 7], and value is ['n', 0, 'y', 1].\n", cmd);
    }
  }
  else {
    printDebug(text_return);
  }
  return ret;
}
