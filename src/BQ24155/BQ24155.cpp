/*
File:   BQ24155.cpp
Author: J. Ian Lindsay
Date:   2017.06.08

Copyright 2016 Manuvr, Inc

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.


Support for the Texas Instruments li-ion charger.
*/

#include "BQ24155.h"

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

const char* BQ24155_STATE_STR[] = {
  "READY",
  "CHARGING",
  "CHARGED",
  "FAULT"
};

const char* BQ24155_FAULT_STR[] = {
  "NOMINAL",
  "VBUS_OVP",
  "SLEEP",
  "VBUS_SAG",
  "OUTPUT_OVP",
  "THERMAL",
  "TIMER",
  "NO_BATTERY"
};


/* Converts a register address back into an enum. */
BQ24155RegID BQ24155::_reg_id_from_addr(const uint8_t reg_addr) {
  switch (reg_addr & 0x3F) {
    case 0x00:   return BQ24155RegID::STATUS;
    case 0x01:   return BQ24155RegID::LIMITS;
    case 0x02:   return BQ24155RegID::BATT_REGU;
    case 0x03:   return BQ24155RegID::PART_REV;
    case 0x04:   return BQ24155RegID::FAST_CHRG;
  }
  return BQ24155RegID::INVALID;
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
BQ24155::BQ24155(const BQ24155Opts* o) : I2CDevice(BQ24155_I2CADDR), _opts(o), _flgs(_opts._flgs_initial) {
}


/*
* Destructor.
*/
BQ24155::~BQ24155() {}


int8_t BQ24155::init() {
  if (_flgs.value(BQ24155_FLAG_PINS_INITD)) {
    if (_opts.useISELPin()) {
      // This is the default value. If we do not set the pin high, we risk interrupting
      //   our own power supply if a battery is not present.
      pinMode(_opts.isel_pin, GPIOMode::OUTPUT);
      setPin(_opts.isel_pin, _isel_state());
    }
    if (_opts.useStatPin()) {
      // TODO: Choose.
      pinMode(_opts.stat_pin, GPIOMode::INPUT_PULLUP);
      // int8_t setPinEvent(_opts._stat_pin, uint8_t condition, ManuvrMsg* isr_event);
      // int8_t setPinFxn(_opts._stat_pin, uint8_t condition, FxnPointer fxn);
    }
    _flgs.set(BQ24155_FLAG_PINS_INITD);
  }

  // Wipe the init indicators to prevent immediate commit, before we start
  //   writing registers.
  _flgs.raw &= ~BQ24155_FLAG_MASK_INIT_CMPLT;
  return refresh();
}



/*******************************************************************************
* Functions specific to this class....                                         *
*******************************************************************************/

/**
* Applies the user-defined battery parameters to the charger.
* NOTE: If the device is operating without a battery, calling this function
*   amounts to fiddling with our own regulator. Risk of brown-outs and other
*   hard to debug problems.
*
* @return 0 on success. Nonzero otherwise.
*/
int8_t BQ24155::_post_discovery_init() {
  int8_t ret = -1;
  if (disableSTATPin()) {
    uint8_t int_val = _get_shadow_value(BQ24155RegID::STATUS);
    _set_shadow_value(BQ24155RegID::STATUS, (int_val & 0xBF));
  }
  // Set the limit register.
  if (0 == charger_enabled(true, true)) {
    ret--;
    if (0 == batt_weak_voltage(_opts.battery.voltage_weak, true)) {
      ret--;
      if (0 == batt_reg_voltage(_opts.battery.voltage_max)) {
        ret--;
        if (0 == usb_current_limit(_opts.src_limit)) {  // Causes the LIMITS register to be written.
          ret = 0;
        }
      }
    }
  }
  c3p_log(LOG_LEV_INFO, __PRETTY_FUNCTION__, "_post_discovery_init(): %d", ret);
  return ret;
}


/**
* Returns the charger's current state.
*/
int8_t BQ24155::refresh() {
  return _read_registers(BQ24155RegID::STATUS, 5);
}

int8_t BQ24155::refresh_status() {
  return _read_registers(BQ24155RegID::STATUS, 1);
}

/**
* Returns the charger's current state.
*/
BQ24155State BQ24155::getChargerState() {
  return (BQ24155State) (0x03 & ((uint8_t) _get_shadow_value(BQ24155RegID::STATUS)) >> 4);
}

/**
* Returns the fault condition.
*/
BQ24155Fault BQ24155::getFault() {
  return (BQ24155Fault) (0x07 & (uint8_t) _get_shadow_value(BQ24155RegID::STATUS));
}

/**
*
* @return 0 on success, non-zero otherwise.
*/
int8_t BQ24155::punch_safety_timer() {
  int8_t ret = -1;
  if (initComplete()) {
    uint8_t int_val = _get_shadow_value(BQ24155RegID::STATUS);
    ret--;
    if (0 == _set_shadow_value(BQ24155RegID::STATUS, (0x80 | int_val))) {
      ret = _write_registers(BQ24155RegID::STATUS, 1);
    }
  }
  return ret;
}

/**
* Reset the charger's internal state-machine.
*
* @return 0 on success, non-zero otherwise.
*/
int8_t BQ24155::reset_charger_fsm() {
  int8_t ret = -1;
  if (initComplete()) {
    uint8_t int_val = _get_shadow_value(BQ24155RegID::FAST_CHRG);
    ret--;
    if (0 == _set_shadow_value(BQ24155RegID::FAST_CHRG, (0x80 | int_val))) {
      ret = _write_registers(BQ24155RegID::FAST_CHRG, 1);
    }
  }
  return ret;
}

/**
* Set the battery voltage. It is very important to get this right.
*
* @param desired Valid range is [3.5 - 4.44]
* @return 0 on success, non-zero otherwise.
*/
int8_t BQ24155::batt_reg_voltage(float desired) {
  int8_t ret = -1;
  if ((desired >= BQ24155_VOREGU_OFFSET) && (desired < 4.44f)) {   //
    uint8_t offset_val = (uint8_t) ((desired - BQ24155_VOREGU_OFFSET) / 0.02);
    ret--;
    if (0 == _set_shadow_value(BQ24155RegID::BATT_REGU, (offset_val << 2))) {
      ret = _write_registers(BQ24155RegID::BATT_REGU, 1);
    }
  }
  return ret;
}

/**
* Get the battery voltage.
*
* @return The battery regulation voltage.
*/
float BQ24155::batt_reg_voltage() {
  uint8_t int_val = _get_shadow_value(BQ24155RegID::BATT_REGU);
  return ((int_val >> 2) * 0.02f) + BQ24155_VOREGU_OFFSET;
}

/**
* Get the battery weakness threshold.
*
* @return The number of mV at which point the battery is considered weak.
*/
float BQ24155::batt_weak_voltage() {
  uint8_t int_val = _get_shadow_value(BQ24155RegID::LIMITS);
  return (((int_val >> 4) & 0x03) * 0.1f) + BQ24155_VLOW_OFFSET;
}

/**
* Sets the voltage at which the battery is considered weak.
* Charger has 2-bits for this purpose, with 100mV increments.
*
* @param The number of mV at which point the battery is to be considered weak.
* @return 0 on success, non-zero otherwise.
*/
int8_t BQ24155::batt_weak_voltage(float desired, bool defer) {
  int8_t ret = -1;
  if ((desired >= BQ24155_VLOW_OFFSET) && (desired <= 3.7f)) {
    uint8_t bw_step = 0;
    ret--;
    if (3.5f <= desired) {  bw_step++;  }
    if (3.6f <= desired) {  bw_step++;  }
    if (3.7f <= desired) {  bw_step++;  }
    uint8_t int_val = _get_shadow_value(BQ24155RegID::LIMITS);
    if (0 == _set_shadow_value(BQ24155RegID::LIMITS, (int_val & 0xCF) | (bw_step << 4))) {
      ret = defer ? 0 : _write_registers(BQ24155RegID::LIMITS, 1);
    }
  }
  return ret;
}

/**
* @return The number of mA the charger is limited to outputing.
*/
int16_t BQ24155::charge_current_limit() {
  return _isel_state() ? 500 : 100;
}

/**
* What is the host-imposed (or self-configured) current limit?
* Answer given in amps. For the sake of confining our concerns to finite
*   arithmetic, we will interpret the "unlimited" condition reported by the
*   charger to mean some very large (but limited) number, beyond the TDP of the
*   charger IC itself.
*
* @return The number of mA the USB host is able to supply. -1 if unlimited.
*/
int16_t BQ24155::usb_current_limit() {
  uint8_t int_val = _get_shadow_value(BQ24155RegID::LIMITS);
  switch ((int_val >> 6) & 0x03) {
    case 3:  return -1;    // This is practically unlimited in this case.
    case 2:  return 800;
    case 1:  return 500;
  }
  return 100;  // This is the default condition.
}

/**
* We can artificially limit the draw from the USB port.
*
* @param The number of mA to draw from the USB host.
* @return 0 on success, non-zero otherwise.
*/
int8_t BQ24155::usb_current_limit(int16_t milliamps, bool defer) {
  BQ24155USBCurrent climit_enum = BQ24155USBCurrent::LIMIT_100;
  if (milliamps > 800) {       climit_enum = BQ24155USBCurrent::NO_LIMIT;  }
  else if (milliamps > 500) {  climit_enum = BQ24155USBCurrent::LIMIT_800; }
  return usb_current_limit(climit_enum, defer);
}

/**
* We can artificially limit the draw from the USB port.
*
* @param The number of mA to draw from the USB host.
* @return 0 on success, non-zero otherwise.
*/
int8_t BQ24155::usb_current_limit(BQ24155USBCurrent milliamps, bool defer) {
  int8_t ret = -1;
  uint8_t c_step  = 0;
  uint8_t int_val = _get_shadow_value(BQ24155RegID::LIMITS);
  switch (milliamps) {
    case BQ24155USBCurrent::NO_LIMIT:  c_step++;
    case BQ24155USBCurrent::LIMIT_800: c_step++;
    case BQ24155USBCurrent::LIMIT_500: c_step++;
    case BQ24155USBCurrent::LIMIT_100:
    default:  // NOTE: The list obove is exhaustive, but some GCC builds don't know so.
      if (0 == _set_shadow_value(BQ24155RegID::LIMITS, (int_val & 0x3F) | (c_step << 6))) {
        ret = defer ? 0 : _write_registers(BQ24155RegID::LIMITS, 1);
      }
      break;
  }
  return ret;
}

/**
* Is the charger enabled?
*
* @return True if the charger is enabled.
*/
bool BQ24155::charger_enabled() {
  return (0 == (0x04 & (uint8_t) _get_shadow_value(BQ24155RegID::LIMITS)));
}

/**
* Enable or disable the charger.
*
* @param True to enable the charger.
* @return 0 on success, non-zero otherwise.
*/
int8_t BQ24155::charger_enabled(bool en, bool defer) {
  int8_t ret = 0;
  uint8_t int_val = _get_shadow_value(BQ24155RegID::LIMITS);
  if ((!en) ^ (int_val & 0x04)) {
    ret--;
    int_val = (!en) ? (int_val | 0x04) : (int_val & 0xFB);
    if (0 == _set_shadow_value(BQ24155RegID::LIMITS, int_val)) {
      ret = defer ? 0 : _write_registers(BQ24155RegID::LIMITS, 1);
    }
  }
  return ret;
}

/**
* Is the charger in Hi-Z mode?
*
* @return True if the charger is enabled.
*/
bool BQ24155::hi_z_mode() {
  return (0x02 & (uint8_t) _get_shadow_value(BQ24155RegID::LIMITS));
}

/**
* Enable or disable Hi-Z.
*
* @param True to enable the charger.
* @return 0 on success, non-zero otherwise.
*/
int8_t BQ24155::hi_z_mode(bool en, bool defer) {
  int8_t ret = -1;
  uint8_t int_val = _get_shadow_value(BQ24155RegID::LIMITS);
  if (en ^ (int_val & 0x02)) {
    int_val = (en) ? (int_val | 0x02) : (int_val & 0xFC);
    if (0 == _set_shadow_value(BQ24155RegID::LIMITS, int_val)) {
      ret = defer ? 0 : _write_registers(BQ24155RegID::LIMITS, 1);
    }
  }
  return ret;
}

/**
* Is the charge termination phase enabled?
*
* @return True if the charge termination phase is enabled.
*/
bool BQ24155::charge_current_termination_enabled() {
  return (0x08 & (uint8_t) _get_shadow_value(BQ24155RegID::LIMITS));
}

/**
* Enable or disable the charge termination phase.
*
* @param True to enable the charge termination phase.
* @return 0 on success, non-zero otherwise.
*/
int8_t BQ24155::charge_current_termination_enabled(bool en, bool defer) {
  int8_t ret = -1;
  uint8_t int_val = _get_shadow_value(BQ24155RegID::LIMITS);
  if (en ^ (int_val & 0x08)) {
    int_val = (en) ? (int_val | 0x08) : (int_val & 0xF7);
    if (0 == _set_shadow_value(BQ24155RegID::LIMITS, int_val)) {
      ret = defer ? 0 : _write_registers(BQ24155RegID::LIMITS, 1);
    }
  }
  return ret;
}


/**
* @param ISEL pin state.
*/
void BQ24155::_isel_state(bool x) {
  _flgs.set(BQ24155_FLAG_ISEL_HIGH, x);
  if (_opts.useISELPin()) {
    setPin(_opts.isel_pin, x);
  }
}



int8_t  BQ24155::_set_shadow_value(BQ24155RegID r, uint8_t val) {
  int8_t ret = 0;
  switch (r) {
    case BQ24155RegID::STATUS:  // This register only has two writable bits.
      shadows[(uint8_t) r] = val & 0xC0;
      break;
    case BQ24155RegID::BATT_REGU:
      shadows[(uint8_t) r] = val & 0xFC;
      break;
    case BQ24155RegID::LIMITS:
    case BQ24155RegID::FAST_CHRG:
      shadows[(uint8_t) r] = val;
      break;
    default:
      ret = -1;
      break;
  }
  return ret;
}


uint8_t BQ24155::_get_shadow_value(BQ24155RegID r) {
  uint8_t ret = 0;
  switch (r) {
    case BQ24155RegID::STATUS:
    case BQ24155RegID::LIMITS:
    case BQ24155RegID::BATT_REGU:
    case BQ24155RegID::PART_REV:
    case BQ24155RegID::FAST_CHRG:
      ret = shadows[(uint8_t) r];
      break;
    default:
      // Illegal. A bad mistake was made somewhere.
      break;
  }
  return ret;
}


int8_t  BQ24155::_read_registers(BQ24155RegID r, uint8_t len) {
  int8_t ret = -1;
  uint8_t* ptr = &shadows[(uint8_t) r];
  I2CBusOp* op = _bus->new_op(BusOpcode::RX, this);
  if (nullptr != op) {
    ret--;
    op->dev_addr = _dev_addr;
    op->sub_addr = (uint8_t) r;
    op->setBuffer(ptr, len);
    if (0 == _bus->queue_io_job(op)) {
      ret = 0;
    }
  }
  return ret;
}


int8_t  BQ24155::_write_registers(BQ24155RegID r, uint8_t len) {
  int8_t ret = -1;
  uint8_t* ptr = &shadows[(uint8_t) r];
  I2CBusOp* op = _bus->new_op(BusOpcode::TX, this);
  if (nullptr != op) {
    ret--;
    op->dev_addr = _dev_addr;
    op->sub_addr = (uint8_t) r;
    op->setBuffer(ptr, len);
    if (0 == _bus->queue_io_job(op)) {
      ret = 0;
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
int8_t BQ24155::io_op_callahead(BusOp* _op) {   return 0;   }


/*
* Register I/O calls back to this function for BOTH devices (MAG/IMU). So we
*   split the function up into two halves in private scope in the superclass.
* Bus operations that call back with errors are ignored.
*/
int8_t BQ24155::io_op_callback(BusOp* _op) {
  I2CBusOp* op = (I2CBusOp*) _op;
  int8_t ret = BUSOP_CALLBACK_NOMINAL;

  if (!op->hasFault()) {
    uint8_t* buf     = op->buffer();
    uint32_t len     = op->bufferLen();
    uint8_t  reg_idx = (uint8_t) _reg_id_from_addr(op->sub_addr);
    bool run_post_init_fxn = false;
    switch (op->get_opcode()) {
      case BusOpcode::TX:
        for (uint32_t i = 0; i < len; i++) {
          uint8_t value = *buf++;
          switch ((BQ24155RegID) reg_idx) {
            case BQ24155RegID::STATUS:
              _flgs.set(BQ24155_FLAG_INIT_CTRL);
              break;
            case BQ24155RegID::LIMITS:
              _flgs.set(BQ24155_FLAG_INIT_LIMITS);
              _flgs.clear(BQ24155_FLAG_LIMIT_WRITING);
              break;
            case BQ24155RegID::BATT_REGU:
              _flgs.set(BQ24155_FLAG_INIT_BATT_REG);
              break;
            case BQ24155RegID::FAST_CHRG:
              _flgs.set(BQ24155_FLAG_INIT_FAST_CHRG);
              break;
            default:  // Illegal. A bad mistake was made somewhere.
              break;
          }
          reg_idx++;
        }
        break;

      case BusOpcode::RX:
        for (uint32_t i = 0; i < len; i++) {
          uint8_t val = *buf++;
          switch ((BQ24155RegID) reg_idx) {
            case BQ24155RegID::STATUS:
              _isel_state(val & 0x80);
              break;
            case BQ24155RegID::LIMITS:
              break;
            case BQ24155RegID::BATT_REGU:
              break;
            case BQ24155RegID::PART_REV:
              // Must be 0b01001xxx. If so, we init...
              if (devFound() && !initComplete()) {
                run_post_init_fxn = true;
              }
              break;
            case BQ24155RegID::FAST_CHRG:
              break;
            default:  // Illegal. A bad mistake was made somewhere.
              break;
          }
          reg_idx++;
        }
        if (run_post_init_fxn) {
          _post_discovery_init();
        }
        break;

      default:
        break;
    }
  }
  return ret;
}


/**
* Dump this item to the dev log.
*
* @param  output  The buffer to receive the output.
*/
void BQ24155::printDebug(StringBuilder* output) {
  output->concatf("-- BQ24155 r%u %sinitialized\n", _part_revision(), (initComplete() ? "" : "un"));
  output->concatf("\tSTAT pin:          %u (%sabled)\n", _opts.stat_pin, (disableSTATPin()) ? "Dis" : "En");
  output->concatf("\tISEL pin:          %u (State: %c)\n", _opts.isel_pin, _isel_state() ? 'H' : 'L');
  output->concatf("\tSense resistor:    %d mOhm\n", _opts.sense_milliohms);
  output->concatf("\tCharger state:     %s\n", (const char*) BQ24155_STATE_STR[(uint8_t) getChargerState()]);
  output->concatf("\tFault:             %s\n", (const char*) BQ24155_FAULT_STR[(uint8_t) getFault()]);
  output->concat("\tLimits:\n");
  output->concatf("\t  USB current:     %d mA\n", usb_current_limit());
  output->concatf("\t  Charge current:  %d mA\n", charge_current_limit());
  output->concatf("\t  Battery weak:    %.2fV\n", (double) batt_weak_voltage());
  output->concatf("\t  Batt regulation: %.2fV\n", (double) batt_reg_voltage());
  output->concat("\tFast-charge:\n\t  Bulk rates  Termination rates\n\t  ----------  -----------------\n");
  uint8_t c_btc_idx = (_get_shadow_value(BQ24155RegID::FAST_CHRG) & 0x07);
  uint8_t c_bcc_idx = ((_get_shadow_value(BQ24155RegID::FAST_CHRG) >> 4) & 0x07);
  for (int i = 0; i < 8; i++) {
    output->concatf("\t  [%c] %.3fA  [%c] %.3fA\n",
      (c_bcc_idx == i) ? '*' : ' ',
      (double) bulkChargeCurrent(i),
      (c_btc_idx == i) ? '*' : ' ',
      (double) terminateChargeCurrent(i)
    );
  }
}

/*
* Dump the contents of this device to the logger.
*/
void BQ24155::printRegisters(StringBuilder* output) {
  StringBuilder::styleHeader1(output, "BQ24155 shadows");
  StringBuilder::printBuffer(output, shadows, sizeof(shadows), "\t");
  output->concat("\n");
}
