/*
File:   DS1881.cpp
Author: J. Ian Lindsay
Date:   2016.12.26

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

*/

#include "DS1881.h"

#define DS1881_REG_WR0   0x00
#define DS1881_REG_WR1   0x40
#define DS1881_REG_CONF  0x80



/*******************************************************************************
*   ___ _              ___      _ _              _      _
*  / __| |__ _ ______ | _ ) ___(_) |___ _ _ _ __| |__ _| |_ ___
* | (__| / _` (_-<_-< | _ \/ _ \ | / -_) '_| '_ \ / _` |  _/ -_)
*  \___|_\__,_/__/__/ |___/\___/_|_\___|_| | .__/_\__,_|\__\___|
*                                          |_|
* Constructors/destructors, class initialization functions and so-forth...
*******************************************************************************/

DS1881::DS1881(uint8_t addr) : I2CDevice(addr) {
  dev_init    = false;
  preserve_state_on_destroy = false;
}


DS1881::~DS1881() {
}


/*******************************************************************************
* ___     _       _                      These members are mandatory overrides
*  |   / / \ o   | \  _     o  _  _      for implementing I/O callbacks. They
* _|_ /  \_/ o   |_/ (/_ \/ | (_ (/_     are also implemented by Adapters.
*******************************************************************************/
int8_t DS1881::io_op_callback(BusOp* op) {
  I2CBusOp* completed = (I2CBusOp*) op;
  uint8_t byte0 = *(completed->buffer()+0);

  switch (completed->get_opcode()) {
    case BusOpcode::RX:
      if (!completed->hasFault()) {
        dev_init = true;
      }
      switch (byte0 & 0xC0) {
        case DS1881_REG_WR0:
        case DS1881_REG_WR1:
          break;
        case DS1881_REG_CONF:
          preserve_state_on_destroy = !(byte0 & 0x04);
          if (0x02 != (byte0 & 0x03)) {
            // Enforces zero-cross and high-resolution.
            registers[2] = preserve_state_on_destroy ? 0x86 : 0x82;
            writeX(-1, 1, &registers[2]);
          }
          break;
        default:
          break;
      }
      break;
    case BusOpcode::TX:
      switch (byte0 & 0xC0) {
        case DS1881_REG_WR0:
        case DS1881_REG_WR1:
        case DS1881_REG_CONF:
        default:
          break;
      }
      break;
    default:
      break;
  }
  return 0;
}


/*
* Dump this item to the dev log.
*/
void DS1881::printDebug(StringBuilder* output) {
  output->concat("DS1881 digital potentiometer");
  output->concat(PRINT_DIVIDER_1_STR);
  I2CDevice::printDebug(output);

  if (!dev_init) {
    output->concat("\tNot initialized\n");
    return;
  }

  for (int i = 0; i < 1; i++) {
    output->concatf("\tPOT %d: 0x%02x\n", i, 0x3F & registers[i]);
  }
  output->concatf("\n");
}



/*******************************************************************************
* Class-specific functions...                                                  *
*******************************************************************************/
/*
* Call to read the device and cause this class's state to reflect that of the device.
*/
DIGITALPOT_ERROR DS1881::init() {
  DIGITALPOT_ERROR return_value = DIGITALPOT_ERROR::BUS;
  if (readX(-1, 3, registers)) {
    return_value = DIGITALPOT_ERROR::NO_ERROR;
  }
  return return_value;
}


/*
* Set the value of the given wiper to the given value.
*/
DIGITALPOT_ERROR DS1881::setValue(uint8_t pot, uint8_t val) {
  if (pot > 1)    return DIGITALPOT_ERROR::INVALID_POT;
  if (!dev_init)  return DIGITALPOT_ERROR::DEVICE_DISABLED;

  DIGITALPOT_ERROR return_value = DIGITALPOT_ERROR::NO_ERROR;
  uint8_t tmp_val = strict_min(val, (uint8_t) 63);
  switch (tmp_val) {
    case 0:
      return_value = DIGITALPOT_ERROR::PEGGED_MIN;
      break;
    case 63:
      return_value = (tmp_val == val) ? DIGITALPOT_ERROR::PEGGED_MAX : DIGITALPOT_ERROR::ALREADY_AT_MAX;
      break;
    default:
      break;
  }
  if (0 <= (int8_t) return_value) {
    alt_values[pot] = registers[pot];
    registers[pot] = (1 == pot ? DS1881_REG_WR1 : DS1881_REG_WR0) & tmp_val;
    if (!writeX(-1, 1, &registers[pot])) {
      return_value = DIGITALPOT_ERROR::BUS;
    }
  }
  return return_value;
}


/*
* Set the value of the given wiper to the given value.
*/
DIGITALPOT_ERROR DS1881::setValue(uint8_t val) {
  if (!dev_init)  return DIGITALPOT_ERROR::DEVICE_DISABLED;
  DIGITALPOT_ERROR return_value = DIGITALPOT_ERROR::NO_ERROR;
  uint8_t tmp_val = strict_min(val, (uint8_t) 63);
  switch (tmp_val) {
    case 0:
      return_value = DIGITALPOT_ERROR::PEGGED_MIN;
      break;
    case 63:
      return_value = (tmp_val == val) ? DIGITALPOT_ERROR::PEGGED_MAX : DIGITALPOT_ERROR::ALREADY_AT_MAX;
      break;
    default:
      break;
  }
  if (0 <= (int8_t) return_value) {
    alt_values[0] = registers[0];
    alt_values[1] = registers[1];
    registers[0] = DS1881_REG_WR0 & tmp_val;
    registers[1] = DS1881_REG_WR1 & tmp_val;
    if (!writeX(-1, 2, registers)) {
      return_value = DIGITALPOT_ERROR::BUS;
    }
  }
  return return_value;
}


DIGITALPOT_ERROR DS1881::reset(uint8_t val) {
  return setValue(val);
}


/*
* Enabling the device unmutes it by restoring the previous value from the alt_values array.
* Disabling the device stacks the current value into alt_values and then sets to mute.
* Retains wiper settings.
*/
DIGITALPOT_ERROR DS1881::_enable(bool x) {
  DIGITALPOT_ERROR return_value = DIGITALPOT_ERROR::NO_ERROR;
  return return_value;
}
