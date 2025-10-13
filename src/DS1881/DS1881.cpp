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


const char* const DS1881::errorToStr(DIGITALPOT_ERROR err) {
  switch (err) {
    case DIGITALPOT_ERROR::DEVICE_DISABLED:  return "DEVICE_DISABLED";
    case DIGITALPOT_ERROR::PEGGED_MAX:       return "PEGGED_MAX";
    case DIGITALPOT_ERROR::PEGGED_MIN:       return "PEGGED_MIN";
    case DIGITALPOT_ERROR::NO_ERROR:         return "NO_ERROR";
    case DIGITALPOT_ERROR::ABSENT:           return "ABSENT";
    case DIGITALPOT_ERROR::BUS:              return "BUS";
    case DIGITALPOT_ERROR::ALREADY_AT_MAX:   return "ALREADY_AT_MAX";
    case DIGITALPOT_ERROR::ALREADY_AT_MIN:   return "ALREADY_AT_MIN";
    case DIGITALPOT_ERROR::INVALID_POT:      return "INVALID_POT";
    default:                                 return "UNKNOWN";
  }
}


/*******************************************************************************
*   ___ _              ___      _ _              _      _
*  / __| |__ _ ______ | _ ) ___(_) |___ _ _ _ __| |__ _| |_ ___
* | (__| / _` (_-<_-< | _ \/ _ \ | / -_) '_| '_ \ / _` |  _/ -_)
*  \___|_\__,_/__/__/ |___/\___/_|_\___|_| | .__/_\__,_|\__\___|
*                                          |_|
* Constructors/destructors, class initialization functions and so-forth...
*******************************************************************************/

DS1881::DS1881(const uint8_t addr) : I2CDevice(addr), _flags(0) {
  for (uint8_t i = 0; i < sizeof(registers); i++) {   registers[i] = 0;  }
  for (uint8_t i = 0; i < sizeof(alt_values); i++) {  alt_values[i] = 0; }
}


DS1881::DS1881(const uint8_t* buf, const unsigned int len) : DS1881(*(buf + 1)) {
  unserialize(buf, len);
}


DS1881::~DS1881() {
}


/*******************************************************************************
* ___     _       _                      These members are mandatory overrides
*  |   / / \ o   | \  _     o  _  _      for implementing I/O callbacks. They
* _|_ /  \_/ o   |_/ (/_ \/ | (_ (/_     are also implemented by Adapters.
*******************************************************************************/
/**
* Reads all device registers from the hardware.
*
* @return 0 on success
*        -1 if the device isn't yet found
*        -2 if there is already I/O in flight
*        -3 I/O failure
*/
DIGITALPOT_ERROR DS1881::refresh() {
  DIGITALPOT_ERROR ret = DIGITALPOT_ERROR::BUS;
  if (nullptr != _bus) {
    I2CBusOp* op = _bus->new_op(BusOpcode::RX, this);
    if (nullptr != op) {
      op->dev_addr = _dev_addr;
      op->sub_addr = -1;
      op->setBuffer(registers, 3);
      op->shouldFreeBuffer(false);
      if (0 == queue_io_job(op)) {
        _ds_set_flag(DS1881_FLAG_IO_IN_FLIGHT);
        ret = DIGITALPOT_ERROR::NO_ERROR;
      }
    }
  }
  return ret;
}


/**
* Writes the given register with the given value to the hardware.
*
* @param reg is the register address
* @param len is the number of registers to write.
* @return 0 on success
*        -1 if the device isn't yet found
*        -2 if there is already I/O in flight
*        -3 on BusOp allocation failure
*        -4 I/O rejection
*/
int8_t DS1881::_write_register(uint8_t reg, uint8_t len) {
  int8_t ret = -1;
  if (devFound()) {
    ret--;
    uint8_t reg_idx = 2;
    switch (reg & 0xC0) {
      case DS1881_REG_WR0:  reg_idx--;  // NOTE: No break
      case DS1881_REG_WR1:  reg_idx--;  // NOTE: No break
      case DS1881_REG_CONF:
        {
          ret--;
          I2CBusOp* op = _bus->new_op(BusOpcode::TX, this);
          if (nullptr != op) {
            ret--;
            op->dev_addr = _dev_addr;
            op->sub_addr = -1;
            op->setBuffer(&registers[reg_idx], len);
            op->shouldFreeBuffer(false);
            if (0 == queue_io_job(op)) {
              _ds_set_flag(DS1881_FLAG_IO_IN_FLIGHT);
              ret = 0;
            }
          }
        }
      default:
        break;
    }
  }
  return ret;
}


int8_t DS1881::io_op_callback(BusOp* _op) {
  I2CBusOp* op  = (I2CBusOp*) _op;
  int8_t    ret = BUSOP_CALLBACK_NOMINAL;

  if (op->hasFault()) {
    ret = BUSOP_CALLBACK_ERROR;
  }
  else {
    const uint8_t byte0 = *(op->buffer());
    switch (op->get_opcode()) {
      case BusOpcode::RX:
        // This part doesn't support readback of an isolated register. So all
        //   three are read in a single operation. Execution of this block implies
        //   that all registers are fresh.

        _ds_set_flag(DS1881_FLAG_INITIALIZED, true);
        if (preserveOnDestroy()) {
            // TODO: Just because we want the hardware state to remain as-is when
            //   this driver is torn down doesn't imply that we don't want to
            //   clobber it when we start up.
        }
        //preserveOnDestroy(!(byte0 & 0x04));
        if (0x02 != (registers[2] & 0x03)) {
          // This driver enforces both zero-cross and high-resolution operation.
          // If those bits are not set, set them and re-write the value,
          //   preserving it in NVRAM.
          registers[2] = preserveOnDestroy() ? 0x86 : 0x82;
          _write_register(DS1881_REG_CONF, 1);
          //TODO: Faster and lighter:  ret = BUSOP_CALLBACK_RECYCLE;
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

      case BusOpcode::TX_CMD:
        // There is nothing like an ID register to read. So we construe a live
        //   ping of the given address to be evidence of presence.
        if (!devFound()) {
          _ds_set_flag(DS1881_FLAG_DEV_FOUND, true);
          refresh();  // TODO: Error handling...
        }
        break;

      default:
        break;
    }
  }

  _ds_set_flag(DS1881_FLAG_IO_IN_FLIGHT, (BUSOP_CALLBACK_RECYCLE == ret));
  return ret;
}



/*******************************************************************************
* Class-specific functions...                                                  *
*******************************************************************************/
/*
* Call to read the device and cause this class's state to reflect that of the device.
*/
DIGITALPOT_ERROR DS1881::init(I2CAdapter* b) {
  DIGITALPOT_ERROR ret = DIGITALPOT_ERROR::BUS;
  if (nullptr != b) {
    setAdapter(b);
  }
  if (nullptr != _bus) {
    _flags = (DS1881_FLAG_SERIAL_MASK & _flags);
    ping_device();
  }
  return ret;
}


/*
* Set the value of the given wiper to the given value.
*/
DIGITALPOT_ERROR DS1881::setValue(uint8_t pot, uint8_t val) {
  if (pot > 1)         return DIGITALPOT_ERROR::INVALID_POT;
  if (!initialized())  return DIGITALPOT_ERROR::DEVICE_DISABLED;
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
    const uint8_t REG_ID = (1 == pot ? DS1881_REG_WR1 : DS1881_REG_WR0);
    registers[pot] = REG_ID & (0x3F & tmp_val);
    if (0 != _write_register(REG_ID, 1)) {
      return_value = DIGITALPOT_ERROR::BUS;
    }
  }
  return return_value;
}


/*
* Set the value of the given wiper to the given value.
*/
DIGITALPOT_ERROR DS1881::setValue(uint8_t val) {
  if (!initialized())  return DIGITALPOT_ERROR::DEVICE_DISABLED;
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
    registers[0] = DS1881_REG_WR0 | (0x3F & tmp_val);
    registers[1] = DS1881_REG_WR1 | (0x3F & tmp_val);
    if (0 != _write_register(DS1881_REG_WR0, 2)) {
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
DIGITALPOT_ERROR DS1881::enable(bool x) {
  DIGITALPOT_ERROR return_value = DIGITALPOT_ERROR::NO_ERROR;
  return return_value;
}




int8_t DS1881::unserialize(const uint8_t* buf, const unsigned int len) {
  uint8_t offset = 0;
  uint8_t expected_sz = 255;
  if (len >= DS1881_SERIALIZE_SIZE) {
    uint8_t vals[3] = {0, 0, 0};
    switch (*(buf + offset++)) {
      case DS1881_SERIALIZE_VERSION:
        expected_sz = DS1881_SERIALIZE_SIZE;
        offset += 1;  // We'll have already constructed with _ADDR.
        _flags = (_flags & ~DS1881_FLAG_SERIAL_MASK) | (*(buf + offset++) & DS1881_FLAG_SERIAL_MASK);
        vals[0] = *(buf + offset++);
        vals[1] = *(buf + offset++);
        vals[2] = *(buf + offset++);
        break;
      default:  // Unhandled serializer version.
        return -1;
    }
    if (_ds_flag(DS1881_FLAG_INITIALIZED)) {
      // If the device has already been initialized, we impart the new conf.
      for (uint8_t i = 0; i < 3; i++) {
        if (0 != _write_register((i << 6), 1)) {
          return -2;
        }
      }
    }
    else {
      _ds_set_flag(DS1881_FLAG_FROM_BLOB);
      for (uint8_t i = 0; i < 3; i++) {
        registers[i] = vals[i];   // Save state for init()
      }
    }
  }
  return (expected_sz == offset) ? 0 : -1;
}


/*******************************************************************************
* Optional console stuff
*******************************************************************************/
#if defined(DS1881_CONSOLE)
/*
* Dump this item to the given buffer.
*/
void DS1881::printDebug(StringBuilder* output) {
  output->concatf("DS1881 digital potentiometer\n");
  I2CDevice::printDebug(output);
  output->concatf("\tInitialized:    %c\n", initialized() ? 'y' : 'n');
  if (initialized()) {
    output->concatf("\tCONF:           0x%02x\n", registers[2]);
    output->concatf("\tRange:          %u\n", getRange());
    output->concatf("\tZero-cross:     %c\n", zerocrossWait() ? 'y' : 'n');
    for (int i = 0; i < 2; i++) {
      output->concatf("\tPOT %u:  %u  (Alt: %u)\n", i, 0x3F & registers[i], alt_values[i]);
    }
  }
  else {
    output->concat("\tNot initialized\n");
  }
  output->concatf("\n");
}



/**
* @page console-handlers
* @section ds1881-tools DS1881 tools
*
* This is the console handler for using the DS1881 digipot. If invoked without
*   arguments, it will print its state.
*
* @subsection cmd-actions Actions
*
* Action    | Description | Additional arguments
* --------- | ----------- | --------------------
* `init`    | Manually invoke the driver's `init()` function. | None
* `regs`    | Show the register shadows. | None
* `refresh` | Refresh the register shadows from the hardware. | None
*/
int8_t DS1881::console_handler(StringBuilder* text_return, StringBuilder* args) {
  int ret = 0;
  if (0 < args->count()) {
    char* cmd = args->position_trimmed(0);
    if (0 == StringBuilder::strcasecmp(cmd, "refresh")) {
      text_return->concatf("refresh() returns %d.\n", refresh());
    }
    else if (0 == StringBuilder::strcasecmp(cmd, "init")) {
      text_return->concatf("init() returns %d.\n", init());
    }
    else if (0 == StringBuilder::strcasecmp(cmd, "regs")) {
      text_return->concatf("TODO: Unimplemented.\n");
    }
    else if (0 == StringBuilder::strcasecmp(cmd, "vol")) {
      const uint8_t VAL = (uint8_t) args->position_as_int(1);
      text_return->concatf("setValue(%u) returns %d,\n", VAL, setValue(VAL));
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

#endif  // DS1881_CONSOLE
