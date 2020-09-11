/*
File:   SX1503.cpp
Author: J. Ian Lindsay
Date:   2019.11.30

Copyright 2019 Manuvr, Inc

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

#include "SX1503.h"
#include <StringBuilder.h>

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

volatile static bool sx1503_isr_fired = false;

/* Real register addresses */
static const uint8_t SX1503_REG_ADDR[31] = {
  0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
  0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F,
  0x10, 0x11, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25,
  0x26, 0x27, 0x28, 0x29, 0x2A, 0x2B, 0xAD
};



/*
* This is an ISR.
*/
void sx1503_isr() {
  sx1503_isr_fired = true;
}


const SX1503RegId _reg_id_from_addr(uint8_t addr) {
  switch (addr) {
    case 0x00:   return SX1503RegId::DATA_B;
    case 0x01:   return SX1503RegId::DATA_A;
    case 0x02:   return SX1503RegId::DIR_B;
    case 0x03:   return SX1503RegId::DIR_A;
    case 0x04:   return SX1503RegId::PULLUP_B;
    case 0x05:   return SX1503RegId::PULLUP_A;
    case 0x06:   return SX1503RegId::PULLDOWN_B;
    case 0x07:   return SX1503RegId::PULLDOWN_A;
    case 0x08:   return SX1503RegId::IRQ_MASK_B;
    case 0x09:   return SX1503RegId::IRQ_MASK_A;
    case 0x0A:   return SX1503RegId::SENSE_H_B;
    case 0x0B:   return SX1503RegId::SENSE_H_A;
    case 0x0C:   return SX1503RegId::SENSE_L_B;
    case 0x0D:   return SX1503RegId::SENSE_L_A;
    case 0x0E:   return SX1503RegId::IRQ_SRC_B;
    case 0x0F:   return SX1503RegId::IRQ_SRC_A;
    case 0x10:   return SX1503RegId::EVENT_STAT_B;
    case 0x11:   return SX1503RegId::EVENT_STAT_A;
    case 0x20:   return SX1503RegId::PLD_MODE_B;
    case 0x21:   return SX1503RegId::PLD_MODE_A;
    case 0x22:   return SX1503RegId::PLD_TABLE_0B;
    case 0x23:   return SX1503RegId::PLD_TABLE_0A;
    case 0x24:   return SX1503RegId::PLD_TABLE_1B;
    case 0x25:   return SX1503RegId::PLD_TABLE_1A;
    case 0x26:   return SX1503RegId::PLD_TABLE_2B;
    case 0x27:   return SX1503RegId::PLD_TABLE_2A;
    case 0x28:   return SX1503RegId::PLD_TABLE_3B;
    case 0x29:   return SX1503RegId::PLD_TABLE_3A;
    case 0x2A:   return SX1503RegId::PLD_TABLE_4B;
    case 0x2B:   return SX1503RegId::PLD_TABLE_4A;
    case 0xAD:   return SX1503RegId::ADVANCED;
  }
  return SX1503RegId::INVALID;
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
SX1503::SX1503(const uint8_t irq_pin, const uint8_t reset_pin) :
  I2CDevice(SX1503_I2C_ADDR), _IRQ_PIN(irq_pin), _RESET_PIN(reset_pin) {}

/*
* Constructor.
*/
SX1503::SX1503(const uint8_t* buf, const unsigned int len) : SX1503(*(buf + 1), *(buf + 2)) {
  unserialize(buf, len);
}

/*
* Destructor.
*/
SX1503::~SX1503() {
  if (255 != _IRQ_PIN) {
    unsetPinFxn(_IRQ_PIN);
  }
  if (!preserveOnDestroy() && (255 != _RESET_PIN)) {
    setPin(_RESET_PIN, 0);  // Leave the part in reset state.
  }
}


bool SX1503::isrFired() {
  return (255 != _IRQ_PIN) ? sx1503_isr_fired : true;
}


/*
* @return 0 on success, non-zero otherwise.
*/
int8_t SX1503::init(I2CAdapter* b) {
  int8_t ret = -1;
  _sx_clear_flag(SX1503_FLAG_INITIALIZED);
  if (!_sx_flag(SX1503_FLAG_PINS_CONFD)) {
    _ll_pin_init();
  }
  if (nullptr != b) {
    _bus = b;
  }

  if (_from_blob()) {
    // Copy the blob-imparted values and clear the flag so we don't do this again.
    _sx_clear_flag(SX1503_FLAG_FROM_BLOB);
    int8_t ret = _write_registers(SX1503RegId::DATA_B, 0x12);
    if (0 == ret) {  ret = _write_registers(SX1503RegId::PLD_MODE_B, 0x0C);  }
    if (0 == ret) {  ret = _write_registers(SX1503RegId::ADVANCED, 1);       }
    if (0 != ret) {
      return -3;
    }
    ret = 0;
  }
  else if (preserveOnDestroy()) {
    // We take no action against the present hardware state. Just read it.
    ret = refresh();
  }
  else {
    // Reset ahead of baseline class configuration.
    ret = reset();
  }
  return ret;
}


int8_t SX1503::reset() {
  int8_t ret = -1;
  _a_dat = 0;
  _b_dat = 0;
  _sx_clear_flag(SX1503_FLAG_INITIALIZED);
  sx1503_isr_fired = false;
  uint8_t vals[31] = {
    0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00,
    0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
  };
  for (uint8_t i = 0; i < 31; i++) {
    registers[i] = vals[i];
  }
  if (255 != _RESET_PIN) {
    setPin(_RESET_PIN, 0);
    sleep_us(1);   // Datasheet says 300ns.
    setPin(_RESET_PIN, 1);
    sleep_us(10);
    if (255 != _IRQ_PIN) {
      // Wait on the IRQ pin to go high.
      uint32_t millis_abort = millis() + 15;
      while ((millis() < millis_abort) && (!readPin(_IRQ_PIN))) {}
      if (readPin(_IRQ_PIN)) {
        ret = 0;
      }
    }
    else {
      sleep_ms(15);   // Datasheet says chip comes back in 7ms.
    }
    ret = _write_register(SX1503RegId::ADVANCED, 0x04);
  }
  else {
    // Steamroll the registers with the default values.
    ret = _write_registers(SX1503RegId::DATA_B, 0x12);
    if (0 == ret) {  ret = _write_registers(SX1503RegId::PLD_MODE_B, 0x0C);  }
    if (0 == ret) {  ret = _write_registers(SX1503RegId::ADVANCED, 1);       }
  }
  return ret;
}


/*
* Poll the class for updates.
* If the application has setup a callback for a given pin, its change will not
*   be counted in the return value. Thus, if callbacks are setup for every pin
*   that is configured for input, this function will never return a value (>0).
*
* Returns...
*   -3 if not initialized.
*   -1 if the hardware needed to be read, but doing so failed.
*   0  if nothing needs doing.
*   1  if a read was initiated.
*/
int8_t SX1503::poll() {
  int8_t ret = -3;
  if (initialized()) {
    ret = 0;
    if (255 == _IRQ_PIN) {
      // Without an IRQ pin, we hammer this every polling cycle.
      sx1503_isr_fired = true;
    }
    if (sx1503_isr_fired) {
      ret--;
      _sx_set_flag(SX1503_FLAG_READ_IN_FLIGHT);
      if (0 == _read_registers(SX1503RegId::DATA_B, 2)) {
        // This will only be set again once the IRQ condition clears in the
        //   hardware, and is subsequently re-asserted. So if the read fails for
        //   some reason, or the IRQ autoclear bit isn't set in the ADVANCED
        //   register, the class will hang.
        // TODO: More care is warranted to ensure that this doesn't happen.
        sx1503_isr_fired = false;
        ret = 1;
      }
    }
  }
  return ret;
}


int8_t SX1503::refresh() {
  int8_t ret = _read_registers(SX1503RegId::DATA_B, 0x12);
  if (0 == ret) {  ret = _read_registers(SX1503RegId::PLD_MODE_B, 0x0C);  }
  if (0 == ret) {  ret = _read_registers(SX1503RegId::ADVANCED, 1);  }
  return ret;
}


int8_t SX1503::attachInterrupt(uint8_t pin, PinCallback cb, IRQCondition condition) {
  pin &= 0x0F;
  int8_t ret = -1;
  if (16 > pin) {
    callbacks[pin]  = cb;
    ret = 0;
  }
  return ret;
}


/*
* Returns the number of interrupts removed.
*/
int8_t SX1503::detachInterrupt(uint8_t pin) {
  pin &= 0x0F;
  int8_t ret = (nullptr == callbacks[pin]) ? 0 : 1;
  callbacks[pin]  = nullptr;
  return ret;
}


/*
* Returns the number of interrupts removed.
*/
int8_t SX1503::detachInterrupt(PinCallback cb) {
  int8_t ret = 0;
  for (uint8_t i = 0; i < 16; i++) {
    if (cb == callbacks[i]) {
      callbacks[i] = nullptr;
      ret++;
    }
  }
  return ret;
}


/*
* TODO: Implement open-drain.
*/
int8_t SX1503::digitalWrite(uint8_t pin, bool value) {
  int8_t ret = -2;
  if (pin < 16) {
    ret = 0;
    SX1503RegId reg0 = (pin < 8) ? SX1503RegId::DATA_A : SX1503RegId::DATA_B;
    uint8_t val0 = _get_shadow_value(reg0);
    uint8_t val1 = val0;
    pin = pin & 0x07; // Restrict to [0, 7].
    uint8_t f = 1 << pin;
    val0 = value ? (val0 | f) : (val0 & ~f);
    if (val1 != val0) {
      ret = _write_register(reg0, val0);
    }
  }
  return ret;
}


/**
*
* @return
*   0 or 1 to reflect the logic level
*   0 if the pin is unsupported
*/
uint8_t SX1503::digitalRead(uint8_t pin) {
  uint8_t ret = 0;
  if (pin < 8) {
    ret = (_a_dat >> pin) & 0x01;
  }
  else if (pin < 16) {
    ret = (_b_dat >> (pin & 0x07)) & 0x01;
  }
  return ret;
}


/**
*
* @return The current verified state of the GPIO pins.
*/
uint16_t SX1503::getPinValues() {
  return (_a_dat | ((uint16_t) _b_dat << 8));
}


/**
*
* @return
*   -2 on unsupported mode
*   -1 on bad pin
*   0 on success
*/
int8_t SX1503::gpioMode(uint8_t pin, GPIOMode mode) {
  uint8_t ret = -1;
  if (pin < 16) {
    bool in = true;
    bool pu = false;
    bool pd = false;
    switch (mode) {
      // TODO: This part is capable of supporting open-drain output, but it
      //   would take slieght-of-hand that is not yet demanded of this driver.
      case GPIOMode::OUTPUT:
        in = false;
        break;
      case GPIOMode::INPUT_PULLUP:
        pu = true;
        break;
      case GPIOMode::INPUT_PULLDOWN:
        pd = true;
        // No break;
      case GPIOMode::INPUT:
        break;
      default:
        return -2;
    }
    ret = 0;

    SX1503RegId reg0 = (pin < 8) ? SX1503RegId::DIR_A : SX1503RegId::DIR_B;
    SX1503RegId reg1 = (pin < 8) ? SX1503RegId::PULLUP_A : SX1503RegId::PULLUP_B;
    SX1503RegId reg2 = (pin < 8) ? SX1503RegId::PULLDOWN_A : SX1503RegId::PULLDOWN_B;
    SX1503RegId reg3 = (pin < 8) ? SX1503RegId::IRQ_MASK_A : SX1503RegId::IRQ_MASK_B;
    SX1503RegId reg4 = (pin < 8) ? SX1503RegId::SENSE_H_A : SX1503RegId::SENSE_H_B;
    SX1503RegId reg5 = (pin < 8) ? SX1503RegId::SENSE_L_A : SX1503RegId::SENSE_L_B;

    uint8_t val0 = _get_shadow_value(reg0);
    uint8_t val1 = _get_shadow_value(reg1);
    uint8_t val2 = _get_shadow_value(reg2);
    uint8_t val3 = _get_shadow_value(reg3);
    uint8_t val4 = _get_shadow_value(reg4);
    uint8_t val5 = _get_shadow_value(reg5);
    pin = pin & 0x07; // Restrict to 8-bits.
    uint8_t f = 1 << pin;

    // Set direction, and pull resistors.
    val0 = (in) ? (val0 | f)  : (val0 & ~f);
    val1 = (pu) ? (val1 | f)  : (val1 & ~f);
    val2 = (pd) ? (val2 | f)  : (val2 & ~f);

    // Pin being set as an input means we need to unmask the interrupt.
    val3 = (in) ? (val3 & ~f) : (val3 | f);

    // Disable pin events for output pins, and enable them on both edges for inputs.
    uint8_t restricted_mask = 0x03 << ((pin & 0x03) << 1);
    if (pin > 3) {
      val4 = (in) ? (val4 | restricted_mask) : (val4 & ~restricted_mask);
    }
    else {
      val5 = (in) ? (val5 | restricted_mask) : (val5 & ~restricted_mask);
    }

    if ((0 == ret) & (_get_shadow_value(reg0) != val0)) {  ret = _write_register(reg0, val0);   }
    if ((0 == ret) & (_get_shadow_value(reg1) != val1)) {  ret = _write_register(reg1, val1);   }
    if ((0 == ret) & (_get_shadow_value(reg2) != val2)) {  ret = _write_register(reg2, val2);   }
    if ((0 == ret) & (_get_shadow_value(reg3) != val3)) {  ret = _write_register(reg3, val3);   }
    if ((0 == ret) & (_get_shadow_value(reg4) != val4)) {  ret = _write_register(reg4, val4);   }
    if ((0 == ret) & (_get_shadow_value(reg5) != val5)) {  ret = _write_register(reg5, val5);   }
  }
  return ret;
}



/*******************************************************************************
* Hidden machinery
*******************************************************************************/

int8_t SX1503::_invoke_pin_callback(uint8_t pin, bool value) {
  int8_t ret = -1;
  pin &= 0x0F;
  if (nullptr != callbacks[pin]) {
    callbacks[pin](pin, value?1:0);
    ret = 0;
  }
  return ret;
}


/*
* Setup the low-level pin details.
* Both pins are optional, so this can't fail.
*/
int8_t SX1503::_ll_pin_init() {
  if (255 != _IRQ_PIN) {
    pinMode(_IRQ_PIN, GPIOMode::INPUT_PULLUP);
    sx1503_isr_fired = !readPin(_IRQ_PIN);
    setPinFxn(_IRQ_PIN, IRQCondition::FALLING, sx1503_isr);
  }
  if (255 != _RESET_PIN) {
    pinMode(_RESET_PIN, GPIOMode::OUTPUT);
  }
  _sx_set_flag(SX1503_FLAG_PINS_CONFD);
  return 0;
}


int8_t SX1503::_write_register(SX1503RegId reg, uint8_t val) {
  int8_t ret = -2;
  if (nullptr != _bus) {
    ret++;
    _set_shadow_value(reg, val);
    ret = _write_registers(reg, 1);
  }
  return ret;
}


/*
* This is basically only used to sync the class state back into the hardware.
* Does not update the shadow.
*/
int8_t SX1503::_write_registers(SX1503RegId reg, uint8_t len) {
  int8_t ret = -1;
  if (nullptr != _bus) {
    // TODO: Address continuity check.
    uint8_t reg_idx = (uint8_t) reg;
    I2CBusOp* op = _bus->new_op(BusOpcode::TX, this);
    if (nullptr != op) {
      op->dev_addr = _dev_addr;
      op->sub_addr = SX1503_REG_ADDR[reg_idx];
      op->setBuffer(&registers[reg_idx], len);
      if (0 == queue_io_job(op)) {
        ret = 0;
      }
    }
  }
  return ret;
}


int8_t SX1503::_read_registers(SX1503RegId reg, uint8_t len) {
  int8_t ret = -2;
  if (nullptr != _bus) {
    // TODO: Address continuity check.
    uint8_t reg_idx = (uint8_t) reg;
    if (len > 0) {
      ret++;
      I2CBusOp* op = _bus->new_op(BusOpcode::RX, this);
      if (nullptr != op) {
        op->dev_addr = _dev_addr;
        op->sub_addr = SX1503_REG_ADDR[reg_idx];
        op->setBuffer(&registers[reg_idx], len);
        if (0 == queue_io_job(op)) {
          ret = 0;
        }
      }
    }
  }
  return ret;
}


/*******************************************************************************
* Serialization fxns
*******************************************************************************/

/*
* Stores everything about the class in the provided buffer in this format...
*   Offset | Data
*   -------|----------------------
*   0      | Serializer version
*   1      | IRQ pin
*   2      | Reset pin
*   3      | Flags MSB
*   4      | Flags LSB
*   5-35   | Full register set
*
* Returns the number of bytes written to the buffer.
*/
uint8_t SX1503::serialize(uint8_t* buf, unsigned int len) {
  uint8_t offset = 0;
  if (len >= SX1503_SERIALIZE_SIZE) {
    if (_sx_flag(SX1503_FLAG_INITIALIZED)) {
      uint16_t f = _flags & SX1503_FLAG_SERIAL_MASK;
      *(buf + offset++) = SX1503_SERIALIZE_VERSION;
      *(buf + offset++) = _IRQ_PIN;
      *(buf + offset++) = _RESET_PIN;
      *(buf + offset++) = (uint8_t) 0xFF & (f >> 8);
      *(buf + offset++) = (uint8_t) 0xFF & f;
      for (uint8_t i = 0; i < 31; i++) {
        *(buf + offset++) = registers[i];
      }
    }
  }
  return offset;
}


int8_t SX1503::unserialize(const uint8_t* buf, const unsigned int len) {
  uint8_t offset = 0;
  uint8_t expected_sz = 255;
  if (len >= SX1503_SERIALIZE_SIZE) {
    uint16_t f = 0;
    switch (*(buf + offset++)) {
      case SX1503_SERIALIZE_VERSION:
        expected_sz = SX1503_SERIALIZE_SIZE;
        f = (*(buf + 3) << 8) | *(buf + 4);
        _flags = (_flags & ~SX1503_FLAG_SERIAL_MASK) | (f & SX1503_FLAG_SERIAL_MASK);
        offset += 4;  // Skip to the register offset.
        for (uint8_t i = 0; i < 31; i++) {
          registers[i] = *(buf + offset++);
        }
        break;
      default:  // Unhandled serializer version.
        return -2;
    }
    if (_sx_flag(SX1503_FLAG_INITIALIZED)) {
      // If the device has already been initialized, we impart the new conf.
      int8_t ret = _write_registers(SX1503RegId::DATA_B, 0x12);
      if (0 == ret) {  ret = _write_registers(SX1503RegId::PLD_MODE_B, 0x0C);  }
      if (0 == ret) {  ret = _write_registers(SX1503RegId::ADVANCED, 1);       }
    }
    else {
      _sx_set_flag(SX1503_FLAG_FROM_BLOB);
    }
  }
  return (expected_sz == offset) ? 0 : -1;
}


/*******************************************************************************
* ___     _       _                      These members are mandatory overrides
*  |   / / \ o   | \  _     o  _  _      for implementing I/O callbacks. They
* _|_ /  \_/ o   |_/ (/_ \/ | (_ (/_     are also implemented by Adapters.
*******************************************************************************/

/* Transfers always permitted. */
int8_t SX1503::io_op_callahead(BusOp* _op) {   return 0;   }


/*
* Register I/O calls back to this function for BOTH devices (MAG/IMU). So we
*   split the function up into two halves in private scope in the superclass.
* Bus operations that call back with errors are ignored.
*/
int8_t SX1503::io_op_callback(BusOp* _op) {
  I2CBusOp* op  = (I2CBusOp*) _op;
  int8_t    ret = BUSOP_CALLBACK_NOMINAL;

  if (!op->hasFault()) {
    uint8_t*    buf   = op->buffer();
    uint        len   = op->bufferLen();
    if (!_sx_flag(SX1503_FLAG_DEVICE_PRESENT)) {
      _sx_set_flag(SX1503_FLAG_DEVICE_PRESENT);
    }
    switch (op->get_opcode()) {
      case BusOpcode::TX:
        for (uint i = 0; i < len; i++) {
          SX1503RegId reg   = _reg_id_from_addr(i + op->sub_addr);
          uint8_t     value = *(buf + i);
          switch (reg) {
            case SX1503RegId::DATA_B:
            case SX1503RegId::DATA_A:
              {
                uint8_t dir_val = _get_shadow_value((reg == SX1503RegId::DATA_B) ? SX1503RegId::DIR_B : SX1503RegId::DIR_A);
                uint8_t* dat_val = (reg == SX1503RegId::DATA_B) ? &_b_dat : &_a_dat;
                //*dat_val = (value & dir_val) | (value | ~dir_val);
                *dat_val = value;
              }
              break;

            case SX1503RegId::DIR_B:
            case SX1503RegId::DIR_A:
            case SX1503RegId::PULLUP_B:
            case SX1503RegId::PULLUP_A:
            case SX1503RegId::PULLDOWN_B:
            case SX1503RegId::PULLDOWN_A:
            case SX1503RegId::IRQ_MASK_B:
            case SX1503RegId::IRQ_MASK_A:
            case SX1503RegId::SENSE_H_B:
            case SX1503RegId::SENSE_H_A:
            case SX1503RegId::SENSE_L_B:
            case SX1503RegId::SENSE_L_A:
            case SX1503RegId::IRQ_SRC_B:
            case SX1503RegId::IRQ_SRC_A:
            case SX1503RegId::EVENT_STAT_B:
            case SX1503RegId::EVENT_STAT_A:
            case SX1503RegId::PLD_MODE_B:
            case SX1503RegId::PLD_MODE_A:
            case SX1503RegId::PLD_TABLE_0B:
            case SX1503RegId::PLD_TABLE_0A:
            case SX1503RegId::PLD_TABLE_1B:
            case SX1503RegId::PLD_TABLE_1A:
            case SX1503RegId::PLD_TABLE_2B:
            case SX1503RegId::PLD_TABLE_2A:
            case SX1503RegId::PLD_TABLE_3B:
            case SX1503RegId::PLD_TABLE_3A:
            case SX1503RegId::PLD_TABLE_4B:
            case SX1503RegId::PLD_TABLE_4A:
              break;
            case SX1503RegId::ADVANCED:
              if (0x04 == (value & 0x04)) {
                _sx_set_flag(SX1503_FLAG_INITIALIZED);
              }
              break;
            default:  // Anything else is invalid.
              break;
          }
        }
        break;

      case BusOpcode::RX:
        for (uint i = 0; i < len; i++) {
          SX1503RegId reg   = _reg_id_from_addr(i + op->sub_addr);
          uint8_t     value = *(buf + i);
          switch (reg) {
            case SX1503RegId::DATA_B:
            case SX1503RegId::DATA_A:
              {
                uint8_t dir_val = _get_shadow_value((reg == SX1503RegId::DATA_B) ? SX1503RegId::DIR_B : SX1503RegId::DIR_A);
                uint8_t* dat_val = (reg == SX1503RegId::DATA_B) ? &_b_dat : &_a_dat;
                uint8_t d = (*dat_val ^ value) & dir_val;  // Filter for changes in input values.
                uint8_t cb_base = (reg == SX1503RegId::DATA_B) ? 8 : 0;
                if (d) {
                  for (uint8_t i = 0; i < 8; i++) {
                    if ((d >> i) & 1) {
                      // We don't worry about initialized state here.
                      _invoke_pin_callback((i+cb_base), ((value >> i) & 1));
                    }
                  }
                  //*dat_val = (value & ~dir_val) | (value | dir_val);
                  *dat_val = value;
                }
              }
              sx1503_isr_fired = (255 != _IRQ_PIN) ? !readPin(_IRQ_PIN) : true;
              break;

            case SX1503RegId::DIR_B:
            case SX1503RegId::DIR_A:
            case SX1503RegId::PULLUP_B:
            case SX1503RegId::PULLUP_A:
            case SX1503RegId::PULLDOWN_B:
            case SX1503RegId::PULLDOWN_A:
            case SX1503RegId::IRQ_MASK_B:
            case SX1503RegId::IRQ_MASK_A:
            case SX1503RegId::SENSE_H_B:
            case SX1503RegId::SENSE_H_A:
            case SX1503RegId::SENSE_L_B:
            case SX1503RegId::SENSE_L_A:
            case SX1503RegId::IRQ_SRC_B:
            case SX1503RegId::IRQ_SRC_A:
            case SX1503RegId::EVENT_STAT_B:
            case SX1503RegId::EVENT_STAT_A:
            case SX1503RegId::PLD_MODE_B:
            case SX1503RegId::PLD_MODE_A:
            case SX1503RegId::PLD_TABLE_0B:
            case SX1503RegId::PLD_TABLE_0A:
            case SX1503RegId::PLD_TABLE_1B:
            case SX1503RegId::PLD_TABLE_1A:
            case SX1503RegId::PLD_TABLE_2B:
            case SX1503RegId::PLD_TABLE_2A:
            case SX1503RegId::PLD_TABLE_3B:
            case SX1503RegId::PLD_TABLE_3A:
            case SX1503RegId::PLD_TABLE_4B:
            case SX1503RegId::PLD_TABLE_4A:
              break;
            case SX1503RegId::ADVANCED:
              if (0x04 != (value & 0x04)) {
                _write_register(SX1503RegId::ADVANCED, value & 0x04);
              }
              break;
            default:  // Anything else is invalid.
              break;
          }
        }
        break;

      default:
        break;
    }
  }
  return ret;
}



/*******************************************************************************
* Debugging fxns
*******************************************************************************/
/*
*
*/
void SX1503::printDebug(StringBuilder* output) {
  output->concat("---< SX1503 >---------------------------------------------------\n");
  output->concatf("\tRESET Pin:   %u\n", _RESET_PIN);
  output->concatf("\tIRQ Pin:     %u\n", _IRQ_PIN);
  output->concatf("\tISR fired:   %c\n", sx1503_isr_fired ? 'y' : 'n');
  output->concatf("\tInitialized: %c\n", initialized() ? 'y' : 'n');
  output->concatf("\tPreserve:    %c\n", preserveOnDestroy() ? 'y' : 'n');
  output->concatf("\t_x_dat:      0x%04x\n", _a_dat | ((uint16_t) _b_dat << 8));
}


/*
*
*/
void SX1503::printRegs(StringBuilder* output) {
  for (uint8_t i = 0; i < sizeof(registers); i++) {
    output->concatf("\t0x%02x:\t0x%02x\n", SX1503_REG_ADDR[i], registers[i]);
  }
}
