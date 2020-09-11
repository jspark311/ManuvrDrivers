/*
File:   SX1503.h
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

#ifndef __SX1503_DRIVER_H__
#define __SX1503_DRIVER_H__

#include <CppPotpourri.h>
#include <AbstractPlatform.h>
#include <I2CAdapter.h>

class StringBuilder;

#define SX1503_I2C_ADDR           0x20  // Not configurable for SX1503.
#define SX1503_SERIALIZE_VERSION  0x01  // Version code for serialized states.
#define SX1503_SERIALIZE_SIZE       36


/* Class flags. */
#define SX1503_FLAG_PRESERVE_STATE   0x0001
#define SX1503_FLAG_READ_IN_FLIGHT   0x0002  // Waiting on the return of a data read.
#define SX1503_FLAG_WRITE_IN_FLIGHT  0x0004  // Waiting on the return of a data write.
#define SX1503_FLAG_DEVICE_PRESENT   0x1000
#define SX1503_FLAG_PINS_CONFD       0x2000
#define SX1503_FLAG_INITIALIZED      0x4000
#define SX1503_FLAG_FROM_BLOB        0x8000

#define SX1503_FLAG_SERIAL_MASK      0x000F  // Only these bits are serialized.


/* These are the i2c register indicies. NOT their addresses. */
enum class SX1503RegId : uint8_t {
  DATA_B       = 0x00,
  DATA_A       = 0x01,
  DIR_B        = 0x02,
  DIR_A        = 0x03,
  PULLUP_B     = 0x04,
  PULLUP_A     = 0x05,
  PULLDOWN_B   = 0x06,
  PULLDOWN_A   = 0x07,
  IRQ_MASK_B   = 0x08,
  IRQ_MASK_A   = 0x09,
  SENSE_H_B    = 0x0A,
  SENSE_H_A    = 0x0B,
  SENSE_L_B    = 0x0C,
  SENSE_L_A    = 0x0D,
  IRQ_SRC_B    = 0x0E,
  IRQ_SRC_A    = 0x0F,
  EVENT_STAT_B = 0x10,
  EVENT_STAT_A = 0x11,
  PLD_MODE_B   = 0x12,
  PLD_MODE_A   = 0x13,
  PLD_TABLE_0B = 0x14,
  PLD_TABLE_0A = 0x15,
  PLD_TABLE_1B = 0x16,
  PLD_TABLE_1A = 0x17,
  PLD_TABLE_2B = 0x18,
  PLD_TABLE_2A = 0x19,
  PLD_TABLE_3B = 0x1A,
  PLD_TABLE_3A = 0x1B,
  PLD_TABLE_4B = 0x1C,
  PLD_TABLE_4A = 0x1D,
  ADVANCED     = 0x1E,
  INVALID      = 0x1F
};


/*
* Driver class.
*/
class SX1503 : public I2CDevice {
  public:
    SX1503(const uint8_t irq_pin, const uint8_t reset_pin);
    SX1503(const uint8_t* buf, const unsigned int len);  // Takes serialized state as args.
    ~SX1503();

    /* Overrides from the BusOpCallback interface */
    int8_t io_op_callahead(BusOp*);
    int8_t io_op_callback(BusOp*);

    inline int8_t init() {  return init(_bus);  };
    int8_t init(I2CAdapter*);
    int8_t reset();
    int8_t poll();
    bool isrFired();
    int8_t refresh();

    // Basic usage as pins...
    int8_t  gpioMode(uint8_t pin, GPIOMode mode);
    int8_t  digitalWrite(uint8_t pin, bool value);
    uint8_t digitalRead(uint8_t pin);
    uint16_t getPinValues();

    // Interrupt and callback management...
    int8_t  attachInterrupt(uint8_t pin, PinCallback, IRQCondition condition);
    int8_t  detachInterrupt(uint8_t pin);
    int8_t  detachInterrupt(PinCallback);

    // Advanced usage...
    int8_t  setPLD();  // TODO: Define API for this feature.
    int8_t  useBoost(bool enable);

    // No NVM on this part, so these fxns help do init in a single step.
    uint8_t serialize(uint8_t* buf, unsigned int len);
    int8_t  unserialize(const uint8_t* buf, const unsigned int len);

    inline bool initialized() {  return _sx_flag(SX1503_FLAG_INITIALIZED);  };
    inline bool preserveOnDestroy() {
      return _sx_flag(SX1503_FLAG_PRESERVE_STATE);
    };
    inline void preserveOnDestroy(bool x) {
      _sx_set_flag(SX1503_FLAG_PRESERVE_STATE, x);
    };

    // Debugging fxns. Mask out if debugging isn't desired.
    void printDebug(StringBuilder*);
    void printRegs(StringBuilder*);


  private:
    const uint8_t  _IRQ_PIN;
    const uint8_t  _RESET_PIN;
    uint16_t       _flags = 0;
    uint8_t        _a_dat = 0;
    uint8_t        _b_dat = 0;
    PinCallback    callbacks[16] = {nullptr, };
    uint8_t        registers[31] = {0, };

    int8_t _invoke_pin_callback(uint8_t pin, bool value);
    int8_t _write_register(SX1503RegId, uint8_t val);
    int8_t _write_registers(SX1503RegId, uint8_t len);
    int8_t _read_registers(SX1503RegId, uint8_t len);
    int8_t _ll_pin_init();
    inline uint8_t _get_shadow_value(SX1503RegId r) {
      return registers[((uint8_t) r) & 0x1F];
    };
    inline void _set_shadow_value(SX1503RegId r, uint8_t val) {
      registers[((uint8_t) r) & 0x1F] = val;
    };

    inline bool _from_blob() {   return _sx_flag(SX1503_FLAG_FROM_BLOB);  };

    /* Flag manipulation inlines */
    inline uint16_t _sx_flags() {                return _flags;           };
    inline bool _sx_flag(uint16_t _flag) {       return (_flags & _flag); };
    inline void _sx_clear_flag(uint16_t _flag) { _flags &= ~_flag;        };
    inline void _sx_set_flag(uint16_t _flag) {   _flags |= _flag;         };
    inline void _sx_set_flag(uint16_t _flag, bool nu) {
      if (nu) _flags |= _flag;
      else    _flags &= ~_flag;
    };
};

#endif   // __SX1503_DRIVER_H__
